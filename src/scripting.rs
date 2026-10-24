//! Project-owned Rhai boundary profiles. No solver mutation or external I/O API.
use crate::{unstructured_incompressible::IncompressibleBoundaryCondition as B, Vec3};
use rhai::{CallFnOptions, Dynamic, Engine, Scope, AST};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
pub const CONTRACT_VERSION: u32 = 1;
fn contract_version() -> u32 {
    CONTRACT_VERSION
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Quantity {
    Velocity,
    Pressure,
    Temperature,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Script {
    pub source: String,
    #[serde(default)]
    pub time_dependent: bool,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Binding {
    pub script: String,
    pub function: String,
    pub quantity: Quantity,
    #[serde(default)]
    pub parameters: BTreeMap<String, f64>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    #[serde(default = "contract_version")]
    pub contract_version: u32,
    pub scripts: BTreeMap<String, Script>,
    pub bindings: BTreeMap<String, Vec<Binding>>,
    /// Set by the transient driver, never persisted as project configuration.
    #[serde(skip)]
    pub evaluation_time: Option<f64>,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            contract_version: CONTRACT_VERSION,
            scripts: Default::default(),
            bindings: Default::default(),
            evaluation_time: None,
        }
    }
}
fn engine() -> Engine {
    use rhai::packages::{
        ArithmeticPackage, BasicArrayPackage, BasicIteratorPackage, BasicMapPackage,
        BasicMathPackage, LogicPackage, Package,
    };
    // Whitelist numeric containers/operators. StandardPackage also exposes
    // wall-clock and blocking sleep functions, which boundary callbacks must not use.
    let mut e = Engine::new_raw();
    ArithmeticPackage::new().register_into_engine(&mut e);
    LogicPackage::new().register_into_engine(&mut e);
    BasicMathPackage::new().register_into_engine(&mut e);
    BasicArrayPackage::new().register_into_engine(&mut e);
    BasicMapPackage::new().register_into_engine(&mut e);
    BasicIteratorPackage::new().register_into_engine(&mut e);
    e.set_max_operations(20_000);
    e.set_max_call_levels(16);
    e.set_max_variables(64);
    e.set_max_functions(64);
    e.set_max_modules(0);
    e.set_max_expr_depths(48, 24);
    e.set_max_array_size(64);
    e.set_max_map_size(64);
    e.set_max_string_size(4096);
    e.disable_symbol("eval");
    e.disable_symbol("import");
    e.on_print(|_| {});
    e.on_debug(|_, _, _| {});
    e.register_fn("vec3", |x: f64, y: f64, z: f64| -> rhai::Array {
        vec![x.into(), y.into(), z.into()]
    });
    e
}
pub struct Compiled {
    engine: Engine,
    ast: AST,
    timed: bool,
}
impl Script {
    pub fn compile(&self) -> Result<std::sync::Arc<Compiled>, String> {
        if self.source.len() > 65_536 {
            return Err("Script exceeds 64 KiB".into());
        }
        type Cache = BTreeMap<(String, bool), std::sync::Arc<Compiled>>;
        static CACHE: std::sync::OnceLock<std::sync::Mutex<Cache>> = std::sync::OnceLock::new();
        let mut cache = CACHE
            .get_or_init(Default::default)
            .lock()
            .map_err(|_| "Script cache unavailable")?;
        let key = (self.source.clone(), self.time_dependent);
        if let Some(c) = cache.get(&key) {
            return Ok(c.clone());
        }
        let e = engine();
        let ast = e.compile(&self.source).map_err(|e| e.to_string())?;
        // Only function bodies run; global statements and imports are never executed.
        let c = std::sync::Arc::new(Compiled {
            engine: e,
            ast: ast.clone_functions_only(),
            timed: self.time_dependent,
        });
        if cache.len() >= 64 {
            cache.clear();
        }
        cache.insert(key, c.clone());
        Ok(c)
    }
}
impl Compiled {
    pub fn value(&self, b: &Binding, pos: Vec3, time: Option<f64>) -> Result<Vec3, String> {
        crate::compute::checkpoint().map_err(|e| format!("Script evaluation interrupted: {e}"))?;
        if time.is_some_and(|t| !t.is_finite() || t < 0.0) {
            return Err("Script time must be finite and nonnegative".into());
        }
        if ![pos.x, pos.y, pos.z].iter().all(|v| v.is_finite()) {
            return Err("Script coordinates must be finite".into());
        }
        if self.timed && time.is_none() {
            return Err("Time-dependent scripts require transient integration".into());
        }
        if b.parameters.len() > 64 || b.parameters.keys().any(|k| k.len() > 128) {
            return Err("At most 64 parameters with names up to 128 bytes are allowed".into());
        }
        if !b.parameters.values().all(|v| v.is_finite()) {
            return Err("Script parameters must be finite".into());
        }
        let mut scope = Scope::new();
        let params: rhai::Map = b
            .parameters
            .iter()
            .map(|(k, v)| (k.as_str().into(), Dynamic::from_float(*v)))
            .collect();
        scope.push_constant("params", params);
        scope.push_constant("PI", std::f64::consts::PI);
        let options = CallFnOptions::new().eval_ast(false);
        let result = if self.timed {
            self.engine.call_fn_with_options::<Dynamic>(
                options,
                &mut scope,
                &self.ast,
                &b.function,
                (pos.x, pos.y, pos.z, time.unwrap()),
            )
        } else {
            self.engine.call_fn_with_options::<Dynamic>(
                options,
                &mut scope,
                &self.ast,
                &b.function,
                (pos.x, pos.y, pos.z),
            )
        }
        .map_err(|e| {
            format!(
                "Script {} / {} at ({}, {}, {}), t={time:?}: {e}",
                b.script, b.function, pos.x, pos.y, pos.z
            )
        })?;
        let scalar = |v: Dynamic| -> Result<f64, String> {
            let n = if v.is::<rhai::INT>() {
                v.cast::<rhai::INT>() as f64
            } else {
                v.as_float()
                    .map_err(|_| "Expected numeric output".to_string())?
            };
            if n.is_finite() {
                Ok(n)
            } else {
                Err("Script returned NaN or infinity".into())
            }
        };
        let v = if b.quantity == Quantity::Velocity {
            let a = result
                .try_cast::<rhai::Array>()
                .ok_or("Velocity must return vec3(Ux, Uy, Uz)")?;
            if a.len() != 3 {
                return Err("Velocity requires exactly three components".into());
            }
            Vec3::new(
                scalar(a[0].clone())?,
                scalar(a[1].clone())?,
                scalar(a[2].clone())?,
            )
        } else {
            Vec3::new(scalar(result)?, 0.0, 0.0)
        };
        if b.quantity == Quantity::Temperature && v.x <= 0.0 {
            return Err("Temperature must be positive Kelvin".into());
        }
        Ok(v)
    }
}
impl Settings {
    pub fn validate(&self, transient: bool) -> Result<(), String> {
        if self.contract_version != CONTRACT_VERSION {
            return Err(format!(
                "Unsupported script contract {}",
                self.contract_version
            ));
        }
        if self.scripts.len() > 64 || self.bindings.len() > 1024 {
            return Err("Script library or binding limit exceeded".into());
        }
        for (id, s) in &self.scripts {
            if id.trim().is_empty() {
                return Err("Script name cannot be empty".into());
            }
            s.compile().map_err(|e| format!("Script {id}: {e}"))?;
        }
        for (patch, bindings) in &self.bindings {
            if bindings.len() > 3 {
                return Err(format!("Too many script bindings on {patch}"));
            }
            for (i, b) in bindings.iter().enumerate() {
                if bindings[..i].iter().any(|p| p.quantity == b.quantity) {
                    return Err(format!("Duplicate script quantity on {patch}"));
                }
                let s = self
                    .scripts
                    .get(&b.script)
                    .ok_or_else(|| format!("Missing script {} on {patch}", b.script))?;
                if s.time_dependent && !transient {
                    return Err(format!(
                        "Script {} on {patch} requires transient integration",
                        b.script
                    ));
                }
                if b.function.trim().is_empty() || !b.parameters.values().all(|v| v.is_finite()) {
                    return Err(format!("Invalid script binding on {patch}"));
                }
                let c = s.compile()?;
                let arity = if s.time_dependent { 4 } else { 3 };
                if !c
                    .ast
                    .iter_functions()
                    .any(|f| f.name == b.function && f.params.len() == arity)
                {
                    return Err(format!(
                        "{} requires fn {} with {arity} arguments",
                        b.script, b.function
                    ));
                }
            }
        }
        Ok(())
    }
    pub fn check_boundary(
        &self,
        patch: &str,
        bc: B,
        thermal: Option<crate::thermal::Boundary>,
    ) -> Result<(), String> {
        for b in self.bindings.get(patch).into_iter().flatten() {
            let ok = match b.quantity {
                Quantity::Velocity => matches!(bc, B::VelocityInlet { .. }),
                Quantity::Pressure => matches!(
                    bc,
                    B::PressureOutlet { .. } | B::PressureOutletBackflow { .. }
                ),
                Quantity::Temperature => matches!(
                    thermal,
                    Some(
                        crate::thermal::Boundary::FixedTemperature { .. }
                            | crate::thermal::Boundary::InletOutlet { .. }
                    )
                ),
            };
            if !ok {
                return Err(format!("Script {:?} is incompatible with boundary {patch}; assign a supported physical condition first",b.quantity));
            }
        }
        Ok(())
    }
    pub fn compiled(&self, b: &Binding) -> Result<std::sync::Arc<Compiled>, String> {
        self.scripts
            .get(&b.script)
            .ok_or_else(|| format!("Missing script {}", b.script))?
            .compile()
    }
}
