//! Conservative ideal-gas finite-volume flow.
use crate::{UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
pub mod benchmarks;
mod flux;
mod output;
mod time;
mod viscous;
pub use output::write_vtu;
pub use time::{solve, solve_controlled, Report, Solution};
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct IdealGas {
    pub gamma: f64,
    pub gas_constant: f64,
}
impl Default for IdealGas {
    fn default() -> Self {
        Self {
            gamma: 1.4,
            gas_constant: 287.05,
        }
    }
}
impl IdealGas {
    pub fn validate(self) -> Result<(), String> {
        if !self.gamma.is_finite()
            || self.gamma <= 1.0
            || !self.gas_constant.is_finite()
            || self.gas_constant <= 0.0
        {
            Err("Ideal gas requires finite gamma > 1 and R > 0".into())
        } else {
            Ok(())
        }
    }
    pub fn cv(self) -> f64 {
        self.gas_constant / (self.gamma - 1.0)
    }
    pub fn cp(self) -> f64 {
        self.gamma * self.cv()
    }
    pub fn temperature(self, p: Primitive) -> f64 {
        p.pressure / (p.density * self.gas_constant)
    }
    pub fn sound_speed(self, p: Primitive) -> f64 {
        (self.gamma * p.pressure / p.density).sqrt()
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Primitive {
    pub density: f64,
    pub velocity: Vec3,
    pub pressure: f64,
}
impl Primitive {
    fn validate(self) -> Result<(), String> {
        if ![
            self.density,
            self.pressure,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z,
        ]
        .iter()
        .all(|v| v.is_finite())
            || self.density <= 0.0
            || self.pressure <= 0.0
        {
            Err(
                "Compressible state requires finite rho > 0, absolute p > 0 and finite velocity"
                    .into(),
            )
        } else {
            Ok(())
        }
    }
    fn conservative(self, gas: IdealGas) -> Conservative {
        Conservative([
            self.density,
            self.density * self.velocity.x,
            self.density * self.velocity.y,
            self.density * self.velocity.z,
            self.pressure / (gas.gamma - 1.0)
                + 0.5 * self.density * self.velocity.dot(self.velocity),
        ])
    }
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Conservative(pub [f64; 5]);
impl Conservative {
    fn primitive(self, gas: IdealGas) -> Result<Primitive, String> {
        let q = self.0;
        let density = q[0];
        let velocity = Vec3::new(q[1] / density, q[2] / density, q[3] / density);
        let pressure = (gas.gamma - 1.0) * (q[4] - 0.5 * density * velocity.dot(velocity));
        let p = Primitive {
            density,
            velocity,
            pressure,
        };
        p.validate()?;
        if q.iter().any(|v| !v.is_finite()) {
            return Err("Non-finite conservative state".into());
        }
        Ok(p)
    }
    fn add(self, b: Self) -> Self {
        Self(std::array::from_fn(|i| self.0[i] + b.0[i]))
    }
    fn sub(self, b: Self) -> Self {
        Self(std::array::from_fn(|i| self.0[i] - b.0[i]))
    }
    fn scale(self, a: f64) -> Self {
        Self(self.0.map(|v| v * a))
    }
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "kebab-case")]
pub enum FluxScheme {
    Rusanov,
    Hllc,
    #[default]
    CentralUpwind,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum Boundary {
    SlipWall,
    NoSlipWall {
        #[serde(default)]
        temperature: Option<f64>,
    },
    /// Prescribed exterior Riemann state; not a non-reflecting characteristic BC.
    FarField {
        state: Primitive,
    },
    SupersonicInlet {
        state: Primitive,
    },
    TotalInlet {
        pressure: f64,
        temperature: f64,
        direction: Vec3,
    },
    Transmissive,
    /// Basic subsonic pressure ghost state; not a general characteristic outlet.
    PressureOutlet {
        pressure: f64,
    },
}
impl Boundary {
    fn ghost(self, owner: Primitive, normal: Vec3, gas: IdealGas) -> Result<Primitive, String> {
        Ok(match self {
            Self::SlipWall | Self::NoSlipWall { .. } => Primitive {
                velocity: owner.velocity - normal * (2.0 * owner.velocity.dot(normal)),
                ..owner
            },
            Self::FarField { state } => state,
            Self::TotalInlet {
                pressure,
                temperature,
                direction,
            } => {
                let direction = direction / direction.norm();
                let cosine = direction.dot(normal);
                if cosine >= 0.0 {
                    return Err("Total-condition inlet direction must point into the domain".into());
                }
                let invariant =
                    owner.velocity.dot(normal) + 2.0 * gas.sound_speed(owner) / (gas.gamma - 1.0);
                let mut lo = 0.0;
                let mut hi =
                    (2.0 * gas.gamma * gas.gas_constant * temperature / (gas.gamma + 1.0)).sqrt();
                for _ in 0..64 {
                    let q = 0.5 * (lo + hi);
                    let a = ((gas.gamma - 1.0) * (gas.cp() * temperature - 0.5 * q * q)).sqrt();
                    if q * cosine + 2.0 * a / (gas.gamma - 1.0) > invariant {
                        lo = q;
                    } else {
                        hi = q;
                    }
                }
                let speed = 0.5 * (lo + hi);
                let t = temperature - speed * speed / (2.0 * gas.cp());
                let p = pressure * (t / temperature).powf(gas.gamma / (gas.gamma - 1.0));
                Primitive {
                    density: p / (gas.gas_constant * t),
                    pressure: p,
                    velocity: direction * speed,
                }
            }
            Self::SupersonicInlet { state } => {
                if state.velocity.dot(normal) >= -gas.sound_speed(state) {
                    return Err("Supersonic inlet requires inward normal Mach > 1 on every inlet face; use far-field for a prescribed subsonic exterior state".into());
                }
                state
            }
            Self::Transmissive => owner,
            Self::PressureOutlet { pressure } => {
                let un = owner.velocity.dot(normal);
                if un < 0.0 {
                    return Err("Compressible pressure-outlet backflow needs an incoming thermodynamic state; use a far-field exterior state".into());
                }
                if un >= gas.sound_speed(owner) {
                    owner
                } else {
                    Primitive {
                        pressure,
                        density: owner.density * (pressure / owner.pressure).powf(1.0 / gas.gamma),
                        ..owner
                    }
                }
            }
        })
    }
}
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum Viscosity {
    #[default]
    Inviscid,
    Constant {
        mu: f64,
        prandtl: f64,
    },
    Sutherland {
        mu_ref: f64,
        temperature_ref: f64,
        sutherland: f64,
        prandtl: f64,
    },
}
impl Viscosity {
    fn coefficients(self, t: f64, gas: IdealGas) -> Result<(f64, f64), String> {
        if !t.is_finite() || t <= 0.0 {
            return Err("Nonpositive gas temperature".into());
        }
        let (mu, pr) = match self {
            Self::Inviscid => return Ok((0.0, 0.0)),
            Self::Constant { mu, prandtl } => (mu, prandtl),
            Self::Sutherland {
                mu_ref,
                temperature_ref,
                sutherland,
                prandtl,
            } => {
                if ![mu_ref, temperature_ref, sutherland]
                    .iter()
                    .all(|x| x.is_finite())
                    || mu_ref <= 0.0
                    || temperature_ref <= 0.0
                    || sutherland < 0.0
                {
                    return Err("Invalid Sutherland constants".into());
                }
                (
                    mu_ref * (t / temperature_ref).powf(1.5) * (temperature_ref + sutherland)
                        / (t + sutherland),
                    prandtl,
                )
            }
        };
        if !mu.is_finite() || mu <= 0.0 || !pr.is_finite() || pr <= 0.0 {
            return Err("Viscosity and Prandtl must be positive and finite".into());
        }
        let conductivity = mu * gas.cp() / pr;
        if !conductivity.is_finite() {
            return Err("Non-finite thermal conductivity".into());
        }
        Ok((mu, conductivity))
    }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct InitialRegion {
    pub minimum: Vec3,
    pub maximum: Vec3,
    pub state: Primitive,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Settings {
    pub gas: IdealGas,
    pub viscosity: Viscosity,
    pub flux: FluxScheme,
    pub courant: f64,
    pub max_time_step: f64,
    pub min_time_step: f64,
    pub end_time: f64,
    pub max_steps: usize,
    pub initial: Primitive,
    pub regions: Vec<InitialRegion>,
    pub boundaries: BTreeMap<String, Boundary>,
    pub cyclic_pairs: Vec<crate::periodic::TranslationalCyclicPair>,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            gas: IdealGas::default(),
            viscosity: Viscosity::Inviscid,
            flux: FluxScheme::default(),
            courant: 0.3,
            max_time_step: 1e-3,
            min_time_step: 1e-14,
            end_time: 0.1,
            max_steps: 100000,
            initial: Primitive {
                density: 1.225,
                pressure: 101325.0,
                velocity: Vec3::ZERO,
            },
            regions: vec![],
            boundaries: BTreeMap::new(),
            cyclic_pairs: Vec::new(),
        }
    }
}
impl Settings {
    pub fn validate(
        &self,
        mesh: &UnstructuredMesh,
    ) -> Result<(Vec<Option<Boundary>>, crate::periodic::CyclicMap), String> {
        self.gas.validate()?;
        self.initial.validate()?;
        self.viscosity
            .coefficients(self.gas.temperature(self.initial), self.gas)?;
        if ![
            self.courant,
            self.max_time_step,
            self.min_time_step,
            self.end_time,
        ]
        .iter()
        .all(|v| v.is_finite() && *v > 0.0)
            || self.courant > 0.5
            || self.min_time_step > self.max_time_step
            || self.max_steps == 0
        {
            return Err(
                "Require 0 < CFL ≤ 0.5, positive time limits, min dt ≤ max dt and max_steps > 0"
                    .into(),
            );
        }
        let check = crate::workbench::diagnostics::mesh_preflight(mesh);
        if check.blocked() {
            return Err(check.summary());
        }
        for r in &self.regions {
            r.state.validate()?;
            for (a, b) in [
                (r.minimum.x, r.maximum.x),
                (r.minimum.y, r.maximum.y),
                (r.minimum.z, r.maximum.z),
            ] {
                if !a.is_finite() || !b.is_finite() || a > b {
                    return Err("Invalid initial-region bounds".into());
                }
            }
        }
        let cyclic = crate::periodic::resolve(mesh, &self.cyclic_pairs)?;
        let mut bc = vec![None; mesh.face_count()];
        for patch in mesh.boundary_patches() {
            if patch
                .face_indices
                .first()
                .is_some_and(|&i| cyclic.link(i).is_some())
            {
                if self.boundaries.contains_key(&patch.name) {
                    return Err(format!(
                        "Cyclic patch {} also has a physical boundary; remove that assignment",
                        patch.name
                    ));
                }
                continue;
            }
            let b = *self
                .boundaries
                .get(&patch.name)
                .ok_or_else(|| format!("Missing compressible boundary {:?}", patch.name))?;
            match b {
                Boundary::TotalInlet {
                    pressure,
                    temperature,
                    direction,
                } => {
                    if !pressure.is_finite()
                        || pressure <= 0.0
                        || !temperature.is_finite()
                        || temperature <= 0.0
                        || !direction.norm().is_finite()
                        || direction.norm() <= 0.0
                    {
                        return Err("Total inlet requires positive finite stagnation pressure, temperature and nonzero direction".into());
                    }
                    for &i in &patch.face_indices {
                        if direction.dot(mesh.faces()[i].normal) >= 0.0 {
                            return Err(
                                "Total inlet direction must be inward on every inlet face".into()
                            );
                        }
                    }
                }
                Boundary::FarField { state } | Boundary::SupersonicInlet { state } => {
                    state.validate()?
                }
                Boundary::PressureOutlet { pressure }
                    if !pressure.is_finite() || pressure <= 0.0 =>
                {
                    return Err("Outlet absolute pressure must be positive".into());
                }
                Boundary::NoSlipWall { temperature } => {
                    if matches!(self.viscosity, Viscosity::Inviscid) {
                        return Err(
                            "No-slip requires molecular viscosity; use slip wall for Euler".into(),
                        );
                    }
                    if temperature.is_some_and(|t| !t.is_finite() || t <= 0.0) {
                        return Err("Wall absolute temperature must be positive".into());
                    }
                }
                _ => {}
            }
            for &f in &patch.face_indices {
                if bc[f].replace(b).is_some() {
                    return Err("Overlapping compressible boundary patches".into());
                }
            }
        }
        if self
            .boundaries
            .keys()
            .any(|name| !mesh.boundary_patches().iter().any(|p| &p.name == name))
        {
            return Err("Compressible boundary references a missing mesh patch".into());
        }
        if mesh
            .faces()
            .iter()
            .enumerate()
            .any(|(i, f)| f.neighbour.is_none() && bc[i].is_none() && cyclic.link(i).is_none())
        {
            return Err("Unassigned exterior face".into());
        }
        Ok((bc, cyclic))
    }
}

#[cfg(test)]
mod total_inlet_tests {
    use super::*;
    #[test]
    fn total_inlet_recovers_isentropic_state_and_stagnation_conditions() {
        let gas = IdealGas::default();
        let (pt, tt, mach) = (101325.0, 300.0, 0.3_f64);
        let t = tt / (1.0 + 0.5 * (gas.gamma - 1.0) * mach * mach);
        let p = pt * (t / tt).powf(gas.gamma / (gas.gamma - 1.0));
        let owner = Primitive {
            density: p / (gas.gas_constant * t),
            pressure: p,
            velocity: Vec3::new(mach * (gas.gamma * gas.gas_constant * t).sqrt(), 0., 0.),
        };
        let boundary = Boundary::TotalInlet {
            pressure: pt,
            temperature: tt,
            direction: Vec3::new(1., 0., 0.),
        };
        let state = boundary.ghost(owner, Vec3::new(-1., 0., 0.), gas).unwrap();
        assert!((state.velocity.x - owner.velocity.x).abs() < 1e-10);
        assert!((state.pressure - owner.pressure).abs() < 1e-8);
        let total_temperature =
            gas.temperature(state) + state.velocity.norm_squared() / (2. * gas.cp());
        assert!((total_temperature - tt).abs() < 1e-10);
        assert!(boundary.ghost(owner, Vec3::new(1., 0., 0.), gas).is_err());
    }
}
