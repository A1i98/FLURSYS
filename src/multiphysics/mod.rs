//! Prescribed-flow two-phase/solid sensible-energy transport.
//!
//! This is an independent transport path: no variable-density momentum,
//! pressure feedback, capillary force, phase change or mesh motion is implied.
pub mod archive;
mod catalog;
pub mod checkpoint;
pub mod energy;
pub use catalog::Selection as CatalogSelection;
pub mod metrics;
pub mod phase;
use crate::{LinearSolverOptions, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Properties {
    pub density: f64,
    pub specific_heat: f64,
    pub conductivity: f64,
}
impl Properties {
    fn validate(self) -> Result<(), String> {
        if [
            self.density,
            self.specific_heat,
            self.conductivity,
            self.density * self.specific_heat,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err(
                "Transport density, Cp and conductivity must be finite positive SI values".into(),
            );
        }
        Ok(())
    }
    /// Use the same validated material snapshots as the Workbench catalogue.
    pub fn from_snapshot(snapshot: &crate::materials::MaterialSnapshot) -> Result<Self, String> {
        snapshot.validate()?;
        let value = Self {
            density: snapshot.density,
            specific_heat: snapshot.specific_heat,
            conductivity: snapshot.conductivity,
        };
        value.validate()?;
        Ok(value)
    }
    fn capacity(self) -> f64 {
        self.density * self.specific_heat
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Region {
    pub minimum: Vec3,
    pub maximum: Vec3,
    pub phase_fraction: f64,
    pub temperature: f64,
    /// Solid region overrides both phase materials; advection must be zero.
    pub solid: Option<Properties>,
    pub source_w_m3: f64,
}
impl Region {
    fn contains(self, p: Vec3) -> bool {
        p.x >= self.minimum.x
            && p.x <= self.maximum.x
            && p.y >= self.minimum.y
            && p.y <= self.maximum.y
            && p.z >= self.minimum.z
            && p.z <= self.maximum.z
    }
    fn validate(self) -> Result<(), String> {
        let a = self.minimum;
        let b = self.maximum;
        if [
            a.x,
            a.y,
            a.z,
            b.x,
            b.y,
            b.z,
            self.temperature,
            self.phase_fraction,
            self.source_w_m3,
        ]
        .iter()
        .any(|v| !v.is_finite())
            || a.x > b.x
            || a.y > b.y
            || a.z > b.z
            || self.temperature <= 0.0
            || !(0.0..=1.0).contains(&self.phase_fraction)
        {
            return Err("Invalid transport region bounds/initial fields".into());
        }
        if let Some(p) = self.solid {
            p.validate()?;
        }
        Ok(())
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Options {
    /// Provenance history; numerical values remain authoritative.
    pub provenance: Vec<serde_json::Value>,
    pub catalog: Option<CatalogSelection>,
    pub coordinates: metrics::Coordinates,
    /// Required when face_flux is supplied; protects face ordering.
    pub mesh_signature: Option<String>,
    pub first: Properties,
    pub second: Properties,
    pub initial_phase_fraction: f64,
    pub initial_temperature: f64,
    pub regions: Vec<Region>,
    pub phase: phase::PhaseOptions,
    pub energy: energy::EnergyOptions,
    pub dt: f64,
    pub steps: usize,
    pub write_every: usize,
    /// Already integrated physical volume flux, owner-outward, m³/s (2D
    /// Cartesian is per unit depth). Axisymmetric requires revolved flux.
    /// Omit for pure conduction / quiescent phases. No automatic scaling.
    pub face_flux: Option<Vec<f64>>,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            provenance: Vec::new(),
            catalog: None,
            coordinates: Default::default(),
            mesh_signature: None,
            first: Properties {
                density: 1000.0,
                specific_heat: 4180.0,
                conductivity: 0.6,
            },
            second: Properties {
                density: 1.2,
                specific_heat: 1005.0,
                conductivity: 0.026,
            },
            initial_phase_fraction: 1.0,
            initial_temperature: 293.15,
            regions: Vec::new(),
            phase: Default::default(),
            energy: Default::default(),
            dt: 0.001,
            steps: 100,
            write_every: 10,
            face_flux: None,
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct State {
    pub step: usize,
    pub time: f64,
    pub phase_fraction: Vec<f64>,
    pub temperature: Vec<f64>,
    pub density: Vec<f64>,
    pub solid: Vec<bool>,
}
#[derive(Clone, Debug, Serialize)]
pub struct StepReport {
    pub step: usize,
    pub time: f64,
    pub phase: phase::PhaseReport,
    pub energy: energy::EnergyReport,
    pub initial_fluid_mass: f64,
    pub final_fluid_mass: f64,
    pub net_boundary_mass_outflow: f64,
    pub mass_balance_error: f64,
}
/// Advances phase and sensible energy with consistent mixture-capacity flux.
/// The observer receives only accepted steps. Errors leave caller-owned data
/// unchanged; frame output belongs to the caller and never overwrites projects.
pub fn run(
    mesh: &UnstructuredMesh,
    options: &Options,
    linear: LinearSolverOptions,
    observer: impl FnMut(&State, &StepReport) -> Result<(), String>,
) -> Result<State, String> {
    run_controlled(mesh, options, linear, None, None, observer)
}
pub fn run_controlled(
    mesh: &UnstructuredMesh,
    options: &Options,
    linear: LinearSolverOptions,
    restart: Option<&checkpoint::Checkpoint>,
    control: Option<crate::compute::LinearControl>,
    mut observer: impl FnMut(&State, &StepReport) -> Result<(), String>,
) -> Result<State, String> {
    if options.catalog.is_some() {
        let (resolved, _) = options.resolve_catalog()?;
        return run_controlled(mesh, &resolved, linear, restart, control, observer);
    }
    let execution =
        crate::compute::Execution::controlled(crate::compute::ComputeOptions::default(), control)?;
    let _execution_scope = crate::compute::Scope::enter(execution);
    if let Some(restart) = restart {
        restart.validate(mesh, options)?;
    }
    options.first.validate()?;
    options.second.validate()?;
    if !options.dt.is_finite()
        || options.dt <= 0.0
        || options.steps == 0
        || options.steps > 1_000_000
        || options.write_every == 0
        || !(options.dt * options.steps as f64).is_finite()
        || !options.initial_phase_fraction.is_finite()
        || !(0.0..=1.0).contains(&options.initial_phase_fraction)
        || !options.initial_temperature.is_finite()
        || options.initial_temperature <= 0.0
    {
        return Err(
            "Invalid transport initial values, dt, step budget or recording interval".into(),
        );
    }
    for r in &options.regions {
        r.validate()?;
    }
    if options
        .mesh_signature
        .as_ref()
        .is_some_and(|s| s != &mesh_signature(mesh))
        || (options.face_flux.is_some() && options.mesh_signature.is_none())
    {
        return Err(
            "Transport mesh signature mismatch or missing signature for supplied face flux".into(),
        );
    }
    let metrics = options.coordinates.measures(mesh)?;
    let n = mesh.cell_count();
    let flux = options
        .face_flux
        .clone()
        .unwrap_or_else(|| vec![0.0; mesh.face_count()]);
    if flux.len() != mesh.face_count() || flux.iter().any(|q| !q.is_finite()) {
        return Err("Transport face_flux must match restored mesh face ordering".into());
    }
    let mut alpha = vec![options.initial_phase_fraction; n];
    let mut temperature = vec![options.initial_temperature; n];
    let mut solid = vec![false; n];
    let mut material = vec![None; n];
    let mut source = vec![0.0; n];
    for (i, c) in mesh.cells().iter().enumerate() {
        let mut assigned = false;
        for region in &options.regions {
            if region.contains(c.center) {
                if assigned {
                    return Err(format!("Overlapping transport initial regions at cell {i}"));
                }
                assigned = true;
                alpha[i] = region.phase_fraction;
                temperature[i] = region.temperature;
                material[i] = region.solid;
                solid[i] = region.solid.is_some();
                source[i] = region.source_w_m3;
            }
        }
    }
    for (i, f) in mesh.faces().iter().enumerate() {
        if (solid[f.owner] || f.neighbour.is_some_and(|j| solid[j])) && flux[i] != 0.0 {
            return Err(format!("Nonzero prescribed flow through solid face {i}"));
        }
    }
    let blend = |a: f64, x: f64, y: f64| a * x + (1.0 - a) * y;
    let properties = |alpha: &[f64]| {
        let mut capacity = Vec::with_capacity(n);
        let mut k = Vec::with_capacity(n);
        let mut density = Vec::with_capacity(n);
        for i in 0..n {
            if let Some(p) = material[i] {
                capacity.push(p.capacity());
                k.push(p.conductivity);
                density.push(p.density);
            } else {
                let a = alpha[i];
                capacity.push(blend(
                    a,
                    options.first.capacity(),
                    options.second.capacity(),
                ));
                k.push(blend(
                    a,
                    options.first.conductivity,
                    options.second.conductivity,
                ));
                density.push(blend(a, options.first.density, options.second.density));
            }
        }
        (capacity, k, density)
    };
    let (mut old_capacity, _, initial_density) = properties(&alpha);
    let mut state = State {
        step: 0,
        time: 0.0,
        phase_fraction: alpha,
        temperature,
        density: initial_density,
        solid,
    };
    if let Some(restart) = restart {
        if restart.state.solid != state.solid {
            return Err("Restart solid mask does not match configured regions".into());
        }
        state = restart.state.clone();
        let expected = properties(&state.phase_fraction);
        if state
            .density
            .iter()
            .zip(&expected.2)
            .any(|(a, b)| (a - b).abs() > 1e-10 * b.abs().max(1.0))
        {
            return Err("Restart density does not match frozen phase properties".into());
        }
        old_capacity = expected.0;
    }
    let end_step = state
        .step
        .checked_add(options.steps)
        .ok_or("Scalar restart step counter overflow")?;
    for step in state.step + 1..=end_step {
        crate::compute::checkpoint().map_err(|e| e.to_string())?;

        let mut next_alpha = state.phase_fraction.clone();
        let mut phase_report = phase::advance(
            mesh,
            &metrics,
            &flux,
            &mut next_alpha,
            options.dt,
            &options.phase,
        )?;
        // Phase inventories describe fluid only. The immutable solid occupies
        // geometric volume but must not masquerade as either fluid phase.
        let solid_phase_volume = state
            .solid
            .iter()
            .enumerate()
            .filter(|(_, s)| **s)
            .map(|(i, _)| state.phase_fraction[i] * metrics.volumes[i])
            .sum::<f64>();
        phase_report.initial_volume -= solid_phase_volume;
        phase_report.final_volume -= solid_phase_volume;
        let mut fluid_fractions = next_alpha
            .iter()
            .enumerate()
            .filter(|(i, _)| !state.solid[*i])
            .map(|(_, a)| *a);
        if let Some(first) = fluid_fractions.next() {
            phase_report.minimum = first;
            phase_report.maximum = first;
            for a in fluid_fractions {
                phase_report.minimum = phase_report.minimum.min(a);
                phase_report.maximum = phase_report.maximum.max(a);
            }
        } else {
            phase_report.minimum = 0.0;
            phase_report.maximum = 0.0;
        }

        let (capacity, k, density) = properties(&next_alpha);
        let capacity_flux = phase_report
            .phase_flux
            .iter()
            .zip(&flux)
            .map(|(a, q)| options.first.capacity() * a + options.second.capacity() * (q - a))
            .collect::<Vec<_>>();
        let mut next_temperature = state.temperature.clone();
        let energy_report = energy::advance(
            mesh,
            &metrics,
            &old_capacity,
            &capacity,
            &k,
            &capacity_flux,
            &source,
            &state.solid,
            &mut next_temperature,
            options.dt,
            &options.energy,
            linear,
        )?;
        state.step = step;
        state.time += options.dt;
        if !state.time.is_finite() {
            return Err("Scalar physical time overflow".into());
        }
        state.phase_fraction = next_alpha;
        state.temperature = next_temperature;
        state.density = density;
        old_capacity = capacity;
        let fluid_volume = state
            .solid
            .iter()
            .enumerate()
            .filter(|(_, v)| !**v)
            .map(|(i, _)| metrics.volumes[i])
            .sum::<f64>();
        let net_volume_flux = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_none())
            .map(|(i, _)| flux[i])
            .sum::<f64>();
        let rho_difference = options.first.density - options.second.density;
        let initial_mass =
            options.second.density * fluid_volume + rho_difference * phase_report.initial_volume;
        let final_mass =
            options.second.density * fluid_volume + rho_difference * phase_report.final_volume;
        let boundary_mass = options.second.density * net_volume_flux
            + rho_difference * phase_report.net_boundary_outflow;
        observer(
            &state,
            &StepReport {
                step,
                time: state.time,
                phase: phase_report,
                energy: energy_report,
                initial_fluid_mass: initial_mass,
                final_fluid_mass: final_mass,
                net_boundary_mass_outflow: boundary_mass,
                mass_balance_error: final_mass - initial_mass + options.dt * boundary_mass,
            },
        )?;
    }
    Ok(state)
}

/// Geometry, connectivity and patch-order identity for external face fluxes.
pub fn mesh_signature(mesh: &UnstructuredMesh) -> String {
    use sha2::{Digest, Sha256};
    let mut h = Sha256::new();
    h.update(format!(
        "{:?}:{}:{}",
        mesh.dimension(),
        mesh.cell_count(),
        mesh.face_count()
    ));
    for p in mesh.points() {
        for x in [p.position.x, p.position.y, p.position.z] {
            h.update(x.to_bits().to_le_bytes());
        }
    }
    for f in mesh.faces() {
        h.update((f.owner as u64).to_le_bytes());
        h.update((f.neighbour.map_or(u64::MAX, |v| v as u64)).to_le_bytes());
        h.update((f.vertices.len() as u64).to_le_bytes());
        for &v in &f.vertices {
            h.update((v as u64).to_le_bytes());
        }
    }
    for p in mesh.boundary_patches() {
        h.update((p.name.len() as u64).to_le_bytes());
        h.update(p.name.as_bytes());
        for &f in &p.face_indices {
            h.update((f as u64).to_le_bytes());
        }
    }
    format!("{:x}", h.finalize())
}
