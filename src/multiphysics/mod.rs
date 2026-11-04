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
pub mod species;
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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_transport_courant: Option<f64>,
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
            max_transport_courant: None,
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
pub struct PhaseBudget {
    pub initial_volume: f64,
    pub final_volume: f64,
    pub net_boundary_outflow: f64,
    pub volume_balance_error: f64,
    pub initial_mass: f64,
    pub final_mass: f64,
    pub net_boundary_mass_outflow: f64,
    pub mass_balance_error: f64,
}

#[derive(Clone, Debug, Serialize)]
pub struct StepReport {
    pub step: usize,
    pub time: f64,
    pub time_step: f64,
    pub transport_courant: f64,
    pub volume_unit: &'static str,
    pub mass_unit: &'static str,
    pub phase: phase::PhaseReport,
    pub phase_budgets: [PhaseBudget; 2],
    pub energy: energy::EnergyReport,
    pub initial_fluid_mass: f64,
    pub final_fluid_mass: f64,
    pub net_boundary_mass_outflow: f64,
    pub mass_balance_error: f64,
    pub max_cell_mass_residual: f64,
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
    if options
        .max_transport_courant
        .is_some_and(|v| !v.is_finite() || v <= 0.0 || v > 1.0)
    {
        return Err("Maximum transport Courant must be finite and in (0, 1]".into());
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
    let mut outgoing = vec![0.0; n];
    for (i, face) in mesh.faces().iter().enumerate() {
        outgoing[face.owner] += flux[i].max(0.0);
        if let Some(neighbour) = face.neighbour {
            outgoing[neighbour] += (-flux[i]).max(0.0);
        }
    }
    let max_rate = outgoing
        .iter()
        .zip(&metrics.volumes)
        .map(|(q, v)| q / v)
        .fold(0.0_f64, f64::max);
    if !max_rate.is_finite() {
        return Err("Invalid transport Courant rate".into());
    }
    let actual_dt = options
        .max_transport_courant
        .filter(|_| max_rate > 0.0)
        .map_or(options.dt, |limit| options.dt.min(limit / max_rate));
    if !actual_dt.is_finite() || actual_dt <= 0.0 {
        return Err("Transport time step is too small".into());
    }
    let transport_courant = actual_dt * max_rate;
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
            actual_dt,
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
        let mass_flux = phase::mixture_mass_flux(
            &phase_report.phase_flux,
            &flux,
            options.first.density,
            options.second.density,
        )?;
        let mut mass_divergence = vec![0.0; n];
        for (i, face) in mesh.faces().iter().enumerate() {
            mass_divergence[face.owner] += mass_flux[i];
            if let Some(neighbour) = face.neighbour {
                mass_divergence[neighbour] -= mass_flux[i];
            }
        }
        let mut max_cell_mass_residual = 0.0_f64;
        for i in 0..n {
            if !state.solid[i] {
                let residual = (density[i] - state.density[i]) / actual_dt
                    + mass_divergence[i] / metrics.volumes[i];
                if !residual.is_finite() {
                    return Err(format!("Non-finite mass continuity residual in cell {i}"));
                }
                max_cell_mass_residual = max_cell_mass_residual.max(residual.abs());
            }
        }
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
            actual_dt,
            &options.energy,
            linear,
        )?;
        state.step = step;
        state.time += actual_dt;
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
        let phase_budget = |initial: f64, final_volume: f64, outflow: f64, density: f64| {
            let volume_balance_error = final_volume - initial + actual_dt * outflow;
            PhaseBudget {
                initial_volume: initial,
                final_volume,
                net_boundary_outflow: outflow,
                volume_balance_error,
                initial_mass: density * initial,
                final_mass: density * final_volume,
                net_boundary_mass_outflow: density * outflow,
                mass_balance_error: density * volume_balance_error,
            }
        };
        let phase_budgets = [
            phase_budget(
                phase_report.initial_volume,
                phase_report.final_volume,
                phase_report.net_boundary_outflow,
                options.first.density,
            ),
            phase_budget(
                fluid_volume - phase_report.initial_volume,
                fluid_volume - phase_report.final_volume,
                net_volume_flux - phase_report.net_boundary_outflow,
                options.second.density,
            ),
        ];
        observer(
            &state,
            &StepReport {
                step,
                time: state.time,
                time_step: actual_dt,
                transport_courant,
                volume_unit: if mesh.dimension() == crate::MeshDimension::TwoD
                    && options.coordinates == metrics::Coordinates::Cartesian
                {
                    "m²"
                } else {
                    "m³"
                },
                mass_unit: if mesh.dimension() == crate::MeshDimension::TwoD
                    && options.coordinates == metrics::Coordinates::Cartesian
                {
                    "kg/m"
                } else {
                    "kg"
                },
                phase: phase_report,
                phase_budgets,
                energy: energy_report,
                initial_fluid_mass: initial_mass,
                final_fluid_mass: final_mass,
                net_boundary_mass_outflow: boundary_mass,
                mass_balance_error: final_mass - initial_mass + actual_dt * boundary_mass,
                max_cell_mass_residual,
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

#[cfg(test)]
mod adaptive_transport_tests {
    use super::*;
    use crate::{BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point};

    #[test]
    fn courant_limit_controls_all_equations_and_restart_time() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(2., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(1., 1., 0.),
                Point::new(2., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 4, 3]),
                CellDefinition::polygon(vec![1, 2, 5, 4]),
            ],
        )
        .unwrap();
        let faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![BoundaryPatch {
                name: "boundary".into(),
                face_indices: faces,
                boundary_type: BoundaryType::ZeroGradient,
            }])
            .unwrap();
        let mut options = Options {
            dt: 1.0,
            steps: 2,
            max_transport_courant: Some(0.5),
            initial_phase_fraction: 0.0,
            mesh_signature: Some(mesh_signature(&mesh)),
            face_flux: Some(mesh.faces().iter().map(|f| f.area_vector.x).collect()),
            ..Default::default()
        };
        options.phase.scheme = phase::AdvectionScheme::FluxCorrected;
        options.phase.incoming.insert("boundary".into(), 1.0);
        options.energy.boundaries.insert(
            "boundary".into(),
            energy::Boundary::InletOutlet {
                incoming_temperature: 293.15,
            },
        );
        let mut reports = Vec::new();
        let state = run(&mesh, &options, LinearSolverOptions::default(), |_, r| {
            reports.push((
                r.time_step,
                r.transport_courant,
                r.mass_balance_error,
                r.energy.balance_error / r.energy.initial_energy.abs().max(1.0),
                r.max_cell_mass_residual,
                r.phase_budgets[0].mass_balance_error,
                r.phase_budgets[1].mass_balance_error,
            ));
            Ok(())
        })
        .unwrap();
        assert_eq!(state.time, 1.0);
        assert!(
            reports.iter().all(|r| r.0 == 0.5
                && r.1 == 0.5
                && r.2.abs() < 1e-10
                && r.3.abs() < 1e-12
                && r.4 < 1e-10
                && r.5.abs() < 1e-10
                && r.6.abs() < 1e-10),
            "{reports:?}"
        );
        let checkpoint = checkpoint::Checkpoint::capture(&mesh, &options, &state).unwrap();
        let continued = run_controlled(
            &mesh,
            &options,
            LinearSolverOptions::default(),
            Some(&checkpoint),
            None,
            |_, _| Ok(()),
        )
        .unwrap();
        assert_eq!(continued.time, 2.0);
        assert_eq!(continued.step, 4);
        options.max_transport_courant = None;
        let fixed = run(&mesh, &options, LinearSolverOptions::default(), |_, _| {
            Ok(())
        })
        .unwrap();
        assert_eq!(fixed.time, 2.0);
        options.max_transport_courant = Some(0.0);
        assert!(
            run(&mesh, &options, LinearSolverOptions::default(), |_, _| Ok(
                ()
            ))
            .is_err()
        );
    }
}
