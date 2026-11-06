use super::{
    coupled,
    metrics::Coordinates,
    phase::{self, AdvectionScheme, PhaseOptions},
};
use crate::{CellField, Diffusivity, FaceField, SimpleOptions, SimpleState, UnstructuredMesh};
use std::collections::{BTreeMap, HashSet};

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MaterialPhase {
    pub name: String,
    pub density: f64,
    pub viscosity: f64,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Interface {
    pub first: usize,
    pub second: usize,
    pub surface_tension: f64,
    #[serde(default)]
    pub contact_angles: BTreeMap<String, f64>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Settings {
    pub phases: Vec<MaterialPhase>,
    #[serde(default)]
    pub interfaces: Vec<Interface>,
    pub incoming: BTreeMap<String, Vec<f64>>,
    pub phase: PhaseOptions,
    pub solver: Controls,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Controls {
    pub gravity: crate::Vec3,
    pub time_step: f64,
    pub max_courant: Option<f64>,
    pub min_time_step: f64,
    pub max_step_retries: usize,
    pub max_iterations: usize,
    pub velocity_tolerance: f64,
    pub flux_tolerance: f64,
    pub phase_tolerance: f64,
    pub pressure_tolerance: f64,
    pub momentum_absolute_tolerance: f64,
    pub momentum_relative_tolerance: f64,
}
impl Default for Controls {
    fn default() -> Self {
        let s = coupled::Settings::default();
        Self {
            gravity: s.gravity,
            time_step: s.time_step,
            max_courant: s.max_courant,
            min_time_step: s.min_time_step,
            max_step_retries: s.max_step_retries,
            max_iterations: s.max_iterations,
            velocity_tolerance: s.velocity_tolerance,
            flux_tolerance: s.flux_tolerance,
            phase_tolerance: s.phase_tolerance,
            pressure_tolerance: s.pressure_tolerance,
            momentum_absolute_tolerance: s.momentum_absolute_tolerance,
            momentum_relative_tolerance: s.momentum_relative_tolerance,
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhaseRegion {
    pub minimum: crate::Vec3,
    pub maximum: crate::Vec3,
    pub fractions: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Configuration {
    pub settings: Settings,
    pub initial_fractions: Vec<f64>,
    pub regions: Vec<PhaseRegion>,
    pub steps: usize,
    pub checkpoint_interval: usize,
}
impl Default for Configuration {
    fn default() -> Self {
        Self {
            settings: Settings {
                phases: vec![
                    MaterialPhase {
                        name: "phase 1".into(),
                        density: 1000.0,
                        viscosity: 0.001,
                    },
                    MaterialPhase {
                        name: "phase 2".into(),
                        density: 1.2,
                        viscosity: 0.000018,
                    },
                ],
                interfaces: Vec::new(),
                incoming: BTreeMap::new(),
                phase: PhaseOptions::default(),
                solver: Controls::default(),
            },
            initial_fractions: vec![1.0, 0.0],
            regions: Vec::new(),
            steps: 100,
            checkpoint_interval: 10,
        }
    }
}
impl Configuration {
    pub fn initial_fields(&self, mesh: &UnstructuredMesh) -> Result<Vec<Vec<f64>>, String> {
        if self.steps == 0
            || self.checkpoint_interval == 0
            || self.initial_fractions.len() != self.settings.phases.len()
        {
            return Err(
                "Invalid multiphase steps, checkpoint interval, or initial fractions".into(),
            );
        }
        let mut fields: Vec<_> = self
            .initial_fractions
            .iter()
            .map(|a| vec![*a; mesh.cell_count()])
            .collect();
        let mut assigned = vec![false; mesh.cell_count()];
        for region in &self.regions {
            let a = region.minimum;
            let b = region.maximum;
            if region.fractions.len() != fields.len()
                || [a.x, a.y, a.z, b.x, b.y, b.z]
                    .iter()
                    .any(|v| !v.is_finite())
                || a.x > b.x
                || a.y > b.y
                || a.z > b.z
            {
                return Err("Invalid multiphase region bounds or fractions".into());
            }
            for (i, cell) in mesh.cells().iter().enumerate() {
                let p = cell.center;
                if p.x >= a.x && p.x <= b.x && p.y >= a.y && p.y <= b.y && p.z >= a.z && p.z <= b.z
                {
                    if assigned[i] {
                        return Err(format!("Overlapping multiphase regions at cell {i}"));
                    }
                    assigned[i] = true;
                    for (field, value) in fields.iter_mut().zip(&region.fractions) {
                        field[i] = *value;
                    }
                }
            }
            if region
                .fractions
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
                || (region.fractions.iter().sum::<f64>() - 1.0).abs() > 1e-10
            {
                return Err("Region fractions must be bounded and sum to one".into());
            }
        }
        validate(&self.settings, &fields, mesh.cell_count())?;
        if self
            .initial_fractions
            .iter()
            .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
            || (self.initial_fractions.iter().sum::<f64>() - 1.0).abs() > 1e-10
        {
            return Err("Initial fractions must be bounded and sum to one".into());
        }
        Ok(fields)
    }
}

#[derive(Clone, Debug)]
pub struct State {
    pub flow: SimpleState,
    pub fractions: Vec<Vec<f64>>,
    pub time: f64,
    pub step: usize,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Checkpoint {
    pub format_version: u32,
    pub mesh_signature: String,
    pub physics_signature: String,
    pub fractions: Vec<Vec<f64>>,
    pub velocity: Vec<crate::Vec3>,
    pub pressure: Vec<f64>,
    pub face_flux: Vec<f64>,
    pub time: f64,
    pub step: usize,
}

fn checkpoint_signature(
    mesh: &UnstructuredMesh,
    settings: &Settings,
    simple: SimpleOptions<'_>,
) -> Result<String, String> {
    use sha2::{Digest, Sha256};
    let physical = coupled::Settings {
        gravity: settings.solver.gravity,
        phase: settings.phase.clone(),
        ..Default::default()
    };
    let boundary = coupled::physics_signature(mesh, &physical, simple)?;
    let data = serde_json::to_vec(&(
        boundary,
        &settings.phases,
        &settings.interfaces,
        &settings.incoming,
        &settings.phase,
    ))
    .map_err(|e| e.to_string())?;
    Ok(format!("{:x}", Sha256::digest(data)))
}

impl Checkpoint {
    pub fn capture(
        mesh: &UnstructuredMesh,
        state: &State,
        settings: &Settings,
        simple: SimpleOptions<'_>,
    ) -> Result<Self, String> {
        state
            .flow
            .velocity
            .ensure_mesh(mesh)
            .map_err(|e| format!("{e:?}"))?;
        state
            .flow
            .pressure
            .ensure_mesh(mesh)
            .map_err(|e| format!("{e:?}"))?;
        state
            .flow
            .face_flux
            .ensure_mesh(mesh)
            .map_err(|e| format!("{e:?}"))?;
        let result = Self {
            format_version: 1,
            mesh_signature: super::mesh_signature(mesh),
            physics_signature: checkpoint_signature(mesh, settings, simple)?,
            fractions: state.fractions.clone(),
            velocity: state.flow.velocity.values().to_vec(),
            pressure: state.flow.pressure.values().to_vec(),
            face_flux: state.flow.face_flux.values().to_vec(),
            time: state.time,
            step: state.step,
        };
        result.restore(mesh, settings, simple)?;
        Ok(result)
    }
    pub fn save(&self, path: &std::path::Path) -> Result<(), String> {
        crate::storage::write_json(path, self, None, || Ok(()))?;
        Ok(())
    }
    pub fn load(path: &std::path::Path) -> Result<Self, String> {
        serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())
    }
    pub fn restore(
        &self,
        mesh: &UnstructuredMesh,
        settings: &Settings,
        simple: SimpleOptions<'_>,
    ) -> Result<State, String> {
        if self.format_version != 1
            || self.mesh_signature != super::mesh_signature(mesh)
            || self.physics_signature != checkpoint_signature(mesh, settings, simple)?
        {
            return Err("Multiphase checkpoint requires matching mesh, materials, phase boundaries and flow boundaries".into());
        }
        validate(settings, &self.fractions, mesh.cell_count())?;
        if !self.time.is_finite()
            || self.time < 0.0
            || self
                .velocity
                .iter()
                .any(|v| !v.x.is_finite() || !v.y.is_finite() || !v.z.is_finite())
            || self
                .pressure
                .iter()
                .chain(&self.face_flux)
                .any(|v| !v.is_finite())
        {
            return Err("Invalid multiphase checkpoint fields or time".into());
        }
        let velocity =
            CellField::from_values(mesh, self.velocity.clone()).map_err(|e| format!("{e:?}"))?;
        let pressure =
            CellField::from_values(mesh, self.pressure.clone()).map_err(|e| format!("{e:?}"))?;
        let flux =
            FaceField::from_values(mesh, self.face_flux.clone()).map_err(|e| format!("{e:?}"))?;
        Ok(State {
            flow: SimpleState::new(mesh, velocity, pressure, flux).map_err(|e| e.to_string())?,
            fractions: self.fractions.clone(),
            time: self.time,
            step: self.step,
        })
    }
}

#[derive(Clone, Debug)]
pub struct TransportReport {
    pub phase_fluxes: Vec<Vec<f64>>,
    pub mass_flux: Vec<f64>,
    pub balance_errors: Vec<f64>,
}

fn validate(settings: &Settings, fractions: &[Vec<f64>], cells: usize) -> Result<(), String> {
    if settings.phases.len() < 2 || settings.phases.len() != fractions.len() {
        return Err(
            "At least two material phases and one fraction field per phase are required".into(),
        );
    }
    let mut names = HashSet::new();
    for (p, values) in settings.phases.iter().zip(fractions) {
        if p.name.trim().is_empty()
            || !names.insert(&p.name)
            || !p.density.is_finite()
            || p.density <= 0.0
            || !p.viscosity.is_finite()
            || p.viscosity <= 0.0
            || values.len() != cells
            || values
                .iter()
                .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
        {
            return Err("Invalid phase name, properties, or fraction field".into());
        }
    }
    let mut pairs = HashSet::new();
    for interface in &settings.interfaces {
        if interface.first >= interface.second
            || interface.second >= settings.phases.len()
            || !pairs.insert((interface.first, interface.second))
            || !interface.surface_tension.is_finite()
            || interface.surface_tension < 0.0
            || interface
                .contact_angles
                .values()
                .any(|angle| !angle.is_finite() || !(0.0..=180.0).contains(angle))
        {
            return Err("Invalid or duplicate phase interface or contact angle".into());
        }
    }
    for i in 0..cells {
        if (fractions.iter().map(|f| f[i]).sum::<f64>() - 1.0).abs() > 1e-10 {
            return Err(format!("Phase fractions do not sum to one in cell {i}"));
        }
    }
    for values in settings.incoming.values() {
        if values.len() != fractions.len()
            || values
                .iter()
                .any(|a| !a.is_finite() || !(0.0..=1.0).contains(a))
            || (values.iter().sum::<f64>() - 1.0).abs() > 1e-10
        {
            return Err("Incoming phase fractions must be bounded and sum to one".into());
        }
    }
    if settings.phase.scheme != AdvectionScheme::Upwind || !settings.phase.incoming.is_empty() {
        return Err(
            "Multiphase transport requires upwind and per-patch multiphase incoming fractions"
                .into(),
        );
    }
    Ok(())
}

pub fn transport(
    mesh: &UnstructuredMesh,
    flux: &FaceField<f64>,
    fractions: &mut Vec<Vec<f64>>,
    dt: f64,
    settings: &Settings,
) -> Result<TransportReport, String> {
    validate(settings, fractions, mesh.cell_count())?;
    flux.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
    let measures = Coordinates::Cartesian.measures(mesh)?;
    let mut next = fractions.clone();
    let mut phase_fluxes = Vec::with_capacity(next.len());
    let mut balance_errors = Vec::with_capacity(next.len());
    for (i, values) in next.iter_mut().enumerate() {
        let mut options = settings.phase.clone();
        options.incoming = settings
            .incoming
            .iter()
            .map(|(name, alpha)| (name.clone(), alpha[i]))
            .collect();
        let report = phase::advance(mesh, &measures, flux.values(), values, dt, &options)?;
        phase_fluxes.push(report.phase_flux);
        balance_errors.push(report.conservation_error);
    }
    validate(settings, &next, mesh.cell_count())?;
    let mut mass_flux = vec![0.0; mesh.face_count()];
    for face in 0..mesh.face_count() {
        let total = phase_fluxes.iter().map(|q| q[face]).sum::<f64>();
        if (total - flux[face]).abs() > 1e-10 * flux[face].abs() + 1e-13 {
            return Err(format!(
                "Phase fluxes do not sum to volume flux at face {face}"
            ));
        }
        mass_flux[face] = phase_fluxes
            .iter()
            .zip(&settings.phases)
            .map(|(q, p)| q[face] * p.density)
            .sum();
    }
    *fractions = next;
    Ok(TransportReport {
        phase_fluxes,
        mass_flux,
        balance_errors,
    })
}

fn mixture(
    mesh: &UnstructuredMesh,
    fractions: &[Vec<f64>],
    phases: &[MaterialPhase],
    density: bool,
) -> Result<CellField<f64>, String> {
    CellField::from_values(
        mesh,
        (0..mesh.cell_count())
            .map(|cell| {
                fractions
                    .iter()
                    .zip(phases)
                    .map(|(a, p)| a[cell] * if density { p.density } else { p.viscosity })
                    .sum()
            })
            .collect(),
    )
    .map_err(|e| format!("{e:?}"))
}

enum StepError {
    NonConverged,
    Other(String),
}
impl From<String> for StepError {
    fn from(error: String) -> Self {
        Self::Other(error)
    }
}
impl From<&str> for StepError {
    fn from(error: &str) -> Self {
        Self::Other(error.into())
    }
}

pub fn advance(
    mesh: &UnstructuredMesh,
    state: &mut State,
    settings: &Settings,
    simple: SimpleOptions<'_>,
) -> Result<coupled::Report, String> {
    validate(settings, &state.fractions, mesh.cell_count())?;
    let s = &settings.solver;
    if !state.time.is_finite()
        || state.time < 0.0
        || !s.time_step.is_finite()
        || s.time_step <= 0.0
        || !s.min_time_step.is_finite()
        || s.min_time_step <= 0.0
        || s.min_time_step > s.time_step
        || s.max_iterations == 0
        || s.max_step_retries > 32
        || s.max_courant
            .is_some_and(|v| !v.is_finite() || v <= 0.0 || v > 1.0)
        || [s.gravity.x, s.gravity.y, s.gravity.z]
            .iter()
            .any(|v| !v.is_finite())
        || [
            s.velocity_tolerance,
            s.flux_tolerance,
            s.phase_tolerance,
            s.pressure_tolerance,
            s.momentum_absolute_tolerance,
            s.momentum_relative_tolerance,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
    {
        return Err("Invalid multiphase time, gravity, or convergence controls".into());
    }
    for interface in &settings.interfaces {
        for patch in interface.contact_angles.keys() {
            let p = mesh
                .boundary_patches()
                .iter()
                .find(|p| &p.name == patch)
                .ok_or_else(|| format!("Unknown wetting patch {patch}"))?;
            if p.boundary_type != crate::BoundaryType::Wall
                || settings
                    .phase
                    .cyclic_pairs
                    .iter()
                    .any(|pair| &pair.first == patch || &pair.second == patch)
            {
                return Err(format!(
                    "Contact angle requires a non-periodic wall: {patch}"
                ));
            }
        }
    }
    let cyclic = crate::periodic::resolve(mesh, &settings.phase.cyclic_pairs)?;
    for i in 0..mesh.face_count() {
        let expected = cyclic.link(i).map(|l| l.peer_face);
        for component in [
            crate::MomentumComponent::X,
            crate::MomentumComponent::Y,
            crate::MomentumComponent::Z,
        ] {
            if simple
                .velocity_boundary
                .component(component)
                .cyclic_link(i)
                .map(|l| l.peer_face)
                != expected
            {
                return Err("Multiphase and velocity cyclic boundaries must match".into());
            }
        }
        if simple
            .pressure_boundary
            .and_then(|b| b.cyclic_link(i))
            .map(|l| l.peer_face)
            != expected
            || simple
                .pressure_correction_boundary
                .and_then(|b| b.cyclic_link(i))
                .map(|l| l.peer_face)
                != expected
        {
            return Err("Multiphase and pressure cyclic boundaries must match".into());
        }
        if let Some(link) = cyclic.link(i) {
            if s.gravity.dot(link.translation).abs()
                > 1e-12 * (s.gravity.norm() * link.translation.norm()).max(1.0)
            {
                return Err("Gravity cannot cross a physical-pressure periodic boundary".into());
            }
        }
    }
    let mut dt = s.time_step;
    let h = mesh
        .cells()
        .iter()
        .map(|cell| {
            let area = cell
                .faces
                .iter()
                .map(|&i| mesh.faces()[i].area)
                .fold(0.0_f64, f64::max);
            cell.volume / area
        })
        .fold(f64::INFINITY, f64::min);
    for interface in &settings.interfaces {
        if interface.surface_tension > 0.0 {
            let rho = settings.phases[interface.first].density
                + settings.phases[interface.second].density;
            let limit = 0.5
                * (rho * h.powi(3) / (2.0 * std::f64::consts::PI * interface.surface_tension))
                    .sqrt();
            if !limit.is_finite() || limit <= 0.0 {
                return Err("Invalid capillary time-step limit".into());
            }
            dt = dt.min(limit);
        }
    }
    let initial_rate = coupled::courant_rate(mesh, &state.flow.face_flux)?;
    if let Some(limit) = s.max_courant {
        if initial_rate > 0.0 {
            dt = dt.min(limit / initial_rate);
        }
    }
    for rejected in 0..=s.max_step_retries {
        if dt < s.min_time_step {
            return Err(
                "Multiphase stability or convergence requires a step below the minimum; state retained".into(),
            );
        }
        let mut candidate = state.clone();
        let mut report = match advance_attempt(mesh, &mut candidate, settings, simple, dt) {
            Ok(report) => report,
            Err(StepError::NonConverged) => {
                dt *= 0.5;
                continue;
            }
            Err(StepError::Other(error)) => return Err(error),
        };
        let courant = dt * coupled::courant_rate(mesh, &candidate.flow.face_flux)?;
        if s.max_courant
            .is_none_or(|limit| courant <= limit * (1.0 + 1e-10))
        {
            report.time_step = dt;
            report.max_courant = courant;
            report.rejected_attempts = rejected;
            *state = candidate;
            return Ok(report);
        }
        dt *= (0.9 * s.max_courant.unwrap() / courant).min(0.5);
    }
    Err("Multiphase Courant or convergence retry budget exceeded; state retained".into())
}

fn advance_attempt(
    mesh: &UnstructuredMesh,
    state: &mut State,
    settings: &Settings,
    simple: SimpleOptions<'_>,
    dt: f64,
) -> Result<coupled::Report, StepError> {
    let s = &settings.solver;
    let next_step = state
        .step
        .checked_add(1)
        .ok_or("Multiphase step overflow")?;
    let next_time = state.time + dt;
    if !next_time.is_finite() || next_time <= state.time {
        return Err("Invalid multiphase time increment".into());
    }
    let min_volume = mesh
        .cells()
        .iter()
        .map(|c| c.volume)
        .fold(f64::INFINITY, f64::min);
    let pressure_limit = (5e-14 * min_volume / dt).max(f64::MIN_POSITIVE);
    let mut pressure_solver = simple.pressure_solver;
    pressure_solver.absolute_tolerance = if pressure_solver.absolute_tolerance > 0.0 {
        pressure_solver.absolute_tolerance.min(pressure_limit)
    } else {
        pressure_limit
    };
    pressure_solver.relative_tolerance = 0.0;
    let mut equations = simple.equations.cloned();
    if let Some(e) = &mut equations {
        e.pressure.absolute_tolerance = if e.pressure.absolute_tolerance > 0.0 {
            e.pressure.absolute_tolerance.min(pressure_limit)
        } else {
            pressure_limit
        };
        e.pressure.relative_tolerance = 0.0;
    }
    let old = state.fractions.clone();
    let old_velocity = state.flow.velocity.clone();
    let old_flux = state.flow.face_flux.clone();
    let old_density = mixture(mesh, &old, &settings.phases, true)?;
    let mut flow = state.flow.clone();
    let mut momentum_reference = 0.0_f64;
    let mut workspace = crate::simple::SimpleWorkspace::default();
    for iteration in 1..=s.max_iterations {
        crate::compute::checkpoint().map_err(|e| e.to_string())?;
        let previous = flow.clone();
        let mut fractions = old.clone();
        let transport_report = transport(mesh, &previous.face_flux, &mut fractions, dt, settings)?;
        let density = mixture(mesh, &fractions, &settings.phases, true)?;
        let viscosity = mixture(mesh, &fractions, &settings.phases, false)?;
        let mass_flux = FaceField::from_values(mesh, transport_report.mass_flux)
            .map_err(|e| format!("{e:?}"))?;
        let surface_force = if settings.interfaces.iter().any(|i| i.surface_tension > 0.0) {
            Some(super::capillary::multiphase_surface_force(
                mesh,
                &fractions,
                &settings.interfaces,
                simple.pressure_boundary,
            )?)
        } else {
            None
        };
        let options = SimpleOptions {
            time_step: Some(dt),
            time_velocity: Some(&old_velocity),
            time_flux: Some(&old_flux),
            max_outer_iterations: 1,
            pressure_solver,
            equations: equations.as_ref(),
            viscosity: Diffusivity::CellField(&viscosity),
            variable_density: Some(crate::simple::VariableDensityInput {
                density: crate::TransientDensity {
                    current: &density,
                    previous: &old_density,
                },
                mass_flux: &mass_flux,
                gravity: s.gravity,
                surface_force: surface_force.as_ref(),
            }),
            ..simple
        };
        let report = crate::simple::solve_simple_cached(mesh, &mut flow, options, &mut workspace)
            .map_err(|e| e.to_string())?;
        let momentum = report.final_momentum_initial_residual;
        momentum_reference = momentum_reference.max(momentum);
        let relative = if momentum_reference > 0.0 {
            momentum / momentum_reference
        } else {
            0.0
        };
        let velocity_change = flow
            .velocity
            .values()
            .iter()
            .zip(previous.velocity.values())
            .map(|(a, b)| (*a - *b).norm())
            .fold(0.0_f64, f64::max);
        let pressure_change = flow
            .pressure
            .values()
            .iter()
            .zip(previous.pressure.values())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        let flux_change = flow
            .face_flux
            .values()
            .iter()
            .zip(previous.face_flux.values())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        if velocity_change <= s.velocity_tolerance
            && pressure_change <= s.pressure_tolerance
            && flux_change <= s.flux_tolerance
            && (momentum <= s.momentum_absolute_tolerance
                || relative <= s.momentum_relative_tolerance)
        {
            let mut accepted = old.clone();
            let phase_report = transport(mesh, &flow.face_flux, &mut accepted, dt, settings)?;
            let phase_change = accepted
                .iter()
                .zip(&fractions)
                .flat_map(|(a, b)| a.iter().zip(b))
                .map(|(a, b)| (a - b).abs())
                .fold(0.0_f64, f64::max);
            if phase_change <= s.phase_tolerance
                && report.final_continuity_rms <= simple.continuity_absolute_tolerance
            {
                state.flow = flow;
                state.fractions = accepted;
                state.time = next_time;
                state.step = next_step;
                return Ok(coupled::Report {
                    iterations: iteration,
                    time_step: dt,
                    max_courant: 0.0,
                    rejected_attempts: 0,
                    velocity_change,
                    flux_change,
                    phase_change,
                    pressure_change,
                    momentum_initial_residual: momentum,
                    momentum_relative_residual: relative,
                    continuity_rms: report.final_continuity_rms,
                    phase_balance: phase_report
                        .balance_errors
                        .iter()
                        .map(|e| e.abs())
                        .fold(0.0_f64, f64::max),
                });
            }
        }
    }
    Err(StepError::NonConverged)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BoundaryPatch, BoundaryType, CellDefinition, LeastSquaresGradientStencil, MeshDimension,
        Point, ResolvedScalarBoundaryConditions, ResolvedVelocityBoundaryConditions,
        ScalarBoundaryCondition, Vec3, VelocityBoundaryCondition,
    };

    #[test]
    fn three_phase_transport_and_momentum_preserve_sum_and_uniform_velocity() {
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
                boundary_type: BoundaryType::Wall,
            }])
            .unwrap();
        let velocity = Vec3::new(1., 0., 0.);
        let ub = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[(
                "boundary",
                VelocityBoundaryCondition::FixedVelocity(velocity),
            )],
        )
        .unwrap();
        let pb = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("boundary", ScalarBoundaryCondition::FixedValue(0.))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pb).unwrap();
        let mut simple = SimpleOptions::steady(Diffusivity::Constant(0.1), &ub, &stencil);
        simple.pressure_boundary = Some(&pb);
        let flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, velocity),
            CellField::filled(&mesh, 0.0),
            FaceField::from_faces(&mesh, |_, f| f.area_vector.x),
        )
        .unwrap();
        let mut state = State {
            flow,
            fractions: vec![vec![0., 0.], vec![0., 0.], vec![1., 1.]],
            time: 0.,
            step: 0,
        };
        let settings = Settings {
            phases: vec![
                MaterialPhase {
                    name: "water".into(),
                    density: 1000.,
                    viscosity: 0.1,
                },
                MaterialPhase {
                    name: "oil".into(),
                    density: 800.,
                    viscosity: 0.1,
                },
                MaterialPhase {
                    name: "gas".into(),
                    density: 1.,
                    viscosity: 0.1,
                },
            ],
            interfaces: Vec::new(),
            incoming: [("boundary".into(), vec![0.25, 0.75, 0.0])]
                .into_iter()
                .collect(),
            phase: PhaseOptions::default(),
            solver: Controls {
                gravity: Vec3::ZERO,
                time_step: 0.2,
                max_courant: None,
                velocity_tolerance: 1e-8,
                flux_tolerance: 1e-8,
                phase_tolerance: 1e-8,
                pressure_tolerance: 1e-8,
                momentum_absolute_tolerance: 1e-8,
                ..Default::default()
            },
        };
        let initial = state.clone();
        let report = advance(&mesh, &mut state, &settings, simple).unwrap();
        let mut four = initial;
        four.fractions[2].fill(0.6);
        four.fractions.push(vec![0.4; mesh.cell_count()]);
        let mut four_settings = settings.clone();
        four_settings.phases.push(MaterialPhase {
            name: "fourth".into(),
            density: 500.0,
            viscosity: 0.1,
        });
        *four_settings.incoming.get_mut("boundary").unwrap() = vec![0.2, 0.3, 0.1, 0.4];
        advance(&mesh, &mut four, &four_settings, simple).unwrap();
        for (actual, expected) in four
            .fractions
            .iter()
            .map(|phase| phase[0])
            .zip([0.04, 0.06, 0.5, 0.4])
        {
            assert!((actual - expected).abs() < 1e-10);
        }
        assert!(four
            .flow
            .velocity
            .values()
            .iter()
            .all(|u| (*u - velocity).norm() < 1e-8));
        assert!((state.fractions[0][0] - 0.05).abs() < 1e-10);
        assert!((state.fractions[1][0] - 0.15).abs() < 1e-10);
        assert!((state.fractions[2][0] - 0.8).abs() < 1e-10);
        assert!(state
            .flow
            .velocity
            .values()
            .iter()
            .all(|u| (*u - velocity).norm() < 1e-8));
        assert!(report.phase_balance < 1e-12 && report.continuity_rms < 1e-10);
        let mut capillary_state = state.clone();
        let mut capillary_settings = settings.clone();
        capillary_settings.interfaces.push(Interface {
            first: 0,
            second: 2,
            surface_tension: 0.02,
            contact_angles: Default::default(),
        });
        let capillary_report =
            advance(&mesh, &mut capillary_state, &capillary_settings, simple).unwrap();
        assert!(capillary_report.continuity_rms < 1e-10 && capillary_report.phase_balance < 1e-10);
        assert!(capillary_state
            .flow
            .pressure
            .values()
            .iter()
            .all(|v| v.is_finite()));
        let capillary_checkpoint =
            Checkpoint::capture(&mesh, &capillary_state, &capillary_settings, simple).unwrap();
        let mut changed_interface = capillary_settings.clone();
        changed_interface.interfaces[0].surface_tension *= 2.0;
        assert!(capillary_checkpoint
            .restore(&mesh, &changed_interface, simple)
            .is_err());
        let mut rejected_state = capillary_state.clone();
        rejected_state.flow.velocity[0] = Vec3::new(2.0, 0.5, 0.0);
        let saved_state = rejected_state.clone();
        let mut strict = capillary_settings.clone();
        strict.solver.max_iterations = 1;
        strict.solver.max_step_retries = 1;
        strict.solver.velocity_tolerance = 1e-30;
        strict.solver.flux_tolerance = 1e-30;
        strict.solver.pressure_tolerance = 1e-30;
        assert!(advance(&mesh, &mut rejected_state, &strict, simple).is_err());
        assert_eq!(rejected_state.fractions, saved_state.fractions);
        assert_eq!(
            rejected_state.flow.velocity.values(),
            saved_state.flow.velocity.values()
        );
        assert_eq!(rejected_state.step, saved_state.step);
        let checkpoint = Checkpoint::capture(&mesh, &state, &settings, simple).unwrap();
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("checkpoint.json");
        checkpoint.save(&path).unwrap();
        let mut resumed = Checkpoint::load(&path)
            .unwrap()
            .restore(&mesh, &settings, simple)
            .unwrap();
        let mut continued = state.clone();
        advance(&mesh, &mut resumed, &settings, simple).unwrap();
        advance(&mesh, &mut continued, &settings, simple).unwrap();
        assert_eq!(resumed.fractions, continued.fractions);
        assert_eq!(
            resumed.flow.face_flux.values(),
            continued.flow.face_flux.values()
        );
        let mut changed = settings.clone();
        changed.phases[0].density *= 0.9;
        assert!(checkpoint.restore(&mesh, &changed, simple).is_err());
        let before = state.clone();
        let mut bad = settings.clone();
        bad.incoming.get_mut("boundary").unwrap()[0] = 0.8;
        assert!(advance(&mesh, &mut state, &bad, simple).is_err());
        assert_eq!(before.fractions, state.fractions);
        assert_eq!(before.step, state.step);
    }
}
