use super::{
    metrics::Coordinates,
    phase::{self, PhaseOptions},
};
use crate::{
    CellField, Diffusivity, FaceField, SimpleOptions, SimpleState, UnstructuredMesh, Vec3,
};

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Settings {
    pub first_density: f64,
    pub second_density: f64,
    pub first_viscosity: f64,
    pub second_viscosity: f64,
    pub gravity: Vec3,
    pub surface_tension: f64,
    pub contact_angle_degrees: Option<f64>,
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
    pub phase: PhaseOptions,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            first_density: 1000.0,
            second_density: 1.2,
            first_viscosity: 0.001,
            second_viscosity: 0.000018,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            surface_tension: 0.0,
            contact_angle_degrees: None,
            time_step: 0.001,
            max_courant: Some(0.5),
            min_time_step: 1e-8,
            max_step_retries: 8,
            max_iterations: 200,
            velocity_tolerance: 1e-7,
            flux_tolerance: 1e-9,
            phase_tolerance: 1e-7,
            pressure_tolerance: 1e-4,
            momentum_absolute_tolerance: 1e-6,
            momentum_relative_tolerance: 1e-6,
            phase: PhaseOptions::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct State {
    pub flow: SimpleState,
    pub phase_fraction: Vec<f64>,
    pub time: f64,
    pub step: usize,
}

#[derive(Clone, Debug)]
pub struct Report {
    pub iterations: usize,
    pub time_step: f64,
    pub max_courant: f64,
    pub rejected_attempts: usize,
    pub velocity_change: f64,
    pub flux_change: f64,
    pub phase_change: f64,
    pub pressure_change: f64,
    pub momentum_initial_residual: f64,
    pub momentum_relative_residual: f64,
    pub continuity_rms: f64,
    pub phase_balance: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Checkpoint {
    pub format_version: u32,
    pub mesh_signature: String,
    pub physics_signature: String,
    pub step: usize,
    pub time: f64,
    pub phase_fraction: Vec<f64>,
    pub velocity: Vec<Vec3>,
    pub pressure: Vec<f64>,
    pub face_flux: Vec<f64>,
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
        let checkpoint = Self {
            format_version: 1,
            mesh_signature: super::mesh_signature(mesh),
            physics_signature: physics_signature(mesh, settings, simple)?,
            step: state.step,
            time: state.time,
            phase_fraction: state.phase_fraction.clone(),
            velocity: state.flow.velocity.values().to_vec(),
            pressure: state.flow.pressure.values().to_vec(),
            face_flux: state.flow.face_flux.values().to_vec(),
        };
        checkpoint.restore(mesh, settings, simple)?;
        Ok(checkpoint)
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
            || self.physics_signature != physics_signature(mesh, settings, simple)?
        {
            return Err(
                "Coupled checkpoint requires the same mesh, phase properties and boundaries".into(),
            );
        }
        if !self.time.is_finite()
            || self.time < 0.0
            || self.phase_fraction.len() != mesh.cell_count()
            || self
                .phase_fraction
                .iter()
                .any(|a| !a.is_finite() || !(0.0..=1.0).contains(a))
            || self
                .velocity
                .iter()
                .any(|v| [v.x, v.y, v.z].iter().any(|x| !x.is_finite()))
            || self
                .pressure
                .iter()
                .chain(&self.face_flux)
                .any(|x| !x.is_finite())
        {
            return Err("Invalid coupled checkpoint fields or time".into());
        }
        let velocity =
            CellField::from_values(mesh, self.velocity.clone()).map_err(|e| format!("{e:?}"))?;
        let pressure =
            CellField::from_values(mesh, self.pressure.clone()).map_err(|e| format!("{e:?}"))?;
        let face_flux =
            FaceField::from_values(mesh, self.face_flux.clone()).map_err(|e| format!("{e:?}"))?;
        Ok(State {
            flow: SimpleState::new(mesh, velocity, pressure, face_flux)
                .map_err(|e| e.to_string())?,
            phase_fraction: self.phase_fraction.clone(),
            time: self.time,
            step: self.step,
        })
    }
}
pub(super) fn physics_signature(
    mesh: &UnstructuredMesh,
    settings: &Settings,
    simple: SimpleOptions<'_>,
) -> Result<String, String> {
    use sha2::{Digest, Sha256};
    if [
        settings.first_density,
        settings.second_density,
        settings.first_viscosity,
        settings.second_viscosity,
    ]
    .iter()
    .any(|v| !v.is_finite() || *v <= 0.0)
        || [settings.gravity.x, settings.gravity.y, settings.gravity.z]
            .iter()
            .any(|v| !v.is_finite())
        || !settings.phase.max_courant.is_finite()
        || settings.phase.max_courant <= 0.0
        || settings.phase.max_courant > 1.0
        || settings.phase.max_subcycles == 0
        || settings
            .phase
            .incoming
            .values()
            .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
    {
        return Err("Invalid coupled checkpoint physics".into());
    }
    let boundary = |b: &crate::ResolvedScalarBoundaryConditions| -> Result<Vec<(u8, f64, Option<usize>)>, String> {
        b.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
        (0..mesh.face_count()).map(|i| {
            let peer=b.cyclic_link(i).map(|l| l.peer_face);
            let (kind, value) = match b.condition(i) {
                Some(crate::ScalarBoundaryCondition::FixedValue(v)) if v.is_finite() => (1, v),
                Some(crate::ScalarBoundaryCondition::FixedValue(_)) => return Err("Non-finite coupled boundary".into()),
                Some(crate::ScalarBoundaryCondition::ZeroGradient) => (2,0.0),
                None => (0,0.0),
            };
            Ok((kind,value,peer))
        }).collect()
    };
    let velocity = [
        crate::MomentumComponent::X,
        crate::MomentumComponent::Y,
        crate::MomentumComponent::Z,
    ]
    .map(|c| boundary(simple.velocity_boundary.component(c)))
    .into_iter()
    .collect::<Result<Vec<_>, _>>()?;
    let pressure = simple.pressure_boundary.map(boundary).transpose()?;
    let correction = simple
        .pressure_correction_boundary
        .map(boundary)
        .transpose()?;
    let mut value = serde_json::json!({"densities":[settings.first_density,settings.second_density],"viscosities":[settings.first_viscosity,settings.second_viscosity],"gravity":settings.gravity,"phase":settings.phase,"velocity_boundary":velocity,"pressure_boundary":pressure,"correction_boundary":correction,"reference_cell":simple.reference_cell});
    if !settings.surface_tension.is_finite()
        || settings.surface_tension < 0.0
        || settings
            .contact_angle_degrees
            .is_some_and(|a| !a.is_finite() || !(0.0..=180.0).contains(&a))
    {
        return Err("Invalid surface tension or contact angle".into());
    }
    if settings.surface_tension != 0.0 || settings.contact_angle_degrees.is_some() {
        value["surface_tension"] = serde_json::json!(settings.surface_tension);
        value["contact_angle_degrees"] = serde_json::json!(settings.contact_angle_degrees);
    }
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&value).map_err(|e| e.to_string())?)
    ))
}

/// One transient laminar step. Pressure is physical Pa and viscosities are Pa s.
/// The input state is committed only when phase, velocity and flux changes satisfy the tolerances.
pub fn advance(
    mesh: &UnstructuredMesh,
    state: &mut State,
    settings: &Settings,
    simple: SimpleOptions<'_>,
) -> Result<Report, String> {
    if !settings.time_step.is_finite()
        || settings.time_step <= 0.0
        || !settings.min_time_step.is_finite()
        || settings.min_time_step <= 0.0
        || settings.min_time_step > settings.time_step
        || settings.max_step_retries > 32
        || settings
            .max_courant
            .is_some_and(|v| !v.is_finite() || v <= 0.0 || v > 1.0)
    {
        return Err("Invalid coupled time-step or Courant controls".into());
    }
    if !settings.surface_tension.is_finite()
        || settings.surface_tension < 0.0
        || settings
            .contact_angle_degrees
            .is_some_and(|a| !a.is_finite() || !(0.0..=180.0).contains(&a))
    {
        return Err("Invalid surface tension or contact angle".into());
    }
    let initial_rate = courant_rate(mesh, &state.flow.face_flux)?;
    let mut dt = settings.time_step;
    if settings.surface_tension > 0.0 {
        let h = mesh
            .cells()
            .iter()
            .map(|cell| {
                let largest_face = cell
                    .faces
                    .iter()
                    .map(|&i| mesh.faces()[i].area)
                    .fold(0.0_f64, f64::max);
                cell.volume / largest_face
            })
            .fold(f64::INFINITY, f64::min);
        let limit = 0.5
            * ((settings.first_density + settings.second_density) * h.powi(3)
                / (2.0 * std::f64::consts::PI * settings.surface_tension))
                .sqrt();
        if !limit.is_finite() || limit <= 0.0 {
            return Err("Invalid capillary time-step limit".into());
        }
        dt = dt.min(limit);
    }
    if let Some(limit) = settings.max_courant {
        if initial_rate > 0.0 {
            dt = dt.min(limit / initial_rate);
        }
    }
    for rejected in 0..=settings.max_step_retries {
        if !dt.is_finite() || dt < settings.min_time_step {
            return Err(
                "Coupled Courant or capillary limit requires a time step below the minimum; state retained"
                    .into(),
            );
        }
        let mut attempt_settings = settings.clone();
        attempt_settings.time_step = dt;
        let mut candidate = state.clone();
        let mut report = advance_attempt(mesh, &mut candidate, &attempt_settings, simple)?;
        let courant = dt * courant_rate(mesh, &candidate.flow.face_flux)?;
        if settings
            .max_courant
            .is_none_or(|limit| courant <= limit * (1.0 + 1e-10))
        {
            report.time_step = dt;
            report.max_courant = courant;
            report.rejected_attempts = rejected;
            *state = candidate;
            return Ok(report);
        }
        let limit = settings.max_courant.unwrap();
        dt *= (0.9 * limit / courant).min(0.5);
    }
    Err("Coupled Courant limit exceeded the retry budget; state retained".into())
}

pub(super) fn courant_rate(mesh: &UnstructuredMesh, flux: &FaceField<f64>) -> Result<f64, String> {
    flux.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
    let mut outgoing = vec![0.0; mesh.cell_count()];
    for (i, face) in mesh.faces().iter().enumerate() {
        let q = flux[i];
        if !q.is_finite() {
            return Err("Non-finite coupled face flux".into());
        }
        outgoing[face.owner] += q.max(0.0);
        if let Some(neighbour) = face.neighbour {
            outgoing[neighbour] += (-q).max(0.0);
        }
    }
    let rate = outgoing
        .iter()
        .zip(mesh.cells())
        .map(|(q, cell)| q / cell.volume)
        .fold(0.0_f64, f64::max);
    if !rate.is_finite() {
        return Err("Invalid coupled Courant rate".into());
    }
    Ok(rate)
}

fn advance_attempt(
    mesh: &UnstructuredMesh,
    state: &mut State,
    settings: &Settings,
    simple: SimpleOptions<'_>,
) -> Result<Report, String> {
    if [
        settings.first_density,
        settings.second_density,
        settings.first_viscosity,
        settings.second_viscosity,
        settings.time_step,
        settings.velocity_tolerance,
        settings.flux_tolerance,
        settings.phase_tolerance,
        settings.pressure_tolerance,
        settings.momentum_absolute_tolerance,
        settings.momentum_relative_tolerance,
    ]
    .iter()
    .any(|v| !v.is_finite() || *v <= 0.0)
        || [
            settings.gravity.x,
            settings.gravity.y,
            settings.gravity.z,
            state.time,
        ]
        .iter()
        .any(|v| !v.is_finite())
        || state.time < 0.0
        || settings.max_iterations == 0
        || state.phase_fraction.len() != mesh.cell_count()
        || state
            .phase_fraction
            .iter()
            .any(|v| !v.is_finite() || !(0.0..=1.0).contains(v))
    {
        return Err("Invalid coupled two-phase state or settings".into());
    }
    let cyclic = crate::periodic::resolve(mesh, &settings.phase.cyclic_pairs)?;
    for i in 0..mesh.face_count() {
        let expected = cyclic.link(i).map(|link| link.peer_face);
        for component in [
            crate::MomentumComponent::X,
            crate::MomentumComponent::Y,
            crate::MomentumComponent::Z,
        ] {
            let actual = simple
                .velocity_boundary
                .component(component)
                .cyclic_link(i)
                .map(|link| link.peer_face);
            if actual != expected {
                return Err("Coupled phase and velocity cyclic boundaries must match".into());
            }
        }
        let pressure = simple
            .pressure_boundary
            .and_then(|b| b.cyclic_link(i))
            .map(|link| link.peer_face);
        let correction = simple
            .pressure_correction_boundary
            .and_then(|b| b.cyclic_link(i))
            .map(|link| link.peer_face);
        if pressure != expected || correction != expected {
            return Err("Coupled phase and pressure cyclic boundaries must match".into());
        }
        if let Some(link) = cyclic.link(i) {
            let gravity_head = settings.gravity.dot(link.translation);
            if gravity_head.abs()
                > 1e-12 * (settings.gravity.norm() * link.translation.norm()).max(1.0)
            {
                return Err("Physical-pressure periodic boundaries cannot cross a gravitational head; use a compatible pressure treatment".into());
            }
        }
    }
    let next_time = state.time + settings.time_step;
    let next_step = state.step.checked_add(1).ok_or("Coupled step overflow")?;
    if !next_time.is_finite() || next_time <= state.time {
        return Err("Coupled time step cannot advance time".into());
    }
    let measures = Coordinates::Cartesian.measures(mesh)?;
    let min_volume = measures
        .volumes
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);
    let pressure_limit = (5e-14 * min_volume / settings.time_step).max(f64::MIN_POSITIVE);
    let mut pressure_solver = simple.pressure_solver;
    pressure_solver.absolute_tolerance = if pressure_solver.absolute_tolerance > 0.0 {
        pressure_solver.absolute_tolerance.min(pressure_limit)
    } else {
        pressure_limit
    };
    pressure_solver.relative_tolerance = 0.0;
    let mut equations = simple.equations.cloned();
    if let Some(equations) = &mut equations {
        equations.pressure.absolute_tolerance = if equations.pressure.absolute_tolerance > 0.0 {
            equations.pressure.absolute_tolerance.min(pressure_limit)
        } else {
            pressure_limit
        };
        equations.pressure.relative_tolerance = 0.0;
    }
    let old_alpha = state.phase_fraction.clone();
    let old_velocity = state.flow.velocity.clone();
    let old_flux = state.flow.face_flux.clone();
    let old_density = CellField::from_values(
        mesh,
        old_alpha
            .iter()
            .map(|a| a * settings.first_density + (1.0 - a) * settings.second_density)
            .collect(),
    )
    .map_err(|e| format!("{e:?}"))?;
    let mut flow = state.flow.clone();
    let mut momentum_reference = 0.0_f64;
    for iteration in 1..=settings.max_iterations {
        crate::compute::checkpoint().map_err(|e| e.to_string())?;
        let previous_velocity = flow.velocity.clone();
        let previous_pressure = flow.pressure.clone();
        let previous_flux = flow.face_flux.clone();
        let mut alpha = old_alpha.clone();
        let phase_report = phase::advance(
            mesh,
            &measures,
            previous_flux.values(),
            &mut alpha,
            settings.time_step,
            &settings.phase,
        )?;
        let mass_flux = FaceField::from_values(
            mesh,
            phase::mixture_mass_flux(
                &phase_report.phase_flux,
                previous_flux.values(),
                settings.first_density,
                settings.second_density,
            )?,
        )
        .map_err(|e| format!("{e:?}"))?;
        let density = CellField::from_values(
            mesh,
            alpha
                .iter()
                .map(|a| a * settings.first_density + (1.0 - a) * settings.second_density)
                .collect(),
        )
        .map_err(|e| format!("{e:?}"))?;
        let viscosity = CellField::from_values(
            mesh,
            alpha
                .iter()
                .map(|a| a * settings.first_viscosity + (1.0 - a) * settings.second_viscosity)
                .collect(),
        )
        .map_err(|e| format!("{e:?}"))?;
        let surface_force = if settings.surface_tension > 0.0 {
            Some(super::capillary::surface_force(
                mesh,
                &alpha,
                settings.surface_tension,
                settings.contact_angle_degrees,
                simple.pressure_boundary,
            )?)
        } else {
            None
        };
        let iteration_options = SimpleOptions {
            time_step: Some(settings.time_step),
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
                gravity: settings.gravity,
                surface_force: surface_force.as_ref(),
            }),
            ..simple
        };
        let report = crate::simple::solve_simple(mesh, &mut flow, iteration_options)
            .map_err(|e| e.to_string())?;
        let momentum = report.final_momentum_initial_residual;
        momentum_reference = momentum_reference.max(momentum);
        let momentum_relative = if momentum_reference > 0.0 {
            momentum / momentum_reference
        } else {
            0.0
        };
        let pressure_change = flow
            .pressure
            .values()
            .iter()
            .zip(previous_pressure.values())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        let velocity_change = flow
            .velocity
            .values()
            .iter()
            .zip(previous_velocity.values())
            .map(|(a, b)| (*a - *b).norm())
            .fold(0.0_f64, f64::max);
        let flux_change = flow
            .face_flux
            .values()
            .iter()
            .zip(previous_flux.values())
            .map(|(a, b)| (a - b).abs())
            .fold(0.0_f64, f64::max);
        if velocity_change <= settings.velocity_tolerance
            && flux_change <= settings.flux_tolerance
            && pressure_change <= settings.pressure_tolerance
            && (momentum <= settings.momentum_absolute_tolerance
                || momentum_relative <= settings.momentum_relative_tolerance)
        {
            let mut accepted_alpha = old_alpha.clone();
            let accepted = phase::advance(
                mesh,
                &measures,
                flow.face_flux.values(),
                &mut accepted_alpha,
                settings.time_step,
                &settings.phase,
            )?;
            let phase_change = accepted_alpha
                .iter()
                .zip(&alpha)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0_f64, f64::max);
            if phase_change <= settings.phase_tolerance
                && report.final_continuity_rms <= simple.continuity_absolute_tolerance
            {
                state.flow = flow;
                state.phase_fraction = accepted_alpha;
                state.time = next_time;
                state.step = next_step;
                return Ok(Report {
                    iterations: iteration,
                    time_step: settings.time_step,
                    max_courant: 0.0,
                    rejected_attempts: 0,
                    velocity_change,
                    flux_change,
                    phase_change,
                    pressure_change,
                    momentum_initial_residual: momentum,
                    momentum_relative_residual: momentum_relative,
                    continuity_rms: report.final_continuity_rms,
                    phase_balance: accepted.conservation_error,
                });
            }
        }
    }
    Err(format!(
        "Coupled step did not converge after {} iterations; input state retained",
        settings.max_iterations
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BoundaryPatch, BoundaryType, CellDefinition, LeastSquaresGradientStencil, MeshDimension,
        Point, ResolvedScalarBoundaryConditions, ResolvedVelocityBoundaryConditions,
        ScalarBoundaryCondition, VelocityBoundaryCondition,
    };

    #[test]
    fn coupled_step_transports_density_and_preserves_uniform_flow() {
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
        let velocity = Vec3::new(1.0, 0.0, 0.0);
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[(
                "boundary",
                VelocityBoundaryCondition::FixedVelocity(velocity),
            )],
        )
        .unwrap();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("boundary", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let mut simple =
            SimpleOptions::steady(Diffusivity::Constant(0.1), &velocity_boundary, &stencil);
        simple.pressure_boundary = Some(&pressure_boundary);
        let flux = FaceField::from_faces(&mesh, |_, f| f.area_vector.x);
        let flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, velocity),
            CellField::filled(&mesh, 0.0),
            flux,
        )
        .unwrap();
        let mut state = State {
            flow,
            phase_fraction: vec![0.0, 0.0],
            time: 0.0,
            step: 0,
        };
        let mut phase = PhaseOptions::default();
        phase.incoming.insert("boundary".into(), 1.0);
        let mut settings = Settings {
            first_density: 1000.,
            second_density: 1.,
            first_viscosity: 0.1,
            second_viscosity: 0.1,
            gravity: Vec3::ZERO,
            time_step: 0.2,
            max_courant: None,
            min_time_step: 1e-12,
            max_step_retries: 8,
            max_iterations: 10,
            velocity_tolerance: 1e-8,
            flux_tolerance: 1e-8,
            phase_tolerance: 1e-8,
            pressure_tolerance: 1e-8,
            momentum_absolute_tolerance: 1e-8,
            momentum_relative_tolerance: 1e-6,
            surface_tension: 0.0,
            contact_angle_degrees: None,
            phase,
        };
        let original = state.clone();
        let report = advance(&mesh, &mut state, &settings, simple).unwrap();
        assert!(state
            .flow
            .velocity
            .values()
            .iter()
            .all(|u| (*u - velocity).norm() < 1e-8));
        assert!(
            (state.phase_fraction[0] - 0.2).abs() < 1e-10 && state.phase_fraction[1].abs() < 1e-10
        );
        assert!(report.continuity_rms < 1e-10 && report.phase_balance.abs() < 1e-12);
        assert_eq!(state.step, 1);
        assert_eq!(state.time, 0.2);
        let mut limited = original.clone();
        let mut limited_settings = settings.clone();
        limited_settings.max_courant = Some(0.1);
        let limited_report = advance(&mesh, &mut limited, &limited_settings, simple).unwrap();
        assert!((limited_report.time_step - 0.1).abs() < 1e-12);
        assert!(limited_report.max_courant <= 0.1 * (1.0 + 1e-10));
        assert!((limited.phase_fraction[0] - 0.1).abs() < 1e-10);
        let mut rejected = original.clone();
        limited_settings.min_time_step = 0.15;
        assert!(advance(&mesh, &mut rejected, &limited_settings, simple).is_err());
        assert_eq!(rejected.step, original.step);
        assert_eq!(rejected.phase_fraction, original.phase_fraction);
        let saved = Checkpoint::capture(&mesh, &state, &settings, simple).unwrap();
        let folder = tempfile::tempdir().unwrap();
        let path = folder.path().join("coupled.json");
        saved.save(&path).unwrap();
        let loaded = Checkpoint::load(&path).unwrap();
        let mut resumed = loaded.restore(&mesh, &settings, simple).unwrap();
        let mut uninterrupted = state.clone();
        advance(&mesh, &mut resumed, &settings, simple).unwrap();
        advance(&mesh, &mut uninterrupted, &settings, simple).unwrap();
        assert_eq!(resumed.phase_fraction, uninterrupted.phase_fraction);
        assert_eq!(
            resumed.flow.velocity.values(),
            uninterrupted.flow.velocity.values()
        );
        assert_eq!(
            resumed.flow.face_flux.values(),
            uninterrupted.flow.face_flux.values()
        );
        let mut changed_settings = settings.clone();
        changed_settings.first_density = 999.0;
        assert!(loaded.restore(&mesh, &changed_settings, simple).is_err());
        let before = state.clone();
        settings.first_density = -1.0;
        assert!(advance(&mesh, &mut state, &settings, simple).is_err());
        assert_eq!(state.step, before.step);
        assert_eq!(state.phase_fraction, before.phase_fraction);
        assert_eq!(state.flow.velocity.values(), before.flow.velocity.values());
    }
    #[test]
    fn coupled_step_preserves_uniform_density_hydrostatic_rest() {
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
        let patches = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_none())
            .map(|(i, _)| BoundaryPatch {
                name: format!("face-{i}"),
                face_indices: vec![i],
                boundary_type: BoundaryType::Wall,
            })
            .collect();
        let mesh = mesh.with_boundary_patches(patches).unwrap();
        let pressure = |p: Vec3| 50_000.0 - 9_810.0 * p.y;
        let vbc: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|p| {
                (
                    p.name.as_str(),
                    VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO),
                )
            })
            .collect();
        let pbc: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|p| {
                (
                    p.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(pressure(
                        mesh.faces()[p.face_indices[0]].center,
                    )),
                )
            })
            .collect();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(&mesh, &vbc).unwrap();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &pbc).unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let mut simple =
            SimpleOptions::steady(Diffusivity::Constant(0.1), &velocity_boundary, &stencil);
        simple.pressure_boundary = Some(&pressure_boundary);
        let flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::from_cells(&mesh, |_, c| pressure(c.center)),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut state = State {
            flow,
            phase_fraction: vec![1.0, 1.0],
            time: 0.0,
            step: 0,
        };
        let settings = Settings {
            first_density: 1000.,
            second_density: 1.,
            first_viscosity: 0.1,
            second_viscosity: 0.1,
            gravity: Vec3::new(0., -9.81, 0.),
            time_step: 0.1,
            max_courant: None,
            min_time_step: 1e-12,
            max_step_retries: 8,
            max_iterations: 20,
            velocity_tolerance: 1e-8,
            flux_tolerance: 1e-8,
            phase_tolerance: 1e-8,
            pressure_tolerance: 1e-6,
            momentum_absolute_tolerance: 1e-6,
            momentum_relative_tolerance: 1e-6,
            phase: PhaseOptions::default(),
            surface_tension: 0.0,
            contact_angle_degrees: None,
        };
        let report = advance(&mesh, &mut state, &settings, simple).unwrap();
        assert!(state.flow.velocity.values().iter().all(|u| u.norm() < 1e-8));
        assert!(report.continuity_rms < 1e-10);
        assert_eq!(state.phase_fraction, vec![1.0, 1.0]);
    }
    #[test]
    fn stratified_hydrostatic_state_has_no_spurious_velocity() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(1., 1., 0.),
                Point::new(0., 2., 0.),
                Point::new(1., 2., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 3, 2]),
                CellDefinition::polygon(vec![2, 3, 5, 4]),
            ],
        )
        .unwrap();
        let patches = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_none())
            .map(|(i, _)| BoundaryPatch {
                name: format!("face-{i}"),
                face_indices: vec![i],
                boundary_type: BoundaryType::Wall,
            })
            .collect();
        let mesh = mesh.with_boundary_patches(patches).unwrap();
        let pressure = |p: Vec3| 50_000.0 - 9.81 * (1000.0 * p.y.min(1.0) + (p.y - 1.0).max(0.0));
        let vbc: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|p| {
                (
                    p.name.as_str(),
                    VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO),
                )
            })
            .collect();
        let pbc: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|p| {
                (
                    p.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(pressure(
                        mesh.faces()[p.face_indices[0]].center,
                    )),
                )
            })
            .collect();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(&mesh, &vbc).unwrap();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &pbc).unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let mut simple =
            SimpleOptions::steady(Diffusivity::Constant(0.1), &velocity_boundary, &stencil);
        simple.pressure_boundary = Some(&pressure_boundary);
        let flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::from_cells(&mesh, |_, c| pressure(c.center)),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut state = State {
            flow,
            phase_fraction: vec![1.0, 0.0],
            time: 0.0,
            step: 0,
        };
        let settings = Settings {
            first_density: 1000.,
            second_density: 1.,
            first_viscosity: 0.1,
            second_viscosity: 0.1,
            gravity: Vec3::new(0., -9.81, 0.),
            time_step: 0.1,
            max_courant: None,
            min_time_step: 1e-12,
            max_step_retries: 8,
            max_iterations: 100,
            velocity_tolerance: 1e-8,
            flux_tolerance: 1e-8,
            phase_tolerance: 1e-8,
            pressure_tolerance: 1e-6,
            momentum_absolute_tolerance: 1e-6,
            momentum_relative_tolerance: 1e-6,
            phase: PhaseOptions::default(),
            surface_tension: 0.0,
            contact_angle_degrees: None,
        };
        let report = advance(&mesh, &mut state, &settings, simple).unwrap();
        assert!(
            state.flow.velocity.values().iter().all(|u| u.norm() < 1e-7),
            "{:?}",
            state.flow.velocity.values()
        );
        assert!(report.continuity_rms < 1e-10);
        assert!(
            (state.phase_fraction[0] - 1.0).abs() < 1e-12 && state.phase_fraction[1].abs() < 1e-12
        );
        let zero_pbc: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|p| (p.name.as_str(), ScalarBoundaryCondition::ZeroGradient))
            .collect();
        let zero_pressure_boundary =
            ResolvedScalarBoundaryConditions::strict(&mesh, &zero_pbc).unwrap();
        let zero_stencil =
            LeastSquaresGradientStencil::new(&mesh, &zero_pressure_boundary).unwrap();
        let mut zero_simple = SimpleOptions::steady(
            Diffusivity::Constant(0.1),
            &velocity_boundary,
            &zero_stencil,
        );
        zero_simple.pressure_boundary = Some(&zero_pressure_boundary);
        let flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::from_cells(&mesh, |_, c| pressure(c.center)),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut zero_state = State {
            flow,
            phase_fraction: vec![1.0, 0.0],
            time: 0.0,
            step: 0,
        };
        advance(&mesh, &mut zero_state, &settings, zero_simple).unwrap();
        assert!(zero_state
            .flow
            .velocity
            .values()
            .iter()
            .all(|u| u.norm() < 1e-7));
        let cold_flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut cold = State {
            flow: cold_flow,
            phase_fraction: vec![1.0, 0.0],
            time: 0.0,
            step: 0,
        };
        advance(&mesh, &mut cold, &settings, zero_simple).unwrap();
        assert!(
            cold.flow.velocity.values().iter().all(|u| u.norm() < 1e-7),
            "{:?}",
            cold.flow.velocity.values()
        );
    }
    #[test]
    fn stratified_hydrostatic_rest_is_preserved_on_multiple_columns() {
        let point = |x: usize, y: usize| x + 3 * y;
        let points = (0..=4)
            .flat_map(|y| (0..=2).map(move |x| Point::new(x as f64, 0.5 * y as f64, 0.)))
            .collect();
        let cells = (0..4)
            .flat_map(|y| {
                (0..2).map(move |x| {
                    CellDefinition::polygon(vec![
                        point(x, y),
                        point(x + 1, y),
                        point(x + 1, y + 1),
                        point(x, y + 1),
                    ])
                })
            })
            .collect();
        let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
        let faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![BoundaryPatch {
                name: "wall".into(),
                face_indices: faces,
                boundary_type: BoundaryType::Wall,
            }])
            .unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::ZeroGradient)],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let mut simple =
            SimpleOptions::steady(Diffusivity::Constant(0.1), &velocity_boundary, &stencil);
        simple.pressure_boundary = Some(&pressure_boundary);
        let flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let alpha = mesh
            .cells()
            .iter()
            .map(|c| if c.center.y < 1.0 { 1.0 } else { 0.0 })
            .collect();
        let mut state = State {
            flow,
            phase_fraction: alpha,
            time: 0.0,
            step: 0,
        };
        let settings = Settings {
            first_density: 1000.,
            second_density: 1.,
            first_viscosity: 0.1,
            second_viscosity: 0.1,
            gravity: Vec3::new(0., -9.81, 0.),
            time_step: 0.01,
            max_courant: None,
            min_time_step: 1e-12,
            max_step_retries: 8,
            max_iterations: 500,
            velocity_tolerance: 1e-8,
            flux_tolerance: 1e-8,
            phase_tolerance: 1e-8,
            pressure_tolerance: 1e-5,
            momentum_absolute_tolerance: 1e-5,
            momentum_relative_tolerance: 1e-6,
            phase: PhaseOptions::default(),
            surface_tension: 0.0,
            contact_angle_degrees: None,
        };
        let report = advance(&mesh, &mut state, &settings, simple).unwrap();
        assert!(state.flow.velocity.values().iter().all(|u| u.norm() < 1e-6));
        assert!(report.continuity_rms < 1e-10);
    }
    #[test]
    fn gravity_drives_flow_for_non_stratified_density() {
        let point = |x: usize, y: usize| x + 3 * y;
        let points = (0..=2)
            .flat_map(|y| (0..=2).map(move |x| Point::new(x as f64, y as f64, 0.)))
            .collect();
        let cells = (0..2)
            .flat_map(|y| {
                (0..2).map(move |x| {
                    CellDefinition::polygon(vec![
                        point(x, y),
                        point(x + 1, y),
                        point(x + 1, y + 1),
                        point(x, y + 1),
                    ])
                })
            })
            .collect();
        let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
        let faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![BoundaryPatch {
                name: "wall".into(),
                face_indices: faces,
                boundary_type: BoundaryType::Wall,
            }])
            .unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::ZeroGradient)],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let mut simple =
            SimpleOptions::steady(Diffusivity::Constant(0.1), &velocity_boundary, &stencil);
        simple.pressure_boundary = Some(&pressure_boundary);
        let flow = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut state = State {
            flow,
            phase_fraction: vec![1.0, 0.0, 0.0, 0.0],
            time: 0.0,
            step: 0,
        };
        let settings = Settings {
            first_density: 1000.,
            second_density: 1.,
            first_viscosity: 0.001,
            second_viscosity: 0.000018,
            gravity: Vec3::new(0., -9.81, 0.),
            time_step: 0.001,
            max_courant: Some(0.5),
            min_time_step: 1e-8,
            max_step_retries: 8,
            max_iterations: 500,
            velocity_tolerance: 1e-8,
            flux_tolerance: 1e-8,
            phase_tolerance: 1e-8,
            pressure_tolerance: 1e-5,
            momentum_absolute_tolerance: 1e-5,
            momentum_relative_tolerance: 1e-6,
            phase: PhaseOptions::default(),
            surface_tension: 0.0,
            contact_angle_degrees: None,
        };
        let report = advance(&mesh, &mut state, &settings, simple).unwrap();
        assert!(state.flow.velocity.values().iter().any(|u| u.norm() > 1e-8));
        assert!(report.continuity_rms < 1e-10 && report.phase_balance.abs() < 1e-10);
        assert!((state.phase_fraction.iter().sum::<f64>() - 1.0).abs() < 1e-10);

        let mut capillary = State {
            flow: SimpleState::new(
                &mesh,
                CellField::filled(&mesh, Vec3::ZERO),
                CellField::filled(&mesh, 0.0),
                FaceField::filled(&mesh, 0.0),
            )
            .unwrap(),
            phase_fraction: vec![1.0, 0.0, 0.0, 0.0],
            time: 0.0,
            step: 0,
        };
        let mut capillary_settings = settings.clone();
        capillary_settings.gravity = Vec3::ZERO;
        capillary_settings.surface_tension = 0.072;
        let report = advance(&mesh, &mut capillary, &capillary_settings, simple).unwrap();
        assert!(report.continuity_rms < 1e-10 && report.phase_balance.abs() < 1e-10);
        assert!(capillary
            .flow
            .pressure
            .values()
            .iter()
            .any(|p| p.abs() > 1e-8));
        let checkpoint =
            Checkpoint::capture(&mesh, &capillary, &capillary_settings, simple).unwrap();
        let mut altered = capillary_settings.clone();
        altered.surface_tension *= 2.0;
        assert!(checkpoint.restore(&mesh, &altered, simple).is_err());
    }
}
