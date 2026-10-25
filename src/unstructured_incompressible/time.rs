//! Fixed-mesh Euler/BDF2; only converged, Courant-accepted steps advance time.
use super::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AdaptiveTimeStep {
    pub target_courant: f64,
    /// Optional diffusion-number accuracy bound for implicit transport.
    pub max_diffusion: Option<f64>,
    pub min_dt: f64,
    pub max_dt: f64,
    pub max_growth: f64,
}
impl Default for AdaptiveTimeStep {
    fn default() -> Self {
        Self {
            target_courant: 0.5,
            max_diffusion: None,
            min_dt: 1e-8,
            max_dt: 0.01,
            max_growth: 1.2,
        }
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "kebab-case")]
pub enum TimeIntegration {
    #[default]
    Steady,
    Euler {
        dt: f64,
        steps: usize,
    },
    Backward {
        dt: f64,
        steps: usize,
    },
    AdaptiveEuler {
        dt: f64,
        steps: usize,
        controls: AdaptiveTimeStep,
    },
    AdaptiveBackward {
        dt: f64,
        steps: usize,
        controls: AdaptiveTimeStep,
    },
}
impl TimeIntegration {
    pub fn is_transient(self) -> bool {
        !matches!(self, Self::Steady)
    }
    pub fn parameters(self) -> Option<(f64, usize)> {
        match self {
            Self::Steady => None,
            Self::Euler { dt, steps }
            | Self::Backward { dt, steps }
            | Self::AdaptiveEuler { dt, steps, .. }
            | Self::AdaptiveBackward { dt, steps, .. } => Some((dt, steps)),
        }
    }
    pub fn adaptive(self) -> Option<AdaptiveTimeStep> {
        match self {
            Self::AdaptiveEuler { controls, .. } | Self::AdaptiveBackward { controls, .. } => {
                Some(controls)
            }
            _ => None,
        }
    }
    pub fn validate(self) -> Result<(), String> {
        let Some((dt, steps)) = self.parameters() else {
            return Ok(());
        };
        if !dt.is_finite() || dt <= 0.0 || !(1..=1_000_000).contains(&steps) {
            return Err(
                "Transient integration requires positive finite dt and 1..1000000 time steps"
                    .into(),
            );
        }
        if let Some(c) = self.adaptive() {
            if c.max_diffusion.is_some_and(|v| !v.is_finite() || v <= 0.0) {
                return Err("Maximum diffusion number must be finite positive or unset".into());
            }
            if [c.target_courant, c.min_dt, c.max_dt, c.max_growth]
                .iter()
                .any(|v| !v.is_finite())
                || c.target_courant <= 0.0
                || c.min_dt <= 0.0
                || c.max_dt < c.min_dt
                || !(1.0..=2.0).contains(&c.max_growth)
                || dt < c.min_dt
                || dt > c.max_dt
            {
                return Err("Adaptive time stepping requires positive Courant, 0 < min dt <= initial dt <= max dt, and growth in [1,2]".into());
            }
        }
        Ok(())
    }
}
fn courant_rate(mesh: &UnstructuredMesh, flux: &FaceField<f64>) -> Result<f64, SimpleError> {
    flux.ensure_mesh(mesh)?;
    let mut sum = vec![0.0; mesh.cell_count()];
    for (f, q) in mesh.faces().iter().zip(flux.values()) {
        sum[f.owner] += q.abs();
        if let Some(n) = f.neighbour {
            sum[n] += q.abs();
        }
    }
    let rate = sum
        .iter()
        .zip(mesh.cells())
        .map(|(q, c)| 0.5 * q / c.volume)
        .fold(0.0, f64::max);
    if !rate.is_finite() {
        return Err(SimpleError::PhysicalBoundary(
            "Non-finite flux-based Courant rate".into(),
        ));
    }
    Ok(rate)
}
fn diffusion_rate(case: &IncompressibleCase, nut: f64) -> f64 {
    let t = &case.solver.thermal;
    let alpha = if t.enabled {
        t.conductivity / (case.material.density * t.specific_heat) + nut / t.turbulent_prandtl
    } else {
        0.0
    };
    let diffusivity = (case.material.kinematic_viscosity + nut).max(alpha);
    let mesh = &case.mesh;
    let mut sum = vec![0.0; mesh.cell_count()];
    for f in mesh.faces() {
        let d = f
            .neighbour
            .map(|n| mesh.cells()[n].center - mesh.cells()[f.owner].center)
            .unwrap_or(f.center - mesh.cells()[f.owner].center);
        let projection = f.area_vector.dot(d).abs();
        if projection > 0.0 {
            let conductance = diffusivity * f.area_vector.norm_squared() / projection;
            sum[f.owner] += conductance;
            if let Some(n) = f.neighbour {
                sum[n] += conductance;
            }
        }
    }
    sum.iter()
        .zip(mesh.cells())
        .map(|(s, c)| 0.5 * s / c.volume)
        .fold(0.0, f64::max)
}
fn initial_rate(case: &IncompressibleCase) -> Result<f64, IncompressibleSolveError> {
    if let IncompressibleInitialConditions::Restart { face_flux, .. } = &case.initial_conditions {
        return Ok(courant_rate(&case.mesh, face_flux)?);
    }
    let velocity = match &case.initial_conditions {
        IncompressibleInitialConditions::Uniform { velocity, .. } => {
            CellField::filled(&case.mesh, *velocity)
        }
        IncompressibleInitialConditions::Fields { velocity, .. } => velocity.clone(),
        _ => unreachable!(),
    };
    let boundary = case.resolve_boundaries()?;
    Ok(courant_rate(
        &case.mesh,
        &initial_face_flux(&case.mesh, &velocity, boundary.boundary_flux())?,
    )?)
}
pub(super) fn solve_time_loop(
    case: &IncompressibleCase,
    control: Option<&SolveControl<'_>>,
) -> Result<IncompressibleSolution, IncompressibleSolveError> {
    case.solver
        .volume_models
        .validate_case(case)
        .map_err(SimpleError::PhysicalBoundary)?;
    if let Some(cb) = control.and_then(|c| c.on_message.as_ref()) {
        if let Some(p) = case.solver.volume_models.resistance {
            let count = case
                .mesh
                .cells()
                .iter()
                .filter(|c| p.region.contains(c.center))
                .count();
            cb(format!(
                "Isotropic momentum resistance: {count} cells; Darcy {} 1/m², Forchheimer {} 1/m; implicit lagged-speed sink.",
                p.darcy, p.forchheimer
            ));
        }
        if let Some(r) = case.solver.volume_models.rotating_frame {
            cb(format!(
                "Global rotating frame: omega={:?} rad/s; origin={:?} m. All velocity fields, boundary velocities and face fluxes are relative. No multi-zone MRF interface or mesh motion.",
                r.angular_velocity, r.origin
            ));
        }
    }
    case.solver
        .pressure_coupling
        .validate(case.solver.time.is_transient())
        .map_err(SimpleError::PhysicalBoundary)?;
    case.solver
        .time
        .validate()
        .map_err(SimpleError::PhysicalBoundary)?;
    if !case.solver.time.is_transient()
        && matches!(
            case.solver.turbulence.model,
            crate::turbulence::TurbulenceModel::Smagorinsky
                | crate::turbulence::TurbulenceModel::Wale
        )
    {
        return Err(SimpleError::PhysicalBoundary(
            "LES requires transient physical-time integration".into(),
        )
        .into());
    }
    case.solver
        .scripting
        .validate(case.solver.time.is_transient())
        .map_err(SimpleError::PhysicalBoundary)?;
    for (name, bindings) in &case.solver.scripting.bindings {
        if case.solver.boundary_profiles.contains_key(name) {
            return Err(SimpleError::PhysicalBoundary(format!(
                "Remove the time table before attaching a script on {name}"
            ))
            .into());
        }
        if case
            .solver
            .cyclic_pairs
            .iter()
            .any(|p| p.first == *name || p.second == *name)
        {
            return Err(SimpleError::PhysicalBoundary(format!(
                "Cyclic patch {name} cannot use physical scripts"
            ))
            .into());
        }
        if !case.solver.thermal.enabled
            && bindings
                .iter()
                .any(|b| b.quantity == crate::scripting::Quantity::Temperature)
        {
            return Err(SimpleError::PhysicalBoundary(
                "Temperature scripts require enabled thermal transport".into(),
            )
            .into());
        }
    }
    if !case.solver.time.is_transient() && !case.solver.boundary_profiles.is_empty() {
        return Err(SimpleError::PhysicalBoundary(
            "Time-table boundaries require transient integration".into(),
        )
        .into());
    }
    let end_time = case.solver.physical_end_time;
    if end_time.is_some_and(|t| !t.is_finite() || t <= 0.0 || !case.solver.time.is_transient()) {
        return Err(SimpleError::PhysicalBoundary(
            "Physical end time requires transient integration and a finite positive time".into(),
        )
        .into());
    }
    let Some((initial_dt, steps)) = case.solver.time.parameters() else {
        let result = solve_incompressible_on_pool(case, control, None, None)?;
        if let Some(cb) = control.and_then(|c| c.on_step.as_ref()) {
            cb(result.report.outer_iterations, &result).map_err(SimpleError::Monitor)?;
        }
        return Ok(result);
    };
    if let Some(cb) = control.and_then(|c| c.on_message.as_ref()) {
        let coupling = case.solver.pressure_coupling;
        cb(format!(
            "Transient coupling {:?}: up to {} outer iterations per time step, {} inner pressure corrector(s), {} extra non-orthogonal pass(es), limiter {}. Physical history is frozen until acceptance.",
            coupling.algorithm,
            case.solver.max_outer_iterations,
            coupling.pressure_correctors,
            coupling.non_orthogonal_correctors,
            coupling.non_orthogonal_limiter
        ));
    }
    let backward = matches!(
        case.solver.time,
        TimeIntegration::Backward { .. } | TimeIntegration::AdaptiveBackward { .. }
    );
    if backward {
        if let Some(cb) = control.and_then(|c| c.on_message.as_ref()) {
            cb("BDF2: complete checkpoints retain both time levels; only a fresh or legacy one-level start uses Euler.".into());
        }
    }
    let mut older = match &case.initial_conditions {
        IncompressibleInitialConditions::Restart { previous, .. } if backward => previous
            .as_deref()
            .map(IncompressibleInitialConditions::without_history),
        _ => None,
    };
    let adaptive = case.solver.time.adaptive();
    let mut step_case = case.clone();
    step_case.solver.time = TimeIntegration::Steady;
    let (mut time, previous_dt) = match &case.initial_conditions {
        IncompressibleInitialConditions::Restart { time, last_dt, .. } => (*time, *last_dt),
        _ => (0.0, None),
    };
    if end_time.is_some_and(|end| time >= end) {
        return Err(SimpleError::PhysicalBoundary(
            "Restart time has already reached the requested physical end time".into(),
        )
        .into());
    }
    profiles::apply(&mut step_case, time)?;
    // Freeze actual initial face flux at t_n before any future boundary evaluation.
    if backward
        && !matches!(
            step_case.initial_conditions,
            IncompressibleInitialConditions::Restart { .. }
        )
    {
        let (u, p) = match &step_case.initial_conditions {
            IncompressibleInitialConditions::Uniform { velocity, pressure } => (
                CellField::filled(&case.mesh, *velocity),
                CellField::filled(&case.mesh, *pressure),
            ),
            IncompressibleInitialConditions::Fields { velocity, pressure } => {
                (velocity.clone(), pressure.clone())
            }
            _ => unreachable!(),
        };
        let b = step_case.resolve_boundaries()?;
        let phi = initial_face_flux(&case.mesh, &u, b.boundary_flux())?;
        step_case.initial_conditions = IncompressibleInitialConditions::Restart {
            previous: None,
            velocity: u,
            pressure: p,
            face_flux: phi,
            time,
            last_dt: None,
            temperature: case
                .solver
                .thermal
                .enabled
                .then(|| CellField::filled(&case.mesh, case.solver.thermal.initial_temperature)),
            turbulence: (case.solver.turbulence.model
                != crate::turbulence::TurbulenceModel::Laminar)
                .then(|| {
                    crate::turbulence::TurbulenceFields::new(&case.mesh, &case.solver.turbulence)
                }),
        };
    }
    let mut dt = if adaptive.is_some() {
        previous_dt.unwrap_or(initial_dt)
    } else {
        initial_dt
    };
    let mut rate = if adaptive.is_some() {
        initial_rate(&step_case)?
    } else {
        0.0
    };
    let mut diffusion_rate = diffusion_rate(case, 0.0);
    let mut last = None;
    let mut initial_continuity = None;
    let mut histories = Vec::new();
    let mut momentum_history = Vec::new();
    let mut velocity_history = Vec::new();
    let mut pressure_history = Vec::new();
    let mut correctors = 0;
    let mut rejected = 0;
    for step in 1..=steps {
        if control.is_some_and(SolveControl::cancelled) {
            return Err(IncompressibleSolveError::Cancelled);
        }
        if let Some(c) = adaptive {
            let growth = if step == 1 && previous_dt.is_none() {
                1.0
            } else {
                c.max_growth
            };
            dt = (dt * growth).clamp(c.min_dt, c.max_dt);
            if rate > 0.0 {
                dt = dt.min(c.target_courant / rate);
            }
            if dt < c.min_dt {
                return Err(SimpleError::PhysicalBoundary(format!(
                    "Courant target requires dt={dt:e}, below min dt={:e}; time was not advanced",
                    c.min_dt
                ))
                .into());
            }
        }
        if let Some(max) = adaptive.and_then(|c| c.max_diffusion) {
            if diffusion_rate > 0.0 {
                let limit = max / diffusion_rate;
                if limit < dt {
                    dt = limit;
                    if let Some(cb) = control.and_then(|c| c.on_message.as_ref()) {
                        cb(format!(
                            "Diffusion-number accuracy bound limits dt to {dt:.6e} s"
                        ));
                    }
                }
                if dt < adaptive.unwrap().min_dt {
                    return Err(SimpleError::PhysicalBoundary(
                        "Diffusion-number bound requires dt below configured minimum".into(),
                    )
                    .into());
                }
            }
        }
        if let Some(end) = end_time {
            dt = dt.min(end - time);
        }
        let mut accepted = None;
        for attempt in 0..=8 {
            let next = time + dt;
            if !next.is_finite() || next <= time {
                return Err(SimpleError::PhysicalBoundary(
                    "Physical time overflow or dt below time resolution".into(),
                )
                .into());
            }
            profiles::apply(&mut step_case, next)?;
            let inner = SolveControl::new(
                || control.is_some_and(SolveControl::cancelled),
                |_, _, _| {},
            );
            // Variable-step BDF2: a0 U[n+1] = a1 U[n] + a2 U[n-1] + dt RHS.
            // A restart has only one historical level and therefore starts with Euler.
            let (effective_dt, history_weights) = if backward && older.is_some() {
                let old_dt = match &step_case.initial_conditions {
                    IncompressibleInitialConditions::Restart {
                        last_dt: Some(v), ..
                    } => *v,
                    _ => dt,
                };
                let ratio = dt / old_dt;
                if !ratio.is_finite() || ratio <= 0.0 || ratio >= 1.0 + 2.0_f64.sqrt() {
                    return Err(SimpleError::PhysicalBoundary(
                        "BDF2 step ratio must be positive and below 1+sqrt(2)".into(),
                    )
                    .into());
                }
                let a0 = (1.0 + 2.0 * ratio) / (1.0 + ratio);
                (
                    dt / a0,
                    [(1.0 + ratio) / a0, -ratio * ratio / ((1.0 + ratio) * a0)],
                )
            } else {
                (dt, [1.0, 0.0])
            };
            let mut result = solve_incompressible_on_pool(
                &step_case,
                Some(&inner),
                Some(effective_dt),
                if backward {
                    older.as_ref().map(|old| (old, history_weights))
                } else {
                    None
                },
            )?;
            initial_continuity.get_or_insert(result.report.initial_continuity_rms);
            correctors += result.report.outer_iterations;
            let next_rate = courant_rate(&case.mesh, &result.face_flux)?;
            let co = dt * next_rate;
            if !co.is_finite() {
                return Err(SimpleError::PhysicalBoundary(
                    "Courant overflow; physical time was not advanced".into(),
                )
                .into());
            }
            let next_diffusion_rate = self::diffusion_rate(
                case,
                result
                    .turbulence
                    .as_ref()
                    .map(|t| t.nut.values().iter().copied().fold(0.0, f64::max))
                    .unwrap_or(0.0),
            );
            let diffusion = dt * next_diffusion_rate;
            if !diffusion.is_finite() {
                return Err(SimpleError::PhysicalBoundary(
                    "Diffusion number overflow; physical time was not advanced".into(),
                )
                .into());
            }
            let reject = !result.report.converged()
                || adaptive.is_some_and(|c| {
                    co > c.target_courant * (1.0 + 1e-6)
                        || c.max_diffusion
                            .is_some_and(|max| diffusion > max * (1.0 + 1e-6))
                });
            if reject {
                if let Some(c) = adaptive {
                    rejected += 1;
                    let factor = if result.report.converged() {
                        (0.9 * c.target_courant / co)
                            .min(
                                c.max_diffusion
                                    .map(|max| 0.9 * max / diffusion)
                                    .unwrap_or(0.9),
                            )
                            .min(0.9)
                    } else {
                        0.5
                    };
                    let smaller = (dt * factor).max(c.min_dt);
                    if let Some(cb) = control.and_then(|c| c.on_message.as_ref()) {
                        cb(format!(
                            "Rejected time step {step}, attempt {}: dt={dt:e}, Co={co:.4e}, diffusion={diffusion:.4e}, coupled_converged={}; retry dt={smaller:e}; fields and physical time rolled back",
                            attempt + 1,
                            result.report.converged()
                        ));
                    }
                    if attempt == 8 || smaller >= dt {
                        return Err(SimpleError::PhysicalBoundary(format!("Time step {step} cannot satisfy coupled convergence / Courant / diffusion within adaptive limits; last accepted time is {time:e} s")).into());
                    }
                    dt = smaller;
                    continue;
                }
                return Err(SimpleError::PhysicalBoundary(format!("Time step {step} at t={next:e} failed nonlinear convergence after {} correctors; physical time was not advanced",result.report.outer_iterations)).into());
            }
            result.physical_time = Some(next);
            result.last_time_step = Some(dt);
            result.rejected_time_steps = rejected;
            result.budgets = crate::workbench::budgets::evaluate(
                &step_case,
                &result,
                Some((
                    &step_case.initial_conditions,
                    older.as_ref(),
                    effective_dt,
                    history_weights,
                )),
            );
            result.max_diffusion_number = Some(diffusion);
            diffusion_rate = next_diffusion_rate;
            result.completed_time_steps = step;
            if backward {
                result.previous = Some(Box::new(step_case.initial_conditions.without_history()));
            }
            result.max_courant = Some(co);
            result.nonlinear_iterations = correctors;
            time = next;
            rate = next_rate;
            accepted = Some(result);
            break;
        }
        let result = accepted.expect("retry loop accepts a step or returns an error");
        histories.push(result.report.final_continuity_rms);
        momentum_history.push(result.report.simple.final_momentum_relative_residual);
        velocity_history.push(result.report.simple.final_velocity_relative_change);
        pressure_history.push(
            result
                .report
                .simple
                .pressure_report
                .as_ref()
                .map_or(0.0, |r| r.final_residual),
        );
        if let Some(cb) = control.and_then(|c| c.on_step.as_ref()) {
            cb(step, &result).map_err(SimpleError::Monitor)?;
        }
        if let Some(c) = control {
            c.report_iteration(step, steps, result.report.final_continuity_rms);
            if let Some(cb) = &c.on_metrics {
                cb(
                    step,
                    [
                        result.report.final_continuity_rms,
                        result.report.simple.final_momentum_relative_residual,
                        result.report.simple.final_velocity_relative_change,
                        *pressure_history.last().unwrap(),
                    ],
                );
            }
            if let Some(cb) = &c.on_fields {
                cb(step, &result.velocity, &result.pressure).map_err(SimpleError::Monitor)?;
            }
        }
        if backward {
            older = Some(step_case.initial_conditions.without_history());
        }
        step_case.initial_conditions = result.restart();
        last = Some(result);
        if end_time.is_some_and(|end| time >= end) {
            break;
        }
    }
    let mut result = last.expect("validated nonzero time steps");
    result.report.outer_iterations = result.completed_time_steps;
    result.report.simple.outer_iterations = result.completed_time_steps;
    result.report.initial_continuity_rms = initial_continuity.unwrap();
    result.report.simple.initial_continuity_rms = initial_continuity.unwrap();
    result.report.continuity_history = histories.clone();
    result.report.simple.continuity_history = histories;
    result.report.simple.momentum_relative_history = momentum_history;
    result.report.simple.velocity_change_history = velocity_history;
    result.report.simple.pressure_linear_history = pressure_history;
    Ok(result)
}
