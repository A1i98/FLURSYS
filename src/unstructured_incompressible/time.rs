//! Fixed-mesh implicit Euler; only converged, Courant-accepted steps advance time.
use super::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AdaptiveTimeStep {
    pub target_courant: f64,
    pub min_dt: f64,
    pub max_dt: f64,
    pub max_growth: f64,
}
impl Default for AdaptiveTimeStep {
    fn default() -> Self {
        Self {
            target_courant: 0.5,
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
    AdaptiveEuler {
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
            Self::Euler { dt, steps } | Self::AdaptiveEuler { dt, steps, .. } => Some((dt, steps)),
        }
    }
    pub fn adaptive(self) -> Option<AdaptiveTimeStep> {
        match self {
            Self::AdaptiveEuler { controls, .. } => Some(controls),
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
    let Some((initial_dt, steps)) = case.solver.time.parameters() else {
        let result = solve_incompressible_on_pool(case, control, None)?;
        if let Some(cb) = control.and_then(|c| c.on_step.as_ref()) {
            cb(result.report.outer_iterations, &result).map_err(SimpleError::Monitor)?;
        }
        return Ok(result);
    };
    let adaptive = case.solver.time.adaptive();
    let mut step_case = case.clone();
    step_case.solver.time = TimeIntegration::Steady;
    let (mut time, previous_dt) = match &case.initial_conditions {
        IncompressibleInitialConditions::Restart { time, last_dt, .. } => (*time, *last_dt),
        _ => (0.0, None),
    };
    let mut dt = if adaptive.is_some() {
        previous_dt.unwrap_or(initial_dt)
    } else {
        initial_dt
    };
    let mut rate = if adaptive.is_some() {
        initial_rate(case)?
    } else {
        0.0
    };
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
        let mut accepted = None;
        for attempt in 0..=8 {
            let next = time + dt;
            if !next.is_finite() || next <= time {
                return Err(SimpleError::PhysicalBoundary(
                    "Physical time overflow or dt below time resolution".into(),
                )
                .into());
            }
            let inner = SolveControl::new(
                || control.is_some_and(SolveControl::cancelled),
                |_, _, _| {},
            );
            let mut result = solve_incompressible_on_pool(&step_case, Some(&inner), Some(dt))?;
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
            let reject = !result.report.converged()
                || adaptive.is_some_and(|c| co > c.target_courant * (1.0 + 1e-6));
            if reject {
                if let Some(c) = adaptive {
                    rejected += 1;
                    let factor = if result.report.converged() {
                        (0.9 * c.target_courant / co).min(0.9)
                    } else {
                        0.5
                    };
                    let smaller = (dt * factor).max(c.min_dt);
                    if let Some(cb) = control.and_then(|c| c.on_message.as_ref()) {
                        cb(format!("Rejected time step {step}, attempt {}: dt={dt:e}, Co={co:.4e}, coupled_converged={}; retry dt={smaller:e}; fields and physical time rolled back",attempt+1,result.report.converged()));
                    }
                    if attempt == 8 || smaller >= dt {
                        return Err(SimpleError::PhysicalBoundary(format!("Time step {step} cannot satisfy coupled convergence / Courant within adaptive limits; last accepted time is {time:e} s")).into());
                    }
                    dt = smaller;
                    continue;
                }
                return Err(SimpleError::PhysicalBoundary(format!("Time step {step} at t={next:e} failed nonlinear convergence after {} correctors; physical time was not advanced",result.report.outer_iterations)).into());
            }
            result.physical_time = Some(next);
            result.last_time_step = Some(dt);
            result.rejected_time_steps = rejected;
            result.completed_time_steps = step;
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
        step_case.initial_conditions = result.restart();
        last = Some(result);
    }
    let mut result = last.expect("validated nonzero time steps");
    result.report.outer_iterations = steps;
    result.report.simple.outer_iterations = steps;
    result.report.initial_continuity_rms = initial_continuity.unwrap();
    result.report.simple.initial_continuity_rms = initial_continuity.unwrap();
    result.report.continuity_history = histories.clone();
    result.report.simple.continuity_history = histories;
    result.report.simple.momentum_relative_history = momentum_history;
    result.report.simple.velocity_change_history = velocity_history;
    result.report.simple.pressure_linear_history = pressure_history;
    Ok(result)
}
