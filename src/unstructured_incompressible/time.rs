//! Fixed-mesh physical-time integration. Nonlinear iterations never advance time.
use super::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "kebab-case")]
pub enum TimeIntegration {
    #[default]
    Steady,
    /// Implicit Euler; `max_outer_iterations` is the nonlinear budget per step.
    Euler { dt: f64, steps: usize },
}
impl TimeIntegration {
    pub fn validate(self) -> Result<(), String> {
        match self {
            Self::Steady => Ok(()),
            Self::Euler { dt, steps }
                if dt.is_finite() && dt > 0.0 && steps > 0 && steps <= 1_000_000 =>
            {
                Ok(())
            }
            _ => Err(
                "Transient integration requires positive finite dt and 1..1000000 time steps"
                    .into(),
            ),
        }
    }
}

pub(super) fn solve_time_loop(
    case: &IncompressibleCase,
    control: Option<&SolveControl<'_>>,
) -> Result<IncompressibleSolution, IncompressibleSolveError> {
    case.solver
        .time
        .validate()
        .map_err(|e| SimpleError::PhysicalBoundary(e))?;
    if matches!(case.solver.time, TimeIntegration::Steady)
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
    let TimeIntegration::Euler { dt, steps } = case.solver.time else {
        let result = solve_incompressible_on_pool(case, control, None)?;
        if let Some(cb) = control.and_then(|c| c.on_step.as_ref()) {
            cb(result.report.outer_iterations, &result).map_err(SimpleError::Monitor)?;
        }
        return Ok(result);
    };
    let mut step_case = case.clone();
    step_case.solver.time = TimeIntegration::Steady;
    let mut time = match &case.initial_conditions {
        IncompressibleInitialConditions::Restart { time, .. } => *time,
        _ => 0.0,
    };
    let mut last = None;
    let mut initial_continuity = None;
    let mut histories = Vec::new();
    let mut momentum_history = Vec::new();
    let mut velocity_history = Vec::new();
    let mut pressure_history = Vec::new();
    let mut correctors = 0;
    for step in 1..=steps {
        if control.is_some_and(SolveControl::cancelled) {
            return Err(IncompressibleSolveError::Cancelled);
        }
        let next = time + dt;
        if !next.is_finite() || next <= time {
            return Err(SimpleError::PhysicalBoundary(
                "Physical time overflow or dt below time resolution".into(),
            )
            .into());
        }
        // Inner iteration callbacks are suppressed: public iterations denote
        // completed physical steps for this mode, not SIMPLE iterates.
        let inner = SolveControl::new(
            || control.is_some_and(SolveControl::cancelled),
            |_, _, _| {},
        );
        let mut result = solve_incompressible_on_pool(&step_case, Some(&inner), Some(dt))?;
        if !result.report.converged() {
            return Err(SimpleError::PhysicalBoundary(format!("Time step {step} at t={next:e} failed nonlinear convergence after {} correctors; physical time was not advanced", result.report.outer_iterations)).into());
        }
        initial_continuity.get_or_insert(result.report.initial_continuity_rms);
        correctors += result.report.outer_iterations;
        time = next;
        result.physical_time = Some(time);
        let mut sum = vec![0.0; case.mesh.cell_count()];
        for (f, phi) in case.mesh.faces().iter().zip(result.face_flux.values()) {
            sum[f.owner] += phi.abs();
            if let Some(n) = f.neighbour {
                sum[n] += phi.abs();
            }
        }
        result.max_courant = Some(
            sum.iter()
                .zip(case.mesh.cells())
                .map(|(s, c)| 0.5 * dt * s / c.volume)
                .fold(0.0, f64::max),
        );
        result.completed_time_steps = step;
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
        result.nonlinear_iterations = correctors;
        if let Some(cb) = control.and_then(|c| c.on_step.as_ref()) {
            cb(step, &result).map_err(SimpleError::Monitor)?;
        }
        if let Some(control) = control {
            control.report_iteration(step, steps, result.report.final_continuity_rms);
            if let Some(cb) = &control.on_metrics {
                cb(
                    step,
                    [
                        result.report.final_continuity_rms,
                        result.report.simple.final_momentum_relative_residual,
                        result.report.simple.final_velocity_relative_change,
                        result
                            .report
                            .simple
                            .pressure_report
                            .as_ref()
                            .map_or(0.0, |r| r.final_residual),
                    ],
                );
            }
            if let Some(cb) = &control.on_fields {
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
