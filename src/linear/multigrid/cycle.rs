use super::{
    hierarchy::{Level, Scratch},
    smoother, MultigridLevelStatistics, MultigridOptions,
};
use crate::LinearAlgebraError;
use std::time::Instant;

pub(super) fn cycle(
    levels: &[Level],
    scratch: &mut [Scratch],
    statistics: &mut [MultigridLevelStatistics],
    options: MultigridOptions,
    depth: usize,
    rhs: &[f64],
    x: &mut [f64],
) -> Result<(), LinearAlgebraError> {
    let started = options.profile.then(Instant::now);
    let level = &levels[0];
    let (work, rest) = scratch
        .split_first_mut()
        .ok_or(LinearAlgebraError::Breakdown {
            reason: "missing multigrid scratch",
        })?;
    let (stats, remaining) = statistics
        .split_first_mut()
        .ok_or(LinearAlgebraError::Breakdown {
            reason: "missing multigrid statistics",
        })?;
    stats.visits += 1;
    let result = (|| {
        if level.aggregates.is_none() {
            for row in 0..rhs.len() {
                x[row] = rhs[row] * level.kernel.inverse[row];
            }
            if let Some(terminal) = &level.terminal {
                stats.coarse_iterations += terminal.solve(rhs, x, work, options)? as u64;
            }
            if x.iter().any(|x| !x.is_finite()) {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "non-finite coarse solution",
                });
            }
            return Ok(());
        }
        let extra = options
            .sweep_level_multiplier
            .saturating_mul(depth.saturating_sub(1));
        let pre = if depth == 0 {
            0
        } else {
            options
                .pre_sweeps
                .saturating_add(if options.pre_sweeps == 0 { 0 } else { extra })
                .min(options.max_sweeps)
        };
        smoother::smooth(
            &level.kernel,
            &level.inverse_dic,
            options.smoother,
            rhs,
            x,
            &mut work.work,
            &mut work.correction,
            pre,
        )?;
        // Every recursive correction starts at zero. With no pre-sweeps,
        // the residual is exactly rhs: do not multiply A by a zero vector.
        let residual = if pre == 0 {
            rhs
        } else {
            level.kernel.apply(x, &mut work.product);
            for row in 0..rhs.len() {
                work.residual[row] = rhs[row] - work.product[row];
            }
            &work.residual
        };
        let map = level.aggregates.as_ref().unwrap();
        work.coarse_rhs.fill(0.0);
        for row in 0..rhs.len() {
            work.coarse_rhs[map[row]] += residual[row];
        }
        work.coarse_solution.fill(0.0);
        cycle(
            &levels[1..],
            rest,
            remaining,
            options,
            depth + 1,
            &work.coarse_rhs,
            &mut work.coarse_solution,
        )?;
        for row in 0..rhs.len() {
            work.correction[row] = work.coarse_solution[map[row]];
        }
        if options.scale_correction {
            level.kernel.apply(&work.correction, &mut work.product);
            let (mut numerator, mut denominator) = (0.0, 0.0);
            for row in 0..rhs.len() {
                numerator += work.correction[row] * residual[row];
                denominator += work.correction[row] * work.product[row];
            }
            if !numerator.is_finite() || !denominator.is_finite() {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "non-finite multigrid correction product",
                });
            }
            let factor = if denominator > 0.0 {
                numerator / denominator
            } else {
                0.0
            };
            if !factor.is_finite() {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "non-finite multigrid correction scale",
                });
            }
            for row in 0..rhs.len() {
                x[row] += factor * work.correction[row]
                    + (residual[row] - factor * work.product[row]) * level.kernel.inverse[row];
            }
        } else {
            for row in 0..rhs.len() {
                x[row] += work.correction[row];
            }
        }
        let post = if depth == 0 {
            options.finest_sweeps
        } else {
            options
                .post_sweeps
                .saturating_add(extra)
                .min(options.max_sweeps)
        };
        smoother::smooth(
            &level.kernel,
            &level.inverse_dic,
            options.smoother,
            rhs,
            x,
            &mut work.work,
            &mut work.correction,
            post,
        )?;
        stats.smoothing_sweeps += (pre + post) as u64;
        Ok(())
    })();
    if let Some(started) = started {
        stats.inclusive_seconds += started.elapsed().as_secs_f64();
    }
    result
}
