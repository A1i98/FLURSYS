//! Restarted right-preconditioned flexible GMRES. All Arnoldi algebra and true
//! residual checks use f64, independently of preconditioner working precision.
use super::vector::{dot_with_scratch, norm_l2_with_scratch};
use super::{
    axpy, residual_into, CsrMatrix, LinearAlgebraError, LinearSolveReport, LinearSolverOptions,
    LinearSolverStatus,
};
pub(crate) fn fgmres(
    matrix: &CsrMatrix,
    rhs: &[f64],
    x: &mut [f64],
    options: LinearSolverOptions,
    restart: usize,
    mut precondition: impl FnMut(&[f64]) -> Result<Vec<f64>, LinearAlgebraError>,
) -> Result<LinearSolveReport, LinearAlgebraError> {
    super::validate_solve_inputs(matrix, rhs, x, options)?;
    let n = matrix.nrows();
    let restart = restart.clamp(2, 64).min(options.max_iterations.max(1));
    let mut r = vec![0.; n];
    let mut partials = Vec::new();
    residual_into(matrix, x, rhs, &mut r)?;
    let initial = norm_l2_with_scratch(&r, &mut partials)?;
    let target = options
        .absolute_tolerance
        .max(options.relative_tolerance * initial);
    let mut norm = initial;
    let mut iterations = 0;
    while norm > target && iterations < options.max_iterations {
        crate::compute::checkpoint()?;
        let length = restart.min(options.max_iterations - iterations);
        let mut basis = Vec::with_capacity(length + 1);
        basis.push(r.iter().map(|v| v / norm).collect::<Vec<_>>());
        let mut corrections = Vec::with_capacity(length);
        let mut h = vec![vec![0.; length]; length + 1];
        let mut cosine = vec![0.; length];
        let mut sine = vec![0.; length];
        let mut g = vec![0.; length + 1];
        g[0] = norm;
        let mut used = 0;
        for j in 0..length {
            crate::compute::checkpoint()?;
            let z = precondition(&basis[j])?;
            if z.len() != n {
                return Err(LinearAlgebraError::DimensionMismatch {
                    expected: n,
                    actual: z.len(),
                });
            }
            let mut w = matrix.spmv(&z)?;
            let product_norm = norm_l2_with_scratch(&w, &mut partials)?;
            // Two MGS passes are deliberate: mixed-precision preconditioning
            // can otherwise lose the Arnoldi basis near tight tolerances.
            for _ in 0..2 {
                for i in 0..=j {
                    let value = dot_with_scratch(&basis[i], &w, &mut partials)?;
                    h[i][j] += value;
                    axpy(-value, &basis[i], &mut w)?;
                }
            }
            let next = norm_l2_with_scratch(&w, &mut partials)?;
            h[j + 1][j] = next;
            corrections.push(z);
            let happy = next <= f64::EPSILON * product_norm.max(f64::MIN_POSITIVE);
            if !happy {
                basis.push(w.iter().map(|v| v / next).collect());
            }
            for i in 0..j {
                let a = h[i][j];
                let b = h[i + 1][j];
                h[i][j] = cosine[i] * a + sine[i] * b;
                h[i + 1][j] = -sine[i] * a + cosine[i] * b;
            }
            let diagonal = h[j][j].hypot(h[j + 1][j]);
            if !diagonal.is_finite() || diagonal == 0. {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "FGMRES rank-deficient preconditioned search space",
                });
            }
            cosine[j] = h[j][j] / diagonal;
            sine[j] = h[j + 1][j] / diagonal;
            h[j][j] = diagonal;
            h[j + 1][j] = 0.;
            g[j + 1] = -sine[j] * g[j];
            g[j] *= cosine[j];
            used = j + 1;
            iterations += 1;
            if happy || g[j + 1].abs() <= target {
                break;
            }
        }
        let mut y = vec![0.; used];
        for i in (0..used).rev() {
            let mut value = g[i];
            for k in i + 1..used {
                value -= h[i][k] * y[k];
            }
            y[i] = value / h[i][i];
        }
        for (z, &weight) in corrections.iter().zip(&y) {
            axpy(weight, z, x)?;
        }
        residual_into(matrix, x, rhs, &mut r)?;
        norm = norm_l2_with_scratch(&r, &mut partials)?;
        // Never accept a small projected residual without b-Ax. A new cycle
        // starts from the recomputed residual if its estimate drifted.
    }
    Ok(LinearSolveReport {
        status: if norm <= target {
            LinearSolverStatus::Converged
        } else {
            LinearSolverStatus::MaxIterations
        },
        iterations,
        initial_residual: initial,
        final_residual: norm,
        relative_residual: if initial > 0. { norm / initial } else { 0. },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compute::{Backend, ComputeOptions, Execution, Scope};
    use crate::{norm_l2, residual, CsrBuilder};

    #[test]
    fn cpu_fgmres_restarts_preserve_serial_parallel_results_and_true_residual() {
        let n = 4097;
        let mut builder = CsrBuilder::new(n, n);
        for i in 0..n {
            builder.add(i, i, 1.0 + (i % 5) as f64 * 0.1).unwrap();
        }
        let matrix = builder.finalize().unwrap();
        let exact: Vec<_> = (0..n).map(|i| (i as f64 * 0.13).sin()).collect();
        let rhs = matrix.spmv(&exact).unwrap();
        let options = LinearSolverOptions {
            absolute_tolerance: 1e-10,
            relative_tolerance: 0.0,
            max_iterations: 100,
        };
        let mut expected = None;
        for backend in [Backend::Serial, Backend::Parallel] {
            let runtime = Execution::new(ComputeOptions {
                backend,
                ..Default::default()
            })
            .unwrap();
            let result = rayon::ThreadPoolBuilder::new()
                .num_threads(2)
                .build()
                .unwrap()
                .install(|| {
                    let _scope = Scope::enter(runtime);
                    let mut x = vec![0.0; n];
                    let report =
                        fgmres(&matrix, &rhs, &mut x, options, 2, |r| Ok(r.to_vec())).unwrap();
                    assert!(report.converged());
                    assert!(report.iterations > 2, "must exercise restart scratch reuse");
                    let actual = norm_l2(&residual(&matrix, &x, &rhs).unwrap()).unwrap();
                    assert!(actual <= options.absolute_tolerance);
                    assert_eq!(report.final_residual.to_bits(), actual.to_bits());
                    (report, x)
                });
            if let Some(expected) = &expected {
                assert_eq!(&result, expected);
            } else {
                expected = Some(result);
            }
        }
    }
}
