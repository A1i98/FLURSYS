//! Restarted right-preconditioned flexible GMRES. All Arnoldi algebra and true
//! residual checks use f64, independently of preconditioner working precision.
use super::{
    axpy, dot, norm_l2, residual_into, CsrMatrix, LinearAlgebraError, LinearSolveReport,
    LinearSolverOptions, LinearSolverStatus,
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
    residual_into(matrix, x, rhs, &mut r)?;
    let initial = norm_l2(&r)?;
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
            let product_norm = norm_l2(&w)?;
            // Two MGS passes are deliberate: mixed-precision preconditioning
            // can otherwise lose the Arnoldi basis near tight tolerances.
            for _ in 0..2 {
                for i in 0..=j {
                    let value = dot(&basis[i], &w)?;
                    h[i][j] += value;
                    axpy(-value, &basis[i], &mut w)?;
                }
            }
            let next = norm_l2(&w)?;
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
        norm = norm_l2(&r)?;
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
