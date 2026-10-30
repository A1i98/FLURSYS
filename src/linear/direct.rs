use super::{
    residual_into, CsrMatrix, LinearAlgebraError, LinearSolveReport, LinearSolverOptions,
    LinearSolverStatus,
};
use sprs::{CsMatView, PermOwned, SymmetryCheck};
use sprs_ldl::{LdlNumeric, LdlSymbolic};

pub(crate) struct PressureDirect {
    matrix: CsrMatrix,
    factor: LdlNumeric<f64, usize>,
}
impl PressureDirect {
    pub fn new(matrix: &CsrMatrix) -> Result<Option<Self>, LinearAlgebraError> {
        let _timer = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerSetup);
        let n = matrix.nrows();
        if n > 1_000_000 || matrix.nnz() > i32::MAX as usize {
            return Ok(None);
        }
        crate::compute::checkpoint()?;
        let offsets: Vec<i32> = matrix.row_offsets().iter().map(|&i| i as i32).collect();
        let indices: Vec<i32> = matrix.column_indices().iter().map(|&i| i as i32).collect();
        let (permutation, _, _) =
            amd::order(n as i32, &offsets, &indices, &amd::Control::default()).map_err(|_| {
                LinearAlgebraError::Breakdown {
                    reason: "pressure AMD ordering failed",
                }
            })?;
        crate::compute::checkpoint()?;
        let view = view(matrix);
        if !sprs::is_symmetric(&view) {
            return Ok(None);
        }
        let symbolic = LdlSymbolic::new_perm(
            view,
            PermOwned::new(permutation.into_iter().map(|i| i as usize).collect()),
            SymmetryCheck::DontCheckSymmetry,
        );
        let bytes = symbolic
            .nnz()
            .checked_mul(16)
            .and_then(|v| v.checked_add(n.saturating_mul(96)));
        if bytes.is_none_or(|bytes| bytes > 256 * 1024 * 1024) {
            return Ok(None);
        }
        crate::compute::checkpoint()?;
        let factor = symbolic
            .factor(view)
            .map_err(|e| LinearAlgebraError::Backend {
                message: format!("Pressure LDLT: {e}"),
            })?;
        validate_diagonal(&factor)?;
        Ok(Some(Self {
            matrix: matrix.clone(),
            factor,
        }))
    }
    fn update(&mut self, matrix: &CsrMatrix) -> Result<(), LinearAlgebraError> {
        let _setup = crate::compute::StageTimer::new(crate::compute::Stage::PreconditionerSetup);
        self.factor
            .update(view(matrix))
            .map_err(|e| LinearAlgebraError::Backend {
                message: format!("Pressure LDLT update: {e}"),
            })?;
        validate_diagonal(&self.factor)?;
        self.matrix = matrix.clone();
        Ok(())
    }
    pub fn matches_pattern(&self, matrix: &CsrMatrix) -> bool {
        self.matrix.row_offsets() == matrix.row_offsets()
            && self.matrix.column_indices() == matrix.column_indices()
    }
    pub fn solve(
        &mut self,
        matrix: &CsrMatrix,
        rhs: &[f64],
        x: &mut [f64],
        options: LinearSolverOptions,
    ) -> Result<LinearSolveReport, LinearAlgebraError> {
        super::validate_solve_inputs(matrix, rhs, x, options)?;
        if !self.matches_pattern(matrix) {
            return Err(LinearAlgebraError::Breakdown {
                reason: "pressure factor belongs to a different sparsity pattern",
            });
        }
        let _timer = crate::compute::Timer::linear();
        let changed = self.matrix != *matrix;
        let reuse = changed
            && options.max_iterations >= 6
            && self
                .matrix
                .values()
                .iter()
                .zip(matrix.values())
                .all(|(old, new)| (old - new).abs() <= 1e-3 * old.abs().max(new.abs()));
        if changed && !reuse {
            self.update(matrix)?;
        }
        let mut residual = vec![0.; rhs.len()];
        residual_into(matrix, x, rhs, &mut residual)?;
        let initial = super::norm_l2(&residual)?;
        let target = options.absolute_tolerance + options.relative_tolerance * initial;
        let mut final_residual = initial;
        let mut iterations = 0;
        while final_residual > target
            && iterations < options.max_iterations.min(if reuse { 6 } else { 3 })
        {
            crate::compute::checkpoint()?;
            if reuse && iterations == 3 {
                self.update(matrix)?;
            }
            let correction: Vec<f64> = self.factor.solve(&residual);
            for (value, delta) in x.iter_mut().zip(correction) {
                *value += delta;
            }
            residual_into(matrix, x, rhs, &mut residual)?;
            final_residual = super::norm_l2(&residual)?;
            iterations += 1;
        }
        Ok(LinearSolveReport {
            status: if final_residual <= target {
                LinearSolverStatus::Converged
            } else {
                LinearSolverStatus::MaxIterations
            },
            iterations,
            initial_residual: initial,
            final_residual,
            relative_residual: if initial > 0. {
                final_residual / initial
            } else {
                0.
            },
        })
    }
}
fn view(matrix: &CsrMatrix) -> CsMatView<'_, f64> {
    // A symmetric CSR matrix has the same arrays as its CSC transpose.
    CsMatView::new_csc(
        (matrix.nrows(), matrix.ncols()),
        matrix.row_offsets(),
        matrix.column_indices(),
        matrix.values(),
    )
}
fn validate_diagonal(factor: &LdlNumeric<f64, usize>) -> Result<(), LinearAlgebraError> {
    if factor.d().iter().any(|d| !d.is_finite() || *d <= 0.) {
        return Err(LinearAlgebraError::Breakdown {
            reason: "pressure LDLT requires a positive-definite operator",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CsrBuilder;
    fn matrix(scale: f64) -> CsrMatrix {
        let mut builder = CsrBuilder::new(4, 4);
        for (a, b, w) in [(0, 1, 1.), (1, 2, 1e4), (2, 3, 1.)] {
            for (row, col, value) in [(a, a, w), (b, b, w), (a, b, -w), (b, a, -w)] {
                builder.add(row, col, value * scale).unwrap();
            }
        }
        builder.add(3, 3, scale).unwrap();
        builder.finalize().unwrap()
    }
    #[test]
    fn direct_pressure_reuses_factors_and_refreshes_changed_coefficients() {
        let expected = [3., 2., 1., 0.];
        let mut solver = PressureDirect::new(&matrix(1.)).unwrap().unwrap();
        for scale in [1., 0.1, 1e3] {
            let matrix = matrix(scale);
            let mut rhs = [0.; 4];
            matrix.spmv_into(&expected, &mut rhs).unwrap();
            let mut x = [0.; 4];
            let options = LinearSolverOptions {
                absolute_tolerance: 1e-10 * scale,
                relative_tolerance: 1e-9,
                max_iterations: 3,
            };
            let report = solver.solve(&matrix, &rhs, &mut x, options).unwrap();
            assert!(report.converged(), "{report:?}");
            assert!(
                report.final_residual
                    <= options.absolute_tolerance
                        + options.relative_tolerance * report.initial_residual
            );
            assert!(x.iter().zip(expected).all(|(a, b)| (a - b).abs() < 1e-8));
            let unchanged = solver.solve(&matrix, &rhs, &mut x, options).unwrap();
            assert_eq!(unchanged.iterations, 0);
        }
    }
    #[test]
    fn small_coefficient_changes_reuse_factor_with_true_residual_refinement() {
        let original = matrix(1.);
        let changed = matrix(1.00001);
        let mut solver = PressureDirect::new(&original).unwrap().unwrap();
        let expected = [3., 2., 1., 0.];
        let mut rhs = [0.; 4];
        changed.spmv_into(&expected, &mut rhs).unwrap();
        let mut x = [0.; 4];
        let report = solver
            .solve(&changed, &rhs, &mut x, LinearSolverOptions::default())
            .unwrap();
        assert!(report.converged(), "{report:?}");
        assert_eq!(solver.matrix, original);
        assert!(x.iter().zip(expected).all(|(a, b)| (a - b).abs() < 1e-7));
    }
    #[test]
    fn direct_pressure_rejects_non_positive_operator() {
        let mut builder = CsrBuilder::new(2, 2);
        builder.add(0, 0, 1.).unwrap();
        builder.add(1, 1, -1.).unwrap();
        assert!(PressureDirect::new(&builder.finalize().unwrap()).is_err());
    }
}
