//! Compact terminal solve: independent diagonal constraints never enter PCG.
use super::hierarchy::Scratch;
use super::{
    hierarchy::{cholesky, solve_factor},
    matrix::SymmetricMatrix,
    smoother, MultigridCoarseSolver, MultigridOptions,
};
use crate::{
    pcg_with_preconditioner_and_workspace, CsrMatrix, LinearAlgebraError, LinearSolverOptions,
};

pub(super) struct CoarseSystem {
    rows: Vec<usize>,
    entries: Vec<usize>,
    matrix: CsrMatrix,
    kernel: SymmetricMatrix,
    inverse_dic: Vec<f64>,
    factor: Option<Vec<f64>>,
}

impl CoarseSystem {
    pub fn new(
        matrix: &CsrMatrix,
        options: MultigridOptions,
    ) -> Result<Option<Self>, LinearAlgebraError> {
        // Use structural connections, including zero coefficients: later updates
        // may activate an edge without changing the cached matrix topology.
        let rows: Vec<_> = (0..matrix.nrows())
            .filter(|&row| {
                matrix.column_indices()[matrix.row_offsets()[row]..matrix.row_offsets()[row + 1]]
                    .iter()
                    .any(|&column| column != row)
            })
            .collect();
        if rows.is_empty() {
            return Ok(None);
        }
        let mut map = vec![usize::MAX; matrix.nrows()];
        for (local, &row) in rows.iter().enumerate() {
            map[row] = local;
        }
        let mut offsets = vec![0];
        let mut columns = Vec::new();
        let mut entries = Vec::new();
        let mut values = Vec::new();
        for &row in &rows {
            for entry in matrix.row_offsets()[row]..matrix.row_offsets()[row + 1] {
                columns.push(map[matrix.column_indices()[entry]]);
                entries.push(entry);
                values.push(matrix.values()[entry]);
            }
            offsets.push(values.len());
        }
        let compact = CsrMatrix::new(rows.len(), rows.len(), offsets, columns, values)?;
        let kernel = SymmetricMatrix::new(&compact)?;
        let factor = if rows.len() <= options.coarsest_cells
            && options.coarse_solver == MultigridCoarseSolver::Cholesky
        {
            Some(cholesky(&compact)?)
        } else {
            None
        };
        let inverse_dic = if factor.is_none() {
            smoother::dic_diagonal(&kernel)?
        } else {
            Vec::new()
        };
        Ok(Some(Self {
            rows,
            entries,
            matrix: compact,
            kernel,
            inverse_dic,
            factor,
        }))
    }

    pub fn len(&self) -> usize {
        self.rows.len()
    }

    pub fn update(&mut self, matrix: &CsrMatrix) -> Result<(), LinearAlgebraError> {
        for (value, &entry) in self.matrix.values_mut().iter_mut().zip(&self.entries) {
            *value = matrix.values()[entry];
        }
        self.kernel.update(&self.matrix)?;
        if self.factor.is_some() {
            self.factor = Some(cholesky(&self.matrix)?);
        } else {
            smoother::update_dic_diagonal(&self.kernel, &mut self.inverse_dic)?;
        }
        Ok(())
    }

    pub fn solve(
        &self,
        rhs: &[f64],
        x: &mut [f64],
        work: &mut Scratch,
        options: MultigridOptions,
    ) -> Result<usize, LinearAlgebraError> {
        for (value, &row) in work.coarse_rhs.iter_mut().zip(&self.rows) {
            *value = rhs[row];
        }
        let iterations = if let Some(factor) = &self.factor {
            solve_factor(factor, &work.coarse_rhs, &mut work.coarse_solution);
            0
        } else {
            work.coarse_solution.fill(0.0);
            let preconditioner = smoother::DicPreconditioner {
                matrix: &self.kernel,
                inverse: &self.inverse_dic,
            };
            let report = pcg_with_preconditioner_and_workspace(
                &self.matrix,
                &work.coarse_rhs,
                &mut work.coarse_solution,
                &preconditioner,
                LinearSolverOptions {
                    absolute_tolerance: options.coarse_absolute_tolerance,
                    relative_tolerance: options.coarse_relative_tolerance,
                    max_iterations: options.coarse_max_iterations,
                },
                &mut work.cg,
            )?;
            if !report.converged() {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "multigrid coarse iterative solve did not converge",
                });
            }
            report.iterations
        };
        for (&value, &row) in work.coarse_solution.iter().zip(&self.rows) {
            x[row] = value;
        }
        Ok(iterations)
    }
}
