//! Standalone algebraic multigrid with connected aggregation and scaled corrections.
//! Residual-dependent scaling and directional smoothers are kept outside PCG.
mod acceleration;
mod aggregation;
mod coarse;
mod cycle;
mod hierarchy;
mod matrix;
mod options;
mod smoother;
use crate::{
    CsrMatrix, LinearAlgebraError, LinearSolveReport, LinearSolverOptions, LinearSolverStatus,
};
use hierarchy::{Level, Scratch};
pub use options::{
    MultigridCoarseSolver, MultigridOptions, MultigridResidualNorm, MultigridSmoother,
};

#[derive(Clone, Debug, Default)]
pub struct MultigridLevelStatistics {
    pub cells: usize,
    pub visits: u64,
    pub smoothing_sweeps: u64,
    pub coarse_iterations: u64,
    /// Includes recursive child levels; do not sum these times across levels.
    pub inclusive_seconds: f64,
}

pub struct AlgebraicMultigrid {
    options: MultigridOptions,
    levels: Vec<Level>,
    scratch: Vec<Scratch>,
    statistics: Vec<MultigridLevelStatistics>,
    product: Vec<f64>,
    residual: Vec<f64>,
    correction: Vec<f64>,
    row_sums: Vec<f64>,
    acceleration: acceleration::GcrWorkspace,
}

impl AlgebraicMultigrid {
    pub fn new(matrix: &CsrMatrix, options: MultigridOptions) -> Result<Self, LinearAlgebraError> {
        Self::with_weights(matrix, options, None)
    }

    /// Use fixed geometric face strengths instead of flow-dependent matrix
    /// coefficients for connected aggregation. Weights follow CSR entries;
    /// diagonal weights are ignored and internal weights sum on coarse levels.
    pub fn new_with_face_weights(
        matrix: &CsrMatrix,
        face_weights: &[f64],
        options: MultigridOptions,
    ) -> Result<Self, LinearAlgebraError> {
        if face_weights.len() != matrix.nnz() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: matrix.nnz(),
                actual: face_weights.len(),
            });
        }
        if face_weights
            .iter()
            .any(|weight| !weight.is_finite() || *weight < 0.0)
        {
            return Err(LinearAlgebraError::Breakdown {
                reason: "invalid geometric multigrid face weights",
            });
        }
        Self::with_weights(matrix, options, Some(face_weights))
    }

    fn with_weights(
        matrix: &CsrMatrix,
        options: MultigridOptions,
        weights: Option<&[f64]>,
    ) -> Result<Self, LinearAlgebraError> {
        let (levels, scratch) = hierarchy::build(matrix, options, weights)?;
        let statistics = levels
            .iter()
            .map(|level| MultigridLevelStatistics {
                cells: level.matrix.nrows(),
                ..Default::default()
            })
            .collect();
        let mut row_sums = vec![0.0; matrix.nrows()];
        if options.residual_norm == MultigridResidualNorm::NormalizedL1 {
            levels[0].kernel.row_sums_into(&mut row_sums);
        }
        Ok(Self {
            options,
            levels,
            scratch,
            statistics,
            product: vec![0.0; matrix.nrows()],
            residual: vec![0.0; matrix.nrows()],
            correction: vec![0.0; matrix.nrows()],
            row_sums,
            acceleration: acceleration::GcrWorkspace::new(matrix.nrows(), options.krylov_dimension),
        })
    }

    /// Refresh coefficients/factors without changing the cached connectivity.
    pub fn update_values(&mut self, matrix: &CsrMatrix) -> Result<(), LinearAlgebraError> {
        hierarchy::update(&mut self.levels, matrix)?;
        self.acceleration.reset();
        if self.options.residual_norm == MultigridResidualNorm::NormalizedL1 {
            self.levels[0].kernel.row_sums_into(&mut self.row_sums);
        }
        Ok(())
    }

    pub fn statistics(&self) -> &[MultigridLevelStatistics] {
        &self.statistics
    }

    pub fn solve(
        &mut self,
        rhs: &[f64],
        x: &mut [f64],
        options: LinearSolverOptions,
    ) -> Result<LinearSolveReport, LinearAlgebraError> {
        let n = self.product.len();
        for size in [rhs.len(), x.len()] {
            if size != n {
                return Err(LinearAlgebraError::DimensionMismatch {
                    expected: n,
                    actual: size,
                });
            }
        }
        if options.max_iterations == 0
            || !options.absolute_tolerance.is_finite()
            || !options.relative_tolerance.is_finite()
            || options.absolute_tolerance < 0.0
            || options.relative_tolerance < 0.0
            || rhs.iter().chain(x.iter()).any(|v| !v.is_finite())
        {
            return Err(LinearAlgebraError::Breakdown {
                reason: "invalid multigrid solve inputs",
            });
        }
        self.acceleration.reset();
        self.levels[0].kernel.apply(x, &mut self.product);
        let factor = match self.options.residual_norm {
            MultigridResidualNorm::AbsoluteL2 => 1.0,
            MultigridResidualNorm::NormalizedL1 => {
                let mean = x.iter().sum::<f64>() / n as f64;
                rhs.iter()
                    .zip(&self.product)
                    .zip(&self.row_sums)
                    .map(|((&b, &ax), &sum)| (ax - sum * mean).abs() + (b - sum * mean).abs())
                    .sum::<f64>()
                    .max(1e-300)
            }
        };
        if !factor.is_finite() {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite multigrid normalization",
            });
        }
        for row in 0..n {
            self.residual[row] = rhs[row] - self.product[row];
        }
        let initial = self.measure(factor)?;
        let threshold = options
            .absolute_tolerance
            .max(options.relative_tolerance * initial);
        let report = |status, iterations, final_residual| LinearSolveReport {
            status,
            iterations,
            initial_residual: initial,
            final_residual,
            relative_residual: if initial > 0.0 {
                final_residual / initial
            } else {
                0.0
            },
        };
        if initial <= threshold {
            return Ok(report(LinearSolverStatus::Converged, 0, initial));
        }
        for iteration in 1..=options.max_iterations {
            self.correction.fill(0.0);
            cycle::cycle(
                &self.levels,
                &mut self.scratch,
                &mut self.statistics,
                self.options,
                0,
                &self.residual,
                &mut self.correction,
            )?;
            if self.options.krylov_dimension == 0 {
                for row in 0..n {
                    x[row] += self.correction[row];
                }
            } else {
                self.levels[0]
                    .kernel
                    .apply(&self.correction, &mut self.product);
                self.acceleration.correct(
                    &mut self.correction,
                    &mut self.product,
                    &self.residual,
                    x,
                )?;
            }
            // Always check the true residual, including after a restart.
            self.levels[0].kernel.apply(x, &mut self.product);
            for row in 0..n {
                self.residual[row] = rhs[row] - self.product[row];
            }
            let residual = self.measure(factor)?;
            if residual <= threshold {
                return Ok(report(LinearSolverStatus::Converged, iteration, residual));
            }
        }
        Ok(report(
            LinearSolverStatus::MaxIterations,
            options.max_iterations,
            self.measure(factor)?,
        ))
    }

    fn measure(&self, factor: f64) -> Result<f64, LinearAlgebraError> {
        let value = match self.options.residual_norm {
            MultigridResidualNorm::AbsoluteL2 => {
                self.residual.iter().map(|v| v * v).sum::<f64>().sqrt()
            }
            MultigridResidualNorm::NormalizedL1 => {
                self.residual.iter().map(|v| v.abs()).sum::<f64>() / factor
            }
        };
        if !value.is_finite() {
            return Err(LinearAlgebraError::Breakdown {
                reason: "non-finite multigrid residual",
            });
        }
        Ok(value)
    }
}
