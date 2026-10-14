use crate::LinearAlgebraError;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MultigridSmoother {
    GaussSeidel,
    SymmetricGaussSeidel,
    DicGaussSeidel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MultigridResidualNorm {
    AbsoluteL2,
    NormalizedL1,
}

/// Terminal-level solve. Dense factorization is bounded by coarsest_cells.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MultigridCoarseSolver {
    PcgDic,
    Cholesky,
}

/// Controls for a standalone algebraic multigrid solve, not a PCG preconditioner.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MultigridOptions {
    pub smoother: MultigridSmoother,
    pub residual_norm: MultigridResidualNorm,
    pub pre_sweeps: usize,
    pub post_sweeps: usize,
    pub finest_sweeps: usize,
    pub sweep_level_multiplier: usize,
    pub max_sweeps: usize,
    /// Maximum coupled unknowns on the terminal level; independent diagonal
    /// constraints are solved separately and do not extend the hierarchy.
    pub coarsest_cells: usize,
    pub max_levels: usize,
    /// Pairing stages combined into one stored level.
    pub merge_levels: usize,
    /// Restart length for flexible GCR acceleration; zero selects plain cycles.
    pub krylov_dimension: usize,
    pub coarse_solver: MultigridCoarseSolver,
    pub coarse_absolute_tolerance: f64,
    pub coarse_relative_tolerance: f64,
    pub coarse_max_iterations: usize,
    pub scale_correction: bool,
    pub profile: bool,
}

impl Default for MultigridOptions {
    fn default() -> Self {
        Self {
            smoother: MultigridSmoother::DicGaussSeidel,
            residual_norm: MultigridResidualNorm::NormalizedL1,
            pre_sweeps: 0,
            post_sweeps: 2,
            finest_sweeps: 1,
            sweep_level_multiplier: 1,
            max_sweeps: 4,
            coarsest_cells: 10,
            max_levels: 64,
            merge_levels: 2,
            krylov_dimension: 4,
            coarse_solver: MultigridCoarseSolver::PcgDic,
            coarse_absolute_tolerance: 1e-12,
            coarse_relative_tolerance: 0.01,
            coarse_max_iterations: 500,
            scale_correction: true,
            profile: false,
        }
    }
}

impl MultigridOptions {
    pub fn validate(&self) -> Result<(), LinearAlgebraError> {
        if self.max_sweeps == 0
            || self.max_sweeps > 64
            || self.pre_sweeps > self.max_sweeps
            || self.post_sweeps > self.max_sweeps
            || self.finest_sweeps == 0
            || self.finest_sweeps > 64
            || self.coarsest_cells == 0
            || self.coarsest_cells > 256
            || self.max_levels == 0
            || self.max_levels > 128
            || !(1..=4).contains(&self.merge_levels)
            || self.krylov_dimension > 16
            || !self.coarse_absolute_tolerance.is_finite()
            || self.coarse_absolute_tolerance < 0.0
            || !self.coarse_relative_tolerance.is_finite()
            || self.coarse_relative_tolerance < 0.0
            || self.coarse_max_iterations == 0
        {
            return Err(LinearAlgebraError::Breakdown {
                reason: "invalid multigrid controls",
            });
        }
        Ok(())
    }
}
