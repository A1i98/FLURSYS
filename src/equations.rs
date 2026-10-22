//! Per-equation sparse solver controls. Residual tolerances use Euclidean norms.
use crate::*;
use serde::{Deserialize, Serialize};
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EquationPreconditioner {
    #[default]
    None,
    Jacobi,
    Ic0,
    /// Standalone aggregation multigrid with flexible GCR acceleration.
    Gamg,
    /// IC0 when factorization succeeds; Jacobi if a positive IC pivot is unavailable.
    Auto,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EquationControl {
    pub absolute_tolerance: f64,
    pub relative_tolerance: f64,
    pub max_iterations: usize,
    pub preconditioner: EquationPreconditioner,
}
impl Default for EquationControl {
    fn default() -> Self {
        Self::from_options(LinearSolverOptions::default())
    }
}
impl EquationControl {
    pub fn from_options(o: LinearSolverOptions) -> Self {
        Self {
            absolute_tolerance: o.absolute_tolerance,
            relative_tolerance: o.relative_tolerance,
            max_iterations: o.max_iterations,
            preconditioner: EquationPreconditioner::None,
        }
    }
    pub fn options(self) -> LinearSolverOptions {
        LinearSolverOptions {
            absolute_tolerance: self.absolute_tolerance,
            relative_tolerance: self.relative_tolerance,
            max_iterations: self.max_iterations,
        }
    }
    pub fn validate(self, pressure: bool) -> Result<(), String> {
        if !self.absolute_tolerance.is_finite()
            || self.absolute_tolerance < 0.0
            || !self.relative_tolerance.is_finite()
            || !(0.0..1.0).contains(&self.relative_tolerance)
            || (self.absolute_tolerance == 0.0 && self.relative_tolerance == 0.0)
            || !(1..=1_000_000).contains(&self.max_iterations)
        {
            return Err("Require finite absTol ≥ 0, 0 ≤ relTol < 1, at least one positive tolerance and 1..1000000 linear iterations".into());
        }
        if !pressure
            && !matches!(
                self.preconditioner,
                EquationPreconditioner::None | EquationPreconditioner::Jacobi
            )
        {
            return Err(
                "IC0, GAMG and Auto are restricted to pressure; transport supports None or Jacobi"
                    .into(),
            );
        }
        Ok(())
    }
    pub fn solve(
        self,
        a: &CsrMatrix,
        b: &[f64],
        x: &mut [f64],
        pressure: bool,
    ) -> Result<LinearSolveReport, LinearAlgebraError> {
        self.validate(pressure)
            .map_err(|_| LinearAlgebraError::Breakdown {
                reason: "invalid equation solver controls",
            })?;
        if pressure
            && matches!(
                self.preconditioner,
                EquationPreconditioner::Gamg | EquationPreconditioner::Auto
            )
        {
            return PressureWorkspace::default().solve(self, a, b, x);
        }
        let o = self.options();
        match (pressure, self.preconditioner) {
            (true, EquationPreconditioner::None) => cg(a, b, x, o),
            (true, EquationPreconditioner::Jacobi) => {
                pcg(a, b, x, &JacobiPreconditioner::new(a)?, o)
            }
            (true, EquationPreconditioner::Ic0) => {
                pcg(a, b, x, &IncompleteCholeskyPreconditioner::new(a)?, o)
            }
            (false, EquationPreconditioner::None) => bicgstab(a, b, x, o),
            (false, EquationPreconditioner::Jacobi) => {
                bicgstab_with_preconditioner(a, b, x, &JacobiPreconditioner::new(a)?, o)
            }
            _ => Err(LinearAlgebraError::Breakdown {
                reason: "IC0 requires symmetric pressure matrix",
            }),
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct EquationSolvers {
    pub velocity: EquationControl,
    pub pressure: EquationControl,
    pub temperature: EquationControl,
    pub k: EquationControl,
    pub omega: EquationControl,
    pub epsilon: EquationControl,
    pub nu_tilda: EquationControl,
}
impl Default for EquationSolvers {
    fn default() -> Self {
        Self {
            pressure: EquationControl {
                preconditioner: EquationPreconditioner::Ic0,
                ..Default::default()
            },
            velocity: Default::default(),
            temperature: Default::default(),
            k: Default::default(),
            omega: Default::default(),
            epsilon: Default::default(),
            nu_tilda: Default::default(),
        }
    }
}
impl EquationSolvers {
    pub fn validate(&self) -> Result<(), String> {
        self.pressure.validate(true)?;
        for c in [
            self.velocity,
            self.temperature,
            self.k,
            self.omega,
            self.epsilon,
            self.nu_tilda,
        ] {
            c.validate(false)?;
        }
        Ok(())
    }
}

// Workspaces live for one coupled solve. Topology is checked exactly before
// refreshing coefficients; far-field switching may change the pressure graph.
#[derive(Default)]
pub(crate) struct PressureWorkspace {
    multigrid: Option<AlgebraicMultigrid>,
    topology: Option<(Vec<usize>, Vec<usize>)>,
    cg: CgWorkspace,
    warned_fallback: bool,
}
impl PressureWorkspace {
    pub(crate) fn solve(
        &mut self,
        control: EquationControl,
        a: &CsrMatrix,
        b: &[f64],
        x: &mut [f64],
    ) -> Result<LinearSolveReport, LinearAlgebraError> {
        control
            .validate(true)
            .map_err(|_| LinearAlgebraError::Breakdown {
                reason: "invalid pressure solver controls",
            })?;
        let options = control.options();
        match control.preconditioner {
            EquationPreconditioner::Gamg => {
                let same = self.topology.as_ref().is_some_and(|(rows, cols)| {
                    rows == a.row_offsets() && cols == a.column_indices()
                });
                if same {
                    if let Some(mg) = self.multigrid.as_mut() {
                        mg.update_values(a)?;
                    }
                } else {
                    // Preserve the equation dialog's L2 tolerances. Normalized
                    // L1 criteria used elsewhere are not numerically equivalent.
                    let mg = AlgebraicMultigrid::new(
                        a,
                        MultigridOptions {
                            residual_norm: MultigridResidualNorm::AbsoluteL2,
                            smoother: MultigridSmoother::SymmetricGaussSeidel,
                            coarse_solver: MultigridCoarseSolver::Cholesky,
                            coarsest_cells: 32,
                            ..Default::default()
                        },
                    )?;
                    self.multigrid = Some(mg);
                    self.topology = Some((a.row_offsets().to_vec(), a.column_indices().to_vec()));
                }
                self.multigrid
                    .as_mut()
                    .ok_or(LinearAlgebraError::Breakdown {
                        reason: "missing pressure multigrid hierarchy",
                    })?
                    .solve(b, x, options)
            }
            EquationPreconditioner::Ic0 | EquationPreconditioner::Auto => {
                match IncompleteCholeskyPreconditioner::new(a) {
                    Ok(p) => {
                        pcg_with_preconditioner_and_workspace(a, b, x, &p, options, &mut self.cg)
                    }
                    Err(LinearAlgebraError::IncompleteCholeskyPivot { .. })
                        if control.preconditioner == EquationPreconditioner::Auto =>
                    {
                        if !self.warned_fallback {
                            eprintln!("Pressure Auto: IC0 pivot failed; using Jacobi PCG with unchanged residual tolerances.");
                            self.warned_fallback = true;
                        }
                        pcg_with_preconditioner_and_workspace(
                            a,
                            b,
                            x,
                            &JacobiPreconditioner::new(a)?,
                            options,
                            &mut self.cg,
                        )
                    }
                    Err(e) => Err(e),
                }
            }
            EquationPreconditioner::Jacobi => pcg_with_preconditioner_and_workspace(
                a,
                b,
                x,
                &JacobiPreconditioner::new(a)?,
                options,
                &mut self.cg,
            ),
            EquationPreconditioner::None => cg(a, b, x, options),
        }
    }
}
