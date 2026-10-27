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
    /// Bounded sparse LDLT for difficult pressure systems, then IC0 or Jacobi.
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
    pub fn transport(options: LinearSolverOptions) -> Self {
        Self {
            preconditioner: EquationPreconditioner::Jacobi,
            ..Self::from_options(options)
        }
    }
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
        self.solve_with_workspace(a, b, x, pressure, None)
    }
    pub(crate) fn solve_with_workspace(
        self,
        a: &CsrMatrix,
        b: &[f64],
        x: &mut [f64],
        pressure: bool,
        workspace: Option<&mut BiCgStabWorkspace>,
    ) -> Result<LinearSolveReport, LinearAlgebraError> {
        self.validate(pressure)
            .map_err(|_| LinearAlgebraError::Breakdown {
                reason: "invalid equation solver controls",
            })?;
        crate::linear::validate_solve_inputs(a, b, x, self.options())?;
        if !(pressure
            && matches!(
                self.preconditioner,
                EquationPreconditioner::Gamg | EquationPreconditioner::Auto
            ))
        {
            if let Some(result) = crate::compute::try_gpu(a, b, x, self.options()) {
                return result;
            }
        }
        let _cpu = if !(pressure
            && matches!(
                self.preconditioner,
                EquationPreconditioner::Gamg | EquationPreconditioner::Auto
            )) {
            Some(crate::compute::CpuLinearScope::enter())
        } else {
            None
        };
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
            (false, EquationPreconditioner::None) => match workspace {
                Some(workspace) => bicgstab_with_workspace(a, b, x, o, workspace),
                None => bicgstab(a, b, x, o),
            },
            (false, EquationPreconditioner::Jacobi) => {
                let preconditioner = JacobiPreconditioner::new(a)?;
                match workspace {
                    Some(workspace) => bicgstab_with_preconditioner_and_workspace(
                        a,
                        b,
                        x,
                        &preconditioner,
                        o,
                        workspace,
                    ),
                    None => bicgstab_with_preconditioner(a, b, x, &preconditioner, o),
                }
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
                preconditioner: EquationPreconditioner::Auto,
                ..Default::default()
            },
            velocity: EquationControl::transport(LinearSolverOptions::default()),
            temperature: EquationControl::transport(LinearSolverOptions::default()),
            k: EquationControl::transport(LinearSolverOptions::default()),
            omega: EquationControl::transport(LinearSolverOptions::default()),
            epsilon: EquationControl::transport(LinearSolverOptions::default()),
            nu_tilda: EquationControl::transport(LinearSolverOptions::default()),
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
    direct: Option<crate::linear::direct::PressureDirect>,
    direct_unavailable: bool,
    multigrid: Option<AlgebraicMultigrid>,
    ic0: Option<(
        CsrMatrix,
        Result<IncompleteCholeskyPreconditioner, LinearAlgebraError>,
    )>,
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
        crate::linear::validate_solve_inputs(a, b, x, options)?;
        if let Some(result) = crate::compute::try_gpu(a, b, x, options) {
            return result;
        }
        let _cpu = crate::compute::CpuLinearScope::enter();
        if control.preconditioner == EquationPreconditioner::Auto
            && a.nrows() >= 4096
            && !self.direct_unavailable
        {
            if self
                .direct
                .as_ref()
                .is_none_or(|solver| !solver.matches_pattern(a))
            {
                match crate::linear::direct::PressureDirect::new(a) {
                    Ok(Some(solver)) => self.direct = Some(solver),
                    Ok(None) => self.direct_unavailable = true,
                    Err(error) => {
                        eprintln!("Pressure Auto: sparse LDLT unavailable ({error}); using IC0.");
                        self.direct_unavailable = true;
                    }
                }
            }
            if !self.direct_unavailable {
                if let Some(solver) = &mut self.direct {
                    return solver.solve(a, b, x, options);
                }
            }
        }
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
                // Non-orthogonal passes change the RHS, not the matrix. Reuse
                // factors only after exact topology AND coefficient equality.
                if self.ic0.as_ref().is_none_or(|(matrix, _)| matrix != a) {
                    self.ic0 = Some((a.clone(), IncompleteCholeskyPreconditioner::new(a)));
                }
                match &self
                    .ic0
                    .as_ref()
                    .ok_or(LinearAlgebraError::Breakdown {
                        reason: "missing cached pressure factor",
                    })?
                    .1
                {
                    Ok(p) => {
                        pcg_with_preconditioner_and_workspace(a, b, x, p, options, &mut self.cg)
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
                    Err(e) => Err(e.clone()),
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
            EquationPreconditioner::None => cg_with_workspace(a, b, x, options, &mut self.cg),
        }
    }
}
