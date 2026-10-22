use super::*;

mod assembly;
mod correctors;
mod predictor;

/// Owned setup reused by every corrector for one fixed momentum matrix.
enum ImplicitPressurePreconditioner {
    Dic(IncompleteCholeskyPreconditioner),
    Mg(StructuredMultigridPreconditioner),
    Jacobi(JacobiPreconditioner),
}

enum ImplicitPressureSolver {
    Pcg(ImplicitPressurePreconditioner),
    Gamg(Box<AlgebraicMultigrid>),
}

impl crate::Preconditioner for ImplicitPressurePreconditioner {
    fn len(&self) -> usize {
        match self {
            Self::Dic(p) => crate::Preconditioner::len(p),
            Self::Mg(p) => crate::Preconditioner::len(p),
            Self::Jacobi(p) => crate::Preconditioner::len(p),
        }
    }

    fn apply_into(&self, rhs: &[f64], output: &mut [f64]) -> Result<(), crate::LinearAlgebraError> {
        match self {
            Self::Dic(p) => p.apply_into(rhs, output),
            Self::Mg(p) => p.apply_into(rhs, output),
            Self::Jacobi(p) => p.apply_into(rhs, output),
        }
    }
}

impl IncompressibleSolver {
    pub(super) fn correct_implicit_pressure(
        &mut self,
        workspace: &mut ImplicitAssemblyWorkspace,
        u_pressure_diagonal: &[f64],
        v_pressure_diagonal: &[f64],
        pressure_correctors: usize,
        alpha_p: f64,
        transient: bool,
        transient_pressure_outlet: bool,
        solver_options: LinearSolverOptions,
    ) -> Result<(f64, usize), String> {
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let reference = self.assemble_implicit_pressure_matrix(
            workspace,
            u_pressure_diagonal,
            v_pressure_diagonal,
            transient_pressure_outlet,
        )?;
        let setup_profile = ProfileScope::new(ProfileStage::PressurePreconditionerSetup);
        let mut pressure_solver = match self.cfg.pressure_solver {
            PressureSolverKind::Gamg => {
                let solver = match self.algebraic_multigrid.take() {
                    Some(mut solver) => {
                        if let Err(error) = solver.update_values(&workspace.pressure_matrix) {
                            self.algebraic_multigrid = Some(solver);
                            return Err(format!("cannot update algebraic multigrid: {error}"));
                        }
                        solver
                    }
                    None => {
                        // Pressure has Cartesian cell unknowns. Geometric face
                        // strength keeps aggregation independent of the initial
                        // transient momentum coefficients and favors compact
                        // coarse cells as shared face areas are accumulated.
                        let matrix = &workspace.pressure_matrix;
                        let mut weights = vec![0.0; matrix.nnz()];
                        for row in 0..matrix.nrows() {
                            for (entry, weight) in weights
                                .iter_mut()
                                .enumerate()
                                .take(matrix.row_offsets()[row + 1])
                                .skip(matrix.row_offsets()[row])
                            {
                                let column = matrix.column_indices()[entry];
                                if column != row {
                                    *weight = if column.abs_diff(row) == nx {
                                        self.grid.dx
                                    } else {
                                        self.grid.dy
                                    };
                                }
                            }
                        }
                        AlgebraicMultigrid::new_with_face_weights(
                            matrix,
                            &weights,
                            self.algebraic_multigrid_options,
                        )
                        .map_err(|error| format!("cannot build algebraic multigrid: {error}"))?
                    }
                };
                ImplicitPressureSolver::Gamg(Box::new(solver))
            }
            PressureSolverKind::PcgDic => {
                ImplicitPressureSolver::Pcg(ImplicitPressurePreconditioner::Dic(
                    IncompleteCholeskyPreconditioner::new(&workspace.pressure_matrix)
                        .map_err(|error| format!("cannot build pressure DIC(0): {error}"))?,
                ))
            }
            PressureSolverKind::PcgMg => {
                let mg = match self.multigrid_preconditioner.take() {
                    Some(mut mg) => {
                        if let Err(error) = mg.update_values(&workspace.pressure_matrix) {
                            self.multigrid_preconditioner = Some(mg);
                            return Err(format!("cannot update pressure multigrid: {error}"));
                        }
                        mg
                    }
                    None => StructuredMultigridPreconditioner::with_smoothing_steps(
                        &workspace.pressure_matrix,
                        nx,
                        ny,
                        self.multigrid_smoothing_steps,
                    )
                    .map_err(|error| format!("cannot build pressure multigrid: {error}"))?,
                };
                ImplicitPressureSolver::Pcg(ImplicitPressurePreconditioner::Mg(mg))
            }
            PressureSolverKind::Pcg | PressureSolverKind::Sor => {
                ImplicitPressureSolver::Pcg(ImplicitPressurePreconditioner::Jacobi(
                    JacobiPreconditioner::new(&workspace.pressure_matrix)
                        .map_err(|e| e.to_string())?,
                ))
            }
        };
        drop(setup_profile);
        let result = self.apply_implicit_pressure_correctors(
            workspace,
            u_pressure_diagonal,
            v_pressure_diagonal,
            pressure_correctors,
            alpha_p,
            transient,
            transient_pressure_outlet,
            solver_options,
            reference,
            &mut pressure_solver,
        );
        // Restore the hierarchy even if a linear solve failed.
        match pressure_solver {
            ImplicitPressureSolver::Pcg(ImplicitPressurePreconditioner::Mg(mg)) => {
                self.multigrid_preconditioner = Some(mg)
            }
            ImplicitPressureSolver::Gamg(solver) => self.algebraic_multigrid = Some(*solver),
            _ => {}
        }
        result
    }
}
