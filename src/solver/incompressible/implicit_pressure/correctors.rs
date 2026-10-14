use super::*;

impl IncompressibleSolver {
    pub(super) fn apply_implicit_pressure_correctors(
        &mut self,
        workspace: &mut ImplicitAssemblyWorkspace,
        u_pressure_diagonal: &[f64],
        v_pressure_diagonal: &[f64],
        pressure_correctors: usize,
        alpha_p: f64,
        transient: bool,
        transient_pressure_outlet: bool,
        solver_options: LinearSolverOptions,
        reference: Option<(usize, usize)>,
        linear_solver: &mut ImplicitPressureSolver,
    ) -> Result<(f64, usize), String> {
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let rho = self.cfg.case.density();
        let mut final_pressure_residual = 0.0;
        let mut total_pressure_iterations = 0;
        for corrector in 0..pressure_correctors {
            if transient && corrector > 0 {
                self.rebuild_transient_pressure_predictor(
                    workspace,
                    u_pressure_diagonal,
                    v_pressure_diagonal,
                )?;
            }
            let pressure_options = if transient {
                LinearSolverOptions {
                    absolute_tolerance: self.cfg.pressure_tolerance,
                    relative_tolerance: if corrector + 1 == pressure_correctors {
                        0.0
                    } else {
                        self.cfg.pimple.first_pressure_relative_tolerance
                    },
                    ..solver_options
                }
            } else {
                solver_options
            };
            let rhs_profile = ProfileScope::new(ProfileStage::PressureRhs);
            workspace.pressure_rhs.fill(0.0);
            for j in 0..ny {
                for i in 0..nx {
                    if self.solid[(i, j)]
                        || (!transient_pressure_outlet && self.is_pressure_outlet_cell(i, j))
                        || reference == Some((i, j))
                    {
                        continue;
                    }
                    let divergence = (self.u_star.as_slice()[i + 1 + (nx + 1) * j]
                        - self.u_star.as_slice()[i + (nx + 1) * j])
                        / self.grid.dx
                        + (self.v_star.as_slice()[i + nx * (j + 1)]
                            - self.v_star.as_slice()[i + nx * j])
                            / self.grid.dy;
                    workspace.pressure_rhs[i + nx * j] = -divergence;
                }
            }
            drop(rhs_profile);
            workspace.pressure_correction.fill(0.0);
            let pressure_solve_profile = ProfileScope::new(ProfileStage::PressureSolve);
            let pressure_report = match linear_solver {
                ImplicitPressureSolver::Gamg(solver) => solver.solve(
                    &workspace.pressure_rhs,
                    &mut workspace.pressure_correction,
                    pressure_options,
                ),
                ImplicitPressureSolver::Pcg(preconditioner) => {
                    pcg_with_preconditioner_and_workspace(
                        &workspace.pressure_matrix,
                        &workspace.pressure_rhs,
                        &mut workspace.pressure_correction,
                        preconditioner,
                        pressure_options,
                        &mut self.pressure_cg_workspace,
                    )
                }
            }
            .map_err(|e| e.to_string())?;
            drop(pressure_solve_profile);
            if !pressure_report.converged() {
                return Err(format!(
                "Coefficient-based pressure-correction solve did not converge: residual {:.3e} after {} iterations",
                pressure_report.final_residual, pressure_report.iterations
            ));
            }
            final_pressure_residual = pressure_report.final_residual;
            total_pressure_iterations += pressure_report.iterations;

            let pressure_correction_profile = ProfileScope::new(ProfileStage::PressureCorrection);
            for j in 0..ny {
                for i in 1..=nx {
                    let face = i + (nx + 1) * j;
                    if i == nx {
                        if transient_pressure_outlet && u_face_open(&self.solid, nx, i, j) {
                            self.u_star.as_mut_slice()[face] +=
                                2.0 * self.grid.dy * workspace.pressure_correction[nx - 1 + nx * j]
                                    / (rho * u_pressure_diagonal[face]);
                        }
                        continue;
                    }
                    if u_face_open(&self.solid, nx, i, j) {
                        self.u_star.as_mut_slice()[face] -= self.grid.dy
                            * (workspace.pressure_correction[i + nx * j]
                                - workspace.pressure_correction[i - 1 + nx * j])
                            / (rho * u_pressure_diagonal[face]);
                    }
                }
            }
            for j in 1..ny {
                for i in 0..nx {
                    let face = i + nx * j;
                    if v_face_open(&self.solid, ny, i, j) {
                        self.v_star.as_mut_slice()[face] -= self.grid.dx
                            * (workspace.pressure_correction[i + nx * j]
                                - workspace.pressure_correction[i + nx * (j - 1)])
                            / (rho * v_pressure_diagonal[face]);
                    }
                }
            }
            for (pressure, dp) in self
                .p
                .as_mut_slice()
                .iter_mut()
                .zip(workspace.pressure_correction.iter().copied())
            {
                *pressure += alpha_p * dp;
            }
            drop(pressure_correction_profile);
            self.u
                .as_mut_slice()
                .copy_from_slice(self.u_star.as_slice());
            self.v
                .as_mut_slice()
                .copy_from_slice(self.v_star.as_slice());
        }
        Ok((final_pressure_residual, total_pressure_iterations))
    }
}
