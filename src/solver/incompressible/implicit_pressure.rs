use super::*;

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
        let rho = self.cfg.case.density();
        let mut final_pressure_residual = 0.0;
        let mut total_pressure_iterations = 0;
        for corrector in 0..pressure_correctors {
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
            workspace.pressure_builder.clear();
            let pressure_builder = &mut workspace.pressure_builder;
            let mut pressure_rhs = vec![0.0; nx * ny];
            let reference = self
                .pressure_reference_required()
                .then(|| self.find_reference_cell())
                .flatten();
            for j in 0..ny {
                for i in 0..nx {
                    let row = i + nx * j;
                    if self.solid[(i, j)]
                        || (!transient_pressure_outlet && self.is_pressure_outlet_cell(i, j))
                        || reference == Some((i, j))
                    {
                        pressure_builder
                            .add(row, row, 1.0)
                            .map_err(|e| e.to_string())?;
                        continue;
                    }
                    let divergence = (self.u_star.as_slice()[i + 1 + (nx + 1) * j]
                        - self.u_star.as_slice()[i + (nx + 1) * j])
                        / self.grid.dx
                        + (self.v_star.as_slice()[i + nx * (j + 1)]
                            - self.v_star.as_slice()[i + nx * j])
                            / self.grid.dy;
                    pressure_rhs[row] = -divergence;
                    let neighbors = [
                        (
                            i + 1 < nx,
                            (i + 1).min(nx - 1),
                            j,
                            self.grid.dy / (rho * self.grid.dx),
                            1.0 / u_pressure_diagonal[i + 1 + (nx + 1) * j],
                        ),
                        (
                            i > 0,
                            i.saturating_sub(1),
                            j,
                            self.grid.dy / (rho * self.grid.dx),
                            1.0 / u_pressure_diagonal[i + (nx + 1) * j],
                        ),
                        (
                            j + 1 < ny,
                            i,
                            (j + 1).min(ny - 1),
                            self.grid.dx / (rho * self.grid.dy),
                            1.0 / v_pressure_diagonal[i + nx * (j + 1)],
                        ),
                        (
                            j > 0,
                            i,
                            j.saturating_sub(1),
                            self.grid.dx / (rho * self.grid.dy),
                            1.0 / v_pressure_diagonal[i + nx * j],
                        ),
                    ];
                    let mut diagonal = 0.0;
                    for (exists, ni, nj, metric, reciprocal) in neighbors {
                        if !exists || self.solid[(ni, nj)] {
                            continue;
                        }
                        let coefficient = metric * reciprocal;
                        diagonal += coefficient;
                        let neighbor = ni + nx * nj;
                        if (transient_pressure_outlet || !self.is_pressure_outlet_cell(ni, nj))
                            && reference != Some((ni, nj))
                        {
                            pressure_builder
                                .add(row, neighbor, -coefficient)
                                .map_err(|e| e.to_string())?;
                        }
                    }
                    if transient_pressure_outlet && i + 1 == nx {
                        // Dirichlet pressure is prescribed on the outlet face,
                        // half a cell from the last pressure unknown.
                        diagonal += 2.0 * self.grid.dy
                            / (rho * self.grid.dx * u_pressure_diagonal[nx + (nx + 1) * j]);
                    }
                    pressure_builder
                        .add(row, row, diagonal.max(f64::MIN_POSITIVE))
                        .map_err(|e| e.to_string())?;
                }
            }
            workspace
                .pressure_builder
                .finalize_into(&mut workspace.pressure_matrix)
                .map_err(|e| e.to_string())?;
            let pressure_matrix = &workspace.pressure_matrix;
            let mut correction = vec![0.0; nx * ny];
            let pressure_report = match self.cfg.pressure_solver {
                PressureSolverKind::PcgDic => {
                    let preconditioner = IncompleteCholeskyPreconditioner::new(&pressure_matrix)
                        .map_err(|error| format!("cannot build pressure DIC(0): {error}"))?;
                    pcg_with_preconditioner(
                        &pressure_matrix,
                        &pressure_rhs,
                        &mut correction,
                        &preconditioner,
                        pressure_options,
                    )
                }
                PressureSolverKind::PcgMg => {
                    let preconditioner = match self.multigrid_preconditioner.take() {
                        Some(mut preconditioner) => {
                            if let Err(error) = preconditioner.update_values(&pressure_matrix) {
                                self.multigrid_preconditioner = Some(preconditioner);
                                return Err(format!("cannot update pressure multigrid: {error}"));
                            }
                            preconditioner
                        }
                        None => StructuredMultigridPreconditioner::new(&pressure_matrix, nx, ny)
                            .map_err(|error| format!("cannot build pressure multigrid: {error}"))?,
                    };
                    let result = pcg_with_preconditioner(
                        &pressure_matrix,
                        &pressure_rhs,
                        &mut correction,
                        &preconditioner,
                        pressure_options,
                    );
                    self.multigrid_preconditioner = Some(preconditioner);
                    result
                }
                PressureSolverKind::Pcg | PressureSolverKind::Sor => {
                    let preconditioner =
                        JacobiPreconditioner::new(&pressure_matrix).map_err(|e| e.to_string())?;
                    pcg(
                        &pressure_matrix,
                        &pressure_rhs,
                        &mut correction,
                        &preconditioner,
                        pressure_options,
                    )
                }
            }
            .map_err(|e| e.to_string())?;
            if !pressure_report.converged() {
                return Err(format!(
                "Coefficient-based pressure-correction solve did not converge: residual {:.3e} after {} iterations",
                pressure_report.final_residual, pressure_report.iterations
            ));
            }
            final_pressure_residual = pressure_report.final_residual;
            total_pressure_iterations += pressure_report.iterations;

            for j in 0..ny {
                for i in 1..=nx {
                    let face = i + (nx + 1) * j;
                    if i == nx {
                        if transient_pressure_outlet && u_face_open(&self.solid, nx, i, j) {
                            self.u_star.as_mut_slice()[face] +=
                                2.0 * self.grid.dy * correction[nx - 1 + nx * j]
                                    / (rho * u_pressure_diagonal[face]);
                        }
                        continue;
                    }
                    if u_face_open(&self.solid, nx, i, j) {
                        self.u_star.as_mut_slice()[face] -= self.grid.dy
                            * (correction[i + nx * j] - correction[i - 1 + nx * j])
                            / (rho * u_pressure_diagonal[face]);
                    }
                }
            }
            for j in 1..ny {
                for i in 0..nx {
                    let face = i + nx * j;
                    if v_face_open(&self.solid, ny, i, j) {
                        self.v_star.as_mut_slice()[face] -= self.grid.dx
                            * (correction[i + nx * j] - correction[i + nx * (j - 1)])
                            / (rho * v_pressure_diagonal[face]);
                    }
                }
            }
            for (pressure, dp) in self.p.as_mut_slice().iter_mut().zip(correction) {
                *pressure += alpha_p * dp;
            }
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
