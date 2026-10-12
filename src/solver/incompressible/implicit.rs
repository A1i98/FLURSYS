use super::*;

/// Reusable triplet and CSR storage for implicit momentum and pressure assembly.
pub(super) struct ImplicitAssemblyWorkspace {
    u_builder: CsrBuilder,
    v_builder: CsrBuilder,
    pub(super) pressure_builder: CsrBuilder,
    u_matrix: CsrMatrix,
    v_matrix: CsrMatrix,
    pub(super) pressure_matrix: CsrMatrix,
}

impl ImplicitAssemblyWorkspace {
    pub(super) fn new(nx: usize, ny: usize) -> Result<Self, String> {
        let n_u = (nx + 1) * ny;
        let n_v = nx * (ny + 1);
        let n_pressure = nx * ny;
        let empty_matrix = |size| {
            CsrMatrix::new(size, size, vec![0; size + 1], Vec::new(), Vec::new())
                .map_err(|error| error.to_string())
        };
        Ok(Self {
            u_builder: CsrBuilder::with_capacity(n_u, n_u, n_u.saturating_mul(5)),
            v_builder: CsrBuilder::with_capacity(n_v, n_v, n_v.saturating_mul(5)),
            pressure_builder: CsrBuilder::with_capacity(
                n_pressure,
                n_pressure,
                n_pressure.saturating_mul(5),
            ),
            u_matrix: empty_matrix(n_u)?,
            v_matrix: empty_matrix(n_v)?,
            pressure_matrix: empty_matrix(n_pressure)?,
        })
    }
}

impl IncompressibleSolver {
    /// One coefficient-based SIMPLE outer iteration on the native staggered grid.
    /// Momentum is assembled implicitly using the configured convection
    /// scheme; pressure correction uses SIMPLE (rAU) or SIMPLEC (rAtU).
    pub(super) fn advance_implicit_simple(
        &mut self,
        step_dt: f64,
        consistent: bool,
    ) -> Result<(f64, usize, f64), String> {
        self.advance_implicit_coefficient_based(step_dt, consistent, None, 1)
    }

    pub(super) fn advance_implicit_pimple(
        &mut self,
        step_dt: f64,
        old_u: &Field2D,
        old_v: &Field2D,
    ) -> Result<(f64, usize, f64), String> {
        self.advance_implicit_coefficient_based(
            step_dt,
            false,
            Some((old_u.as_slice(), old_v.as_slice())),
            self.cfg.pimple.pressure_correctors,
        )
    }

    pub(super) fn advance_implicit_coefficient_based(
        &mut self,
        step_dt: f64,
        consistent: bool,
        transient_reference: Option<(&[f64], &[f64])>,
        pressure_correctors: usize,
    ) -> Result<(f64, usize, f64), String> {
        let mut workspace = match self.implicit_assembly_workspace.take() {
            Some(workspace) => workspace,
            None => ImplicitAssemblyWorkspace::new(self.grid.nx, self.grid.ny)?,
        };
        let result = self.advance_implicit_coefficient_with_workspace(
            step_dt,
            consistent,
            transient_reference,
            pressure_correctors,
            &mut workspace,
        );
        self.implicit_assembly_workspace = Some(workspace);
        result
    }

    pub(super) fn advance_implicit_coefficient_with_workspace(
        &mut self,
        step_dt: f64,
        consistent: bool,
        transient_reference: Option<(&[f64], &[f64])>,
        pressure_correctors: usize,
        workspace: &mut ImplicitAssemblyWorkspace,
    ) -> Result<(f64, usize, f64), String> {
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let rho = self.cfg.case.density();
        let nu = self.cfg.case.kinematic_viscosity();
        let alpha_u = if transient_reference.is_some() {
            1.0
        } else {
            self.cfg.velocity_relaxation
        };
        let alpha_p = if transient_reference.is_some() {
            1.0
        } else {
            self.cfg.pressure_relaxation
        };
        // The current velocity fields stay unchanged until both momentum
        // systems and the pressure correction are assembled, so borrow them
        // as snapshots instead of cloning both face fields every outer step.
        let u_before = self.u.as_slice();
        let v_before = self.v.as_slice();
        let transient_coefficient = if transient_reference.is_some() {
            self.grid.dx * self.grid.dy / step_dt
        } else {
            0.0
        };
        // Include the laminar deviatoric-stress grad(div(U)) contribution
        // explicitly from the current staggered velocity iterate.
        let mut cell_divergence = vec![0.0; nx * ny];
        for j in 0..ny {
            for i in 0..nx {
                cell_divergence[i + nx * j] =
                    (u_before[i + 1 + (nx + 1) * j] - u_before[i + (nx + 1) * j]) / self.grid.dx
                        + (v_before[i + nx * (j + 1)] - v_before[i + nx * j]) / self.grid.dy;
            }
        }
        let p = self.p.as_slice();
        let n_u = (nx + 1) * ny;
        let n_v = nx * (ny + 1);
        let solver_options = LinearSolverOptions {
            absolute_tolerance: 1.0e-12,
            relative_tolerance: self.cfg.pressure_tolerance,
            max_iterations: self.cfg.pressure_max_iters.max(100),
        };
        let momentum_options = if transient_reference.is_some() {
            LinearSolverOptions {
                absolute_tolerance: self.cfg.pimple.momentum_tolerance,
                relative_tolerance: 0.0,
                ..solver_options
            }
        } else {
            solver_options
        };
        let u_sample = |i: isize, j: isize| {
            (i >= 0 && i <= nx as isize && j >= 0 && j < ny as isize)
                .then_some((i as usize, j as usize))
                .filter(|&(i, j)| u_face_open(&self.solid, nx, i, j))
                .map(|(i, j)| u_before[i + (nx + 1) * j])
        };
        let v_sample = |i: isize, j: isize| {
            (i >= 0 && i < nx as isize && j >= 0 && j <= ny as isize)
                .then_some((i as usize, j as usize))
                .filter(|&(i, j)| v_face_open(&self.solid, ny, i, j))
                .map(|(i, j)| v_before[i + nx * j])
        };

        workspace.u_builder.clear();
        let u_builder = &mut workspace.u_builder;
        let mut u_rhs = vec![0.0; n_u];
        for j in 0..ny {
            for i in 0..=nx {
                let row = i + (nx + 1) * j;
                if i == 0 || i == nx || !u_face_open(&self.solid, nx, i, j) {
                    u_builder.add(row, row, 1.0).map_err(|e| e.to_string())?;
                    u_rhs[row] = u_before[row];
                    continue;
                }
                let up = u_before[row];
                let ue_nb = u_before[row + 1];
                let uw_nb = u_before[row - 1];
                let fe = 0.5 * (up + ue_nb) * self.grid.dy;
                let fw = 0.5 * (uw_nb + up) * self.grid.dy;
                let vn = 0.5
                    * (v_before[i - 1 + nx * (j + 1)] + v_before[i + nx * (j + 1)])
                    * self.grid.dx;
                let vs = 0.5 * (v_before[i - 1 + nx * j] + v_before[i + nx * j]) * self.grid.dx;
                let de = nu * self.grid.dy / self.grid.dx;
                let dn = nu * self.grid.dx / self.grid.dy;
                let (ae, aw, an, as_) = match self.cfg.convection {
                    ConvectionScheme::FirstOrderUpwind => (
                        (-fe).max(0.0) + de,
                        fw.max(0.0) + de,
                        (-vn).max(0.0) + dn,
                        vs.max(0.0) + dn,
                    ),
                    ConvectionScheme::Central => {
                        (de - 0.5 * fe, de + 0.5 * fw, dn - 0.5 * vn, dn + 0.5 * vs)
                    }
                    ConvectionScheme::VanLeerTvd => (
                        (-fe).max(0.0) + de,
                        fw.max(0.0) + de,
                        (-vn).max(0.0) + dn,
                        vs.max(0.0) + dn,
                    ),
                };
                let ap0 = ae + aw + an + as_ + fe - fw + vn - vs;
                let mut ap = ap0 + transient_coefficient;
                let mut boundary_source = 0.0;
                for (neighbor, coeff) in [(row + 1, ae), (row - 1, aw)] {
                    let ni = neighbor % (nx + 1);
                    let nj = neighbor / (nx + 1);
                    if u_face_open(&self.solid, nx, ni, nj) {
                        u_builder
                            .add(row, neighbor, -coeff)
                            .map_err(|e| e.to_string())?;
                    } else {
                        ap += coeff;
                    }
                }
                for (inside, neighbor, coeff, side) in [
                    (j + 1 < ny, row + nx + 1, an, Side::Top),
                    (j > 0, row.saturating_sub(nx + 1), as_, Side::Bottom),
                ] {
                    if inside
                        && u_face_open(&self.solid, nx, neighbor % (nx + 1), neighbor / (nx + 1))
                    {
                        u_builder
                            .add(row, neighbor, -coeff)
                            .map_err(|e| e.to_string())?;
                    } else {
                        ap += coeff;
                        if !inside {
                            let x = self.grid.u_face_x(i);
                            let y = if side == Side::Top {
                                self.grid.height
                            } else {
                                0.0
                            };
                            let wall_u = self
                                .cfg
                                .boundary_overrides
                                .velocity(&self.cfg.case, side, x, y, self.time)
                                .0;
                            boundary_source += 2.0 * coeff * wall_u;
                        }
                    }
                }
                u_builder
                    .add(row, row, ap / alpha_u)
                    .map_err(|e| e.to_string())?;
                let pressure_gradient = (p[i + nx * j] - p[i - 1 + nx * j]) / self.grid.dx;
                let buoyancy = match (self.cfg.physics.buoyancy, self.temperature.as_ref()) {
                    (
                        BuoyancyModel::Boussinesq {
                            thermal_expansion,
                            gravity_x,
                            reference_temperature,
                            ..
                        },
                        Some(t),
                    ) => {
                        gravity_x
                            * thermal_expansion
                            * (0.5 * (t[(i - 1, j)] + t[(i, j)]) - reference_temperature)
                    }
                    _ => 0.0,
                };
                let grad_div_u =
                    (cell_divergence[i + nx * j] - cell_divergence[i - 1 + nx * j]) / self.grid.dx;
                let convection_correction = if self.cfg.convection == ConvectionScheme::VanLeerTvd {
                    limited_face_correction(
                        fe,
                        up,
                        ue_nb,
                        u_sample(i as isize - 1, j as isize),
                        u_sample(i as isize + 2, j as isize),
                    ) - limited_face_correction(
                        fw,
                        uw_nb,
                        up,
                        u_sample(i as isize - 2, j as isize),
                        u_sample(i as isize + 1, j as isize),
                    ) + limited_face_correction(
                        vn,
                        up,
                        if j + 1 < ny {
                            u_before[row + nx + 1]
                        } else {
                            up
                        },
                        u_sample(i as isize, j as isize - 1),
                        u_sample(i as isize, j as isize + 2),
                    ) - limited_face_correction(
                        vs,
                        if j > 0 { u_before[row - nx - 1] } else { up },
                        up,
                        u_sample(i as isize, j as isize - 2),
                        u_sample(i as isize, j as isize + 1),
                    )
                } else {
                    0.0
                };
                u_rhs[row] = -pressure_gradient * self.grid.dx * self.grid.dy / rho
                    + (nu / 3.0) * grad_div_u * self.grid.dx * self.grid.dy
                    + buoyancy * self.grid.dx * self.grid.dy
                    + boundary_source
                    - convection_correction
                    + transient_coefficient
                        * transient_reference.map_or(0.0, |(old_u, _)| old_u[row])
                    + ((1.0 - alpha_u) / alpha_u) * ap * up;
            }
        }
        workspace
            .u_builder
            .finalize_into(&mut workspace.u_matrix)
            .map_err(|e| e.to_string())?;
        let u_matrix = &workspace.u_matrix;
        let u_diagonal = u_matrix.diagonal().map_err(|e| e.to_string())?;
        let mut u_pressure_diagonal =
            Self::pressure_coupling_diagonal(&u_matrix, &u_diagonal, consistent);
        let transient_pressure_outlet = transient_reference.is_some()
            && matches!(
                self.cfg
                    .boundary_overrides
                    .kind(&self.cfg.case, Side::Right),
                BoundaryKind::PressureOutlet { .. }
            );
        if transient_pressure_outlet {
            for j in 0..ny {
                let boundary_face = nx + (nx + 1) * j;
                u_pressure_diagonal[boundary_face] = u_pressure_diagonal[boundary_face - 1];
            }
        }
        // Reuse the solver's persistent face-field storage for the momentum
        // solution. Cloning `u_before` here allocated another full field on
        // every implicit iteration even though `u_star` already has the right
        // size and is reused by the other coupling modes.
        self.u_star.as_mut_slice().copy_from_slice(&u_before);
        let u_preconditioner = SymmetricGaussSeidelPreconditioner::new(&u_matrix)
            .map_err(|e| format!("cannot build u-momentum Gauss-Seidel preconditioner: {e}"))?;
        let u_report = bicgstab_with_preconditioner(
            &u_matrix,
            &u_rhs,
            self.u_star.as_mut_slice(),
            &u_preconditioner,
            momentum_options,
        )
        .map_err(|e| e.to_string())?;
        if !u_report.converged() {
            return Err(format!(
                "Implicit u-momentum solve did not converge: residual {:.3e} after {} iterations",
                u_report.final_residual, u_report.iterations
            ));
        }
        let u_equation_residual = normalized_linear_residual(u_report.final_residual, &u_rhs);

        workspace.v_builder.clear();
        let v_builder = &mut workspace.v_builder;
        let mut v_rhs = vec![0.0; n_v];
        for j in 0..=ny {
            for i in 0..nx {
                let row = i + nx * j;
                if j == 0 || j == ny || !v_face_open(&self.solid, ny, i, j) {
                    v_builder.add(row, row, 1.0).map_err(|e| e.to_string())?;
                    v_rhs[row] = v_before[row];
                    continue;
                }
                let vp = v_before[row];
                let vn_nb = v_before[row + nx];
                let vs_nb = v_before[row - nx];
                let ue = 0.5
                    * (u_before[i + 1 + (nx + 1) * (j - 1)] + u_before[i + 1 + (nx + 1) * j])
                    * self.grid.dy;
                let uw = 0.5
                    * (u_before[i + (nx + 1) * (j - 1)] + u_before[i + (nx + 1) * j])
                    * self.grid.dy;
                let fn_ = 0.5 * (vp + vn_nb) * self.grid.dx;
                let fs = 0.5 * (vs_nb + vp) * self.grid.dx;
                let de = nu * self.grid.dy / self.grid.dx;
                let dn = nu * self.grid.dx / self.grid.dy;
                let (ae, aw, an, as_) = match self.cfg.convection {
                    ConvectionScheme::FirstOrderUpwind => (
                        (-ue).max(0.0) + de,
                        uw.max(0.0) + de,
                        (-fn_).max(0.0) + dn,
                        fs.max(0.0) + dn,
                    ),
                    ConvectionScheme::Central => {
                        (de - 0.5 * ue, de + 0.5 * uw, dn - 0.5 * fn_, dn + 0.5 * fs)
                    }
                    ConvectionScheme::VanLeerTvd => (
                        (-ue).max(0.0) + de,
                        uw.max(0.0) + de,
                        (-fn_).max(0.0) + dn,
                        fs.max(0.0) + dn,
                    ),
                };
                let ap0 = ae + aw + an + as_ + ue - uw + fn_ - fs;
                let mut ap = ap0 + transient_coefficient;
                let mut boundary_source = 0.0;
                for (inside, neighbor, coeff, side) in [
                    (i + 1 < nx, row + 1, ae, Side::Right),
                    (i > 0, row.saturating_sub(1), aw, Side::Left),
                ] {
                    if inside && v_face_open(&self.solid, ny, neighbor % nx, neighbor / nx) {
                        v_builder
                            .add(row, neighbor, -coeff)
                            .map_err(|e| e.to_string())?;
                    } else {
                        ap += coeff;
                        if !inside {
                            let x = if side == Side::Right {
                                self.grid.length
                            } else {
                                0.0
                            };
                            let wall_v = self
                                .cfg
                                .boundary_overrides
                                .velocity(&self.cfg.case, side, x, self.grid.v_face_y(j), self.time)
                                .1;
                            boundary_source += 2.0 * coeff * wall_v;
                        }
                    }
                }
                for (neighbor, coeff) in [(row + nx, an), (row - nx, as_)] {
                    if v_face_open(&self.solid, ny, neighbor % nx, neighbor / nx) {
                        v_builder
                            .add(row, neighbor, -coeff)
                            .map_err(|e| e.to_string())?;
                    } else {
                        ap += coeff;
                    }
                }
                v_builder
                    .add(row, row, ap / alpha_u)
                    .map_err(|e| e.to_string())?;
                let pressure_gradient = (p[i + nx * j] - p[i + nx * (j - 1)]) / self.grid.dy;
                let buoyancy = match (self.cfg.physics.buoyancy, self.temperature.as_ref()) {
                    (
                        BuoyancyModel::Boussinesq {
                            thermal_expansion,
                            gravity_y,
                            reference_temperature,
                            ..
                        },
                        Some(t),
                    ) => {
                        gravity_y
                            * thermal_expansion
                            * (0.5 * (t[(i, j - 1)] + t[(i, j)]) - reference_temperature)
                    }
                    _ => 0.0,
                };
                let grad_div_v = (cell_divergence[i + nx * j] - cell_divergence[i + nx * (j - 1)])
                    / self.grid.dy;
                let convection_correction = if self.cfg.convection == ConvectionScheme::VanLeerTvd {
                    limited_face_correction(
                        ue,
                        vp,
                        v_sample(i as isize + 1, j as isize).unwrap_or(vp),
                        v_sample(i as isize - 1, j as isize),
                        v_sample(i as isize + 2, j as isize),
                    ) - limited_face_correction(
                        uw,
                        if i > 0 { v_before[row - 1] } else { vp },
                        vp,
                        v_sample(i as isize - 2, j as isize),
                        v_sample(i as isize + 1, j as isize),
                    ) + limited_face_correction(
                        fn_,
                        vp,
                        vn_nb,
                        v_sample(i as isize, j as isize - 1),
                        v_sample(i as isize, j as isize + 2),
                    ) - limited_face_correction(
                        fs,
                        vs_nb,
                        vp,
                        v_sample(i as isize, j as isize - 2),
                        v_sample(i as isize, j as isize + 1),
                    )
                } else {
                    0.0
                };
                v_rhs[row] = -pressure_gradient * self.grid.dx * self.grid.dy / rho
                    + (nu / 3.0) * grad_div_v * self.grid.dx * self.grid.dy
                    + buoyancy * self.grid.dx * self.grid.dy
                    + boundary_source
                    - convection_correction
                    + transient_coefficient
                        * transient_reference.map_or(0.0, |(_, old_v)| old_v[row])
                    + ((1.0 - alpha_u) / alpha_u) * ap * vp;
            }
        }
        workspace
            .v_builder
            .finalize_into(&mut workspace.v_matrix)
            .map_err(|e| e.to_string())?;
        let v_matrix = &workspace.v_matrix;
        let v_diagonal = v_matrix.diagonal().map_err(|e| e.to_string())?;
        let v_pressure_diagonal =
            Self::pressure_coupling_diagonal(&v_matrix, &v_diagonal, consistent);
        // Match the u path: keep the BiCGStab iterate in the persistent face
        // field rather than allocating a fresh vector for each outer step.
        self.v_star.as_mut_slice().copy_from_slice(&v_before);
        let v_preconditioner = SymmetricGaussSeidelPreconditioner::new(&v_matrix)
            .map_err(|e| format!("cannot build v-momentum Gauss-Seidel preconditioner: {e}"))?;
        let v_report = bicgstab_with_preconditioner(
            &v_matrix,
            &v_rhs,
            self.v_star.as_mut_slice(),
            &v_preconditioner,
            momentum_options,
        )
        .map_err(|e| e.to_string())?;
        if !v_report.converged() {
            return Err(format!(
                "Implicit v-momentum solve did not converge: residual {:.3e} after {} iterations",
                v_report.final_residual, v_report.iterations
            ));
        }
        let v_equation_residual = normalized_linear_residual(v_report.final_residual, &v_rhs);

        let (pressure_residual, pressure_iterations) = self.correct_implicit_pressure(
            workspace,
            &u_pressure_diagonal,
            &v_pressure_diagonal,
            pressure_correctors,
            alpha_p,
            transient_reference.is_some(),
            transient_pressure_outlet,
            solver_options,
        )?;
        Ok((
            pressure_residual,
            pressure_iterations,
            u_equation_residual.max(v_equation_residual),
        ))
    }

    pub(super) fn pressure_coupling_diagonal(
        matrix: &crate::linear::CsrMatrix,
        diagonal: &[f64],
        consistent: bool,
    ) -> Vec<f64> {
        if !consistent {
            return diagonal.to_vec();
        }

        let rows = matrix.row_offsets();
        let columns = matrix.column_indices();
        let values = matrix.values();
        diagonal
            .iter()
            .enumerate()
            .map(|(row, &a_p)| {
                // Consistent coupling uses rAtU = 1/(1/rAU - H1). For this
                // CSR convention H1 = -sum(A_PN), hence denominator aP+sum(A_PN).
                let off_diagonal_sum: f64 = (rows[row]..rows[row + 1])
                    .filter(|&entry| columns[entry] != row)
                    .map(|entry| values[entry])
                    .sum();
                let consistent_a_p = a_p + off_diagonal_sum;
                if consistent_a_p.is_finite() && consistent_a_p > f64::EPSILON * a_p.abs().max(1.0)
                {
                    consistent_a_p
                } else {
                    // Central differencing or unusual boundary fluxes can make
                    // SIMPLEC's diagonal non-positive; retain SIMPLE locally.
                    a_p
                }
            })
            .collect()
    }

    pub fn field_update(&self) -> FieldUpdate {
        let mut velocity_x = Vec::with_capacity(self.grid.nx * self.grid.ny);
        let mut velocity_y = Vec::with_capacity(self.grid.nx * self.grid.ny);
        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                velocity_x.push(0.5 * (self.u[(i, j)] + self.u[(i + 1, j)]));
                velocity_y.push(0.5 * (self.v[(i, j)] + self.v[(i, j + 1)]));
            }
        }
        FieldUpdate {
            nx: self.grid.nx,
            ny: self.grid.ny,
            pressure: self.p.as_slice().to_vec(),
            speed: self.speed.as_slice().to_vec(),
            velocity_x,
            velocity_y,
            vorticity: self.vorticity.as_slice().to_vec(),
            solid: self.solid.as_slice().to_vec(),
            temperature: self
                .temperature
                .as_ref()
                .map(|field| field.as_slice().to_vec()),
        }
    }
}

pub(super) fn normalized_linear_residual(initial_residual: f64, rhs: &[f64]) -> f64 {
    let rhs_norm = rhs.iter().map(|value| value * value).sum::<f64>().sqrt();
    if rhs_norm > f64::MIN_POSITIVE {
        initial_residual / rhs_norm
    } else if initial_residual <= f64::MIN_POSITIVE {
        0.0
    } else {
        f64::MAX
    }
}

pub(super) fn limited_face_correction(
    flux: f64,
    left: f64,
    right: f64,
    far_left: Option<f64>,
    far_right: Option<f64>,
) -> f64 {
    if flux == 0.0 {
        return 0.0;
    }
    let jump = right - left;
    if jump.abs() <= 1.0e-30 {
        return 0.0;
    }
    let ratio = if flux > 0.0 {
        let Some(upstream) = far_left else {
            return 0.0;
        };
        (left - upstream) / jump
    } else {
        let Some(upstream) = far_right else {
            return 0.0;
        };
        (right - upstream) / -jump
    };
    if !ratio.is_finite() || ratio <= 0.0 {
        return 0.0;
    }
    let limiter = 2.0 * ratio / (1.0 + ratio);
    let face_correction = if flux > 0.0 {
        0.5 * limiter * jump
    } else {
        -0.5 * limiter * jump
    };
    flux * face_correction
}
