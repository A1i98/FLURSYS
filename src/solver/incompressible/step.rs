use super::*;

impl IncompressibleSolver {
    pub(super) fn initialize_velocity(&mut self) {
        for j in 0..self.grid.ny {
            for i in 0..=self.grid.nx {
                let x = self.grid.u_face_x(i);
                let y = self.grid.u_face_y(j);
                let mut value = self.cfg.case.initial_velocity(x, y).0;
                if !self.u_face_open(i, j) {
                    value = 0.0;
                }
                self.u[(i, j)] = value;
            }
        }
        for j in 0..=self.grid.ny {
            for i in 0..self.grid.nx {
                let x = self.grid.v_face_x(i);
                let y = self.grid.v_face_y(j);
                let mut value = self.cfg.case.initial_velocity(x, y).1;
                if !self.v_face_open(i, j) {
                    value = 0.0;
                }
                self.v[(i, j)] = value;
            }
        }
        apply_velocity_boundaries(
            &self.cfg.case,
            &self.cfg.boundary_overrides,
            &self.grid,
            &self.solid,
            self.time,
            &mut self.u,
            &mut self.v,
        );
    }

    pub(super) fn advance_one_step(&mut self) -> Result<Diagnostics, String> {
        let _profile = ProfileScope::new(ProfileStage::Step);
        if self.reached_end_time() {
            return Err("projection simulation has already reached t_end".to_string());
        }

        let mut step_dt = self.cfg.dt;
        if self.cfg.time_step.adaptive {
            let thermal_diffusivity = (self.cfg.physics.thermal.model
                == EnergyModel::ConstantProperties)
                .then_some(self.cfg.physics.thermal.thermal_diffusivity);
            step_dt = adaptive_time_step(
                self.current_dt,
                AdaptiveTimeStepInputs {
                    max_abs_u: self.u.max_abs(),
                    max_abs_v: self.v.max_abs(),
                    dx: self.grid.dx,
                    dy: self.grid.dy,
                    kinematic_viscosity: self.cfg.case.kinematic_viscosity(),
                    thermal_diffusivity,
                },
                self.cfg.time_step,
            )?;
        }
        if self.cfg.coupling.is_transient() {
            let remaining_time = self.cfg.t_end - self.time;
            // The final step may be below min_dt only to land exactly on t_end.
            step_dt = step_dt.min(remaining_time);
            if step_dt <= 0.0 {
                return Err("projection simulation has already reached t_end".to_string());
            }
        }
        if self.cfg.time_step.adaptive {
            self.current_dt = step_dt;
        }
        let old_u = self.u.clone();
        let old_v = self.v.clone();

        let simple = matches!(
            self.cfg.coupling,
            PressureVelocityCoupling::Simple
                | PressureVelocityCoupling::SimpleImplicit
                | PressureVelocityCoupling::SimpleConsistent
        );
        let (pressure_residual, pressure_iterations, momentum_equation_residual) =
            if self.cfg.coupling == PressureVelocityCoupling::Pimple {
                let (pressure, pressure_iterations, momentum_residual) =
                    self.advance_implicit_pimple(step_dt, &old_u, &old_v)?;
                (pressure, pressure_iterations, Some(momentum_residual))
            } else if self.cfg.coupling.is_coefficient_based() {
                let (pressure, pressure_iterations, momentum_residual) =
                    self.advance_implicit_simple(step_dt, self.cfg.coupling.is_consistent())?;
                (pressure, pressure_iterations, Some(momentum_residual))
            } else {
                self.predict_momentum(simple, step_dt);
                apply_velocity_boundaries(
                    &self.cfg.case,
                    &self.cfg.boundary_overrides,
                    &self.grid,
                    &self.solid,
                    self.time + step_dt,
                    &mut self.u_star,
                    &mut self.v_star,
                );
                let old_pressure = simple.then(|| self.p.clone());
                if simple {
                    // During SIMPLE the Poisson solve is for p', not the absolute pressure.
                    self.p.fill(0.0);
                }
                self.build_pressure_rhs(step_dt);
                let result = self.solve_pressure_poisson()?;
                self.correct_velocity(
                    if simple {
                        self.cfg.velocity_relaxation
                    } else {
                        1.0
                    },
                    step_dt,
                );
                if let Some(old_pressure) = old_pressure {
                    let pressure_relaxation = self.cfg.pressure_relaxation;
                    for (pressure, correction) in self
                        .p
                        .as_mut_slice()
                        .iter_mut()
                        .zip(old_pressure.as_slice())
                    {
                        *pressure = *correction + pressure_relaxation * *pressure;
                    }
                }
                let (pressure, iterations) = result;
                (pressure, iterations, None)
            };
        apply_velocity_boundaries(
            &self.cfg.case,
            &self.cfg.boundary_overrides,
            &self.grid,
            &self.solid,
            self.time + step_dt,
            &mut self.u,
            &mut self.v,
        );
        if self.cfg.coupling == PressureVelocityCoupling::Pimple
            && matches!(
                self.cfg
                    .boundary_overrides
                    .kind(&self.cfg.case, Side::Right),
                BoundaryKind::PressureOutlet { .. }
            )
        {
            for j in 0..self.grid.ny {
                self.u[(self.grid.nx, j)] = self.u_star[(self.grid.nx, j)];
            }
        }

        self.step += 1;
        if self.cfg.coupling.is_transient() {
            self.time += step_dt;
        } else {
            self.time = self.step as f64 * self.cfg.dt;
        }
        self.compute_cell_fields();
        self.advance_temperature(step_dt)?;
        self.ensure_finite()?;

        let max_abs_u = self.u.max_abs();
        let max_abs_v = self.v.max_abs();
        let (momentum_cfl, viscous_diffusion_number) = momentum_stability_numbers(
            step_dt,
            self.cfg.case.kinematic_viscosity(),
            self.grid.dx,
            self.grid.dy,
            max_abs_u,
            max_abs_v,
        );
        let max_divergence = self.max_divergence();
        let max_speed = self.speed.max_abs();
        let velocity_change =
            max_field_difference(&self.u, &old_u).max(max_field_difference(&self.v, &old_v));
        let forces = self.cylinder_force_coefficients();
        let reattachment = self.reattachment_length_ratio();

        Ok(Diagnostics {
            time_step: step_dt,
            pressure_residual,
            pressure_iterations,
            max_divergence,
            max_speed,
            momentum_cfl,
            viscous_diffusion_number,
            velocity_change,
            momentum_equation_residual,
            cd: forces.cd(),
            cl: forces.cl(),
            cd_pressure: forces.cd_pressure,
            cd_viscous: forces.cd_viscous,
            cl_pressure: forces.cl_pressure,
            cl_viscous: forces.cl_viscous,
            reattachment,
        })
    }

    pub(super) fn compute_cell_fields(&mut self) {
        let _profile = ProfileScope::new(ProfileStage::CellFields);
        let Self {
            pool,
            grid,
            solid,
            u,
            v,
            u_cell,
            v_cell,
            speed,
            vorticity,
            ..
        } = self;
        let nx = grid.nx;
        let u_values = u.as_slice();
        let v_values = v.as_slice();
        pool.install(|| {
            u_cell
                .as_mut_slice()
                .par_chunks_mut(nx)
                .zip(v_cell.as_mut_slice().par_chunks_mut(nx))
                .zip(speed.as_mut_slice().par_chunks_mut(nx))
                .enumerate()
                .for_each(|(j, ((u_row, v_row), speed_row))| {
                    for i in 0..nx {
                        if solid[(i, j)] {
                            u_row[i] = 0.0;
                            v_row[i] = 0.0;
                            speed_row[i] = 0.0;
                            continue;
                        }
                        let uc =
                            0.5 * (u_values[i + (nx + 1) * j] + u_values[i + 1 + (nx + 1) * j]);
                        let vc = 0.5 * (v_values[i + nx * j] + v_values[i + nx * (j + 1)]);
                        u_row[i] = uc;
                        v_row[i] = vc;
                        speed_row[i] = (uc * uc + vc * vc).sqrt();
                    }
                });

            let u_cell_values = u_cell.as_slice();
            let v_cell_values = v_cell.as_slice();
            vorticity
                .as_mut_slice()
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row)| {
                    for i in 0..nx {
                        row[i] = if solid[(i, j)] {
                            0.0
                        } else {
                            let dvdx = derivative_x_slice(v_cell_values, solid, nx, i, j, grid.dx);
                            let dudy = derivative_y_slice(u_cell_values, solid, nx, i, j, grid.dy);
                            dvdx - dudy
                        };
                    }
                });
        });
    }
}
