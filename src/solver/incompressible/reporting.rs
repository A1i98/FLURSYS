use super::*;

impl IncompressibleSolver {
    pub(super) fn append_histories(&self, diag: Diagnostics) -> Result<(), String> {
        let history = self.cfg.output_dir.join("history.csv");
        let row = format!(
            "{},{:.12},{:.12e},{},{:.12e},{:.12e},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e}",
            self.step,
            self.time,
            diag.pressure_residual,
            diag.pressure_iterations,
            diag.max_divergence,
            diag.velocity_change,
            diag.momentum_equation_residual
                .map(|value| format!("{value:.12e}"))
                .unwrap_or_else(|| "".to_string()),
            diag.max_speed,
            diag.cd,
            diag.cl,
            diag.reattachment,
            diag.momentum_cfl,
            diag.viscous_diffusion_number,
            diag.time_step,
            diag.cd_pressure,
            diag.cd_viscous,
            diag.cl_pressure,
            diag.cl_viscous
        );
        output::append_history(
            &history,
            "step,time,pressure_residual,pressure_iterations,max_divergence,velocity_change,momentum_equation_residual,max_speed,cd,cl,reattachment_x_over_h,momentum_cfl,viscous_diffusion_number,time_step,cd_pressure,cd_viscous,cl_pressure,cl_viscous",
            &row,
        )
    }

    pub(super) fn write_snapshot(&mut self) -> Result<(), String> {
        self.compute_cell_fields();
        let frame_dir = self.cfg.output_dir.join("frames");
        let speed_path = frame_dir.join(format!("speed_{:05}.ppm", self.frame_index));
        let vort_path = frame_dir.join(format!("vorticity_{:05}.ppm", self.frame_index));
        output::write_ppm_frame(&speed_path, &self.grid, &self.solid, &self.speed, false)?;
        output::write_ppm_frame(&vort_path, &self.grid, &self.solid, &self.vorticity, true)?;
        if let Some(temperature) = &self.temperature {
            let temperature_path =
                frame_dir.join(format!("temperature_{:05}.ppm", self.frame_index));
            output::write_ppm_frame(
                &temperature_path,
                &self.grid,
                &self.solid,
                temperature,
                false,
            )?;
        }
        self.frame_index += 1;
        Ok(())
    }

    pub(super) fn write_final_outputs(&self) -> Result<(), String> {
        output::write_field_csv(
            &self.cfg.output_dir.join("field.csv"),
            &self.grid,
            &self.solid,
            output::CsvFields {
                pressure: &self.p,
                u: &self.u_cell,
                v: &self.v_cell,
                vorticity: &self.vorticity,
                temperature: self.temperature.as_ref(),
            },
        )?;
        output::write_legacy_vtk(
            &self.cfg.output_dir.join("field.vtk"),
            self.cfg.case.name(),
            &self.grid,
            &self.solid,
            output::VtkFields {
                pressure: &self.p,
                u: &self.u_cell,
                v: &self.v_cell,
                vorticity: &self.vorticity,
                temperature: self.temperature.as_ref(),
            },
        )?;
        self.write_case_specific_profiles()?;
        Ok(())
    }

    pub(super) fn write_case_specific_profiles(&self) -> Result<(), String> {
        match self.cfg.case.kind() {
            CaseKind::LidDrivenCavity => {
                let mut u_file = BufWriter::new(
                    File::create(self.cfg.output_dir.join("centerline_u_vertical.csv"))
                        .map_err(|e| e.to_string())?,
                );
                writeln!(u_file, "y,u").map_err(|e| e.to_string())?;
                let i = self.grid.nx / 2;
                for j in 0..self.grid.ny {
                    writeln!(
                        u_file,
                        "{:.12},{:.12e}",
                        self.grid.cell_y(j),
                        self.u_cell[(i, j)]
                    )
                    .map_err(|e| e.to_string())?;
                }

                let mut v_file = BufWriter::new(
                    File::create(self.cfg.output_dir.join("centerline_v_horizontal.csv"))
                        .map_err(|e| e.to_string())?,
                );
                writeln!(v_file, "x,v").map_err(|e| e.to_string())?;
                let j = self.grid.ny / 2;
                for i in 0..self.grid.nx {
                    writeln!(
                        v_file,
                        "{:.12},{:.12e}",
                        self.grid.cell_x(i),
                        self.v_cell[(i, j)]
                    )
                    .map_err(|e| e.to_string())?;
                }
            }
            CaseKind::CylinderRe100 => {
                let mut file = BufWriter::new(
                    File::create(self.cfg.output_dir.join("wake_centerline.csv"))
                        .map_err(|e| e.to_string())?,
                );
                writeln!(file, "x,u,v,p").map_err(|e| e.to_string())?;
                let j = self.grid.ny / 2;
                for i in 0..self.grid.nx {
                    if !self.solid[(i, j)] {
                        writeln!(
                            file,
                            "{:.12},{:.12e},{:.12e},{:.12e}",
                            self.grid.cell_x(i),
                            self.u_cell[(i, j)],
                            self.v_cell[(i, j)],
                            self.p[(i, j)]
                        )
                        .map_err(|e| e.to_string())?;
                    }
                }
            }
            CaseKind::BackwardFacingStep => {
                let mut file = BufWriter::new(
                    File::create(self.cfg.output_dir.join("bottom_wall_shear.csv"))
                        .map_err(|e| e.to_string())?,
                );
                writeln!(file, "x,tau_wall").map_err(|e| e.to_string())?;
                let mu = self.cfg.case.density() * self.cfg.case.kinematic_viscosity();
                for i in 0..self.grid.nx {
                    if !self.solid[(i, 0)] {
                        let tau = mu * self.u_cell[(i, 0)] / (0.5 * self.grid.dy);
                        writeln!(file, "{:.12},{:.12e}", self.grid.cell_x(i), tau)
                            .map_err(|e| e.to_string())?;
                    }
                }
            }
            CaseKind::Channel => {
                let mut file = BufWriter::new(
                    File::create(self.cfg.output_dir.join("outlet_velocity_profile.csv"))
                        .map_err(|e| e.to_string())?,
                );
                writeln!(file, "y,u,v,p").map_err(|e| e.to_string())?;
                let i = self.grid.nx - 1;
                for j in 0..self.grid.ny {
                    writeln!(
                        file,
                        "{:.12},{:.12e},{:.12e},{:.12e}",
                        self.grid.cell_y(j),
                        self.u_cell[(i, j)],
                        self.v_cell[(i, j)],
                        self.p[(i, j)]
                    )
                    .map_err(|e| e.to_string())?;
                }
            }
        }
        Ok(())
    }

    pub(super) fn write_case_summary(&self) -> Result<(), String> {
        let file = File::create(self.cfg.output_dir.join("case_summary.txt"))
            .map_err(|e| format!("Cannot write case summary: {e}"))?;
        let mut w = BufWriter::new(file);
        writeln!(w, "Case: {}", self.cfg.case.name()).map_err(|e| e.to_string())?;
        writeln!(
            w,
            "Method: finite volume, staggered grid, {:?}",
            self.cfg.coupling
        )
        .map_err(|e| e.to_string())?;
        writeln!(w, "Grid: {} x {}", self.grid.nx, self.grid.ny).map_err(|e| e.to_string())?;
        writeln!(w, "Domain: {} x {}", self.grid.length, self.grid.height)
            .map_err(|e| e.to_string())?;
        writeln!(w, "dx: {}", self.grid.dx).map_err(|e| e.to_string())?;
        writeln!(w, "dy: {}", self.grid.dy).map_err(|e| e.to_string())?;
        writeln!(w, "rho: {}", self.cfg.case.density()).map_err(|e| e.to_string())?;
        writeln!(w, "nu: {}", self.cfg.case.kinematic_viscosity()).map_err(|e| e.to_string())?;
        match self.cfg.coupling {
            PressureVelocityCoupling::Projection | PressureVelocityCoupling::Pimple => {
                writeln!(w, "dt: {}", self.cfg.dt).map_err(|e| e.to_string())?;
                if self.cfg.time_step.adaptive {
                    let settings = self.cfg.time_step;
                    writeln!(
                        w,
                        "time_stepping: adaptive, initial_dt={}, min_dt={}, max_dt={}, target_cfl={}, target_diffusion={}, max_growth={}",
                        self.cfg.dt,
                        settings.min_dt,
                        settings.max_dt,
                        settings.target_cfl,
                        settings.target_diffusion_number,
                        settings.max_growth_factor
                    )
                    .map_err(|e| e.to_string())?;
                } else {
                    writeln!(w, "time_stepping: fixed, dt={}", self.cfg.dt)
                        .map_err(|e| e.to_string())?;
                }
                writeln!(w, "t_end: {}", self.cfg.t_end).map_err(|e| e.to_string())?;
                writeln!(w, "max_steps: {}", self.cfg.max_steps).map_err(|e| e.to_string())?;
            }
            PressureVelocityCoupling::Simple => {
                writeln!(w, "pseudo_dt: {}", self.cfg.dt).map_err(|e| e.to_string())?;
                writeln!(w, "time_stepping: fixed, dt={}", self.cfg.dt)
                    .map_err(|e| e.to_string())?;
                writeln!(w, "max_iterations: {}", self.cfg.max_steps).map_err(|e| e.to_string())?;
            }
            PressureVelocityCoupling::SimpleImplicit
            | PressureVelocityCoupling::SimpleConsistent => {
                writeln!(w, "iteration_dt: {}", self.cfg.dt).map_err(|e| e.to_string())?;
                writeln!(w, "time_stepping: fixed, iteration based").map_err(|e| e.to_string())?;
                writeln!(w, "max_iterations: {}", self.cfg.max_steps).map_err(|e| e.to_string())?;
                writeln!(w, "momentum_convection: {:?}", self.cfg.convection)
                    .map_err(|e| e.to_string())?;
                writeln!(w, "momentum_coupling: {:?}", self.cfg.coupling)
                    .map_err(|e| e.to_string())?;
                writeln!(w, "momentum_solver: BiCGStab").map_err(|e| e.to_string())?;
                let preconditioner = match self.cfg.pressure_solver {
                    PressureSolverKind::Pcg => "Jacobi",
                    PressureSolverKind::PcgDic => "DIC(0)",
                    PressureSolverKind::PcgMg => "structured multigrid",
                    PressureSolverKind::Sor => "Jacobi (SOR unavailable in coefficient-based mode)",
                };
                writeln!(w, "pressure_correction_solver: PCG + {preconditioner}")
                    .map_err(|e| e.to_string())?;
            }
        }
        writeln!(w, "cpu_threads: {}", self.pool.current_num_threads())
            .map_err(|e| e.to_string())?;
        writeln!(w, "convection: {:?}", self.cfg.convection).map_err(|e| e.to_string())?;
        writeln!(w, "coupling: {:?}", self.cfg.coupling).map_err(|e| e.to_string())?;
        if self.cfg.coupling == PressureVelocityCoupling::Pimple {
            writeln!(
                w,
                "pressure_correctors: {}",
                self.cfg.pimple.pressure_correctors
            )
            .map_err(|e| e.to_string())?;
            writeln!(
                w,
                "momentum_absolute_tolerance: {}",
                self.cfg.pimple.momentum_tolerance
            )
            .map_err(|e| e.to_string())?;
            writeln!(
                w,
                "pressure_absolute_tolerance: {}",
                self.cfg.pressure_tolerance
            )
            .map_err(|e| e.to_string())?;
            writeln!(
                w,
                "first_pressure_relative_tolerance: {}",
                self.cfg.pimple.first_pressure_relative_tolerance
            )
            .map_err(|e| e.to_string())?;
            writeln!(w, "final_pressure_relative_tolerance: 0").map_err(|e| e.to_string())?;
        }
        writeln!(w, "pressure_solver: {:?}", self.cfg.pressure_solver)
            .map_err(|e| e.to_string())?;
        writeln!(w, "velocity_relaxation: {}", self.cfg.velocity_relaxation)
            .map_err(|e| e.to_string())?;
        writeln!(w, "pressure_relaxation: {}", self.cfg.pressure_relaxation)
            .map_err(|e| e.to_string())?;
        writeln!(w, "energy_model: {:?}", self.cfg.physics.thermal.model)
            .map_err(|e| e.to_string())?;
        writeln!(w, "buoyancy_model: {:?}", self.cfg.physics.buoyancy)
            .map_err(|e| e.to_string())?;
        Ok(())
    }
}
