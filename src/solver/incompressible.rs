use crate::cases::{BoundaryKind, Case, CaseKind, Side};
use crate::field::{Field2D, Mask2D};
use crate::grid::UniformGrid2D;
use crate::output;
use rayon::prelude::*;
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConvectionScheme {
    FirstOrderUpwind,
    Central,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PressureSolverKind {
    Pcg,
    Sor,
}

#[derive(Clone, Debug)]
pub struct SimulationConfig {
    pub case: Case,
    pub nx: usize,
    pub ny: usize,
    pub dt: f64,
    pub max_steps: usize,
    pub t_end: f64,
    pub convection: ConvectionScheme,
    pub pressure_solver: PressureSolverKind,
    pub pressure_max_iters: usize,
    pub pressure_tolerance: f64,
    pub pressure_omega: f64,
    pub print_every: usize,
    pub output_every: usize,
    pub frame_every: usize,
    pub steady_tolerance: f64,
    pub minimum_steps: usize,
    /// Number of worker threads. Zero selects Rayon's automatic CPU count.
    pub threads: usize,
    pub output_dir: PathBuf,
}

impl SimulationConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.nx < 4 || self.ny < 4 {
            return Err("nx and ny must be at least 4".to_string());
        }
        if !(self.dt > 0.0 && self.t_end > 0.0) {
            return Err("dt and t_end must be positive".to_string());
        }
        if self.max_steps == 0 {
            return Err("max_steps must be positive".to_string());
        }
        if self.pressure_max_iters == 0 {
            return Err("pressure_max_iters must be positive".to_string());
        }
        if !(0.0 < self.pressure_omega && self.pressure_omega < 2.0) {
            return Err("pressure_omega must lie between 0 and 2".to_string());
        }
        if self.print_every == 0 || self.output_every == 0 || self.frame_every == 0 {
            return Err("Output intervals must be positive".to_string());
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct RunSummary {
    pub case_name: String,
    pub steps: usize,
    pub final_time: f64,
    pub max_divergence: f64,
    pub pressure_residual: f64,
    pub elapsed: Duration,
    pub converged: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct Diagnostics {
    pressure_residual: f64,
    pressure_iterations: usize,
    max_divergence: f64,
    max_speed: f64,
    velocity_change: f64,
    cd: f64,
    cl: f64,
    reattachment: f64,
}

pub struct IncompressibleSolver {
    cfg: SimulationConfig,
    grid: UniformGrid2D,
    solid: Mask2D,
    p: Field2D,
    rhs: Field2D,
    u: Field2D,
    v: Field2D,
    u_star: Field2D,
    v_star: Field2D,
    u_cell: Field2D,
    v_cell: Field2D,
    speed: Field2D,
    vorticity: Field2D,
    step: usize,
    time: f64,
    frame_index: usize,
    last_diag: Diagnostics,
    pool: ThreadPool,
}

impl IncompressibleSolver {
    pub fn new(cfg: SimulationConfig) -> Result<Self, String> {
        cfg.validate()?;
        let (length, height) = cfg.case.domain();
        let grid = UniformGrid2D::new(cfg.nx, cfg.ny, length, height)?;
        let pool = if cfg.threads == 0 {
            ThreadPoolBuilder::new().build()
        } else {
            ThreadPoolBuilder::new().num_threads(cfg.threads).build()
        }
        .map_err(|error| format!("Cannot create CPU worker pool: {error}"))?;
        let mut solid = Mask2D::new(cfg.nx, cfg.ny, false);
        for j in 0..cfg.ny {
            for i in 0..cfg.nx {
                solid[(i, j)] = cfg.case.is_solid(grid.cell_x(i), grid.cell_y(j));
            }
        }

        let mut solver = Self {
            p: Field2D::new(cfg.nx, cfg.ny, 0.0),
            rhs: Field2D::new(cfg.nx, cfg.ny, 0.0),
            u: Field2D::new(cfg.nx + 1, cfg.ny, 0.0),
            v: Field2D::new(cfg.nx, cfg.ny + 1, 0.0),
            u_star: Field2D::new(cfg.nx + 1, cfg.ny, 0.0),
            v_star: Field2D::new(cfg.nx, cfg.ny + 1, 0.0),
            u_cell: Field2D::new(cfg.nx, cfg.ny, 0.0),
            v_cell: Field2D::new(cfg.nx, cfg.ny, 0.0),
            speed: Field2D::new(cfg.nx, cfg.ny, 0.0),
            vorticity: Field2D::new(cfg.nx, cfg.ny, 0.0),
            cfg,
            grid,
            solid,
            step: 0,
            time: 0.0,
            frame_index: 0,
            last_diag: Diagnostics::default(),
            pool,
        };
        solver.initialize_velocity();
        solver.compute_cell_fields();
        Ok(solver)
    }

    pub fn run(&mut self) -> Result<RunSummary, String> {
        output::ensure_output_tree(&self.cfg.output_dir)?;
        self.write_case_summary()?;
        self.write_snapshot()?;
        let started = Instant::now();
        let mut converged = false;

        println!("Case: {}", self.cfg.case.name());
        println!(
            "Grid: {} x {}, dx={:.6}, dy={:.6}",
            self.grid.nx, self.grid.ny, self.grid.dx, self.grid.dy
        );
        println!(
            "rho={:.6}, nu={:.6}, dt={:.6}, max_steps={}, t_end={:.6}",
            self.cfg.case.density(),
            self.cfg.case.kinematic_viscosity(),
            self.cfg.dt,
            self.cfg.max_steps,
            self.cfg.t_end
        );
        println!("CPU worker threads: {}", self.pool.current_num_threads());
        match self.cfg.pressure_solver {
            PressureSolverKind::Pcg => println!(
                "Pressure: PCG + Jacobi preconditioner, tol={:.3e}, max_iters={}",
                self.cfg.pressure_tolerance, self.cfg.pressure_max_iters
            ),
            PressureSolverKind::Sor => println!(
                "Pressure: SOR omega={:.3}, tol={:.3e}, max_iters={}",
                self.cfg.pressure_omega, self.cfg.pressure_tolerance, self.cfg.pressure_max_iters
            ),
        }
        if self.cfg.pressure_solver == PressureSolverKind::Sor
            && self.pool.current_num_threads() > 1
        {
            println!(
                "SOR uses sequential in-place sweeps; use PCG to parallelize the pressure solve."
            );
        }

        while self.step < self.cfg.max_steps && self.time < self.cfg.t_end {
            let diag = self.advance_one_step()?;
            self.last_diag = diag;

            if self.step % self.cfg.output_every == 0 {
                self.append_histories(diag)?;
            }
            if self.step % self.cfg.frame_every == 0 {
                self.write_snapshot()?;
            }
            if self.step % self.cfg.print_every == 0 {
                println!(
                    "step {:>8}/{:<8} t={:>10.4} div={:.3e} p_res={:.3e} p_it={:>4} du={:.3e} umax={:.5} Cd={:.5} Cl={:.5} xr/h={:.5} elapsed={:.1}s",
                    self.step,
                    self.cfg.max_steps,
                    self.time,
                    diag.max_divergence,
                    diag.pressure_residual,
                    diag.pressure_iterations,
                    diag.velocity_change,
                    diag.max_speed,
                    diag.cd,
                    diag.cl,
                    diag.reattachment,
                    started.elapsed().as_secs_f64()
                );
            }

            if self.cfg.case.kind() != CaseKind::CylinderRe100
                && self.step >= self.cfg.minimum_steps
                && diag.velocity_change < self.cfg.steady_tolerance
                && diag.max_divergence < 10.0 * self.cfg.pressure_tolerance
            {
                converged = true;
                break;
            }
        }

        self.compute_cell_fields();
        self.write_final_outputs()?;
        Ok(RunSummary {
            case_name: self.cfg.case.name().to_string(),
            steps: self.step,
            final_time: self.time,
            max_divergence: self.last_diag.max_divergence,
            pressure_residual: self.last_diag.pressure_residual,
            elapsed: started.elapsed(),
            converged,
        })
    }

    fn initialize_velocity(&mut self) {
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
            &self.grid,
            &self.solid,
            self.time,
            &mut self.u,
            &mut self.v,
        );
    }

    fn advance_one_step(&mut self) -> Result<Diagnostics, String> {
        let old_u = self.u.clone();
        let old_v = self.v.clone();

        self.predict_momentum();
        apply_velocity_boundaries(
            &self.cfg.case,
            &self.grid,
            &self.solid,
            self.time + self.cfg.dt,
            &mut self.u_star,
            &mut self.v_star,
        );
        self.build_pressure_rhs();
        let (pressure_residual, pressure_iterations) = self.solve_pressure_poisson()?;
        self.correct_velocity();
        apply_velocity_boundaries(
            &self.cfg.case,
            &self.grid,
            &self.solid,
            self.time + self.cfg.dt,
            &mut self.u,
            &mut self.v,
        );

        self.step += 1;
        self.time = self.step as f64 * self.cfg.dt;
        self.compute_cell_fields();
        self.ensure_finite()?;

        let max_divergence = self.max_divergence();
        let max_speed = self.speed.max_abs();
        let velocity_change =
            max_field_difference(&self.u, &old_u).max(max_field_difference(&self.v, &old_v));
        let (cd, cl) = self.cylinder_force_coefficients();
        let reattachment = self.reattachment_length_ratio();

        Ok(Diagnostics {
            pressure_residual,
            pressure_iterations,
            max_divergence,
            max_speed,
            velocity_change,
            cd,
            cl,
            reattachment,
        })
    }

    fn predict_momentum(&mut self) {
        self.u_star.fill(0.0);
        self.v_star.fill(0.0);
        let Self {
            pool,
            cfg,
            grid,
            solid,
            u,
            v,
            u_star,
            v_star,
            ..
        } = self;
        let nx = grid.nx;
        let ny = grid.ny;
        let dx = grid.dx;
        let dy = grid.dy;
        let dt = cfg.dt;
        let nu = cfg.case.kinematic_viscosity();
        let convection_scheme = cfg.convection;
        let u_values = u.as_slice();
        let v_values = v.as_slice();
        let stencil = MomentumStencil {
            solid,
            case: &cfg.case,
            grid,
        };

        pool.install(|| {
            u_star
                .as_mut_slice()
                .par_chunks_mut(nx + 1)
                .enumerate()
                .for_each(|(j, row)| {
                    for i in 1..nx {
                        if !u_face_open(solid, nx, i, j) {
                            continue;
                        }
                        let up = u_values[i + (nx + 1) * j];
                        let ue_nb = stencil.u_neighbor(u_values, i, j, 1, 0, up);
                        let uw_nb = stencil.u_neighbor(u_values, i, j, -1, 0, up);
                        let un_nb = stencil.u_neighbor(u_values, i, j, 0, 1, up);
                        let us_nb = stencil.u_neighbor(u_values, i, j, 0, -1, up);

                        let ue = 0.5 * (up + ue_nb);
                        let uw = 0.5 * (uw_nb + up);
                        let vn =
                            0.5 * (v_values[i - 1 + nx * (j + 1)] + v_values[i + nx * (j + 1)]);
                        let vs = 0.5 * (v_values[i - 1 + nx * j] + v_values[i + nx * j]);

                        let phi_e = transported(convection_scheme, ue, up, ue_nb);
                        let phi_w = transported(convection_scheme, uw, uw_nb, up);
                        let phi_n = transported(convection_scheme, vn, up, un_nb);
                        let phi_s = transported(convection_scheme, vs, us_nb, up);

                        let convection =
                            (ue * phi_e - uw * phi_w) / dx + (vn * phi_n - vs * phi_s) / dy;
                        let diffusion = (ue_nb - 2.0 * up + uw_nb) / (dx * dx)
                            + (un_nb - 2.0 * up + us_nb) / (dy * dy);
                        row[i] = up + dt * (-convection + nu * diffusion);
                    }
                });

            v_star
                .as_mut_slice()
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row)| {
                    if j == 0 || j >= ny {
                        return;
                    }
                    for i in 0..nx {
                        if !v_face_open(solid, ny, i, j) {
                            continue;
                        }
                        let vp = v_values[i + nx * j];
                        let ve_nb = stencil.v_neighbor(v_values, i, j, 1, 0, vp);
                        let vw_nb = stencil.v_neighbor(v_values, i, j, -1, 0, vp);
                        let vn_nb = stencil.v_neighbor(v_values, i, j, 0, 1, vp);
                        let vs_nb = stencil.v_neighbor(v_values, i, j, 0, -1, vp);

                        let ue = 0.5
                            * (u_values[i + 1 + (nx + 1) * (j - 1)]
                                + u_values[i + 1 + (nx + 1) * j]);
                        let uw =
                            0.5 * (u_values[i + (nx + 1) * (j - 1)] + u_values[i + (nx + 1) * j]);
                        let vn = 0.5 * (vp + vn_nb);
                        let vs = 0.5 * (vs_nb + vp);

                        let phi_e = transported(convection_scheme, ue, vp, ve_nb);
                        let phi_w = transported(convection_scheme, uw, vw_nb, vp);
                        let phi_n = transported(convection_scheme, vn, vp, vn_nb);
                        let phi_s = transported(convection_scheme, vs, vs_nb, vp);

                        let convection =
                            (ue * phi_e - uw * phi_w) / dx + (vn * phi_n - vs * phi_s) / dy;
                        let diffusion = (ve_nb - 2.0 * vp + vw_nb) / (dx * dx)
                            + (vn_nb - 2.0 * vp + vs_nb) / (dy * dy);
                        row[i] = vp + dt * (-convection + nu * diffusion);
                    }
                });
        });
    }

    fn build_pressure_rhs(&mut self) {
        let Self {
            pool,
            cfg,
            grid,
            solid,
            u_star,
            v_star,
            rhs,
            ..
        } = self;
        let nx = grid.nx;
        let rho_over_dt = cfg.case.density() / cfg.dt;
        let u_values = u_star.as_slice();
        let v_values = v_star.as_slice();
        pool.install(|| {
            rhs.as_mut_slice()
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row)| {
                    for i in 0..nx {
                        if solid[(i, j)] || pressure_outlet_cell(&cfg.case, solid, nx, i, j) {
                            row[i] = 0.0;
                            continue;
                        }
                        let div = (u_values[i + 1 + (nx + 1) * j] - u_values[i + (nx + 1) * j])
                            / grid.dx
                            + (v_values[i + nx * (j + 1)] - v_values[i + nx * j]) / grid.dy;
                        row[i] = rho_over_dt * div;
                    }
                });
        });
    }

    fn solve_pressure_poisson(&mut self) -> Result<(f64, usize), String> {
        match self.cfg.pressure_solver {
            PressureSolverKind::Pcg => self.solve_pressure_pcg(),
            PressureSolverKind::Sor => self.solve_pressure_sor(),
        }
    }

    fn solve_pressure_sor(&mut self) -> Result<(f64, usize), String> {
        let idx2 = 1.0 / (self.grid.dx * self.grid.dx);
        let idy2 = 1.0 / (self.grid.dy * self.grid.dy);
        let omega = self.cfg.pressure_omega;
        let mut residual = f64::INFINITY;

        let mut iterations = 0_usize;
        for iteration in 0..self.cfg.pressure_max_iters {
            iterations = iteration + 1;
            for j in 0..self.grid.ny {
                for i in 0..self.grid.nx {
                    if self.solid[(i, j)] {
                        self.p[(i, j)] = 0.0;
                        continue;
                    }
                    if self.is_pressure_outlet_cell(i, j) {
                        self.p[(i, j)] = self.outlet_pressure();
                        continue;
                    }
                    let mut ap = 0.0;
                    let mut sum = 0.0;
                    if i + 1 < self.grid.nx && !self.solid[(i + 1, j)] {
                        ap += idx2;
                        sum += idx2 * self.p[(i + 1, j)];
                    }
                    if i > 0 && !self.solid[(i - 1, j)] {
                        ap += idx2;
                        sum += idx2 * self.p[(i - 1, j)];
                    }
                    if j + 1 < self.grid.ny && !self.solid[(i, j + 1)] {
                        ap += idy2;
                        sum += idy2 * self.p[(i, j + 1)];
                    }
                    if j > 0 && !self.solid[(i, j - 1)] {
                        ap += idy2;
                        sum += idy2 * self.p[(i, j - 1)];
                    }
                    if ap > 0.0 {
                        let candidate = (sum - self.rhs[(i, j)]) / ap;
                        let old = self.p[(i, j)];
                        self.p[(i, j)] = old + omega * (candidate - old);
                    }
                }
            }

            if self.cfg.case.pressure_reference_required() {
                if let Some((ri, rj)) = self.find_reference_cell() {
                    let reference_pressure = self.p[(ri, rj)];
                    for jj in 0..self.grid.ny {
                        for ii in 0..self.grid.nx {
                            if !self.solid[(ii, jj)] {
                                self.p[(ii, jj)] -= reference_pressure;
                            }
                        }
                    }
                }
            }

            residual = self.pressure_residual();
            if !residual.is_finite() {
                return Err("Pressure solver produced a non-finite residual".to_string());
            }
            if residual < self.cfg.pressure_tolerance {
                break;
            }
        }
        Ok((residual, iterations))
    }

    fn solve_pressure_pcg(&mut self) -> Result<(f64, usize), String> {
        let n = self.grid.nx * self.grid.ny;
        let mut x = self.p.as_slice().to_vec();
        let mut b = vec![0.0_f64; n];
        let mut r = vec![0.0_f64; n];
        let mut z = vec![0.0_f64; n];
        let mut direction = vec![0.0_f64; n];
        let mut q = vec![0.0_f64; n];

        let nx = self.grid.nx;
        let solid = &self.solid;
        let case = &self.cfg.case;
        let rhs = self.rhs.as_slice();
        let outlet_pressure = self.outlet_pressure();
        self.pool.install(|| {
            x.par_chunks_mut(nx)
                .zip(b.par_chunks_mut(nx))
                .enumerate()
                .for_each(|(j, (x_row, b_row))| {
                    for i in 0..nx {
                        let k = i + nx * j;
                        if solid[(i, j)] || pressure_outlet_cell(case, solid, nx, i, j) {
                            let value = if pressure_outlet_cell(case, solid, nx, i, j) {
                                outlet_pressure
                            } else {
                                0.0
                            };
                            x_row[i] = value;
                            b_row[i] = value;
                        } else {
                            b_row[i] = -rhs[k];
                        }
                    }
                });
        });

        if self.cfg.case.pressure_reference_required() {
            self.project_pressure_zero_mean(&mut x);
            self.project_pressure_zero_mean(&mut b);
        }

        self.apply_pressure_operator(&x, &mut q);
        self.pool.install(|| {
            r.par_iter_mut()
                .zip(b.par_iter())
                .zip(q.par_iter())
                .for_each(|((r_value, &b_value), &q_value)| *r_value = b_value - q_value);
        });
        if self.cfg.case.pressure_reference_required() {
            self.project_pressure_zero_mean(&mut r);
        }

        let mut residual = self.parallel_max_abs(&r);
        if residual < self.cfg.pressure_tolerance {
            self.p.as_mut_slice().copy_from_slice(&x);
            return Ok((residual, 0));
        }

        self.apply_jacobi_preconditioner(&r, &mut z);
        if self.cfg.case.pressure_reference_required() {
            self.project_pressure_zero_mean(&mut z);
        }
        direction.copy_from_slice(&z);
        let mut rz_old = self.parallel_dot(&r, &z);

        let mut iterations = 0_usize;
        for iteration in 0..self.cfg.pressure_max_iters {
            iterations = iteration + 1;
            self.apply_pressure_operator(&direction, &mut q);
            let denominator = self.parallel_dot(&direction, &q);
            if denominator.abs() < 1.0e-30 || !denominator.is_finite() {
                return Err(
                    "PCG pressure solver encountered a singular search direction".to_string(),
                );
            }
            let alpha = rz_old / denominator;
            if !alpha.is_finite() {
                return Err("PCG pressure solver produced a non-finite step length".to_string());
            }
            self.pool.install(|| {
                x.par_iter_mut()
                    .zip(r.par_iter_mut())
                    .zip(direction.par_iter())
                    .zip(q.par_iter())
                    .for_each(|(((x_value, r_value), &direction_value), &q_value)| {
                        *x_value += alpha * direction_value;
                        *r_value -= alpha * q_value;
                    });
            });
            if self.cfg.case.pressure_reference_required() {
                self.project_pressure_zero_mean(&mut x);
                self.project_pressure_zero_mean(&mut r);
            }

            residual = self.parallel_max_abs(&r);
            if !residual.is_finite() {
                return Err("PCG pressure solver produced a non-finite residual".to_string());
            }
            if residual < self.cfg.pressure_tolerance {
                break;
            }

            self.apply_jacobi_preconditioner(&r, &mut z);
            if self.cfg.case.pressure_reference_required() {
                self.project_pressure_zero_mean(&mut z);
            }
            let rz_new = self.parallel_dot(&r, &z);
            if rz_old.abs() < 1.0e-30 {
                break;
            }
            let beta = rz_new / rz_old;
            self.pool.install(|| {
                direction.par_iter_mut().zip(z.par_iter()).for_each(
                    |(direction_value, &z_value)| {
                        *direction_value = z_value + beta * *direction_value;
                    },
                );
            });
            if self.cfg.case.pressure_reference_required() {
                self.project_pressure_zero_mean(&mut direction);
            }
            rz_old = rz_new;
        }

        let case = &self.cfg.case;
        let solid = &self.solid;
        let outlet_pressure = self.outlet_pressure();
        self.pool.install(|| {
            self.p
                .as_mut_slice()
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row)| {
                    for i in 0..nx {
                        row[i] = if solid[(i, j)] {
                            0.0
                        } else if pressure_outlet_cell(case, solid, nx, i, j) {
                            outlet_pressure
                        } else {
                            x[i + nx * j]
                        };
                    }
                });
        });
        Ok((residual, iterations))
    }

    fn apply_pressure_operator(&self, x: &[f64], result: &mut [f64]) {
        let idx2 = 1.0 / (self.grid.dx * self.grid.dx);
        let idy2 = 1.0 / (self.grid.dy * self.grid.dy);
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let solid = &self.solid;
        let case = &self.cfg.case;
        self.pool.install(|| {
            result.par_chunks_mut(nx).enumerate().for_each(|(j, row)| {
                for i in 0..nx {
                    let k = i + nx * j;
                    if solid[(i, j)] || pressure_outlet_cell(case, solid, nx, i, j) {
                        row[i] = x[k];
                        continue;
                    }

                    let mut diagonal = 0.0;
                    let mut neighbour_sum = 0.0;
                    if i + 1 < nx && !solid[(i + 1, j)] {
                        diagonal += idx2;
                        neighbour_sum += idx2 * x[k + 1];
                    }
                    if i > 0 && !solid[(i - 1, j)] {
                        diagonal += idx2;
                        neighbour_sum += idx2 * x[k - 1];
                    }
                    if j + 1 < ny && !solid[(i, j + 1)] {
                        diagonal += idy2;
                        neighbour_sum += idy2 * x[k + nx];
                    }
                    if j > 0 && !solid[(i, j - 1)] {
                        diagonal += idy2;
                        neighbour_sum += idy2 * x[k - nx];
                    }
                    row[i] = diagonal * x[k] - neighbour_sum;
                }
            });
        });
    }

    fn apply_jacobi_preconditioner(&self, r: &[f64], z: &mut [f64]) {
        let idx2 = 1.0 / (self.grid.dx * self.grid.dx);
        let idy2 = 1.0 / (self.grid.dy * self.grid.dy);
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let solid = &self.solid;
        let case = &self.cfg.case;
        self.pool.install(|| {
            z.par_chunks_mut(nx).enumerate().for_each(|(j, row)| {
                for i in 0..nx {
                    let k = i + nx * j;
                    if solid[(i, j)] || pressure_outlet_cell(case, solid, nx, i, j) {
                        row[i] = r[k];
                        continue;
                    }
                    let mut diagonal = 0.0;
                    if i + 1 < nx && !solid[(i + 1, j)] {
                        diagonal += idx2;
                    }
                    if i > 0 && !solid[(i - 1, j)] {
                        diagonal += idx2;
                    }
                    if j + 1 < ny && !solid[(i, j + 1)] {
                        diagonal += idy2;
                    }
                    if j > 0 && !solid[(i, j - 1)] {
                        diagonal += idy2;
                    }
                    row[i] = if diagonal > 0.0 {
                        r[k] / diagonal
                    } else {
                        r[k]
                    };
                }
            });
        });
    }

    fn project_pressure_zero_mean(&self, values: &mut [f64]) {
        let pool = &self.pool;
        let solid = &self.solid;
        let case = &self.cfg.case;
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        project_zero_mean(pool, values, solid, nx, ny, |i, j| {
            pressure_outlet_cell(case, solid, nx, i, j)
        });
    }

    fn parallel_dot(&self, a: &[f64], b: &[f64]) -> f64 {
        self.pool
            .install(|| a.par_iter().zip(b.par_iter()).map(|(&x, &y)| x * y).sum())
    }

    fn parallel_max_abs(&self, values: &[f64]) -> f64 {
        self.pool.install(|| {
            values
                .par_iter()
                .map(|value| value.abs())
                .reduce(|| 0.0, f64::max)
        })
    }

    fn correct_velocity(&mut self) {
        let dt_over_rho = self.cfg.dt / self.cfg.case.density();
        self.u
            .as_mut_slice()
            .copy_from_slice(self.u_star.as_slice());
        self.v
            .as_mut_slice()
            .copy_from_slice(self.v_star.as_slice());
        let Self {
            pool,
            grid,
            solid,
            p,
            u,
            v,
            u_star,
            v_star,
            ..
        } = self;
        let nx = grid.nx;
        let ny = grid.ny;
        let p_values = p.as_slice();
        let u_star_values = u_star.as_slice();
        let v_star_values = v_star.as_slice();
        pool.install(|| {
            u.as_mut_slice()
                .par_chunks_mut(nx + 1)
                .enumerate()
                .for_each(|(j, row)| {
                    for i in 1..nx {
                        if u_face_open(solid, nx, i, j) {
                            let gradient =
                                (p_values[i + nx * j] - p_values[i - 1 + nx * j]) / grid.dx;
                            row[i] = u_star_values[i + (nx + 1) * j] - dt_over_rho * gradient;
                        } else {
                            row[i] = 0.0;
                        }
                    }
                });
            v.as_mut_slice()
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row)| {
                    if j == 0 || j >= ny {
                        return;
                    }
                    for i in 0..nx {
                        if v_face_open(solid, ny, i, j) {
                            let gradient =
                                (p_values[i + nx * j] - p_values[i + nx * (j - 1)]) / grid.dy;
                            row[i] = v_star_values[i + nx * j] - dt_over_rho * gradient;
                        } else {
                            row[i] = 0.0;
                        }
                    }
                });
        });
    }

    fn compute_cell_fields(&mut self) {
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

    fn pressure_residual(&self) -> f64 {
        let idx2 = 1.0 / (self.grid.dx * self.grid.dx);
        let idy2 = 1.0 / (self.grid.dy * self.grid.dy);
        let mut max_r = 0.0_f64;
        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                if self.solid[(i, j)] || self.is_pressure_outlet_cell(i, j) {
                    continue;
                }
                let p0 = self.p[(i, j)];
                let pe = if i + 1 < self.grid.nx && !self.solid[(i + 1, j)] {
                    self.p[(i + 1, j)]
                } else {
                    p0
                };
                let pw = if i > 0 && !self.solid[(i - 1, j)] {
                    self.p[(i - 1, j)]
                } else {
                    p0
                };
                let pn = if j + 1 < self.grid.ny && !self.solid[(i, j + 1)] {
                    self.p[(i, j + 1)]
                } else {
                    p0
                };
                let ps = if j > 0 && !self.solid[(i, j - 1)] {
                    self.p[(i, j - 1)]
                } else {
                    p0
                };
                let lap = (pe - 2.0 * p0 + pw) * idx2 + (pn - 2.0 * p0 + ps) * idy2;
                max_r = max_r.max((lap - self.rhs[(i, j)]).abs());
            }
        }
        max_r
    }

    fn max_divergence(&self) -> f64 {
        let mut max_div = 0.0_f64;
        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                if self.solid[(i, j)] {
                    continue;
                }
                let div = (self.u[(i + 1, j)] - self.u[(i, j)]) / self.grid.dx
                    + (self.v[(i, j + 1)] - self.v[(i, j)]) / self.grid.dy;
                max_div = max_div.max(div.abs());
            }
        }
        max_div
    }

    fn u_face_open(&self, i: usize, j: usize) -> bool {
        u_face_open(&self.solid, self.grid.nx, i, j)
    }

    fn v_face_open(&self, i: usize, j: usize) -> bool {
        v_face_open(&self.solid, self.grid.ny, i, j)
    }

    fn is_pressure_outlet_cell(&self, i: usize, j: usize) -> bool {
        i + 1 == self.grid.nx
            && !self.solid[(i, j)]
            && matches!(
                self.cfg.case.boundary_kind(Side::Right),
                BoundaryKind::PressureOutlet { .. }
            )
    }

    fn outlet_pressure(&self) -> f64 {
        match self.cfg.case.boundary_kind(Side::Right) {
            BoundaryKind::PressureOutlet { pressure } => pressure,
            _ => 0.0,
        }
    }

    fn find_reference_cell(&self) -> Option<(usize, usize)> {
        if !self.cfg.case.pressure_reference_required() {
            return None;
        }
        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                if !self.solid[(i, j)] {
                    return Some((i, j));
                }
            }
        }
        None
    }

    fn cylinder_force_coefficients(&self) -> (f64, f64) {
        let Some(cyl) = self.cfg.case.cylinder() else {
            return (0.0, 0.0);
        };
        let mu = cyl.mu;
        let mut fx = 0.0;
        let mut fy = 0.0;

        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                if self.solid[(i, j)] {
                    continue;
                }
                let p = self.p[(i, j)];
                let uc = self.u_cell[(i, j)];
                let vc = self.v_cell[(i, j)];

                if i + 1 < self.grid.nx && self.solid[(i + 1, j)] {
                    fx += p * self.grid.dy;
                    fy += mu * vc / (0.5 * self.grid.dx) * self.grid.dy;
                }
                if i > 0 && self.solid[(i - 1, j)] {
                    fx -= p * self.grid.dy;
                    fy += mu * vc / (0.5 * self.grid.dx) * self.grid.dy;
                }
                if j + 1 < self.grid.ny && self.solid[(i, j + 1)] {
                    fy += p * self.grid.dx;
                    fx += mu * uc / (0.5 * self.grid.dy) * self.grid.dx;
                }
                if j > 0 && self.solid[(i, j - 1)] {
                    fy -= p * self.grid.dx;
                    fx += mu * uc / (0.5 * self.grid.dy) * self.grid.dx;
                }
            }
        }

        let q = 0.5 * cyl.rho * cyl.u_inf * cyl.u_inf * cyl.diameter;
        if q > 0.0 {
            (fx / q, fy / q)
        } else {
            (0.0, 0.0)
        }
    }

    fn reattachment_length_ratio(&self) -> f64 {
        let Some(step) = self.cfg.case.backward_step() else {
            return 0.0;
        };
        let start_i = ((step.step_x / self.grid.dx).floor() as usize).min(self.grid.nx - 2);
        let j = 0;
        let mu = step.rho * step.nu;
        let mut previous_tau: Option<f64> = None;
        let mut previous_x = 0.0;

        for i in (start_i + 1)..(self.grid.nx - 1) {
            if self.solid[(i, j)] {
                continue;
            }
            let tau = mu * self.u_cell[(i, j)] / (0.5 * self.grid.dy);
            let x = self.grid.cell_x(i);
            if let Some(tau0) = previous_tau {
                if tau0 <= 0.0 && tau > 0.0 {
                    let fraction = (-tau0 / (tau - tau0)).clamp(0.0, 1.0);
                    let xr = previous_x + fraction * (x - previous_x);
                    return (xr - step.step_x) / step.step_height;
                }
            }
            previous_tau = Some(tau);
            previous_x = x;
        }
        f64::NAN
    }

    fn ensure_finite(&self) -> Result<(), String> {
        for (name, field) in [
            ("pressure", &self.p),
            ("u", &self.u),
            ("v", &self.v),
            ("speed", &self.speed),
        ] {
            if let Some((index, value)) = field
                .as_slice()
                .iter()
                .copied()
                .enumerate()
                .find(|(_, value)| !value.is_finite())
            {
                return Err(format!(
                    "Non-finite {name} value at flat index {index}: {value} (step {})",
                    self.step
                ));
            }
        }
        Ok(())
    }

    fn append_histories(&self, diag: Diagnostics) -> Result<(), String> {
        let history = self.cfg.output_dir.join("history.csv");
        let row = format!(
            "{},{:.12},{:.12e},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e}",
            self.step,
            self.time,
            diag.pressure_residual,
            diag.pressure_iterations,
            diag.max_divergence,
            diag.velocity_change,
            diag.max_speed,
            diag.cd,
            diag.cl,
            diag.reattachment
        );
        output::append_history(
            &history,
            "step,time,pressure_residual,pressure_iterations,max_divergence,velocity_change,max_speed,cd,cl,reattachment_x_over_h",
            &row,
        )
    }

    fn write_snapshot(&mut self) -> Result<(), String> {
        self.compute_cell_fields();
        let frame_dir = self.cfg.output_dir.join("frames");
        let speed_path = frame_dir.join(format!("speed_{:05}.ppm", self.frame_index));
        let vort_path = frame_dir.join(format!("vorticity_{:05}.ppm", self.frame_index));
        output::write_ppm_frame(&speed_path, &self.grid, &self.solid, &self.speed, false)?;
        output::write_ppm_frame(&vort_path, &self.grid, &self.solid, &self.vorticity, true)?;
        self.frame_index += 1;
        Ok(())
    }

    fn write_final_outputs(&self) -> Result<(), String> {
        output::write_field_csv(
            &self.cfg.output_dir.join("field.csv"),
            &self.grid,
            &self.solid,
            &self.p,
            &self.u_cell,
            &self.v_cell,
            &self.vorticity,
        )?;
        output::write_legacy_vtk(
            &self.cfg.output_dir.join("field.vtk"),
            self.cfg.case.name(),
            &self.grid,
            &self.solid,
            &self.p,
            &self.u_cell,
            &self.v_cell,
            &self.vorticity,
        )?;
        self.write_case_specific_profiles()?;
        Ok(())
    }

    fn write_case_specific_profiles(&self) -> Result<(), String> {
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
        }
        Ok(())
    }

    fn write_case_summary(&self) -> Result<(), String> {
        let file = File::create(self.cfg.output_dir.join("case_summary.txt"))
            .map_err(|e| format!("Cannot write case summary: {e}"))?;
        let mut w = BufWriter::new(file);
        writeln!(w, "Case: {}", self.cfg.case.name()).map_err(|e| e.to_string())?;
        writeln!(
            w,
            "Method: finite volume, staggered grid, projection method"
        )
        .map_err(|e| e.to_string())?;
        writeln!(w, "Grid: {} x {}", self.grid.nx, self.grid.ny).map_err(|e| e.to_string())?;
        writeln!(w, "Domain: {} x {}", self.grid.length, self.grid.height)
            .map_err(|e| e.to_string())?;
        writeln!(w, "dx: {}", self.grid.dx).map_err(|e| e.to_string())?;
        writeln!(w, "dy: {}", self.grid.dy).map_err(|e| e.to_string())?;
        writeln!(w, "rho: {}", self.cfg.case.density()).map_err(|e| e.to_string())?;
        writeln!(w, "nu: {}", self.cfg.case.kinematic_viscosity()).map_err(|e| e.to_string())?;
        writeln!(w, "dt: {}", self.cfg.dt).map_err(|e| e.to_string())?;
        writeln!(w, "t_end: {}", self.cfg.t_end).map_err(|e| e.to_string())?;
        writeln!(w, "max_steps: {}", self.cfg.max_steps).map_err(|e| e.to_string())?;
        writeln!(w, "cpu_threads: {}", self.pool.current_num_threads())
            .map_err(|e| e.to_string())?;
        writeln!(w, "convection: {:?}", self.cfg.convection).map_err(|e| e.to_string())?;
        writeln!(w, "pressure_solver: {:?}", self.cfg.pressure_solver)
            .map_err(|e| e.to_string())?;
        Ok(())
    }
}

fn apply_velocity_boundaries(
    case: &Case,
    grid: &UniformGrid2D,
    solid: &Mask2D,
    time: f64,
    u: &mut Field2D,
    v: &mut Field2D,
) {
    for j in 0..grid.ny {
        let y = grid.u_face_y(j);
        match case.boundary_kind(Side::Left) {
            BoundaryKind::Velocity => {
                u[(0, j)] = case.boundary_velocity(Side::Left, 0.0, y, time).0
            }
            BoundaryKind::Symmetry => u[(0, j)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => u[(0, j)] = u[(1, j)],
        }
        match case.boundary_kind(Side::Right) {
            BoundaryKind::Velocity => {
                u[(grid.nx, j)] = case.boundary_velocity(Side::Right, grid.length, y, time).0
            }
            BoundaryKind::Symmetry => u[(grid.nx, j)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => u[(grid.nx, j)] = u[(grid.nx - 1, j)],
        }
    }

    for i in 0..grid.nx {
        let x = grid.v_face_x(i);
        match case.boundary_kind(Side::Bottom) {
            BoundaryKind::Velocity => {
                v[(i, 0)] = case.boundary_velocity(Side::Bottom, x, 0.0, time).1
            }
            BoundaryKind::Symmetry => v[(i, 0)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => v[(i, 0)] = v[(i, 1)],
        }
        match case.boundary_kind(Side::Top) {
            BoundaryKind::Velocity => {
                v[(i, grid.ny)] = case.boundary_velocity(Side::Top, x, grid.height, time).1
            }
            BoundaryKind::Symmetry => v[(i, grid.ny)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => v[(i, grid.ny)] = v[(i, grid.ny - 1)],
        }
    }

    // Tangential velocity conditions are represented through ghost values in the
    // momentum stencil. Here only blocked faces are explicitly zeroed.
    for j in 0..grid.ny {
        for i in 1..grid.nx {
            if !u_face_open(solid, grid.nx, i, j) {
                u[(i, j)] = 0.0;
            }
        }
    }
    for j in 1..grid.ny {
        for i in 0..grid.nx {
            if !v_face_open(solid, grid.ny, i, j) {
                v[(i, j)] = 0.0;
            }
        }
    }
}

fn u_face_open(solid: &Mask2D, nx: usize, i: usize, j: usize) -> bool {
    if i == 0 {
        !solid[(0, j)]
    } else if i == nx {
        !solid[(nx - 1, j)]
    } else {
        !solid[(i - 1, j)] && !solid[(i, j)]
    }
}

fn v_face_open(solid: &Mask2D, ny: usize, i: usize, j: usize) -> bool {
    if j == 0 {
        !solid[(i, 0)]
    } else if j == ny {
        !solid[(i, ny - 1)]
    } else {
        !solid[(i, j - 1)] && !solid[(i, j)]
    }
}

fn pressure_outlet_cell(case: &Case, solid: &Mask2D, nx: usize, i: usize, j: usize) -> bool {
    i + 1 == nx
        && !solid[(i, j)]
        && matches!(
            case.boundary_kind(Side::Right),
            BoundaryKind::PressureOutlet { .. }
        )
}

struct MomentumStencil<'a> {
    solid: &'a Mask2D,
    case: &'a Case,
    grid: &'a UniformGrid2D,
}

impl MomentumStencil<'_> {
    fn u_neighbor(&self, u: &[f64], i: usize, j: usize, di: isize, dj: isize, center: f64) -> f64 {
        let ii = i as isize + di;
        let jj = j as isize + dj;
        if ii >= 0 && ii <= self.grid.nx as isize && jj >= 0 && jj < self.grid.ny as isize {
            let iu = ii as usize;
            let ju = jj as usize;
            return if u_face_open(self.solid, self.grid.nx, iu, ju) {
                u[iu + (self.grid.nx + 1) * ju]
            } else {
                -center
            };
        }

        if jj < 0 {
            return tangential_ghost_u(self.case, Side::Bottom, center, self.grid.u_face_x(i));
        }
        if jj >= self.grid.ny as isize {
            return tangential_ghost_u(self.case, Side::Top, center, self.grid.u_face_x(i));
        }
        center
    }

    fn v_neighbor(&self, v: &[f64], i: usize, j: usize, di: isize, dj: isize, center: f64) -> f64 {
        let ii = i as isize + di;
        let jj = j as isize + dj;
        if ii >= 0 && ii < self.grid.nx as isize && jj >= 0 && jj <= self.grid.ny as isize {
            let iu = ii as usize;
            let ju = jj as usize;
            return if v_face_open(self.solid, self.grid.ny, iu, ju) {
                v[iu + self.grid.nx * ju]
            } else {
                -center
            };
        }

        if ii < 0 {
            return tangential_ghost_v(self.case, Side::Left, center, self.grid.v_face_y(j));
        }
        if ii >= self.grid.nx as isize {
            return tangential_ghost_v(self.case, Side::Right, center, self.grid.v_face_y(j));
        }
        center
    }
}

fn tangential_ghost_u(case: &Case, side: Side, center: f64, x: f64) -> f64 {
    match case.boundary_kind(side) {
        BoundaryKind::Velocity => {
            let y = if side == Side::Bottom {
                0.0
            } else {
                case.domain().1
            };
            let wall_u = case.boundary_velocity(side, x, y, 0.0).0;
            2.0 * wall_u - center
        }
        BoundaryKind::Symmetry | BoundaryKind::PressureOutlet { .. } => center,
    }
}

fn tangential_ghost_v(case: &Case, side: Side, center: f64, y: f64) -> f64 {
    match case.boundary_kind(side) {
        BoundaryKind::Velocity => {
            let x = if side == Side::Left {
                0.0
            } else {
                case.domain().0
            };
            let wall_v = case.boundary_velocity(side, x, y, 0.0).1;
            2.0 * wall_v - center
        }
        BoundaryKind::Symmetry | BoundaryKind::PressureOutlet { .. } => center,
    }
}

fn transported(scheme: ConvectionScheme, face_velocity: f64, left: f64, right: f64) -> f64 {
    match scheme {
        ConvectionScheme::FirstOrderUpwind => {
            if face_velocity >= 0.0 {
                left
            } else {
                right
            }
        }
        ConvectionScheme::Central => 0.5 * (left + right),
    }
}

fn derivative_x_slice(
    values: &[f64],
    solid: &Mask2D,
    nx: usize,
    i: usize,
    j: usize,
    dx: f64,
) -> f64 {
    let center = values[i + nx * j];
    let east = if i + 1 < nx && !solid[(i + 1, j)] {
        values[i + 1 + nx * j]
    } else {
        center
    };
    let west = if i > 0 && !solid[(i - 1, j)] {
        values[i - 1 + nx * j]
    } else {
        center
    };
    if i > 0 && i + 1 < nx && !solid[(i - 1, j)] && !solid[(i + 1, j)] {
        (east - west) / (2.0 * dx)
    } else if i + 1 < nx && !solid[(i + 1, j)] {
        (east - center) / dx
    } else if i > 0 && !solid[(i - 1, j)] {
        (center - west) / dx
    } else {
        0.0
    }
}

fn derivative_y_slice(
    values: &[f64],
    solid: &Mask2D,
    nx: usize,
    i: usize,
    j: usize,
    dy: f64,
) -> f64 {
    let center = values[i + nx * j];
    let north = if j + 1 < solid.ny() && !solid[(i, j + 1)] {
        values[i + nx * (j + 1)]
    } else {
        center
    };
    let south = if j > 0 && !solid[(i, j - 1)] {
        values[i + nx * (j - 1)]
    } else {
        center
    };
    if j > 0 && j + 1 < solid.ny() && !solid[(i, j - 1)] && !solid[(i, j + 1)] {
        (north - south) / (2.0 * dy)
    } else if j + 1 < solid.ny() && !solid[(i, j + 1)] {
        (north - center) / dy
    } else if j > 0 && !solid[(i, j - 1)] {
        (center - south) / dy
    } else {
        0.0
    }
}

fn max_field_difference(a: &Field2D, b: &Field2D) -> f64 {
    a.as_slice()
        .iter()
        .zip(b.as_slice())
        .fold(0.0_f64, |m, (&x, &y)| m.max((x - y).abs()))
}

fn project_zero_mean<F>(
    pool: &ThreadPool,
    values: &mut [f64],
    solid: &Mask2D,
    nx: usize,
    ny: usize,
    is_dirichlet: F,
) where
    F: Fn(usize, usize) -> bool + Sync,
{
    let (sum, count) = pool.install(|| {
        (0..ny)
            .into_par_iter()
            .map(|j| {
                let mut row_sum = 0.0;
                let mut row_count = 0_usize;
                for i in 0..nx {
                    if !solid[(i, j)] && !is_dirichlet(i, j) {
                        row_sum += values[i + nx * j];
                        row_count += 1;
                    }
                }
                (row_sum, row_count)
            })
            .reduce(|| (0.0, 0_usize), |a, b| (a.0 + b.0, a.1 + b.1))
    });
    if count == 0 {
        return;
    }
    let mean = sum / count as f64;
    pool.install(|| {
        values.par_chunks_mut(nx).enumerate().for_each(|(j, row)| {
            for i in 0..nx {
                if !solid[(i, j)] && !is_dirichlet(i, j) {
                    row[i] -= mean;
                }
            }
        });
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cases::CavityCase;

    #[test]
    fn cavity_configuration_builds() {
        let cfg = SimulationConfig {
            case: Case::LidDrivenCavity(CavityCase::default()),
            nx: 16,
            ny: 16,
            dt: 1.0e-3,
            max_steps: 2,
            t_end: 0.002,
            convection: ConvectionScheme::FirstOrderUpwind,
            pressure_solver: PressureSolverKind::Pcg,
            pressure_max_iters: 10,
            pressure_tolerance: 1.0e-4,
            pressure_omega: 1.5,
            print_every: 1,
            output_every: 1,
            frame_every: 1,
            steady_tolerance: 1.0e-8,
            minimum_steps: 1,
            threads: 1,
            output_dir: PathBuf::from("target/test-output"),
        };
        let solver = IncompressibleSolver::new(cfg);
        assert!(solver.is_ok());
    }
}
