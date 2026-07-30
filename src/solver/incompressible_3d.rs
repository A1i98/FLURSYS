use crate::field::Field3D;
use crate::grid::UniformGrid3D;
use crate::output;
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// Settings for the first real 3D solver: a cubic or rectangular lid-driven cavity.
#[derive(Clone, Debug)]
pub struct LidDrivenCavity3DConfig {
    pub nx: usize,
    pub ny: usize,
    pub nz: usize,
    pub length: f64,
    pub height: f64,
    pub depth: f64,
    pub density: f64,
    pub lid_velocity: f64,
    pub reynolds: f64,
    pub dt: f64,
    pub max_steps: usize,
    pub pressure_max_iters: usize,
    pub pressure_tolerance: f64,
    pub pressure_omega: f64,
    pub output_dir: PathBuf,
}

impl Default for LidDrivenCavity3DConfig {
    fn default() -> Self {
        Self {
            nx: 32,
            ny: 32,
            nz: 32,
            length: 1.0,
            height: 1.0,
            depth: 1.0,
            density: 1.0,
            lid_velocity: 1.0,
            reynolds: 100.0,
            dt: 5.0e-4,
            max_steps: 10_000,
            pressure_max_iters: 2_000,
            pressure_tolerance: 1.0e-6,
            pressure_omega: 1.7,
            output_dir: PathBuf::from("results/cavity-3d"),
        }
    }
}

impl LidDrivenCavity3DConfig {
    fn validate(&self) -> Result<(), String> {
        UniformGrid3D::new(
            self.nx,
            self.ny,
            self.nz,
            self.length,
            self.height,
            self.depth,
        )?;
        if !(self.density.is_finite() && self.density > 0.0) {
            return Err("density must be finite and positive".to_string());
        }
        if !(self.lid_velocity.is_finite() && self.lid_velocity > 0.0) {
            return Err("lid_velocity must be finite and positive".to_string());
        }
        if !(self.reynolds.is_finite() && self.reynolds > 0.0) {
            return Err("reynolds must be finite and positive".to_string());
        }
        if !(self.dt.is_finite() && self.dt > 0.0) {
            return Err("dt must be finite and positive".to_string());
        }
        if self.max_steps == 0 || self.pressure_max_iters == 0 {
            return Err("iteration limits must be positive".to_string());
        }
        if !(self.pressure_tolerance.is_finite() && self.pressure_tolerance > 0.0) {
            return Err("pressure_tolerance must be finite and positive".to_string());
        }
        if !(self.pressure_omega.is_finite()
            && self.pressure_omega > 0.0
            && self.pressure_omega < 2.0)
        {
            return Err("pressure_omega must lie between 0 and 2".to_string());
        }
        Ok(())
    }

    fn viscosity(&self) -> f64 {
        self.lid_velocity * self.length / self.reynolds
    }
}

#[derive(Clone, Debug)]
pub struct RunSummary3D {
    pub steps: usize,
    pub final_time: f64,
    pub max_divergence: f64,
    pub pressure_residual: f64,
    pub max_speed: f64,
    pub elapsed: Duration,
}

/// Three-dimensional, staggered-grid projection solver for a lid-driven cavity.
/// Pressure is cell-centred; u, v, and w live on their corresponding cell faces.
pub struct LidDrivenCavity3DSolver {
    cfg: LidDrivenCavity3DConfig,
    grid: UniformGrid3D,
    pressure: Field3D,
    rhs: Field3D,
    u: Field3D,
    v: Field3D,
    w: Field3D,
    u_star: Field3D,
    v_star: Field3D,
    w_star: Field3D,
    step: usize,
}

impl LidDrivenCavity3DSolver {
    pub fn new(cfg: LidDrivenCavity3DConfig) -> Result<Self, String> {
        cfg.validate()?;
        let grid = UniformGrid3D::new(cfg.nx, cfg.ny, cfg.nz, cfg.length, cfg.height, cfg.depth)?;
        Ok(Self {
            pressure: Field3D::new(grid.nx, grid.ny, grid.nz, 0.0),
            rhs: Field3D::new(grid.nx, grid.ny, grid.nz, 0.0),
            u: Field3D::new(grid.nx + 1, grid.ny, grid.nz, 0.0),
            v: Field3D::new(grid.nx, grid.ny + 1, grid.nz, 0.0),
            w: Field3D::new(grid.nx, grid.ny, grid.nz + 1, 0.0),
            u_star: Field3D::new(grid.nx + 1, grid.ny, grid.nz, 0.0),
            v_star: Field3D::new(grid.nx, grid.ny + 1, grid.nz, 0.0),
            w_star: Field3D::new(grid.nx, grid.ny, grid.nz + 1, 0.0),
            cfg,
            grid,
            step: 0,
        })
    }

    pub fn run(&mut self) -> Result<RunSummary3D, String> {
        output::ensure_output_tree(&self.cfg.output_dir)?;
        let started = Instant::now();
        let mut pressure_residual = f64::INFINITY;
        for _ in 0..self.cfg.max_steps {
            pressure_residual = self.advance_one_step()?;
        }
        let max_divergence = self.max_divergence();
        self.ensure_finite()?;
        output::write_legacy_vtk_3d(
            &self.cfg.output_dir.join("field.vtk"),
            "FLURSYS 3D lid-driven cavity",
            &self.grid,
            &self.pressure,
            &self.u,
            &self.v,
            &self.w,
        )?;
        Ok(RunSummary3D {
            steps: self.step,
            final_time: self.step as f64 * self.cfg.dt,
            max_divergence,
            pressure_residual,
            max_speed: self.max_speed(),
            elapsed: started.elapsed(),
        })
    }

    fn advance_one_step(&mut self) -> Result<f64, String> {
        self.predict_momentum();
        self.build_pressure_rhs();
        let residual = self.solve_pressure_poisson()?;
        self.correct_velocity();
        self.step += 1;
        self.ensure_finite()?;
        Ok(residual)
    }

    fn predict_momentum(&mut self) {
        let nu = self.cfg.viscosity();
        let dt = self.cfg.dt;
        self.u_star.fill(0.0);
        self.v_star.fill(0.0);
        self.w_star.fill(0.0);

        for k in 0..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 1..self.grid.nx {
                    let center = self.u[(i, j, k)];
                    let east = self.u_neighbor(i, j, k, 1, 0, 0, center);
                    let west = self.u_neighbor(i, j, k, -1, 0, 0, center);
                    let north = self.u_neighbor(i, j, k, 0, 1, 0, center);
                    let south = self.u_neighbor(i, j, k, 0, -1, 0, center);
                    let front = self.u_neighbor(i, j, k, 0, 0, 1, center);
                    let back = self.u_neighbor(i, j, k, 0, 0, -1, center);
                    let v = 0.25
                        * (self.v[(i - 1, j, k)]
                            + self.v[(i, j, k)]
                            + self.v[(i - 1, j + 1, k)]
                            + self.v[(i, j + 1, k)]);
                    let w = 0.25
                        * (self.w[(i - 1, j, k)]
                            + self.w[(i, j, k)]
                            + self.w[(i - 1, j, k + 1)]
                            + self.w[(i, j, k + 1)]);
                    self.u_star[(i, j, k)] = center
                        + dt * (-center * (east - west) / (2.0 * self.grid.dx)
                            - v * (north - south) / (2.0 * self.grid.dy)
                            - w * (front - back) / (2.0 * self.grid.dz)
                            + nu * ((east - 2.0 * center + west) / self.grid.dx.powi(2)
                                + (north - 2.0 * center + south) / self.grid.dy.powi(2)
                                + (front - 2.0 * center + back) / self.grid.dz.powi(2)));
                }
            }
        }

        for k in 0..self.grid.nz {
            for j in 1..self.grid.ny {
                for i in 0..self.grid.nx {
                    let center = self.v[(i, j, k)];
                    let east = self.v_neighbor(i, j, k, 1, 0, 0, center);
                    let west = self.v_neighbor(i, j, k, -1, 0, 0, center);
                    let north = self.v_neighbor(i, j, k, 0, 1, 0, center);
                    let south = self.v_neighbor(i, j, k, 0, -1, 0, center);
                    let front = self.v_neighbor(i, j, k, 0, 0, 1, center);
                    let back = self.v_neighbor(i, j, k, 0, 0, -1, center);
                    let u = 0.25
                        * (self.u[(i, j - 1, k)]
                            + self.u[(i + 1, j - 1, k)]
                            + self.u[(i, j, k)]
                            + self.u[(i + 1, j, k)]);
                    let w = 0.25
                        * (self.w[(i, j - 1, k)]
                            + self.w[(i, j, k)]
                            + self.w[(i, j - 1, k + 1)]
                            + self.w[(i, j, k + 1)]);
                    self.v_star[(i, j, k)] = center
                        + dt * (-u * (east - west) / (2.0 * self.grid.dx)
                            - center * (north - south) / (2.0 * self.grid.dy)
                            - w * (front - back) / (2.0 * self.grid.dz)
                            + nu * ((east - 2.0 * center + west) / self.grid.dx.powi(2)
                                + (north - 2.0 * center + south) / self.grid.dy.powi(2)
                                + (front - 2.0 * center + back) / self.grid.dz.powi(2)));
                }
            }
        }

        for k in 1..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 0..self.grid.nx {
                    let center = self.w[(i, j, k)];
                    let east = self.w_neighbor(i, j, k, 1, 0, 0, center);
                    let west = self.w_neighbor(i, j, k, -1, 0, 0, center);
                    let north = self.w_neighbor(i, j, k, 0, 1, 0, center);
                    let south = self.w_neighbor(i, j, k, 0, -1, 0, center);
                    let front = self.w_neighbor(i, j, k, 0, 0, 1, center);
                    let back = self.w_neighbor(i, j, k, 0, 0, -1, center);
                    let u = 0.25
                        * (self.u[(i, j, k - 1)]
                            + self.u[(i + 1, j, k - 1)]
                            + self.u[(i, j, k)]
                            + self.u[(i + 1, j, k)]);
                    let v = 0.25
                        * (self.v[(i, j, k - 1)]
                            + self.v[(i, j + 1, k - 1)]
                            + self.v[(i, j, k)]
                            + self.v[(i, j + 1, k)]);
                    self.w_star[(i, j, k)] = center
                        + dt * (-u * (east - west) / (2.0 * self.grid.dx)
                            - v * (north - south) / (2.0 * self.grid.dy)
                            - center * (front - back) / (2.0 * self.grid.dz)
                            + nu * ((east - 2.0 * center + west) / self.grid.dx.powi(2)
                                + (north - 2.0 * center + south) / self.grid.dy.powi(2)
                                + (front - 2.0 * center + back) / self.grid.dz.powi(2)));
                }
            }
        }
    }

    fn build_pressure_rhs(&mut self) {
        let scale = self.cfg.density / self.cfg.dt;
        for k in 0..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 0..self.grid.nx {
                    self.rhs[(i, j, k)] = scale
                        * (self.u_star[(i + 1, j, k)] - self.u_star[(i, j, k)])
                        / self.grid.dx
                        + scale * (self.v_star[(i, j + 1, k)] - self.v_star[(i, j, k)])
                            / self.grid.dy
                        + scale * (self.w_star[(i, j, k + 1)] - self.w_star[(i, j, k)])
                            / self.grid.dz;
                }
            }
        }
        self.rhs[(0, 0, 0)] = 0.0;
    }

    fn solve_pressure_poisson(&mut self) -> Result<f64, String> {
        let ix2 = 1.0 / self.grid.dx.powi(2);
        let iy2 = 1.0 / self.grid.dy.powi(2);
        let iz2 = 1.0 / self.grid.dz.powi(2);
        let diagonal = 2.0 * (ix2 + iy2 + iz2);
        let mut residual = f64::INFINITY;
        for _ in 0..self.cfg.pressure_max_iters {
            for k in 0..self.grid.nz {
                for j in 0..self.grid.ny {
                    for i in 0..self.grid.nx {
                        if (i, j, k) == (0, 0, 0) {
                            self.pressure[(i, j, k)] = 0.0;
                            continue;
                        }
                        let center = self.pressure[(i, j, k)];
                        let sum = ix2
                            * (self.pressure_neighbor(i, j, k, 1, 0, 0, center)
                                + self.pressure_neighbor(i, j, k, -1, 0, 0, center))
                            + iy2
                                * (self.pressure_neighbor(i, j, k, 0, 1, 0, center)
                                    + self.pressure_neighbor(i, j, k, 0, -1, 0, center))
                            + iz2
                                * (self.pressure_neighbor(i, j, k, 0, 0, 1, center)
                                    + self.pressure_neighbor(i, j, k, 0, 0, -1, center));
                        let candidate = (sum - self.rhs[(i, j, k)]) / diagonal;
                        self.pressure[(i, j, k)] +=
                            self.cfg.pressure_omega * (candidate - self.pressure[(i, j, k)]);
                    }
                }
            }
            residual = self.pressure_residual();
            if !residual.is_finite() {
                return Err("3D pressure solver produced a non-finite residual".to_string());
            }
            if residual < self.cfg.pressure_tolerance {
                break;
            }
        }
        Ok(residual)
    }

    fn correct_velocity(&mut self) {
        let scale = self.cfg.dt / self.cfg.density;
        for k in 0..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 1..self.grid.nx {
                    self.u[(i, j, k)] = self.u_star[(i, j, k)]
                        - scale * (self.pressure[(i, j, k)] - self.pressure[(i - 1, j, k)])
                            / self.grid.dx;
                }
            }
        }
        for k in 0..self.grid.nz {
            for j in 1..self.grid.ny {
                for i in 0..self.grid.nx {
                    self.v[(i, j, k)] = self.v_star[(i, j, k)]
                        - scale * (self.pressure[(i, j, k)] - self.pressure[(i, j - 1, k)])
                            / self.grid.dy;
                }
            }
        }
        for k in 1..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 0..self.grid.nx {
                    self.w[(i, j, k)] = self.w_star[(i, j, k)]
                        - scale * (self.pressure[(i, j, k)] - self.pressure[(i, j, k - 1)])
                            / self.grid.dz;
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn u_neighbor(
        &self,
        i: usize,
        j: usize,
        k: usize,
        di: isize,
        dj: isize,
        dk: isize,
        c: f64,
    ) -> f64 {
        let ni = i as isize + di;
        let nj = j as isize + dj;
        let nk = k as isize + dk;
        if ni >= 0
            && ni <= self.grid.nx as isize
            && nj >= 0
            && nj < self.grid.ny as isize
            && nk >= 0
            && nk < self.grid.nz as isize
        {
            return self.u[(ni as usize, nj as usize, nk as usize)];
        }
        if nj >= self.grid.ny as isize {
            2.0 * self.cfg.lid_velocity - c
        } else {
            -c
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn v_neighbor(
        &self,
        i: usize,
        j: usize,
        k: usize,
        di: isize,
        dj: isize,
        dk: isize,
        c: f64,
    ) -> f64 {
        let ni = i as isize + di;
        let nj = j as isize + dj;
        let nk = k as isize + dk;
        if ni >= 0
            && ni < self.grid.nx as isize
            && nj >= 0
            && nj <= self.grid.ny as isize
            && nk >= 0
            && nk < self.grid.nz as isize
        {
            self.v[(ni as usize, nj as usize, nk as usize)]
        } else {
            -c
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn w_neighbor(
        &self,
        i: usize,
        j: usize,
        k: usize,
        di: isize,
        dj: isize,
        dk: isize,
        c: f64,
    ) -> f64 {
        let ni = i as isize + di;
        let nj = j as isize + dj;
        let nk = k as isize + dk;
        if ni >= 0
            && ni < self.grid.nx as isize
            && nj >= 0
            && nj < self.grid.ny as isize
            && nk >= 0
            && nk <= self.grid.nz as isize
        {
            self.w[(ni as usize, nj as usize, nk as usize)]
        } else {
            -c
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn pressure_neighbor(
        &self,
        i: usize,
        j: usize,
        k: usize,
        di: isize,
        dj: isize,
        dk: isize,
        c: f64,
    ) -> f64 {
        let ni = i as isize + di;
        let nj = j as isize + dj;
        let nk = k as isize + dk;
        if ni >= 0
            && ni < self.grid.nx as isize
            && nj >= 0
            && nj < self.grid.ny as isize
            && nk >= 0
            && nk < self.grid.nz as isize
        {
            self.pressure[(ni as usize, nj as usize, nk as usize)]
        } else {
            c
        }
    }

    fn pressure_residual(&self) -> f64 {
        let ix2 = 1.0 / self.grid.dx.powi(2);
        let iy2 = 1.0 / self.grid.dy.powi(2);
        let iz2 = 1.0 / self.grid.dz.powi(2);
        let mut maximum = 0.0_f64;
        for k in 0..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 0..self.grid.nx {
                    if (i, j, k) == (0, 0, 0) {
                        continue;
                    }
                    let center = self.pressure[(i, j, k)];
                    let laplacian = ix2
                        * (self.pressure_neighbor(i, j, k, 1, 0, 0, center) - 2.0 * center
                            + self.pressure_neighbor(i, j, k, -1, 0, 0, center))
                        + iy2
                            * (self.pressure_neighbor(i, j, k, 0, 1, 0, center) - 2.0 * center
                                + self.pressure_neighbor(i, j, k, 0, -1, 0, center))
                        + iz2
                            * (self.pressure_neighbor(i, j, k, 0, 0, 1, center) - 2.0 * center
                                + self.pressure_neighbor(i, j, k, 0, 0, -1, center));
                    maximum = maximum.max((laplacian - self.rhs[(i, j, k)]).abs());
                }
            }
        }
        maximum
    }

    fn max_divergence(&self) -> f64 {
        let mut maximum = 0.0_f64;
        for k in 0..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 0..self.grid.nx {
                    let divergence = (self.u[(i + 1, j, k)] - self.u[(i, j, k)]) / self.grid.dx
                        + (self.v[(i, j + 1, k)] - self.v[(i, j, k)]) / self.grid.dy
                        + (self.w[(i, j, k + 1)] - self.w[(i, j, k)]) / self.grid.dz;
                    maximum = maximum.max(divergence.abs());
                }
            }
        }
        maximum
    }

    fn max_speed(&self) -> f64 {
        let mut maximum = 0.0_f64;
        for k in 0..self.grid.nz {
            for j in 0..self.grid.ny {
                for i in 0..self.grid.nx {
                    let u = 0.5 * (self.u[(i, j, k)] + self.u[(i + 1, j, k)]);
                    let v = 0.5 * (self.v[(i, j, k)] + self.v[(i, j + 1, k)]);
                    let w = 0.5 * (self.w[(i, j, k)] + self.w[(i, j, k + 1)]);
                    maximum = maximum.max((u * u + v * v + w * w).sqrt());
                }
            }
        }
        maximum
    }

    fn ensure_finite(&self) -> Result<(), String> {
        for (name, field) in [
            ("pressure", &self.pressure),
            ("u", &self.u),
            ("v", &self.v),
            ("w", &self.w),
        ] {
            if let Some((index, value)) = field
                .as_slice()
                .iter()
                .copied()
                .enumerate()
                .find(|(_, value)| !value.is_finite())
            {
                return Err(format!(
                    "Non-finite 3D {name} value at flat index {index}: {value}"
                ));
            }
        }
        Ok(())
    }
}
