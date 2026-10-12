use super::*;

#[derive(Default)]
pub(super) struct PressureStencil2D {
    pub(super) diagonal: Vec<f64>,
    pub(super) neighbours: Vec<u8>,
    pub(super) identity: Vec<bool>,
    pub(super) dic_inverse_diagonal: Vec<f64>,
    pub(super) dic_supported: bool,
}

impl PressureStencil2D {
    const WEST: u8 = 1 << 0;
    const EAST: u8 = 1 << 1;
    const SOUTH: u8 = 1 << 2;
    const NORTH: u8 = 1 << 3;

    pub(super) fn new(
        grid: &UniformGrid2D,
        solid: &Mask2D,
        case: &Case,
        boundaries: &SolverBoundaryOverrides,
    ) -> Self {
        let nx = grid.nx;
        let ny = grid.ny;
        let idx2 = 1.0 / (grid.dx * grid.dx);
        let idy2 = 1.0 / (grid.dy * grid.dy);
        let mut stencil = Self {
            diagonal: vec![0.0; nx * ny],
            neighbours: vec![0; nx * ny],
            identity: vec![false; nx * ny],
            dic_inverse_diagonal: vec![1.0; nx * ny],
            dic_supported: false,
        };

        for j in 0..ny {
            for i in 0..nx {
                let k = i + nx * j;
                if solid[(i, j)] || pressure_outlet_cell(case, boundaries, solid, nx, i, j) {
                    stencil.identity[k] = true;
                    stencil.diagonal[k] = 1.0;
                    continue;
                }

                let mut diagonal = 0.0;
                let mut neighbours = 0;
                if i + 1 < nx && !solid[(i + 1, j)] {
                    diagonal += idx2;
                    neighbours |= Self::EAST;
                }
                if i > 0 && !solid[(i - 1, j)] {
                    diagonal += idx2;
                    neighbours |= Self::WEST;
                }
                if j + 1 < ny && !solid[(i, j + 1)] {
                    diagonal += idy2;
                    neighbours |= Self::NORTH;
                }
                if j > 0 && !solid[(i, j - 1)] {
                    diagonal += idy2;
                    neighbours |= Self::SOUTH;
                }
                stencil.diagonal[k] = diagonal;
                stencil.neighbours[k] = neighbours;
            }
        }

        let has_solid = solid.as_slice().iter().any(|&cell| cell);
        let has_pressure_outlet = matches!(
            boundaries.kind(case, Side::Right),
            BoundaryKind::PressureOutlet { .. }
        );
        stencil.dic_supported = !has_solid && !has_pressure_outlet;
        if stencil.dic_supported {
            for k in 0..nx * ny {
                let mut pivot = stencil.diagonal[k] + if k == 0 { 1.0 } else { 0.0 };
                let mask = stencil.neighbours[k];
                if mask & Self::WEST != 0 {
                    pivot -= idx2 * idx2 * stencil.dic_inverse_diagonal[k - 1];
                }
                if mask & Self::SOUTH != 0 {
                    pivot -= idy2 * idy2 * stencil.dic_inverse_diagonal[k - nx];
                }
                if !pivot.is_finite() || pivot <= 0.0 {
                    stencil.dic_supported = false;
                    break;
                }
                stencil.dic_inverse_diagonal[k] = 1.0 / pivot;
            }
        }
        stencil
    }
}

/// Reusable vectors for the pressure PCG solve. Keeping these allocations alive
/// avoids allocator churn on every outer SIMPLE/projection iteration.
#[derive(Default)]
pub(super) struct PcgWorkspace {
    x: Vec<f64>,
    b: Vec<f64>,
    r: Vec<f64>,
    z: Vec<f64>,
    direction: Vec<f64>,
    q: Vec<f64>,
}

impl PcgWorkspace {
    pub(super) fn resize(&mut self, len: usize) {
        self.x.resize(len, 0.0);
        self.b.resize(len, 0.0);
        self.r.resize(len, 0.0);
        self.z.resize(len, 0.0);
        self.direction.resize(len, 0.0);
        self.q.resize(len, 0.0);
    }
}

impl IncompressibleSolver {
    pub(super) fn build_pressure_rhs(&mut self, step_dt: f64) {
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
        let correction_scale = if cfg.coupling == PressureVelocityCoupling::Simple {
            cfg.velocity_relaxation
        } else {
            1.0
        };
        let rho_over_dt = cfg.case.density() / (correction_scale * step_dt);
        let u_values = u_star.as_slice();
        let v_values = v_star.as_slice();
        pool.install(|| {
            rhs.as_mut_slice()
                .par_chunks_mut(nx)
                .enumerate()
                .for_each(|(j, row)| {
                    for i in 0..nx {
                        if solid[(i, j)]
                            || pressure_outlet_cell(
                                &cfg.case,
                                &cfg.boundary_overrides,
                                solid,
                                nx,
                                i,
                                j,
                            )
                        {
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

    pub(super) fn solve_pressure_poisson(&mut self) -> Result<(f64, usize), String> {
        match self.cfg.pressure_solver {
            PressureSolverKind::Pcg => self.solve_pressure_pcg(false),
            PressureSolverKind::PcgDic | PressureSolverKind::PcgMg => self.solve_pressure_pcg(true),
            PressureSolverKind::Sor => self.solve_pressure_sor(),
        }
    }

    pub(super) fn solve_pressure_sor(&mut self) -> Result<(f64, usize), String> {
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

            if self.pressure_reference_required() {
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

    pub(super) fn solve_pressure_pcg(&mut self, use_dic: bool) -> Result<(f64, usize), String> {
        let n = self.grid.nx * self.grid.ny;
        let mut workspace = std::mem::take(&mut self.pcg_workspace);
        workspace.resize(n);
        let result = {
            let PcgWorkspace {
                ref mut x,
                ref mut b,
                ref mut r,
                ref mut z,
                ref mut direction,
                ref mut q,
            } = &mut workspace;
            x.copy_from_slice(self.p.as_slice());
            (|| {
                let nx = self.grid.nx;
                let solid = &self.solid;
                let case = &self.cfg.case;
                let boundaries = &self.cfg.boundary_overrides;
                let rhs = self.rhs.as_slice();
                let outlet_pressure = self.outlet_pressure();
                if self.pool.current_num_threads() == 1 {
                    for j in 0..self.grid.ny {
                        for i in 0..nx {
                            let k = i + nx * j;
                            if solid[(i, j)] {
                                x[k] = 0.0;
                                b[k] = 0.0;
                            } else if pressure_outlet_cell(case, boundaries, solid, nx, i, j) {
                                x[k] = outlet_pressure;
                                b[k] = outlet_pressure;
                            } else {
                                b[k] = -rhs[k];
                            }
                        }
                    }
                } else {
                    self.pool.install(|| {
                        x.par_chunks_mut(nx)
                            .zip(b.par_chunks_mut(nx))
                            .enumerate()
                            .for_each(|(j, (x_row, b_row))| {
                                for i in 0..nx {
                                    let k = i + nx * j;
                                    if solid[(i, j)]
                                        || pressure_outlet_cell(case, boundaries, solid, nx, i, j)
                                    {
                                        let value = if pressure_outlet_cell(
                                            case, boundaries, solid, nx, i, j,
                                        ) {
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
                }

                if self.pressure_reference_required() {
                    self.project_pressure_zero_mean(x);
                    self.project_pressure_zero_mean(b);
                }

                self.apply_pressure_operator(x, q);
                if self.pool.current_num_threads() == 1 {
                    for ((r_value, &b_value), &q_value) in r.iter_mut().zip(b.iter()).zip(q.iter())
                    {
                        *r_value = b_value - q_value;
                    }
                } else {
                    self.pool.install(|| {
                        r.par_iter_mut()
                            .zip(b.par_iter())
                            .zip(q.par_iter())
                            .for_each(|((r_value, &b_value), &q_value)| {
                                *r_value = b_value - q_value
                            });
                    });
                }
                if self.pressure_reference_required() {
                    self.project_pressure_zero_mean(r);
                }

                let mut residual = self.parallel_max_abs(r);
                if residual < self.cfg.pressure_tolerance {
                    self.p.as_mut_slice().copy_from_slice(x);
                    return Ok((residual, 0));
                }

                self.apply_pressure_preconditioner(r, z, use_dic);
                if self.pressure_reference_required() {
                    self.project_pressure_zero_mean(z);
                }
                direction.copy_from_slice(z);
                let mut rz_old = self.parallel_dot(r, z);

                let mut iterations = 0_usize;
                for iteration in 0..self.cfg.pressure_max_iters {
                    iterations = iteration + 1;
                    self.apply_pressure_operator(direction, q);
                    let denominator = self.parallel_dot(direction, q);
                    if denominator.abs() < 1.0e-30 || !denominator.is_finite() {
                        return Err(
                            "PCG pressure solver encountered a singular search direction"
                                .to_string(),
                        );
                    }
                    let alpha = rz_old / denominator;
                    if !alpha.is_finite() {
                        return Err(
                            "PCG pressure solver produced a non-finite step length".to_string()
                        );
                    }
                    if self.pool.current_num_threads() == 1 {
                        for (((x_value, r_value), &direction_value), &q_value) in x
                            .iter_mut()
                            .zip(r.iter_mut())
                            .zip(direction.iter())
                            .zip(q.iter())
                        {
                            *x_value += alpha * direction_value;
                            *r_value -= alpha * q_value;
                        }
                    } else {
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
                    }
                    if self.pressure_reference_required() {
                        self.project_pressure_zero_mean(x);
                        self.project_pressure_zero_mean(r);
                    }

                    residual = self.parallel_max_abs(r);
                    if !residual.is_finite() {
                        return Err(
                            "PCG pressure solver produced a non-finite residual".to_string()
                        );
                    }
                    if residual < self.cfg.pressure_tolerance {
                        break;
                    }

                    self.apply_pressure_preconditioner(r, z, use_dic);
                    if self.pressure_reference_required() {
                        self.project_pressure_zero_mean(z);
                    }
                    let rz_new = self.parallel_dot(r, z);
                    if rz_old.abs() < 1.0e-30 {
                        break;
                    }
                    let beta = rz_new / rz_old;
                    if self.pool.current_num_threads() == 1 {
                        for (direction_value, &z_value) in direction.iter_mut().zip(z.iter()) {
                            *direction_value = z_value + beta * *direction_value;
                        }
                    } else {
                        self.pool.install(|| {
                            direction.par_iter_mut().zip(z.par_iter()).for_each(
                                |(direction_value, &z_value)| {
                                    *direction_value = z_value + beta * *direction_value;
                                },
                            );
                        });
                    }
                    if self.pressure_reference_required() {
                        self.project_pressure_zero_mean(direction);
                    }
                    rz_old = rz_new;
                }

                let case = &self.cfg.case;
                let boundaries = &self.cfg.boundary_overrides;
                let solid = &self.solid;
                let outlet_pressure = self.outlet_pressure();
                if self.pool.current_num_threads() == 1 {
                    for j in 0..self.grid.ny {
                        for i in 0..nx {
                            let k = i + nx * j;
                            self.p[(i, j)] = if solid[(i, j)] {
                                0.0
                            } else if pressure_outlet_cell(case, boundaries, solid, nx, i, j) {
                                outlet_pressure
                            } else {
                                x[k]
                            };
                        }
                    }
                } else {
                    self.pool.install(|| {
                        self.p
                            .as_mut_slice()
                            .par_chunks_mut(nx)
                            .enumerate()
                            .for_each(|(j, row)| {
                                for i in 0..nx {
                                    row[i] = if solid[(i, j)] {
                                        0.0
                                    } else if pressure_outlet_cell(
                                        case, boundaries, solid, nx, i, j,
                                    ) {
                                        outlet_pressure
                                    } else {
                                        x[i + nx * j]
                                    };
                                }
                            });
                    });
                }
                Ok((residual, iterations))
            })()
        };
        self.pcg_workspace = workspace;
        result
    }

    pub(super) fn apply_pressure_operator(&self, x: &[f64], result: &mut [f64]) {
        let idx2 = 1.0 / (self.grid.dx * self.grid.dx);
        let idy2 = 1.0 / (self.grid.dy * self.grid.dy);
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let diagonal = &self.pressure_stencil.diagonal;
        let neighbours = &self.pressure_stencil.neighbours;
        let identity = &self.pressure_stencil.identity;
        if self.pool.current_num_threads() == 1 {
            for j in 0..ny {
                for i in 0..nx {
                    let k = i + nx * j;
                    if identity[k] {
                        result[k] = x[k];
                        continue;
                    }
                    let mut neighbour_sum = 0.0;
                    let mask = neighbours[k];
                    if mask & PressureStencil2D::EAST != 0 {
                        neighbour_sum += idx2 * x[k + 1];
                    }
                    if mask & PressureStencil2D::WEST != 0 {
                        neighbour_sum += idx2 * x[k - 1];
                    }
                    if mask & PressureStencil2D::NORTH != 0 {
                        neighbour_sum += idy2 * x[k + nx];
                    }
                    if mask & PressureStencil2D::SOUTH != 0 {
                        neighbour_sum += idy2 * x[k - nx];
                    }
                    result[k] = diagonal[k] * x[k] - neighbour_sum;
                }
            }
            return;
        }
        self.pool.install(|| {
            result.par_chunks_mut(nx).enumerate().for_each(|(j, row)| {
                for i in 0..nx {
                    let k = i + nx * j;
                    if identity[k] {
                        row[i] = x[k];
                        continue;
                    }
                    let mut neighbour_sum = 0.0;
                    let mask = neighbours[k];
                    if mask & PressureStencil2D::EAST != 0 {
                        neighbour_sum += idx2 * x[k + 1];
                    }
                    if mask & PressureStencil2D::WEST != 0 {
                        neighbour_sum += idx2 * x[k - 1];
                    }
                    if mask & PressureStencil2D::NORTH != 0 {
                        neighbour_sum += idy2 * x[k + nx];
                    }
                    if mask & PressureStencil2D::SOUTH != 0 {
                        neighbour_sum += idy2 * x[k - nx];
                    }
                    row[i] = diagonal[k] * x[k] - neighbour_sum;
                }
            });
        });
    }

    pub(super) fn apply_pressure_preconditioner(&self, r: &[f64], z: &mut [f64], use_dic: bool) {
        if use_dic && self.pressure_stencil.dic_supported {
            self.apply_dic_preconditioner(r, z);
        } else {
            self.apply_jacobi_preconditioner(r, z);
        }
    }

    /// Apply the structured-grid DIC(0) preconditioner. The single pressure
    /// reference cell regularizes the closed-domain Neumann null space in the
    /// preconditioner; the solved operator and zero-mean pressure projection
    /// remain unchanged.
    pub(super) fn apply_dic_preconditioner(&self, r: &[f64], z: &mut [f64]) {
        let nx = self.grid.nx;
        let n = nx * self.grid.ny;
        let idx2 = 1.0 / (self.grid.dx * self.grid.dx);
        let idy2 = 1.0 / (self.grid.dy * self.grid.dy);
        let stencil = &self.pressure_stencil;

        for k in 0..n {
            let mask = stencil.neighbours[k];
            let mut value = r[k];
            if mask & PressureStencil2D::WEST != 0 {
                value += idx2 * z[k - 1];
            }
            if mask & PressureStencil2D::SOUTH != 0 {
                value += idy2 * z[k - nx];
            }
            z[k] = stencil.dic_inverse_diagonal[k] * value;
        }

        for k in (0..n).rev() {
            let mask = stencil.neighbours[k];
            if mask & PressureStencil2D::EAST != 0 {
                z[k] += stencil.dic_inverse_diagonal[k] * idx2 * z[k + 1];
            }
            if mask & PressureStencil2D::NORTH != 0 {
                z[k] += stencil.dic_inverse_diagonal[k] * idy2 * z[k + nx];
            }
        }
    }

    pub(super) fn apply_jacobi_preconditioner(&self, r: &[f64], z: &mut [f64]) {
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        let diagonal = &self.pressure_stencil.diagonal;
        let identity = &self.pressure_stencil.identity;
        if self.pool.current_num_threads() == 1 {
            for j in 0..ny {
                for i in 0..nx {
                    let k = i + nx * j;
                    if identity[k] {
                        z[k] = r[k];
                        continue;
                    }
                    z[k] = if diagonal[k] > 0.0 {
                        r[k] / diagonal[k]
                    } else {
                        r[k]
                    };
                }
            }
            return;
        }
        self.pool.install(|| {
            z.par_chunks_mut(nx).enumerate().for_each(|(j, row)| {
                for i in 0..nx {
                    let k = i + nx * j;
                    if identity[k] {
                        row[i] = r[k];
                        continue;
                    }
                    row[i] = if diagonal[k] > 0.0 {
                        r[k] / diagonal[k]
                    } else {
                        r[k]
                    };
                }
            });
        });
    }

    pub(super) fn project_pressure_zero_mean(&self, values: &mut [f64]) {
        let pool = &self.pool;
        let solid = &self.solid;
        let case = &self.cfg.case;
        let boundaries = &self.cfg.boundary_overrides;
        let nx = self.grid.nx;
        let ny = self.grid.ny;
        project_zero_mean(pool, values, solid, nx, ny, |i, j| {
            pressure_outlet_cell(case, boundaries, solid, nx, i, j)
        });
    }

    pub(super) fn parallel_dot(&self, a: &[f64], b: &[f64]) -> f64 {
        if self.pool.current_num_threads() == 1 {
            return a.iter().zip(b).map(|(&x, &y)| x * y).sum();
        }
        self.pool
            .install(|| a.par_iter().zip(b.par_iter()).map(|(&x, &y)| x * y).sum())
    }

    pub(super) fn parallel_max_abs(&self, values: &[f64]) -> f64 {
        if self.pool.current_num_threads() == 1 {
            return values.iter().map(|value| value.abs()).fold(0.0, f64::max);
        }
        self.pool.install(|| {
            values
                .par_iter()
                .map(|value| value.abs())
                .reduce(|| 0.0, f64::max)
        })
    }

    pub(super) fn pressure_residual(&self) -> f64 {
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

    pub(super) fn max_divergence(&self) -> f64 {
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

    pub(super) fn u_face_open(&self, i: usize, j: usize) -> bool {
        u_face_open(&self.solid, self.grid.nx, i, j)
    }

    pub(super) fn v_face_open(&self, i: usize, j: usize) -> bool {
        v_face_open(&self.solid, self.grid.ny, i, j)
    }

    pub(super) fn is_pressure_outlet_cell(&self, i: usize, j: usize) -> bool {
        i + 1 == self.grid.nx
            && !self.solid[(i, j)]
            && matches!(
                self.cfg
                    .boundary_overrides
                    .kind(&self.cfg.case, Side::Right),
                BoundaryKind::PressureOutlet { .. }
            )
    }

    pub(super) fn outlet_pressure(&self) -> f64 {
        match self
            .cfg
            .boundary_overrides
            .kind(&self.cfg.case, Side::Right)
        {
            BoundaryKind::PressureOutlet { pressure } => pressure,
            _ => 0.0,
        }
    }

    pub(super) fn find_reference_cell(&self) -> Option<(usize, usize)> {
        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                if !self.solid[(i, j)] {
                    return Some((i, j));
                }
            }
        }
        None
    }

    pub(super) fn pressure_reference_required(&self) -> bool {
        !matches!(
            self.cfg
                .boundary_overrides
                .kind(&self.cfg.case, Side::Right),
            BoundaryKind::PressureOutlet { .. }
        )
    }
}

pub(super) fn project_zero_mean<F>(
    pool: &ThreadPool,
    values: &mut [f64],
    solid: &Mask2D,
    nx: usize,
    ny: usize,
    is_dirichlet: F,
) where
    F: Fn(usize, usize) -> bool + Sync,
{
    if pool.current_num_threads() == 1 {
        let mut sum = 0.0;
        let mut count = 0_usize;
        for j in 0..ny {
            for i in 0..nx {
                if !solid[(i, j)] && !is_dirichlet(i, j) {
                    sum += values[i + nx * j];
                    count += 1;
                }
            }
        }
        if count == 0 {
            return;
        }
        let mean = sum / count as f64;
        for j in 0..ny {
            for i in 0..nx {
                if !solid[(i, j)] && !is_dirichlet(i, j) {
                    values[i + nx * j] -= mean;
                }
            }
        }
        return;
    }

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
