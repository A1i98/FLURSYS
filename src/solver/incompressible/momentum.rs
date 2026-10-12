use super::*;

impl IncompressibleSolver {
    pub(super) fn predict_momentum(&mut self, include_pressure_gradient: bool, step_dt: f64) {
        self.u_star.fill(0.0);
        self.v_star.fill(0.0);
        let Self {
            pool,
            cfg,
            grid,
            solid,
            p,
            u,
            v,
            temperature,
            u_star,
            v_star,
            ..
        } = self;
        let nx = grid.nx;
        let ny = grid.ny;
        let dx = grid.dx;
        let dy = grid.dy;
        let dt = step_dt;
        let nu = cfg.case.kinematic_viscosity();
        let convection_scheme = cfg.convection;
        let u_values = u.as_slice();
        let v_values = v.as_slice();
        let p_values = p.as_slice();
        let rho = cfg.case.density();
        let velocity_relaxation = if include_pressure_gradient {
            cfg.velocity_relaxation
        } else {
            1.0
        };
        let buoyancy = cfg.physics.buoyancy;
        let reference_temperature = match buoyancy {
            BuoyancyModel::Boussinesq {
                reference_temperature,
                ..
            } => reference_temperature,
            BuoyancyModel::Off => 0.0,
        };
        let temperature_values = temperature.as_ref().map(|field| field.as_slice());
        let stencil = MomentumStencil {
            solid,
            case: &cfg.case,
            boundaries: &cfg.boundary_overrides,
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
                        let pressure_gradient = if include_pressure_gradient {
                            (p_values[i + nx * j] - p_values[i - 1 + nx * j]) / dx
                        } else {
                            0.0
                        };
                        let buoyancy_x = match (buoyancy, temperature_values) {
                            (
                                BuoyancyModel::Boussinesq {
                                    thermal_expansion,
                                    gravity_x,
                                    ..
                                },
                                Some(t),
                            ) => {
                                let left = t[i - 1 + nx * j];
                                let right = t[i + nx * j];
                                gravity_x
                                    * thermal_expansion
                                    * (0.5 * (left + right) - reference_temperature)
                            }
                            _ => 0.0,
                        };
                        row[i] = up
                            + velocity_relaxation
                                * dt
                                * (-convection + nu * diffusion - pressure_gradient / rho
                                    + buoyancy_x);
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
                        let pressure_gradient = if include_pressure_gradient {
                            (p_values[i + nx * j] - p_values[i + nx * (j - 1)]) / dy
                        } else {
                            0.0
                        };
                        let buoyancy_y = match (buoyancy, temperature_values) {
                            (
                                BuoyancyModel::Boussinesq {
                                    thermal_expansion,
                                    gravity_y,
                                    ..
                                },
                                Some(t),
                            ) => {
                                let below = t[i + nx * (j - 1)];
                                let above = t[i + nx * j];
                                gravity_y
                                    * thermal_expansion
                                    * (0.5 * (below + above) - reference_temperature)
                            }
                            _ => 0.0,
                        };
                        row[i] = vp
                            + velocity_relaxation
                                * dt
                                * (-convection + nu * diffusion - pressure_gradient / rho
                                    + buoyancy_y);
                    }
                });
        });
    }

    pub(super) fn correct_velocity(&mut self, correction_scale: f64, step_dt: f64) {
        let dt_over_rho = correction_scale * step_dt / self.cfg.case.density();
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
}

struct MomentumStencil<'a> {
    solid: &'a Mask2D,
    case: &'a Case,
    boundaries: &'a SolverBoundaryOverrides,
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
            return tangential_ghost_u(
                self.case,
                self.boundaries,
                Side::Bottom,
                center,
                self.grid.u_face_x(i),
            );
        }
        if jj >= self.grid.ny as isize {
            return tangential_ghost_u(
                self.case,
                self.boundaries,
                Side::Top,
                center,
                self.grid.u_face_x(i),
            );
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
            return tangential_ghost_v(
                self.case,
                self.boundaries,
                Side::Left,
                center,
                self.grid.v_face_y(j),
            );
        }
        if ii >= self.grid.nx as isize {
            return tangential_ghost_v(
                self.case,
                self.boundaries,
                Side::Right,
                center,
                self.grid.v_face_y(j),
            );
        }
        center
    }
}

pub(super) fn tangential_ghost_u(
    case: &Case,
    boundaries: &SolverBoundaryOverrides,
    side: Side,
    center: f64,
    x: f64,
) -> f64 {
    match boundaries.kind(case, side) {
        BoundaryKind::Velocity => {
            let y = if side == Side::Bottom {
                0.0
            } else {
                case.domain().1
            };
            let wall_u = boundaries.velocity(case, side, x, y, 0.0).0;
            2.0 * wall_u - center
        }
        BoundaryKind::Symmetry | BoundaryKind::PressureOutlet { .. } => center,
    }
}

pub(super) fn tangential_ghost_v(
    case: &Case,
    boundaries: &SolverBoundaryOverrides,
    side: Side,
    center: f64,
    y: f64,
) -> f64 {
    match boundaries.kind(case, side) {
        BoundaryKind::Velocity => {
            let x = if side == Side::Left {
                0.0
            } else {
                case.domain().0
            };
            let wall_v = boundaries.velocity(case, side, x, y, 0.0).1;
            2.0 * wall_v - center
        }
        BoundaryKind::Symmetry | BoundaryKind::PressureOutlet { .. } => center,
    }
}

pub(super) fn transported(
    scheme: ConvectionScheme,
    face_velocity: f64,
    left: f64,
    right: f64,
) -> f64 {
    match scheme {
        ConvectionScheme::FirstOrderUpwind => {
            if face_velocity >= 0.0 {
                left
            } else {
                right
            }
        }
        ConvectionScheme::Central => 0.5 * (left + right),
        ConvectionScheme::VanLeerTvd => {
            if face_velocity >= 0.0 {
                left
            } else {
                right
            }
        }
    }
}

#[inline]
pub(super) fn upwind_value(
    face_velocity: f64,
    upstream_when_positive: f64,
    upstream_when_negative: f64,
) -> f64 {
    if face_velocity >= 0.0 {
        upstream_when_positive
    } else {
        upstream_when_negative
    }
}

pub(super) fn derivative_x_slice(
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

pub(super) fn derivative_y_slice(
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
