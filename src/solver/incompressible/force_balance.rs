//! Independent force diagnostic from an outer control-volume momentum balance.
//! It does not change the solved fields or replace the wall-traction force.
use super::*;

impl IncompressibleSolver {
    pub(super) fn control_volume_cylinder_force(
        &self,
        dt: f64,
        old_u: &Field2D,
        old_v: &Field2D,
    ) -> Option<(f64, f64)> {
        self.control_volume_cylinder_force_with_extent(dt, old_u, old_v, 1.25)
    }

    pub(super) fn control_volume_cylinder_force_with_extent(
        &self,
        dt: f64,
        old_u: &Field2D,
        old_v: &Field2D,
        extent: f64,
    ) -> Option<(f64, f64)> {
        let cyl = self.cfg.case.cylinder()?;
        if !self.cfg.coupling.is_transient()
            || !matches!(self.cfg.physics.buoyancy, BuoyancyModel::Off)
        {
            return None;
        }
        let (nx, ny) = (self.grid.nx, self.grid.ny);
        let (dx, dy) = (self.grid.dx, self.grid.dy);
        if !extent.is_finite() || extent < 1.0 {
            return None;
        }
        let half_width = extent * cyl.diameter;
        let i0 = ((cyl.xc - half_width) / dx).floor() as isize;
        let i1 = ((cyl.xc + half_width) / dx).ceil() as isize;
        let j0 = ((cyl.yc - half_width) / dy).floor() as isize;
        let j1 = ((cyl.yc + half_width) / dy).ceil() as isize;
        if i0 < 2 || j0 < 2 || i1 > nx as isize - 2 || j1 > ny as isize - 2 {
            return None;
        }
        let (i0, i1, j0, j1) = (i0 as usize, i1 as usize, j0 as usize, j1 as usize);
        // Every stress/flux stencil on the outer contour must be in fluid.
        for j in j0 - 1..=j1 {
            for i in i0 - 1..=i1 {
                if (i <= i0 || i >= i1 - 1 || j <= j0 || j >= j1 - 1) && self.solid[(i, j)] {
                    return None;
                }
            }
        }
        let older = self.previous_time_velocity.as_ref();
        let scheme = if self.cfg.coupling == PressureVelocityCoupling::Pimple {
            self.cfg.pimple.time_scheme
        } else {
            TransientTimeScheme::Euler
        };
        let w = scheme.coefficients(dt, older.map(|x| x.2)).ok()?;
        let mut force = [0.0; 2];
        for j in j0..j1 {
            for i in i0..i1 {
                if self.solid[(i, j)] {
                    continue;
                }
                let old = [
                    0.5 * (old_u[(i, j)] + old_u[(i + 1, j)]),
                    0.5 * (old_v[(i, j)] + old_v[(i, j + 1)]),
                ];
                let previous = older.map_or([0.0; 2], |(u, v, _)| {
                    [
                        0.5 * (u[(i, j)] + u[(i + 1, j)]),
                        0.5 * (v[(i, j)] + v[(i, j + 1)]),
                    ]
                });
                let current = [self.u_cell[(i, j)], self.v_cell[(i, j)]];
                for k in 0..2 {
                    force[k] -= cyl.rho
                        * dx
                        * dy
                        * (w[0] * current[k] - w[1] * old[k] - w[2] * previous[k])
                        / dt;
                }
            }
        }
        // F_fluid_on_body = integral(stress.n - rho U(U.n)) - d(momentum)/dt.
        for (i, sign) in [(i0, -1.0), (i1, 1.0)] {
            for j in j0..j1 {
                let un = self.u[(i, j)];
                let vt = 0.25
                    * (self.v[(i - 1, j)]
                        + self.v[(i - 1, j + 1)]
                        + self.v[(i, j)]
                        + self.v[(i, j + 1)]);
                let ux = (self.u_cell[(i, j)] - self.u_cell[(i - 1, j)]) / dx;
                let uy = (self.u[(i, j + 1)] - self.u[(i, j - 1)]) / (2.0 * dy);
                let vx = (self.v_cell[(i, j)] - self.v_cell[(i - 1, j)]) / dx;
                let vy = 0.5
                    * (self.v[(i - 1, j + 1)] - self.v[(i - 1, j)] + self.v[(i, j + 1)]
                        - self.v[(i, j)])
                    / dy;
                let p = 0.5 * (self.p[(i - 1, j)] + self.p[(i, j)]);
                let xx = -p + cyl.mu * (2.0 * ux - (2.0 / 3.0) * (ux + vy));
                let xy = cyl.mu * (uy + vx);
                force[0] += sign * dy * (xx - cyl.rho * un * un);
                force[1] += sign * dy * (xy - cyl.rho * un * vt);
            }
        }
        for (j, sign) in [(j0, -1.0), (j1, 1.0)] {
            for i in i0..i1 {
                let vn = self.v[(i, j)];
                let ut = 0.25
                    * (self.u[(i, j - 1)]
                        + self.u[(i + 1, j - 1)]
                        + self.u[(i, j)]
                        + self.u[(i + 1, j)]);
                let ux = 0.5
                    * (self.u[(i + 1, j - 1)] - self.u[(i, j - 1)] + self.u[(i + 1, j)]
                        - self.u[(i, j)])
                    / dx;
                let uy = (self.u_cell[(i, j)] - self.u_cell[(i, j - 1)]) / dy;
                let vx = (self.v[(i + 1, j)] - self.v[(i - 1, j)]) / (2.0 * dx);
                let vy = (self.v_cell[(i, j)] - self.v_cell[(i, j - 1)]) / dy;
                let p = 0.5 * (self.p[(i, j - 1)] + self.p[(i, j)]);
                let xy = cyl.mu * (uy + vx);
                let yy = -p + cyl.mu * (2.0 * vy - (2.0 / 3.0) * (ux + vy));
                force[0] += sign * dx * (xy - cyl.rho * ut * vn);
                force[1] += sign * dx * (yy - cyl.rho * vn * vn);
            }
        }
        let q = 0.5 * cyl.rho * cyl.u_inf.powi(2) * cyl.diameter;
        (q > 0.0 && force.iter().all(|x| x.is_finite())).then_some((force[0] / q, force[1] / q))
    }
}
