use super::*;

impl IncompressibleSolver {
    /// Advances rho*cp*(dT/dt + u.grad(T)) = k*laplacian(T) + q using an
    /// explicit, conservative cell-centred finite-volume update.  The
    /// configuration validation enforces the diffusion limit; this method also
    /// enforces the velocity-dependent CFL condition at runtime.
    pub(super) fn advance_temperature(&mut self, step_dt: f64) -> Result<(), String> {
        let Some(old) = self.temperature.as_ref().cloned() else {
            return Ok(());
        };
        let max_u = self.u_cell.max_abs();
        let max_v = self.v_cell.max_abs();
        let cfl = step_dt * (max_u / self.grid.dx + max_v / self.grid.dy);
        if cfl > 1.0 {
            return Err(format!(
                "explicit thermal advection is unstable at step {}: CFL={cfl:.3e} exceeds 1",
                self.step
            ));
        }

        let mut next = old.clone();
        let alpha = self.cfg.physics.thermal.thermal_diffusivity;
        let source = self.cfg.physics.thermal.source_temperature_rate;
        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                if self.solid[(i, j)] {
                    next[(i, j)] = old[(i, j)];
                    continue;
                }
                let center = old[(i, j)];
                let east = self.temperature_neighbor(&old, i, j, 1, 0, center);
                let west = self.temperature_neighbor(&old, i, j, -1, 0, center);
                let north = self.temperature_neighbor(&old, i, j, 0, 1, center);
                let south = self.temperature_neighbor(&old, i, j, 0, -1, center);
                let ue = self.u[(i + 1, j)];
                let uw = self.u[(i, j)];
                let vn = self.v[(i, j + 1)];
                let vs = self.v[(i, j)];
                let flux_e = ue * upwind_value(ue, center, east);
                let flux_w = uw * upwind_value(uw, west, center);
                let flux_n = vn * upwind_value(vn, center, north);
                let flux_s = vs * upwind_value(vs, south, center);
                let convection =
                    (flux_e - flux_w) / self.grid.dx + (flux_n - flux_s) / self.grid.dy;
                let diffusion = (east - 2.0 * center + west) / self.grid.dx.powi(2)
                    + (north - 2.0 * center + south) / self.grid.dy.powi(2);
                next[(i, j)] = center + step_dt * (-convection + alpha * diffusion + source);
            }
        }
        *self
            .temperature
            .as_mut()
            .expect("temperature state checked above") = next;
        Ok(())
    }

    pub(super) fn temperature_neighbor(
        &self,
        field: &Field2D,
        i: usize,
        j: usize,
        di: isize,
        dj: isize,
        center: f64,
    ) -> f64 {
        let ni = i as isize + di;
        let nj = j as isize + dj;
        if ni >= 0 && ni < self.grid.nx as isize && nj >= 0 && nj < self.grid.ny as isize {
            let ni = ni as usize;
            let nj = nj as usize;
            return if self.solid[(ni, nj)] {
                center
            } else {
                field[(ni, nj)]
            };
        }
        let boundary = if ni < 0 {
            self.cfg.physics.thermal.left
        } else if ni >= self.grid.nx as isize {
            self.cfg.physics.thermal.right
        } else if nj < 0 {
            self.cfg.physics.thermal.bottom
        } else {
            self.cfg.physics.thermal.top
        };
        match boundary {
            ThermalBoundaryCondition::Adiabatic => center,
            ThermalBoundaryCondition::FixedTemperature { temperature } => {
                2.0 * temperature - center
            }
        }
    }
}
