use super::*;

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct CylinderForceCoefficients {
    pub cd_pressure: f64,
    pub cd_viscous: f64,
    pub cl_pressure: f64,
    pub cl_viscous: f64,
}

impl CylinderForceCoefficients {
    pub fn cd(self) -> f64 {
        self.cd_pressure + self.cd_viscous
    }

    pub fn cl(self) -> f64 {
        self.cl_pressure + self.cl_viscous
    }
}

impl IncompressibleSolver {
    pub(super) fn cylinder_force_coefficients(&self) -> CylinderForceCoefficients {
        let Some(cyl) = self.cfg.case.cylinder() else {
            return CylinderForceCoefficients::default();
        };
        let mu = cyl.mu;
        let mut forces = CylinderForceCoefficients::default();

        for j in 0..self.grid.ny {
            for i in 0..self.grid.nx {
                if self.solid[(i, j)] {
                    continue;
                }
                let p = self.p[(i, j)];
                let uc = self.u_cell[(i, j)];
                let vc = self.v_cell[(i, j)];

                if i + 1 < self.grid.nx && self.solid[(i + 1, j)] {
                    forces.cd_pressure += p * self.grid.dy;
                    // The no-slip wall lies half a cell from the reconstructed
                    // velocity. At a vertical wall, the deviatoric normal
                    // traction contributes (4/3) mu du/dn to drag.
                    forces.cd_viscous +=
                        (4.0 / 3.0) * mu * uc / (0.5 * self.grid.dx) * self.grid.dy;
                    forces.cl_viscous += mu * vc / (0.5 * self.grid.dx) * self.grid.dy;
                }
                if i > 0 && self.solid[(i - 1, j)] {
                    forces.cd_pressure -= p * self.grid.dy;
                    forces.cd_viscous +=
                        (4.0 / 3.0) * mu * uc / (0.5 * self.grid.dx) * self.grid.dy;
                    forces.cl_viscous += mu * vc / (0.5 * self.grid.dx) * self.grid.dy;
                }
                if j + 1 < self.grid.ny && self.solid[(i, j + 1)] {
                    forces.cl_pressure += p * self.grid.dx;
                    forces.cd_viscous += mu * uc / (0.5 * self.grid.dy) * self.grid.dx;
                    forces.cl_viscous +=
                        (4.0 / 3.0) * mu * vc / (0.5 * self.grid.dy) * self.grid.dx;
                }
                if j > 0 && self.solid[(i, j - 1)] {
                    forces.cl_pressure -= p * self.grid.dx;
                    forces.cd_viscous += mu * uc / (0.5 * self.grid.dy) * self.grid.dx;
                    forces.cl_viscous +=
                        (4.0 / 3.0) * mu * vc / (0.5 * self.grid.dy) * self.grid.dx;
                }
            }
        }

        let q = 0.5 * cyl.rho * cyl.u_inf * cyl.u_inf * cyl.diameter;
        if q > 0.0 {
            forces.cd_pressure /= q;
            forces.cd_viscous /= q;
            forces.cl_pressure /= q;
            forces.cl_viscous /= q;
            forces
        } else {
            CylinderForceCoefficients::default()
        }
    }

    pub(super) fn reattachment_length_ratio(&self) -> f64 {
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

    pub(super) fn ensure_finite(&self) -> Result<(), String> {
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
        if let Some(temperature) = &self.temperature {
            if let Some((index, value)) = temperature
                .as_slice()
                .iter()
                .copied()
                .enumerate()
                .find(|(_, value)| !value.is_finite())
            {
                return Err(format!(
                    "Non-finite temperature value at flat index {index}: {value} (step {})",
                    self.step
                ));
            }
        }
        Ok(())
    }
}

pub(super) fn max_field_difference(a: &Field2D, b: &Field2D) -> f64 {
    a.as_slice()
        .iter()
        .zip(b.as_slice())
        .fold(0.0_f64, |m, (&x, &y)| m.max((x - y).abs()))
}
