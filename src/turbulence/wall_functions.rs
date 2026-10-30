//! Smooth-wall equilibrium k-epsilon treatment; not a general low-Re model.
//! Relations follow nutkWallFunction and epsilonWallFunction in the local
//! reference implementation. Corner cells use face-area weighted values.
use super::*;
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum WallTreatment {
    #[default]
    Resolved,
    EquilibriumKEpsilon,
    EquilibriumKOmega,
    VelocityBlendedKOmega,
}
const KAPPA: f64 = 0.41;
const E: f64 = 9.8;
fn crossover() -> f64 {
    let mut y = 11.0;
    for _ in 0..20 {
        y = (E * y).ln() / KAPPA;
    }
    y
}
/// k-based y+ and turbulent wall viscosity. SI units.
pub fn smooth_wall(k: f64, y: f64, nu: f64) -> Result<(f64, f64), String> {
    if !k.is_finite() || k < 0.0 || !y.is_finite() || y <= 0.0 || !nu.is_finite() || nu <= 0.0 {
        return Err("Wall function requires k ≥ 0, y > 0 and nu > 0".into());
    }
    let yp = 0.09_f64.powf(0.25) * k.sqrt() * y / nu;
    let nut = if yp > crossover() {
        nu * (KAPPA * yp / (E * yp).ln() - 1.0).max(0.0)
    } else {
        0.0
    };
    if !yp.is_finite() || !nut.is_finite() {
        return Err("Non-finite wall-function result".into());
    }
    Ok((yp, nut))
}
pub(super) struct WallValues {
    pub epsilon: Vec<Option<f64>>,
    pub omega: Vec<Option<f64>>,
    pub production: Vec<Option<f64>>,
}
impl Closure {
    pub(super) fn equilibrium_wall(&self) -> bool {
        matches!(
            self.config.wall_treatment,
            WallTreatment::EquilibriumKEpsilon
                | WallTreatment::EquilibriumKOmega
                | WallTreatment::VelocityBlendedKOmega
        )
    }
    pub(super) fn wall_nut(
        &self,
        mesh: &UnstructuredMesh,
        face: usize,
        nu: f64,
    ) -> Result<f64, SimpleError> {
        if !self.equilibrium_wall() {
            return Ok(0.0);
        }
        let f = &mesh.faces()[face];
        let y = (f.center - mesh.cells()[f.owner].center)
            .dot(f.normal)
            .abs();
        if self.config.wall_treatment == WallTreatment::VelocityBlendedKOmega {
            return Ok(self.wall_viscosity[face]);
        }
        smooth_wall(self.fields.k[f.owner], y, nu)
            .map(|(_, v)| v)
            .map_err(SimpleError::PhysicalBoundary)
    }
    pub(super) fn wall_values(
        &self,
        mesh: &UnstructuredMesh,
        state: &SimpleState,
        bc: &ResolvedVelocityBoundaryConditions,
        nu: f64,
    ) -> Result<WallValues, SimpleError> {
        let n = mesh.cell_count();
        let mut weight = vec![0.0; n];
        let mut eps = vec![0.0; n];
        let mut prod = vec![0.0; n];
        let mut omega = vec![0.0; n];
        if self.equilibrium_wall() {
            for (i, f) in mesh.faces().iter().enumerate() {
                if !self.walls[i] {
                    continue;
                }
                let o = f.owner;
                let y = (f.center - mesh.cells()[o].center).dot(f.normal).abs();
                let k = self.fields.k[o].max(1e-14);
                let (yp, mut nut) = smooth_wall(k, y, nu).map_err(SimpleError::PhysicalBoundary)?;
                let value = |c| match bc.component(c).condition(i) {
                    Some(ScalarBoundaryCondition::FixedValue(x)) => x,
                    _ => 0.0,
                };
                let uw = Vec3::new(
                    value(MomentumComponent::X),
                    value(MomentumComponent::Y),
                    value(MomentumComponent::Z),
                );
                let du = state.velocity[o] - uw;
                let tang = du - f.normal * du.dot(f.normal);
                if self.config.wall_treatment == WallTreatment::VelocityBlendedKOmega {
                    nut = self.wall_viscosity[i];
                }
                let (e, mut g) = if yp > crossover() {
                    (
                        0.09_f64.powf(0.75) * k.powf(1.5) / (KAPPA * y),
                        (nu + nut) * tang.norm() / y * 0.09_f64.powf(0.25) * k.sqrt() / (KAPPA * y),
                    )
                } else {
                    (2.0 * nu * k / (y * y), 0.0)
                };
                if self.config.wall_treatment == WallTreatment::VelocityBlendedKOmega {
                    g = (nu + nut) * du.norm() / y * 0.09_f64.powf(0.25) * k.sqrt() / (KAPPA * y);
                }
                if !e.is_finite() || !g.is_finite() {
                    return Err(SimpleError::PhysicalBoundary(
                        "Non-finite wall epsilon or production".into(),
                    ));
                }
                let omega_value = (6.0 * nu / (0.075 * y * y))
                    .hypot(k.sqrt() / (0.09_f64.powf(0.25) * KAPPA * y));
                if !omega_value.is_finite() {
                    return Err(SimpleError::PhysicalBoundary(
                        "Non-finite omega wall function".into(),
                    ));
                }
                let weight_face =
                    if self.config.wall_treatment == WallTreatment::VelocityBlendedKOmega {
                        1.0
                    } else {
                        f.area
                    };
                omega[o] += weight_face * omega_value;
                weight[o] += if self.config.wall_treatment == WallTreatment::VelocityBlendedKOmega {
                    1.0
                } else {
                    f.area
                };
                eps[o] += weight_face * e;
                prod[o] += weight_face * g;
            }
        }
        Ok(WallValues {
            omega: (0..n)
                .map(|i| (weight[i] > 0.0).then(|| omega[i] / weight[i]))
                .collect(),
            epsilon: (0..n)
                .map(|i| (weight[i] > 0.0).then(|| eps[i] / weight[i]))
                .collect(),
            production: (0..n)
                .map(|i| (weight[i] > 0.0).then(|| prod[i] / weight[i]))
                .collect(),
        })
    }
}

pub fn velocity_blended_wall(speed: f64, y: f64, nu: f64) -> Result<(f64, f64), String> {
    velocity_blended_wall_gradient(speed, speed / y, y, nu)
}

pub fn velocity_blended_wall_gradient(
    speed: f64,
    gradient: f64,
    y: f64,
    nu: f64,
) -> Result<(f64, f64), String> {
    if !speed.is_finite() || speed < 0. || !y.is_finite() || y <= 0. || !nu.is_finite() || nu <= 0.
    {
        return Err(
            "Velocity wall function requires finite speed >= 0, distance > 0 and viscosity > 0"
                .into(),
        );
    }
    if speed == 0. {
        return Ok((0., 0.));
    }
    if !gradient.is_finite() || gradient < 0. {
        return Err("Wall gradient must be finite and nonnegative".into());
    }
    if gradient == 0. {
        return Ok((0., 0.));
    }
    let target = |ut: f64| {
        let yp = y * ut / nu;
        let viscous = speed / yp;
        let logarithmic = KAPPA * speed / (E * yp).max(1.0001).ln();
        let scale = viscous.max(logarithmic);
        scale * ((viscous / scale).powi(4) + (logarithmic / scale).powi(4)).powf(0.25)
    };
    let mut lower = 0.0;
    let mut upper = (nu * gradient).sqrt().max(speed).max(1e-150);
    for _ in 0..100 {
        if upper >= target(upper) {
            break;
        }
        upper *= 2.0;
        if !upper.is_finite() {
            return Err("Velocity wall law could not bracket a solution".into());
        }
    }
    for _ in 0..80 {
        let mid = 0.5 * (lower + upper);
        if mid < target(mid) {
            lower = mid;
        } else {
            upper = mid;
        }
    }
    let ut = 0.5 * (lower + upper);
    let nut = (ut * ut / gradient - nu).max(0.);
    let yp = y * ut / nu;
    if !nut.is_finite() || !yp.is_finite() {
        return Err("Nonfinite velocity wall function".into());
    }
    Ok((yp, nut))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn velocity_blending_recovers_its_wall_law_and_stationary_limit() {
        assert_eq!(velocity_blended_wall(0., 0.001, 1e-5).unwrap(), (0., 0.));
        for speed in [0.1, 1., 10., 100.] {
            let y = 0.001;
            let nu = 1e-5;
            let (yp, nut) = velocity_blended_wall(speed, y, nu).unwrap();
            let ut = yp * nu / y;
            let viscous = speed / yp;
            let log = KAPPA * speed / (E * yp).max(1.0001).ln();
            let expected = (viscous.powi(4) + log.powi(4)).powf(0.25);
            assert!((ut - expected).abs() < 1e-8 * ut);
            assert!((ut * ut - (nu + nut) * speed / y).abs() < 1e-8 * ut * ut);
        }
        assert!(velocity_blended_wall(1., 0., 1e-5).is_err());
    }
}
