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
            WallTreatment::EquilibriumKEpsilon | WallTreatment::EquilibriumKOmega
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
                let (yp, nut) = smooth_wall(k, y, nu).map_err(SimpleError::PhysicalBoundary)?;
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
                let (e, g) = if yp > crossover() {
                    (
                        0.09_f64.powf(0.75) * k.powf(1.5) / (KAPPA * y),
                        (nu + nut) * tang.norm() / y * 0.09_f64.powf(0.25) * k.sqrt() / (KAPPA * y),
                    )
                } else {
                    (2.0 * nu * k / (y * y), 0.0)
                };
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
                omega[o] += f.area * omega_value;
                weight[o] += f.area;
                eps[o] += f.area * e;
                prod[o] += f.area * g;
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
