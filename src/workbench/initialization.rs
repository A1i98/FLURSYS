use crate::{CellField, IncompressibleInitialConditions, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum InitialProfile {
    TaylorGreen2D { length: f64, speed: f64 },
}
impl InitialProfile {
    pub fn validate(&self) -> Result<(), String> {
        match *self {
            Self::TaylorGreen2D { length, speed }
                if length.is_finite() && length > 0. && speed.is_finite() && speed > 0. =>
            {
                Ok(())
            }
            _ => Err("Initial vortex length and speed must be finite positive values".into()),
        }
    }
    pub fn sample(&self, position: Vec3, time: f64, nu: f64) -> (Vec3, f64) {
        match *self {
            Self::TaylorGreen2D { length, speed } => {
                let k = std::f64::consts::TAU / length;
                let (x, y) = (k * position.x, k * position.y);
                let decay = (-2. * nu * k * k * time).exp();
                (
                    Vec3::new(
                        -speed * x.cos() * y.sin() * decay,
                        speed * x.sin() * y.cos() * decay,
                        0.,
                    ),
                    -0.25 * speed * speed * ((2. * x).cos() + (2. * y).cos()) * decay * decay,
                )
            }
        }
    }
    pub fn fields(
        &self,
        mesh: &UnstructuredMesh,
    ) -> Result<IncompressibleInitialConditions, String> {
        self.validate()?;
        if mesh.dimension() != crate::MeshDimension::TwoD {
            return Err("Taylor–Green 2D initialization requires a planar 2D mesh".into());
        }
        let values: Vec<_> = mesh
            .cells()
            .iter()
            .map(|c| self.sample(c.center, 0., 0.))
            .collect();
        Ok(IncompressibleInitialConditions::Fields {
            velocity: CellField::from_values(mesh, values.iter().map(|v| v.0).collect())
                .map_err(|e| format!("{e:?}"))?,
            pressure: CellField::from_values(mesh, values.iter().map(|v| v.1).collect())
                .map_err(|e| format!("{e:?}"))?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn benchmark_support_vortex_equations_and_decay() {
        let profile = InitialProfile::TaylorGreen2D {
            length: 1.,
            speed: 1.,
        };
        let p = Vec3::new(0.17, 0.31, 0.);
        let h = 1e-6;
        let (u, pressure) = profile.sample(p, 0., 0.01);
        let (xp, pp) = profile.sample(p + Vec3::new(h, 0., 0.), 0., 0.01);
        let (xm, pm) = profile.sample(p - Vec3::new(h, 0., 0.), 0., 0.01);
        let (yp, _) = profile.sample(p + Vec3::new(0., h, 0.), 0., 0.01);
        let (ym, _) = profile.sample(p - Vec3::new(0., h, 0.), 0., 0.01);
        assert!(((xp.x - xm.x + yp.y - ym.y) / (2. * h)).abs() < 1e-8);
        assert!(
            (u.x * (xp.x - xm.x) / (2. * h)
                + u.y * (yp.x - ym.x) / (2. * h)
                + (pp - pm) / (2. * h))
                .abs()
                < 1e-8
        );
        let (later, plater) = profile.sample(p, 1., 0.01);
        let factor = (-2. * 0.01 * std::f64::consts::TAU.powi(2)).exp();
        assert!((later.norm() - u.norm() * factor).abs() < 1e-12);
        assert!((plater - pressure * factor * factor).abs() < 1e-12);
    }
}
