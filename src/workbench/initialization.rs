use crate::{CellField, IncompressibleInitialConditions, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum InitialProfile {
    TaylorGreen2D {
        length: f64,
        speed: f64,
    },
    Uniform {
        velocity: Vec3,
        pressure: f64,
    },
    BoundaryLayer2D {
        speed: f64,
        wall_y: f64,
        thickness: f64,
        exponent: f64,
        pressure: f64,
    },
}
impl InitialProfile {
    pub fn validate(&self) -> Result<(), String> {
        match *self {
            Self::BoundaryLayer2D { speed, wall_y, thickness, exponent, pressure }
                if [speed, wall_y, thickness, exponent, pressure].iter().all(|v| v.is_finite())
                    && speed > 0.0 && thickness > 0.0 && exponent > 0.0 => Ok(()),
            Self::Uniform { velocity, pressure }
                if [velocity.x, velocity.y, velocity.z, pressure]
                    .iter()
                    .all(|v| v.is_finite()) =>
            {
                Ok(())
            }
            Self::TaylorGreen2D { length, speed }
                if length.is_finite() && length > 0. && speed.is_finite() && speed > 0. =>
            {
                Ok(())
            }
            _ => Err(
                "Initial profile values must be finite; lengths, exponents and prescribed speeds must be positive"
                    .into(),
            ),
        }
    }
    pub fn sample(&self, position: Vec3, time: f64, nu: f64) -> (Vec3, f64) {
        match *self {
            Self::Uniform { velocity, pressure } => (velocity, pressure),
            Self::BoundaryLayer2D {
                speed,
                wall_y,
                thickness,
                exponent,
                pressure,
            } => {
                let fraction = ((position.y - wall_y) / thickness).clamp(0.0, 1.0);
                (
                    Vec3::new(speed * fraction.powf(exponent), 0.0, 0.0),
                    pressure,
                )
            }
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
        if let Self::Uniform { velocity, pressure } = *self {
            return Ok(IncompressibleInitialConditions::Uniform { velocity, pressure });
        }
        if mesh.dimension() != crate::MeshDimension::TwoD {
            return Err("This initialization profile requires a planar 2D mesh".into());
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
    fn boundary_layer_initial_guess_is_bounded_and_reaches_freestream() {
        let p = InitialProfile::BoundaryLayer2D {
            speed: 10.0,
            wall_y: 1.0,
            thickness: 2.0,
            exponent: 1.0 / 7.0,
            pressure: 0.0,
        };
        assert!(p.validate().is_ok());
        assert_eq!(p.sample(Vec3::new(0.0, 1.0, 0.0), 0.0, 0.0).0.x, 0.0);
        assert_eq!(p.sample(Vec3::new(0.0, 4.0, 0.0), 0.0, 0.0).0.x, 10.0);
        let intermediate = p.sample(Vec3::new(0.0, 2.0, 0.0), 0.0, 0.0).0.x;
        assert!(intermediate > 0.0 && intermediate < 10.0);
    }
    #[test]
    fn uniform_profile_round_trips_and_rejects_nonfinite_values() {
        let profile = InitialProfile::Uniform {
            velocity: Vec3::new(44.2, 0.0, 0.0),
            pressure: 0.0,
        };
        let encoded = serde_json::to_string(&profile).unwrap();
        let decoded: InitialProfile = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, profile);
        assert_eq!(
            decoded.sample(Vec3::ZERO, 0.0, 0.0),
            (Vec3::new(44.2, 0.0, 0.0), 0.0)
        );
        assert!(InitialProfile::Uniform {
            velocity: Vec3::new(f64::NAN, 0.0, 0.0),
            pressure: 0.0
        }
        .validate()
        .is_err());
    }
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
