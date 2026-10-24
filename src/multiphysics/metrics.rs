//! Scalar finite-volume measures. Axisymmetric coordinates are axial/radial in XY.
use crate::{MeshDimension, UnstructuredMesh};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum Coordinates {
    #[default]
    Cartesian,
    /// Revolve the XY profile about X; radius is y-origin.
    AxisymmetricX { radial_origin: f64 },
    /// Revolve the XY profile about Y; radius is x-origin.
    AxisymmetricY { radial_origin: f64 },
}
#[derive(Clone, Debug)]
pub struct Measures {
    pub volumes: Vec<f64>,
    pub areas: Vec<f64>,
}
impl Coordinates {
    pub fn measures(self, mesh: &UnstructuredMesh) -> Result<Measures, String> {
        let radius = |p: crate::Vec3| match self {
            Self::Cartesian => 1.0,
            Self::AxisymmetricX { radial_origin } => p.y - radial_origin,
            Self::AxisymmetricY { radial_origin } => p.x - radial_origin,
        };
        if self != Self::Cartesian {
            if mesh.dimension() != MeshDimension::TwoD {
                return Err("Axisymmetric scalar transport requires an XY 2D meridian mesh".into());
            }
            if mesh
                .points()
                .iter()
                .any(|p| !radius(p.position).is_finite() || radius(p.position) < 0.0)
            {
                return Err(
                    "Axisymmetric profile must lie entirely at radius >= 0 with a finite origin"
                        .into(),
                );
            }
        }
        let weight = |p| {
            if self == Self::Cartesian {
                1.0
            } else {
                std::f64::consts::TAU * radius(p)
            }
        };
        let result = Measures {
            volumes: mesh
                .cells()
                .iter()
                .map(|c| c.volume * weight(c.center))
                .collect(),
            areas: mesh
                .faces()
                .iter()
                .map(|f| f.area * weight(f.center))
                .collect(),
        };
        if result.volumes.iter().any(|v| !v.is_finite() || *v <= 0.0)
            || result.areas.iter().any(|v| !v.is_finite() || *v < 0.0)
        {
            return Err("Invalid revolved finite-volume measures".into());
        }
        Ok(result)
    }
}
