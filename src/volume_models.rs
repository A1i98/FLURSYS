//! Optional fixed-mesh momentum resistance and a global rotating coordinate frame.
//! Velocity and boundary data are relative when a rotating frame is enabled.
use crate::{CellField, IncompressibleCase, MeshDimension, SimpleError, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum CellRegion {
    #[default]
    All,
    Box {
        minimum: Vec3,
        maximum: Vec3,
    },
}
impl CellRegion {
    pub fn contains(self, p: Vec3) -> bool {
        match self {
            Self::All => true,
            Self::Box {
                minimum: a,
                maximum: b,
            } => p.x >= a.x && p.x <= b.x && p.y >= a.y && p.y <= b.y && p.z >= a.z && p.z <= b.z,
        }
    }
    fn validate(self) -> Result<(), String> {
        if let Self::Box {
            minimum: a,
            maximum: b,
        } = self
        {
            if !finite(a) || !finite(b) || a.x >= b.x || a.y >= b.y || a.z >= b.z {
                return Err(
                    "Resistance box requires finite minimum < maximum on all axes (metres)".into(),
                );
            }
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PorousResistance {
    pub region: CellRegion,
    /// Isotropic inverse permeability [1/m²].
    pub darcy: f64,
    /// Isotropic inertial resistance [1/m]; factor 1/2 is applied by the solver.
    pub forchheimer: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RotatingFrame {
    /// Constant angular velocity vector [rad/s], coordinates fixed to the frame.
    pub angular_velocity: Vec3,
    pub origin: Vec3,
}
impl RotatingFrame {
    pub fn absolute_velocity(self, relative: Vec3, position: Vec3) -> Vec3 {
        relative + self.angular_velocity.cross(position - self.origin)
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VolumeModels {
    pub resistance: Option<PorousResistance>,
    pub rotating_frame: Option<RotatingFrame>,
}
fn finite(v: Vec3) -> bool {
    [v.x, v.y, v.z].iter().all(|x| x.is_finite())
}
impl VolumeModels {
    pub fn validate(self) -> Result<(), String> {
        if let Some(p) = self.resistance {
            p.region.validate()?;
            if !p.darcy.is_finite()
                || p.darcy < 0.0
                || !p.forchheimer.is_finite()
                || p.forchheimer < 0.0
                || (p.darcy == 0.0 && p.forchheimer == 0.0)
            {
                return Err("Momentum resistance requires nonnegative finite Darcy [1/m²] and Forchheimer [1/m], at least one positive".into());
            }
        }
        if let Some(r) = self.rotating_frame {
            if !finite(r.origin)
                || !finite(r.angular_velocity)
                || !r.angular_velocity.norm().is_finite()
                || r.angular_velocity.norm() == 0.0
            {
                return Err("Rotating frame requires finite origin [m] and nonzero finite angular velocity [rad/s]".into());
            }
        }
        Ok(())
    }
    pub fn validate_case(self, case: &IncompressibleCase) -> Result<(), String> {
        self.validate()?;
        if let Some(p) = self.resistance {
            if !case
                .mesh
                .cells()
                .iter()
                .any(|c| p.region.contains(c.center))
            {
                return Err(
                    "Momentum resistance region selects no cell centres; check box units and mesh"
                        .into(),
                );
            }
        }
        if let Some(r) = self.rotating_frame {
            if matches!(
                case.solver.turbulence.model,
                crate::turbulence::TurbulenceModel::SpalartAllmaras
                    | crate::turbulence::TurbulenceModel::RealizableKEpsilon
            ) {
                return Err("Selected turbulence model rotating-frame vorticity treatment is not implemented; use an inertial frame".into());
            }
            if case.mesh.dimension() != MeshDimension::ThreeD {
                return Err("Global rotating-frame flow currently requires a 3D mesh".into());
            }
            if case.solver.thermal.enabled && case.solver.thermal.buoyancy {
                let g = case.solver.thermal.gravity;
                if r.angular_velocity.cross(g).norm()
                    > 1e-12 * r.angular_velocity.norm() * g.norm().max(1e-30)
                {
                    return Err("Constant rotating-frame buoyancy requires gravity parallel to the rotation axis; time-varying gravity is not implemented".into());
                }
            }
        }
        Ok(())
    }
    pub(crate) fn coefficients(
        self,
        mesh: &UnstructuredMesh,
        velocity: &CellField<Vec3>,
        nu: f64,
    ) -> Result<(CellField<f64>, CellField<Vec3>), SimpleError> {
        self.validate().map_err(SimpleError::PhysicalBoundary)?;
        velocity.ensure_mesh(mesh)?;
        let mut sink = CellField::filled(mesh, 0.0);
        let mut force = CellField::filled(mesh, Vec3::ZERO);
        for (i, c) in mesh.cells().iter().enumerate() {
            if let Some(p) = self.resistance.filter(|p| p.region.contains(c.center)) {
                sink[i] = nu * p.darcy + 0.5 * p.forchheimer * velocity[i].norm();
            }
            if let Some(r) = self.rotating_frame {
                let w = r.angular_velocity;
                force[i] = -w.cross(velocity[i]) * 2.0 - w.cross(w.cross(c.center - r.origin));
            }
            if !sink[i].is_finite() || !finite(force[i]) {
                return Err(SimpleError::PhysicalBoundary(format!(
                    "Non-finite momentum model coefficient in cell {i}"
                )));
            }
        }
        Ok((sink, force))
    }
}
