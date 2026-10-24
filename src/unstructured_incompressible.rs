//! Case configuration, boundary resolution and fixed-mesh time integration
//! for constant-density unstructured incompressible flow.
//!
//! This module owns physical case semantics. The numerical SIMPLE iteration
//! remains in `crate::simple`.

use crate::{
    initial_face_flux, CellField, Diffusivity, FaceField, FieldError, LeastSquaresGradientStencil,
    LinearSolverOptions, NumericsError, ResolvedScalarBoundaryConditions,
    ResolvedVelocityBoundaryConditions, ScalarBoundaryCondition, SimpleError, SimpleOptions,
    SimpleReport, SimpleState, UnstructuredMesh, Vec3, VelocityBoundaryCondition,
};

mod profiles;
mod time;
pub use profiles::{BoundaryProfile, BoundaryProfiles, BoundaryQuantity, TimeSample};
pub use time::{AdaptiveTimeStep, TimeIntegration};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IncompressibleBoundaryCondition {
    /// Kinematic total pressure on inflow, static pressure on outflow; natural velocity opening.
    TotalPressure {
        pressure: f64,
    },
    /// Truncated planar parabola. Velocity is the peak; axis is transverse (0=X,1=Y,2=Z).
    ParabolicInlet {
        velocity: Vec3,
        axis: usize,
        center: f64,
        half_width: f64,
    },
    /// Positive total inflow: m3/s (per unit depth in 2D), or kg/s when mass=true.
    FlowRateInlet {
        rate: f64,
        mass: bool,
    },
    /// Fixed kinematic pressure; specified incoming velocity, extrapolated outflow.
    PressureOutletBackflow {
        pressure: f64,
        velocity: Vec3,
    },
    NoSlipWall,
    /// Impermeable, shear-free boundary with tangential velocity extrapolation.
    Slip,
    /// Symmetry plane: same velocity/pressure constraints as laminar slip.
    Symmetry,
    /// Incompressible free stream: prescribed velocity on inflow, kinematic
    /// reference pressure on outflow. Faces switch using the latest mass flux.
    FarField {
        velocity: Vec3,
        pressure: f64,
    },
    MovingWall {
        velocity: Vec3,
    },
    VelocityInlet {
        velocity: Vec3,
    },
    PressureOutlet {
        pressure: f64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IncompressibleMaterial {
    pub density: f64,
    pub kinematic_viscosity: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct IncompressibleSolverOptions {
    pub compute: crate::compute::ComputeOptions,
    pub time: TimeIntegration,
    pub pressure_coupling: crate::simple::PressureCoupling,
    pub volume_models: crate::volume_models::VolumeModels,
    pub cyclic_pairs: Vec<crate::periodic::TranslationalCyclicPair>,
    pub equations: Option<crate::equations::EquationSolvers>,
    pub boundary_profiles: BoundaryProfiles,
    pub turbulence: crate::turbulence::TurbulenceOptions,
    pub thermal: crate::thermal::Options,
    pub convection: crate::momentum::MomentumConvection,
    /// Private worker-pool size; zero uses the CPUs available to this process.
    /// Momentum components can run concurrently; pressure is currently serial.
    pub threads: usize,
    pub max_outer_iterations: usize,
    pub continuity_absolute_tolerance: f64,
    pub coupling_relative_tolerance: f64,
    pub velocity_relaxation: f64,
    pub pressure_relaxation: f64,
    pub momentum_solver: LinearSolverOptions,
    pub pressure_solver: LinearSolverOptions,
}

impl IncompressibleSolverOptions {
    pub fn resolve_cyclic(
        &self,
        mesh: &UnstructuredMesh,
    ) -> Result<crate::periodic::CyclicMap, String> {
        if !self.cyclic_pairs.is_empty() {
            if self.turbulence.model != crate::turbulence::TurbulenceModel::Laminar
                || self.thermal.enabled
                || self.convection != crate::momentum::MomentumConvection::Upwind
                || self.volume_models.rotating_frame.is_some()
            {
                return Err("Workbench cyclic currently requires laminar, isothermal, first-order upwind flow in an inertial frame".into());
            }
            for pair in &self.cyclic_pairs {
                if self.boundary_profiles.contains_key(&pair.first)
                    || self.boundary_profiles.contains_key(&pair.second)
                {
                    return Err(
                        "Cyclic patches cannot also have physical boundary time tables".into(),
                    );
                }
            }
        }
        crate::periodic::resolve(mesh, &self.cyclic_pairs)
    }
}
impl Default for IncompressibleSolverOptions {
    fn default() -> Self {
        Self {
            compute: Default::default(),
            time: TimeIntegration::Steady,
            pressure_coupling: Default::default(),
            volume_models: Default::default(),
            cyclic_pairs: Vec::new(),
            equations: None,
            boundary_profiles: Default::default(),
            turbulence: crate::turbulence::TurbulenceOptions::default(),
            thermal: crate::thermal::Options::default(),
            convection: crate::momentum::MomentumConvection::Upwind,
            threads: 1,
            max_outer_iterations: 1000,
            continuity_absolute_tolerance: 1.0e-10,
            coupling_relative_tolerance: 1.0e-6,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            momentum_solver: LinearSolverOptions::default(),
            pressure_solver: LinearSolverOptions::default(),
        }
    }
}

#[derive(Clone, Debug)]
#[allow(clippy::large_enum_variant)] // Preserve the public value-based API.
pub enum IncompressibleInitialConditions {
    Restart {
        velocity: CellField<Vec3>,
        pressure: CellField<f64>,
        face_flux: FaceField<f64>,
        time: f64,
        last_dt: Option<f64>,
        temperature: Option<CellField<f64>>,
        turbulence: Option<crate::turbulence::TurbulenceFields>,
    },
    Uniform {
        velocity: Vec3,
        pressure: f64,
    },
    Fields {
        velocity: CellField<Vec3>,
        pressure: CellField<f64>,
    },
}

impl IncompressibleInitialConditions {
    pub fn uniform(velocity: Vec3, pressure: f64) -> Self {
        Self::Uniform { velocity, pressure }
    }

    pub fn fields(velocity: CellField<Vec3>, pressure: CellField<f64>) -> Self {
        Self::Fields { velocity, pressure }
    }
}

#[derive(Clone, Debug)]
pub struct IncompressibleCase {
    pub mesh: UnstructuredMesh,
    pub boundaries: Vec<(String, IncompressibleBoundaryCondition)>,
    pub material: IncompressibleMaterial,
    pub solver: IncompressibleSolverOptions,
    pub initial_conditions: IncompressibleInitialConditions,
}

impl IncompressibleCase {
    pub fn steady(
        mesh: UnstructuredMesh,
        boundaries: Vec<(String, IncompressibleBoundaryCondition)>,
        material: IncompressibleMaterial,
        solver: IncompressibleSolverOptions,
    ) -> Self {
        Self {
            mesh,
            boundaries,
            material,
            solver,
            initial_conditions: IncompressibleInitialConditions::uniform(Vec3::ZERO, 0.0),
        }
    }

    pub fn with_initial_conditions(
        mut self,
        initial_conditions: IncompressibleInitialConditions,
    ) -> Self {
        self.initial_conditions = initial_conditions;
        self
    }

    pub fn resolve_boundaries(
        &self,
    ) -> Result<ResolvedIncompressibleBoundaries, IncompressibleCaseError> {
        self.resolve_boundaries_at(None, None)
    }

    pub(crate) fn resolve_boundaries_at(
        &self,
        velocity: Option<&CellField<Vec3>>,
        flux: Option<&FaceField<f64>>,
    ) -> Result<ResolvedIncompressibleBoundaries, IncompressibleCaseError> {
        if !(self.material.density.is_finite() && self.material.density > 0.0) {
            return Err(IncompressibleCaseError::InvalidDensity {
                value: self.material.density,
            });
        }
        if !(self.material.kinematic_viscosity.is_finite()
            && self.material.kinematic_viscosity >= 0.0)
        {
            return Err(IncompressibleCaseError::InvalidKinematicViscosity {
                value: self.material.kinematic_viscosity,
            });
        }

        let mut velocity_assignments = Vec::with_capacity(self.boundaries.len());
        let mut pressure_assignments = Vec::with_capacity(self.boundaries.len());
        let mut correction_assignments = Vec::with_capacity(self.boundaries.len());
        for (name, condition) in &self.boundaries {
            match *condition {
                IncompressibleBoundaryCondition::ParabolicInlet {
                    velocity,
                    axis,
                    center,
                    half_width,
                } => {
                    if !finite_vector(velocity)
                        || axis > 2
                        || !center.is_finite()
                        || !half_width.is_finite()
                        || half_width <= 0.0
                    {
                        return Err(IncompressibleCaseError::InvalidInletProfile {
                            patch: name.clone(),
                        });
                    }
                    velocity_assignments.push((
                        name.as_str(),
                        VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO),
                    ));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                }
                IncompressibleBoundaryCondition::FlowRateInlet { rate, .. } => {
                    if !rate.is_finite() || rate < 0.0 {
                        return Err(IncompressibleCaseError::InvalidFlowRate {
                            patch: name.clone(),
                            rate,
                        });
                    }
                    velocity_assignments.push((
                        name.as_str(),
                        VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO),
                    ));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                }
                IncompressibleBoundaryCondition::PressureOutletBackflow { pressure, velocity } => {
                    if !finite_vector(velocity) {
                        return Err(IncompressibleCaseError::InvalidVelocityBoundary {
                            patch: name.clone(),
                            velocity,
                        });
                    }
                    if !pressure.is_finite() {
                        return Err(IncompressibleCaseError::InvalidOutletPressure {
                            patch: name.clone(),
                            pressure,
                        });
                    }
                    velocity_assignments
                        .push((name.as_str(), VelocityBoundaryCondition::ZeroGradient));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::FixedValue(pressure)));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::FixedValue(0.0)));
                }
                IncompressibleBoundaryCondition::Slip
                | IncompressibleBoundaryCondition::Symmetry => {
                    velocity_assignments
                        .push((name.as_str(), VelocityBoundaryCondition::ZeroGradient));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                }
                IncompressibleBoundaryCondition::FarField { velocity, pressure } => {
                    if !finite_vector(velocity) {
                        return Err(IncompressibleCaseError::InvalidVelocityBoundary {
                            patch: name.clone(),
                            velocity,
                        });
                    }
                    if !pressure.is_finite() {
                        return Err(IncompressibleCaseError::InvalidOutletPressure {
                            patch: name.clone(),
                            pressure,
                        });
                    }
                    velocity_assignments
                        .push((name.as_str(), VelocityBoundaryCondition::ZeroGradient));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                }
                IncompressibleBoundaryCondition::NoSlipWall => {
                    velocity_assignments.push((
                        name.as_str(),
                        VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO),
                    ));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                }
                IncompressibleBoundaryCondition::MovingWall { velocity }
                | IncompressibleBoundaryCondition::VelocityInlet { velocity } => {
                    if !finite_vector(velocity) {
                        return Err(IncompressibleCaseError::InvalidVelocityBoundary {
                            patch: name.clone(),
                            velocity,
                        });
                    }
                    velocity_assignments.push((
                        name.as_str(),
                        VelocityBoundaryCondition::FixedVelocity(velocity),
                    ));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                }
                IncompressibleBoundaryCondition::TotalPressure { pressure }
                | IncompressibleBoundaryCondition::PressureOutlet { pressure } => {
                    if !pressure.is_finite() {
                        return Err(IncompressibleCaseError::InvalidOutletPressure {
                            patch: name.clone(),
                            pressure,
                        });
                    }
                    velocity_assignments
                        .push((name.as_str(), VelocityBoundaryCondition::ZeroGradient));
                    pressure_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::FixedValue(pressure)));
                    correction_assignments
                        .push((name.as_str(), ScalarBoundaryCondition::FixedValue(0.0)));
                }
            }
        }
        for pair in &self.solver.cyclic_pairs {
            for name in [&pair.first, &pair.second] {
                if self.boundaries.iter().any(|(n, _)| n == name) {
                    return Err(IncompressibleCaseError::Numerics(
                        NumericsError::DuplicateBoundaryPatchCondition {
                            patch: name.clone(),
                        },
                    ));
                }
                velocity_assignments.push((name.as_str(), VelocityBoundaryCondition::ZeroGradient));
                pressure_assignments.push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
                correction_assignments.push((name.as_str(), ScalarBoundaryCondition::ZeroGradient));
            }
        }
        let mut velocity_boundary =
            ResolvedVelocityBoundaryConditions::strict(&self.mesh, &velocity_assignments)?;
        let mut pressure_boundary =
            ResolvedScalarBoundaryConditions::strict(&self.mesh, &pressure_assignments)?;
        let mut pressure_correction_boundary =
            ResolvedScalarBoundaryConditions::strict(&self.mesh, &correction_assignments)?;
        let mut boundary_flux = FaceField::filled(&self.mesh, 0.0);
        for (name, condition) in &self.boundaries {
            let patch = self
                .mesh
                .boundary_patches()
                .iter()
                .find(|patch| patch.name == *name)
                .expect("resolved boundary patch exists");
            let velocity = match *condition {
                IncompressibleBoundaryCondition::Slip
                | IncompressibleBoundaryCondition::Symmetry
                | IncompressibleBoundaryCondition::FarField { .. } => None,
                IncompressibleBoundaryCondition::NoSlipWall => Some(Vec3::ZERO),
                IncompressibleBoundaryCondition::MovingWall { velocity }
                | IncompressibleBoundaryCondition::VelocityInlet { velocity } => Some(velocity),
                IncompressibleBoundaryCondition::PressureOutlet { .. }
                | IncompressibleBoundaryCondition::PressureOutletBackflow { .. }
                | IncompressibleBoundaryCondition::TotalPressure { .. }
                | IncompressibleBoundaryCondition::FlowRateInlet { .. }
                | IncompressibleBoundaryCondition::ParabolicInlet { .. } => None,
            };
            if let Some(velocity) = velocity {
                for &face in &patch.face_indices {
                    boundary_flux[face] = velocity.dot(self.mesh.faces()[face].area_vector);
                }
            }
        }
        for (name, condition) in &self.boundaries {
            let patch = self
                .mesh
                .boundary_patches()
                .iter()
                .find(|p| p.name == *name)
                .unwrap();
            let flow_speed =
                if let IncompressibleBoundaryCondition::FlowRateInlet { rate, mass } = *condition {
                    let area = patch
                        .face_indices
                        .iter()
                        .map(|f| self.mesh.faces()[*f].area_vector.norm())
                        .sum::<f64>();
                    let q = if mass {
                        rate / self.material.density
                    } else {
                        rate
                    };
                    let speed = q / area;
                    if area <= 0.0 || !area.is_finite() || !speed.is_finite() {
                        return Err(IncompressibleCaseError::InvalidFlowRate {
                            patch: name.clone(),
                            rate,
                        });
                    }
                    speed
                } else {
                    0.0
                };
            for &i in &patch.face_indices {
                let face = &self.mesh.faces()[i];
                match *condition {
                    IncompressibleBoundaryCondition::ParabolicInlet {
                        velocity,
                        axis,
                        center,
                        half_width,
                    } => {
                        let c = match axis {
                            0 => face.center.x,
                            1 => face.center.y,
                            _ => face.center.z,
                        };
                        let r = (c - center) / half_width;
                        let value = velocity * (1.0 - r * r).max(0.0);
                        velocity_boundary.set_face_velocity(i, Some(value));
                        boundary_flux[i] = value.dot(face.area_vector);
                    }
                    IncompressibleBoundaryCondition::FlowRateInlet { .. } => {
                        let value = face.area_vector * (-flow_speed / face.area_vector.norm());
                        velocity_boundary.set_face_velocity(i, Some(value));
                        boundary_flux[i] = -flow_speed * face.area_vector.norm();
                    }
                    IncompressibleBoundaryCondition::TotalPressure { pressure } => {
                        let u = velocity.map(|v| v[face.owner]).unwrap_or(Vec3::ZERO);
                        let q = flux
                            .map(|v| v[i])
                            .unwrap_or_else(|| u.dot(face.area_vector));
                        let p = pressure - if q < 0.0 { 0.5 * u.dot(u) } else { 0.0 };
                        if !p.is_finite() {
                            return Err(IncompressibleCaseError::InvalidOutletPressure {
                                patch: name.clone(),
                                pressure: p,
                            });
                        }
                        pressure_boundary.set_condition(i, ScalarBoundaryCondition::FixedValue(p));
                        pressure_correction_boundary
                            .set_condition(i, ScalarBoundaryCondition::FixedValue(0.0));
                    }
                    IncompressibleBoundaryCondition::PressureOutletBackflow {
                        velocity: incoming,
                        ..
                    } => {
                        let current = flux.map(|f| f[i]).unwrap_or_else(|| {
                            velocity
                                .map(|v| v[face.owner].dot(face.area_vector))
                                .unwrap_or(0.0)
                        });
                        velocity_boundary.set_face_velocity(i, (current < 0.0).then_some(incoming));
                    }
                    IncompressibleBoundaryCondition::Slip
                    | IncompressibleBoundaryCondition::Symmetry => {
                        let n = face.area_vector / face.area_vector.norm();
                        let u = velocity.map(|v| v[face.owner]).unwrap_or(Vec3::ZERO);
                        // Deferred cross-component coupling: converged U_b = (I - nnᵀ) U_P.
                        velocity_boundary.set_slip_face(i, n, u);
                        boundary_flux[i] = 0.0;
                    }
                    IncompressibleBoundaryCondition::FarField {
                        velocity: ambient,
                        pressure,
                    } => {
                        let reference_flux = ambient.dot(face.area_vector);
                        let current = flux.map(|f| f[i]).unwrap_or(reference_flux);
                        let owner_flux = velocity
                            .map(|v| v[face.owner].dot(face.area_vector))
                            .unwrap_or(reference_flux);
                        let incoming = far_field_incoming(current, reference_flux, owner_flux);
                        if incoming {
                            velocity_boundary.set_face_velocity(i, Some(ambient));
                            boundary_flux[i] = reference_flux;
                        } else {
                            velocity_boundary.set_face_velocity(i, None);
                            pressure_boundary
                                .set_condition(i, ScalarBoundaryCondition::FixedValue(pressure));
                            pressure_correction_boundary
                                .set_condition(i, ScalarBoundaryCondition::FixedValue(0.0));
                        }
                    }
                    _ => {}
                }
            }
        }
        if !self.solver.cyclic_pairs.is_empty() {
            let cyclic = std::sync::Arc::new(
                self.solver
                    .resolve_cyclic(&self.mesh)
                    .map_err(|message| IncompressibleCaseError::InvalidCyclic { message })?,
            );
            velocity_boundary.set_cyclic(cyclic.clone());
            pressure_boundary.set_cyclic(cyclic.clone());
            pressure_correction_boundary.set_cyclic(cyclic.clone());
            let u = |cell: usize| -> Vec3 {
                if let Some(u) = velocity {
                    return u[cell];
                }
                match &self.initial_conditions {
                    IncompressibleInitialConditions::Uniform { velocity, .. } => *velocity,
                    IncompressibleInitialConditions::Fields { velocity, .. }
                    | IncompressibleInitialConditions::Restart { velocity, .. } => {
                        velocity.values().get(cell).copied().unwrap_or(Vec3::ZERO)
                    }
                }
            };
            for (i, face) in self.mesh.faces().iter().enumerate() {
                if let Some(link) = cyclic.link(i) {
                    if i > link.peer_face {
                        continue;
                    }
                    let peer = &self.mesh.faces()[link.peer_face];
                    let image = self.mesh.cells()[peer.owner].center - link.translation;
                    let a = (face.center - self.mesh.cells()[face.owner].center).norm();
                    let b = (face.center - image).norm();
                    let phi =
                        ((u(face.owner) * b + u(peer.owner) * a) / (a + b)).dot(face.area_vector);
                    boundary_flux[i] = phi;
                    boundary_flux[link.peer_face] = -phi;
                }
            }
        }
        Ok(ResolvedIncompressibleBoundaries {
            velocity_boundary,
            pressure_boundary,
            pressure_correction_boundary,
            boundary_flux,
        })
    }
}

// An imposed tangential ambient velocity sets incoming flux to zero. Using
// that zero to select outflow at the next iteration creates an artificial
// inlet/outlet two-cycle. At grazing incidence retain the local flow direction
// from the adjacent cell until it actually reverses.
fn far_field_incoming(current: f64, ambient: f64, owner: f64) -> bool {
    let threshold = 1e-12 * (ambient.abs() + 1.0);
    if current.abs() > threshold {
        current < 0.0
    } else if ambient.abs() > threshold {
        ambient < 0.0
    } else {
        owner < -threshold
    }
}

#[cfg(test)]
mod far_field_tests {
    use super::far_field_incoming;

    #[test]
    fn grazing_inflow_is_not_reset_by_its_prescribed_zero_flux() {
        assert!(far_field_incoming(-0.1, 0.0, -0.1));
        assert!(far_field_incoming(0.0, 0.0, -0.1));
        assert!(!far_field_incoming(0.0, 0.0, 0.1));
        assert!(!far_field_incoming(0.0, 0.0, 0.0));
    }

    #[test]
    fn resolved_flux_and_non_grazing_ambient_keep_precedence() {
        assert!(far_field_incoming(-0.1, 0.2, 0.2));
        assert!(!far_field_incoming(0.1, -0.2, -0.2));
        assert!(far_field_incoming(0.0, -0.2, 0.2));
        assert!(!far_field_incoming(0.0, 0.2, -0.2));
    }
}

#[derive(Clone, Debug)]
pub struct ResolvedIncompressibleBoundaries {
    velocity_boundary: ResolvedVelocityBoundaryConditions,
    pressure_boundary: ResolvedScalarBoundaryConditions,
    pressure_correction_boundary: ResolvedScalarBoundaryConditions,
    boundary_flux: FaceField<f64>,
}

impl ResolvedIncompressibleBoundaries {
    pub fn velocity_boundary(&self) -> &ResolvedVelocityBoundaryConditions {
        &self.velocity_boundary
    }

    pub fn pressure_boundary(&self) -> &ResolvedScalarBoundaryConditions {
        &self.pressure_boundary
    }

    pub fn pressure_correction_boundary(&self) -> &ResolvedScalarBoundaryConditions {
        &self.pressure_correction_boundary
    }

    pub fn boundary_flux(&self) -> &FaceField<f64> {
        &self.boundary_flux
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum IncompressibleCaseError {
    InvalidCyclic { message: String },
    Field(FieldError),
    Numerics(crate::NumericsError),
    UnknownBoundaryPatch { patch: String },
    InvalidInletProfile { patch: String },
    InvalidInitialConditions,
    InvalidDensity { value: f64 },
    InvalidKinematicViscosity { value: f64 },
    InvalidVelocityBoundary { patch: String, velocity: Vec3 },
    InvalidOutletPressure { patch: String, pressure: f64 },
    InvalidFlowRate { patch: String, rate: f64 },
}

impl std::fmt::Display for IncompressibleCaseError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for IncompressibleCaseError {}

impl From<FieldError> for IncompressibleCaseError {
    fn from(value: FieldError) -> Self {
        Self::Field(value)
    }
}

impl From<crate::NumericsError> for IncompressibleCaseError {
    fn from(value: crate::NumericsError) -> Self {
        Self::Numerics(value)
    }
}

fn finite_vector(value: Vec3) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
}

/// Integrates owner-oriented normal flux across a named exterior patch.
/// Positive values are outward from the computational domain.
pub fn patch_flux(
    mesh: &UnstructuredMesh,
    flux: &FaceField<f64>,
    patch_name: &str,
) -> Result<f64, IncompressibleCaseError> {
    flux.ensure_mesh(mesh)?;
    let patch = mesh
        .boundary_patches()
        .iter()
        .find(|patch| patch.name == patch_name)
        .ok_or_else(|| IncompressibleCaseError::UnknownBoundaryPatch {
            patch: patch_name.to_string(),
        })?;
    Ok(patch.face_indices.iter().map(|&face| flux[face]).sum())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IncompressibleSolveStatus {
    Converged,
    MaxIterations,
}

#[derive(Clone, Debug)]
pub struct IncompressibleSolveReport {
    pub performance: crate::compute::PerformanceReport,
    pub cpu_threads: usize,
    pub status: IncompressibleSolveStatus,
    pub initial_continuity_rms: f64,
    pub final_continuity_rms: f64,
    pub outer_iterations: usize,
    pub continuity_history: Vec<f64>,
    /// Sum of all owner-oriented incoming boundary fluxes; non-positive.
    pub total_inflow: f64,
    /// Sum of all owner-oriented outgoing boundary fluxes; non-negative.
    pub total_outflow: f64,
    /// Net owner-oriented flux across every exterior face.
    pub net_boundary_flux: f64,
    pub simple: SimpleReport,
}

impl IncompressibleSolveReport {
    pub fn converged(&self) -> bool {
        self.status == IncompressibleSolveStatus::Converged
    }

    pub fn status(&self) -> IncompressibleSolveStatus {
        self.status
    }
}

#[derive(Clone, Debug)]
pub struct IncompressibleSolution {
    pub nonlinear_iterations: usize,
    pub max_courant: Option<f64>,
    pub turbulence: Option<crate::turbulence::TurbulenceFields>,
    pub temperature: Option<CellField<f64>>,
    pub last_time_step: Option<f64>,
    pub rejected_time_steps: usize,
    pub physical_time: Option<f64>,
    pub completed_time_steps: usize,
    pub velocity: CellField<Vec3>,
    pub pressure: CellField<f64>,
    pub face_flux: FaceField<f64>,
    pub report: IncompressibleSolveReport,
}

/// Cooperative control hooks for an incompressible solve.
///
/// The solver owns no UI state; callers may use this to publish progress or
/// stop between SIMPLE outer iterations.
pub struct SolveControl<'a> {
    is_cancelled: Box<dyn Fn() -> bool + Send + Sync + 'a>,
    on_iteration: Box<dyn Fn(usize, usize, f64) + Send + Sync + 'a>,
    on_metrics: Option<Box<dyn Fn(usize, [f64; 4]) + Send + Sync + 'a>>,
    on_step: Option<
        Box<dyn Fn(usize, &IncompressibleSolution) -> Result<(), String> + Send + Sync + 'a>,
    >,
    on_restart: Option<
        Box<
            dyn Fn(usize, IncompressibleInitialConditions) -> Result<(), String> + Send + Sync + 'a,
        >,
    >,
    restart_every: usize,
    on_message: Option<Box<dyn Fn(String) + Send + Sync + 'a>>,
    on_performance: Option<Box<dyn Fn(crate::compute::PerformanceReport) + Send + Sync + 'a>>,
    linear_control: Option<crate::compute::LinearControl>,
    on_fields: Option<
        Box<
            dyn Fn(usize, &CellField<Vec3>, &CellField<f64>) -> Result<(), String>
                + Send
                + Sync
                + 'a,
        >,
    >,
}

impl<'a> SolveControl<'a> {
    pub fn new(
        is_cancelled: impl Fn() -> bool + Send + Sync + 'a,
        on_iteration: impl Fn(usize, usize, f64) + Send + Sync + 'a,
    ) -> Self {
        Self {
            is_cancelled: Box::new(is_cancelled),
            on_iteration: Box::new(on_iteration),
            on_metrics: None,
            on_step: None,
            on_restart: None,
            restart_every: 0,
            on_message: None,
            on_performance: None,
            linear_control: None,
            on_fields: None,
        }
    }

    /// Optional detailed progress, without changing legacy iteration callbacks.
    pub fn with_metrics(mut self, callback: impl Fn(usize, [f64; 4]) + Send + Sync + 'a) -> Self {
        self.on_metrics = Some(Box::new(callback));
        self
    }

    pub fn with_fields(
        mut self,
        callback: impl Fn(usize, &CellField<Vec3>, &CellField<f64>) -> Result<(), String>
            + Send
            + Sync
            + 'a,
    ) -> Self {
        self.on_fields = Some(Box::new(callback));
        self
    }

    pub fn with_restart_state(
        mut self,
        every: usize,
        callback: impl Fn(usize, IncompressibleInitialConditions) -> Result<(), String>
            + Send
            + Sync
            + 'a,
    ) -> Self {
        self.on_restart = Some(Box::new(callback));
        self.restart_every = every;
        self
    }
    pub fn with_linear_control(
        mut self,
        callback: impl Fn() -> bool + Send + Sync + 'static,
    ) -> Self {
        self.linear_control = Some(std::sync::Arc::new(callback));
        self
    }
    pub fn with_performance(
        mut self,
        callback: impl Fn(crate::compute::PerformanceReport) + Send + Sync + 'a,
    ) -> Self {
        self.on_performance = Some(Box::new(callback));
        self
    }
    pub fn with_message(mut self, callback: impl Fn(String) + Send + Sync + 'a) -> Self {
        self.on_message = Some(Box::new(callback));
        self
    }
    pub fn with_completed_step(
        mut self,
        callback: impl Fn(usize, &IncompressibleSolution) -> Result<(), String> + Send + Sync + 'a,
    ) -> Self {
        self.on_step = Some(Box::new(callback));
        self
    }
    fn cancelled(&self) -> bool {
        (self.is_cancelled)()
    }

    fn report_iteration(&self, current: usize, max: usize, residual: f64) {
        (self.on_iteration)(current, max, residual);
    }
}

impl IncompressibleSolution {
    pub fn restart(&self) -> IncompressibleInitialConditions {
        IncompressibleInitialConditions::Restart {
            velocity: self.velocity.clone(),
            pressure: self.pressure.clone(),
            face_flux: self.face_flux.clone(),
            time: self.physical_time.unwrap_or(0.0),
            last_dt: self.last_time_step,
            temperature: self.temperature.clone(),
            turbulence: self.turbulence.clone(),
        }
    }

    /// Returns the cell-centred velocity magnitude for result export and
    /// visualization without reconstructing fluxes or mutating solution state.
    pub fn speed(&self) -> CellField<f64> {
        self.velocity.map(|velocity| velocity.norm())
    }
}

#[derive(Clone, Debug)]
pub enum IncompressibleSolveError {
    Cancelled,
    Case(IncompressibleCaseError),
    Simple(SimpleError),
    ThreadPool(String),
    Compute(String),
    BackflowAtPressureOutlet {
        patch: String,
        face: usize,
        flux: f64,
    },
}

impl std::fmt::Display for IncompressibleSolveError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let guidance = match self {
            Self::Compute(_) => "Compute backend could not start. Check device selection, resource limits and CPU fallback settings.",
            Self::Simple(SimpleError::Linear(crate::LinearAlgebraError::Backend { message })) if message.contains("FGMRES exhausted") => "GPU-preconditioned linear solve did not reach the requested true residual. The final L2, target and method are reported below. Review the preconditioner and linear iteration budget; CPU fallback is available without changing tolerances.",
            Self::Simple(SimpleError::Linear(crate::LinearAlgebraError::Backend { .. })) => "Compute backend failed. Check GPU availability, working-buffer limits and recorded fallback policy.",
            Self::Cancelled => "Solve cancelled by the user.",
            Self::Simple(SimpleError::Linear(crate::LinearAlgebraError::IncompleteCholeskyPivot {..})) => "Pressure IC(0) factorization failed. The failing row, pivot and diagonal are included below. Inspect pressure-system scaling and definiteness, plus preceding velocity/momentum growth; a small continuity residual alone does not establish a stable solution.",
            Self::Simple(SimpleError::MomentumLinearDidNotConverge) => "Momentum linear solver did not reach tolerance. Check mesh non-orthogonality and size transitions, inlet speed and viscosity; try lower velocity relaxation and first-order upwind. This does not identify a unique cause.",
            Self::Simple(SimpleError::PressureLinearDidNotConverge) => "Pressure linear solver did not reach tolerance. Check domain connectivity, pressure reference, mesh quality and incompatible flow boundary conditions.",
            Self::Simple(SimpleError::InvalidMomentumDiagonal { .. }) => "Momentum diagonal is invalid. Check cell measures, material properties and assembled coefficients.",
            Self::BackflowAtPressureOutlet { .. } => "Flow enters a strict pressure outlet. Extend the outlet beyond recirculation, or choose a pressure outlet with an explicit backflow velocity when physically appropriate.",
            Self::Case(IncompressibleCaseError::Numerics(crate::NumericsError::SingularLeastSquaresStencil { .. })) | Self::Simple(SimpleError::Numerics(crate::NumericsError::SingularLeastSquaresStencil { .. })) => "Gradient stencil is rank-deficient at the reported cell. Check degenerate cells, collapsed dimensions and disconnected or insufficiently independent neighbour directions; adding iterations will not fix this geometric failure.",
            Self::Case(_) => "Case preparation failed. Check boundary coverage, boundary compatibility and numerical stencils.",
            _ => "Solve failed; inspect the detailed error and mesh preflight report.",
        };
        write!(formatter, "{guidance} Details: {self:?}")
    }
}

impl std::error::Error for IncompressibleSolveError {}

impl From<IncompressibleCaseError> for IncompressibleSolveError {
    fn from(value: IncompressibleCaseError) -> Self {
        Self::Case(value)
    }
}

impl From<SimpleError> for IncompressibleSolveError {
    fn from(value: SimpleError) -> Self {
        Self::Simple(value)
    }
}

/// Runs the verified Phase 7 SIMPLE engine for a fully resolved steady,
/// constant-density laminar case. The current baseline uses reference cell zero
/// for the all-Neumann pressure-correction system; pressure-outlet matrix
/// anchoring is added with the open-boundary SIMPLE extension.
pub fn solve_incompressible(
    case: &IncompressibleCase,
) -> Result<IncompressibleSolution, IncompressibleSolveError> {
    solve_incompressible_impl(case, None)
}

/// Runs an incompressible case with application-owned cancellation and progress
/// hooks. This does not change numerical options or persistence semantics.
pub fn solve_incompressible_with_control(
    case: &IncompressibleCase,
    control: &SolveControl<'_>,
) -> Result<IncompressibleSolution, IncompressibleSolveError> {
    solve_incompressible_impl(case, Some(control))
}

fn solve_incompressible_impl(
    case: &IncompressibleCase,
    control: Option<&SolveControl<'_>>,
) -> Result<IncompressibleSolution, IncompressibleSolveError> {
    let execution = crate::compute::Execution::controlled(
        case.solver.compute.clone(),
        control.and_then(|c| c.linear_control.clone()),
    )
    .map_err(IncompressibleSolveError::Compute)?;
    let threads = if case.solver.compute.backend == crate::compute::Backend::Serial {
        1
    } else if case.solver.threads == 0 {
        std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
    } else {
        case.solver.threads
    };
    if let Some(message) = control.and_then(|c| c.on_message.as_ref()) {
        message(format!(
            "Compute: {:?} · {} CPU workers · GPU corrections use f64 residual acceptance",
            case.solver.compute.backend, threads
        ));
    }
    let worker_execution = execution.clone();
    let pool = rayon::ThreadPoolBuilder::new()
        .start_handler(move |_| crate::compute::install(Some(worker_execution.clone())))
        .num_threads(threads)
        .build()
        .map_err(|e| IncompressibleSolveError::ThreadPool(e.to_string()))?;
    let result = pool.install(|| {
        let _scope = crate::compute::Scope::enter(execution.clone());
        time::solve_time_loop(case, control)
    });
    let result = if execution.cancelled() {
        Err(IncompressibleSolveError::Cancelled)
    } else {
        result
    };
    let performance = execution.report(
        threads,
        case.mesh.cell_count(),
        result.as_ref().map_or(0, |s| s.nonlinear_iterations),
    );
    if let Some(callback) = control.and_then(|c| c.on_performance.as_ref()) {
        callback(performance.clone());
    }
    if let Some(message) = control.and_then(|c| c.on_message.as_ref()) {
        message(format!(
            "Compute: {} · {:.3}s · {:.3e} cell-iterations/s · {} CPU fallbacks{}",
            performance.actual_backend,
            performance.elapsed_seconds,
            performance.cell_iterations_per_second,
            performance.cpu_fallbacks,
            if performance.fallback_reasons.is_empty() {
                String::new()
            } else {
                format!(" · {}", performance.fallback_reasons.join("; "))
            }
        ));
    }
    result.map(|mut solution| {
        solution.report.performance = performance;
        solution
    })
}

fn solve_incompressible_on_pool(
    case: &IncompressibleCase,
    control: Option<&SolveControl<'_>>,
    time_step: Option<f64>,
    older: Option<(&IncompressibleInitialConditions, [f64; 2])>,
) -> Result<IncompressibleSolution, IncompressibleSolveError> {
    if let Some(e) = &case.solver.equations {
        e.validate().map_err(SimpleError::PhysicalBoundary)?;
    }
    case.solver
        .thermal
        .validate_for_case(case)
        .map_err(SimpleError::PhysicalBoundary)?;
    let resolved = case.resolve_boundaries()?;
    let prescribed_flux = case
        .mesh
        .faces()
        .iter()
        .enumerate()
        .filter(|(_, face)| face.neighbour.is_none())
        .map(|(index, _)| resolved.boundary_flux()[index])
        .sum::<f64>();
    if prescribed_flux > 1.0e-12 {
        if let Some((name, _)) = case.boundaries.iter().find(|(_, condition)| {
            matches!(
                condition,
                IncompressibleBoundaryCondition::PressureOutlet { .. }
            )
        }) {
            let patch = case
                .mesh
                .boundary_patches()
                .iter()
                .find(|patch| patch.name == *name)
                .expect("resolved patch exists");
            return Err(IncompressibleSolveError::BackflowAtPressureOutlet {
                patch: name.clone(),
                face: patch.face_indices[0],
                flux: -prescribed_flux,
            });
        }
    }
    let pressure_stencil =
        LeastSquaresGradientStencil::new(&case.mesh, resolved.pressure_boundary())
            .map_err(IncompressibleCaseError::from)?;
    let (velocity, pressure) = match &case.initial_conditions {
        IncompressibleInitialConditions::Uniform { velocity, pressure } => {
            if !finite_vector(*velocity) || !pressure.is_finite() {
                return Err(IncompressibleCaseError::InvalidInitialConditions.into());
            }
            (
                CellField::filled(&case.mesh, *velocity),
                CellField::filled(&case.mesh, *pressure),
            )
        }
        IncompressibleInitialConditions::Fields { velocity, pressure }
        | IncompressibleInitialConditions::Restart {
            velocity, pressure, ..
        } => {
            velocity
                .ensure_mesh(&case.mesh)
                .map_err(IncompressibleCaseError::from)?;
            pressure
                .ensure_mesh(&case.mesh)
                .map_err(IncompressibleCaseError::from)?;
            if !velocity.values().iter().all(|value| finite_vector(*value))
                || !pressure.values().iter().all(|value| value.is_finite())
            {
                return Err(IncompressibleCaseError::InvalidInitialConditions.into());
            }
            (velocity.clone(), pressure.clone())
        }
    };
    let face_flux = if let IncompressibleInitialConditions::Restart { face_flux, .. } =
        &case.initial_conditions
    {
        face_flux
            .ensure_mesh(&case.mesh)
            .map_err(IncompressibleCaseError::from)?;
        let mut flux = face_flux.clone();
        for (i, f) in case.mesh.faces().iter().enumerate() {
            if f.neighbour.is_none()
                && !matches!(
                    resolved
                        .velocity_boundary()
                        .component(crate::MomentumComponent::X)
                        .condition(i),
                    Some(ScalarBoundaryCondition::ZeroGradient)
                )
            {
                flux[i] = resolved.boundary_flux()[i];
            }
        }
        flux
    } else {
        initial_face_flux(&case.mesh, &velocity, resolved.boundary_flux())?
    };
    let mut state = SimpleState::new(&case.mesh, velocity, pressure, face_flux)?;
    let mut options = SimpleOptions::steady(
        Diffusivity::Constant(case.material.kinematic_viscosity),
        resolved.velocity_boundary(),
        &pressure_stencil,
    );
    if case.boundaries.iter().any(|(_, condition)| {
        matches!(
            condition,
            IncompressibleBoundaryCondition::PressureOutlet { .. }
                | IncompressibleBoundaryCondition::PressureOutletBackflow { .. }
                | IncompressibleBoundaryCondition::TotalPressure { .. }
        )
    }) {
        options.pressure_boundary = Some(resolved.pressure_boundary());
        options.pressure_correction_boundary = Some(resolved.pressure_correction_boundary());
    }
    if case.boundaries.iter().any(|(_, b)| {
        matches!(
            b,
            IncompressibleBoundaryCondition::Slip
                | IncompressibleBoundaryCondition::Symmetry
                | IncompressibleBoundaryCondition::FarField { .. }
                | IncompressibleBoundaryCondition::PressureOutletBackflow { .. }
                | IncompressibleBoundaryCondition::TotalPressure { .. }
                | IncompressibleBoundaryCondition::FlowRateInlet { .. }
                | IncompressibleBoundaryCondition::ParabolicInlet { .. }
        )
    }) {
        options.boundary_case = Some(case);
    }
    if case.mesh.dimension() == crate::MeshDimension::ThreeD {
        options.momentum_diffusion.non_orthogonal_correction =
            crate::NonOrthogonalCorrection::Limited;
    }
    options.convection = case.solver.convection;
    options.max_outer_iterations = case.solver.max_outer_iterations;
    options.continuity_absolute_tolerance = case.solver.continuity_absolute_tolerance;
    options.coupling_relative_tolerance = case.solver.coupling_relative_tolerance;
    options.velocity_relaxation = case.solver.velocity_relaxation;
    options.pressure_relaxation = case.solver.pressure_relaxation;
    options.pressure_coupling = case.solver.pressure_coupling;
    options.volume_models = case.solver.volume_models;
    if !case.solver.cyclic_pairs.is_empty() {
        options.pressure_boundary = Some(resolved.pressure_boundary());
        options.pressure_correction_boundary = Some(resolved.pressure_correction_boundary());
    }
    options.equations = case.solver.equations.as_ref();
    options.momentum_solver = case
        .solver
        .equations
        .as_ref()
        .map_or(case.solver.momentum_solver, |e| e.velocity.options());
    options.pressure_solver = case
        .solver
        .equations
        .as_ref()
        .map_or(case.solver.pressure_solver, |e| e.pressure.options());
    let initial_turbulence = match &case.initial_conditions {
        IncompressibleInitialConditions::Restart { turbulence, .. } => turbulence.clone(),
        _ => None,
    };
    let closure = if case.solver.turbulence.model != crate::turbulence::TurbulenceModel::Laminar {
        Some(std::sync::Mutex::new(crate::turbulence::Closure::new(
            &case.mesh,
            case,
            initial_turbulence,
        )?))
    } else {
        None
    };
    let initial_temperature = match &case.initial_conditions {
        IncompressibleInitialConditions::Restart {
            temperature, time, ..
        } => {
            if case.solver.thermal.enabled && *time > 0.0 && temperature.is_none() {
                return Err(IncompressibleSolveError::Simple(
                    SimpleError::PhysicalBoundary(
                        "Thermal transient restart requires saved temperature".into(),
                    ),
                ));
            }
            temperature.clone()
        }
        _ => None,
    };
    case.solver
        .thermal
        .validate()
        .map_err(|e| IncompressibleSolveError::Simple(SimpleError::PhysicalBoundary(e)))?;
    let thermal = if case.solver.thermal.enabled {
        Some(std::sync::Mutex::new(crate::thermal::Closure::new(
            case,
            initial_temperature,
        )?))
    } else {
        None
    };
    options.thermal = thermal.as_ref();
    options.closure = closure.as_ref();
    if closure.is_some() {
        options.boundary_case = Some(case);
    }
    let mut temporal_velocity = time_step.map(|_| state.velocity.clone());
    let mut temporal_flux = time_step.map(|_| state.face_flux.clone());
    if let Some((older, weights)) = older {
        let (u, phi, t, turb) = match older {
            IncompressibleInitialConditions::Restart {
                velocity,
                face_flux,
                temperature,
                turbulence,
                ..
            } => (
                velocity.clone(),
                face_flux.clone(),
                temperature.clone(),
                turbulence.clone(),
            ),
            IncompressibleInitialConditions::Uniform { velocity, .. } => {
                let u = CellField::filled(&case.mesh, *velocity);
                let phi = initial_face_flux(&case.mesh, &u, resolved.boundary_flux())?;
                (u, phi, None, None)
            }
            IncompressibleInitialConditions::Fields { velocity, .. } => {
                let phi = initial_face_flux(&case.mesh, velocity, resolved.boundary_flux())?;
                (velocity.clone(), phi, None, None)
            }
        };
        if let Some(v) = &mut temporal_velocity {
            for (a, b) in v.values_mut().iter_mut().zip(u.values()) {
                *a = *a * weights[0] + *b * weights[1];
            }
        }
        if let Some(v) = &mut temporal_flux {
            for (a, b) in v.values_mut().iter_mut().zip(phi.values()) {
                *a = *a * weights[0] + *b * weights[1];
            }
        }
        if let Some(c) = &thermal {
            c.lock().unwrap().backward_old(t.as_ref(), weights);
        }
        if let Some(c) = &closure {
            c.lock().unwrap().backward_old(turb.as_ref(), weights);
        }
    }
    options.time_step = time_step;
    options.time_velocity = temporal_velocity.as_ref();
    options.time_flux = temporal_flux.as_ref();
    let last_completed = std::cell::Cell::new(0usize);
    let simple = crate::simple::solve_simple_with_observer(
        &case.mesh,
        &mut state,
        options,
        || control.is_some_and(SolveControl::cancelled),
        |iteration, metrics| {
            if let Some(control) = control {
                control.report_iteration(iteration, case.solver.max_outer_iterations, metrics[0]);
                if let Some(callback) = &control.on_metrics {
                    callback(iteration, metrics);
                }
            }
        },
        |iteration, state| {
            last_completed.set(iteration);
            if time_step.is_none()
                && control.is_some_and(|c| {
                    c.restart_every > 0 && (iteration == 1 || iteration % c.restart_every == 0)
                })
            {
                if let Some(callback) = control.and_then(|c| c.on_restart.as_ref()) {
                    callback(
                        iteration,
                        IncompressibleInitialConditions::Restart {
                            velocity: state.velocity.clone(),
                            pressure: state.pressure.clone(),
                            face_flux: state.face_flux.clone(),
                            time: 0.0,
                            last_dt: None,
                            turbulence: closure.as_ref().map(|c| c.lock().unwrap().fields()),
                            temperature: thermal.as_ref().map(|t| t.lock().unwrap().field()),
                        },
                    )
                    .map_err(SimpleError::Monitor)?;
                }
            }
            if let Some(callback) = control.and_then(|c| c.on_fields.as_ref()) {
                callback(iteration, &state.velocity, &state.pressure)
                    .map_err(SimpleError::Monitor)?;
            }
            Ok(())
        },
    )
    .map_err(|error| match error {
        SimpleError::Cancelled => {
            if time_step.is_none() {
                if let Some(callback) = control.and_then(|c| c.on_restart.as_ref()) {
                    if let Err(error) = callback(
                        last_completed.get(),
                        IncompressibleInitialConditions::Restart {
                            velocity: state.velocity.clone(),
                            pressure: state.pressure.clone(),
                            face_flux: state.face_flux.clone(),
                            time: 0.0,
                            last_dt: None,
                            turbulence: closure.as_ref().map(|c| c.lock().unwrap().fields()),
                            temperature: thermal.as_ref().map(|t| t.lock().unwrap().field()),
                        },
                    ) {
                        return IncompressibleSolveError::Simple(SimpleError::Monitor(error));
                    }
                }
            }
            IncompressibleSolveError::Cancelled
        }
        SimpleError::Momentum(crate::MomentumError::BackflowOnZeroGradient { face }) => {
            if let Some((patch, _)) = case.boundaries.iter().find(|(name, condition)| {
                matches!(
                    condition,
                    IncompressibleBoundaryCondition::PressureOutlet { .. }
                ) && case
                    .mesh
                    .boundary_patches()
                    .iter()
                    .any(|p| p.name == *name && p.face_indices.contains(&face))
            }) {
                IncompressibleSolveError::BackflowAtPressureOutlet {
                    patch: patch.clone(),
                    face,
                    flux: state.face_flux[face],
                }
            } else {
                IncompressibleSolveError::Simple(SimpleError::Momentum(
                    crate::MomentumError::BackflowOnZeroGradient { face },
                ))
            }
        }
        error => IncompressibleSolveError::Simple(error),
    })?;
    let status = if simple.converged {
        IncompressibleSolveStatus::Converged
    } else {
        IncompressibleSolveStatus::MaxIterations
    };
    for (patch_name, condition) in &case.boundaries {
        if !matches!(
            condition,
            IncompressibleBoundaryCondition::PressureOutlet { .. }
        ) {
            continue;
        }
        let patch = case
            .mesh
            .boundary_patches()
            .iter()
            .find(|patch| patch.name == *patch_name)
            .expect("resolved pressure-outlet patch exists");
        for &face in &patch.face_indices {
            let flux = state.face_flux[face];
            if flux < 0.0 {
                return Err(IncompressibleSolveError::BackflowAtPressureOutlet {
                    patch: patch_name.clone(),
                    face,
                    flux,
                });
            }
        }
    }
    let (total_inflow, total_outflow) = case
        .mesh
        .faces()
        .iter()
        .enumerate()
        .filter(|(i, face)| {
            face.neighbour.is_none() && resolved.pressure_boundary().cyclic_link(*i).is_none()
        })
        .map(|(face, _)| state.face_flux[face])
        .fold((0.0, 0.0), |(inflow, outflow), flux| {
            if flux < 0.0 {
                (inflow + flux, outflow)
            } else {
                (inflow, outflow + flux)
            }
        });
    Ok(IncompressibleSolution {
        nonlinear_iterations: simple.outer_iterations,
        max_courant: None,
        turbulence: closure.as_ref().map(|c| c.lock().unwrap().fields()),
        temperature: thermal.as_ref().map(|t| t.lock().unwrap().field()),
        physical_time: None,
        last_time_step: None,
        rejected_time_steps: 0,
        completed_time_steps: 0,
        velocity: state.velocity,
        pressure: state.pressure,
        face_flux: state.face_flux,
        report: IncompressibleSolveReport {
            performance: Default::default(),
            cpu_threads: rayon::current_num_threads(),
            status,
            initial_continuity_rms: simple.initial_continuity_rms,
            final_continuity_rms: simple.final_continuity_rms,
            outer_iterations: simple.outer_iterations,
            continuity_history: simple.continuity_history.clone(),
            total_inflow,
            total_outflow,
            net_boundary_flux: total_inflow + total_outflow,
            simple,
        },
    })
}

#[cfg(test)]
mod tetra_cavity_stability_tests {
    use super::*;
    use crate::{BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point};
    #[test]
    fn thin_three_dimensional_tetrahedral_cavity_remains_bounded_and_converges() {
        let (nx, ny, nz) = (4usize, 4usize, 2usize);
        let index = |i: usize, j: usize, k: usize| k * (nx + 1) * (ny + 1) + j * (nx + 1) + i;
        let mut points = Vec::new();
        for k in 0..=nz {
            for j in 0..=ny {
                for i in 0..=nx {
                    points.push(Point::new(
                        i as f64 * 0.1 / nx as f64,
                        j as f64 * 0.1 / ny as f64,
                        k as f64 * 0.01 / nz as f64,
                    ));
                }
            }
        }
        let mut cells = Vec::new();
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let v = [
                        index(i, j, k),
                        index(i + 1, j, k),
                        index(i + 1, j + 1, k),
                        index(i, j + 1, k),
                        index(i, j, k + 1),
                        index(i + 1, j, k + 1),
                        index(i + 1, j + 1, k + 1),
                        index(i, j + 1, k + 1),
                    ];
                    for t in [
                        [0, 1, 2, 6],
                        [0, 2, 3, 6],
                        [0, 3, 7, 6],
                        [0, 7, 4, 6],
                        [0, 4, 5, 6],
                        [0, 5, 1, 6],
                    ] {
                        cells.push(CellDefinition::tetrahedron(t.map(|i| v[i])));
                    }
                }
            }
        }
        let mesh = UnstructuredMesh::from_cells(MeshDimension::ThreeD, points, cells).unwrap();
        let mut wall = vec![];
        let mut lid = vec![];
        for (i, f) in mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_none())
        {
            if (f.center.y - 0.1).abs() < 1e-10 {
                lid.push(i);
            } else {
                wall.push(i);
            }
        }
        let mesh = mesh
            .with_boundary_patches(vec![
                BoundaryPatch {
                    name: "wall".into(),
                    face_indices: wall,
                    boundary_type: BoundaryType::Wall,
                },
                BoundaryPatch {
                    name: "lid".into(),
                    face_indices: lid,
                    boundary_type: BoundaryType::Wall,
                },
            ])
            .unwrap();
        let options = IncompressibleSolverOptions {
            max_outer_iterations: 1000,
            velocity_relaxation: 0.3,
            pressure_relaxation: 0.1,
            ..Default::default()
        };
        let case = IncompressibleCase::steady(
            mesh,
            vec![
                ("wall".into(), IncompressibleBoundaryCondition::NoSlipWall),
                (
                    "lid".into(),
                    IncompressibleBoundaryCondition::MovingWall {
                        velocity: Vec3::new(2., 0., 0.),
                    },
                ),
            ],
            IncompressibleMaterial {
                density: 1.,
                kinematic_viscosity: 0.01,
            },
            options,
        );
        let solution = solve_incompressible(&case).unwrap();
        assert!(solution.report.converged(), "{:?}", solution.report);
        assert!(solution.velocity.values().iter().all(|u| u.norm() < 2.5));
    }
}
