//! High-level case configuration and boundary resolution for steady,
//! constant-density unstructured incompressible flow.
//!
//! This module owns physical case semantics. The numerical SIMPLE iteration
//! remains in `crate::simple`.

use crate::{
    initial_face_flux, solve_simple, CellField, Diffusivity, FaceField, FieldError,
    LeastSquaresGradientStencil, LinearSolverOptions, ResolvedScalarBoundaryConditions,
    ResolvedVelocityBoundaryConditions, ScalarBoundaryCondition, SimpleError, SimpleOptions,
    SimpleReport, SimpleState, UnstructuredMesh, Vec3, VelocityBoundaryCondition,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IncompressibleBoundaryCondition {
    NoSlipWall,
    MovingWall { velocity: Vec3 },
    VelocityInlet { velocity: Vec3 },
    PressureOutlet { pressure: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IncompressibleMaterial {
    pub density: f64,
    pub kinematic_viscosity: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct IncompressibleSolverOptions {
    pub max_outer_iterations: usize,
    pub continuity_absolute_tolerance: f64,
    pub velocity_relaxation: f64,
    pub pressure_relaxation: f64,
    pub momentum_solver: LinearSolverOptions,
    pub pressure_solver: LinearSolverOptions,
}

impl Default for IncompressibleSolverOptions {
    fn default() -> Self {
        Self {
            max_outer_iterations: 200,
            continuity_absolute_tolerance: 1.0e-10,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            momentum_solver: LinearSolverOptions::default(),
            pressure_solver: LinearSolverOptions::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub enum IncompressibleInitialConditions {
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
                IncompressibleBoundaryCondition::PressureOutlet { pressure } => {
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
        let velocity_boundary =
            ResolvedVelocityBoundaryConditions::strict(&self.mesh, &velocity_assignments)?;
        let pressure_boundary =
            ResolvedScalarBoundaryConditions::strict(&self.mesh, &pressure_assignments)?;
        let pressure_correction_boundary =
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
                IncompressibleBoundaryCondition::NoSlipWall => Some(Vec3::ZERO),
                IncompressibleBoundaryCondition::MovingWall { velocity }
                | IncompressibleBoundaryCondition::VelocityInlet { velocity } => Some(velocity),
                IncompressibleBoundaryCondition::PressureOutlet { .. } => None,
            };
            if let Some(velocity) = velocity {
                for &face in &patch.face_indices {
                    boundary_flux[face] = velocity.dot(self.mesh.faces()[face].area_vector);
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
    Field(FieldError),
    Numerics(crate::NumericsError),
    UnknownBoundaryPatch { patch: String },
    InvalidInitialConditions,
    InvalidDensity { value: f64 },
    InvalidKinematicViscosity { value: f64 },
    InvalidVelocityBoundary { patch: String, velocity: Vec3 },
    InvalidOutletPressure { patch: String, pressure: f64 },
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
    pub velocity: CellField<Vec3>,
    pub pressure: CellField<f64>,
    pub face_flux: FaceField<f64>,
    pub report: IncompressibleSolveReport,
}

impl IncompressibleSolution {
    /// Returns the cell-centred velocity magnitude for result export and
    /// visualization without reconstructing fluxes or mutating solution state.
    pub fn speed(&self) -> CellField<f64> {
        self.velocity.map(|velocity| velocity.norm())
    }
}

#[derive(Clone, Debug)]
pub enum IncompressibleSolveError {
    Case(IncompressibleCaseError),
    Simple(SimpleError),
    BackflowAtPressureOutlet {
        patch: String,
        face: usize,
        flux: f64,
    },
}

impl std::fmt::Display for IncompressibleSolveError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
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
    let resolved = case.resolve_boundaries()?;
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
        IncompressibleInitialConditions::Fields { velocity, pressure } => {
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
    let face_flux = initial_face_flux(&case.mesh, &velocity, resolved.boundary_flux())?;
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
        )
    }) {
        options.pressure_boundary = Some(resolved.pressure_boundary());
        options.pressure_correction_boundary = Some(resolved.pressure_correction_boundary());
    }
    options.max_outer_iterations = case.solver.max_outer_iterations;
    options.continuity_absolute_tolerance = case.solver.continuity_absolute_tolerance;
    options.velocity_relaxation = case.solver.velocity_relaxation;
    options.pressure_relaxation = case.solver.pressure_relaxation;
    options.momentum_solver = case.solver.momentum_solver;
    options.pressure_solver = case.solver.pressure_solver;
    let simple = solve_simple(&case.mesh, &mut state, options)?;
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
        .filter(|(_, face)| face.neighbour.is_none())
        .map(|(face, _)| state.face_flux[face])
        .fold((0.0, 0.0), |(inflow, outflow), flux| {
            if flux < 0.0 {
                (inflow + flux, outflow)
            } else {
                (inflow, outflow + flux)
            }
        });
    Ok(IncompressibleSolution {
        velocity: state.velocity,
        pressure: state.pressure,
        face_flux: state.face_flux,
        report: IncompressibleSolveReport {
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
