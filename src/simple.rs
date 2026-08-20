//! Collocated unstructured SIMPLE pressure-velocity coupling.
//!
//! This module is intentionally separate from the Phase 6 momentum predictor.
//! It will own Rhie--Chow flux interpolation, pressure correction, and the
//! segregated SIMPLE outer iteration.

use crate::{
    assemble_momentum_component, interpolate_scalar, interpolate_vector_into,
    least_squares_gradient, momentum_component_field, pressure_gradient_source,
    solve_momentum_velocity, CellField, CsrBuilder, CsrMatrix, DiffusionOptions, Diffusivity,
    FaceField, FieldError, LeastSquaresGradientStencil, LinearAlgebraError, LinearSolveReport,
    LinearSolverOptions, MomentumComponent, MomentumError, MomentumOptions, NumericsError,
    ResolvedScalarBoundaryConditions, ResolvedVelocityBoundaryConditions, ScalarBoundaryCondition,
    ScalarBoundaryValue, UnstructuredMesh, Vec3,
};

#[derive(Clone, Debug, PartialEq)]
pub enum SimpleError {
    Field(FieldError),
    Numerics(NumericsError),
    Linear(LinearAlgebraError),
    Momentum(MomentumError),
    InvalidMomentumDiagonal { cell: usize, value: f64 },
    InvalidPressureFaceCoefficient { face: usize, value: f64 },
    InvalidPressureReference { cell: usize, cell_count: usize },
    InvalidPressureRelaxation { value: f64 },
    InvalidOuterIterations { value: usize },
    MomentumLinearDidNotConverge,
    PressureLinearDidNotConverge,
}

impl std::fmt::Display for SimpleError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for SimpleError {}

impl From<FieldError> for SimpleError {
    fn from(value: FieldError) -> Self {
        Self::Field(value)
    }
}

impl From<NumericsError> for SimpleError {
    fn from(value: NumericsError) -> Self {
        Self::Numerics(value)
    }
}

impl From<LinearAlgebraError> for SimpleError {
    fn from(value: LinearAlgebraError) -> Self {
        Self::Linear(value)
    }
}

impl From<MomentumError> for SimpleError {
    fn from(value: MomentumError) -> Self {
        Self::Momentum(value)
    }
}

/// Coupled, mesh-bound collocated SIMPLE state. After the first iteration,
/// `face_flux` is authoritative and is never regenerated from cell velocity.
#[derive(Clone, Debug)]
pub struct SimpleState {
    pub velocity: CellField<Vec3>,
    pub pressure: CellField<f64>,
    pub face_flux: FaceField<f64>,
}

impl SimpleState {
    pub fn new(
        mesh: &UnstructuredMesh,
        velocity: CellField<Vec3>,
        pressure: CellField<f64>,
        face_flux: FaceField<f64>,
    ) -> Result<Self, SimpleError> {
        velocity.ensure_mesh(mesh)?;
        pressure.ensure_mesh(mesh)?;
        face_flux.ensure_mesh(mesh)?;
        Ok(Self {
            velocity,
            pressure,
            face_flux,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SimpleOptions<'a> {
    pub viscosity: Diffusivity<'a>,
    pub velocity_boundary: &'a ResolvedVelocityBoundaryConditions,
    pub pressure_stencil: &'a LeastSquaresGradientStencil,
    /// Physical pressure values used by Rhie--Chow at pressure outlets.
    pub pressure_boundary: Option<&'a ResolvedScalarBoundaryConditions>,
    /// `ZeroGradient` preserves prescribed velocity-boundary flux; `FixedValue`
    /// supplies a pressure-correction Dirichlet face and allows its flux to move.
    pub pressure_correction_boundary: Option<&'a ResolvedScalarBoundaryConditions>,
    pub reference_cell: usize,
    pub max_outer_iterations: usize,
    pub continuity_absolute_tolerance: f64,
    pub momentum_solver: LinearSolverOptions,
    pub pressure_solver: LinearSolverOptions,
    pub velocity_relaxation: f64,
    pub pressure_relaxation: f64,
}

impl<'a> SimpleOptions<'a> {
    pub fn steady(
        viscosity: Diffusivity<'a>,
        velocity_boundary: &'a ResolvedVelocityBoundaryConditions,
        pressure_stencil: &'a LeastSquaresGradientStencil,
    ) -> Self {
        Self {
            viscosity,
            velocity_boundary,
            pressure_stencil,
            pressure_boundary: None,
            pressure_correction_boundary: None,
            reference_cell: 0,
            max_outer_iterations: 200,
            continuity_absolute_tolerance: 1.0e-10,
            momentum_solver: LinearSolverOptions::default(),
            pressure_solver: LinearSolverOptions::default(),
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SimpleReport {
    pub converged: bool,
    pub outer_iterations: usize,
    pub initial_continuity_rms: f64,
    pub final_continuity_rms: f64,
    pub continuity_history: Vec<f64>,
    pub momentum_reports: Option<[LinearSolveReport; 3]>,
    pub pressure_report: Option<LinearSolveReport>,
}

/// Executes steady, constant-density SIMPLE outer iterations for the initial
/// closed-wall baseline. Boundary fluxes already stored in `state.face_flux`
/// are preserved by Rhie--Chow; nonzero open-flow boundary semantics are not
/// part of this first API.
pub fn solve_simple(
    mesh: &UnstructuredMesh,
    state: &mut SimpleState,
    options: SimpleOptions<'_>,
) -> Result<SimpleReport, SimpleError> {
    state.velocity.ensure_mesh(mesh)?;
    state.pressure.ensure_mesh(mesh)?;
    state.face_flux.ensure_mesh(mesh)?;
    options.pressure_stencil.ensure_mesh(mesh)?;
    if options.max_outer_iterations == 0 {
        return Err(SimpleError::InvalidOuterIterations { value: 0 });
    }
    if !(options.pressure_relaxation.is_finite()
        && 0.0 < options.pressure_relaxation
        && options.pressure_relaxation <= 1.0)
    {
        return Err(SimpleError::InvalidPressureRelaxation {
            value: options.pressure_relaxation,
        });
    }
    if !(options.continuity_absolute_tolerance.is_finite()
        && options.continuity_absolute_tolerance >= 0.0)
    {
        return Err(SimpleError::InvalidPressureRelaxation {
            value: options.continuity_absolute_tolerance,
        });
    }
    let initial = continuity_rms(mesh, &state.face_flux)?;
    let mut history = vec![initial];

    let mut last_momentum = None;
    let mut last_pressure = None;
    for iteration in 1..=options.max_outer_iterations {
        let pressure_source =
            pressure_gradient_source(mesh, &state.pressure, options.pressure_stencil, 1.0)?;
        let source_x = momentum_component_field(mesh, &pressure_source, MomentumComponent::X)?;
        let source_y = momentum_component_field(mesh, &pressure_source, MomentumComponent::Y)?;
        let source_z = momentum_component_field(mesh, &pressure_source, MomentumComponent::Z)?;
        let current_x = momentum_component_field(mesh, &state.velocity, MomentumComponent::X)?;
        let current_y = momentum_component_field(mesh, &state.velocity, MomentumComponent::Y)?;
        let current_z = momentum_component_field(mesh, &state.velocity, MomentumComponent::Z)?;
        let relaxed = options.velocity_relaxation < 1.0;
        let assemble = |component, source, current| {
            assemble_momentum_component(
                mesh,
                MomentumOptions {
                    viscosity: options.viscosity,
                    boundary: options.velocity_boundary.component(component),
                    flux: &state.face_flux,
                    old: None,
                    time_step: None,
                    source: Some(source),
                    relaxation_field: relaxed.then_some(current),
                    correction_field: None,
                    least_squares_stencil: None,
                    under_relaxation: options.velocity_relaxation,
                    diffusion: DiffusionOptions::default(),
                },
            )
        };
        let x_system = assemble(MomentumComponent::X, &source_x, &current_x)?;
        let y_system = assemble(MomentumComponent::Y, &source_y, &current_y)?;
        let z_system = assemble(MomentumComponent::Z, &source_z, &current_z)?;
        let mut predictor = state.velocity.clone();
        let momentum_reports = solve_momentum_velocity(
            [&x_system, &y_system, &z_system],
            &mut predictor,
            options.momentum_solver,
        )?;
        if !momentum_reports.iter().all(LinearSolveReport::converged) {
            return Err(SimpleError::MomentumLinearDidNotConverge);
        }
        let diagonal = x_system.matrix().diagonal()?;
        let r_au = CellField::from_values(mesh, momentum_inverse_diagonal(&diagonal)?)?;
        let mut predicted_flux = rhie_chow_predicted_flux(
            mesh,
            &predictor,
            &state.pressure,
            &r_au,
            options.pressure_stencil,
            &state.face_flux,
        )?;
        if let Some(correction_boundary) = options.pressure_correction_boundary {
            correction_boundary.ensure_mesh(mesh)?;
            for (face_index, face) in mesh.faces().iter().enumerate() {
                if face.neighbour.is_none()
                    && matches!(
                        correction_boundary.condition(face_index),
                        Some(ScalarBoundaryCondition::FixedValue(_))
                    )
                {
                    predicted_flux[face_index] = predictor[face.owner].dot(face.area_vector);
                }
            }
        }
        let coefficients = match options.pressure_correction_boundary {
            Some(boundary) => pressure_face_coefficients_with_boundary(mesh, &r_au, boundary)?,
            None => pressure_face_coefficients(mesh, &r_au)?,
        };
        let pressure_system = match options.pressure_correction_boundary {
            Some(boundary) => assemble_pressure_correction_with_boundary(
                mesh,
                &predicted_flux,
                &coefficients,
                boundary,
            )?,
            None => assemble_pressure_correction(
                mesh,
                &predicted_flux,
                &coefficients,
                options.reference_cell,
            )?,
        };
        let mut correction = CellField::filled(mesh, 0.0);
        let pressure_report =
            solve_pressure_correction(&pressure_system, &mut correction, options.pressure_solver)?;
        if !pressure_report.converged() {
            return Err(SimpleError::PressureLinearDidNotConverge);
        }
        for (pressure, correction) in state
            .pressure
            .values_mut()
            .iter_mut()
            .zip(correction.values())
        {
            *pressure += options.pressure_relaxation * correction;
        }
        let correction_gradient =
            least_squares_gradient(mesh, options.pressure_stencil, &correction)?;
        state.velocity =
            correct_cell_velocity_from_gradient(mesh, &predictor, &r_au, &correction_gradient)?;
        state.face_flux = match options.pressure_correction_boundary {
            Some(boundary) => correct_face_flux_with_boundary(
                mesh,
                &predicted_flux,
                &coefficients,
                &correction,
                boundary,
            )?,
            None => correct_face_flux(mesh, &predicted_flux, &coefficients, &correction)?,
        };
        let continuity = continuity_rms(mesh, &state.face_flux)?;
        history.push(continuity);
        last_momentum = Some(momentum_reports);
        last_pressure = Some(pressure_report);
        if continuity <= options.continuity_absolute_tolerance {
            return Ok(SimpleReport {
                converged: true,
                outer_iterations: iteration,
                initial_continuity_rms: initial,
                final_continuity_rms: continuity,
                continuity_history: history,
                momentum_reports: last_momentum,
                pressure_report: last_pressure,
            });
        }
    }
    Ok(SimpleReport {
        converged: false,
        outer_iterations: options.max_outer_iterations,
        initial_continuity_rms: initial,
        final_continuity_rms: *history.last().expect("initial continuity exists"),
        continuity_history: history,
        momentum_reports: last_momentum,
        pressure_report: last_pressure,
    })
}

/// Computes `rAU = 1/a_P` from the assembled, post-equation-relaxation
/// momentum diagonal. SIMPLE must use this diagonal because it is the
/// coefficient of the solved predictor equation.
pub fn momentum_inverse_diagonal(diagonal: &[f64]) -> Result<Vec<f64>, SimpleError> {
    diagonal
        .iter()
        .copied()
        .enumerate()
        .map(|(cell, value)| {
            if !value.is_finite() || value <= 0.0 {
                Err(SimpleError::InvalidMomentumDiagonal { cell, value })
            } else {
                Ok(1.0 / value)
            }
        })
        .collect()
}

/// Builds the internal pressure-response coefficient `d_f` for a unit-density
/// collocated SIMPLE discretization. `rAU` is linearly interpolated to the face
/// with the established geometric interpolation weight and then multiplied by
/// the projection-consistent area coefficient `(S_f . d) / (d . d)`.
///
/// Boundary coefficients are zero because the initial supported SIMPLE
/// boundary is an impermeable velocity wall, where pressure correction must not
/// alter the prescribed normal flux.
pub fn pressure_face_coefficients(
    mesh: &UnstructuredMesh,
    r_au: &CellField<f64>,
) -> Result<FaceField<f64>, SimpleError> {
    r_au.ensure_mesh(mesh)?;
    let face_r_au = interpolate_scalar(mesh, r_au, ScalarBoundaryValue::OwnerValue)?;
    let mut coefficients = FaceField::filled(mesh, 0.0);
    for (face_index, face) in mesh.faces().iter().enumerate() {
        let Some(neighbour) = face.neighbour else {
            continue;
        };
        let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
        let d2 = d.norm_squared();
        if !d2.is_finite() || d2 <= f64::EPSILON {
            return Err(
                NumericsError::DegenerateOwnerNeighbourDistance { face: face_index }.into(),
            );
        }
        let coefficient = face_r_au[face_index] * face.area_vector.dot(d) / d2;
        if !coefficient.is_finite() || coefficient <= 0.0 {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: coefficient,
            });
        }
        coefficients[face_index] = coefficient;
    }
    Ok(coefficients)
}

/// Extends pressure response coefficients to fixed pressure-correction faces.
/// `ZeroGradient` boundaries retain zero coefficient and therefore preserve
/// caller-prescribed velocity-boundary fluxes.
pub fn pressure_face_coefficients_with_boundary(
    mesh: &UnstructuredMesh,
    r_au: &CellField<f64>,
    boundary: &ResolvedScalarBoundaryConditions,
) -> Result<FaceField<f64>, SimpleError> {
    boundary.ensure_mesh(mesh)?;
    let mut coefficients = pressure_face_coefficients(mesh, r_au)?;
    for (face_index, face) in mesh.faces().iter().enumerate() {
        if face.neighbour.is_some()
            || !matches!(
                boundary.condition(face_index),
                Some(ScalarBoundaryCondition::FixedValue(_))
            )
        {
            continue;
        }
        let d = face.center - mesh.cells()[face.owner].center;
        let d2 = d.norm_squared();
        if !d2.is_finite() || d2 <= f64::EPSILON {
            return Err(NumericsError::DegenerateOwnerFaceDistance { face: face_index }.into());
        }
        let coefficient = r_au[face.owner] * face.area_vector.dot(d) / d2;
        if !coefficient.is_finite() || coefficient <= 0.0 {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: coefficient,
            });
        }
        coefficients[face_index] = coefficient;
    }
    Ok(coefficients)
}

/// Applies the pressure correction to the authoritative owner-oriented face
/// flux. Only internal faces are changed: `Phi = Phi* - d_f (p'_N - p'_P)`.
/// Keeping wall values unchanged enforces impermeability for the first supported
/// closed-cavity SIMPLE boundary model.
pub fn correct_face_flux(
    mesh: &UnstructuredMesh,
    predicted: &FaceField<f64>,
    coefficients: &FaceField<f64>,
    pressure_correction: &CellField<f64>,
) -> Result<FaceField<f64>, SimpleError> {
    predicted.ensure_mesh(mesh)?;
    coefficients.ensure_mesh(mesh)?;
    pressure_correction.ensure_mesh(mesh)?;
    let mut corrected = predicted.clone();
    for (face_index, face) in mesh.faces().iter().enumerate() {
        let Some(neighbour) = face.neighbour else {
            continue;
        };
        let coefficient = coefficients[face_index];
        if !coefficient.is_finite() || coefficient <= 0.0 {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: coefficient,
            });
        }
        corrected[face_index] -=
            coefficient * (pressure_correction[neighbour] - pressure_correction[face.owner]);
        if !corrected[face_index].is_finite() {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: corrected[face_index],
            });
        }
    }
    Ok(corrected)
}

/// Corrects internal faces and fixed pressure-correction boundary faces. For a
/// boundary condition `p'_b`, the owner-oriented update is
/// `Phi_b = Phi_b* - d_b (p'_b - p'_P)`.
pub fn correct_face_flux_with_boundary(
    mesh: &UnstructuredMesh,
    predicted: &FaceField<f64>,
    coefficients: &FaceField<f64>,
    pressure_correction: &CellField<f64>,
    boundary: &ResolvedScalarBoundaryConditions,
) -> Result<FaceField<f64>, SimpleError> {
    predicted.ensure_mesh(mesh)?;
    coefficients.ensure_mesh(mesh)?;
    pressure_correction.ensure_mesh(mesh)?;
    boundary.ensure_mesh(mesh)?;
    let mut corrected = predicted.clone();
    for (face_index, face) in mesh.faces().iter().enumerate() {
        let (boundary_correction, coefficient) = if let Some(neighbour) = face.neighbour {
            (pressure_correction[neighbour], coefficients[face_index])
        } else {
            match boundary.condition(face_index) {
                Some(ScalarBoundaryCondition::FixedValue(value)) => {
                    (value, coefficients[face_index])
                }
                Some(ScalarBoundaryCondition::ZeroGradient) => continue,
                None => {
                    return Err(NumericsError::MissingBoundaryCondition { face: face_index }.into())
                }
            }
        };
        if !coefficient.is_finite() || coefficient <= 0.0 {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: coefficient,
            });
        }
        corrected[face_index] -=
            coefficient * (boundary_correction - pressure_correction[face.owner]);
        if !corrected[face_index].is_finite() {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: corrected[face_index],
            });
        }
    }
    Ok(corrected)
}

/// SPD pressure-correction system for the discrete flux update
/// `Phi = Phi* - d_f (p'_N - p'_P)`. The equation is
/// `A_p p' = -div(Phi*)`; internal entries are `[+d_f, -d_f; -d_f, +d_f]`.
/// A zero correction at `reference_cell` removes the constant null space by
/// symmetric row/column elimination.
#[derive(Clone, Debug)]
pub struct PressureCorrectionSystem {
    mesh_id: crate::MeshId,
    matrix: CsrMatrix,
    rhs: Vec<f64>,
}

impl PressureCorrectionSystem {
    pub fn matrix(&self) -> &CsrMatrix {
        &self.matrix
    }

    pub fn rhs(&self) -> &[f64] {
        &self.rhs
    }

    pub fn true_residual(&self, correction: &[f64]) -> Result<Vec<f64>, SimpleError> {
        Ok(crate::residual(&self.matrix, correction, &self.rhs)?)
    }
}

pub fn assemble_pressure_correction(
    mesh: &UnstructuredMesh,
    predicted_flux: &FaceField<f64>,
    coefficients: &FaceField<f64>,
    reference_cell: usize,
) -> Result<PressureCorrectionSystem, SimpleError> {
    predicted_flux.ensure_mesh(mesh)?;
    coefficients.ensure_mesh(mesh)?;
    if reference_cell >= mesh.cell_count() {
        return Err(SimpleError::InvalidPressureReference {
            cell: reference_cell,
            cell_count: mesh.cell_count(),
        });
    }
    let mut rhs = vec![0.0; mesh.cell_count()];
    let mut entries = Vec::with_capacity(mesh.face_count() * 4);
    for (face_index, face) in mesh.faces().iter().enumerate() {
        let flux = predicted_flux[face_index];
        if !flux.is_finite() {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: flux,
            });
        }
        rhs[face.owner] -= flux;
        let Some(neighbour) = face.neighbour else {
            continue;
        };
        rhs[neighbour] += flux;
        let coefficient = coefficients[face_index];
        if !coefficient.is_finite() || coefficient <= 0.0 {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: coefficient,
            });
        }
        entries.extend_from_slice(&[
            (face.owner, face.owner, coefficient),
            (face.owner, neighbour, -coefficient),
            (neighbour, face.owner, -coefficient),
            (neighbour, neighbour, coefficient),
        ]);
    }
    let mut constrained = Vec::with_capacity(entries.len() + 1);
    for (row, column, value) in entries {
        if row == reference_cell || column == reference_cell {
            continue;
        }
        constrained.push((row, column, value));
    }
    rhs[reference_cell] = 0.0;
    constrained.push((reference_cell, reference_cell, 1.0));
    let mut builder = CsrBuilder::new(mesh.cell_count(), mesh.cell_count());
    for (row, column, value) in constrained {
        builder.add(row, column, value)?;
    }
    Ok(PressureCorrectionSystem {
        mesh_id: mesh.id(),
        matrix: builder.finalize()?,
        rhs,
    })
}

/// Assembles a pressure-correction system anchored by fixed correction values
/// at one or more exterior faces, without an artificial reference cell.
pub fn assemble_pressure_correction_with_boundary(
    mesh: &UnstructuredMesh,
    predicted_flux: &FaceField<f64>,
    coefficients: &FaceField<f64>,
    boundary: &ResolvedScalarBoundaryConditions,
) -> Result<PressureCorrectionSystem, SimpleError> {
    predicted_flux.ensure_mesh(mesh)?;
    coefficients.ensure_mesh(mesh)?;
    boundary.ensure_mesh(mesh)?;
    let mut rhs = vec![0.0; mesh.cell_count()];
    let mut builder = CsrBuilder::new(mesh.cell_count(), mesh.cell_count());
    let mut anchored = false;
    for (face_index, face) in mesh.faces().iter().enumerate() {
        let flux = predicted_flux[face_index];
        if !flux.is_finite() {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: flux,
            });
        }
        rhs[face.owner] -= flux;
        if let Some(neighbour) = face.neighbour {
            rhs[neighbour] += flux;
            let coefficient = coefficients[face_index];
            if !coefficient.is_finite() || coefficient <= 0.0 {
                return Err(SimpleError::InvalidPressureFaceCoefficient {
                    face: face_index,
                    value: coefficient,
                });
            }
            builder.add(face.owner, face.owner, coefficient)?;
            builder.add(face.owner, neighbour, -coefficient)?;
            builder.add(neighbour, face.owner, -coefficient)?;
            builder.add(neighbour, neighbour, coefficient)?;
        } else if let Some(ScalarBoundaryCondition::FixedValue(value)) =
            boundary.condition(face_index)
        {
            let coefficient = coefficients[face_index];
            if !coefficient.is_finite() || coefficient <= 0.0 || !value.is_finite() {
                return Err(SimpleError::InvalidPressureFaceCoefficient {
                    face: face_index,
                    value: coefficient,
                });
            }
            builder.add(face.owner, face.owner, coefficient)?;
            rhs[face.owner] += coefficient * value;
            anchored = true;
        }
    }
    if !anchored {
        return Err(SimpleError::InvalidPressureReference {
            cell: 0,
            cell_count: mesh.cell_count(),
        });
    }
    Ok(PressureCorrectionSystem {
        mesh_id: mesh.id(),
        matrix: builder.finalize()?,
        rhs,
    })
}

/// Solves the SPD pressure-correction system with Jacobi-preconditioned CG.
/// The correction field is mesh-bound so a same-sized field from another mesh
/// cannot become pressure state accidentally.
pub fn solve_pressure_correction(
    system: &PressureCorrectionSystem,
    correction: &mut CellField<f64>,
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, SimpleError> {
    if correction.mesh_id() != system.mesh_id {
        return Err(FieldError::MeshMismatch {
            expected: system.mesh_id,
            actual: correction.mesh_id(),
        }
        .into());
    }
    let preconditioner = crate::JacobiPreconditioner::new(&system.matrix)?;
    Ok(crate::pcg(
        &system.matrix,
        &system.rhs,
        correction.values_mut(),
        &preconditioner,
        options,
    )?)
}

/// Applies the cell-centred SIMPLE velocity update `U = U* - rAU grad(p')`.
/// The pressure-correction gradient is supplied separately so the sign and
/// scaling remain independently testable from LSQ reconstruction.
pub fn correct_cell_velocity_from_gradient(
    mesh: &UnstructuredMesh,
    predictor: &CellField<Vec3>,
    r_au: &CellField<f64>,
    pressure_gradient: &CellField<Vec3>,
) -> Result<CellField<Vec3>, SimpleError> {
    predictor.ensure_mesh(mesh)?;
    r_au.ensure_mesh(mesh)?;
    pressure_gradient.ensure_mesh(mesh)?;
    let mut corrected = predictor.clone();
    for cell in 0..mesh.cell_count() {
        let inverse_diagonal = r_au[cell];
        if !inverse_diagonal.is_finite() || inverse_diagonal <= 0.0 {
            return Err(SimpleError::InvalidMomentumDiagonal {
                cell,
                value: inverse_diagonal,
            });
        }
        corrected[cell] = corrected[cell] - pressure_gradient[cell] * inverse_diagonal;
        let value = corrected[cell];
        if !(value.x.is_finite() && value.y.is_finite() && value.z.is_finite()) {
            return Err(SimpleError::InvalidMomentumDiagonal {
                cell,
                value: inverse_diagonal,
            });
        }
    }
    Ok(corrected)
}

/// Returns the RMS norm of the directly evaluated integrated cell continuity
/// imbalance. Its units are volumetric flux; it is not a volume-normalized
/// divergence and is computed from the authoritative corrected `FaceField`.
pub fn continuity_rms(mesh: &UnstructuredMesh, flux: &FaceField<f64>) -> Result<f64, SimpleError> {
    let imbalance = crate::integrated_divergence(mesh, flux)?;
    let squared_sum: f64 = imbalance.values().iter().map(|value| value * value).sum();
    let result = (squared_sum / mesh.cell_count() as f64).sqrt();
    if result.is_finite() {
        Ok(result)
    } else {
        Err(SimpleError::InvalidPressureFaceCoefficient {
            face: 0,
            value: result,
        })
    }
}

/// Initializes internal transport fluxes from geometrically interpolated
/// cell-centred velocity. Prescribed boundary normal fluxes are copied from
/// `wall_flux`; after SIMPLE starts, `SimpleState::face_flux` is authoritative
/// and must instead be advanced by direct pressure-flux correction.
pub fn initial_face_flux(
    mesh: &UnstructuredMesh,
    velocity: &CellField<Vec3>,
    wall_flux: &FaceField<f64>,
) -> Result<FaceField<f64>, SimpleError> {
    velocity.ensure_mesh(mesh)?;
    wall_flux.ensure_mesh(mesh)?;
    let mut face_velocity = FaceField::filled(mesh, Vec3::ZERO);
    interpolate_vector_into(
        mesh,
        velocity,
        ScalarBoundaryValue::OwnerValue,
        &mut face_velocity,
    )?;
    let mut flux = wall_flux.clone();
    for (face, geometry) in mesh.faces().iter().enumerate() {
        if geometry.neighbour.is_some() {
            flux[face] = face_velocity[face].dot(geometry.area_vector);
        }
    }
    Ok(flux)
}

/// Constructs the pressure-aware Rhie--Chow predictor flux for internal faces.
/// Given cell predictor values `U* = H/A - rAU grad(p)`, the face formula is
/// `Phi* = interpolate(U*) . S + interpolate(rAU grad(p)) . S
///         - d_f (p_N - p_P)`.
/// The interpolated gradient term removes the cell-centre predictor pressure
/// contribution before the direct projection-consistent pressure difference is
/// applied. This direct term is what couples checkerboard pressure modes.
///
/// Boundary fluxes are caller-provided prescribed normal wall fluxes and are
/// copied unchanged, so pressure correction cannot create wall leakage.
pub fn rhie_chow_predicted_flux(
    mesh: &UnstructuredMesh,
    predictor_velocity: &CellField<Vec3>,
    pressure: &CellField<f64>,
    r_au: &CellField<f64>,
    pressure_stencil: &LeastSquaresGradientStencil,
    wall_flux: &FaceField<f64>,
) -> Result<FaceField<f64>, SimpleError> {
    predictor_velocity.ensure_mesh(mesh)?;
    pressure.ensure_mesh(mesh)?;
    r_au.ensure_mesh(mesh)?;
    pressure_stencil.ensure_mesh(mesh)?;
    wall_flux.ensure_mesh(mesh)?;

    let mut face_velocity = FaceField::filled(mesh, Vec3::ZERO);
    interpolate_vector_into(
        mesh,
        predictor_velocity,
        ScalarBoundaryValue::OwnerValue,
        &mut face_velocity,
    )?;
    let pressure_gradient = least_squares_gradient(mesh, pressure_stencil, pressure)?;
    let pressure_response =
        CellField::from_cells(mesh, |cell, _| pressure_gradient[cell] * r_au[cell]);
    let mut face_pressure_response = FaceField::filled(mesh, Vec3::ZERO);
    interpolate_vector_into(
        mesh,
        &pressure_response,
        ScalarBoundaryValue::OwnerValue,
        &mut face_pressure_response,
    )?;
    let coefficients = pressure_face_coefficients(mesh, r_au)?;
    let mut flux = wall_flux.clone();
    for (face_index, face) in mesh.faces().iter().enumerate() {
        let Some(neighbour) = face.neighbour else {
            continue;
        };
        flux[face_index] = face_velocity[face_index].dot(face.area_vector)
            + face_pressure_response[face_index].dot(face.area_vector)
            - coefficients[face_index] * (pressure[neighbour] - pressure[face.owner]);
        if !flux[face_index].is_finite() {
            return Err(SimpleError::InvalidPressureFaceCoefficient {
                face: face_index,
                value: flux[face_index],
            });
        }
    }
    Ok(flux)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BoundaryPatch, BoundaryType, CellDefinition, CellField, Diffusivity,
        LeastSquaresGradientStencil, MeshDimension, Point, ResolvedScalarBoundaryConditions,
        ResolvedVelocityBoundaryConditions, ScalarBoundaryCondition, UnstructuredMesh, Vec3,
        VelocityBoundaryCondition,
    };

    fn two_cells() -> UnstructuredMesh {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(2.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(2.0, 1.0, 0.0),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 4, 3]),
                CellDefinition::polygon(vec![1, 2, 5, 4]),
            ],
        )
        .unwrap();
        let exterior = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(index, face)| face.neighbour.is_none().then_some(index))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "wall".into(),
            face_indices: exterior,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    fn square_grid(nx: usize, ny: usize) -> UnstructuredMesh {
        let point = |x: usize, y: usize| x + (nx + 1) * y;
        let points = (0..=ny)
            .flat_map(|y| (0..=nx).map(move |x| Point::new(x as f64, y as f64, 0.0)))
            .collect();
        let cells = (0..ny)
            .flat_map(|y| {
                (0..nx).map(move |x| {
                    CellDefinition::polygon(vec![
                        point(x, y),
                        point(x + 1, y),
                        point(x + 1, y + 1),
                        point(x, y + 1),
                    ])
                })
            })
            .collect();
        let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
        let exterior = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(index, face)| face.neighbour.is_none().then_some(index))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "wall".into(),
            face_indices: exterior,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    fn cavity_grid(nx: usize, ny: usize) -> UnstructuredMesh {
        let point = |x: usize, y: usize| x + (nx + 1) * y;
        let points = (0..=ny)
            .flat_map(|y| (0..=nx).map(move |x| Point::new(x as f64, y as f64, 0.0)))
            .collect();
        let cells = (0..ny)
            .flat_map(|y| {
                (0..nx).map(move |x| {
                    CellDefinition::polygon(vec![
                        point(x, y),
                        point(x + 1, y),
                        point(x + 1, y + 1),
                        point(x, y + 1),
                    ])
                })
            })
            .collect();
        let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
        let patches = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_none())
            .map(|(face, _)| BoundaryPatch {
                name: format!("face-{face}"),
                face_indices: vec![face],
                boundary_type: BoundaryType::Wall,
            })
            .collect();
        mesh.with_boundary_patches(patches).unwrap()
    }

    fn skewed_cavity_grid(nx: usize, ny: usize) -> UnstructuredMesh {
        let point = |x: usize, y: usize| x + (nx + 1) * y;
        let points = (0..=ny)
            .flat_map(|y| {
                (0..=nx).map(move |x| Point::new(x as f64 + 0.2 * y as f64, y as f64, 0.0))
            })
            .collect();
        let cells = (0..ny)
            .flat_map(|y| {
                (0..nx).map(move |x| {
                    CellDefinition::polygon(vec![
                        point(x, y),
                        point(x + 1, y),
                        point(x + 1, y + 1),
                        point(x, y + 1),
                    ])
                })
            })
            .collect();
        let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
        let patches = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_none())
            .map(|(face, _)| BoundaryPatch {
                name: format!("face-{face}"),
                face_indices: vec![face],
                boundary_type: BoundaryType::Wall,
            })
            .collect();
        mesh.with_boundary_patches(patches).unwrap()
    }

    fn hexahedral_grid(nx: usize, ny: usize, nz: usize) -> UnstructuredMesh {
        let point = |x: usize, y: usize, z: usize| x + (nx + 1) * (y + (ny + 1) * z);
        let points = (0..=nz)
            .flat_map(|z| {
                (0..=ny).flat_map(move |y| {
                    (0..=nx).map(move |x| Point::new(x as f64, y as f64, z as f64))
                })
            })
            .collect();
        let cells = (0..nz)
            .flat_map(|z| {
                (0..ny).flat_map(move |y| {
                    (0..nx).map(move |x| {
                        CellDefinition::Hexahedron([
                            point(x, y, z),
                            point(x + 1, y, z),
                            point(x + 1, y + 1, z),
                            point(x, y + 1, z),
                            point(x, y, z + 1),
                            point(x + 1, y, z + 1),
                            point(x + 1, y + 1, z + 1),
                            point(x, y + 1, z + 1),
                        ])
                    })
                })
            })
            .collect();
        let mesh = UnstructuredMesh::from_cells(MeshDimension::ThreeD, points, cells).unwrap();
        let exterior = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(face, geometry)| geometry.neighbour.is_none().then_some(face))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "wall".into(),
            face_indices: exterior,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    #[test]
    fn momentum_inverse_diagonal_matches_the_hand_calculation() {
        let inverse = momentum_inverse_diagonal(&[2.0, 4.0, 0.5]).unwrap();
        assert_eq!(inverse, vec![0.5, 0.25, 2.0]);
    }

    #[test]
    fn pressure_face_coefficient_matches_the_two_cell_hand_calculation() {
        let mesh = two_cells();
        let r_au = CellField::from_values(&mesh, vec![0.5, 1.5]).unwrap();
        let coefficients = pressure_face_coefficients(&mesh, &r_au).unwrap();
        let internal = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_some())
            .unwrap();
        // rAU_f = (0.5 + 1.5) / 2 = 1, |S| = |d| = 1.
        assert_eq!(coefficients[internal], 1.0);
        for (face, value) in coefficients.iter().enumerate() {
            if face != internal {
                assert_eq!(*value, 0.0);
            }
        }
    }

    #[test]
    fn pressure_flux_correction_matches_the_two_cell_hand_calculation() {
        let mesh = two_cells();
        let predicted =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_some() {
                        3.0
                    } else {
                        0.0
                    }
                },
            );
        let coefficients =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_some() {
                        2.0
                    } else {
                        0.0
                    }
                },
            );
        let correction = CellField::from_values(&mesh, vec![0.0, 1.5]).unwrap();
        let corrected = correct_face_flux(&mesh, &predicted, &coefficients, &correction).unwrap();
        let internal = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_some())
            .unwrap();
        // Phi = Phi* - d_f (p'_N - p'_P) = 3 - 2 * 1.5 = 0.
        assert_eq!(corrected[internal], 0.0);
        for (face, value) in corrected.iter().enumerate() {
            if face != internal {
                assert_eq!(*value, 0.0);
            }
        }
    }

    #[test]
    fn pressure_correction_two_cell_matrix_rhs_and_flux_have_the_derived_sign() {
        let mesh = two_cells();
        let predicted =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_some() {
                        3.0
                    } else {
                        0.0
                    }
                },
            );
        let coefficients =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_some() {
                        2.0
                    } else {
                        0.0
                    }
                },
            );
        let system = assemble_pressure_correction(&mesh, &predicted, &coefficients, 0).unwrap();

        assert_eq!(system.matrix().get(0, 0), Some(1.0));
        assert_eq!(system.matrix().get(0, 1), None);
        assert_eq!(system.matrix().get(1, 0), None);
        assert_eq!(system.matrix().get(1, 1), Some(2.0));
        // A p' = -div(Phi*); before reference, the RHS is [-3, +3].
        assert_eq!(system.rhs(), &[0.0, 3.0]);
    }

    #[test]
    fn pressure_correction_solve_and_direct_flux_update_eliminate_the_two_cell_imbalance() {
        let mesh = two_cells();
        let predicted =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_some() {
                        3.0
                    } else {
                        0.0
                    }
                },
            );
        let coefficients =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_some() {
                        2.0
                    } else {
                        0.0
                    }
                },
            );
        let system = assemble_pressure_correction(&mesh, &predicted, &coefficients, 0).unwrap();
        let mut correction = CellField::filled(&mesh, 0.0);
        let report = solve_pressure_correction(
            &system,
            &mut correction,
            crate::LinearSolverOptions {
                absolute_tolerance: 1.0e-14,
                relative_tolerance: 1.0e-14,
                max_iterations: 20,
            },
        )
        .unwrap();
        assert!(report.converged());
        assert_eq!(correction.values(), &[0.0, 1.5]);
        let corrected = correct_face_flux(&mesh, &predicted, &coefficients, &correction).unwrap();
        let imbalance = crate::integrated_divergence(&mesh, &corrected).unwrap();
        assert!(imbalance.values().iter().all(|value| value.abs() < 1.0e-12));
        assert!(system
            .true_residual(correction.values())
            .unwrap()
            .iter()
            .all(|value| value.abs() < 1.0e-12));
    }

    #[test]
    fn velocity_pressure_correction_has_the_derived_negative_gradient_sign() {
        let mesh = two_cells();
        let predictor = CellField::from_values(
            &mesh,
            vec![Vec3::new(4.0, -1.0, 2.0), Vec3::new(1.0, 3.0, -2.0)],
        )
        .unwrap();
        let r_au = CellField::from_values(&mesh, vec![0.5, 2.0]).unwrap();
        let gradient = CellField::from_values(
            &mesh,
            vec![Vec3::new(2.0, -4.0, 1.0), Vec3::new(-1.0, 0.5, 3.0)],
        )
        .unwrap();
        let corrected =
            correct_cell_velocity_from_gradient(&mesh, &predictor, &r_au, &gradient).unwrap();
        assert_eq!(corrected[0], Vec3::new(3.0, 1.0, 1.5));
        assert_eq!(corrected[1], Vec3::new(3.0, 2.0, -8.0));
    }

    #[test]
    fn continuity_rms_is_the_direct_integrated_flux_imbalance_norm() {
        let mesh = two_cells();
        let flux = FaceField::from_faces(
            &mesh,
            |_, face| {
                if face.neighbour.is_some() {
                    3.0
                } else {
                    0.0
                }
            },
        );
        // Integrated cell residuals are [+3, -3], whose RMS is 3.
        assert_eq!(continuity_rms(&mesh, &flux).unwrap(), 3.0);
    }

    #[test]
    fn initial_face_flux_interpolates_cell_velocity_but_preserves_wall_fluxes() {
        let mesh = two_cells();
        let velocity = CellField::filled(&mesh, Vec3::new(1.0, 0.0, 0.0));
        let wall_flux =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_none() {
                        4.0
                    } else {
                        0.0
                    }
                },
            );
        let flux = initial_face_flux(&mesh, &velocity, &wall_flux).unwrap();
        for (face, geometry) in mesh.faces().iter().enumerate() {
            if geometry.neighbour.is_some() {
                assert_eq!(flux[face], velocity[0].dot(geometry.area_vector));
            } else {
                assert_eq!(flux[face], 4.0);
            }
        }
    }

    #[test]
    fn simple_converges_a_zero_velocity_closed_cavity_without_creating_pressure_or_flux() {
        let mesh = two_cells();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let options =
            SimpleOptions::steady(Diffusivity::Constant(1.0), &velocity_boundary, &stencil);
        let report = solve_simple(&mesh, &mut state, options).unwrap();
        assert!(report.converged);
        assert!(report.final_continuity_rms < 1.0e-12);
        assert!(state
            .velocity
            .values()
            .iter()
            .all(|value| *value == Vec3::ZERO));
        assert!(state.pressure.values().iter().all(|value| *value == 0.0));
        assert!(state.face_flux.values().iter().all(|value| *value == 0.0));
    }

    #[test]
    fn simple_uses_the_corrected_face_flux_to_remove_a_nonzero_initial_two_cell_imbalance() {
        let mesh = two_cells();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let initial_flux =
            FaceField::from_faces(
                &mesh,
                |_, face| {
                    if face.neighbour.is_some() {
                        3.0
                    } else {
                        0.0
                    }
                },
            );
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            initial_flux,
        )
        .unwrap();
        let mut options =
            SimpleOptions::steady(Diffusivity::Constant(1.0), &velocity_boundary, &stencil);
        options.velocity_relaxation = 1.0;
        let report = solve_simple(&mesh, &mut state, options).unwrap();
        assert!(report.converged);
        assert_eq!(report.outer_iterations, 1);
        assert_eq!(report.initial_continuity_rms, 3.0);
        assert!(report.final_continuity_rms < 1.0e-12);
        assert!(state
            .face_flux
            .values()
            .iter()
            .all(|value| value.abs() < 1.0e-12));
    }

    #[test]
    fn simple_reports_max_iterations_when_an_immutable_boundary_flux_prevents_continuity() {
        let mesh = two_cells();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let leaking_wall = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_none())
            .unwrap();
        let initial_flux =
            FaceField::from_faces(
                &mesh,
                |face, _| {
                    if face == leaking_wall {
                        1.0
                    } else {
                        0.0
                    }
                },
            );
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            initial_flux,
        )
        .unwrap();
        let mut options =
            SimpleOptions::steady(Diffusivity::Constant(1.0), &velocity_boundary, &stencil);
        options.max_outer_iterations = 1;
        options.continuity_absolute_tolerance = 1.0e-12;
        let report = solve_simple(&mesh, &mut state, options).unwrap();
        assert!(!report.converged);
        assert_eq!(report.outer_iterations, 1);
        assert!(report.final_continuity_rms > 1.0e-12);
    }

    #[test]
    fn simple_rejects_an_invalid_pressure_relaxation() {
        let mesh = two_cells();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut options =
            SimpleOptions::steady(Diffusivity::Constant(1.0), &velocity_boundary, &stencil);
        options.pressure_relaxation = 0.0;
        assert!(matches!(
            solve_simple(&mesh, &mut state, options),
            Err(SimpleError::InvalidPressureRelaxation { value: 0.0 })
        ));
    }

    #[test]
    fn simple_rejects_same_sized_state_and_pressure_stencil_from_another_mesh() {
        let mesh = two_cells();
        let foreign = two_cells();
        assert!(matches!(
            SimpleState::new(
                &mesh,
                CellField::filled(&foreign, Vec3::ZERO),
                CellField::filled(&mesh, 0.0),
                FaceField::filled(&mesh, 0.0),
            ),
            Err(SimpleError::Field(FieldError::MeshMismatch { .. }))
        ));

        let foreign_pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &foreign,
            &[("wall", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap();
        let foreign_stencil =
            LeastSquaresGradientStencil::new(&foreign, &foreign_pressure_boundary).unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let options = SimpleOptions::steady(
            Diffusivity::Constant(1.0),
            &velocity_boundary,
            &foreign_stencil,
        );
        assert!(matches!(
            solve_simple(&mesh, &mut state, options),
            Err(SimpleError::Numerics(NumericsError::Field(
                FieldError::MeshMismatch { .. }
            )))
        ));
    }

    #[test]
    fn low_reynolds_closed_cavity_simple_converges_without_wall_leakage() {
        let mesh = cavity_grid(3, 3);
        let mut pressure_names = Vec::new();
        let mut velocity_names = Vec::new();
        for (face, geometry) in mesh.faces().iter().enumerate() {
            if geometry.neighbour.is_none() {
                let name = format!("face-{face}");
                pressure_names.push((name.clone(), ScalarBoundaryCondition::ZeroGradient));
                let velocity = if (geometry.center.y - 3.0).abs() < 1.0e-12 {
                    Vec3::new(1.0, 0.0, 0.0)
                } else {
                    Vec3::ZERO
                };
                velocity_names.push((name, VelocityBoundaryCondition::FixedVelocity(velocity)));
            }
        }
        let pressure_assignments: Vec<_> = pressure_names
            .iter()
            .map(|(name, condition)| (name.as_str(), *condition))
            .collect();
        let velocity_assignments: Vec<_> = velocity_names
            .iter()
            .map(|(name, condition)| (name.as_str(), *condition))
            .collect();
        let pressure_boundary =
            ResolvedScalarBoundaryConditions::strict(&mesh, &pressure_assignments).unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let velocity_boundary =
            ResolvedVelocityBoundaryConditions::strict(&mesh, &velocity_assignments).unwrap();
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut options =
            SimpleOptions::steady(Diffusivity::Constant(0.1), &velocity_boundary, &stencil);
        options.max_outer_iterations = 300;
        options.continuity_absolute_tolerance = 1.0e-9;
        options.momentum_solver.absolute_tolerance = 1.0e-12;
        options.pressure_solver.absolute_tolerance = 1.0e-12;
        let report = solve_simple(&mesh, &mut state, options).unwrap();
        assert!(report.converged, "{report:?}");
        assert!(report.final_continuity_rms < 1.0e-9);
        assert!(state
            .velocity
            .values()
            .iter()
            .any(|value| value.norm_squared() > 1.0e-12));
        assert!(state
            .velocity
            .values()
            .iter()
            .all(|value| { value.x.is_finite() && value.y.is_finite() && value.z.is_finite() }));
        assert!(state
            .pressure
            .values()
            .iter()
            .all(|value| value.is_finite()));
        assert!(mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_none())
            .all(|(face, _)| state.face_flux[face].abs() < 1.0e-12));
    }

    #[test]
    fn skewed_low_reynolds_closed_cavity_has_finite_converged_simple_coupling() {
        let mesh = skewed_cavity_grid(3, 3);
        let mut pressure_names = Vec::new();
        let mut velocity_names = Vec::new();
        for (face, geometry) in mesh.faces().iter().enumerate() {
            if geometry.neighbour.is_none() {
                let name = format!("face-{face}");
                pressure_names.push((name.clone(), ScalarBoundaryCondition::ZeroGradient));
                let velocity = if (geometry.center.y - 3.0).abs() < 1.0e-12 {
                    Vec3::new(1.0, 0.0, 0.0)
                } else {
                    Vec3::ZERO
                };
                velocity_names.push((name, VelocityBoundaryCondition::FixedVelocity(velocity)));
            }
        }
        let pressure_assignments: Vec<_> = pressure_names
            .iter()
            .map(|(name, condition)| (name.as_str(), *condition))
            .collect();
        let velocity_assignments: Vec<_> = velocity_names
            .iter()
            .map(|(name, condition)| (name.as_str(), *condition))
            .collect();
        let pressure_boundary =
            ResolvedScalarBoundaryConditions::strict(&mesh, &pressure_assignments).unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let velocity_boundary =
            ResolvedVelocityBoundaryConditions::strict(&mesh, &velocity_assignments).unwrap();
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            FaceField::filled(&mesh, 0.0),
        )
        .unwrap();
        let mut options =
            SimpleOptions::steady(Diffusivity::Constant(0.1), &velocity_boundary, &stencil);
        options.max_outer_iterations = 500;
        options.continuity_absolute_tolerance = 1.0e-8;
        options.momentum_solver.absolute_tolerance = 1.0e-12;
        options.pressure_solver.absolute_tolerance = 1.0e-12;
        let report = solve_simple(&mesh, &mut state, options).unwrap();
        assert!(report.converged, "{report:?}");
        assert!(report.final_continuity_rms < 1.0e-8);
        assert!(state
            .velocity
            .values()
            .iter()
            .all(|value| { value.x.is_finite() && value.y.is_finite() && value.z.is_finite() }));
        assert!(state
            .pressure
            .values()
            .iter()
            .all(|value| value.is_finite()));
        assert!(mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_none())
            .all(|(face, _)| state.face_flux[face].abs() < 1.0e-12));
    }

    #[test]
    fn three_dimensional_simple_regression_exercises_all_predictors_and_pressure_correction() {
        let mesh = hexahedral_grid(2, 2, 2);
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::ZeroGradient)],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let velocity_boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[("wall", VelocityBoundaryCondition::FixedVelocity(Vec3::ZERO))],
        )
        .unwrap();
        let driven_face = mesh
            .faces()
            .iter()
            .position(|geometry| geometry.neighbour.is_some())
            .unwrap();
        let initial_flux = FaceField::from_faces(&mesh, |face, geometry| {
            if geometry.neighbour.is_some() && face == driven_face {
                1.0
            } else {
                0.0
            }
        });
        let mut state = SimpleState::new(
            &mesh,
            CellField::filled(&mesh, Vec3::ZERO),
            CellField::filled(&mesh, 0.0),
            initial_flux,
        )
        .unwrap();
        let mut options =
            SimpleOptions::steady(Diffusivity::Constant(1.0), &velocity_boundary, &stencil);
        options.velocity_relaxation = 1.0;
        options.max_outer_iterations = 20;
        options.continuity_absolute_tolerance = 1.0e-10;
        let report = solve_simple(&mesh, &mut state, options).unwrap();
        assert!(report.converged, "{report:?}");
        assert!(report.final_continuity_rms < 1.0e-10);
        assert!(report.momentum_reports.is_some());
        assert!(report.pressure_report.is_some());
        assert!(state
            .velocity
            .values()
            .iter()
            .all(|value| { value.x.is_finite() && value.y.is_finite() && value.z.is_finite() }));
        assert!(state
            .pressure
            .values()
            .iter()
            .all(|value| value.is_finite()));
        assert!(mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_none())
            .all(|(face, _)| state.face_flux[face].abs() < 1.0e-12));
    }

    #[test]
    fn rhie_chow_preserves_a_constant_pressure_without_spurious_flux() {
        let mesh = two_cells();
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::FixedValue(7.0))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let predictor = CellField::filled(&mesh, Vec3::ZERO);
        let pressure = CellField::filled(&mesh, 7.0);
        let r_au = CellField::filled(&mesh, 1.0);
        let wall_flux = FaceField::filled(&mesh, 0.0);

        let flux =
            rhie_chow_predicted_flux(&mesh, &predictor, &pressure, &r_au, &stencil, &wall_flux)
                .unwrap();
        assert!(flux.values().iter().all(|value| value.abs() < 1.0e-12));
    }

    #[test]
    fn rhie_chow_reproduces_the_consistent_face_flux_for_a_linear_pressure_field() {
        let mesh = cavity_grid(3, 3);
        let mut names = Vec::new();
        for (face, geometry) in mesh.faces().iter().enumerate() {
            if geometry.neighbour.is_none() {
                let value = 2.0 * geometry.center.x - 3.0 * geometry.center.y + 5.0;
                names.push((
                    format!("face-{face}"),
                    ScalarBoundaryCondition::FixedValue(value),
                ));
            }
        }
        let assignments: Vec<_> = names
            .iter()
            .map(|(name, condition)| (name.as_str(), *condition))
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let pressure = CellField::from_cells(&mesh, |_, cell| {
            2.0 * cell.center.x - 3.0 * cell.center.y + 5.0
        });
        let predictor = CellField::filled(&mesh, Vec3::new(-2.0, 3.0, 0.0));
        let r_au = CellField::filled(&mesh, 1.0);
        let wall_flux = FaceField::filled(&mesh, 0.0);
        let flux =
            rhie_chow_predicted_flux(&mesh, &predictor, &pressure, &r_au, &stencil, &wall_flux)
                .unwrap();
        for (face, geometry) in mesh.faces().iter().enumerate() {
            if geometry.neighbour.is_some() {
                let expected = Vec3::new(-2.0, 3.0, 0.0).dot(geometry.area_vector);
                assert!((flux[face] - expected).abs() < 1.0e-12);
            }
        }
    }

    #[test]
    fn rhie_chow_direct_pressure_term_detects_a_checkerboard_mode() {
        let mesh = square_grid(3, 3);
        let pressure_boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("wall", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &pressure_boundary).unwrap();
        let predictor = CellField::filled(&mesh, Vec3::ZERO);
        let pressure = CellField::from_cells(&mesh, |cell, _| {
            let x = cell % 3;
            let y = cell / 3;
            if (x + y) % 2 == 0 {
                1.0
            } else {
                -1.0
            }
        });
        let r_au = CellField::filled(&mesh, 1.0);
        let wall_flux = FaceField::filled(&mesh, 0.0);
        let flux =
            rhie_chow_predicted_flux(&mesh, &predictor, &pressure, &r_au, &stencil, &wall_flux)
                .unwrap();

        // Naive interpolation of the zero predictor is zero at every face.
        // The direct pressure-difference term must still couple this mode.
        assert!(mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_some())
            .any(|(face, _)| flux[face].abs() > 1.0e-8));
    }
}
