//! Conservative reference operators for unstructured finite-volume meshes.

use crate::{CellField, FaceField, FieldError, MeshId, UnstructuredMesh, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScalarBoundaryValue {
    OwnerValue,
    FixedValue(f64),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NonOrthogonalCorrection {
    None,
    Explicit,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GradientScheme {
    Gauss,
    LeastSquares,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScalarBoundaryCondition {
    FixedValue(f64),
    ZeroGradient,
}
#[derive(Clone, Debug, PartialEq)]
pub struct ResolvedScalarBoundaryConditions {
    mesh_id: MeshId,
    conditions: Vec<Option<ScalarBoundaryCondition>>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiffusivityInterpolation {
    Linear,
    Harmonic,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DiffusionOptions {
    pub diffusivity_interpolation: DiffusivityInterpolation,
    pub non_orthogonal_correction: NonOrthogonalCorrection,
    pub gradient_scheme: GradientScheme,
}
impl Default for DiffusionOptions {
    fn default() -> Self {
        Self {
            diffusivity_interpolation: DiffusivityInterpolation::Linear,
            non_orthogonal_correction: NonOrthogonalCorrection::None,
            gradient_scheme: GradientScheme::Gauss,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub enum Diffusivity<'a> {
    Constant(f64),
    CellField(&'a CellField<f64>),
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NonOrthogonalDecomposition {
    pub orthogonal: Vec3,
    pub non_orthogonal: Vec3,
}
#[derive(Clone, Debug, PartialEq)]
pub enum NumericsError {
    Field(FieldError),
    DegenerateOwnerNeighbourDistance { face: usize },
    DegenerateOwnerFaceDistance { face: usize },
    InvalidCellMeasure { cell: usize },
    UnknownBoundaryPatch { patch: String },
    DuplicateBoundaryPatchCondition { patch: String },
    MissingBoundaryCondition { face: usize },
    BoundaryConditionOnInternalFace { face: usize },
    InvalidDiffusivity { cell: usize, value: f64 },
    InvalidFaceDiffusivity { face: usize },
    InvalidNonOrthogonalGeometry { face: usize },
    SingularLeastSquaresStencil { cell: usize },
    MissingLeastSquaresGradientStencil,
}

/// A mesh- and boundary-condition-bound cache for weighted least-squares gradients.
///
/// Construct this once per immutable mesh and resolved scalar boundary-condition set,
/// then reuse it for cell fields with those same fixed boundary values. Internal
/// cell-centre equations and `FixedValue` face-centre equations are cached with
/// inverse-distance-squared weights; `ZeroGradient` faces deliberately contribute
/// no equation.
#[derive(Clone, Debug)]
pub struct LeastSquaresGradientStencil {
    mesh_id: MeshId,
    equations: Vec<Vec<LeastSquaresEquation>>,
    inverse_normal_matrices: Vec<[f64; 9]>,
}

#[derive(Clone, Copy, Debug)]
struct LeastSquaresEquation {
    rhs_coefficient: Vec3,
    source: LeastSquaresSource,
}

#[derive(Clone, Copy, Debug)]
enum LeastSquaresSource {
    Neighbour(usize),
    FixedValue(f64),
    /// Homogeneous Neumann data constrains a directional derivative, not a
    /// fabricated face value. It therefore contributes zero to the RHS.
    ZeroNormalDerivative,
}

impl LeastSquaresGradientStencil {
    /// Builds and validates a reusable weighted least-squares gradient cache.
    pub fn new(
        mesh: &UnstructuredMesh,
        boundary: &ResolvedScalarBoundaryConditions,
    ) -> Result<Self, NumericsError> {
        boundary.ensure_mesh(mesh)?;
        let mut equations = Vec::with_capacity(mesh.cell_count());
        let mut inverse_normal_matrices = Vec::with_capacity(mesh.cell_count());

        for (cell_index, cell) in mesh.cells().iter().enumerate() {
            let mut normal = [0.0; 9];
            let mut cell_equations = Vec::with_capacity(cell.faces.len());
            let mut zero_gradient_normals = Vec::new();
            for &face_index in &cell.faces {
                let face = &mesh.faces()[face_index];
                let (offset, source) = if let Some(neighbour) = face.neighbour {
                    let neighbour = if face.owner == cell_index {
                        neighbour
                    } else {
                        face.owner
                    };
                    (
                        mesh.cells()[neighbour].center - cell.center,
                        LeastSquaresSource::Neighbour(neighbour),
                    )
                } else {
                    match boundary
                        .condition(face_index)
                        .ok_or(NumericsError::MissingBoundaryCondition { face: face_index })?
                    {
                        ScalarBoundaryCondition::FixedValue(value) => (
                            face.center - cell.center,
                            LeastSquaresSource::FixedValue(value),
                        ),
                        ScalarBoundaryCondition::ZeroGradient => {
                            zero_gradient_normals.push(face.normal);
                            continue;
                        }
                    }
                };
                let distance_squared = offset.norm_squared();
                if distance_squared <= f64::EPSILON {
                    return Err(if face.neighbour.is_some() {
                        NumericsError::DegenerateOwnerNeighbourDistance { face: face_index }
                    } else {
                        NumericsError::DegenerateOwnerFaceDistance { face: face_index }
                    });
                }
                let weight = 1.0 / distance_squared;
                let rhs_coefficient = offset * weight;
                normal[0] += rhs_coefficient.x * offset.x;
                normal[1] += rhs_coefficient.x * offset.y;
                normal[2] += rhs_coefficient.x * offset.z;
                normal[3] += rhs_coefficient.y * offset.x;
                normal[4] += rhs_coefficient.y * offset.y;
                normal[5] += rhs_coefficient.y * offset.z;
                normal[6] += rhs_coefficient.z * offset.x;
                normal[7] += rhs_coefficient.z * offset.y;
                normal[8] += rhs_coefficient.z * offset.z;
                cell_equations.push(LeastSquaresEquation {
                    rhs_coefficient,
                    source,
                });
            }
            let inverse = match inverse_normal_matrix(mesh, normal, cell_index) {
                Ok(inverse) => inverse,
                Err(NumericsError::SingularLeastSquaresStencil { .. })
                    if mesh.dimension() == crate::MeshDimension::ThreeD =>
                {
                    for normal_direction in zero_gradient_normals {
                        let length = normal_direction.norm();
                        if !(length.is_finite() && length > f64::EPSILON) {
                            continue;
                        }
                        let direction = normal_direction / length;
                        add_normal_outer_product(&mut normal, direction);
                        cell_equations.push(LeastSquaresEquation {
                            rhs_coefficient: Vec3::ZERO,
                            source: LeastSquaresSource::ZeroNormalDerivative,
                        });
                    }
                    inverse_normal_matrix(mesh, normal, cell_index)?
                }
                Err(error) => return Err(error),
            };
            equations.push(cell_equations);
            inverse_normal_matrices.push(inverse);
        }
        Ok(Self {
            mesh_id: mesh.id(),
            equations,
            inverse_normal_matrices,
        })
    }

    /// Rejects use of this cache with a different mesh identity.
    pub fn ensure_mesh(&self, mesh: &UnstructuredMesh) -> Result<(), NumericsError> {
        if self.mesh_id == mesh.id() {
            Ok(())
        } else {
            Err(NumericsError::Field(FieldError::MeshMismatch {
                expected: self.mesh_id,
                actual: mesh.id(),
            }))
        }
    }
}

fn add_normal_outer_product(matrix: &mut [f64; 9], direction: Vec3) {
    matrix[0] += direction.x * direction.x;
    matrix[1] += direction.x * direction.y;
    matrix[2] += direction.x * direction.z;
    matrix[3] += direction.y * direction.x;
    matrix[4] += direction.y * direction.y;
    matrix[5] += direction.y * direction.z;
    matrix[6] += direction.z * direction.x;
    matrix[7] += direction.z * direction.y;
    matrix[8] += direction.z * direction.z;
}

fn inverse_normal_matrix(
    mesh: &UnstructuredMesh,
    normal: [f64; 9],
    cell: usize,
) -> Result<[f64; 9], NumericsError> {
    let singular = || Err(NumericsError::SingularLeastSquaresStencil { cell });
    if !normal.iter().all(|value| value.is_finite()) {
        return singular();
    }
    match mesh.dimension() {
        crate::MeshDimension::TwoD => {
            let determinant = normal[0] * normal[4] - normal[1] * normal[3];
            let scale = normal[0].abs().max(normal[4].abs()).max(1.0);
            if determinant.abs() <= f64::EPSILON * scale * scale {
                return singular();
            }
            Ok([
                normal[4] / determinant,
                -normal[1] / determinant,
                0.0,
                -normal[3] / determinant,
                normal[0] / determinant,
                0.0,
                0.0,
                0.0,
                0.0,
            ])
        }
        crate::MeshDimension::ThreeD => {
            let determinant = normal[0] * (normal[4] * normal[8] - normal[5] * normal[7])
                - normal[1] * (normal[3] * normal[8] - normal[5] * normal[6])
                + normal[2] * (normal[3] * normal[7] - normal[4] * normal[6]);
            let scale = normal
                .iter()
                .fold(1.0_f64, |scale, value| scale.max(value.abs()));
            if determinant.abs() <= f64::EPSILON * scale * scale * scale {
                return singular();
            }
            Ok([
                (normal[4] * normal[8] - normal[5] * normal[7]) / determinant,
                (normal[2] * normal[7] - normal[1] * normal[8]) / determinant,
                (normal[1] * normal[5] - normal[2] * normal[4]) / determinant,
                (normal[5] * normal[6] - normal[3] * normal[8]) / determinant,
                (normal[0] * normal[8] - normal[2] * normal[6]) / determinant,
                (normal[2] * normal[3] - normal[0] * normal[5]) / determinant,
                (normal[3] * normal[7] - normal[4] * normal[6]) / determinant,
                (normal[1] * normal[6] - normal[0] * normal[7]) / determinant,
                (normal[0] * normal[4] - normal[1] * normal[3]) / determinant,
            ])
        }
    }
}

impl From<FieldError> for NumericsError {
    fn from(value: FieldError) -> Self {
        Self::Field(value)
    }
}

fn check_cell_measure(mesh: &UnstructuredMesh) -> Result<(), NumericsError> {
    for (index, cell) in mesh.cells().iter().enumerate() {
        if cell.volume <= 0.0 {
            return Err(NumericsError::InvalidCellMeasure { cell: index });
        }
    }
    Ok(())
}
fn weight(mesh: &UnstructuredMesh, face_index: usize) -> Result<f64, NumericsError> {
    let face = &mesh.faces()[face_index];
    let neighbour = face.neighbour.expect("internal face required");
    let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
    let d2 = d.norm_squared();
    if d2 <= f64::EPSILON {
        return Err(NumericsError::DegenerateOwnerNeighbourDistance { face: face_index });
    }
    Ok((face.center - mesh.cells()[face.owner].center).dot(d) / d2)
}

impl ResolvedScalarBoundaryConditions {
    pub fn strict(
        mesh: &UnstructuredMesh,
        assignments: &[(&str, ScalarBoundaryCondition)],
    ) -> Result<Self, NumericsError> {
        let mut conditions = vec![None; mesh.face_count()];
        let mut names = std::collections::HashSet::new();
        for (name, condition) in assignments {
            if !names.insert(*name) {
                return Err(NumericsError::DuplicateBoundaryPatchCondition {
                    patch: (*name).to_string(),
                });
            }
            let patch = mesh
                .boundary_patches()
                .iter()
                .find(|patch| patch.name == *name)
                .ok_or_else(|| NumericsError::UnknownBoundaryPatch {
                    patch: (*name).to_string(),
                })?;
            for &face in &patch.face_indices {
                if mesh.faces()[face].neighbour.is_some() {
                    return Err(NumericsError::BoundaryConditionOnInternalFace { face });
                }
                if conditions[face].replace(*condition).is_some() {
                    return Err(NumericsError::BoundaryConditionOnInternalFace { face });
                }
            }
        }
        for (face, geometry) in mesh.faces().iter().enumerate() {
            if geometry.neighbour.is_none() && conditions[face].is_none() {
                return Err(NumericsError::MissingBoundaryCondition { face });
            }
        }
        Ok(Self {
            mesh_id: mesh.id(),
            conditions,
        })
    }

    pub fn ensure_mesh(&self, mesh: &UnstructuredMesh) -> Result<(), NumericsError> {
        if self.mesh_id == mesh.id() {
            Ok(())
        } else {
            Err(NumericsError::Field(FieldError::MeshMismatch {
                expected: self.mesh_id,
                actual: mesh.id(),
            }))
        }
    }

    pub fn condition(&self, face: usize) -> Option<ScalarBoundaryCondition> {
        self.conditions[face]
    }
}

fn checked_diffusivity(value: f64, cell: usize) -> Result<f64, NumericsError> {
    if value.is_finite() && value >= 0.0 {
        Ok(value)
    } else {
        Err(NumericsError::InvalidDiffusivity { cell, value })
    }
}

fn cell_diffusivity(
    mesh: &UnstructuredMesh,
    diffusivity: Diffusivity<'_>,
    cell: usize,
) -> Result<f64, NumericsError> {
    match diffusivity {
        Diffusivity::Constant(value) => checked_diffusivity(value, cell),
        Diffusivity::CellField(values) => {
            values.ensure_mesh(mesh)?;
            checked_diffusivity(values[cell], cell)
        }
    }
}

pub fn interpolate_diffusivity_into(
    mesh: &UnstructuredMesh,
    diffusivity: Diffusivity<'_>,
    scheme: DiffusivityInterpolation,
    output: &mut FaceField<f64>,
) -> Result<(), NumericsError> {
    output.ensure_mesh(mesh)?;
    if let Diffusivity::CellField(values) = diffusivity {
        values.ensure_mesh(mesh)?;
    }
    for (index, face) in mesh.faces().iter().enumerate() {
        let owner = cell_diffusivity(mesh, diffusivity, face.owner)?;
        output[index] = if let Some(neighbour) = face.neighbour {
            let adjacent = cell_diffusivity(mesh, diffusivity, neighbour)?;
            let lambda = weight(mesh, index)?;
            match scheme {
                DiffusivityInterpolation::Linear => (1.0 - lambda) * owner + lambda * adjacent,
                DiffusivityInterpolation::Harmonic if owner == 0.0 || adjacent == 0.0 => 0.0,
                DiffusivityInterpolation::Harmonic => {
                    let denominator = (1.0 - lambda) / owner + lambda / adjacent;
                    if denominator.is_finite() && denominator > 0.0 {
                        1.0 / denominator
                    } else {
                        return Err(NumericsError::InvalidFaceDiffusivity { face: index });
                    }
                }
            }
        } else {
            owner
        };
        if !output[index].is_finite() || output[index] < 0.0 {
            return Err(NumericsError::InvalidFaceDiffusivity { face: index });
        }
    }
    Ok(())
}

pub fn interpolate_diffusivity(
    mesh: &UnstructuredMesh,
    diffusivity: Diffusivity<'_>,
    scheme: DiffusivityInterpolation,
) -> Result<FaceField<f64>, NumericsError> {
    let mut output = FaceField::filled(mesh, 0.0);
    interpolate_diffusivity_into(mesh, diffusivity, scheme, &mut output)?;
    Ok(output)
}

pub fn non_orthogonal_decomposition(
    mesh: &UnstructuredMesh,
    face_index: usize,
) -> Result<NonOrthogonalDecomposition, NumericsError> {
    let face = &mesh.faces()[face_index];
    let neighbour = face
        .neighbour
        .ok_or(NumericsError::BoundaryConditionOnInternalFace { face: face_index })?;
    let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
    let denominator = face.area_vector.dot(d);
    if !denominator.is_finite() || denominator <= f64::EPSILON {
        return Err(NumericsError::InvalidNonOrthogonalGeometry { face: face_index });
    }
    let orthogonal = d * (face.area_vector.norm_squared() / denominator);
    Ok(NonOrthogonalDecomposition {
        orthogonal,
        non_orthogonal: face.area_vector - orthogonal,
    })
}

/// Splits an internal face area vector using the projection consistent with the
/// two-point diffusion coefficient `(S·d)/(d·d)`.
pub fn projection_non_orthogonal_decomposition(
    mesh: &UnstructuredMesh,
    face_index: usize,
) -> Result<NonOrthogonalDecomposition, NumericsError> {
    let face = &mesh.faces()[face_index];
    let neighbour = face
        .neighbour
        .ok_or(NumericsError::BoundaryConditionOnInternalFace { face: face_index })?;
    let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
    let d2 = d.norm_squared();
    let projection = face.area_vector.dot(d);
    if !d2.is_finite()
        || d2 <= f64::EPSILON
        || !projection.is_finite()
        || projection <= f64::EPSILON
    {
        return Err(NumericsError::InvalidNonOrthogonalGeometry { face: face_index });
    }
    let orthogonal = d * (projection / d2);
    Ok(NonOrthogonalDecomposition {
        orthogonal,
        non_orthogonal: face.area_vector - orthogonal,
    })
}

fn corrected_internal_diffusion_flux(
    mesh: &UnstructuredMesh,
    face_index: usize,
    diffusivity: f64,
    values: &CellField<f64>,
    gradient: Vec3,
) -> Result<f64, NumericsError> {
    let face = &mesh.faces()[face_index];
    let neighbour = face
        .neighbour
        .ok_or(NumericsError::BoundaryConditionOnInternalFace { face: face_index })?;
    let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
    let d2 = d.norm_squared();
    if d2 <= f64::EPSILON {
        return Err(NumericsError::DegenerateOwnerNeighbourDistance { face: face_index });
    }
    let orthogonal =
        diffusivity * face.area_vector.dot(d) / d2 * (values[neighbour] - values[face.owner]);
    let non_orthogonal = diffusivity
        * gradient.dot(projection_non_orthogonal_decomposition(mesh, face_index)?.non_orthogonal);
    Ok(orthogonal + non_orthogonal)
}

pub fn interpolate_scalar_into(
    mesh: &UnstructuredMesh,
    cells: &CellField<f64>,
    boundary: ScalarBoundaryValue,
    output: &mut FaceField<f64>,
) -> Result<(), NumericsError> {
    cells.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    for (index, face) in mesh.faces().iter().enumerate() {
        output[index] = match face.neighbour {
            Some(neighbour) => {
                let lambda = weight(mesh, index)?;
                (1.0 - lambda) * cells[face.owner] + lambda * cells[neighbour]
            }
            None => match boundary {
                ScalarBoundaryValue::OwnerValue => cells[face.owner],
                ScalarBoundaryValue::FixedValue(value) => value,
            },
        };
    }
    Ok(())
}
pub fn interpolate_scalar(
    mesh: &UnstructuredMesh,
    cells: &CellField<f64>,
    boundary: ScalarBoundaryValue,
) -> Result<FaceField<f64>, NumericsError> {
    let mut output = FaceField::filled(mesh, 0.0);
    interpolate_scalar_into(mesh, cells, boundary, &mut output)?;
    Ok(output)
}
pub fn interpolate_vector_into(
    mesh: &UnstructuredMesh,
    cells: &CellField<Vec3>,
    boundary: ScalarBoundaryValue,
    output: &mut FaceField<Vec3>,
) -> Result<(), NumericsError> {
    cells.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    for (index, face) in mesh.faces().iter().enumerate() {
        output[index] = match face.neighbour {
            Some(neighbour) => {
                let lambda = weight(mesh, index)?;
                cells[face.owner] * (1.0 - lambda) + cells[neighbour] * lambda
            }
            None => match boundary {
                ScalarBoundaryValue::OwnerValue => cells[face.owner],
                ScalarBoundaryValue::FixedValue(value) => Vec3::new(value, value, value),
            },
        };
    }
    Ok(())
}
pub fn face_flux_into(
    mesh: &UnstructuredMesh,
    velocity: &FaceField<Vec3>,
    output: &mut FaceField<f64>,
) -> Result<(), NumericsError> {
    velocity.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    for (index, face) in mesh.faces().iter().enumerate() {
        output[index] = velocity[index].dot(face.area_vector);
    }
    Ok(())
}
pub fn face_flux(
    mesh: &UnstructuredMesh,
    velocity: &FaceField<Vec3>,
) -> Result<FaceField<f64>, NumericsError> {
    let mut output = FaceField::filled(mesh, 0.0);
    face_flux_into(mesh, velocity, &mut output)?;
    Ok(output)
}
pub fn integrated_divergence_into(
    mesh: &UnstructuredMesh,
    flux: &FaceField<f64>,
    output: &mut CellField<f64>,
) -> Result<(), NumericsError> {
    flux.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    output.fill(0.0);
    for (index, face) in mesh.faces().iter().enumerate() {
        output[face.owner] += flux[index];
        if let Some(neighbour) = face.neighbour {
            output[neighbour] -= flux[index];
        }
    }
    Ok(())
}
pub fn integrated_divergence(
    mesh: &UnstructuredMesh,
    flux: &FaceField<f64>,
) -> Result<CellField<f64>, NumericsError> {
    let mut output = CellField::filled(mesh, 0.0);
    integrated_divergence_into(mesh, flux, &mut output)?;
    Ok(output)
}
pub fn divergence_into(
    mesh: &UnstructuredMesh,
    flux: &FaceField<f64>,
    output: &mut CellField<f64>,
) -> Result<(), NumericsError> {
    check_cell_measure(mesh)?;
    integrated_divergence_into(mesh, flux, output)?;
    for (value, cell) in output.values_mut().iter_mut().zip(mesh.cells()) {
        *value /= cell.volume;
    }
    Ok(())
}
pub fn divergence(
    mesh: &UnstructuredMesh,
    flux: &FaceField<f64>,
) -> Result<CellField<f64>, NumericsError> {
    let mut output = CellField::filled(mesh, 0.0);
    divergence_into(mesh, flux, &mut output)?;
    Ok(output)
}
pub fn gauss_gradient_from_faces_into(
    mesh: &UnstructuredMesh,
    face_values: &FaceField<f64>,
    output: &mut CellField<Vec3>,
) -> Result<(), NumericsError> {
    check_cell_measure(mesh)?;
    face_values.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    output.fill(Vec3::ZERO);
    for (index, face) in mesh.faces().iter().enumerate() {
        output[face.owner] += face.area_vector * face_values[index];
        if let Some(neighbour) = face.neighbour {
            output[neighbour] += -(face.area_vector * face_values[index]);
        }
    }
    for (value, cell) in output.values_mut().iter_mut().zip(mesh.cells()) {
        *value = *value / cell.volume;
    }
    Ok(())
}
pub fn gauss_gradient_from_faces(
    mesh: &UnstructuredMesh,
    values: &FaceField<f64>,
) -> Result<CellField<Vec3>, NumericsError> {
    let mut output = CellField::filled(mesh, Vec3::ZERO);
    gauss_gradient_from_faces_into(mesh, values, &mut output)?;
    Ok(output)
}

/// Reconstructs cell-centred scalar gradients using a prevalidated weighted LSQ cache.
///
/// The stencil uses cell-centre differences for internal faces and face-centre
/// `FixedValue` boundary differences. `ZeroGradient` boundaries are omitted.
pub fn least_squares_gradient_into(
    mesh: &UnstructuredMesh,
    stencil: &LeastSquaresGradientStencil,
    values: &CellField<f64>,
    output: &mut CellField<Vec3>,
) -> Result<(), NumericsError> {
    stencil.ensure_mesh(mesh)?;
    values.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    for (cell, (equations, inverse)) in stencil
        .equations
        .iter()
        .zip(&stencil.inverse_normal_matrices)
        .enumerate()
    {
        let mut rhs = Vec3::ZERO;
        for equation in equations {
            let adjacent = match equation.source {
                LeastSquaresSource::Neighbour(neighbour) => values[neighbour],
                LeastSquaresSource::FixedValue(value) => value,
                LeastSquaresSource::ZeroNormalDerivative => values[cell],
            };
            rhs += equation.rhs_coefficient * (adjacent - values[cell]);
        }
        output[cell] = Vec3::new(
            inverse[0] * rhs.x + inverse[1] * rhs.y + inverse[2] * rhs.z,
            inverse[3] * rhs.x + inverse[4] * rhs.y + inverse[5] * rhs.z,
            inverse[6] * rhs.x + inverse[7] * rhs.y + inverse[8] * rhs.z,
        );
    }
    Ok(())
}

/// Allocates and reconstructs cell-centred scalar gradients with weighted LSQ.
pub fn least_squares_gradient(
    mesh: &UnstructuredMesh,
    stencil: &LeastSquaresGradientStencil,
    values: &CellField<f64>,
) -> Result<CellField<Vec3>, NumericsError> {
    let mut output = CellField::filled(mesh, Vec3::ZERO);
    least_squares_gradient_into(mesh, stencil, values, &mut output)?;
    Ok(output)
}

pub fn integrated_diffusion_into(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
    output: &mut CellField<f64>,
) -> Result<(), NumericsError> {
    integrated_diffusion_with_stencil_into(
        mesh,
        values,
        diffusivity,
        boundary,
        options,
        None,
        output,
    )
}

/// Computes conservative diffusion using an optional reusable LSQ gradient cache.
///
/// `Explicit + LeastSquares` requires a cache so its mesh-bound geometry can be
/// retained across repeated evaluations.
pub fn integrated_diffusion_with_stencil_into(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
    stencil: Option<&LeastSquaresGradientStencil>,
    output: &mut CellField<f64>,
) -> Result<(), NumericsError> {
    check_cell_measure(mesh)?;
    values.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    boundary.ensure_mesh(mesh)?;
    if let Some(stencil) = stencil {
        stencil.ensure_mesh(mesh)?;
    }
    let gamma = interpolate_diffusivity(mesh, diffusivity, options.diffusivity_interpolation)?;
    let mut face_values = FaceField::filled(mesh, 0.0);
    for (index, face) in mesh.faces().iter().enumerate() {
        face_values[index] = match face.neighbour {
            Some(neighbour) => {
                let lambda = weight(mesh, index)?;
                (1.0 - lambda) * values[face.owner] + lambda * values[neighbour]
            }
            None => match boundary
                .condition(index)
                .ok_or(NumericsError::MissingBoundaryCondition { face: index })?
            {
                ScalarBoundaryCondition::FixedValue(value) => value,
                ScalarBoundaryCondition::ZeroGradient => values[face.owner],
            },
        };
    }
    let gradients = if options.non_orthogonal_correction == NonOrthogonalCorrection::Explicit {
        match options.gradient_scheme {
            GradientScheme::Gauss => Some(gauss_gradient_from_faces(mesh, &face_values)?),
            GradientScheme::LeastSquares => Some(least_squares_gradient(
                mesh,
                stencil.ok_or(NumericsError::MissingLeastSquaresGradientStencil)?,
                values,
            )?),
        }
    } else {
        None
    };
    output.fill(0.0);
    for (index, face) in mesh.faces().iter().enumerate() {
        if let Some(neighbour) = face.neighbour {
            let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
            let d2 = d.norm_squared();
            if d2 <= f64::EPSILON {
                return Err(NumericsError::DegenerateOwnerNeighbourDistance { face: index });
            }
            let mut contribution = gamma[index] * face.area_vector.dot(d) / d2
                * (values[neighbour] - values[face.owner]);
            if let Some(gradients) = &gradients {
                let lambda = weight(mesh, index)?;
                let gradient =
                    gradients[face.owner] * (1.0 - lambda) + gradients[neighbour] * lambda;
                contribution =
                    corrected_internal_diffusion_flux(mesh, index, gamma[index], values, gradient)?;
            }
            output[face.owner] += contribution;
            output[neighbour] -= contribution;
        } else if let ScalarBoundaryCondition::FixedValue(value) = boundary
            .condition(index)
            .ok_or(NumericsError::MissingBoundaryCondition { face: index })?
        {
            let d = face.center - mesh.cells()[face.owner].center;
            let d2 = d.norm_squared();
            if d2 <= f64::EPSILON {
                return Err(NumericsError::DegenerateOwnerFaceDistance { face: index });
            }
            output[face.owner] +=
                gamma[index] * face.area_vector.dot(d) / d2 * (value - values[face.owner]);
        }
    }
    Ok(())
}
pub fn integrated_diffusion(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
) -> Result<CellField<f64>, NumericsError> {
    let mut output = CellField::filled(mesh, 0.0);
    integrated_diffusion_into(mesh, values, diffusivity, boundary, options, &mut output)?;
    Ok(output)
}
pub fn integrated_diffusion_with_stencil(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
    stencil: Option<&LeastSquaresGradientStencil>,
) -> Result<CellField<f64>, NumericsError> {
    let mut output = CellField::filled(mesh, 0.0);
    integrated_diffusion_with_stencil_into(
        mesh,
        values,
        diffusivity,
        boundary,
        options,
        stencil,
        &mut output,
    )?;
    Ok(output)
}
pub fn laplacian_into(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
    output: &mut CellField<f64>,
) -> Result<(), NumericsError> {
    integrated_diffusion_into(mesh, values, diffusivity, boundary, options, output)?;
    for (value, cell) in output.values_mut().iter_mut().zip(mesh.cells()) {
        *value /= cell.volume;
    }
    Ok(())
}
pub fn laplacian(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
) -> Result<CellField<f64>, NumericsError> {
    let mut output = CellField::filled(mesh, 0.0);
    laplacian_into(mesh, values, diffusivity, boundary, options, &mut output)?;
    Ok(output)
}
pub fn laplacian_with_stencil_into(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
    stencil: Option<&LeastSquaresGradientStencil>,
    output: &mut CellField<f64>,
) -> Result<(), NumericsError> {
    integrated_diffusion_with_stencil_into(
        mesh,
        values,
        diffusivity,
        boundary,
        options,
        stencil,
        output,
    )?;
    for (value, cell) in output.values_mut().iter_mut().zip(mesh.cells()) {
        *value /= cell.volume;
    }
    Ok(())
}
pub fn laplacian_with_stencil(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: Diffusivity<'_>,
    boundary: &ResolvedScalarBoundaryConditions,
    options: DiffusionOptions,
    stencil: Option<&LeastSquaresGradientStencil>,
) -> Result<CellField<f64>, NumericsError> {
    let mut output = CellField::filled(mesh, 0.0);
    laplacian_with_stencil_into(
        mesh,
        values,
        diffusivity,
        boundary,
        options,
        stencil,
        &mut output,
    )?;
    Ok(output)
}
pub fn orthogonal_laplacian_into(
    mesh: &UnstructuredMesh,
    values: &CellField<f64>,
    diffusivity: f64,
    _correction: NonOrthogonalCorrection,
    output: &mut CellField<f64>,
) -> Result<(), NumericsError> {
    check_cell_measure(mesh)?;
    values.ensure_mesh(mesh)?;
    output.ensure_mesh(mesh)?;
    output.fill(0.0);
    for (index, face) in mesh.faces().iter().enumerate() {
        if let Some(neighbour) = face.neighbour {
            let d = (mesh.cells()[neighbour].center - mesh.cells()[face.owner].center).norm();
            if d <= f64::EPSILON {
                return Err(NumericsError::DegenerateOwnerNeighbourDistance { face: index });
            }
            let contribution =
                diffusivity * face.area / d * (values[neighbour] - values[face.owner]);
            output[face.owner] += contribution;
            output[neighbour] -= contribution;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{parse_gmsh, BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point};
    const TOL: f64 = 1.0e-10;

    fn assert_close(actual: f64, expected: f64) {
        assert!((actual - expected).abs() <= TOL * expected.abs().max(1.0));
    }

    fn assert_vec_close(actual: Vec3, expected: Vec3) {
        assert_close(actual.x, expected.x);
        assert_close(actual.y, expected.y);
        assert_close(actual.z, expected.z);
    }

    fn mesh() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(1., 1., 0.),
                Point::new(0., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 2]),
                CellDefinition::polygon(vec![0, 2, 3]),
            ],
        )
        .unwrap()
    }

    fn quarter_mesh() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(-1., -1., 0.),
                Point::new(1., -1., 0.),
                Point::new(1., 1., 0.),
                Point::new(-1., 1., 0.),
                Point::new(7., -1., 0.),
                Point::new(7., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 2, 3]),
                CellDefinition::polygon(vec![1, 4, 5, 2]),
            ],
        )
        .unwrap()
    }

    fn tetra() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(0., 0., 1.),
            ],
            vec![CellDefinition::tetrahedron([0, 1, 2, 3])],
        )
        .unwrap()
    }

    fn cube() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(1., 1., 0.),
                Point::new(0., 1., 0.),
                Point::new(0., 0., 1.),
                Point::new(1., 0., 1.),
                Point::new(1., 1., 1.),
                Point::new(0., 1., 1.),
            ],
            vec![CellDefinition::Hexahedron([0, 1, 2, 3, 4, 5, 6, 7])],
        )
        .unwrap()
    }

    fn three_cells() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(2., 0., 0.),
                Point::new(3., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(1., 1., 0.),
                Point::new(2., 1., 0.),
                Point::new(3., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 5, 4]),
                CellDefinition::polygon(vec![1, 2, 6, 5]),
                CellDefinition::polygon(vec![2, 3, 7, 6]),
            ],
        )
        .unwrap()
    }

    fn surface_sum(mesh: &UnstructuredMesh, values: &FaceField<f64>, cell: usize) -> Vec3 {
        mesh.cells()[cell]
            .faces
            .iter()
            .fold(Vec3::ZERO, |sum, &index| {
                let face = &mesh.faces()[index];
                sum + if face.owner == cell {
                    face.area_vector * values[index]
                } else {
                    -(face.area_vector * values[index])
                }
            })
    }

    #[test]
    fn interpolation_uses_geometric_weight_and_fluxes_conserve() {
        let mesh = mesh();
        let scalar = CellField::from_values(&mesh, vec![2., 4.]).unwrap();
        let faces = interpolate_scalar(&mesh, &scalar, ScalarBoundaryValue::OwnerValue).unwrap();
        let shared = mesh
            .faces()
            .iter()
            .position(|f| f.neighbour.is_some())
            .unwrap();
        assert!((faces[shared] - 3.).abs() < 1e-12);
        let velocity = FaceField::filled(&mesh, Vec3::new(3., -2., 0.));
        let flux = face_flux(&mesh, &velocity).unwrap();
        let div = integrated_divergence(&mesh, &flux).unwrap();
        assert!(div.values().iter().all(|v| v.abs() < 1e-12));
    }
    #[test]
    fn gauss_gradient_and_internal_fluxes_are_conservative() {
        let mesh = mesh();
        let values = CellField::from_cells(&mesh, |_, c| 2. * c.center.x - 3. * c.center.y + 4.);
        let faces =
            interpolate_scalar(&mesh, &values, ScalarBoundaryValue::FixedValue(0.)).unwrap();
        let gradient = gauss_gradient_from_faces(&mesh, &faces).unwrap();
        assert_eq!(gradient.len(), mesh.cell_count());
        let mut flux = FaceField::filled(&mesh, 0.);
        for (i, f) in mesh.faces().iter().enumerate() {
            if f.neighbour.is_some() {
                flux[i] = i as f64 + 1.;
            }
        }
        let residual = integrated_divergence(&mesh, &flux).unwrap();
        assert!(residual.values().iter().sum::<f64>().abs() < 1e-12);
    }

    #[test]
    fn scalar_and_vector_interpolation_use_actual_quarter_weight() {
        let mesh = quarter_mesh();
        let shared = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_some())
            .unwrap();
        let scalar = CellField::from_values(&mesh, vec![0., 8.]).unwrap();
        assert_close(
            interpolate_scalar(&mesh, &scalar, ScalarBoundaryValue::OwnerValue).unwrap()[shared],
            2.,
        );
        let vector =
            CellField::from_values(&mesh, vec![Vec3::ZERO, Vec3::new(4., 8., 12.)]).unwrap();
        let mut output = FaceField::filled(&mesh, Vec3::ZERO);
        interpolate_vector_into(&mesh, &vector, ScalarBoundaryValue::OwnerValue, &mut output)
            .unwrap();
        assert_vec_close(output[shared], Vec3::new(1., 2., 3.));
    }

    #[test]
    fn exact_face_values_recover_constant_and_linear_gauss_gradients_in_2d_and_3d() {
        for mesh in [mesh(), tetra(), cube()] {
            let constant = FaceField::filled(&mesh, 7.);
            let gradient = gauss_gradient_from_faces(&mesh, &constant).unwrap();
            assert!(gradient.values().iter().all(|value| value.norm() < TOL));
            let face_values = FaceField::from_faces(&mesh, |_, face| {
                2. * face.center.x - 3. * face.center.y + 4. * face.center.z + 5.
            });
            let gradient = gauss_gradient_from_faces(&mesh, &face_values).unwrap();
            let expected = match mesh.dimension() {
                MeshDimension::TwoD => Vec3::new(2., -3., 0.),
                MeshDimension::ThreeD => Vec3::new(2., -3., 4.),
            };
            for value in gradient.values() {
                assert_vec_close(*value, expected);
            }
        }
    }

    #[test]
    fn constant_velocity_chain_and_api_variants_are_conservative() {
        for mesh in [mesh(), tetra(), cube()] {
            let cells = CellField::filled(&mesh, Vec3::new(1.2, -0.7, 0.4));
            let mut face_velocity = FaceField::filled(&mesh, Vec3::new(99., 99., 99.));
            interpolate_vector_into(
                &mesh,
                &cells,
                ScalarBoundaryValue::OwnerValue,
                &mut face_velocity,
            )
            .unwrap();
            let flux = face_flux(&mesh, &face_velocity).unwrap();
            let mut flux_into = FaceField::filled(&mesh, 99.);
            face_flux_into(&mesh, &face_velocity, &mut flux_into).unwrap();
            assert_eq!(flux.values(), flux_into.values());
            let integrated = integrated_divergence(&mesh, &flux).unwrap();
            let normalized = divergence(&mesh, &flux).unwrap();
            for (index, value) in integrated.values().iter().enumerate() {
                assert_close(*value, 0.);
                assert_close(normalized[index], value / mesh.cells()[index].volume);
            }
        }
    }

    #[test]
    fn aligned_interpolation_and_gauss_surface_identity_are_exact() {
        let quarter = quarter_mesh();
        let internal = quarter
            .faces()
            .iter()
            .position(|face| face.neighbour.is_some())
            .unwrap();
        let cells = CellField::from_cells(&quarter, |_, cell| {
            2. * cell.center.x + 3. * cell.center.y - cell.center.z + 5.
        });
        assert_close(
            interpolate_scalar(&quarter, &cells, ScalarBoundaryValue::OwnerValue).unwrap()
                [internal],
            7.,
        );
        for mesh in [mesh(), tetra()] {
            let values = FaceField::from_faces(&mesh, |_, face| {
                2. * face.center.x - 3. * face.center.y + 4. * face.center.z + 5.
            });
            let expected = match mesh.dimension() {
                MeshDimension::TwoD => Vec3::new(2., -3., 0.),
                MeshDimension::ThreeD => Vec3::new(2., -3., 4.),
            };
            assert_vec_close(
                surface_sum(&mesh, &values, 0),
                expected * mesh.cells()[0].volume,
            );
        }
    }

    #[test]
    fn parity_reuse_and_multiface_conservation_hold() {
        let mesh = three_cells();
        let mut flux = FaceField::filled(&mesh, 0.);
        let internal: Vec<_> = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_some())
            .collect();
        flux[internal[0].0] = 1.75;
        flux[internal[1].0] = -3.2;
        let allocated = integrated_divergence(&mesh, &flux).unwrap();
        let mut reused = CellField::filled(&mesh, 99.);
        integrated_divergence_into(&mesh, &flux, &mut reused).unwrap();
        assert_eq!(allocated.values(), reused.values());
        assert_close(allocated.values().iter().sum(), 0.);
        let normalized = divergence(&mesh, &flux).unwrap();
        for (index, value) in allocated.values().iter().enumerate() {
            assert_close(normalized[index], value / mesh.cells()[index].volume);
        }
        for (_, face) in internal {
            assert_close(1.75 + -1.75, 0.);
            assert!(face.neighbour.is_some());
        }
    }

    #[test]
    fn skewed_interpolation_and_mesh_mismatch_are_checked() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(-1., -1., 0.),
                Point::new(1., -1., 0.),
                Point::new(2., 1., 0.),
                Point::new(-1., 1., 0.),
                Point::new(7., -1., 0.),
                Point::new(7., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 2, 3]),
                CellDefinition::polygon(vec![1, 4, 5, 2]),
            ],
        )
        .unwrap();
        let shared = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_some())
            .unwrap();
        let face = &mesh.faces()[shared];
        let d = mesh.cells()[1].center - mesh.cells()[0].center;
        let r = face.center - mesh.cells()[0].center;
        assert!((r - d * (r.dot(d) / d.norm_squared())).norm() > TOL);
        let lambda = r.dot(d) / d.norm_squared();
        let values = CellField::from_values(&mesh, vec![2., 10.]).unwrap();
        assert_close(
            interpolate_scalar(&mesh, &values, ScalarBoundaryValue::OwnerValue).unwrap()[shared],
            (1. - lambda) * 2. + lambda * 10.,
        );
        let velocity = FaceField::from_faces(&mesh, |index, face| {
            if index == shared {
                face.area_vector * 2.
            } else {
                Vec3::ZERO
            }
        });
        assert_close(
            face_flux(&mesh, &velocity).unwrap()[shared],
            2. * face.area * face.area,
        );
        let foreign = quarter_mesh();
        assert!(matches!(
            interpolate_scalar(&foreign, &values, ScalarBoundaryValue::OwnerValue),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        assert!(matches!(
            face_flux(&foreign, &velocity),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
    }

    #[test]
    fn outputs_are_reusable_and_finite_for_two_distinct_inputs() {
        let mesh = mesh();
        let a = FaceField::from_faces(&mesh, |index, _| index as f64 + 1.);
        let b = FaceField::from_faces(&mesh, |index, _| -(index as f64 + 2.));
        let mut integrated = CellField::filled(&mesh, 99.);
        integrated_divergence_into(&mesh, &a, &mut integrated).unwrap();
        let first = integrated.values().to_vec();
        integrated_divergence_into(&mesh, &b, &mut integrated).unwrap();
        assert_ne!(integrated.values(), first.as_slice());
        let mut normalized = CellField::filled(&mesh, 99.);
        divergence_into(&mesh, &a, &mut normalized).unwrap();
        divergence_into(&mesh, &b, &mut normalized).unwrap();
        let mut gradient = CellField::filled(&mesh, Vec3::new(99., 99., 99.));
        gauss_gradient_from_faces_into(&mesh, &a, &mut gradient).unwrap();
        gauss_gradient_from_faces_into(&mesh, &b, &mut gradient).unwrap();
        let velocity_a = FaceField::filled(&mesh, Vec3::new(1., 2., 0.));
        let velocity_b = FaceField::filled(&mesh, Vec3::new(-2., 1., 0.));
        let mut flux = FaceField::filled(&mesh, 99.);
        face_flux_into(&mesh, &velocity_a, &mut flux).unwrap();
        let first_flux = flux.values().to_vec();
        face_flux_into(&mesh, &velocity_b, &mut flux).unwrap();
        assert_ne!(flux.values(), first_flux.as_slice());
        assert!(flux
            .values()
            .iter()
            .chain(integrated.values())
            .chain(normalized.values())
            .all(|value| value.is_finite()));
        assert!(gradient
            .values()
            .iter()
            .all(|value| value.x.is_finite() && value.y.is_finite() && value.z.is_finite()));
    }

    #[test]
    fn remaining_api_parity_mismatch_and_3d_finiteness_are_direct() {
        let mesh = quarter_mesh();
        let scalar = CellField::from_values(&mesh, vec![2., 10.]).unwrap();
        let allocated =
            interpolate_scalar(&mesh, &scalar, ScalarBoundaryValue::OwnerValue).unwrap();
        let mut into = FaceField::filled(&mesh, 99.);
        interpolate_scalar_into(&mesh, &scalar, ScalarBoundaryValue::OwnerValue, &mut into)
            .unwrap();
        assert_eq!(allocated.values(), into.values());
        let flux = FaceField::from_faces(&mesh, |index, _| index as f64 + 1.);
        let allocated = divergence(&mesh, &flux).unwrap();
        let mut into = CellField::filled(&mesh, 99.);
        divergence_into(&mesh, &flux, &mut into).unwrap();
        assert_eq!(allocated.values(), into.values());
        let values = FaceField::from_faces(&mesh, |_, face| {
            2. * face.center.x - 3. * face.center.y + 5.
        });
        let allocated = gauss_gradient_from_faces(&mesh, &values).unwrap();
        let mut into = CellField::filled(&mesh, Vec3::ZERO);
        gauss_gradient_from_faces_into(&mesh, &values, &mut into).unwrap();
        assert_eq!(allocated.values(), into.values());
        let foreign = quarter_mesh();
        assert!(matches!(
            divergence(&foreign, &flux),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        assert!(matches!(
            gauss_gradient_from_faces(&foreign, &values),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        let mesh = tetra();
        let scalar = CellField::filled(&mesh, 3.);
        let vector = CellField::filled(&mesh, Vec3::new(1., 2., 3.));
        let scalar_faces =
            interpolate_scalar(&mesh, &scalar, ScalarBoundaryValue::OwnerValue).unwrap();
        let mut vector_faces = FaceField::filled(&mesh, Vec3::ZERO);
        interpolate_vector_into(
            &mesh,
            &vector,
            ScalarBoundaryValue::OwnerValue,
            &mut vector_faces,
        )
        .unwrap();
        let flux = face_flux(&mesh, &vector_faces).unwrap();
        let integrated = integrated_divergence(&mesh, &flux).unwrap();
        let normalized = divergence(&mesh, &flux).unwrap();
        let gradient = gauss_gradient_from_faces(&mesh, &scalar_faces).unwrap();
        assert!(scalar_faces
            .values()
            .iter()
            .chain(flux.values())
            .chain(integrated.values())
            .chain(normalized.values())
            .all(|x| x.is_finite()));
        assert!(vector_faces
            .values()
            .iter()
            .chain(gradient.values())
            .all(|v| v.x.is_finite() && v.y.is_finite() && v.z.is_finite()));
    }

    #[test]
    fn imported_two_tetra_constant_velocity_chain_is_conservative() {
        let input = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n2 9 \"walls\"\n$EndPhysicalNames\n$Entities\n0 0 1 1\n1 0 0 -1 1 1 1 1 9 4 1 2 3 4\n1 0 0 -1 1 1 1 0 1 1\n$EndEntities\n$Nodes\n1 5 1 5\n3 1 0 5\n1 2 3 4 5\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n0 0 -1\n$EndNodes\n$Elements\n2 8 1 8\n2 1 2 6\n1 1 2 4\n2 2 3 4\n3 3 1 4\n4 1 3 5\n5 3 2 5\n6 2 1 5\n3 1 4 2\n7 1 2 3 4\n8 1 3 2 5\n$EndElements\n";
        let mesh = parse_gmsh(input).unwrap();
        assert_eq!(mesh.cell_count(), 2);
        let internal: Vec<_> = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_some())
            .collect();
        assert_eq!(internal.len(), 1);
        let (shared, face) = internal[0];
        let neighbour = face.neighbour.unwrap();
        assert_ne!(face.owner, neighbour);
        assert!(
            (mesh.cells()[neighbour].center - mesh.cells()[face.owner].center)
                .dot(face.area_vector)
                > 0.0
        );
        let velocity = CellField::filled(&mesh, Vec3::new(1.2, -0.7, 0.4));
        let mut face_velocity = FaceField::filled(&mesh, Vec3::ZERO);
        interpolate_vector_into(
            &mesh,
            &velocity,
            ScalarBoundaryValue::OwnerValue,
            &mut face_velocity,
        )
        .unwrap();
        for value in face_velocity.values() {
            assert_vec_close(*value, Vec3::new(1.2, -0.7, 0.4));
        }
        let flux = face_flux(&mesh, &face_velocity).unwrap();
        assert!(flux[shared].is_finite());
        let integrated = integrated_divergence(&mesh, &flux).unwrap();
        let normalized = divergence(&mesh, &flux).unwrap();
        for cell in 0..2 {
            assert_close(integrated[cell], 0.);
            assert_close(normalized[cell], 0.);
            assert!(integrated[cell].is_finite() && normalized[cell].is_finite());
        }
        assert_close(flux[shared] + -flux[shared], 0.);
    }

    #[test]
    fn each_internal_face_contributes_equal_and_opposite_residuals() {
        let mesh = three_cells();
        let internal: Vec<_> = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_some())
            .collect();
        assert!(internal.len() >= 2);
        for ((index, face), value) in internal.into_iter().zip([2.75, -1.4]) {
            let mut flux = FaceField::filled(&mesh, 0.0);
            flux[index] = value;
            let residual = integrated_divergence(&mesh, &flux).unwrap();
            let neighbour = face.neighbour.unwrap();
            assert_close(residual[face.owner], value);
            assert_close(residual[neighbour], -value);
            assert_close(residual[face.owner] + residual[neighbour], 0.);
            for (cell, result) in residual.values().iter().enumerate() {
                if cell != face.owner && cell != neighbour {
                    assert_close(*result, 0.);
                }
            }
            assert_close(residual.values().iter().sum(), 0.);
        }
    }

    fn line_mesh_with_patches() -> UnstructuredMesh {
        let mesh = three_cells();
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut walls = Vec::new();
        for (index, face) in mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_none())
        {
            if face.center.x == 0.0 {
                left.push(index);
            } else if face.center.x == 3.0 {
                right.push(index);
            } else {
                walls.push(index);
            }
        }
        mesh.with_boundary_patches(vec![
            BoundaryPatch {
                name: "left".into(),
                face_indices: left,
                boundary_type: BoundaryType::Wall,
            },
            BoundaryPatch {
                name: "right".into(),
                face_indices: right,
                boundary_type: BoundaryType::Wall,
            },
            BoundaryPatch {
                name: "walls".into(),
                face_indices: walls,
                boundary_type: BoundaryType::Wall,
            },
        ])
        .unwrap()
    }

    #[test]
    fn patch_resolution_and_orthogonal_diffusion_are_verified() {
        let mesh = line_mesh_with_patches();
        let boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[
                ("left", ScalarBoundaryCondition::FixedValue(0.0)),
                ("right", ScalarBoundaryCondition::FixedValue(9.0)),
                ("walls", ScalarBoundaryCondition::ZeroGradient),
            ],
        )
        .unwrap();
        assert!(matches!(
            ResolvedScalarBoundaryConditions::strict(
                &mesh,
                &[("missing", ScalarBoundaryCondition::ZeroGradient)]
            ),
            Err(NumericsError::UnknownBoundaryPatch { .. })
        ));
        let values = CellField::from_cells(&mesh, |_, cell| cell.center.x * cell.center.x);
        let integrated = integrated_diffusion(
            &mesh,
            &values,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions::default(),
        )
        .unwrap();
        let lap = laplacian(
            &mesh,
            &values,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions::default(),
        )
        .unwrap();
        assert_close(lap[1], 2.0);
        assert_close(integrated[1], 2.0 * mesh.cells()[1].volume);
    }

    #[test]
    fn variable_diffusivity_decomposition_and_reuse_are_verified() {
        let mesh = line_mesh_with_patches();
        let boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[
                ("left", ScalarBoundaryCondition::ZeroGradient),
                ("right", ScalarBoundaryCondition::ZeroGradient),
                ("walls", ScalarBoundaryCondition::ZeroGradient),
            ],
        )
        .unwrap();
        let gamma = CellField::from_values(&mesh, vec![2.0, 6.0, 3.0]).unwrap();
        let first = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_some())
            .unwrap();
        let linear = interpolate_diffusivity(
            &mesh,
            Diffusivity::CellField(&gamma),
            DiffusivityInterpolation::Linear,
        )
        .unwrap();
        let harmonic = interpolate_diffusivity(
            &mesh,
            Diffusivity::CellField(&gamma),
            DiffusivityInterpolation::Harmonic,
        )
        .unwrap();
        assert_close(linear[first], 4.0);
        assert_close(harmonic[first], 3.0);
        let decomposition = non_orthogonal_decomposition(&mesh, first).unwrap();
        assert_vec_close(
            decomposition.orthogonal + decomposition.non_orthogonal,
            mesh.faces()[first].area_vector,
        );
        assert_close(decomposition.non_orthogonal.norm(), 0.0);
        let values = CellField::from_values(&mesh, vec![1.5, -0.75, 2.0]).unwrap();
        let allocated = integrated_diffusion(
            &mesh,
            &values,
            Diffusivity::CellField(&gamma),
            &boundary,
            DiffusionOptions::default(),
        )
        .unwrap();
        let mut reused = CellField::filled(&mesh, 99.0);
        integrated_diffusion_into(
            &mesh,
            &values,
            Diffusivity::CellField(&gamma),
            &boundary,
            DiffusionOptions::default(),
            &mut reused,
        )
        .unwrap();
        assert_eq!(allocated.values(), reused.values());
        assert_close(allocated.values().iter().sum(), 0.0);
        assert!(matches!(
            interpolate_diffusivity(
                &mesh,
                Diffusivity::Constant(-1.0),
                DiffusivityInterpolation::Linear
            ),
            Err(NumericsError::InvalidDiffusivity { .. })
        ));
    }

    fn individually_patched(mesh: UnstructuredMesh) -> UnstructuredMesh {
        let patches = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_none())
            .map(|(face, _)| BoundaryPatch {
                name: format!("boundary-{face}"),
                face_indices: vec![face],
                boundary_type: BoundaryType::Wall,
            })
            .collect();
        mesh.with_boundary_patches(patches).unwrap()
    }

    fn analytic_fixed_boundaries(
        mesh: &UnstructuredMesh,
        function: impl Fn(Vec3) -> f64,
    ) -> ResolvedScalarBoundaryConditions {
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(function(
                        mesh.faces()[patch.face_indices[0]].center,
                    )),
                )
            })
            .collect();
        ResolvedScalarBoundaryConditions::strict(mesh, &assignments).unwrap()
    }

    #[test]
    fn least_squares_reconstruction_is_linear_exact_reusable_and_mesh_safe_in_2d_and_3d() {
        for candidate in [individually_patched(mesh()), cube_block_3d()] {
            let gradient = match candidate.dimension() {
                MeshDimension::TwoD => Vec3::new(2.0, -3.0, 0.0),
                MeshDimension::ThreeD => Vec3::new(2.0, -3.0, 4.0),
            };
            let values =
                CellField::from_cells(&candidate, |_, cell| gradient.dot(cell.center) + 5.0);
            let boundary = analytic_fixed_boundaries(&candidate, |point| gradient.dot(point) + 5.0);
            let stencil = LeastSquaresGradientStencil::new(&candidate, &boundary).unwrap();
            let allocated = least_squares_gradient(&candidate, &stencil, &values).unwrap();
            assert!(allocated
                .values()
                .iter()
                .all(|actual| (*actual - gradient).norm() < TOL));

            let mut reused = CellField::filled(&candidate, Vec3::new(99.0, 99.0, 99.0));
            least_squares_gradient_into(&candidate, &stencil, &values, &mut reused).unwrap();
            assert_eq!(allocated.values(), reused.values());

            let foreign = match candidate.dimension() {
                MeshDimension::TwoD => individually_patched(mesh()),
                MeshDimension::ThreeD => cube_block_3d(),
            };
            let foreign_values = CellField::filled(&foreign, 0.0);
            assert!(matches!(
                least_squares_gradient(&candidate, &stencil, &foreign_values),
                Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
            ));
        }
    }

    #[test]
    fn least_squares_omits_zero_gradient_boundaries_and_rejects_singular_stencils() {
        let mesh = line_mesh_with_patches();
        let zero_gradient = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[
                ("left", ScalarBoundaryCondition::ZeroGradient),
                ("right", ScalarBoundaryCondition::ZeroGradient),
                ("walls", ScalarBoundaryCondition::ZeroGradient),
            ],
        )
        .unwrap();
        assert!(matches!(
            LeastSquaresGradientStencil::new(&mesh, &zero_gradient),
            Err(NumericsError::SingularLeastSquaresStencil { .. })
        ));

        let mesh = individually_patched(three_cells());
        let fixed = analytic_fixed_boundaries(&mesh, |point| 2.0 * point.x - 3.0 * point.y + 1.0);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &fixed).unwrap();
        let values = CellField::from_cells(&mesh, |_, cell| {
            2.0 * cell.center.x - 3.0 * cell.center.y + 1.0
        });
        let gradients = least_squares_gradient(&mesh, &stencil, &values).unwrap();
        assert!(gradients
            .values()
            .iter()
            .all(|gradient| (*gradient - Vec3::new(2.0, -3.0, 0.0)).norm() < TOL));
    }

    fn skewed_two_cell_mesh() -> UnstructuredMesh {
        individually_patched(
            UnstructuredMesh::from_cells(
                MeshDimension::TwoD,
                vec![
                    Point::new(0., 0., 0.),
                    Point::new(1., 0., 0.),
                    Point::new(1., 1., 0.),
                    Point::new(0., 1., 0.),
                    Point::new(2., 0.5, 0.),
                    Point::new(2., 1.5, 0.),
                ],
                vec![
                    CellDefinition::polygon(vec![0, 1, 2, 3]),
                    CellDefinition::polygon(vec![1, 4, 5, 2]),
                ],
            )
            .unwrap(),
        )
    }

    fn skewed_multi_cell_mesh_2d() -> UnstructuredMesh {
        let point = |x: usize, y: usize| {
            Point::new(
                x as f64 + 0.14 * (y * y) as f64,
                y as f64 + 0.06 * (x * x) as f64,
                0.0,
            )
        };
        let mut points = Vec::new();
        for y in 0..4 {
            for x in 0..4 {
                points.push(point(x, y));
            }
        }
        let index = |x: usize, y: usize| x + 4 * y;
        let mut cells = Vec::new();
        for y in 0..3 {
            for x in 0..3 {
                cells.push(CellDefinition::polygon(vec![
                    index(x, y),
                    index(x + 1, y),
                    index(x + 1, y + 1),
                    index(x, y + 1),
                ]));
            }
        }
        individually_patched(
            UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap(),
        )
    }

    fn skewed_tetrahedron_mesh_3d() -> UnstructuredMesh {
        individually_patched(
            UnstructuredMesh::from_cells(
                MeshDimension::ThreeD,
                vec![
                    Point::new(0.0, 0.0, 0.0),
                    Point::new(1.7, 0.2, 0.1),
                    Point::new(0.3, 1.4, 0.2),
                    Point::new(0.2, 0.4, 1.8),
                ],
                vec![CellDefinition::tetrahedron([0, 1, 2, 3])],
            )
            .unwrap(),
        )
    }

    fn rms_gradient_error(
        gradients: &CellField<Vec3>,
        expected: impl Fn(Vec3) -> Vec3,
        mesh: &UnstructuredMesh,
    ) -> f64 {
        (gradients
            .values()
            .iter()
            .zip(mesh.cells())
            .map(|(actual, cell)| (*actual - expected(cell.center)).norm_squared())
            .sum::<f64>()
            / gradients.len() as f64)
            .sqrt()
    }

    fn interpolated_internal_exact_boundary_faces(
        mesh: &UnstructuredMesh,
        values: &CellField<f64>,
        analytic: impl Fn(Vec3) -> f64,
    ) -> FaceField<f64> {
        let mut faces = interpolate_scalar(mesh, values, ScalarBoundaryValue::OwnerValue).unwrap();
        for (index, face) in mesh.faces().iter().enumerate() {
            if face.neighbour.is_none() {
                faces[index] = analytic(face.center);
            }
        }
        faces
    }

    #[test]
    fn least_squares_constant_field_is_zero_on_orthogonal_and_skewed_2d_meshes() {
        for mesh in [individually_patched(mesh()), skewed_multi_cell_mesh_2d()] {
            let values = CellField::filled(&mesh, 7.25);
            let boundary = analytic_fixed_boundaries(&mesh, |_| 7.25);
            let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
            let gradients = least_squares_gradient(&mesh, &stencil, &values).unwrap();
            assert!(gradients.values().iter().all(|gradient| {
                gradient.x.is_finite()
                    && gradient.y.is_finite()
                    && gradient.z.is_finite()
                    && gradient.norm() < TOL
            }));
        }
    }

    #[test]
    fn least_squares_is_exact_on_skewed_multi_cell_2d_linear_field_and_beats_interpolated_gauss() {
        let mesh = skewed_multi_cell_mesh_2d();
        let expected = Vec3::new(2.0, -3.0, 0.0);
        let analytic = |point: Vec3| expected.dot(point) + 5.0;
        let values = CellField::from_cells(&mesh, |_, cell| analytic(cell.center));
        let boundary = analytic_fixed_boundaries(&mesh, analytic);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let least_squares = least_squares_gradient(&mesh, &stencil, &values).unwrap();
        let exact_gauss = gauss_gradient_from_faces(
            &mesh,
            &FaceField::from_faces(&mesh, |_, face| analytic(face.center)),
        )
        .unwrap();
        let interpolated_gauss = gauss_gradient_from_faces(
            &mesh,
            &interpolated_internal_exact_boundary_faces(&mesh, &values, analytic),
        )
        .unwrap();
        let linear = |_| expected;
        let least_squares_error = rms_gradient_error(&least_squares, linear, &mesh);
        let exact_gauss_error = rms_gradient_error(&exact_gauss, linear, &mesh);
        let interpolated_gauss_error = rms_gradient_error(&interpolated_gauss, linear, &mesh);
        assert!(least_squares_error.is_finite());
        assert!(exact_gauss_error.is_finite());
        assert!(interpolated_gauss_error.is_finite());
        assert!(least_squares_error < TOL, "LSQ error={least_squares_error}");
        assert!(
            exact_gauss_error < TOL,
            "exact Gauss error={exact_gauss_error}"
        );
        assert!(
            interpolated_gauss_error > 1.0e-4,
            "interpolated Gauss error={interpolated_gauss_error}"
        );
        assert!(
            least_squares_error < interpolated_gauss_error,
            "LSQ={least_squares_error}, interpolated Gauss={interpolated_gauss_error}"
        );
    }

    #[test]
    fn least_squares_is_linear_exact_and_finite_on_a_skewed_unstructured_3d_tetrahedron() {
        let mesh = skewed_tetrahedron_mesh_3d();
        let expected = Vec3::new(2.0, -3.0, 4.0);
        let analytic = |point: Vec3| expected.dot(point) - 1.5;
        let values = CellField::from_cells(&mesh, |_, cell| analytic(cell.center));
        let boundary = analytic_fixed_boundaries(&mesh, analytic);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let gradients = least_squares_gradient(&mesh, &stencil, &values).unwrap();
        assert!(rms_gradient_error(&gradients, |_| expected, &mesh) < TOL);
        assert!(gradients.values().iter().all(|gradient| {
            gradient.x.is_finite() && gradient.y.is_finite() && gradient.z.is_finite()
        }));
    }

    #[test]
    fn least_squares_quadratic_gradient_errors_are_finite_in_2d_and_3d() {
        for mesh in [skewed_multi_cell_mesh_2d(), skewed_tetrahedron_mesh_3d()] {
            let quadratic = |point: Vec3| point.norm_squared() + 0.5 * point.x * point.y;
            let exact_gradient = |point: Vec3| {
                Vec3::new(
                    2.0 * point.x + 0.5 * point.y,
                    2.0 * point.y + 0.5 * point.x,
                    2.0 * point.z,
                )
            };
            let values = CellField::from_cells(&mesh, |_, cell| quadratic(cell.center));
            let boundary = analytic_fixed_boundaries(&mesh, quadratic);
            let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
            let gradients = least_squares_gradient(&mesh, &stencil, &values).unwrap();
            let error = rms_gradient_error(&gradients, exact_gradient, &mesh);
            assert!(error.is_finite() && error > TOL, "quadratic error={error}");
            assert!(gradients.values().iter().all(|gradient| {
                gradient.x.is_finite() && gradient.y.is_finite() && gradient.z.is_finite()
            }));
        }
    }

    #[test]
    fn least_squares_reuse_and_all_foreign_mesh_inputs_are_rejected() {
        let mesh = skewed_multi_cell_mesh_2d();
        let expected = Vec3::new(2.0, -3.0, 0.0);
        let analytic = |point: Vec3| expected.dot(point) + 5.0;
        let values = CellField::from_cells(&mesh, |_, cell| analytic(cell.center));
        let boundary = analytic_fixed_boundaries(&mesh, analytic);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let allocated = least_squares_gradient(&mesh, &stencil, &values).unwrap();
        let mut reused = CellField::filled(&mesh, Vec3::new(99.0, 99.0, 99.0));
        least_squares_gradient_into(&mesh, &stencil, &values, &mut reused).unwrap();
        assert_eq!(allocated.values(), reused.values());
        let changed = CellField::filled(&mesh, -2.0);
        let changed_allocated = least_squares_gradient(&mesh, &stencil, &changed).unwrap();
        least_squares_gradient_into(&mesh, &stencil, &changed, &mut reused).unwrap();
        assert_eq!(changed_allocated.values(), reused.values());
        assert_ne!(allocated.values(), reused.values());

        let foreign = skewed_multi_cell_mesh_2d();
        let foreign_values = CellField::filled(&foreign, 0.0);
        let mut foreign_output = CellField::filled(&foreign, Vec3::ZERO);
        let foreign_boundary = analytic_fixed_boundaries(&foreign, |_| 0.0);
        let foreign_stencil =
            LeastSquaresGradientStencil::new(&foreign, &foreign_boundary).unwrap();
        assert!(matches!(
            LeastSquaresGradientStencil::new(&mesh, &foreign_boundary),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        assert!(matches!(
            least_squares_gradient(&mesh, &stencil, &foreign_values),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        assert!(matches!(
            least_squares_gradient(&mesh, &foreign_stencil, &values),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        assert!(matches!(
            least_squares_gradient_into(&mesh, &stencil, &values, &mut foreign_output),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
    }

    #[test]
    fn least_squares_uses_three_dimensional_zero_gradient_normals_as_constraints() {
        let mesh = skewed_tetrahedron_mesh_3d();
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| (patch.name.as_str(), ScalarBoundaryCondition::ZeroGradient))
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let values = CellField::filled(&mesh, 2.5);
        let gradients = least_squares_gradient(&mesh, &stencil, &values).unwrap();
        assert!(gradients
            .values()
            .iter()
            .all(|gradient| gradient.norm() < TOL));
    }

    #[test]
    fn projection_decomposition_matches_a_hand_calculated_nonorthogonal_face_flux() {
        let mesh = skewed_two_cell_mesh();
        let (face_index, face) = mesh
            .faces()
            .iter()
            .enumerate()
            .find(|(_, face)| face.neighbour.is_some())
            .unwrap();
        let neighbour = face.neighbour.unwrap();
        let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
        let s = face.area_vector;
        let s_orth = d * (s.dot(d) / d.norm_squared());
        let s_nonorth = s - s_orth;
        let gamma = 1.7;
        let phi_owner = 1.25;
        let phi_neighbour = -0.75;
        let gradient = Vec3::new(2.0, -3.0, 0.0);
        let expected_orth = gamma * s.dot(d) / d.norm_squared() * (phi_neighbour - phi_owner);
        let expected_nonorth = gamma * gradient.dot(s_nonorth);
        let expected_total = expected_orth + expected_nonorth;
        let mut values = CellField::filled(&mesh, 0.0);
        values[face.owner] = phi_owner;
        values[neighbour] = phi_neighbour;

        let decomposition = projection_non_orthogonal_decomposition(&mesh, face_index).unwrap();
        assert_vec_close(decomposition.orthogonal, s_orth);
        assert_vec_close(decomposition.non_orthogonal, s_nonorth);
        assert_vec_close(decomposition.orthogonal + decomposition.non_orthogonal, s);
        assert!(expected_orth.abs() > TOL);
        assert!(expected_nonorth.abs() > TOL);
        assert!(expected_total.abs() > TOL);
        assert_close(
            corrected_internal_diffusion_flux(&mesh, face_index, gamma, &values, gradient).unwrap(),
            expected_total,
        );
    }

    #[test]
    fn explicit_least_squares_correction_reduces_skewed_linear_field_error() {
        let mesh = skewed_multi_cell_mesh_2d();
        let analytic = |point: Vec3| 2. * point.x - 3. * point.y + 5.;
        let values = CellField::from_cells(&mesh, |_, cell| analytic(cell.center));
        let boundary = analytic_fixed_boundaries(&mesh, analytic);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let internal: Vec<_> = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, face)| face.neighbour.is_some())
            .collect();
        assert!(!internal.is_empty());
        let (face, geometry) = internal[0];
        let d =
            mesh.cells()[geometry.neighbour.unwrap()].center - mesh.cells()[geometry.owner].center;
        assert!(geometry.area_vector.dot(d) > 0.0);
        let decomposition = projection_non_orthogonal_decomposition(&mesh, face).unwrap();
        assert_vec_close(
            decomposition.orthogonal + decomposition.non_orthogonal,
            geometry.area_vector,
        );
        assert!(decomposition.non_orthogonal.norm() > 0.1);
        let none = laplacian(
            &mesh,
            &values,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions::default(),
        )
        .unwrap();
        let gauss = laplacian(
            &mesh,
            &values,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions {
                diffusivity_interpolation: DiffusivityInterpolation::Linear,
                non_orthogonal_correction: NonOrthogonalCorrection::Explicit,
                gradient_scheme: GradientScheme::Gauss,
            },
        )
        .unwrap();
        let explicit = laplacian_with_stencil(
            &mesh,
            &values,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions {
                diffusivity_interpolation: DiffusivityInterpolation::Linear,
                non_orthogonal_correction: NonOrthogonalCorrection::Explicit,
                gradient_scheme: GradientScheme::LeastSquares,
            },
            Some(&stencil),
        )
        .unwrap();
        let interior: Vec<_> = mesh
            .cells()
            .iter()
            .enumerate()
            .filter_map(|(index, cell)| {
                cell.faces
                    .iter()
                    .all(|&face| mesh.faces()[face].neighbour.is_some())
                    .then_some(index)
            })
            .collect();
        assert!(!interior.is_empty());
        let error = |field: &CellField<f64>| {
            (interior
                .iter()
                .map(|&cell| field[cell] * field[cell])
                .sum::<f64>()
                / interior.len() as f64)
                .sqrt()
        };
        let none_error = error(&none);
        let gauss_error = error(&gauss);
        let explicit_error = error(&explicit);
        assert!(none_error.is_finite() && gauss_error.is_finite() && explicit_error.is_finite());
        assert!(
            explicit_error < none_error,
            "none={none_error}, gauss={gauss_error}, least_squares={explicit_error}"
        );
        assert!(
            explicit_error < gauss_error,
            "none={none_error}, gauss={gauss_error}, least_squares={explicit_error}"
        );
    }

    fn cube_block_3d() -> UnstructuredMesh {
        let mut points = Vec::new();
        for z in 0..4 {
            for y in 0..4 {
                for x in 0..4 {
                    points.push(Point::new(x as f64, y as f64, z as f64));
                }
            }
        }
        let point = |x: usize, y: usize, z: usize| x + 4 * (y + 4 * z);
        let mut cells = Vec::new();
        for z in 0..3 {
            for y in 0..3 {
                for x in 0..3 {
                    cells.push(CellDefinition::Hexahedron([
                        point(x, y, z),
                        point(x + 1, y, z),
                        point(x + 1, y + 1, z),
                        point(x, y + 1, z),
                        point(x, y, z + 1),
                        point(x + 1, y, z + 1),
                        point(x + 1, y + 1, z + 1),
                        point(x, y + 1, z + 1),
                    ]));
                }
            }
        }
        individually_patched(
            UnstructuredMesh::from_cells(MeshDimension::ThreeD, points, cells).unwrap(),
        )
    }

    fn skewed_block_3d() -> UnstructuredMesh {
        let mut points = Vec::new();
        for z in 0..4 {
            for y in 0..4 {
                for x in 0..4 {
                    points.push(Point::new(
                        x as f64 + 0.14 * (y * y) as f64,
                        y as f64 + 0.06 * (x * x) as f64,
                        z as f64,
                    ));
                }
            }
        }
        let point = |x: usize, y: usize, z: usize| x + 4 * (y + 4 * z);
        let mut cells = Vec::new();
        for z in 0..3 {
            for y in 0..3 {
                for x in 0..3 {
                    cells.push(CellDefinition::Hexahedron([
                        point(x, y, z),
                        point(x + 1, y, z),
                        point(x + 1, y + 1, z),
                        point(x, y + 1, z),
                        point(x, y, z + 1),
                        point(x + 1, y, z + 1),
                        point(x + 1, y + 1, z + 1),
                        point(x, y + 1, z + 1),
                    ]));
                }
            }
        }
        individually_patched(
            UnstructuredMesh::from_cells(MeshDimension::ThreeD, points, cells).unwrap(),
        )
    }

    fn fully_internal_cells(mesh: &UnstructuredMesh) -> Vec<usize> {
        mesh.cells()
            .iter()
            .enumerate()
            .filter_map(|(index, cell)| {
                cell.faces
                    .iter()
                    .all(|&face| mesh.faces()[face].neighbour.is_some())
                    .then_some(index)
            })
            .collect()
    }

    fn rms_cell_error(values: &CellField<f64>, cells: &[usize]) -> f64 {
        (cells
            .iter()
            .map(|&cell| values[cell] * values[cell])
            .sum::<f64>()
            / cells.len() as f64)
            .sqrt()
    }

    #[test]
    fn explicit_least_squares_correction_reduces_skewed_three_dimensional_linear_error() {
        let mesh = skewed_block_3d();
        let analytic = |point: Vec3| 2.0 * point.x - 3.0 * point.y + 4.0 * point.z + 5.0;
        let values = CellField::from_cells(&mesh, |_, cell| analytic(cell.center));
        let boundary = analytic_fixed_boundaries(&mesh, analytic);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let none = laplacian(
            &mesh,
            &values,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions::default(),
        )
        .unwrap();
        let least_squares = laplacian_with_stencil(
            &mesh,
            &values,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions {
                diffusivity_interpolation: DiffusivityInterpolation::Linear,
                non_orthogonal_correction: NonOrthogonalCorrection::Explicit,
                gradient_scheme: GradientScheme::LeastSquares,
            },
            Some(&stencil),
        )
        .unwrap();
        let interior = fully_internal_cells(&mesh);
        assert!(!interior.is_empty());
        let none_error = rms_cell_error(&none, &interior);
        let least_squares_error = rms_cell_error(&least_squares, &interior);
        assert!(none_error.is_finite() && least_squares_error.is_finite());
        assert!(
            least_squares_error < none_error,
            "none={none_error}, least_squares={least_squares_error}"
        );
    }

    #[test]
    fn corrected_diffusion_preserves_constant_and_orthogonal_baselines() {
        let skewed = skewed_multi_cell_mesh_2d();
        let constant = CellField::filled(&skewed, 7.25);
        let constant_boundary = analytic_fixed_boundaries(&skewed, |_| 7.25);
        let constant_stencil =
            LeastSquaresGradientStencil::new(&skewed, &constant_boundary).unwrap();
        for corrected in [
            laplacian(
                &skewed,
                &constant,
                Diffusivity::Constant(1.0),
                &constant_boundary,
                DiffusionOptions::default(),
            )
            .unwrap(),
            laplacian(
                &skewed,
                &constant,
                Diffusivity::Constant(1.0),
                &constant_boundary,
                DiffusionOptions {
                    diffusivity_interpolation: DiffusivityInterpolation::Linear,
                    non_orthogonal_correction: NonOrthogonalCorrection::Explicit,
                    gradient_scheme: GradientScheme::Gauss,
                },
            )
            .unwrap(),
            laplacian_with_stencil(
                &skewed,
                &constant,
                Diffusivity::Constant(1.0),
                &constant_boundary,
                DiffusionOptions {
                    diffusivity_interpolation: DiffusivityInterpolation::Linear,
                    non_orthogonal_correction: NonOrthogonalCorrection::Explicit,
                    gradient_scheme: GradientScheme::LeastSquares,
                },
                Some(&constant_stencil),
            )
            .unwrap(),
        ] {
            assert!(corrected
                .values()
                .iter()
                .all(|value| value.is_finite() && value.abs() < TOL));
        }

        for mesh in [individually_patched(three_cells()), cube_block_3d()] {
            let gradient = match mesh.dimension() {
                MeshDimension::TwoD => Vec3::new(2.0, -3.0, 0.0),
                MeshDimension::ThreeD => Vec3::new(2.0, -3.0, 4.0),
            };
            let values = CellField::from_cells(&mesh, |_, cell| gradient.dot(cell.center) + 5.0);
            let boundary = analytic_fixed_boundaries(&mesh, |point| gradient.dot(point) + 5.0);
            let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
            let none = laplacian(
                &mesh,
                &values,
                Diffusivity::Constant(1.0),
                &boundary,
                DiffusionOptions::default(),
            )
            .unwrap();
            let least_squares = laplacian_with_stencil(
                &mesh,
                &values,
                Diffusivity::Constant(1.0),
                &boundary,
                DiffusionOptions {
                    diffusivity_interpolation: DiffusivityInterpolation::Linear,
                    non_orthogonal_correction: NonOrthogonalCorrection::Explicit,
                    gradient_scheme: GradientScheme::LeastSquares,
                },
                Some(&stencil),
            )
            .unwrap();
            for (base, corrected) in none.values().iter().zip(least_squares.values()) {
                assert_close(*corrected, *base);
            }
        }
    }

    #[test]
    fn three_dimensional_diffusion_api_and_mesh_safety_are_verified() {
        let mesh = cube_block_3d();
        let constant_bc = analytic_fixed_boundaries(&mesh, |_| 7.25);
        let constant = CellField::filled(&mesh, 7.25);
        let integrated = integrated_diffusion(
            &mesh,
            &constant,
            Diffusivity::Constant(2.5),
            &constant_bc,
            DiffusionOptions::default(),
        )
        .unwrap();
        let lap = laplacian(
            &mesh,
            &constant,
            Diffusivity::Constant(2.5),
            &constant_bc,
            DiffusionOptions::default(),
        )
        .unwrap();
        assert!(integrated
            .values()
            .iter()
            .chain(lap.values())
            .all(|v| v.is_finite() && v.abs() < TOL));
        let linear = CellField::from_cells(&mesh, |_, c| {
            2. * c.center.x - 3. * c.center.y + 4. * c.center.z + 5.
        });
        let linear_bc = analytic_fixed_boundaries(&mesh, |p| 2. * p.x - 3. * p.y + 4. * p.z + 5.);
        assert!(laplacian(
            &mesh,
            &linear,
            Diffusivity::Constant(1.),
            &linear_bc,
            DiffusionOptions::default()
        )
        .unwrap()
        .values()
        .iter()
        .all(|v| v.is_finite() && v.abs() < TOL));
        let quadratic = CellField::from_cells(&mesh, |_, c| c.center.norm_squared());
        let quadratic_bc = analytic_fixed_boundaries(&mesh, |p| p.norm_squared());
        assert_close(
            laplacian(
                &mesh,
                &quadratic,
                Diffusivity::Constant(1.),
                &quadratic_bc,
                DiffusionOptions::default(),
            )
            .unwrap()[13],
            6.,
        );
        let gamma = CellField::from_cells(&mesh, |_, c| 1. + 0.2 * c.center.x + 0.1 * c.center.y);
        let variable_integrated = integrated_diffusion(
            &mesh,
            &quadratic,
            Diffusivity::CellField(&gamma),
            &quadratic_bc,
            DiffusionOptions::default(),
        )
        .unwrap();
        let variable_lap = laplacian(
            &mesh,
            &quadratic,
            Diffusivity::CellField(&gamma),
            &quadratic_bc,
            DiffusionOptions::default(),
        )
        .unwrap();
        assert!(variable_integrated
            .values()
            .iter()
            .chain(variable_lap.values())
            .all(|v| v.is_finite()));
        let internal = mesh
            .faces()
            .iter()
            .position(|f| f.neighbour.is_some())
            .unwrap();
        let f = &mesh.faces()[internal];
        let lambda = weight(&mesh, internal).unwrap();
        let harmonic = interpolate_diffusivity(
            &mesh,
            Diffusivity::CellField(&gamma),
            DiffusivityInterpolation::Harmonic,
        )
        .unwrap();
        assert_close(
            harmonic[internal],
            1. / ((1. - lambda) / gamma[f.owner] + lambda / gamma[f.neighbour.unwrap()]),
        );
        let mut into = CellField::filled(&mesh, 99.);
        laplacian_into(
            &mesh,
            &quadratic,
            Diffusivity::CellField(&gamma),
            &quadratic_bc,
            DiffusionOptions::default(),
            &mut into,
        )
        .unwrap();
        assert_eq!(variable_lap.values(), into.values());
        let other = CellField::from_cells(&mesh, |_, c| c.center.x);
        laplacian_into(
            &mesh,
            &other,
            Diffusivity::CellField(&gamma),
            &quadratic_bc,
            DiffusionOptions::default(),
            &mut into,
        )
        .unwrap();
        assert_ne!(variable_lap.values(), into.values());
        for i in 0..mesh.cell_count() {
            assert_close(
                variable_lap[i],
                variable_integrated[i] / mesh.cells()[i].volume,
            );
        }
        let foreign = cube_block_3d();
        let foreign_field = CellField::filled(&foreign, 1.);
        let foreign_bc = analytic_fixed_boundaries(&foreign, |_| 1.);
        let mut foreign_output = CellField::filled(&foreign, 0.);
        assert!(matches!(
            integrated_diffusion(
                &mesh,
                &foreign_field,
                Diffusivity::Constant(1.),
                &constant_bc,
                DiffusionOptions::default()
            ),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        assert!(matches!(
            integrated_diffusion(
                &mesh,
                &constant,
                Diffusivity::Constant(1.),
                &foreign_bc,
                DiffusionOptions::default()
            ),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
        assert!(matches!(
            integrated_diffusion_into(
                &mesh,
                &constant,
                Diffusivity::CellField(&foreign_field),
                &constant_bc,
                DiffusionOptions::default(),
                &mut foreign_output
            ),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
    }

    #[test]
    fn variable_diffusivity_3d_internal_diffusion_is_globally_conservative() {
        let mesh = cube_block_3d();
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| (patch.name.as_str(), ScalarBoundaryCondition::ZeroGradient))
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let values = CellField::from_cells(&mesh, |_, cell| {
            1.5 * cell.center.x - 0.75 * cell.center.y + 0.4 * cell.center.z + 2.0
        });
        let gamma = CellField::from_cells(&mesh, |_, cell| {
            1.0 + 0.15 * cell.center.x + 0.10 * cell.center.y + 0.05 * cell.center.z
        });
        let integrated = integrated_diffusion(
            &mesh,
            &values,
            Diffusivity::CellField(&gamma),
            &boundary,
            DiffusionOptions::default(),
        )
        .unwrap();
        assert!(integrated.values().iter().all(|value| value.is_finite()));
        assert!(integrated.values().iter().any(|value| value.abs() > TOL));
        assert_close(integrated.values().iter().sum(), 0.0);

        let foreign = cube_block_3d();
        let foreign_gamma = CellField::filled(&foreign, 1.0);
        let mut output = CellField::filled(&mesh, 0.0);
        assert!(matches!(
            integrated_diffusion_into(
                &mesh,
                &values,
                Diffusivity::CellField(&foreign_gamma),
                &boundary,
                DiffusionOptions::default(),
                &mut output,
            ),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
    }

    #[test]
    fn corrected_least_squares_diffusion_reuses_buffers_and_rejects_invalid_stencils() {
        let mesh = skewed_multi_cell_mesh_2d();
        let boundary =
            analytic_fixed_boundaries(&mesh, |point| 2.0 * point.x - 3.0 * point.y + 5.0);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let options = DiffusionOptions {
            diffusivity_interpolation: DiffusivityInterpolation::Linear,
            non_orthogonal_correction: NonOrthogonalCorrection::Explicit,
            gradient_scheme: GradientScheme::LeastSquares,
        };
        let first = CellField::from_cells(&mesh, |_, cell| {
            2.0 * cell.center.x - 3.0 * cell.center.y + 5.0
        });
        let second = CellField::from_cells(&mesh, |_, cell| cell.center.x * cell.center.y + 1.0);
        let mut reused = CellField::filled(&mesh, 99.0);
        integrated_diffusion_with_stencil_into(
            &mesh,
            &first,
            Diffusivity::Constant(1.0),
            &boundary,
            options,
            Some(&stencil),
            &mut reused,
        )
        .unwrap();
        let first_result = reused.clone();
        integrated_diffusion_with_stencil_into(
            &mesh,
            &second,
            Diffusivity::Constant(1.0),
            &boundary,
            options,
            Some(&stencil),
            &mut reused,
        )
        .unwrap();
        let expected = integrated_diffusion_with_stencil(
            &mesh,
            &second,
            Diffusivity::Constant(1.0),
            &boundary,
            options,
            Some(&stencil),
        )
        .unwrap();
        assert_ne!(first_result.values(), reused.values());
        assert_eq!(expected.values(), reused.values());
        assert!(reused.values().iter().all(|value| value.is_finite()));

        let gamma = CellField::from_cells(&mesh, |_, cell| {
            1.0 - 0.2 * cell.center.x - 0.1 * cell.center.y
        });
        let face_gamma = interpolate_diffusivity(
            &mesh,
            Diffusivity::CellField(&gamma),
            DiffusivityInterpolation::Linear,
        )
        .unwrap();
        let gradients = least_squares_gradient(&mesh, &stencil, &first).unwrap();
        let mut internal_residual = CellField::filled(&mesh, 0.0);
        for (face_index, face) in mesh.faces().iter().enumerate() {
            let Some(neighbour) = face.neighbour else {
                continue;
            };
            let lambda = weight(&mesh, face_index).unwrap();
            let gradient = gradients[face.owner] * (1.0 - lambda) + gradients[neighbour] * lambda;
            let flux = corrected_internal_diffusion_flux(
                &mesh,
                face_index,
                face_gamma[face_index],
                &first,
                gradient,
            )
            .unwrap();
            assert!(flux.is_finite());
            let mut isolated = CellField::filled(&mesh, 0.0);
            isolated[face.owner] += flux;
            isolated[neighbour] -= flux;
            assert_close(isolated[face.owner], flux);
            assert_close(isolated[neighbour], -flux);
            for (cell, contribution) in isolated.values().iter().enumerate() {
                if cell != face.owner && cell != neighbour {
                    assert_close(*contribution, 0.0);
                }
            }
            internal_residual[face.owner] += flux;
            internal_residual[neighbour] -= flux;
        }
        assert!(internal_residual
            .values()
            .iter()
            .all(|value| value.is_finite()));
        assert_close(internal_residual.values().iter().sum(), 0.0);
        assert!(matches!(
            integrated_diffusion_with_stencil(
                &mesh,
                &second,
                Diffusivity::Constant(1.0),
                &boundary,
                options,
                None,
            ),
            Err(NumericsError::MissingLeastSquaresGradientStencil)
        ));

        let foreign = skewed_multi_cell_mesh_2d();
        let foreign_boundary = analytic_fixed_boundaries(&foreign, |_| 0.0);
        let foreign_stencil =
            LeastSquaresGradientStencil::new(&foreign, &foreign_boundary).unwrap();
        assert!(matches!(
            integrated_diffusion_with_stencil(
                &mesh,
                &second,
                Diffusivity::Constant(1.0),
                &boundary,
                options,
                Some(&foreign_stencil),
            ),
            Err(NumericsError::Field(FieldError::MeshMismatch { .. }))
        ));
    }
}
