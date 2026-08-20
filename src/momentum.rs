//! Generic unstructured finite-volume assembly for one transported velocity component.
//!
//! The equation convention is `transient + div(phi u) - div(nu grad(u)) = source`.
//! A positive face flux is owner-to-neighbour internally and outward at a boundary.

use crate::{
    bicgstab, integrated_diffusion, integrated_diffusion_with_stencil, interpolate_diffusivity,
    least_squares_gradient, residual, CellField, CsrBuilder, CsrMatrix, DiffusionOptions,
    Diffusivity, FaceField, FieldError, LeastSquaresGradientStencil, LinearAlgebraError,
    LinearSolveReport, LinearSolverOptions, NonOrthogonalCorrection, NumericsError,
    ResolvedScalarBoundaryConditions, ScalarBoundaryCondition, UnstructuredMesh, Vec3,
};

#[derive(Clone, Copy, Debug)]
pub struct MomentumOptions<'a> {
    pub viscosity: Diffusivity<'a>,
    pub boundary: &'a ResolvedScalarBoundaryConditions,
    pub flux: &'a FaceField<f64>,
    pub old: Option<&'a CellField<f64>>,
    pub time_step: Option<f64>,
    pub source: Option<&'a CellField<f64>>,
    /// Current component iterate used exclusively by equation under-relaxation.
    /// It is distinct from `old`, which is the previous physical time level.
    pub relaxation_field: Option<&'a CellField<f64>>,
    pub correction_field: Option<&'a CellField<f64>>,
    pub least_squares_stencil: Option<&'a LeastSquaresGradientStencil>,
    pub under_relaxation: f64,
    pub diffusion: DiffusionOptions,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MomentumComponent {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VelocityBoundaryCondition {
    FixedVelocity(Vec3),
    ZeroGradient,
}

/// Mesh-bound, component-resolved velocity boundary conditions. Assembly uses
/// one component view at a time, with no patch-name lookup in face loops.
#[derive(Clone, Debug)]
pub struct ResolvedVelocityBoundaryConditions {
    x: ResolvedScalarBoundaryConditions,
    y: ResolvedScalarBoundaryConditions,
    z: ResolvedScalarBoundaryConditions,
}

impl ResolvedVelocityBoundaryConditions {
    pub fn strict(
        mesh: &UnstructuredMesh,
        assignments: &[(&str, VelocityBoundaryCondition)],
    ) -> Result<Self, NumericsError> {
        let scalar = |component: MomentumComponent| {
            assignments
                .iter()
                .map(|(name, condition)| {
                    let value = match condition {
                        VelocityBoundaryCondition::FixedVelocity(value) => match component {
                            MomentumComponent::X => ScalarBoundaryCondition::FixedValue(value.x),
                            MomentumComponent::Y => ScalarBoundaryCondition::FixedValue(value.y),
                            MomentumComponent::Z => ScalarBoundaryCondition::FixedValue(value.z),
                        },
                        VelocityBoundaryCondition::ZeroGradient => {
                            ScalarBoundaryCondition::ZeroGradient
                        }
                    };
                    (*name, value)
                })
                .collect::<Vec<_>>()
        };
        Ok(Self {
            x: ResolvedScalarBoundaryConditions::strict(mesh, &scalar(MomentumComponent::X))?,
            y: ResolvedScalarBoundaryConditions::strict(mesh, &scalar(MomentumComponent::Y))?,
            z: ResolvedScalarBoundaryConditions::strict(mesh, &scalar(MomentumComponent::Z))?,
        })
    }

    pub fn component(&self, component: MomentumComponent) -> &ResolvedScalarBoundaryConditions {
        match component {
            MomentumComponent::X => &self.x,
            MomentumComponent::Y => &self.y,
            MomentumComponent::Z => &self.z,
        }
    }
}

impl<'a> MomentumOptions<'a> {
    pub fn steady(
        viscosity: Diffusivity<'a>,
        boundary: &'a ResolvedScalarBoundaryConditions,
        flux: &'a FaceField<f64>,
    ) -> Self {
        Self {
            viscosity,
            boundary,
            flux,
            old: None,
            time_step: None,
            source: None,
            relaxation_field: None,
            correction_field: None,
            least_squares_stencil: None,
            under_relaxation: 1.0,
            diffusion: DiffusionOptions::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct MomentumSystem {
    mesh_id: crate::MeshId,
    matrix: CsrMatrix,
    rhs: Vec<f64>,
    unrelaxed_diagonal: Vec<f64>,
}

impl MomentumSystem {
    pub fn mesh_id(&self) -> crate::MeshId {
        self.mesh_id
    }
    pub fn matrix(&self) -> &CsrMatrix {
        &self.matrix
    }
    pub fn rhs(&self) -> &[f64] {
        &self.rhs
    }
    /// Pre-under-relaxation momentum diagonal, intended for future rAU use.
    pub fn unrelaxed_diagonal(&self) -> &[f64] {
        &self.unrelaxed_diagonal
    }
    pub fn true_residual(&self, solution: &[f64]) -> Result<Vec<f64>, MomentumError> {
        Ok(residual(&self.matrix, solution, &self.rhs)?)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum MomentumError {
    Field(FieldError),
    Numerics(NumericsError),
    Linear(LinearAlgebraError),
    InvalidTimeStep { value: f64 },
    InvalidUnderRelaxation { value: f64 },
    InvalidDensity { value: f64 },
    MissingVelocityForTransient,
    MissingVelocityForUnderRelaxation,
    MissingCorrectionField,
    BackflowOnZeroGradient { face: usize },
    NonFiniteFlux { face: usize, value: f64 },
    NonFiniteSource { cell: usize, value: f64 },
}
impl std::fmt::Display for MomentumError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for MomentumError {}
impl From<FieldError> for MomentumError {
    fn from(value: FieldError) -> Self {
        Self::Field(value)
    }
}
impl From<NumericsError> for MomentumError {
    fn from(value: NumericsError) -> Self {
        Self::Numerics(value)
    }
}
impl From<LinearAlgebraError> for MomentumError {
    fn from(value: LinearAlgebraError) -> Self {
        Self::Linear(value)
    }
}

/// Assembles first-order-upwind convection, two-point diffusion, optional backward
/// Euler transient storage, and volumetric explicit source for one component.
pub fn assemble_momentum_component(
    mesh: &UnstructuredMesh,
    options: MomentumOptions<'_>,
) -> Result<MomentumSystem, MomentumError> {
    options.boundary.ensure_mesh(mesh)?;
    options.flux.ensure_mesh(mesh)?;
    if let Some(old) = options.old {
        old.ensure_mesh(mesh)?;
    }
    if let Some(source) = options.source {
        source.ensure_mesh(mesh)?;
    }
    if let Some(field) = options.relaxation_field {
        field.ensure_mesh(mesh)?;
    }
    if let Some(field) = options.correction_field {
        field.ensure_mesh(mesh)?;
    }
    if let Some(stencil) = options.least_squares_stencil {
        stencil.ensure_mesh(mesh)?;
    }
    if !(options.under_relaxation.is_finite()
        && 0.0 < options.under_relaxation
        && options.under_relaxation <= 1.0)
    {
        return Err(MomentumError::InvalidUnderRelaxation {
            value: options.under_relaxation,
        });
    }
    if let Some(dt) = options.time_step {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(MomentumError::InvalidTimeStep { value: dt });
        }
        if options.old.is_none() {
            return Err(MomentumError::MissingVelocityForTransient);
        }
    }
    let viscosity = interpolate_diffusivity(
        mesh,
        options.viscosity,
        options.diffusion.diffusivity_interpolation,
    )?;
    let mut entries = Vec::with_capacity(mesh.face_count() * 4 + mesh.cell_count());
    let mut rhs = vec![0.0; mesh.cell_count()];
    for (cell_index, cell) in mesh.cells().iter().enumerate() {
        if let Some(source) = options.source {
            if !source[cell_index].is_finite() {
                return Err(MomentumError::NonFiniteSource {
                    cell: cell_index,
                    value: source[cell_index],
                });
            }
            rhs[cell_index] += source[cell_index] * cell.volume;
        }
        if let Some(dt) = options.time_step {
            let coefficient = cell.volume / dt;
            entries.push((cell_index, cell_index, coefficient));
            rhs[cell_index] +=
                coefficient * options.old.expect("validated transient velocity")[cell_index];
        }
    }
    for (face_index, face) in mesh.faces().iter().enumerate() {
        let phi = options.flux[face_index];
        if !phi.is_finite() {
            return Err(MomentumError::NonFiniteFlux {
                face: face_index,
                value: phi,
            });
        }
        if let Some(neighbour) = face.neighbour {
            let d = mesh.cells()[neighbour].center - mesh.cells()[face.owner].center;
            let d2 = d.norm_squared();
            if !d2.is_finite() || d2 <= f64::EPSILON {
                return Err(
                    NumericsError::DegenerateOwnerNeighbourDistance { face: face_index }.into(),
                );
            }
            let diffusion = viscosity[face_index] * face.area_vector.dot(d) / d2;
            if !diffusion.is_finite() || diffusion < 0.0 {
                return Err(
                    NumericsError::InvalidNonOrthogonalGeometry { face: face_index }.into(),
                );
            }
            // Diffusion: [+D,-D;-D,+D]. Upwind convection: phi>=0 gives
            // owner +phi*u_P, neighbour -phi*u_P; phi<0 gives owner +phi*u_N,
            // neighbour -phi*u_N.
            entries.extend_from_slice(&[
                (face.owner, face.owner, diffusion + phi.max(0.0)),
                (face.owner, neighbour, -diffusion + phi.min(0.0)),
                (neighbour, face.owner, -diffusion - phi.max(0.0)),
                (neighbour, neighbour, diffusion - phi.min(0.0)),
            ]);
        } else {
            let condition = options
                .boundary
                .condition(face_index)
                .ok_or(NumericsError::MissingBoundaryCondition { face: face_index })?;
            let d = face.center - mesh.cells()[face.owner].center;
            let d2 = d.norm_squared();
            if !d2.is_finite() || d2 <= f64::EPSILON {
                return Err(NumericsError::DegenerateOwnerFaceDistance { face: face_index }.into());
            }
            let diffusion = viscosity[face_index] * face.area_vector.dot(d) / d2;
            match condition {
                ScalarBoundaryCondition::FixedValue(value) => {
                    if !value.is_finite() {
                        return Err(MomentumError::NonFiniteSource {
                            cell: face.owner,
                            value,
                        });
                    }
                    entries.push((face.owner, face.owner, diffusion + phi.max(0.0)));
                    rhs[face.owner] += diffusion * value - phi.min(0.0) * value;
                }
                ScalarBoundaryCondition::ZeroGradient => {
                    if phi < 0.0 {
                        return Err(MomentumError::BackflowOnZeroGradient { face: face_index });
                    }
                    entries.push((face.owner, face.owner, phi));
                }
            }
        }
    }
    add_explicit_nonorthogonal_rhs(mesh, options, &mut rhs)?;
    let mut builder = CsrBuilder::new(mesh.cell_count(), mesh.cell_count());
    for (row, column, value) in entries {
        builder.add(row, column, value)?;
    }
    let unrelaxed = builder.finalize()?;
    let unrelaxed_diagonal = unrelaxed.diagonal()?;
    if options.under_relaxation == 1.0 {
        return Ok(MomentumSystem {
            mesh_id: mesh.id(),
            matrix: unrelaxed,
            rhs,
            unrelaxed_diagonal,
        });
    }
    let mut relaxed = CsrBuilder::new(mesh.cell_count(), mesh.cell_count());
    for row in 0..mesh.cell_count() {
        for index in unrelaxed.row_offsets()[row]..unrelaxed.row_offsets()[row + 1] {
            let column = unrelaxed.column_indices()[index];
            let mut value = unrelaxed.values()[index];
            if row == column {
                value /= options.under_relaxation;
            }
            relaxed.add(row, column, value)?;
        }
        let current = options
            .relaxation_field
            .ok_or(MomentumError::MissingVelocityForUnderRelaxation)?[row];
        rhs[row] += (1.0 - options.under_relaxation) / options.under_relaxation
            * unrelaxed_diagonal[row]
            * current;
    }
    Ok(MomentumSystem {
        mesh_id: mesh.id(),
        matrix: relaxed.finalize()?,
        rhs,
        unrelaxed_diagonal,
    })
}

fn add_explicit_nonorthogonal_rhs(
    mesh: &UnstructuredMesh,
    options: MomentumOptions<'_>,
    rhs: &mut [f64],
) -> Result<(), MomentumError> {
    if options.diffusion.non_orthogonal_correction != NonOrthogonalCorrection::Explicit {
        return Ok(());
    }
    let field = options
        .correction_field
        .ok_or(MomentumError::MissingCorrectionField)?;
    let mut baseline_options = options.diffusion;
    baseline_options.non_orthogonal_correction = NonOrthogonalCorrection::None;
    let baseline = integrated_diffusion(
        mesh,
        field,
        options.viscosity,
        options.boundary,
        baseline_options,
    )?;
    let corrected = integrated_diffusion_with_stencil(
        mesh,
        field,
        options.viscosity,
        options.boundary,
        options.diffusion,
        options.least_squares_stencil,
    )?;
    for (rhs, (corrected, baseline)) in rhs
        .iter_mut()
        .zip(corrected.values().iter().zip(baseline.values()))
    {
        *rhs += corrected - baseline;
    }
    Ok(())
}

pub fn solve_momentum_component(
    system: &MomentumSystem,
    initial_guess: &mut CellField<f64>,
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, MomentumError> {
    if system.mesh_id() != initial_guess.mesh_id() {
        return Err(FieldError::MeshMismatch {
            expected: system.mesh_id(),
            actual: initial_guess.mesh_id(),
        }
        .into());
    }
    bicgstab(
        system.matrix(),
        system.rhs(),
        initial_guess.values_mut(),
        options,
    )
    .map_err(Into::into)
}

/// Reconstructs the explicit normalized source `-(1/rho) grad(p)` from a
/// caller-owned least-squares stencil. This does not modify pressure or flux.
pub fn pressure_gradient_source(
    mesh: &UnstructuredMesh,
    pressure: &CellField<f64>,
    stencil: &LeastSquaresGradientStencil,
    density: f64,
) -> Result<CellField<Vec3>, MomentumError> {
    if !density.is_finite() || density <= 0.0 {
        return Err(MomentumError::InvalidDensity { value: density });
    }
    let mut source = least_squares_gradient(mesh, stencil, pressure)?;
    for value in source.values_mut() {
        *value = -*value / density;
    }
    Ok(source)
}

/// Constructs a mesh-bound constant acceleration/body-force field for the
/// three velocity components. Component assembly consumes its selected scalar
/// component as a volumetric explicit source.
pub fn constant_body_force(mesh: &UnstructuredMesh, value: Vec3) -> CellField<Vec3> {
    CellField::filled(mesh, value)
}

/// Extracts one velocity/source component while retaining the active mesh
/// identity. This bridges vector body-force and pressure-gradient fields to
/// scalar component assembly without allowing a foreign vector field.
pub fn momentum_component_field(
    mesh: &UnstructuredMesh,
    field: &CellField<Vec3>,
    component: MomentumComponent,
) -> Result<CellField<f64>, MomentumError> {
    field.ensure_mesh(mesh)?;
    Ok(CellField::from_cells(mesh, |index, _| match component {
        MomentumComponent::X => field[index].x,
        MomentumComponent::Y => field[index].y,
        MomentumComponent::Z => field[index].z,
    }))
}

/// Solves independently assembled x/y/z component systems into one cell-centred
/// velocity field. The supplied field is retained as each component's initial
/// guess, which is required by future segregated outer iterations.
pub fn solve_momentum_velocity(
    systems: [&MomentumSystem; 3],
    velocity: &mut CellField<Vec3>,
    options: LinearSolverOptions,
) -> Result<[LinearSolveReport; 3], MomentumError> {
    for system in systems {
        if system.mesh_id() != velocity.mesh_id() {
            return Err(FieldError::MeshMismatch {
                expected: system.mesh_id(),
                actual: velocity.mesh_id(),
            }
            .into());
        }
    }
    let mut x: Vec<f64> = velocity.values().iter().map(|value| value.x).collect();
    let mut y: Vec<f64> = velocity.values().iter().map(|value| value.y).collect();
    let mut z: Vec<f64> = velocity.values().iter().map(|value| value.z).collect();
    let reports = [
        bicgstab(systems[0].matrix(), systems[0].rhs(), &mut x, options)?,
        bicgstab(systems[1].matrix(), systems[1].rhs(), &mut y, options)?,
        bicgstab(systems[2].matrix(), systems[2].rhs(), &mut z, options)?,
    ];
    for (value, ((x, y), z)) in velocity
        .values_mut()
        .iter_mut()
        .zip(x.into_iter().zip(y).zip(z))
    {
        *value = Vec3::new(x, y, z);
    }
    Ok(reports)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point, ScalarBoundaryCondition,
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
        let faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "boundary".into(),
            face_indices: faces,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    fn boundary(mesh: &UnstructuredMesh) -> ResolvedScalarBoundaryConditions {
        ResolvedScalarBoundaryConditions::strict(
            mesh,
            &[("boundary", ScalarBoundaryCondition::FixedValue(0.0))],
        )
        .unwrap()
    }

    fn internal_flux(mesh: &UnstructuredMesh, value: f64) -> FaceField<f64> {
        FaceField::from_faces(
            mesh,
            |_, face| if face.neighbour.is_some() { value } else { 0.0 },
        )
    }

    fn channel(ny: usize) -> UnstructuredMesh {
        let point = |x: usize, y: usize| x + 2 * y;
        let mut points = Vec::new();
        for y in 0..=ny {
            points.push(Point::new(0.0, y as f64 / ny as f64, 0.0));
            points.push(Point::new(1.0, y as f64 / ny as f64, 0.0));
        }
        let cells = (0..ny)
            .map(|y| {
                CellDefinition::polygon(vec![
                    point(0, y),
                    point(1, y),
                    point(1, y + 1),
                    point(0, y + 1),
                ])
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

    fn single_hexahedron() -> UnstructuredMesh {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 1.0),
                Point::new(1.0, 1.0, 1.0),
                Point::new(0.0, 1.0, 1.0),
            ],
            vec![CellDefinition::Hexahedron([0, 1, 2, 3, 4, 5, 6, 7])],
        )
        .unwrap();
        let patches = mesh
            .faces()
            .iter()
            .enumerate()
            .map(|(face, _)| BoundaryPatch {
                name: format!("face-{face}"),
                face_indices: vec![face],
                boundary_type: BoundaryType::Wall,
            })
            .collect();
        mesh.with_boundary_patches(patches).unwrap()
    }

    fn single_skewed_hexahedron() -> UnstructuredMesh {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.3, 1.0, 0.0),
                Point::new(0.3, 1.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
                Point::new(1.0, 0.0, 1.0),
                Point::new(1.3, 1.0, 1.0),
                Point::new(0.3, 1.0, 1.0),
            ],
            vec![CellDefinition::Hexahedron([0, 1, 2, 3, 4, 5, 6, 7])],
        )
        .unwrap();
        let patches = mesh
            .faces()
            .iter()
            .enumerate()
            .map(|(face, _)| BoundaryPatch {
                name: format!("face-{face}"),
                face_indices: vec![face],
                boundary_type: BoundaryType::Wall,
            })
            .collect();
        mesh.with_boundary_patches(patches).unwrap()
    }

    fn skewed_grid() -> UnstructuredMesh {
        let (nx, ny) = (3, 3);
        let point = |x: usize, y: usize| x + (nx + 1) * y;
        let points = (0..=ny)
            .flat_map(|y| {
                (0..=nx).map(move |x| {
                    Point::new(
                        x as f64 + 0.13 * (y * y) as f64,
                        y as f64 + 0.07 * (x * x) as f64,
                        0.0,
                    )
                })
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

    fn square_grid(nx: usize, ny: usize) -> UnstructuredMesh {
        let point = |x: usize, y: usize| x + (nx + 1) * y;
        let points = (0..=ny)
            .flat_map(|y| {
                (0..=nx).map(move |x| Point::new(x as f64 / nx as f64, y as f64 / ny as f64, 0.0))
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

    fn couette_boundary(mesh: &UnstructuredMesh) -> ResolvedScalarBoundaryConditions {
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                let y = mesh.faces()[patch.face_indices[0]].center.y;
                let condition = if y < 1.0e-12 {
                    ScalarBoundaryCondition::FixedValue(0.0)
                } else if (y - 1.0).abs() < 1.0e-12 {
                    ScalarBoundaryCondition::FixedValue(1.0)
                } else {
                    ScalarBoundaryCondition::ZeroGradient
                };
                (patch.name.as_str(), condition)
            })
            .collect();
        ResolvedScalarBoundaryConditions::strict(mesh, &assignments).unwrap()
    }
    #[test]
    fn first_order_upwind_component_assembly_is_available() {
        let _ = assemble_momentum_component;
    }

    #[test]
    fn positive_and_negative_internal_upwind_coefficients_are_hand_correct() {
        let mesh = two_cells();
        let boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("boundary", ScalarBoundaryCondition::ZeroGradient)],
        )
        .unwrap();
        let internal = mesh
            .faces()
            .iter()
            .find(|face| face.neighbour.is_some())
            .unwrap();
        let owner = internal.owner;
        let neighbour = internal.neighbour.unwrap();
        for flux_value in [2.0, -2.0] {
            let flux = internal_flux(&mesh, flux_value);
            assert_eq!(
                flux.values().iter().filter(|value| **value != 0.0).count(),
                1
            );
            let system = assemble_momentum_component(
                &mesh,
                MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux),
            )
            .unwrap();
            let positive = flux_value > 0.0;
            let expected_owner_diagonal = 1.0 + if positive { 2.0 } else { 0.0 };
            let expected_neighbour_diagonal = 1.0 + if positive { 0.0 } else { 2.0 };
            assert_eq!(
                system.matrix().get(owner, owner),
                Some(expected_owner_diagonal)
            );
            assert_eq!(
                system.matrix().get(neighbour, neighbour),
                Some(expected_neighbour_diagonal)
            );
            assert_eq!(
                system.matrix().get(owner, neighbour),
                Some(-1.0 + flux_value.min(0.0))
            );
            assert_eq!(
                system.matrix().get(neighbour, owner),
                Some(-1.0 - flux_value.max(0.0))
            );
        }
    }

    #[test]
    fn backward_euler_transient_and_source_terms_match_the_hand_calculation() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let old = CellField::from_values(&mesh, vec![3.0, -2.0]).unwrap();
        let source = CellField::filled(&mesh, 4.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        options.old = Some(&old);
        options.time_step = Some(0.5);
        options.source = Some(&source);
        let system = assemble_momentum_component(&mesh, options).unwrap();
        // V=1 and dt=0.5 give a_t=2, b=a_t*u_old + source*V.
        assert_eq!(system.matrix().get(0, 0), Some(2.0));
        assert_eq!(system.matrix().get(1, 1), Some(2.0));
        assert_eq!(system.rhs(), &[10.0, 0.0]);
    }

    #[test]
    fn velocity_boundary_conditions_resolve_each_component_once() {
        let mesh = two_cells();
        let boundary = ResolvedVelocityBoundaryConditions::strict(
            &mesh,
            &[(
                "boundary",
                VelocityBoundaryCondition::FixedVelocity(Vec3::new(2.0, -3.0, 4.0)),
            )],
        )
        .unwrap();
        let boundary_face = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_none())
            .unwrap();
        assert_eq!(
            boundary
                .component(MomentumComponent::X)
                .condition(boundary_face),
            Some(ScalarBoundaryCondition::FixedValue(2.0))
        );
        assert_eq!(
            boundary
                .component(MomentumComponent::Y)
                .condition(boundary_face),
            Some(ScalarBoundaryCondition::FixedValue(-3.0))
        );
        assert_eq!(
            boundary
                .component(MomentumComponent::Z)
                .condition(boundary_face),
            Some(ScalarBoundaryCondition::FixedValue(4.0))
        );
    }

    #[test]
    fn vector_component_solve_keeps_component_rhs_values_separate() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let system = |value| {
            let source = CellField::filled(&mesh, value);
            let old = CellField::filled(&mesh, 0.0);
            let mut options = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
            options.old = Some(&old);
            options.time_step = Some(1.0);
            options.source = Some(&source);
            assemble_momentum_component(&mesh, options).unwrap()
        };
        let x = system(1.0);
        let y = system(-2.0);
        let z = system(3.0);
        let mut velocity = CellField::filled(&mesh, Vec3::new(9.0, 8.0, 7.0));
        let reports = solve_momentum_velocity(
            [&x, &y, &z],
            &mut velocity,
            LinearSolverOptions {
                absolute_tolerance: 1.0e-13,
                relative_tolerance: 1.0e-13,
                max_iterations: 100,
            },
        )
        .unwrap();
        assert!(reports.iter().all(LinearSolveReport::converged));
        assert!(velocity
            .values()
            .iter()
            .all(|value| *value == Vec3::new(1.0, -2.0, 3.0)));
    }

    #[test]
    fn explicit_momentum_correction_requires_current_component_field() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
        options.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Explicit;
        options.diffusion.gradient_scheme = crate::GradientScheme::LeastSquares;
        assert!(matches!(
            assemble_momentum_component(&mesh, options),
            Err(MomentumError::MissingCorrectionField)
        ));
    }

    #[test]
    fn equation_under_relaxation_scales_diagonal_and_preserves_current_component() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let old = CellField::filled(&mesh, 3.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        options.old = Some(&old);
        options.time_step = Some(0.5);
        options.relaxation_field = Some(&old);
        options.under_relaxation = 0.5;
        let system = assemble_momentum_component(&mesh, options).unwrap();
        // Unrelaxed a_P=V/dt=2. For alpha=0.5: a_P'=4 and b'=2*3 + 2*3=12.
        assert_eq!(system.unrelaxed_diagonal(), &[2.0, 2.0]);
        assert_eq!(system.matrix().get(0, 0), Some(4.0));
        assert_eq!(system.rhs(), &[12.0, 12.0]);
    }

    #[test]
    fn steady_diffusion_recovers_the_couette_linear_profile() {
        let mesh = channel(12);
        let boundary = couette_boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let system = assemble_momentum_component(
            &mesh,
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux),
        )
        .unwrap();
        let mut velocity = CellField::filled(&mesh, 0.0);
        let report = solve_momentum_component(
            &system,
            &mut velocity,
            LinearSolverOptions {
                absolute_tolerance: 1.0e-13,
                relative_tolerance: 1.0e-13,
                max_iterations: 10_000,
            },
        )
        .unwrap();
        assert!(report.converged());
        for (value, cell) in velocity.values().iter().zip(mesh.cells()) {
            assert!((*value - cell.center.y).abs() < 1.0e-10);
        }
    }

    #[test]
    fn poiseuille_source_profile_converges_under_refinement() {
        let mut errors = Vec::new();
        for resolution in [8, 16, 32] {
            let mesh = channel(resolution);
            let assignments: Vec<_> = mesh
                .boundary_patches()
                .iter()
                .map(|patch| {
                    let y = mesh.faces()[patch.face_indices[0]].center.y;
                    (
                        patch.name.as_str(),
                        if y < 1.0e-12 || (y - 1.0).abs() < 1.0e-12 {
                            ScalarBoundaryCondition::FixedValue(0.0)
                        } else {
                            ScalarBoundaryCondition::ZeroGradient
                        },
                    )
                })
                .collect();
            let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
            let flux = FaceField::filled(&mesh, 0.0);
            let source = CellField::filled(&mesh, 2.0);
            let mut options = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
            options.source = Some(&source);
            let system = assemble_momentum_component(&mesh, options).unwrap();
            let mut velocity = CellField::filled(&mesh, 0.0);
            let report = solve_momentum_component(
                &system,
                &mut velocity,
                LinearSolverOptions {
                    absolute_tolerance: 1.0e-13,
                    relative_tolerance: 1.0e-13,
                    max_iterations: 10_000,
                },
            )
            .unwrap();
            assert!(report.converged());
            errors.push(
                (velocity
                    .values()
                    .iter()
                    .zip(mesh.cells())
                    .map(|(value, cell)| (value - cell.center.y * (1.0 - cell.center.y)).powi(2))
                    .sum::<f64>()
                    / resolution as f64)
                    .sqrt(),
            );
        }
        assert!(errors[2] < errors[1] && errors[1] < errors[0]);
    }

    #[test]
    fn linear_pressure_has_the_expected_constant_explicit_momentum_source() {
        let mesh = channel(3);
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                let centre = mesh.faces()[patch.face_indices[0]].center;
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(2.0 * centre.x - 3.0 * centre.y + 7.0),
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let pressure = CellField::from_cells(&mesh, |_, cell| {
            2.0 * cell.center.x - 3.0 * cell.center.y + 7.0
        });
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let source = pressure_gradient_source(&mesh, &pressure, &stencil, 2.0).unwrap();
        for value in source.values() {
            assert!((value.x + 1.0).abs() < 1.0e-12);
            assert!((value.y - 1.5).abs() < 1.0e-12);
            assert!(value.z.abs() < 1.0e-12);
        }
    }

    #[test]
    fn constant_body_force_is_mesh_bound_and_component_exact() {
        let mesh = two_cells();
        let force = constant_body_force(&mesh, Vec3::new(1.0, -2.0, 3.0));
        assert_eq!(force.values(), &[Vec3::new(1.0, -2.0, 3.0); 2]);
    }

    #[test]
    fn vector_solve_rejects_a_same_sized_velocity_from_another_mesh() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let system = assemble_momentum_component(
            &mesh,
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux),
        )
        .unwrap();
        let foreign = two_cells();
        let mut velocity = CellField::filled(&foreign, Vec3::default());
        assert!(matches!(
            solve_momentum_velocity(
                [&system, &system, &system],
                &mut velocity,
                LinearSolverOptions::default(),
            ),
            Err(MomentumError::Field(FieldError::MeshMismatch { .. }))
        ));
    }

    #[test]
    fn steady_under_relaxation_uses_the_current_component_not_transient_history() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let current = CellField::filled(&mesh, 3.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
        options.relaxation_field = Some(&current);
        options.under_relaxation = 0.5;
        let system = assemble_momentum_component(&mesh, options).unwrap();
        for row in 0..mesh.cell_count() {
            assert_eq!(
                system.matrix().get(row, row),
                Some(2.0 * system.unrelaxed_diagonal()[row])
            );
            assert_eq!(system.rhs()[row], 3.0 * system.unrelaxed_diagonal()[row]);
        }
    }

    #[test]
    fn vector_component_field_extracts_the_requested_mesh_bound_source() {
        let mesh = two_cells();
        let vector = CellField::filled(&mesh, Vec3::new(1.0, -2.0, 3.0));
        assert_eq!(
            momentum_component_field(&mesh, &vector, MomentumComponent::Y)
                .unwrap()
                .values(),
            &[-2.0, -2.0]
        );
    }

    #[test]
    fn fixed_velocity_inflow_contributes_only_the_prescribed_rhs_flux() {
        let mesh = channel(1);
        let inflow_face = mesh
            .faces()
            .iter()
            .enumerate()
            .find(|(_, face)| face.neighbour.is_none() && face.center.x < 1.0e-12)
            .map(|(index, _)| index)
            .unwrap();
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(if patch.face_indices[0] == inflow_face {
                        5.0
                    } else {
                        0.0
                    }),
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let flux = FaceField::from_faces(
            &mesh,
            |index, _| if index == inflow_face { -2.0 } else { 0.0 },
        );
        let old = CellField::filled(&mesh, 0.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        options.old = Some(&old);
        options.time_step = Some(1.0);
        let system = assemble_momentum_component(&mesh, options).unwrap();
        assert_eq!(system.rhs(), &[10.0]);
        assert_eq!(system.matrix().get(0, 0), Some(1.0));
    }

    #[test]
    fn zero_gradient_outflow_adds_owner_coefficient_without_boundary_rhs() {
        let mesh = channel(1);
        let outflow_face = mesh
            .faces()
            .iter()
            .enumerate()
            .find(|(_, face)| face.neighbour.is_none() && (face.center.x - 1.0).abs() < 1.0e-12)
            .map(|(index, _)| index)
            .unwrap();
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| (patch.name.as_str(), ScalarBoundaryCondition::ZeroGradient))
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let flux = FaceField::from_faces(
            &mesh,
            |index, _| if index == outflow_face { 2.0 } else { 0.0 },
        );
        let old = CellField::filled(&mesh, 0.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        options.old = Some(&old);
        options.time_step = Some(1.0);
        let system = assemble_momentum_component(&mesh, options).unwrap();
        assert_eq!(system.matrix().get(0, 0), Some(3.0));
        assert_eq!(system.rhs(), &[0.0]);
    }

    #[test]
    fn zero_gradient_backflow_is_rejected() {
        let mesh = channel(1);
        let face = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_none())
            .unwrap();
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| (patch.name.as_str(), ScalarBoundaryCondition::ZeroGradient))
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let flux = FaceField::from_faces(&mesh, |index, _| if index == face { -1.0 } else { 0.0 });
        assert!(matches!(
            assemble_momentum_component(
                &mesh,
                MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux)
            ),
            Err(MomentumError::BackflowOnZeroGradient { face: actual }) if actual == face
        ));
    }

    #[test]
    fn component_solve_rejects_a_same_sized_initial_guess_from_another_mesh() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let old = CellField::filled(&mesh, 0.0);
        let mut assembly = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        assembly.old = Some(&old);
        assembly.time_step = Some(1.0);
        let system = assemble_momentum_component(&mesh, assembly).unwrap();
        let foreign = two_cells();
        let mut guess = CellField::filled(&foreign, 0.0);
        assert!(matches!(
            solve_momentum_component(&system, &mut guess, LinearSolverOptions::default()),
            Err(MomentumError::Field(FieldError::MeshMismatch { .. }))
        ));
    }

    #[test]
    fn backward_euler_preserves_a_uniform_velocity_without_spatial_terms() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let old = CellField::filled(&mesh, 4.5);
        let mut assembly = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        assembly.old = Some(&old);
        assembly.time_step = Some(0.37);
        let system = assemble_momentum_component(&mesh, assembly).unwrap();
        let mut solution = CellField::filled(&mesh, 0.0);
        let report =
            solve_momentum_component(&system, &mut solution, LinearSolverOptions::default())
                .unwrap();
        assert!(report.converged());
        assert_eq!(solution.values(), &[4.5, 4.5]);
    }

    #[test]
    fn backward_euler_constant_source_advances_by_time_step() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let old = CellField::filled(&mesh, 1.25);
        let source = CellField::filled(&mesh, -3.0);
        let mut assembly = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        assembly.old = Some(&old);
        assembly.time_step = Some(0.2);
        assembly.source = Some(&source);
        let system = assemble_momentum_component(&mesh, assembly).unwrap();
        let mut solution = CellField::filled(&mesh, 0.0);
        solve_momentum_component(&system, &mut solution, LinearSolverOptions::default()).unwrap();
        assert_eq!(solution.values(), &[0.65, 0.65]);
    }

    #[test]
    fn variable_viscosity_uses_the_selected_linear_or_harmonic_face_interpolation() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let viscosity = CellField::from_values(&mesh, vec![1.0, 3.0]).unwrap();
        for (interpolation, expected) in [
            (crate::DiffusivityInterpolation::Linear, 2.0),
            (crate::DiffusivityInterpolation::Harmonic, 1.5),
        ] {
            let mut options =
                MomentumOptions::steady(Diffusivity::CellField(&viscosity), &boundary, &flux);
            options.diffusion.diffusivity_interpolation = interpolation;
            let system = assemble_momentum_component(&mesh, options).unwrap();
            let internal = mesh
                .faces()
                .iter()
                .find(|face| face.neighbour.is_some())
                .unwrap();
            assert_eq!(
                system
                    .matrix()
                    .get(internal.owner, internal.neighbour.unwrap()),
                Some(-expected)
            );
        }
    }

    #[test]
    fn constant_velocity_is_invariant_under_a_closed_conservative_boundary_flux() {
        let mesh = channel(1);
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(4.0),
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let flux = FaceField::from_faces(&mesh, |_, face| {
            if face.neighbour.is_some() {
                0.0
            } else if face.center.x < 1.0e-12 {
                -2.0
            } else if (face.center.x - 1.0).abs() < 1.0e-12 {
                2.0
            } else {
                0.0
            }
        });
        let system = assemble_momentum_component(
            &mesh,
            MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux),
        )
        .unwrap();
        assert_eq!(system.true_residual(&[4.0]).unwrap(), vec![0.0]);
    }

    #[test]
    fn one_internal_upwind_face_scatters_equal_and_opposite_convective_flux() {
        let mesh = two_cells();
        let boundary = ResolvedScalarBoundaryConditions::strict(
            &mesh,
            &[("boundary", ScalarBoundaryCondition::ZeroGradient)],
        )
        .unwrap();
        let old = CellField::filled(&mesh, 0.0);
        let assemble = |face_flux| {
            let flux = internal_flux(&mesh, face_flux);
            let mut options = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
            options.old = Some(&old);
            options.time_step = Some(1.0);
            assemble_momentum_component(&mesh, options).unwrap()
        };
        let no_convection = assemble(0.0);
        let convection = assemble(2.0);
        let value = [3.0, 5.0];
        let baseline = no_convection.true_residual(&value).unwrap();
        let with_convection = convection.true_residual(&value).unwrap();
        // F = Phi * u_upwind = 2 * 3. Residual is b-Au, so subtracting
        // the residuals recovers the conservative owner +F/neighbour -F scatter.
        assert_eq!(baseline[0] - with_convection[0], 6.0);
        assert_eq!(baseline[1] - with_convection[1], -6.0);
    }

    #[test]
    fn orthogonal_three_dimensional_diffusion_recovers_a_linear_component() {
        let mesh = single_hexahedron();
        let analytic = |point: Vec3| 2.0 * point.x - 3.0 * point.y + 4.0 * point.z + 5.0;
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(analytic(
                        mesh.faces()[patch.face_indices[0]].center,
                    )),
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let flux = FaceField::filled(&mesh, 0.0);
        let system = assemble_momentum_component(
            &mesh,
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux),
        )
        .unwrap();
        let mut value = CellField::filled(&mesh, 0.0);
        solve_momentum_component(&system, &mut value, LinearSolverOptions::default()).unwrap();
        assert!((value[0] - analytic(mesh.cells()[0].center)).abs() < 1.0e-12);
    }

    #[test]
    fn skewed_lsq_momentum_correction_is_finite_and_changes_only_the_rhs() {
        let mesh = skewed_grid();
        let analytic = |point: Vec3| 2.0 * point.x - 3.0 * point.y + 5.0;
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(analytic(
                        mesh.faces()[patch.face_indices[0]].center,
                    )),
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let flux = FaceField::filled(&mesh, 0.0);
        let baseline = assemble_momentum_component(
            &mesh,
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux),
        )
        .unwrap();
        let current = CellField::from_cells(&mesh, |_, cell| analytic(cell.center));
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let mut options = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
        options.correction_field = Some(&current);
        options.least_squares_stencil = Some(&stencil);
        options.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Explicit;
        options.diffusion.gradient_scheme = crate::GradientScheme::LeastSquares;
        let corrected = assemble_momentum_component(&mesh, options).unwrap();
        assert_eq!(corrected.matrix(), baseline.matrix());
        assert!(corrected.rhs().iter().all(|value| value.is_finite()));
        let mut solution = CellField::filled(&mesh, 0.0);
        assert!(solve_momentum_component(
            &corrected,
            &mut solution,
            LinearSolverOptions::default()
        )
        .unwrap()
        .converged());
    }

    #[test]
    fn skewed_three_dimensional_lsq_momentum_correction_is_finite_and_converges() {
        let mesh = single_skewed_hexahedron();
        let analytic = |point: Vec3| 2.0 * point.x - 3.0 * point.y + 4.0 * point.z + 5.0;
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(analytic(
                        mesh.faces()[patch.face_indices[0]].center,
                    )),
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let current = CellField::from_cells(&mesh, |_, cell| analytic(cell.center));
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let flux = FaceField::filled(&mesh, 0.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
        options.correction_field = Some(&current);
        options.least_squares_stencil = Some(&stencil);
        options.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Explicit;
        options.diffusion.gradient_scheme = crate::GradientScheme::LeastSquares;
        let system = assemble_momentum_component(&mesh, options).unwrap();
        assert!(system.rhs().iter().all(|value| value.is_finite()));
        let mut solution = CellField::filled(&mesh, 0.0);
        assert!(
            solve_momentum_component(&system, &mut solution, LinearSolverOptions::default())
                .unwrap()
                .converged()
        );
        assert!((solution[0] - analytic(mesh.cells()[0].center)).abs() < 1.0e-10);
    }

    #[test]
    fn manufactured_two_dimensional_convection_diffusion_solution_converges_under_refinement() {
        let mut errors = Vec::new();
        for resolution in [8, 16] {
            let mesh = square_grid(resolution, resolution);
            let analytic = |point: Vec3| point.x + point.y;
            let assignments: Vec<_> = mesh
                .boundary_patches()
                .iter()
                .map(|patch| {
                    (
                        patch.name.as_str(),
                        ScalarBoundaryCondition::FixedValue(analytic(
                            mesh.faces()[patch.face_indices[0]].center,
                        )),
                    )
                })
                .collect();
            let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
            let flux = FaceField::from_faces(&mesh, |_, face| face.area_vector.x);
            let source = CellField::filled(&mesh, 1.0);
            let mut options = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
            options.source = Some(&source);
            let system = assemble_momentum_component(&mesh, options).unwrap();
            let mut solution = CellField::filled(&mesh, 0.0);
            assert!(solve_momentum_component(
                &system,
                &mut solution,
                LinearSolverOptions {
                    absolute_tolerance: 1.0e-13,
                    relative_tolerance: 1.0e-13,
                    max_iterations: 10_000,
                }
            )
            .unwrap()
            .converged());
            errors.push(
                (solution
                    .values()
                    .iter()
                    .zip(mesh.cells())
                    .map(|(value, cell)| (value - analytic(cell.center)).powi(2))
                    .sum::<f64>()
                    / mesh.cell_count() as f64)
                    .sqrt(),
            );
        }
        assert!(errors[1] < errors[0]);
    }

    #[test]
    fn unit_under_relaxation_is_exactly_the_unrelaxed_system_without_a_current_field() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = internal_flux(&mesh, 2.0);
        let baseline = assemble_momentum_component(
            &mesh,
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux),
        )
        .unwrap();
        let mut unit = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
        unit.under_relaxation = 1.0;
        let relaxed = assemble_momentum_component(&mesh, unit).unwrap();
        assert_eq!(relaxed.matrix(), baseline.matrix());
        assert_eq!(relaxed.rhs(), baseline.rhs());
        assert_eq!(relaxed.unrelaxed_diagonal(), baseline.unrelaxed_diagonal());
    }

    #[test]
    fn reported_true_momentum_residual_is_the_independent_csr_defect() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let flux = FaceField::filled(&mesh, 0.0);
        let old = CellField::filled(&mesh, 2.0);
        let mut options = MomentumOptions::steady(Diffusivity::Constant(0.0), &boundary, &flux);
        options.old = Some(&old);
        options.time_step = Some(1.0);
        let system = assemble_momentum_component(&mesh, options).unwrap();
        let exact = [2.0, 2.0];
        assert_eq!(system.true_residual(&exact).unwrap(), vec![0.0, 0.0]);
        assert_eq!(system.true_residual(&[1.0, 3.0]).unwrap(), vec![1.0, -1.0]);
    }

    #[test]
    fn fixed_velocity_boundary_diffusion_has_the_expected_diagonal_and_rhs_contribution() {
        let mesh = channel(1);
        let left_face = mesh
            .faces()
            .iter()
            .enumerate()
            .find(|(_, face)| face.neighbour.is_none() && face.center.x < 1.0e-12)
            .map(|(index, _)| index)
            .unwrap();
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .map(|patch| {
                (
                    patch.name.as_str(),
                    ScalarBoundaryCondition::FixedValue(if patch.face_indices[0] == left_face {
                        3.0
                    } else {
                        0.0
                    }),
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let flux = FaceField::filled(&mesh, 0.0);
        let system = assemble_momentum_component(
            &mesh,
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux),
        )
        .unwrap();
        // Each of the four faces has D_b=1*(S·d)/(d·d)=2; only the left
        // boundary has u_b=3, so A_PP=8 and b=2*3.
        assert_eq!(system.matrix().get(0, 0), Some(8.0));
        assert_eq!(system.rhs(), &[6.0]);
    }

    #[test]
    fn non_finite_flux_source_viscosity_and_time_controls_are_rejected() {
        let mesh = two_cells();
        let boundary = boundary(&mesh);
        let old = CellField::filled(&mesh, 0.0);
        let source = CellField::filled(&mesh, f64::NAN);
        let zero_flux = FaceField::filled(&mesh, 0.0);
        let mut source_options =
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &zero_flux);
        source_options.source = Some(&source);
        assert!(matches!(
            assemble_momentum_component(&mesh, source_options),
            Err(MomentumError::NonFiniteSource { .. })
        ));
        for (flux_value, expected) in [(f64::NAN, true), (f64::INFINITY, true), (0.0, false)] {
            let flux = FaceField::filled(&mesh, flux_value);
            let options = MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
            assert_eq!(
                assemble_momentum_component(&mesh, options).is_err(),
                expected
            );
        }
        let flux = FaceField::filled(&mesh, 0.0);
        let invalid_viscosity =
            MomentumOptions::steady(Diffusivity::Constant(f64::NAN), &boundary, &flux);
        assert!(matches!(
            assemble_momentum_component(&mesh, invalid_viscosity),
            Err(MomentumError::Numerics(_))
        ));
        let mut invalid_time =
            MomentumOptions::steady(Diffusivity::Constant(1.0), &boundary, &flux);
        invalid_time.old = Some(&old);
        invalid_time.time_step = Some(f64::INFINITY);
        assert!(matches!(
            assemble_momentum_component(&mesh, invalid_time),
            Err(MomentumError::InvalidTimeStep { .. })
        ));
    }
}
