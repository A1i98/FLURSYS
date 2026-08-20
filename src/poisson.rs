//! Generic unstructured finite-volume Poisson assembly for `-div(Gamma grad(phi)) = q`.

use crate::{
    bicgstab, cg, integrated_diffusion, integrated_diffusion_with_stencil, interpolate_diffusivity,
    pcg, CellField, CsrBuilder, CsrMatrix, DiffusionOptions, Diffusivity, FieldError,
    JacobiPreconditioner, LeastSquaresGradientStencil, LinearAlgebraError, LinearSolveReport,
    LinearSolverOptions, NonOrthogonalCorrection, NumericsError, ResolvedScalarBoundaryConditions,
    ScalarBoundaryCondition, UnstructuredMesh,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoissonReference {
    pub cell: usize,
    pub value: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct PoissonOptions<'a> {
    pub diffusivity: Diffusivity<'a>,
    pub boundary: &'a ResolvedScalarBoundaryConditions,
    pub diffusion: DiffusionOptions,
    pub source: Option<&'a CellField<f64>>,
    /// Current field used exclusively to evaluate explicit non-orthogonal RHS terms.
    pub correction_field: Option<&'a CellField<f64>>,
    pub least_squares_stencil: Option<&'a LeastSquaresGradientStencil>,
    pub reference: Option<PoissonReference>,
}

impl<'a> PoissonOptions<'a> {
    pub fn new(
        diffusivity: Diffusivity<'a>,
        boundary: &'a ResolvedScalarBoundaryConditions,
    ) -> Self {
        Self {
            diffusivity,
            boundary,
            diffusion: DiffusionOptions::default(),
            source: None,
            correction_field: None,
            least_squares_stencil: None,
            reference: None,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PoissonSystem {
    matrix: CsrMatrix,
    rhs: Vec<f64>,
}

impl PoissonSystem {
    pub fn matrix(&self) -> &CsrMatrix {
        &self.matrix
    }

    pub fn rhs(&self) -> &[f64] {
        &self.rhs
    }

    pub fn true_residual(&self, solution: &[f64]) -> Result<Vec<f64>, PoissonError> {
        Ok(crate::residual(&self.matrix, solution, &self.rhs)?)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PoissonLinearSolver {
    Cg,
    Pcg,
    BiCgStab,
}

#[derive(Clone, Debug, PartialEq)]
pub enum PoissonError {
    Field(FieldError),
    Numerics(NumericsError),
    Linear(LinearAlgebraError),
    MissingReferenceForPureNeumann,
    IncompatiblePureNeumannSource { integrated_source: f64 },
    InvalidReferenceCell { cell: usize, cell_count: usize },
    InvalidReferenceValue { value: f64 },
    MissingCorrectionField,
    NonFiniteSource { cell: usize, value: f64 },
}

impl std::fmt::Display for PoissonError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Field(error) => write!(formatter, "Poisson field validation failed: {error:?}"),
            Self::Numerics(error) => write!(formatter, "Poisson assembly failed: {error:?}"),
            Self::Linear(error) => write!(formatter, "Poisson linear solve failed: {error}"),
            Self::MissingReferenceForPureNeumann => write!(
                formatter,
                "pure ZeroGradient Poisson systems require an explicit reference cell"
            ),
            Self::IncompatiblePureNeumannSource { integrated_source } => write!(
                formatter,
                "pure-Neumann source is incompatible: integrated source is {integrated_source}"
            ),
            Self::InvalidReferenceCell { cell, cell_count } => write!(
                formatter,
                "reference cell {cell} is outside the {cell_count}-cell mesh"
            ),
            Self::InvalidReferenceValue { value } => {
                write!(formatter, "reference value {value} is not finite")
            }
            Self::MissingCorrectionField => write!(
                formatter,
                "explicit non-orthogonal Poisson assembly requires a current scalar field"
            ),
            Self::NonFiniteSource { cell, value } => {
                write!(
                    formatter,
                    "source value {value} in cell {cell} is not finite"
                )
            }
        }
    }
}

impl std::error::Error for PoissonError {}

impl From<NumericsError> for PoissonError {
    fn from(error: NumericsError) -> Self {
        Self::Numerics(error)
    }
}

impl From<FieldError> for PoissonError {
    fn from(error: FieldError) -> Self {
        Self::Field(error)
    }
}

impl From<LinearAlgebraError> for PoissonError {
    fn from(error: LinearAlgebraError) -> Self {
        Self::Linear(error)
    }
}

/// Assembles `-div(Gamma grad(phi)) = q` into a symmetric two-point CSR system.
///
/// The implicit matrix contains only projection-consistent two-point diffusion.
/// Explicit non-orthogonal terms are evaluated from `correction_field` and added
/// to the RHS; the orthogonal matrix is therefore independent of correction sweeps.
pub fn assemble_poisson(
    mesh: &UnstructuredMesh,
    options: PoissonOptions<'_>,
) -> Result<PoissonSystem, PoissonError> {
    options.boundary.ensure_mesh(mesh)?;
    if let Some(source) = options.source {
        source.ensure_mesh(mesh)?;
    }
    if let Some(field) = options.correction_field {
        field.ensure_mesh(mesh)?;
    }
    if let Some(stencil) = options.least_squares_stencil {
        stencil.ensure_mesh(mesh)?;
    }

    let gamma = interpolate_diffusivity(
        mesh,
        options.diffusivity,
        options.diffusion.diffusivity_interpolation,
    )?;
    let mut rhs = vec![0.0; mesh.cell_count()];
    let mut entries = Vec::with_capacity(mesh.face_count() * 4);
    let mut has_fixed_value = false;

    for (cell_index, cell) in mesh.cells().iter().enumerate() {
        if let Some(source) = options.source {
            let value = source[cell_index];
            if !value.is_finite() {
                return Err(PoissonError::NonFiniteSource {
                    cell: cell_index,
                    value,
                });
            }
            rhs[cell_index] += value * cell.volume;
        }
    }

    for (face_index, face) in mesh.faces().iter().enumerate() {
        if let Some(neighbour) = face.neighbour {
            let owner_center = mesh.cells()[face.owner].center;
            let d = mesh.cells()[neighbour].center - owner_center;
            let d2 = d.norm_squared();
            if !d2.is_finite() || d2 <= f64::EPSILON {
                return Err(
                    NumericsError::DegenerateOwnerNeighbourDistance { face: face_index }.into(),
                );
            }
            let coefficient = gamma[face_index] * face.area_vector.dot(d) / d2;
            if !coefficient.is_finite() || coefficient <= 0.0 {
                return Err(
                    NumericsError::InvalidNonOrthogonalGeometry { face: face_index }.into(),
                );
            }
            entries.extend_from_slice(&[
                (face.owner, face.owner, coefficient),
                (face.owner, neighbour, -coefficient),
                (neighbour, neighbour, coefficient),
                (neighbour, face.owner, -coefficient),
            ]);
        } else {
            match options
                .boundary
                .condition(face_index)
                .ok_or(NumericsError::MissingBoundaryCondition { face: face_index })?
            {
                ScalarBoundaryCondition::FixedValue(value) => {
                    if !value.is_finite() {
                        return Err(PoissonError::NonFiniteSource {
                            cell: face.owner,
                            value,
                        });
                    }
                    has_fixed_value = true;
                    let d = face.center - mesh.cells()[face.owner].center;
                    let d2 = d.norm_squared();
                    if !d2.is_finite() || d2 <= f64::EPSILON {
                        return Err(NumericsError::DegenerateOwnerFaceDistance {
                            face: face_index,
                        }
                        .into());
                    }
                    let coefficient = gamma[face_index] * face.area_vector.dot(d) / d2;
                    if !coefficient.is_finite() || coefficient <= 0.0 {
                        return Err(NumericsError::InvalidNonOrthogonalGeometry {
                            face: face_index,
                        }
                        .into());
                    }
                    entries.push((face.owner, face.owner, coefficient));
                    rhs[face.owner] += coefficient * value;
                }
                ScalarBoundaryCondition::ZeroGradient => {}
            }
        }
    }

    if !has_fixed_value {
        let integrated_source: f64 = rhs.iter().sum();
        let source_scale: f64 = rhs.iter().map(|value| value.abs()).sum();
        if integrated_source.abs() > 1.0e-12 * source_scale.max(1.0) {
            return Err(PoissonError::IncompatiblePureNeumannSource { integrated_source });
        }
        if options.reference.is_none() {
            return Err(PoissonError::MissingReferenceForPureNeumann);
        }
    }

    add_explicit_correction_rhs(mesh, options, &mut rhs)?;
    let entries = apply_reference(mesh, entries, &mut rhs, options.reference)?;
    let mut builder = CsrBuilder::new(mesh.cell_count(), mesh.cell_count());
    for (row, column, value) in entries {
        builder.add(row, column, value)?;
    }
    Ok(PoissonSystem {
        matrix: builder.finalize()?,
        rhs,
    })
}

fn add_explicit_correction_rhs(
    mesh: &UnstructuredMesh,
    options: PoissonOptions<'_>,
    rhs: &mut [f64],
) -> Result<(), PoissonError> {
    if options.diffusion.non_orthogonal_correction != NonOrthogonalCorrection::Explicit {
        return Ok(());
    }
    let field = options
        .correction_field
        .ok_or(PoissonError::MissingCorrectionField)?;
    let mut baseline_options = options.diffusion;
    baseline_options.non_orthogonal_correction = NonOrthogonalCorrection::None;
    let baseline = integrated_diffusion(
        mesh,
        field,
        options.diffusivity,
        options.boundary,
        baseline_options,
    )?;
    let corrected = integrated_diffusion_with_stencil(
        mesh,
        field,
        options.diffusivity,
        options.boundary,
        options.diffusion,
        options.least_squares_stencil,
    )?;
    for (rhs, (corrected, baseline)) in rhs
        .iter_mut()
        .zip(corrected.values().iter().zip(baseline.values()))
    {
        // With A = -I_orth, -I_orth - I_nonorth = q gives A phi = q + I_nonorth.
        *rhs += corrected - baseline;
    }
    Ok(())
}

fn apply_reference(
    mesh: &UnstructuredMesh,
    entries: Vec<(usize, usize, f64)>,
    rhs: &mut [f64],
    reference: Option<PoissonReference>,
) -> Result<Vec<(usize, usize, f64)>, PoissonError> {
    let Some(reference) = reference else {
        return Ok(entries);
    };
    if reference.cell >= mesh.cell_count() {
        return Err(PoissonError::InvalidReferenceCell {
            cell: reference.cell,
            cell_count: mesh.cell_count(),
        });
    }
    if !reference.value.is_finite() {
        return Err(PoissonError::InvalidReferenceValue {
            value: reference.value,
        });
    }
    let mut constrained = Vec::with_capacity(entries.len() + 1);
    for (row, column, value) in entries {
        if row == reference.cell {
            continue;
        }
        if column == reference.cell {
            rhs[row] -= value * reference.value;
            continue;
        }
        constrained.push((row, column, value));
    }
    rhs[reference.cell] = reference.value;
    constrained.push((reference.cell, reference.cell, 1.0));
    Ok(constrained)
}

pub fn solve_poisson(
    system: &PoissonSystem,
    solver: PoissonLinearSolver,
    solution: &mut [f64],
    options: LinearSolverOptions,
) -> Result<LinearSolveReport, PoissonError> {
    Ok(match solver {
        PoissonLinearSolver::Cg => cg(system.matrix(), system.rhs(), solution, options)?,
        PoissonLinearSolver::Pcg => {
            let preconditioner = JacobiPreconditioner::new(system.matrix())?;
            pcg(
                system.matrix(),
                system.rhs(),
                solution,
                &preconditioner,
                options,
            )?
        }
        PoissonLinearSolver::BiCgStab => {
            bicgstab(system.matrix(), system.rhs(), solution, options)?
        }
    })
}

/// Performs explicit non-orthogonal fixed-point correction sweeps while keeping
/// the implicit two-point matrix formulation unchanged.
pub fn solve_poisson_correction_sweeps(
    mesh: &UnstructuredMesh,
    assembly: PoissonOptions<'_>,
    solver: PoissonLinearSolver,
    solution: &mut [f64],
    solver_options: LinearSolverOptions,
    sweeps: usize,
) -> Result<(PoissonSystem, LinearSolveReport), PoissonError> {
    if solution.len() != mesh.cell_count() {
        return Err(LinearAlgebraError::DimensionMismatch {
            expected: mesh.cell_count(),
            actual: solution.len(),
        }
        .into());
    }
    if assembly.diffusion.non_orthogonal_correction != NonOrthogonalCorrection::Explicit {
        let system = assemble_poisson(mesh, assembly)?;
        let report = solve_poisson(&system, solver, solution, solver_options)?;
        return Ok((system, report));
    }
    let mut result = None;
    for _ in 0..=sweeps {
        let current = CellField::from_values(mesh, solution.to_vec())?;
        let sweep_assembly = PoissonOptions {
            diffusivity: assembly.diffusivity,
            boundary: assembly.boundary,
            diffusion: assembly.diffusion,
            source: assembly.source,
            correction_field: Some(&current),
            least_squares_stencil: assembly.least_squares_stencil,
            reference: assembly.reference,
        };
        let system = assemble_poisson(mesh, sweep_assembly)?;
        let report = solve_poisson(&system, solver, solution, solver_options)?;
        result = Some((system, report));
    }
    Ok(result.expect("the inclusive correction loop always runs once"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point, ScalarBoundaryCondition,
    };

    fn single_square() -> UnstructuredMesh {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2, 3])],
        )
        .unwrap();
        let boundary = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(index, face)| face.neighbour.is_none().then_some(index))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "boundary".to_string(),
            face_indices: boundary,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    fn two_squares() -> UnstructuredMesh {
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
        let boundary = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(index, face)| face.neighbour.is_none().then_some(index))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "boundary".to_string(),
            face_indices: boundary,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    fn boundary(
        mesh: &UnstructuredMesh,
        condition: ScalarBoundaryCondition,
    ) -> ResolvedScalarBoundaryConditions {
        ResolvedScalarBoundaryConditions::strict(mesh, &[("boundary", condition)]).unwrap()
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

    fn grid(nx: usize, ny: usize) -> UnstructuredMesh {
        let index = |x: usize, y: usize| x + (nx + 1) * y;
        let mut points = Vec::new();
        for y in 0..=ny {
            for x in 0..=nx {
                points.push(Point::new(x as f64 / nx as f64, y as f64 / ny as f64, 0.0));
            }
        }
        let mut cells = Vec::new();
        for y in 0..ny {
            for x in 0..nx {
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

    fn skewed_grid() -> UnstructuredMesh {
        let nx = 3;
        let ny = 3;
        let index = |x: usize, y: usize| x + (nx + 1) * y;
        let mut points = Vec::new();
        for y in 0..=ny {
            for x in 0..=nx {
                points.push(Point::new(
                    x as f64 + 0.13 * (y * y) as f64,
                    y as f64 + 0.07 * (x * x) as f64,
                    0.0,
                ));
            }
        }
        let mut cells = Vec::new();
        for y in 0..ny {
            for x in 0..nx {
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

    fn analytic_boundary(
        mesh: &UnstructuredMesh,
        function: impl Fn(crate::Vec3) -> f64,
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

    fn solve(system: &PoissonSystem) -> Vec<f64> {
        let mut values = vec![0.0; system.rhs().len()];
        let report = solve_poisson(
            system,
            PoissonLinearSolver::Pcg,
            &mut values,
            LinearSolverOptions {
                absolute_tolerance: 1.0e-13,
                relative_tolerance: 1.0e-13,
                max_iterations: 10_000,
            },
        )
        .unwrap();
        assert!(report.converged());
        assert!(system
            .true_residual(&values)
            .unwrap()
            .iter()
            .all(|value| value.abs() < 1.0e-10));
        values
    }

    #[test]
    fn poisson_assembly_builds_the_requested_system() {
        let _ = assemble_poisson;
    }

    #[test]
    fn dirichlet_and_source_terms_match_an_independent_single_cell_calculation() {
        let mesh = single_square();
        let source = CellField::filled(&mesh, 2.0);
        let boundary = boundary(&mesh, ScalarBoundaryCondition::FixedValue(3.0));
        let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        options.source = Some(&source);
        let system = assemble_poisson(&mesh, options).unwrap();

        // Four unit-length faces are each a distance 1/2 from the cell centre:
        // a_b = 1 * (1 * 1/2)/(1/2)^2 = 2, so A_00 = 8 and b_0 = 8*3 + 2*1.
        assert_eq!(system.matrix().get(0, 0), Some(8.0));
        assert_eq!(system.rhs(), &[26.0]);
    }

    #[test]
    fn internal_two_point_block_is_symmetric_with_negative_neighbour_coupling() {
        let mesh = two_squares();
        let boundary = boundary(&mesh, ScalarBoundaryCondition::FixedValue(0.0));
        let system = assemble_poisson(
            &mesh,
            PoissonOptions::new(Diffusivity::Constant(1.0), &boundary),
        )
        .unwrap();
        assert_eq!(system.matrix().get(0, 1), Some(-1.0));
        assert_eq!(system.matrix().get(1, 0), Some(-1.0));
        assert!(system.matrix().get(0, 0).unwrap() > 0.0);
        assert!(system.matrix().is_symmetric(1.0e-14));
    }

    #[test]
    fn constant_and_linear_dirichlet_problems_solve_through_pcg() {
        for analytic in [
            Box::new(|_| 7.25) as Box<dyn Fn(crate::Vec3) -> f64>,
            Box::new(|point| 2.0 * point.x - 3.0 * point.y + 5.0),
        ] {
            let mesh = grid(4, 3);
            let boundary = analytic_boundary(&mesh, &analytic);
            let system = assemble_poisson(
                &mesh,
                PoissonOptions::new(Diffusivity::Constant(1.0), &boundary),
            )
            .unwrap();
            let values = solve(&system);
            for (value, cell) in values.iter().zip(mesh.cells()) {
                assert!((*value - analytic(cell.center)).abs() < 1.0e-10);
            }
        }
    }

    #[test]
    fn pure_neumann_requires_a_compatible_source_and_symmetric_reference() {
        let mesh = two_squares();
        let boundary = boundary(&mesh, ScalarBoundaryCondition::ZeroGradient);
        assert!(matches!(
            assemble_poisson(
                &mesh,
                PoissonOptions::new(Diffusivity::Constant(1.0), &boundary),
            ),
            Err(PoissonError::MissingReferenceForPureNeumann)
        ));
        let source = CellField::filled(&mesh, 1.0);
        let mut incompatible = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        incompatible.source = Some(&source);
        incompatible.reference = Some(PoissonReference {
            cell: 0,
            value: 3.2,
        });
        assert!(matches!(
            assemble_poisson(&mesh, incompatible),
            Err(PoissonError::IncompatiblePureNeumannSource { .. })
        ));

        let mut compatible = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        compatible.reference = Some(PoissonReference {
            cell: 0,
            value: 3.2,
        });
        let system = assemble_poisson(&mesh, compatible).unwrap();
        assert!(system.matrix().is_symmetric(1.0e-14));
        assert_eq!(solve(&system), vec![3.2, 3.2]);
    }

    #[test]
    fn zero_gradient_faces_add_no_coefficients_or_rhs() {
        let mesh = single_square();
        let zero_gradient = boundary(&mesh, ScalarBoundaryCondition::ZeroGradient);
        let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &zero_gradient);
        options.reference = Some(PoissonReference {
            cell: 0,
            value: 0.0,
        });
        let system = assemble_poisson(&mesh, options).unwrap();
        assert_eq!(system.matrix().get(0, 0), Some(1.0));
        assert_eq!(system.rhs(), &[0.0]);
    }

    #[test]
    fn mixed_fixed_value_and_zero_gradient_boundaries_solve() {
        let mesh = individually_patched(single_square());
        let assignments: Vec<_> = mesh
            .boundary_patches()
            .iter()
            .enumerate()
            .map(|(index, patch)| {
                (
                    patch.name.as_str(),
                    if index == 0 {
                        ScalarBoundaryCondition::FixedValue(4.0)
                    } else {
                        ScalarBoundaryCondition::ZeroGradient
                    },
                )
            })
            .collect();
        let boundary = ResolvedScalarBoundaryConditions::strict(&mesh, &assignments).unwrap();
        let system = assemble_poisson(
            &mesh,
            PoissonOptions::new(Diffusivity::Constant(1.0), &boundary),
        )
        .unwrap();
        assert_eq!(solve(&system), vec![4.0]);
    }

    #[test]
    fn poisson_rejects_non_finite_and_foreign_mesh_bound_inputs() {
        let mesh = single_square();
        let boundary = boundary(&mesh, ScalarBoundaryCondition::FixedValue(0.0));
        assert!(matches!(
            assemble_poisson(
                &mesh,
                PoissonOptions::new(Diffusivity::Constant(f64::NAN), &boundary),
            ),
            Err(PoissonError::Numerics(_))
        ));
        let invalid_source = CellField::filled(&mesh, f64::INFINITY);
        let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        options.source = Some(&invalid_source);
        assert!(matches!(
            assemble_poisson(&mesh, options),
            Err(PoissonError::NonFiniteSource { .. })
        ));

        let foreign_mesh = single_square();
        let foreign_source = CellField::filled(&foreign_mesh, 0.0);
        let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        options.source = Some(&foreign_source);
        assert!(matches!(
            assemble_poisson(&mesh, options),
            Err(PoissonError::Field(FieldError::MeshMismatch { .. }))
        ));
    }

    #[test]
    fn quadratic_manufactured_solution_converges_under_two_dimensional_refinement() {
        let mut l1_errors = Vec::new();
        let mut l2_errors = Vec::new();
        let mut linf_errors = Vec::new();
        for resolution in [4, 8, 16] {
            let mesh = grid(resolution, resolution);
            let boundary = analytic_boundary(&mesh, |point| point.x * point.x + point.y * point.y);
            let source = CellField::filled(&mesh, -4.0);
            let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
            options.source = Some(&source);
            let values = solve(&assemble_poisson(&mesh, options).unwrap());
            let errors: Vec<_> = values
                .iter()
                .zip(mesh.cells())
                .map(|(value, cell)| {
                    (value - (cell.center.x * cell.center.x + cell.center.y * cell.center.y)).abs()
                })
                .collect();
            l1_errors.push(errors.iter().sum::<f64>() / errors.len() as f64);
            l2_errors.push(
                (errors.iter().map(|value| value * value).sum::<f64>() / errors.len() as f64)
                    .sqrt(),
            );
            linf_errors.push(errors.iter().copied().fold(0.0_f64, f64::max));
        }
        assert!(l1_errors[2] < l1_errors[1] && l1_errors[1] < l1_errors[0]);
        assert!(l2_errors[2] < l2_errors[1] && l2_errors[1] < l2_errors[0]);
        assert!(linf_errors[2] < linf_errors[1] && linf_errors[1] < linf_errors[0]);
        let observed_order = (l2_errors[0] / l2_errors[1]).ln() / 2.0_f64.ln();
        assert!(observed_order.is_finite() && observed_order > 0.5);
    }

    #[test]
    fn manufactured_dirichlet_solution_obeys_global_source_flux_balance() {
        let mesh = grid(12, 12);
        let boundary = analytic_boundary(&mesh, |point| point.x * point.x + point.y * point.y);
        let source = CellField::filled(&mesh, -4.0);
        let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        options.source = Some(&source);
        let values = solve(&assemble_poisson(&mesh, options).unwrap());
        let field = CellField::from_values(&mesh, values).unwrap();
        let integrated_flux = integrated_diffusion(
            &mesh,
            &field,
            Diffusivity::Constant(1.0),
            &boundary,
            DiffusionOptions::default(),
        )
        .unwrap();
        let net_divergence: f64 = integrated_flux.values().iter().sum();
        let integrated_source: f64 = source
            .values()
            .iter()
            .zip(mesh.cells())
            .map(|(source, cell)| source * cell.volume)
            .sum();
        assert!((net_divergence + integrated_source).abs() < 1.0e-10);
    }

    #[test]
    fn variable_harmonic_diffusivity_and_cg_cross_check_use_the_same_spd_system() {
        let mesh = two_squares();
        let boundary = boundary(&mesh, ScalarBoundaryCondition::FixedValue(0.0));
        let gamma = CellField::from_values(&mesh, vec![2.0, 6.0]).unwrap();
        let mut options = PoissonOptions::new(Diffusivity::CellField(&gamma), &boundary);
        options.diffusion.diffusivity_interpolation = crate::DiffusivityInterpolation::Harmonic;
        let system = assemble_poisson(&mesh, options).unwrap();
        // For equal centre-to-face weights: Gamma_f = 1/(0.5/2 + 0.5/6) = 3.
        assert_eq!(system.matrix().get(0, 1), Some(-3.0));
        let mut pcg_solution = vec![0.5, -0.25];
        let mut cg_solution = pcg_solution.clone();
        let options = LinearSolverOptions {
            absolute_tolerance: 1.0e-13,
            relative_tolerance: 1.0e-13,
            max_iterations: 100,
        };
        assert!(solve_poisson(
            &system,
            PoissonLinearSolver::Pcg,
            &mut pcg_solution,
            options
        )
        .unwrap()
        .converged());
        assert!(
            solve_poisson(&system, PoissonLinearSolver::Cg, &mut cg_solution, options)
                .unwrap()
                .converged()
        );
        assert!(pcg_solution
            .iter()
            .zip(&cg_solution)
            .all(|(pcg, cg)| (pcg - cg).abs() < 1.0e-12));
    }

    #[test]
    fn three_dimensional_quadratic_problem_is_finite_and_has_bounded_error() {
        let mesh = individually_patched(
            UnstructuredMesh::from_cells(
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
            .unwrap(),
        );
        let boundary = analytic_boundary(&mesh, |point| {
            point.x * point.x + point.y * point.y + point.z * point.z
        });
        let source = CellField::filled(&mesh, -6.0);
        let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        options.source = Some(&source);
        let values = solve(&assemble_poisson(&mesh, options).unwrap());
        assert!(values.iter().all(|value| value.is_finite()));
        assert!((values[0] - 0.75).abs() < 0.3);
    }

    #[test]
    fn explicit_least_squares_correction_changes_only_the_conservative_rhs() {
        let mesh = skewed_grid();
        let analytic = |point: crate::Vec3| 2.0 * point.x - 3.0 * point.y + 5.0;
        let boundary = analytic_boundary(&mesh, analytic);
        let baseline = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        let baseline_system = assemble_poisson(&mesh, baseline).unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let mut corrected = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        corrected.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Explicit;
        corrected.diffusion.gradient_scheme = crate::GradientScheme::LeastSquares;
        corrected.least_squares_stencil = Some(&stencil);
        // The explicit correction is reconstructed from the caller-provided
        // current scalar state, not silently from zeros.
        let current = CellField::from_values(
            &mesh,
            mesh.cells()
                .iter()
                .map(|cell| analytic(cell.center))
                .collect(),
        )
        .unwrap();
        corrected.correction_field = Some(&current);
        let corrected_system = assemble_poisson(&mesh, corrected).unwrap();
        assert_eq!(corrected_system.matrix(), baseline_system.matrix());
        let correction: Vec<_> = corrected_system
            .rhs()
            .iter()
            .zip(baseline_system.rhs())
            .map(|(corrected, baseline)| corrected - baseline)
            .collect();
        assert!(correction.iter().all(|value| value.is_finite()));
        // Internal explicit face terms must cancel pairwise before any solve.
        assert!(correction.iter().sum::<f64>().abs() < 1.0e-12);
    }

    #[test]
    fn explicit_correction_workflow_reassembles_rhs_from_the_current_solution() {
        let mesh = skewed_grid();
        let analytic = |point: crate::Vec3| 2.0 * point.x - 3.0 * point.y + 5.0;
        let boundary = analytic_boundary(&mesh, analytic);
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let mut assembly = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        assembly.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Explicit;
        assembly.diffusion.gradient_scheme = crate::GradientScheme::LeastSquares;
        assembly.least_squares_stencil = Some(&stencil);
        let mut solution: Vec<_> = mesh
            .cells()
            .iter()
            .map(|cell| analytic(cell.center))
            .collect();
        let (system, report) = solve_poisson_correction_sweeps(
            &mesh,
            assembly,
            PoissonLinearSolver::Pcg,
            &mut solution,
            LinearSolverOptions {
                absolute_tolerance: 1.0e-13,
                relative_tolerance: 1.0e-13,
                max_iterations: 10_000,
            },
            1,
        )
        .unwrap();
        assert!(report.converged());
        assert!(solution.iter().all(|value| value.is_finite()));
        assert!(system
            .true_residual(&solution)
            .unwrap()
            .iter()
            .all(|value| value.abs() < 1.0e-10));
    }

    #[test]
    fn skewed_three_dimensional_lsq_poisson_assembly_is_finite_and_conservative() {
        let mesh = skewed_block_3d();
        let analytic = |point: crate::Vec3| 2.0 * point.x - 3.0 * point.y + 4.0 * point.z + 5.0;
        let boundary = analytic_boundary(&mesh, analytic);
        let baseline = assemble_poisson(
            &mesh,
            PoissonOptions::new(Diffusivity::Constant(1.0), &boundary),
        )
        .unwrap();
        let stencil = LeastSquaresGradientStencil::new(&mesh, &boundary).unwrap();
        let current = CellField::from_values(
            &mesh,
            mesh.cells()
                .iter()
                .map(|cell| analytic(cell.center))
                .collect(),
        )
        .unwrap();
        let mut options = PoissonOptions::new(Diffusivity::Constant(1.0), &boundary);
        options.diffusion.non_orthogonal_correction = NonOrthogonalCorrection::Explicit;
        options.diffusion.gradient_scheme = crate::GradientScheme::LeastSquares;
        options.correction_field = Some(&current);
        options.least_squares_stencil = Some(&stencil);
        let corrected = assemble_poisson(&mesh, options).unwrap();
        assert_eq!(corrected.matrix(), baseline.matrix());
        assert!(corrected.rhs().iter().all(|value| value.is_finite()));
        let correction_sum: f64 = corrected
            .rhs()
            .iter()
            .zip(baseline.rhs())
            .map(|(corrected, baseline)| corrected - baseline)
            .sum();
        assert!(correction_sum.abs() < 1.0e-11);
        let values = solve(&corrected);
        assert!(values.iter().all(|value| value.is_finite()));
    }
}
