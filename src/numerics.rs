//! Conservative reference operators for unstructured finite-volume meshes.

use crate::{CellField, FaceField, FieldError, UnstructuredMesh, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScalarBoundaryValue {
    OwnerValue,
    FixedValue(f64),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NonOrthogonalCorrection {
    None,
}
#[derive(Clone, Debug, PartialEq)]
pub enum NumericsError {
    Field(FieldError),
    DegenerateOwnerNeighbourDistance { face: usize },
    InvalidCellMeasure { cell: usize },
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
    use crate::{parse_gmsh, CellDefinition, MeshDimension, Point};
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
}
