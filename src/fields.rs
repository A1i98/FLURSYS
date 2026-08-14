//! Contiguous unstructured finite-volume field storage.

use crate::{Cell, Face, MeshId, UnstructuredMesh};
use std::ops::{Index, IndexMut};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FieldError {
    SizeMismatch { expected: usize, actual: usize },
    MeshMismatch { expected: MeshId, actual: MeshId },
}

#[derive(Clone, Debug)]
pub struct CellField<T> {
    mesh_id: MeshId,
    values: Vec<T>,
}

#[derive(Clone, Debug)]
pub struct FaceField<T> {
    mesh_id: MeshId,
    values: Vec<T>,
}

macro_rules! field_api {
    ($field:ident, $entity:ident, $items:ident, $count:ident, $from:ident) => {
        impl<T> $field<T> {
            pub fn filled(mesh: &UnstructuredMesh, value: T) -> Self
            where
                T: Clone,
            {
                Self {
                    mesh_id: mesh.id(),
                    values: vec![value; mesh.$count()],
                }
            }

            pub fn with_default(mesh: &UnstructuredMesh) -> Self
            where
                T: Default + Clone,
            {
                Self::filled(mesh, T::default())
            }

            pub fn $from(
                mesh: &UnstructuredMesh,
                mut function: impl FnMut(usize, &$entity) -> T,
            ) -> Self {
                let values = mesh
                    .$items()
                    .iter()
                    .enumerate()
                    .map(|(index, entity)| function(index, entity))
                    .collect();
                Self {
                    mesh_id: mesh.id(),
                    values,
                }
            }

            pub fn from_values(
                mesh: &UnstructuredMesh,
                values: Vec<T>,
            ) -> Result<Self, FieldError> {
                if values.len() != mesh.$count() {
                    return Err(FieldError::SizeMismatch {
                        expected: mesh.$count(),
                        actual: values.len(),
                    });
                }
                Ok(Self {
                    mesh_id: mesh.id(),
                    values,
                })
            }

            pub fn mesh_id(&self) -> MeshId {
                self.mesh_id
            }
            pub fn len(&self) -> usize {
                self.values.len()
            }
            pub fn is_empty(&self) -> bool {
                self.values.is_empty()
            }
            pub fn values(&self) -> &[T] {
                &self.values
            }
            pub fn values_mut(&mut self) -> &mut [T] {
                &mut self.values
            }
            pub fn as_slice(&self) -> &[T] {
                &self.values
            }
            pub fn as_mut_slice(&mut self) -> &mut [T] {
                &mut self.values
            }
            pub fn iter(&self) -> std::slice::Iter<'_, T> {
                self.values.iter()
            }
            pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, T> {
                self.values.iter_mut()
            }
            pub fn ensure_mesh(&self, mesh: &UnstructuredMesh) -> Result<(), FieldError> {
                if self.mesh_id == mesh.id() {
                    Ok(())
                } else {
                    Err(FieldError::MeshMismatch {
                        expected: self.mesh_id,
                        actual: mesh.id(),
                    })
                }
            }
            pub fn copy_from(&mut self, source: &Self) -> Result<(), FieldError>
            where
                T: Clone,
            {
                self.ensure_compatible(source)?;
                self.values.clone_from_slice(&source.values);
                Ok(())
            }
            pub fn ensure_compatible(&self, other: &Self) -> Result<(), FieldError> {
                if self.mesh_id != other.mesh_id {
                    return Err(FieldError::MeshMismatch {
                        expected: self.mesh_id,
                        actual: other.mesh_id,
                    });
                }
                if self.values.len() != other.values.len() {
                    return Err(FieldError::SizeMismatch {
                        expected: self.values.len(),
                        actual: other.values.len(),
                    });
                }
                Ok(())
            }
        }
        impl<T: Clone> $field<T> {
            pub fn fill(&mut self, value: T) {
                self.values.fill(value);
            }
        }
        impl<T> Index<usize> for $field<T> {
            type Output = T;
            fn index(&self, index: usize) -> &T {
                &self.values[index]
            }
        }
        impl<T> IndexMut<usize> for $field<T> {
            fn index_mut(&mut self, index: usize) -> &mut T {
                &mut self.values[index]
            }
        }
    };
}

field_api!(CellField, Cell, cells, cell_count, from_cells);
field_api!(FaceField, Face, faces, face_count, from_faces);

impl CellField<f64> {
    pub fn axpy(&mut self, alpha: f64, x: &Self) -> Result<(), FieldError> {
        self.ensure_compatible(x)?;
        for (value, source) in self.values.iter_mut().zip(&x.values) {
            *value += alpha * source;
        }
        Ok(())
    }
}
impl FaceField<f64> {
    pub fn axpy(&mut self, alpha: f64, x: &Self) -> Result<(), FieldError> {
        self.ensure_compatible(x)?;
        for (value, source) in self.values.iter_mut().zip(&x.values) {
            *value += alpha * source;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CellDefinition, MeshDimension, Point, Vec3};
    fn mesh() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 2]),
                CellDefinition::polygon(vec![0, 2, 3]),
            ],
        )
        .unwrap()
    }
    #[test]
    fn cell_field_is_mesh_sized_reusable_and_geometry_independent() {
        let mesh = mesh();
        let mut field = CellField::from_cells(&mesh, |_, cell| cell.center.x + 2.0 * cell.center.y);
        assert_eq!(field.len(), 2);
        assert!((field[0] - 4.0 / 3.0).abs() < 1e-12);
        assert!((field[1] - 5.0 / 3.0).abs() < 1e-12);
        field[1] = 9.0;
        field.fill(0.0);
        assert_eq!(field.as_slice(), &[0.0, 0.0]);
    }
    #[test]
    fn face_field_stores_one_shared_internal_flux_conservatively() {
        let mesh = mesh();
        let mut flux = FaceField::filled(&mesh, 0.0);
        let shared = mesh
            .faces()
            .iter()
            .position(|face| face.neighbour.is_some())
            .unwrap();
        flux[shared] = 3.0;
        let face = &mesh.faces()[shared];
        let mut residual = CellField::filled(&mesh, 0.0);
        residual[face.owner] += flux[shared];
        residual[face.neighbour.unwrap()] -= flux[shared];
        assert_eq!(residual.values().iter().sum::<f64>(), 0.0);
    }
    #[test]
    fn fields_reject_raw_size_and_cross_mesh_mismatches() {
        let first = mesh();
        let second = mesh();
        assert!(matches!(
            CellField::<f64>::from_values(&first, vec![0.0]),
            Err(FieldError::SizeMismatch { .. })
        ));
        let mut a = CellField::filled(&first, 1.0);
        let b = CellField::filled(&second, 2.0);
        assert!(matches!(
            a.axpy(1.0, &b),
            Err(FieldError::MeshMismatch { .. })
        ));
    }
    #[test]
    fn fields_support_vectors_copy_and_axpy() {
        let mesh = mesh();
        let mut velocity = CellField::filled(&mesh, Vec3::ZERO);
        velocity[0] = Vec3::new(1.0, 2.0, 3.0);
        let mut copy = CellField::with_default(&mesh);
        copy.copy_from(&velocity).unwrap();
        assert_eq!(copy[0], velocity[0]);
        let x = FaceField::filled(&mesh, 2.0);
        let mut y = FaceField::filled(&mesh, 1.0);
        y.axpy(3.0, &x).unwrap();
        assert!(y.values().iter().all(|value| *value == 7.0));
    }
}
