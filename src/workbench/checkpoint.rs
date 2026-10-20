//! Atomic field restarts. Linear/coupling history is intentionally reinitialized.
use crate::{CellField, IncompressibleInitialConditions, UnstructuredMesh, Vec3};
use serde::{Deserialize, Serialize};
use std::{io::Write, path::Path};
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MeshSignature {
    dimension: String,
    points: Vec<Vec3>,
    faces: Vec<(Vec<usize>, usize, Option<usize>)>,
    cells: Vec<Vec<usize>>,
}
impl MeshSignature {
    pub fn new(mesh: &UnstructuredMesh) -> Self {
        Self {
            dimension: format!("{:?}", mesh.dimension()),
            points: mesh.points().iter().map(|p| p.position).collect(),
            faces: mesh
                .faces()
                .iter()
                .map(|f| (f.vertices.clone(), f.owner, f.neighbour))
                .collect(),
            cells: mesh.cells().iter().map(|c| c.faces.clone()).collect(),
        }
    }
}
#[derive(Serialize, Deserialize)]
struct FieldCheckpoint {
    format: u32,
    mesh: MeshSignature,
    iteration: usize,
    velocity: Vec<Vec3>,
    pressure: Vec<f64>,
    context: String,
}
pub fn write(
    path: &Path,
    mesh: &MeshSignature,
    iteration: usize,
    velocity: &[Vec3],
    pressure: &[f64],
    context: &str,
) -> Result<(), String> {
    #[derive(Serialize)]
    struct Record<'a> {
        format: u32,
        mesh: &'a MeshSignature,
        iteration: usize,
        velocity: &'a [Vec3],
        pressure: &'a [f64],
        context: &'a str,
    }
    if velocity.len() != mesh.cells.len()
        || pressure.len() != mesh.cells.len()
        || !pressure.iter().all(|v| v.is_finite())
        || !velocity
            .iter()
            .all(|v| [v.x, v.y, v.z].iter().all(|c| c.is_finite()))
    {
        return Err("Checkpoint fields are invalid".into());
    }
    let parent = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let mut f = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    serde_json::to_writer(
        &mut f,
        &Record {
            format: 1,
            mesh,
            iteration,
            velocity,
            pressure,
            context,
        },
    )
    .map_err(|e| e.to_string())?;
    f.flush().map_err(|e| e.to_string())?;
    f.as_file().sync_all().map_err(|e| e.to_string())?;
    f.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
pub fn load(
    path: &Path,
    mesh: &UnstructuredMesh,
) -> Result<(usize, IncompressibleInitialConditions), String> {
    let saved: FieldCheckpoint =
        serde_json::from_reader(std::fs::File::open(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    if saved.format != 1 || saved.mesh != MeshSignature::new(mesh) {
        return Err(
            "Checkpoint geometry, topology or cell ordering does not match the current mesh".into(),
        );
    }
    if !saved.pressure.iter().all(|v| v.is_finite())
        || !saved
            .velocity
            .iter()
            .all(|v| [v.x, v.y, v.z].iter().all(|c| c.is_finite()))
    {
        return Err("Checkpoint contains non-finite fields".into());
    }
    let velocity = CellField::from_values(mesh, saved.velocity).map_err(|e| format!("{e:?}"))?;
    let pressure = CellField::from_values(mesh, saved.pressure).map_err(|e| format!("{e:?}"))?;
    Ok((
        saved.iteration,
        IncompressibleInitialConditions::fields(velocity, pressure),
    ))
}

#[cfg(test)]
mod checkpoint_tests {
    use super::*;
    use crate::{CellDefinition, MeshDimension, Point};
    fn mesh(scale: f64) -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(scale, 0., 0.),
                Point::new(0., 1., 0.),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap()
    }
    #[test]
    fn field_restart_rebinds_fields_to_reloaded_mesh_and_rejects_changed_geometry() {
        let first = mesh(1.0);
        let reloaded = mesh(1.0);
        let folder = tempfile::tempdir().unwrap();
        let path = folder.path().join("checkpoint.json");
        write(
            &path,
            &MeshSignature::new(&first),
            100,
            &[Vec3::new(1.0, 2.0, 3.0)],
            &[-2.5],
            "fixture",
        )
        .unwrap();
        let (iteration, fields) = load(&path, &reloaded).unwrap();
        assert_eq!(iteration, 100);
        match fields {
            IncompressibleInitialConditions::Fields { velocity, pressure } => {
                assert_eq!(velocity.values(), &[Vec3::new(1.0, 2.0, 3.0)]);
                assert_eq!(pressure.values(), &[-2.5]);
            }
            _ => panic!("expected stored fields"),
        }
        assert!(load(&path, &mesh(2.0)).is_err());
    }
    #[test]
    fn atomic_replacement_preserves_last_valid_checkpoint_on_bad_fields() {
        let mesh = mesh(1.0);
        let signature = MeshSignature::new(&mesh);
        let folder = tempfile::tempdir().unwrap();
        let path = folder.path().join("checkpoint.json");
        write(&path, &signature, 1, &[Vec3::ZERO], &[0.0], "fixture").unwrap();
        write(
            &path,
            &signature,
            100,
            &[Vec3::new(1.0, 0.0, 0.0)],
            &[2.0],
            "fixture",
        )
        .unwrap();
        assert!(write(
            &path,
            &signature,
            101,
            &[Vec3::ZERO],
            &[f64::NAN],
            "fixture"
        )
        .is_err());
        assert_eq!(load(&path, &mesh).unwrap().0, 100);
        assert!(write(&path, &signature, 102, &[], &[], "fixture").is_err());
    }
}
