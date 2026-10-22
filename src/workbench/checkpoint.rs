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
    #[serde(default)]
    time: Option<f64>,
    #[serde(default)]
    last_dt: Option<f64>,
    #[serde(default)]
    face_flux: Option<Vec<f64>>,
    #[serde(default)]
    turbulence: Option<StoredTurbulence>,
    #[serde(default)]
    temperature: Option<Vec<f64>>,
}
#[derive(Serialize, Deserialize)]
struct StoredTurbulence {
    #[serde(default)]
    model: crate::turbulence::TurbulenceModel,
    k: Vec<f64>,
    omega: Vec<f64>,
    nut: Vec<f64>,
    #[serde(default)]
    epsilon: Option<Vec<f64>>,
    #[serde(default)]
    nu_tilda: Option<Vec<f64>>,
}
/// Complete time-level restart, including conservative flux and RANS state.
pub fn write_solution(
    path: &Path,
    mesh: &MeshSignature,
    iteration: usize,
    solution: &crate::IncompressibleSolution,
    context: &str,
) -> Result<(), String> {
    write_restart(
        path,
        mesh,
        iteration,
        &solution.restart(),
        solution.physical_time.is_some(),
        context,
    )
}
pub fn write_restart(
    path: &Path,
    mesh: &MeshSignature,
    iteration: usize,
    initial: &IncompressibleInitialConditions,
    physical: bool,
    context: &str,
) -> Result<(), String> {
    let IncompressibleInitialConditions::Restart {
        velocity,
        pressure,
        face_flux,
        time,
        last_dt,
        temperature,
        turbulence,
    } = initial
    else {
        return Err("A complete restart state is required".into());
    };
    if let Some(t) = turbulence {
        if let Some(v) = &t.nu_tilda {
            if v.values().len() != mesh.cells.len()
                || v.values().iter().any(|v| !v.is_finite() || *v < 0.0)
            {
                return Err("Invalid SA restart field".into());
            }
        }
        if t.model == crate::turbulence::TurbulenceModel::SpalartAllmaras && t.nu_tilda.is_none() {
            return Err("SA restart requires nu_tilda".into());
        }
    }
    if !time.is_finite()
        || *time < 0.0
        || velocity
            .values()
            .iter()
            .any(|v| ![v.x, v.y, v.z].iter().all(|x| x.is_finite()))
        || pressure
            .values()
            .iter()
            .chain(face_flux.values())
            .any(|x| !x.is_finite())
    {
        return Err("Non-finite complete restart state".into());
    }
    if velocity.values().len() != mesh.cells.len()
        || pressure.values().len() != mesh.cells.len()
        || face_flux.values().len() != mesh.faces.len()
    {
        return Err("Restart field sizes do not match mesh".into());
    }
    if let Some(t) = temperature {
        if t.values().len() != mesh.cells.len()
            || t.values().iter().any(|v| !v.is_finite() || *v <= 0.0)
        {
            return Err("Invalid restart temperature field".into());
        }
    }
    let record = FieldCheckpoint {
        format: 1,
        mesh: mesh.clone(),
        iteration,
        velocity: velocity.values().to_vec(),
        pressure: pressure.values().to_vec(),
        context: context.into(),
        time: physical.then_some(*time),
        last_dt: if physical { *last_dt } else { None },
        face_flux: Some(face_flux.values().to_vec()),
        temperature: temperature.as_ref().map(|t| t.values().to_vec()),
        turbulence: turbulence.as_ref().map(|t| StoredTurbulence {
            model: t.model,
            k: t.k.values().to_vec(),
            omega: t.omega.values().to_vec(),
            nut: t.nut.values().to_vec(),
            epsilon: t.epsilon.as_ref().map(|e| e.values().to_vec()),
            nu_tilda: t.nu_tilda.as_ref().map(|v| v.values().to_vec()),
        }),
    };
    let parent = path.parent().unwrap_or(Path::new("."));
    std::fs::create_dir_all(parent).map_err(|e| e.to_string())?;
    let mut file = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    serde_json::to_writer(&mut file, &record).map_err(|e| e.to_string())?;
    file.flush().map_err(|e| e.to_string())?;
    file.as_file().sync_all().map_err(|e| e.to_string())?;
    file.persist(path).map_err(|e| e.to_string())?;
    Ok(())
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
    let initial = if let Some(phi) = saved.face_flux {
        if phi.iter().any(|v| !v.is_finite())
            || saved.time.is_some_and(|t| !t.is_finite() || t < 0.0)
            || saved.last_dt.is_some_and(|dt| !dt.is_finite() || dt <= 0.0)
        {
            return Err("Checkpoint time or flux is invalid".into());
        }
        let turbulence = saved
            .turbulence
            .map(|t| -> Result<crate::turbulence::TurbulenceFields, String> {
                if t.nu_tilda.as_ref().is_some_and(|v| {
                    v.len() != mesh.cell_count() || v.iter().any(|v| !v.is_finite() || *v < 0.0)
                }) {
                    return Err("Invalid SA checkpoint field".into());
                }
                if t.model == crate::turbulence::TurbulenceModel::SpalartAllmaras
                    && t.nu_tilda.is_none()
                {
                    return Err("SA checkpoint is missing nu_tilda".into());
                }
                if t.epsilon
                    .as_ref()
                    .is_some_and(|e| e.iter().any(|v| !v.is_finite() || *v <= 0.0))
                {
                    return Err("Invalid restart epsilon field".into());
                }
                if t.k
                    .iter()
                    .chain(&t.omega)
                    .chain(&t.nut)
                    .any(|x| !x.is_finite() || *x < 0.0)
                {
                    return Err("Invalid restart turbulence fields".into());
                }
                Ok(crate::turbulence::TurbulenceFields {
                    model: t.model,
                    nu_tilda: t
                        .nu_tilda
                        .map(|v| CellField::from_values(mesh, v).map_err(|e| format!("{e:?}")))
                        .transpose()?,
                    k: CellField::from_values(mesh, t.k).map_err(|e| format!("{e:?}"))?,
                    omega: CellField::from_values(mesh, t.omega).map_err(|e| format!("{e:?}"))?,
                    nut: CellField::from_values(mesh, t.nut).map_err(|e| format!("{e:?}"))?,
                    epsilon: t
                        .epsilon
                        .map(|e| CellField::from_values(mesh, e).map_err(|e| format!("{e:?}")))
                        .transpose()?,
                })
            })
            .transpose()?;
        IncompressibleInitialConditions::Restart {
            velocity,
            pressure,
            face_flux: crate::FaceField::from_values(mesh, phi).map_err(|e| format!("{e:?}"))?,
            time: saved.time.unwrap_or(0.0),
            last_dt: saved.last_dt,
            temperature: saved
                .temperature
                .map(|v| {
                    if v.iter().any(|v| !v.is_finite() || *v <= 0.0) {
                        return Err("Invalid checkpoint temperature".into());
                    }
                    CellField::from_values(mesh, v).map_err(|e| format!("{e:?}"))
                })
                .transpose()?,
            turbulence,
        }
    } else {
        IncompressibleInitialConditions::fields(velocity, pressure)
    };
    Ok((saved.iteration, initial))
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
