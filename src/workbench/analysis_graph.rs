use super::{load_workspace, save_workspace, WorkbenchProject};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    io::Write,
    path::{Path, PathBuf},
};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Analysis {
    pub project_id: String,
    pub name: String,
    pub workspace: PathBuf,
    pub geometry_key: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AnalysisGraph {
    pub format: u32,
    pub source_project: String,
    pub analyses: Vec<Analysis>,
}
#[derive(Serialize, Deserialize)]
struct Link {
    format: u32,
    root: PathBuf,
}

pub fn geometry_key(project: &WorkbenchProject) -> Result<String, String> {
    if let Some(key) = &project.cad_geometry_hash {
        return Ok(key.clone());
    }
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&project.geometry).map_err(|e| e.to_string())?)
    ))
}
fn atomic_json(path: &Path, value: &impl Serialize) -> Result<(), String> {
    let parent = path.parent().ok_or("Analysis manifest has no directory")?;
    let mut temp = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
    serde_json::to_writer_pretty(&mut temp, value).map_err(|e| e.to_string())?;
    temp.flush()
        .and_then(|_| temp.as_file().sync_all())
        .map_err(|e| e.to_string())?;
    temp.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
pub fn load(workspace: &Path) -> Result<(PathBuf, AnalysisGraph), String> {
    let link = workspace.join("analysis-link.json");
    let root = if link.exists() {
        let link: Link = serde_json::from_slice(&std::fs::read(link).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        if link.format != 1 || link.root != Path::new("../..") {
            return Err("Unsupported analysis link".into());
        }
        workspace
            .join(link.root)
            .canonicalize()
            .map_err(|e| e.to_string())?
    } else {
        workspace.canonicalize().map_err(|e| e.to_string())?
    };
    let path = root.join("analysis-workbench.json");
    let mut graph = if path.exists() {
        let graph: AnalysisGraph =
            serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        if graph.format != 1
            || graph.analyses.iter().any(|a| {
                a.workspace != Path::new(".")
                    && a.workspace
                        .components()
                        .any(|c| !matches!(c, std::path::Component::Normal(_)))
            })
        {
            return Err("Invalid analysis workbench manifest".into());
        }
        graph
    } else {
        let project = load_workspace(&root).map_err(|e| e.to_string())?;
        AnalysisGraph {
            format: 1,
            source_project: project.project_id.clone(),
            analyses: vec![Analysis {
                project_id: project.project_id.clone(),
                name: project.name.clone(),
                workspace: PathBuf::from("."),
                geometry_key: geometry_key(&project)?,
            }],
        }
    };
    let source = load_workspace(&root).map_err(|e| e.to_string())?;
    if source.project_id != graph.source_project {
        return Err("The workbench source project has changed".into());
    }
    if let Some(entry) = graph
        .analyses
        .iter_mut()
        .find(|a| a.project_id == source.project_id)
    {
        entry.name = source.name.clone();
        entry.geometry_key = geometry_key(&source)?;
    }
    Ok((root, graph))
}

pub fn record_save(workspace: &Path, project: &WorkbenchProject) -> Result<(), String> {
    let (root, mut graph) = load(workspace)?;
    if let Some(entry) = graph
        .analyses
        .iter_mut()
        .find(|a| a.project_id == project.project_id)
    {
        entry.name = project.name.clone();
        entry.geometry_key = geometry_key(project)?;
    } else {
        return Err("This project is not registered in the linked workbench".into());
    }
    atomic_json(&root.join("analysis-workbench.json"), &graph)
}

pub fn branch(workspace: &Path, project: &WorkbenchProject, name: &str) -> Result<PathBuf, String> {
    let name = name.trim();
    if name.is_empty() || name.len() > 200 {
        return Err("Analysis name must contain 1–200 characters".into());
    }
    project.validate().map_err(|e| e.to_string())?;
    let (root, mut graph) = load(workspace)?;
    let source = load_workspace(&root).map_err(|e| e.to_string())?;
    let key = geometry_key(project)?;
    if key != geometry_key(&source)? {
        return Err("This analysis uses a different CAD revision; branch from the source CAD project or save it as a separate workbench".into());
    }
    let directory = root.join("analyses");
    std::fs::create_dir_all(&directory).map_err(|e| e.to_string())?;
    let mut destination = None;
    for i in 1..=10_000 {
        let p = directory.join(format!("analysis-{i:04}"));
        match std::fs::create_dir(&p) {
            Ok(()) => {
                destination = Some(p);
                break;
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {}
            Err(e) => return Err(e.to_string()),
        }
    }
    let destination = destination.ok_or("Analysis directory limit reached")?;
    let result = (|| -> Result<(), String> {
        let mut child = project.clone();
        child.project_id = WorkbenchProject::blank(name).project_id;
        child.name = name.into();
        child.runs.clear();
        if let Some(asset) = &child.mesh_asset {
            let file = format!("{asset}.json");
            let bytes =
                std::fs::read(workspace.join("meshes").join(&file)).map_err(|e| e.to_string())?;
            if format!("{:x}", Sha256::digest(&bytes)) != *asset {
                return Err("The source mesh checksum is invalid".into());
            }
            std::fs::create_dir(destination.join("meshes")).map_err(|e| e.to_string())?;
            let mut target = std::fs::File::create(destination.join("meshes").join(file))
                .map_err(|e| e.to_string())?;
            target
                .write_all(&bytes)
                .and_then(|_| target.sync_all())
                .map_err(|e| e.to_string())?;
        }
        save_workspace(&destination, &child).map_err(|e| e.to_string())?;
        atomic_json(
            &destination.join("analysis-link.json"),
            &Link {
                format: 1,
                root: PathBuf::from("../.."),
            },
        )?;
        if let Some(entry) = graph
            .analyses
            .iter_mut()
            .find(|a| a.project_id == source.project_id)
        {
            entry.name = source.name.clone();
            entry.geometry_key = geometry_key(&source)?;
        }
        graph.analyses.push(Analysis {
            project_id: child.project_id.clone(),
            name: child.name.clone(),
            workspace: destination
                .strip_prefix(&root)
                .map_err(|e| e.to_string())?
                .into(),
            geometry_key: key,
        });
        atomic_json(&root.join("analysis-workbench.json"), &graph)
    })();
    if let Err(e) = result {
        let _ = std::fs::remove_dir_all(&destination);
        return Err(e);
    }
    Ok(destination)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn linked_analyses_keep_independent_settings_and_detect_changed_cad() {
        let directory = tempfile::tempdir().unwrap();
        let mut source = WorkbenchProject::blank("Shared CAD");
        source.cad_geometry_hash = Some("geometry-a".into());
        let mesh = crate::UnstructuredMesh::from_cells(
            crate::MeshDimension::TwoD,
            vec![
                crate::Point::new(0., 0., 0.),
                crate::Point::new(1., 0., 0.),
                crate::Point::new(0., 1., 0.),
            ],
            vec![crate::CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap()
        .with_boundary_patches(vec![crate::BoundaryPatch {
            name: "wall".into(),
            face_indices: vec![0, 1, 2],
            boundary_type: crate::BoundaryType::Wall,
        }])
        .unwrap();
        let generated = crate::GeneratedMesh {
            mesh,
            report: crate::GmshMeshingReport {
                version: crate::GmshVersion {
                    value: "fixture".into(),
                },
                dimension: crate::MeshDimension::TwoD,
                mesh_format: "fixture".into(),
                node_count: 3,
                cell_count: 1,
                patch_count: 1,
                stdout: String::new(),
                stderr: String::new(),
            },
        };
        source.mesh_asset =
            Some(super::super::mesh_storage::save(directory.path(), &source, &generated).unwrap());
        save_workspace(directory.path(), &source).unwrap();
        let first = branch(directory.path(), &source, "Laminar").unwrap();
        let mut child = load_workspace(&first).unwrap();
        assert_ne!(child.project_id, source.project_id);
        assert!(child.runs.is_empty());
        let mut restored = super::super::WorkbenchSession::from_project(&child).unwrap();
        assert!(super::super::mesh_storage::restore(&first, &child, &mut restored).unwrap());
        assert_eq!(restored.mesh().unwrap().mesh.cell_count(), 1);
        child.material.density = 2.;
        save_workspace(&first, &child).unwrap();
        record_save(&first, &child).unwrap();
        let second = branch(&first, &child, "Thermal").unwrap();
        assert_eq!(load_workspace(&second).unwrap().material.density, 2.);
        assert_eq!(
            load_workspace(directory.path()).unwrap().material.density,
            source.material.density
        );
        let (root, graph) = load(&second).unwrap();
        assert_eq!(root, directory.path().canonicalize().unwrap());
        assert_eq!(graph.analyses.len(), 3);
        source.cad_geometry_hash = Some("geometry-b".into());
        save_workspace(directory.path(), &source).unwrap();
        let (_, graph) = load(&first).unwrap();
        assert_ne!(
            graph.analyses[0].geometry_key,
            graph.analyses[1].geometry_key
        );
        assert!(branch(&first, &child, "Stale CAD").is_err());
        assert!(branch(directory.path(), &source, "").is_err());
    }
}
