//! Immutable, checksummed mesh assets referenced by canonical and recovery documents.
use super::{WorkbenchProject, WorkbenchSession};
use crate::{mesh::MeshSnapshot, GeneratedMesh, GmshMeshingReport};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{io::Write, path::Path};
#[derive(Serialize, Deserialize)]
struct StoredMesh {
    format: u32,
    intent: String,
    mesh: MeshSnapshot,
    report: GmshMeshingReport,
}
pub fn intent(project: &WorkbenchProject) -> Result<String, String> {
    let data = serde_json::json!({"geometry":project.geometry,"fluid_face":project.fluid_face,
        "groups":project.named_selections,"mesh":{"dimension":project.mesh.dimension,"global":project.mesh.global_size,
        "min":project.mesh.min_size,"max":project.mesh.max_size,"order":project.mesh.element_order},"recipe":project.mesh_recipe});
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&data).map_err(|e| e.to_string())?)
    ))
}
pub fn save(
    workspace: &Path,
    project: &WorkbenchProject,
    generated: &GeneratedMesh,
) -> Result<String, String> {
    let record = StoredMesh {
        format: 1,
        intent: intent(project)?,
        mesh: MeshSnapshot::capture(&generated.mesh),
        report: generated.report.clone(),
    };
    let bytes = serde_json::to_vec(&record).map_err(|e| e.to_string())?;
    let asset = format!("{:x}", Sha256::digest(&bytes));
    let folder = workspace.join("meshes");
    std::fs::create_dir_all(&folder).map_err(|e| e.to_string())?;
    let target = folder.join(format!("{asset}.json"));
    if target.exists()
        && std::fs::read(&target)
            .map(|old| old == bytes)
            .unwrap_or(false)
    {
        return Ok(asset);
    }
    let mut temporary = tempfile::NamedTempFile::new_in(&folder).map_err(|e| e.to_string())?;
    temporary
        .write_all(&bytes)
        .and_then(|_| temporary.as_file().sync_all())
        .map_err(|e| e.to_string())?;
    temporary.persist(&target).map_err(|e| e.to_string())?;
    Ok(asset)
}
pub fn restore(
    workspace: &Path,
    project: &WorkbenchProject,
    session: &mut WorkbenchSession,
) -> Result<bool, String> {
    let Some(asset) = &project.mesh_asset else {
        return Ok(false);
    };
    if asset.len() != 64 || !asset.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("Invalid mesh asset ID".into());
    }
    let bytes = std::fs::read(workspace.join("meshes").join(format!("{asset}.json")))
        .map_err(|e| format!("Saved mesh cannot be read: {e}"))?;
    if format!("{:x}", Sha256::digest(&bytes)) != *asset {
        return Err("Saved mesh checksum mismatch".into());
    }
    let stored: StoredMesh = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    if stored.format != 1 || stored.intent != intent(project)? {
        return Err("Saved mesh does not match this project's geometry or meshing controls".into());
    }
    let mesh = stored.mesh.restore()?;
    if mesh.dimension() != project.mesh.dimension
        || mesh.cell_count() != stored.report.cell_count
        || mesh.points().len() != stored.report.node_count
        || mesh.boundary_patches().len() != stored.report.patch_count
    {
        return Err("Saved mesh metadata mismatch".into());
    }
    session.install_mesh(GeneratedMesh {
        mesh,
        report: stored.report,
    });
    Ok(true)
}
