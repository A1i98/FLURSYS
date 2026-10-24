//! Shared GUI/CLI phase/energy archive workflow.
use flursys::{
    multiphysics::{self, energy::Boundary, Options},
    UnstructuredMesh,
};
use std::{fs, io::Write, path::Path};
pub fn execute(
    mesh: &UnstructuredMesh,
    config: &Path,
    output: &Path,
    restart: Option<&Path>,
) -> Result<serde_json::Value, String> {
    let input: Options = serde_json::from_slice(&fs::read(config).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    let (options, provenance) = input.resolve_catalog()?;
    let checkpoint = restart
        .map(multiphysics::checkpoint::Checkpoint::load)
        .transpose()?;
    if let Some(c) = &checkpoint {
        c.validate(mesh, &options)?;
    }
    let mut archive = multiphysics::archive::Archive::create(output, &options, &provenance)?;
    fs::copy(config, output.join("transport.json")).map_err(|e| e.to_string())?;
    let result = multiphysics::run_controlled(
        mesh,
        &options,
        flursys::LinearSolverOptions::default(),
        checkpoint.as_ref(),
        None,
        |state, report| archive.record(mesh, state, report),
    );
    let (status, message) = match &result {
        Ok(_) => (
            "completed",
            "Requested additional physical steps completed".to_string(),
        ),
        Err(e) => ("failed", e.clone()),
    };
    let report = archive.finish(mesh, status, &message)?;
    result?;
    Ok(report)
}
pub fn template(mesh: &UnstructuredMesh, path: &Path) -> Result<serde_json::Value, String> {
    let mut options = Options {
        mesh_signature: Some(multiphysics::mesh_signature(mesh)),
        ..Default::default()
    };
    for patch in mesh.boundary_patches() {
        options
            .energy
            .boundaries
            .insert(patch.name.clone(), Boundary::Adiabatic);
    }
    let mut file = fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .map_err(|e| e.to_string())?;
    file.write_all(
        serde_json::to_string_pretty(&options)
            .map_err(|e| e.to_string())?
            .as_bytes(),
    )
    .map_err(|e| e.to_string())?;
    Ok(
        serde_json::json!({"summary":format!("Quiescent phase/energy template written to {}. Edit regions and boundary temperatures before solving.",path.display()),"cells":mesh.cell_count(),"faces":mesh.face_count(),"mesh_signature":options.mesh_signature}),
    )
}
