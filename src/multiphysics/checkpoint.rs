//! Portable scalar restart, tied to mesh and frozen physical input.
use super::*;
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Checkpoint {
    pub format_version: u32,
    pub mesh_signature: String,
    pub physics_signature: String,
    pub state: State,
}
impl Checkpoint {
    pub fn capture(
        mesh: &UnstructuredMesh,
        options: &Options,
        state: &State,
    ) -> Result<Self, String> {
        Ok(Self {
            format_version: 1,
            mesh_signature: mesh_signature(mesh),
            physics_signature: physics_signature(mesh, options)?,
            state: state.clone(),
        })
    }
    pub fn validate(&self, mesh: &UnstructuredMesh, options: &Options) -> Result<(), String> {
        if self.format_version != 1
            || self.mesh_signature != mesh_signature(mesh)
            || self.physics_signature != physics_signature(mesh, options)?
        {
            return Err("Scalar restart requires the same mesh and frozen physics; initialize after changing materials, regions, flux or boundaries".into());
        }
        let s = &self.state;
        let n = mesh.cell_count();
        if [
            s.phase_fraction.len(),
            s.temperature.len(),
            s.density.len(),
            s.solid.len(),
        ]
        .iter()
        .any(|&len| len != n)
            || !s.time.is_finite()
            || s.time < 0.0
            || s.phase_fraction
                .iter()
                .any(|a| !a.is_finite() || !(0.0..=1.0).contains(a))
            || s.temperature
                .iter()
                .chain(&s.density)
                .any(|t| !t.is_finite() || *t <= 0.0)
        {
            return Err("Invalid scalar restart field data".into());
        }
        Ok(())
    }
    pub fn load(path: &std::path::Path) -> Result<Self, String> {
        serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())
    }
    pub fn save(&self, path: &std::path::Path) -> Result<(), String> {
        let temp = path.with_extension("json.tmp");
        std::fs::write(&temp, serde_json::to_vec(self).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        std::fs::rename(&temp, path).map_err(|e| e.to_string())
    }
}
pub fn physics_signature(mesh: &UnstructuredMesh, options: &Options) -> Result<String, String> {
    use sha2::{Digest, Sha256};
    let mut frozen = options.clone();
    // Backward Euler restart allows a new dt and additional step/output budget.
    frozen.provenance.clear();
    // Unused GUI defaults on non-inflow patches are not a physics change.
    frozen.phase.incoming.retain(|name, _| {
        mesh.boundary_patches()
            .iter()
            .find(|p| &p.name == name)
            .is_some_and(|patch| {
                patch.face_indices.iter().any(|&face| {
                    options
                        .face_flux
                        .as_ref()
                        .and_then(|f| f.get(face))
                        .is_some_and(|q| *q < 0.0)
                })
            })
    });
    if frozen
        .face_flux
        .as_ref()
        .is_some_and(|f| f.iter().all(|q| *q == 0.0))
    {
        frozen.face_flux = None;
    }
    frozen.mesh_signature = None; // already guarded independently by the checkpoint

    frozen.dt = 1.0;
    frozen.steps = 1;
    frozen.write_every = 1;
    Ok(format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&frozen).map_err(|e| e.to_string())?)
    ))
}
