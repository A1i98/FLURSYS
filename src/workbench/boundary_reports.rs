//! Conservative patch flow integrals from the solved face flux, not cell velocity.
use crate::{IncompressibleSolution, UnstructuredMesh};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoundaryFlowReport {
    pub patch: String,
    pub faces: usize,
    /// Length in 2D; area in 3D.
    pub measure: f64,
    /// Owner-oriented volumetric flux, negative into the fluid.
    pub volume_flux: f64,
    pub mass_flux: f64,
    /// Area-weighted adjacent-cell kinematic pressure, not wall-face pressure.
    pub adjacent_pressure_mean: Option<f64>,
    pub adjacent_speed_mean: Option<f64>,
}

pub fn boundary_flow_reports(
    mesh: &UnstructuredMesh,
    solution: &IncompressibleSolution,
    density: f64,
) -> Vec<BoundaryFlowReport> {
    mesh.boundary_patches()
        .iter()
        .map(|patch| {
            let mut measure = 0.0;
            let mut flux = 0.0;
            let mut pressure = 0.0;
            let mut speed = 0.0;
            for &index in &patch.face_indices {
                let face = &mesh.faces()[index];
                measure += face.area;
                flux += solution.face_flux[index];
                pressure += face.area * solution.pressure[face.owner];
                speed += face.area * solution.velocity[face.owner].norm();
            }
            BoundaryFlowReport {
                patch: patch.name.clone(),
                faces: patch.face_indices.len(),
                measure,
                volume_flux: flux,
                mass_flux: density * flux,
                adjacent_pressure_mean: (measure > 0.0).then(|| pressure / measure),
                adjacent_speed_mean: (measure > 0.0).then(|| speed / measure),
            }
        })
        .collect()
}
