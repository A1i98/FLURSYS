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

/// Face-based global balance and volume-weighted divergence; no transient dt factor.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ConservationReport {
    pub inflow: f64,
    pub outflow: f64,
    pub net_volume_flux: f64,
    pub net_mass_flux: f64,
    /// |net| / max(inflow,outflow); None for exactly closed/zero-throughflow cases.
    pub relative_boundary_imbalance: Option<f64>,
    pub volume_weighted_abs_divergence: f64,
    pub volume_weighted_signed_divergence: f64,
}
impl ConservationReport {
    pub fn summary(&self) -> String {
        format!("CONSERVATION · face fluxes\nInflow {:.6e} · outflow {:.6e}\nNet Q {:+.6e} · net mass {:+.6e}\nBoundary imbalance {}\nVolume-weighted |div U| {:.6e} s^-1 · signed {:+.6e} s^-1\nClosed domains: use local divergence; zero boundary flux alone does not prove convergence.",
            self.inflow,self.outflow,self.net_volume_flux,self.net_mass_flux,
            self.relative_boundary_imbalance.map(|v|format!("{:.6e}%",100.0*v)).unwrap_or("N/A (zero throughflow)".into()),
            self.volume_weighted_abs_divergence,self.volume_weighted_signed_divergence)
    }
}
pub fn conservation_report(
    mesh: &UnstructuredMesh,
    solution: &IncompressibleSolution,
    density: f64,
) -> ConservationReport {
    conservation_from_flux(mesh, &solution.face_flux, density)
}

fn conservation_from_flux(
    mesh: &UnstructuredMesh,
    flux: &crate::FaceField<f64>,
    density: f64,
) -> ConservationReport {
    let mut divergence = vec![0.0; mesh.cell_count()];
    let (mut inflow, mut outflow) = (0.0, 0.0);
    for (i, f) in mesh.faces().iter().enumerate() {
        let q = flux[i];
        divergence[f.owner] += q;
        if let Some(n) = f.neighbour {
            divergence[n] -= q;
        } else if q < 0.0 {
            inflow -= q;
        } else {
            outflow += q;
        }
    }
    let volume = mesh.cells().iter().map(|c| c.volume).sum::<f64>();
    let net = outflow - inflow;
    let scale = inflow.max(outflow);
    ConservationReport {
        inflow,
        outflow,
        net_volume_flux: net,
        net_mass_flux: density * net,
        relative_boundary_imbalance: (scale > 0.0).then(|| net.abs() / scale),
        volume_weighted_abs_divergence: divergence.iter().map(|v| v.abs()).sum::<f64>() / volume,
        volume_weighted_signed_divergence: divergence.iter().sum::<f64>() / volume,
    }
}

#[cfg(test)]
mod conservation_tests {
    use super::*;
    use crate::{CellDefinition, FaceField, MeshDimension, Point};
    #[test]
    fn closed_boundary_does_not_hide_internal_nonconservation() {
        let mesh = UnstructuredMesh::from_cells(
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
        .unwrap();
        let mut values = vec![0.0; mesh.face_count()];
        let internal = mesh
            .faces()
            .iter()
            .position(|f| f.neighbour.is_some())
            .unwrap();
        values[internal] = 1.0;
        let flux = FaceField::from_values(&mesh, values).unwrap();
        let r = conservation_from_flux(&mesh, &flux, 1.0);
        assert_eq!(r.net_volume_flux, 0.0);
        assert_eq!(r.relative_boundary_imbalance, None);
        assert!((r.volume_weighted_abs_divergence - 2.0).abs() < 1e-12);
        assert_eq!(r.volume_weighted_signed_divergence, 0.0);
    }
    fn mesh() -> UnstructuredMesh {
        UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(2., 0., 0.),
                Point::new(0., 1., 0.),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2])],
        )
        .unwrap()
    }
    #[test]
    fn face_balance_handles_closed_domains_and_nonzero_imbalance() {
        let mesh = mesh();
        let flux = FaceField::from_values(&mesh, vec![0., 0., 0.]).unwrap();
        let report = conservation_from_flux(&mesh, &flux, 2.0);
        assert_eq!(report.relative_boundary_imbalance, None);
        assert_eq!(report.volume_weighted_abs_divergence, 0.0);
        let flux = FaceField::from_values(&mesh, vec![-2., 3., 0.]).unwrap();
        let report = conservation_from_flux(&mesh, &flux, 2.0);
        assert_eq!(report.net_mass_flux, 2.0);
        assert!((report.relative_boundary_imbalance.unwrap() - 1.0 / 3.0).abs() < 1e-12);
        assert!((report.volume_weighted_abs_divergence - 1.0).abs() < 1e-12);
    }
}
