use super::{
    metrics::Measures,
    phase::{self, PhaseOptions},
};
use crate::UnstructuredMesh;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SpeciesOptions {
    pub diffusivity: f64,
    pub max_subcycles: usize,
    pub advection: PhaseOptions,
    pub fixed_boundary_fraction: BTreeMap<String, f64>,
}

impl Default for SpeciesOptions {
    fn default() -> Self {
        Self {
            diffusivity: 0.0,
            max_subcycles: 10_000,
            advection: PhaseOptions::default(),
            fixed_boundary_fraction: BTreeMap::new(),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SpeciesConfiguration {
    pub initial_fraction: f64,
    pub use_flow: bool,
    pub options: SpeciesOptions,
}

#[derive(Clone, Debug, Serialize)]
pub struct SpeciesReport {
    pub advection_subcycles: usize,
    pub diffusion_subcycles: usize,
    pub initial_integral: f64,
    pub final_integral: f64,
    pub boundary_outflow: f64,
    pub conservation_error: f64,
}

/// Constant-density passive mass fraction on a prescribed divergence-free flow.
/// Boundaries default to zero diffusive flux; fixed fractions can be specified per patch.
pub fn advance(
    mesh: &UnstructuredMesh,
    measures: &Measures,
    volume_flux: &[f64],
    fraction: &mut [f64],
    dt: f64,
    options: &SpeciesOptions,
) -> Result<SpeciesReport, String> {
    if !options.diffusivity.is_finite() || options.diffusivity < 0.0 || options.max_subcycles == 0 {
        return Err(
            "Species diffusivity must be finite and nonnegative and max_subcycles positive".into(),
        );
    }
    let mut candidate = fraction.to_vec();
    let advection = phase::advance(
        mesh,
        measures,
        volume_flux,
        &mut candidate,
        dt,
        &options.advection,
    )?;
    let mut fixed = vec![None; mesh.face_count()];
    for (name, &value) in &options.fixed_boundary_fraction {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {
            return Err(format!("Invalid species fraction on patch {name}"));
        }
        let patch = mesh
            .boundary_patches()
            .iter()
            .find(|p| &p.name == name)
            .ok_or_else(|| format!("Unknown species patch {name}"))?;
        for &face in &patch.face_indices {
            fixed[face] = Some(value);
        }
    }
    let mut conductance = vec![0.0; mesh.face_count()];
    let mut outgoing = vec![0.0; mesh.cell_count()];
    if options.diffusivity > 0.0 {
        for (i, face) in mesh.faces().iter().enumerate() {
            let destination = match face.neighbour {
                Some(neighbour) => mesh.cells()[neighbour].center,
                None if fixed[i].is_some() => face.center,
                None => continue,
            };
            let d = destination - mesh.cells()[face.owner].center;
            let projection = face.area_vector.dot(d);
            let d2 = d.norm_squared();
            if !projection.is_finite() || projection <= 0.0 || !d2.is_finite() || d2 <= f64::EPSILON
            {
                return Err(format!("Invalid diffusion geometry on face {i}"));
            }
            let factor = options.diffusivity * measures.areas[i] / face.area * projection / d2;
            if !factor.is_finite() || factor < 0.0 {
                return Err(format!("Invalid diffusion coefficient on face {i}"));
            }
            conductance[i] = factor;
            outgoing[face.owner] += factor;
            if let Some(neighbour) = face.neighbour {
                outgoing[neighbour] += factor;
            }
        }
    }
    let limit = outgoing
        .iter()
        .zip(&measures.volumes)
        .map(|(a, v)| dt * a / v)
        .fold(0.0_f64, f64::max);
    let needed = limit.ceil().max(1.0);
    if !needed.is_finite() || needed > options.max_subcycles as f64 {
        return Err(format!(
            "Species diffusion requires {needed} subcycles; reduce dt or increase max_subcycles"
        ));
    }
    let count = needed as usize;
    let subdt = dt / count as f64;
    let mut delta = vec![0.0; mesh.cell_count()];
    let mut diffusion_outflow = 0.0;
    for _ in 0..count {
        crate::compute::checkpoint().map_err(|e| e.to_string())?;
        delta.fill(0.0);
        for (i, face) in mesh.faces().iter().enumerate() {
            if let Some(neighbour) = face.neighbour {
                let transfer =
                    subdt * conductance[i] * (candidate[neighbour] - candidate[face.owner]);
                delta[face.owner] += transfer;
                delta[neighbour] -= transfer;
            } else if let Some(value) = fixed[i] {
                let transfer = subdt * conductance[i] * (value - candidate[face.owner]);
                delta[face.owner] += transfer;
                diffusion_outflow -= transfer;
            }
        }
        for i in 0..candidate.len() {
            candidate[i] += delta[i] / measures.volumes[i];
            if !candidate[i].is_finite() || candidate[i] < -1e-12 || candidate[i] > 1.0 + 1e-12 {
                return Err(format!("Species fraction outside [0,1] in cell {i}"));
            }
        }
    }
    let final_integral = candidate
        .iter()
        .zip(&measures.volumes)
        .map(|(c, v)| c * v)
        .sum::<f64>();
    let report = SpeciesReport {
        advection_subcycles: advection.subcycles,
        diffusion_subcycles: count,
        initial_integral: advection.initial_volume,
        final_integral,
        boundary_outflow: advection.net_boundary_outflow + diffusion_outflow / dt,
        conservation_error: final_integral - advection.initial_volume
            + dt * advection.net_boundary_outflow
            + diffusion_outflow,
    };
    fraction.copy_from_slice(&candidate);
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point};

    fn mesh() -> UnstructuredMesh {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(2., 0., 0.),
                Point::new(0., 1., 0.),
                Point::new(1., 1., 0.),
                Point::new(2., 1., 0.),
            ],
            vec![
                CellDefinition::polygon(vec![0, 1, 4, 3]),
                CellDefinition::polygon(vec![1, 2, 5, 4]),
            ],
        )
        .unwrap();
        let faces = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        mesh.with_boundary_patches(vec![BoundaryPatch {
            name: "wall".into(),
            face_indices: faces,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap()
    }

    #[test]
    fn diffusion_is_bounded_and_conservative_and_failure_is_transactional() {
        let mesh = mesh();
        let measures = crate::multiphysics::metrics::Coordinates::Cartesian
            .measures(&mesh)
            .unwrap();
        let flux = vec![0.; mesh.face_count()];
        let mut fraction = vec![1., 0.];
        let mut options = SpeciesOptions {
            diffusivity: 1.,
            ..Default::default()
        };
        let report = advance(&mesh, &measures, &flux, &mut fraction, 0.25, &options).unwrap();
        assert!((fraction[0] - 0.75).abs() < 1e-12);
        assert!((fraction[1] - 0.25).abs() < 1e-12);
        assert!(report.conservation_error.abs() < 1e-12);
        options.max_subcycles = 1;
        let previous = fraction.clone();
        assert!(advance(&mesh, &measures, &flux, &mut fraction, 3., &options).is_err());
        assert_eq!(fraction, previous);
    }
    #[test]
    fn fixed_boundary_diffusion_accounts_for_incoming_species() {
        let mesh = mesh();
        let measures = crate::multiphysics::metrics::Coordinates::Cartesian
            .measures(&mesh)
            .unwrap();
        let mut fractions = vec![0.0; mesh.cell_count()];
        let mut options = SpeciesOptions {
            diffusivity: 1.0,
            ..Default::default()
        };
        options.fixed_boundary_fraction.insert("wall".into(), 1.0);
        let report = advance(
            &mesh,
            &measures,
            &vec![0.0; mesh.face_count()],
            &mut fractions,
            0.1,
            &options,
        )
        .unwrap();
        assert!(fractions.iter().all(|f| *f > 0.0 && *f <= 1.0));
        assert!(report.boundary_outflow < 0.0);
        assert!(report.conservation_error.abs() < 1e-12);
    }
}
