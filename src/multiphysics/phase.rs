//! Bounded donor-cell phase advection with conservative face fluxes and subcycling.
//! This transports alpha on a prescribed solenoidal volume flux, not momentum.
use super::metrics::Measures;
use crate::UnstructuredMesh;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum AdvectionScheme {
    #[default]
    Upwind,
    FluxCorrected,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PhaseOptions {
    #[serde(skip_serializing_if = "is_upwind")]
    pub scheme: AdvectionScheme,
    pub max_courant: f64,
    pub max_subcycles: usize,
    /// First-phase fraction for every patch that experiences inflow/backflow.
    pub incoming: BTreeMap<String, f64>,
}
fn is_upwind(scheme: &AdvectionScheme) -> bool {
    *scheme == AdvectionScheme::Upwind
}
impl Default for PhaseOptions {
    fn default() -> Self {
        Self {
            scheme: AdvectionScheme::Upwind,
            max_courant: 0.5,
            max_subcycles: 10000,
            incoming: BTreeMap::new(),
        }
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct PhaseReport {
    pub subcycles: usize,
    pub minimum: f64,
    pub maximum: f64,
    pub initial_volume: f64,
    pub final_volume: f64,
    pub net_boundary_outflow: f64,
    pub conservation_error: f64,
    /// Time-averaged phase-one volume flux; use this for mixture enthalpy flux.
    #[serde(skip_serializing)]
    pub phase_flux: Vec<f64>,
}
pub fn mixture_mass_flux(
    phase_flux: &[f64],
    volume_flux: &[f64],
    first_density: f64,
    second_density: f64,
) -> Result<Vec<f64>, String> {
    if phase_flux.len() != volume_flux.len()
        || !first_density.is_finite()
        || first_density <= 0.0
        || !second_density.is_finite()
        || second_density <= 0.0
        || phase_flux.iter().chain(volume_flux).any(|v| !v.is_finite())
    {
        return Err("Invalid phase or volume flux and phase densities".into());
    }
    let mass_flux: Vec<_> = phase_flux
        .iter()
        .zip(volume_flux)
        .map(|(a, q)| first_density * a + second_density * (q - a))
        .collect();
    if mass_flux.iter().any(|v| !v.is_finite()) {
        return Err("Non-finite mixture mass flux".into());
    }
    Ok(mass_flux)
}

pub fn advance(
    mesh: &UnstructuredMesh,
    metrics: &Measures,
    flux: &[f64],
    alpha: &mut [f64],
    dt: f64,
    options: &PhaseOptions,
) -> Result<PhaseReport, String> {
    if alpha.len() != mesh.cell_count()
        || flux.len() != mesh.face_count()
        || metrics.volumes.len() != mesh.cell_count()
        || metrics.areas.len() != mesh.face_count()
        || !dt.is_finite()
        || dt <= 0.0
        || !options.max_courant.is_finite()
        || options.max_courant <= 0.0
        || options.max_courant > 1.0
        || options.max_subcycles == 0
        || alpha
            .iter()
            .any(|a| !a.is_finite() || !(0.0..=1.0).contains(a))
        || flux.iter().any(|q| !q.is_finite())
        || metrics.volumes.iter().any(|v| !v.is_finite() || *v <= 0.0)
        || metrics.areas.iter().any(|v| !v.is_finite() || *v < 0.0)
    {
        return Err("Invalid phase fields, time step or Courant/subcycle limit".into());
    }
    if options
        .incoming
        .values()
        .any(|a| !a.is_finite() || !(0.0..=1.0).contains(a))
        || options
            .incoming
            .keys()
            .any(|name| !mesh.boundary_patches().iter().any(|p| &p.name == name))
    {
        return Err("Incoming phase fractions must be in [0,1] on existing patches".into());
    }
    let mut incoming = vec![None; mesh.face_count()];
    for patch in mesh.boundary_patches() {
        for &f in &patch.face_indices {
            incoming[f] = options.incoming.get(&patch.name).copied();
        }
    }
    let mut outgoing = vec![0.0; mesh.cell_count()];
    let mut divergence = vec![0.0; mesh.cell_count()];
    let mut magnitude = vec![0.0; mesh.cell_count()];
    for (i, f) in mesh.faces().iter().enumerate() {
        let q = flux[i];
        if metrics.areas[i] == 0.0 && q != 0.0 {
            return Err("Nonzero flux through the symmetry axis".into());
        }
        divergence[f.owner] += q;
        magnitude[f.owner] += q.abs();
        outgoing[f.owner] += q.max(0.0);
        if let Some(n) = f.neighbour {
            divergence[n] -= q;
            magnitude[n] += q.abs();
            outgoing[n] += (-q).max(0.0);
        } else if q < 0.0 && incoming[i].is_none() {
            return Err(format!(
                "Assign incoming phase fraction for inflow face {i}"
            ));
        }
    }
    // A non-solenoidal prescribed flux cannot preserve both alpha bounds and
    // phase volume. Reject it rather than clipping away a conservation defect.
    for i in 0..mesh.cell_count() {
        if divergence[i].abs() > 1e-9 * magnitude[i] + 1e-13 * metrics.volumes[i] / dt {
            return Err(format!(
                "Phase transport requires divergence-free volume flux; cell {i}, defect {}",
                divergence[i]
            ));
        }
    }
    let courant = outgoing
        .iter()
        .zip(&metrics.volumes)
        .map(|(q, v)| dt * q / v)
        .fold(0.0, f64::max);
    let count = (courant / options.max_courant).ceil().max(1.0);
    if count > options.max_subcycles as f64 {
        return Err(format!(
            "Phase transport needs {count} subcycles; reduce dt or increase the explicit budget"
        ));
    }
    let count = count as usize;
    let subdt = dt / count as f64;
    let initial = alpha
        .iter()
        .zip(&metrics.volumes)
        .map(|(a, v)| a * v)
        .sum::<f64>();
    let mut candidate = alpha.to_vec();
    let mut mean = vec![0.0; mesh.face_count()];
    let mut delta = vec![0.0; mesh.cell_count()];
    let mut correction = if options.scheme == AdvectionScheme::FluxCorrected {
        vec![0.0; mesh.face_count()]
    } else {
        Vec::new()
    };
    let mut positive = if correction.is_empty() {
        Vec::new()
    } else {
        vec![0.0; mesh.cell_count()]
    };
    let mut negative = positive.clone();
    for _ in 0..count {
        crate::compute::checkpoint().map_err(|e| e.to_string())?;
        delta.fill(0.0);
        for (i, f) in mesh.faces().iter().enumerate() {
            let donor = if flux[i] >= 0.0 {
                candidate[f.owner]
            } else if let Some(n) = f.neighbour {
                candidate[n]
            } else {
                incoming[i].unwrap_or(candidate[f.owner])
            };
            let q = flux[i] * donor;
            mean[i] += q / count as f64;
            delta[f.owner] -= subdt * q;
            if let Some(n) = f.neighbour {
                delta[n] += subdt * q;
            }
        }
        if options.scheme == AdvectionScheme::FluxCorrected {
            positive.fill(0.0);
            negative.fill(0.0);
            for (i, face) in mesh.faces().iter().enumerate() {
                let Some(neighbour) = face.neighbour else {
                    continue;
                };
                let donor = if flux[i] >= 0.0 {
                    candidate[face.owner]
                } else {
                    candidate[neighbour]
                };
                let anti = subdt
                    * flux[i]
                    * (0.5 * (candidate[face.owner] + candidate[neighbour]) - donor);
                correction[i] = anti;
                if anti >= 0.0 {
                    negative[face.owner] += anti;
                    positive[neighbour] += anti;
                } else {
                    positive[face.owner] -= anti;
                    negative[neighbour] -= anti;
                }
            }
            for i in 0..candidate.len() {
                let low = candidate[i] + delta[i] / metrics.volumes[i];
                positive[i] = if positive[i] > 0.0 {
                    ((1.0 - low).max(0.0) * metrics.volumes[i] / positive[i]).min(1.0)
                } else {
                    1.0
                };
                negative[i] = if negative[i] > 0.0 {
                    (low.max(0.0) * metrics.volumes[i] / negative[i]).min(1.0)
                } else {
                    1.0
                };
            }
            for (i, face) in mesh.faces().iter().enumerate() {
                let Some(neighbour) = face.neighbour else {
                    continue;
                };
                let anti = correction[i];
                let limited = anti
                    * if anti >= 0.0 {
                        negative[face.owner].min(positive[neighbour])
                    } else {
                        positive[face.owner].min(negative[neighbour])
                    };
                delta[face.owner] -= limited;
                delta[neighbour] += limited;
                mean[i] += limited / dt;
            }
        }
        for i in 0..candidate.len() {
            candidate[i] += delta[i] / metrics.volumes[i];
            if !candidate[i].is_finite() || candidate[i] < -1e-10 || candidate[i] > 1.0 + 1e-10 {
                return Err(format!(
                    "Unbounded phase fraction in cell {i}; no clipping applied"
                ));
            }
            // Roundoff-sized excursions only; recorded in conservation_error.
            candidate[i] = candidate[i].clamp(0.0, 1.0);
        }
    }
    let final_volume = candidate
        .iter()
        .zip(&metrics.volumes)
        .map(|(a, v)| a * v)
        .sum::<f64>();
    let boundary = mesh
        .faces()
        .iter()
        .enumerate()
        .filter(|(_, f)| f.neighbour.is_none())
        .map(|(i, _)| mean[i])
        .sum::<f64>();
    let report = PhaseReport {
        subcycles: count,
        minimum: candidate.iter().copied().fold(f64::INFINITY, f64::min),
        maximum: candidate.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        initial_volume: initial,
        final_volume,
        net_boundary_outflow: boundary,
        conservation_error: final_volume - initial + dt * boundary,
        phase_flux: mean,
    };
    alpha.copy_from_slice(&candidate);
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point};

    #[test]
    fn corrected_transport_is_bounded_conservative_and_reports_actual_face_flux() {
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
        let exterior = mesh
            .faces()
            .iter()
            .enumerate()
            .filter_map(|(i, f)| f.neighbour.is_none().then_some(i))
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![BoundaryPatch {
                name: "boundary".into(),
                face_indices: exterior,
                boundary_type: BoundaryType::ZeroGradient,
            }])
            .unwrap();
        let measures = crate::multiphysics::metrics::Coordinates::Cartesian
            .measures(&mesh)
            .unwrap();
        let flux: Vec<_> = mesh.faces().iter().map(|f| f.area_vector.x).collect();
        let mut options = PhaseOptions {
            scheme: AdvectionScheme::FluxCorrected,
            ..Default::default()
        };
        options.incoming.insert("boundary".into(), 1.0);
        let original = vec![1.0, 0.0];
        for dt in [0.2, 1.1] {
            let mut values = original.clone();
            let report = advance(&mesh, &measures, &flux, &mut values, dt, &options).unwrap();
            assert!(values.iter().all(|x| (0.0..=1.0).contains(x)));
            assert!(report.conservation_error.abs() < 1e-12);
            let mut balance = vec![0.0; mesh.cell_count()];
            for (i, face) in mesh.faces().iter().enumerate() {
                balance[face.owner] -= dt * report.phase_flux[i];
                if let Some(n) = face.neighbour {
                    balance[n] += dt * report.phase_flux[i];
                }
            }
            for i in 0..values.len() {
                assert!((values[i] - original[i] - balance[i] / measures.volumes[i]).abs() < 1e-12);
            }
        }
        let mut constant = vec![1.0; 2];
        advance(&mesh, &measures, &flux, &mut constant, 0.4, &options).unwrap();
        assert_eq!(constant, vec![1.0; 2]);
        assert_eq!(
            serde_json::from_str::<PhaseOptions>("{}").unwrap().scheme,
            AdvectionScheme::Upwind
        );
    }
}
