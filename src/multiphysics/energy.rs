//! Conservative sensible-energy equation on a conformal solid/fluid mesh.
//! Orthogonal two-point conduction, implicit upwind advection, backward Euler.
use super::metrics::Measures;
use crate::{CsrBuilder, LinearSolverOptions, LinearSolverStatus, UnstructuredMesh};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
const SIGMA: f64 = 5.670_374_419e-8;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "kebab-case")]
pub enum Boundary {
    Adiabatic,
    FixedTemperature {
        temperature: f64,
    },
    InletOutlet {
        incoming_temperature: f64,
    },
    HeatFlux {
        inward_flux: f64,
    },
    Convection {
        coefficient: f64,
        ambient_temperature: f64,
    },
    /// Diffuse grey exchange with a prescribed large isothermal enclosure.
    /// Not an RTE/participating-media or view-factor radiation model.
    Radiation {
        emissivity: f64,
        ambient_temperature: f64,
    },
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct EnergyOptions {
    pub boundaries: BTreeMap<String, Boundary>,
    /// Area-specific contact resistance, m² K/W, keyed by internal face index.
    pub contact_resistance: BTreeMap<usize, f64>,
    pub max_nonlinear_iterations: usize,
    pub relative_tolerance: f64,
}
impl Default for EnergyOptions {
    fn default() -> Self {
        Self {
            boundaries: BTreeMap::new(),
            contact_resistance: BTreeMap::new(),
            max_nonlinear_iterations: 40,
            relative_tolerance: 1e-9,
        }
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct EnergyReport {
    pub nonlinear_iterations: usize,
    pub linear_iterations: usize,
    pub initial_energy: f64,
    pub final_energy: f64,
    pub net_boundary_power_out: f64,
    pub source_power: f64,
    pub balance_error: f64,
    pub minimum_temperature: f64,
    pub maximum_temperature: f64,
}
/// Capacity is rho*Cp [J/(m³ K)], conductivity W/(m K). Face capacity_flux
/// is time-averaged rho*Cp*volume_flux [W/K], not a mass flux or velocity.
/// New capacity and flux must obey the same discrete phase continuity equation.
#[allow(clippy::too_many_arguments)]
pub fn advance(
    mesh: &UnstructuredMesh,
    metrics: &Measures,
    old_capacity: &[f64],
    capacity: &[f64],
    conductivity: &[f64],
    capacity_flux: &[f64],
    source: &[f64],
    solid: &[bool],
    temperature: &mut [f64],
    dt: f64,
    options: &EnergyOptions,
    linear: LinearSolverOptions,
) -> Result<EnergyReport, String> {
    let n = mesh.cell_count();
    let nf = mesh.face_count();
    if [
        old_capacity.len(),
        capacity.len(),
        conductivity.len(),
        source.len(),
        solid.len(),
        temperature.len(),
        metrics.volumes.len(),
    ]
    .iter()
    .any(|&len| len != n)
        || capacity_flux.len() != nf
        || metrics.areas.len() != nf
        || !dt.is_finite()
        || dt <= 0.0
        || options.max_nonlinear_iterations == 0
        || !options.relative_tolerance.is_finite()
        || options.relative_tolerance <= 0.0
        || old_capacity
            .iter()
            .chain(capacity)
            .chain(conductivity)
            .chain(temperature.iter())
            .any(|v| !v.is_finite() || *v <= 0.0)
        || source.iter().chain(capacity_flux).any(|v| !v.is_finite())
        || metrics.volumes.iter().any(|v| !v.is_finite() || *v <= 0.0)
        || metrics.areas.iter().any(|v| !v.is_finite() || *v < 0.0)
    {
        return Err(
            "Invalid energy fields, material properties, time step or solver controls".into(),
        );
    }
    for (&f, &r) in &options.contact_resistance {
        if f >= nf || mesh.faces()[f].neighbour.is_none() || !r.is_finite() || r < 0.0 {
            return Err(
                "Contact resistance must refer to an internal face and be finite >= 0".into(),
            );
        }
    }
    let mut bc = vec![None; nf];
    for patch in mesh.boundary_patches() {
        let value = *options
            .boundaries
            .get(&patch.name)
            .ok_or_else(|| format!("Assign thermal condition to patch {:?}", patch.name))?;
        let valid = match value {
            Boundary::Adiabatic => true,
            Boundary::FixedTemperature { temperature: t }
            | Boundary::InletOutlet {
                incoming_temperature: t,
            } => t.is_finite() && t > 0.0,
            Boundary::HeatFlux { inward_flux: q } => q.is_finite(),
            Boundary::Convection {
                coefficient: h,
                ambient_temperature: t,
            } => h.is_finite() && h > 0.0 && t.is_finite() && t > 0.0,
            Boundary::Radiation {
                emissivity: e,
                ambient_temperature: t,
            } => e.is_finite() && (0.0..=1.0).contains(&e) && t.is_finite() && t > 0.0,
        };
        if !valid {
            return Err(format!("Invalid energy boundary on {:?}", patch.name));
        }
        for &f in &patch.face_indices {
            bc[f] = Some(value);
        }
    }
    if options
        .boundaries
        .keys()
        .any(|name| !mesh.boundary_patches().iter().any(|p| &p.name == name))
    {
        return Err("Energy boundary names a missing patch".into());
    }
    let mut conductance = vec![0.0; nf];
    let mut balance = vec![0.0; n];
    let mut flux_magnitude = vec![0.0; n];
    for (i, f) in mesh.faces().iter().enumerate() {
        let q = capacity_flux[i];
        if (solid[f.owner] || f.neighbour.is_some_and(|j| solid[j]) || metrics.areas[i] == 0.0)
            && q != 0.0
        {
            return Err(format!(
                "Nonzero advective energy flux at solid/axis face {i}"
            ));
        }
        balance[f.owner] += q;
        flux_magnitude[f.owner] += q.abs();
        if let Some(j) = f.neighbour {
            balance[j] -= q;
            flux_magnitude[j] += q.abs();
        }
        if metrics.areas[i] == 0.0 {
            continue;
        }
        let owner_distance = (f.center - mesh.cells()[f.owner].center).dot(f.normal);
        if owner_distance <= 0.0 {
            return Err(format!("Nonpositive normal thermal distance at face {i}"));
        }
        let resistance = owner_distance / conductivity[f.owner]
            + if let Some(j) = f.neighbour {
                let distance = (mesh.cells()[j].center - f.center).dot(f.normal);
                if distance <= 0.0 {
                    return Err(format!(
                        "Nonpositive neighbour thermal distance at face {i}"
                    ));
                }
                distance / conductivity[j]
                    + options.contact_resistance.get(&i).copied().unwrap_or(0.0)
            } else {
                0.0
            };
        conductance[i] = metrics.areas[i] / resistance;
    }
    // This is essential for constant-temperature preservation with moving phases.
    for i in 0..n {
        let defect = (capacity[i] - old_capacity[i]) * metrics.volumes[i] / dt + balance[i];
        let scale = old_capacity[i] * metrics.volumes[i] / dt + flux_magnitude[i];
        if defect.abs() > 1e-8 * scale {
            return Err(format!(
                "Energy capacity and advective flux are inconsistent in cell {i}: {defect}"
            ));
        }
    }
    let old = temperature.to_vec();
    let mut candidate = old.clone();
    let mut linear_iterations = 0;
    let has_radiation = bc
        .iter()
        .flatten()
        .any(|b| matches!(b, Boundary::Radiation { .. }));
    for iteration in 1..=options.max_nonlinear_iterations {
        crate::compute::checkpoint().map_err(|e| e.to_string())?;
        let mut a = CsrBuilder::with_capacity(n, n, n + 2 * nf);
        let mut rhs = vec![0.0; n];
        let mut add = |i, j, v| a.add(i, j, v).map_err(|e| e.to_string());
        for i in 0..n {
            let d = capacity[i] * metrics.volumes[i] / dt;
            add(i, i, d)?;
            rhs[i] =
                old_capacity[i] * metrics.volumes[i] * old[i] / dt + source[i] * metrics.volumes[i];
        }
        for (i, f) in mesh.faces().iter().enumerate() {
            let o = f.owner;
            let g = conductance[i];
            let q = capacity_flux[i];
            if let Some(j) = f.neighbour {
                add(o, o, g)?;
                add(o, j, -g)?;
                add(j, j, g)?;
                add(j, o, -g)?;
                if q >= 0.0 {
                    add(o, o, q)?;
                    add(j, o, -q)?;
                } else {
                    add(o, j, q)?;
                    add(j, j, -q)?;
                }
            } else {
                let boundary =
                    bc[i].ok_or_else(|| format!("Ungrouped exterior energy face {i}"))?;
                if q >= 0.0 {
                    add(o, o, q)?;
                } else {
                    let t=match boundary {Boundary::FixedTemperature{temperature:t}|Boundary::InletOutlet{incoming_temperature:t}=>t,_=>return Err(format!("Incoming energy flux at face {i} needs a prescribed incoming temperature"))};
                    rhs[o] -= q * t;
                }
                match boundary {
                    Boundary::FixedTemperature { temperature: t } => {
                        add(o, o, g)?;
                        rhs[o] += g * t;
                    }
                    Boundary::HeatFlux { inward_flux: q } => rhs[o] += q * metrics.areas[i],
                    Boundary::Convection {
                        coefficient: h,
                        ambient_temperature: t,
                    } => {
                        let c = 1.0 / (1.0 / g + 1.0 / (h * metrics.areas[i]));
                        if metrics.areas[i] > 0.0 {
                            add(o, o, c)?;
                            rhs[o] += c * t;
                        }
                    }
                    Boundary::Radiation {
                        emissivity: e,
                        ambient_temperature: t,
                    } => {
                        // Tangent of epsilon*sigma*(T^4-Ta^4), with the wall
                        // temperature eliminated through the half-cell resistance.
                        if g > 0.0 && e > 0.0 {
                            let tw = wall_temperature(candidate[o], t, g / metrics.areas[i], e)?;
                            let h = 4.0 * e * SIGMA * tw.powi(3);
                            let c = h * metrics.areas[i] / (1.0 + h * metrics.areas[i] / g);
                            let intercept =
                                e * SIGMA * (3.0 * tw.powi(4) + t.powi(4)) * metrics.areas[i]
                                    / (1.0 + h * metrics.areas[i] / g);
                            add(o, o, c)?;
                            rhs[o] += intercept;
                        }
                    }
                    _ => {}
                }
            }
        }
        drop(add);
        let matrix = a.finalize().map_err(|e| e.to_string())?;
        let previous = candidate.clone();
        let preconditioner =
            crate::JacobiPreconditioner::new(&matrix).map_err(|e| e.to_string())?;
        let report = crate::bicgstab_with_preconditioner(
            &matrix,
            &rhs,
            &mut candidate,
            &preconditioner,
            linear,
        )
        .map_err(|e| e.to_string())?;
        linear_iterations += report.iterations;
        if report.status != LinearSolverStatus::Converged {
            return Err("Sensible-energy linear solve did not converge".into());
        }
        if candidate.iter().any(|t| !t.is_finite() || *t <= 0.0) {
            return Err("Nonphysical Kelvin temperature; state not committed".into());
        }
        let change = previous
            .iter()
            .zip(&candidate)
            .map(|(a, b)| (b - a).abs() / b.abs().max(1.0))
            .fold(0.0, f64::max);
        if !has_radiation || change <= options.relative_tolerance {
            let mut boundary_power = 0.0;
            for (i, f) in mesh
                .faces()
                .iter()
                .enumerate()
                .filter(|(_, f)| f.neighbour.is_none())
            {
                let t = candidate[f.owner];
                let q = capacity_flux[i];
                let g = conductance[i];
                let b = bc[i].unwrap();
                let incoming = match b {
                    Boundary::FixedTemperature { temperature: v }
                    | Boundary::InletOutlet {
                        incoming_temperature: v,
                    } => v,
                    _ => t,
                };
                boundary_power += q * if q >= 0.0 { t } else { incoming };
                boundary_power += match b {
                    Boundary::FixedTemperature {
                        temperature: ambient,
                    } => g * (t - ambient),
                    Boundary::HeatFlux { inward_flux: q } => -q * metrics.areas[i],
                    Boundary::Convection {
                        coefficient: h,
                        ambient_temperature: ambient,
                    } if g > 0.0 => (t - ambient) / (1.0 / g + 1.0 / (h * metrics.areas[i])),
                    Boundary::Radiation {
                        emissivity: e,
                        ambient_temperature: ambient,
                    } if g > 0.0 && e > 0.0 => {
                        let tw = wall_temperature(t, ambient, g / metrics.areas[i], e)?;
                        e * SIGMA * (tw.powi(4) - ambient.powi(4)) * metrics.areas[i]
                    }
                    _ => 0.0,
                };
            }
            let initial = old
                .iter()
                .enumerate()
                .map(|(i, t)| t * old_capacity[i] * metrics.volumes[i])
                .sum::<f64>();
            let final_energy = candidate
                .iter()
                .enumerate()
                .map(|(i, t)| t * capacity[i] * metrics.volumes[i])
                .sum::<f64>();
            let power = source
                .iter()
                .zip(&metrics.volumes)
                .map(|(s, v)| s * v)
                .sum::<f64>();
            let report = EnergyReport {
                nonlinear_iterations: iteration,
                linear_iterations,
                initial_energy: initial,
                final_energy,
                net_boundary_power_out: boundary_power,
                source_power: power,
                balance_error: final_energy - initial + dt * (boundary_power - power),
                minimum_temperature: candidate.iter().copied().fold(f64::INFINITY, f64::min),
                maximum_temperature: candidate.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            };
            temperature.copy_from_slice(&candidate);
            return Ok(report);
        }
    }
    Err("Radiative sensible-energy coupling did not converge; state not committed".into())
}
fn wall_temperature(
    cell: f64,
    ambient: f64,
    conductance: f64,
    emissivity: f64,
) -> Result<f64, String> {
    // Monotone half-cell conduction = enclosure radiation. Bracket remains
    // between two positive temperatures; bisection avoids a negative Newton T.
    let mut lo = cell.min(ambient);
    let mut hi = cell.max(ambient);
    for _ in 0..80 {
        let t = 0.5 * (lo + hi);
        let defect = conductance * (t - cell) + emissivity * SIGMA * (t.powi(4) - ambient.powi(4));
        if !defect.is_finite() {
            return Err("Radiative wall equation overflow".into());
        }
        if defect > 0.0 {
            hi = t
        } else {
            lo = t
        }
    }
    Ok(0.5 * (lo + hi))
}

/// Eliminated radiative wall law q_out = slope*T_cell - intercept [W/m²].
/// Used by the Workbench energy equation as one nonlinear outer update.
pub(crate) fn radiative_linearization(
    cell: f64,
    ambient: f64,
    normal_conductance: f64,
    emissivity: f64,
) -> Result<(f64, f64), String> {
    if !cell.is_finite()
        || cell <= 0.0
        || !ambient.is_finite()
        || ambient <= 0.0
        || !normal_conductance.is_finite()
        || normal_conductance <= 0.0
        || !emissivity.is_finite()
        || !(0.0..=1.0).contains(&emissivity)
    {
        return Err("Invalid radiative wall state".into());
    }
    if emissivity == 0.0 {
        return Ok((0.0, 0.0));
    }
    let tw = wall_temperature(cell, ambient, normal_conductance, emissivity)?;
    let h = 4.0 * emissivity * SIGMA * tw.powi(3);
    let denominator = 1.0 + h / normal_conductance;
    let slope = h / denominator;
    let intercept = emissivity * SIGMA * (3.0 * tw.powi(4) + ambient.powi(4)) / denominator;
    if !slope.is_finite() || !intercept.is_finite() {
        return Err("Radiative wall linearization overflow".into());
    }
    Ok((slope, intercept))
}
