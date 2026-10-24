//! Independent whole-domain surface reconstruction, not an algebraic residual.
use crate::*;
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Report {
    pub momentum: Option<Momentum>,
    pub energy: Option<Energy>,
    pub notes: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Momentum {
    pub storage: Vec3,
    pub outward_advection: Vec3,
    pub outward_pressure: Vec3,
    pub inward_viscous: Vec3,
    pub body_force: Vec3,
    pub imbalance: Vec3,
    pub relative_imbalance: Option<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Energy {
    pub storage_w: f64,
    pub outward_advection_w: f64,
    pub inward_heat_w: f64,
    pub source_w: f64,
    pub imbalance_w: f64,
    pub relative_imbalance: Option<f64>,
}
impl Report {
    pub fn summary(&self) -> String {
        let mut s = "GLOBAL SURFACE BUDGETS · 2D quantities per unit depth\n".to_string();
        if let Some(m) = &self.momentum {
            s.push_str(&format!("Momentum imbalance (N): ({:.5e}, {:.5e}, {:.5e}) · relative {:?}\nStorage {:?}\nOutward advection {:?}\nOutward pressure {:?}\nInward viscous {:?}\nBody force {:?}\n",m.imbalance.x,m.imbalance.y,m.imbalance.z,m.relative_imbalance,m.storage,m.outward_advection,m.outward_pressure,m.inward_viscous,m.body_force));
        }
        if let Some(e) = &self.energy {
            s.push_str(&format!("Sensible-energy imbalance {:.6e} W · relative {:?}\nStorage {:.6e}, outward advection {:.6e}, inward heat {:.6e}, source {:.6e} W\n",e.imbalance_w,e.relative_imbalance,e.storage_w,e.outward_advection_w,e.inward_heat_w,e.source_w));
        }
        for note in &self.notes {
            s.push_str(note);
            s.push('\n');
        }
        s
    }
}
fn initial_velocity(initial: &IncompressibleInitialConditions, cell: usize) -> Vec3 {
    match initial {
        IncompressibleInitialConditions::Uniform { velocity, .. } => *velocity,
        IncompressibleInitialConditions::Fields { velocity, .. }
        | IncompressibleInitialConditions::Restart { velocity, .. } => velocity[cell],
    }
}
fn initial_temperature(
    initial: &IncompressibleInitialConditions,
    cell: usize,
    default: f64,
) -> f64 {
    match initial {
        IncompressibleInitialConditions::Restart {
            temperature: Some(t),
            ..
        } => t[cell],
        _ => default,
    }
}
/// `history` uses the solver's effective dt and normalized Euler/BDF2 weights.
pub fn evaluate(
    case: &IncompressibleCase,
    solution: &IncompressibleSolution,
    history: Option<(
        &IncompressibleInitialConditions,
        Option<&IncompressibleInitialConditions>,
        f64,
        [f64; 2],
    )>,
) -> Result<Report, String> {
    let mesh = &case.mesh;
    let rho = case.material.density;
    solution
        .velocity
        .ensure_mesh(mesh)
        .map_err(|e| format!("{e:?}"))?;
    solution
        .pressure
        .ensure_mesh(mesh)
        .map_err(|e| format!("{e:?}"))?;
    solution
        .face_flux
        .ensure_mesh(mesh)
        .map_err(|e| format!("{e:?}"))?;
    if let Some(t) = &solution.temperature {
        t.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
    }
    if let Some((old, older, dt, weights)) = history {
        if !dt.is_finite() || dt <= 0.0 || weights.iter().any(|w| !w.is_finite()) {
            return Err("Invalid temporal budget weights".into());
        }
        for initial in std::iter::once(old).chain(older) {
            match initial {
                IncompressibleInitialConditions::Uniform { .. } => {}
                IncompressibleInitialConditions::Fields { velocity, pressure }
                | IncompressibleInitialConditions::Restart {
                    velocity, pressure, ..
                } => {
                    velocity.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
                    pressure.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
                }
            }
            if let IncompressibleInitialConditions::Restart {
                temperature: Some(t),
                ..
            } = initial
            {
                t.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
            }
        }
    }

    let mut result=Report {momentum:None,energy:None,notes:vec!["Reconstructed surface integrals: adjacent-cell pressure, first-cell normal velocity differences and upwind face advection; differences from deferred solver fluxes are included. This is not the continuity residual.".into()]};
    if !case.solver.cyclic_pairs.is_empty()
        || case.solver.turbulence.model != crate::turbulence::TurbulenceModel::Laminar
    {
        result.notes.push("Momentum/energy budgets currently unavailable for cyclic or turbulence models; conservative mass reports remain available.".into());
        return Ok(result);
    }
    let resolved = case.resolve_boundaries().map_err(|e| format!("{e:?}"))?;
    let components = [
        MomentumComponent::X,
        MomentumComponent::Y,
        MomentumComponent::Z,
    ];
    let mut m = Momentum {
        storage: Vec3::ZERO,
        outward_advection: Vec3::ZERO,
        outward_pressure: Vec3::ZERO,
        inward_viscous: Vec3::ZERO,
        body_force: Vec3::ZERO,
        imbalance: Vec3::ZERO,
        relative_imbalance: None,
    };
    let thermal = &case.solver.thermal;
    let mut e = Energy {
        storage_w: 0.,
        outward_advection_w: 0.,
        inward_heat_w: 0.,
        source_w: 0.,
        imbalance_w: 0.,
        relative_imbalance: None,
    };
    let mut patch_by_face = vec![None; mesh.face_count()];
    for patch in mesh.boundary_patches() {
        for &i in &patch.face_indices {
            patch_by_face[i] = Some(patch);
        }
    }
    let models = case
        .solver
        .volume_models
        .coefficients(mesh, &solution.velocity, case.material.kinematic_viscosity)
        .map_err(|e| format!("{e:?}"))?;
    for (i, c) in mesh.cells().iter().enumerate() {
        let mut acceleration = models.1[i] - solution.velocity[i] * models.0[i];
        if thermal.enabled && thermal.buoyancy {
            let t = solution
                .temperature
                .as_ref()
                .ok_or("Missing temperature for buoyancy budget")?[i];
            acceleration += thermal.gravity + thermal.relative_buoyancy_acceleration(t)?;
        }
        m.body_force += acceleration * (rho * c.volume);
        if thermal.enabled {
            e.source_w += thermal.source_w_m3 * c.volume;
        }
        if let Some((old, older, dt, weights)) = history {
            let old_u = initial_velocity(old, i) * weights[0]
                + older
                    .map(|v| initial_velocity(v, i) * weights[1])
                    .unwrap_or(Vec3::ZERO);
            m.storage += (solution.velocity[i] - old_u) * (rho * c.volume / dt);
            if let Some(t) = solution.temperature.as_ref() {
                let old_t = initial_temperature(old, i, thermal.initial_temperature) * weights[0]
                    + older
                        .map(|v| {
                            initial_temperature(v, i, thermal.initial_temperature) * weights[1]
                        })
                        .unwrap_or(0.);
                e.storage_w += (t[i] - old_t) * rho * thermal.specific_heat * c.volume / dt;
            }
        }
    }
    for (index, f) in mesh
        .faces()
        .iter()
        .enumerate()
        .filter(|(_, f)| f.neighbour.is_none())
    {
        let i = f.owner;
        let normal = f.area_vector / f.area;
        let distance = (f.center - mesh.cells()[i].center).dot(normal).abs();
        if !distance.is_finite() || distance <= 0. {
            return Err("Invalid boundary distance in surface budget".into());
        }
        let mut ub = [
            solution.velocity[i].x,
            solution.velocity[i].y,
            solution.velocity[i].z,
        ];
        let mut visc = [0.; 3];
        for k in 0..3 {
            if let Some(ScalarBoundaryCondition::FixedValue(v)) = resolved
                .velocity_boundary()
                .component(components[k])
                .condition(index)
            {
                let uc = ub[k];
                ub[k] = v;
                visc[k] = (v - uc) / distance * f.area;
            } else {
                visc[k] = 0.0; // prescribed zero normal gradient
            }
        }
        let ub = Vec3::new(ub[0], ub[1], ub[2]);
        let q = solution.face_flux[index];
        let uf = if q < 0. { ub } else { solution.velocity[i] };
        m.outward_advection += uf * (rho * q);
        let p = match resolved.pressure_boundary().condition(index) {
            Some(ScalarBoundaryCondition::FixedValue(p)) => p,
            _ => solution.pressure[i],
        };
        m.outward_pressure += f.area_vector * (rho * p);
        m.inward_viscous +=
            Vec3::new(visc[0], visc[1], visc[2]) * (rho * case.material.kinematic_viscosity);
        if let Some(t) = solution.temperature.as_ref().filter(|_| thermal.enabled) {
            let patch = patch_by_face[index].ok_or("Unassigned surface budget face")?;
            let bc = thermal
                .boundaries
                .get(&patch.name)
                .ok_or("Missing thermal boundary")?;
            let scripted = case
                .solver
                .scripting
                .bindings
                .get(&patch.name)
                .into_iter()
                .flatten()
                .find(|b| b.quantity == crate::scripting::Quantity::Temperature)
                .map(|b| {
                    case.solver
                        .scripting
                        .compiled(b)
                        .and_then(|s| s.value(b, f.center, case.solver.scripting.evaluation_time))
                        .map(|v| v.x)
                })
                .transpose()?;
            use crate::thermal::Boundary;
            let mut incoming = t[i];
            let heat = match *bc {
                Boundary::Adiabatic => 0.,
                Boundary::FixedTemperature { temperature } => {
                    incoming = scripted.unwrap_or(temperature);
                    thermal.conductivity * (incoming - t[i]) / distance * f.area
                }
                Boundary::InletOutlet {
                    incoming_temperature,
                } => {
                    incoming = scripted.unwrap_or(incoming_temperature);
                    if q < 0. {
                        thermal.conductivity * (incoming - t[i]) / distance * f.area
                    } else {
                        0.
                    }
                }
                Boundary::HeatFlux { inward_flux } => inward_flux * f.area,
                Boundary::Convection {
                    coefficient,
                    ambient_temperature,
                } => {
                    let conductance = thermal.conductivity / distance;
                    coefficient * conductance / (coefficient + conductance)
                        * (ambient_temperature - t[i])
                        * f.area
                }
                Boundary::Radiation {
                    emissivity,
                    ambient_temperature,
                } => {
                    let (slope, intercept) = crate::multiphysics::energy::radiative_linearization(
                        t[i],
                        ambient_temperature,
                        thermal.conductivity / distance,
                        emissivity,
                    )?;
                    (intercept - slope * t[i]) * f.area
                }
            };
            e.inward_heat_w += heat;
            e.outward_advection_w +=
                rho * thermal.specific_heat * q * if q < 0. { incoming } else { t[i] };
        }
    }
    m.imbalance =
        m.storage + m.outward_advection + m.outward_pressure - m.inward_viscous - m.body_force;
    let scale = m.storage.norm()
        + m.outward_advection.norm()
        + m.outward_pressure.norm()
        + m.inward_viscous.norm()
        + m.body_force.norm();
    m.relative_imbalance = (scale > 0.).then(|| m.imbalance.norm() / scale);
    if [m.imbalance.x, m.imbalance.y, m.imbalance.z, scale]
        .iter()
        .any(|v| !v.is_finite())
    {
        return Err("Momentum budget overflow".into());
    }
    result.momentum = Some(m);
    if thermal.enabled {
        e.imbalance_w = e.storage_w + e.outward_advection_w - e.inward_heat_w - e.source_w;
        let scale = e.storage_w.abs()
            + e.outward_advection_w.abs()
            + e.inward_heat_w.abs()
            + e.source_w.abs();
        e.relative_imbalance = (scale > 0.).then(|| e.imbalance_w.abs() / scale);
        if !e.imbalance_w.is_finite() || !scale.is_finite() {
            return Err("Energy budget overflow".into());
        }
        result.energy = Some(e);
    }
    Ok(result)
}
