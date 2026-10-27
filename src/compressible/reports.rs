use super::*;
use crate::workbench::wall_reports::{WallFaceSample, WallPatchReport};
pub fn wall_reports(
    mesh: &UnstructuredMesh,
    settings: &Settings,
    state: &Snapshot,
    detailed: bool,
) -> Result<Vec<WallPatchReport>, String> {
    state.validate(mesh, settings.gas)?;
    let mut output = Vec::new();
    for patch in mesh.boundary_patches() {
        let Some(condition) = settings.boundaries.get(&patch.name).copied() else {
            continue;
        };
        if !matches!(
            condition,
            Boundary::SlipWall | Boundary::NoSlipWall { .. } | Boundary::MovingWall { .. }
        ) {
            continue;
        }
        let mut report = WallPatchReport {
            patch: patch.name.clone(),
            force_unit: if mesh.dimension() == crate::MeshDimension::TwoD {
                "N/m".into()
            } else {
                "N".into()
            },
            faces: patch.face_indices.len(),
            ..Default::default()
        };
        let programs = settings
            .scripting
            .bindings
            .get(&patch.name)
            .map(|bindings| {
                bindings
                    .iter()
                    .map(|binding| Ok((binding.clone(), settings.scripting.compiled(binding)?)))
                    .collect::<Result<Vec<_>, String>>()
            })
            .transpose()?;
        let mut weighted_y = 0.;
        let mut above = 0.;
        let mut heat = 0.;
        let mut viscous = false;
        for &i in &patch.face_indices {
            let face = &mesh.faces()[i];
            let p = state.fields[face.owner];
            let normal = face.area_vector / face.area;
            let distance = (face.center - mesh.cells()[face.owner].center).dot(normal);
            if !distance.is_finite() || distance <= 0. {
                report.invalid_faces += 1;
                continue;
            }
            let mut bc = condition;
            if let Some(programs) = &programs {
                bc = bc.scripted(programs, face.center, state.report.time, settings.gas)?;
            }
            let (wall_velocity, temperature) = match bc {
                Boundary::MovingWall {
                    velocity,
                    temperature,
                } => (velocity, temperature),
                Boundary::NoSlipWall { temperature } => (Vec3::ZERO, temperature),
                _ => (p.velocity, None),
            };
            let t = settings.gas.temperature(p);
            let wall_t = temperature.unwrap_or(t);
            let (mu, conductivity) = settings
                .viscosity
                .coefficients(0.5 * (t + wall_t), settings.gas)?;
            let relative = p.velocity - wall_velocity;
            let tangent = relative - normal * relative.dot(normal);
            let shear = if matches!(bc, Boundary::SlipWall) {
                Vec3::ZERO
            } else {
                tangent * (mu / distance)
            };
            let magnitude = shear.norm();
            let friction = (magnitude / p.density).sqrt();
            let y_plus = if mu > 0. && !matches!(bc, Boundary::SlipWall) {
                viscous = true;
                Some(p.density * friction * distance / mu)
            } else {
                None
            };
            let pressure_force = face.area_vector * p.pressure;
            let shear_force = shear * face.area;
            report.measure += face.area;
            report.pressure_force += pressure_force;
            report.shear_force += shear_force;
            report.pressure_moment += face.center.cross(pressure_force);
            report.shear_moment += face.center.cross(shear_force);
            report.shear_stress_mean += magnitude * face.area;
            report.shear_stress_max = report.shear_stress_max.max(magnitude);
            if let Some(y) = y_plus {
                weighted_y += y * face.area;
                if y > 1. {
                    above += face.area;
                }
                report.y_plus_min = Some(report.y_plus_min.map_or(y, |v| v.min(y)));
                report.y_plus_max = Some(report.y_plus_max.map_or(y, |v| v.max(y)));
            }
            heat += if temperature.is_some() {
                -conductivity * (wall_t - t) / distance * face.area
            } else {
                0.
            };
            if detailed {
                report.samples.push(WallFaceSample {
                    face: i,
                    owner: face.owner,
                    point: face.center,
                    normal_distance: distance,
                    y_plus,
                    shear_stress: shear,
                    friction_velocity: friction,
                    pressure_force,
                    shear_force,
                });
            }
        }
        if report.measure > 0. {
            report.shear_stress_mean /= report.measure;
            if viscous {
                report.y_plus_mean = Some(weighted_y / report.measure);
                report.fraction_measure_above_y_plus_one = Some(above / report.measure);
            }
            report.heat_flux_outward_mean = Some(heat / report.measure);
            report.heat_transfer_outward = Some(heat);
        }
        report.total_force = report.pressure_force + report.shear_force;
        output.push(report);
    }
    Ok(output)
}
