//! First-cell, wall-resolved shear/y+ estimates; not a wall function.
//! Pressure force uses the adjacent-cell kinematic gauge pressure. Forces act
//! on the wall; two-dimensional integrated forces are per unit depth.
use crate::{CellField, IncompressibleBoundaryCondition, IncompressibleCase, Vec3};
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WallFaceSample {
    pub face: usize,
    pub owner: usize,
    pub point: Vec3,
    pub normal_distance: f64,
    pub y_plus: Option<f64>,
    pub shear_stress: Vec3,
    pub friction_velocity: f64,
    pub pressure_force: Vec3,
    pub shear_force: Vec3,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WallPatchReport {
    pub patch: String,
    pub force_unit: String,
    pub faces: usize,
    pub invalid_faces: usize,
    pub measure: f64,
    pub y_plus_min: Option<f64>,
    pub y_plus_mean: Option<f64>,
    pub y_plus_max: Option<f64>,
    pub fraction_measure_above_y_plus_one: Option<f64>,
    pub shear_stress_mean: f64,
    pub shear_stress_max: f64,
    pub pressure_force: Vec3,
    pub shear_force: Vec3,
    pub total_force: Vec3,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub samples: Vec<WallFaceSample>,
}
pub fn reports(
    case: &IncompressibleCase,
    u: &CellField<Vec3>,
    p: &CellField<f64>,
    detailed: bool,
) -> Result<Vec<WallPatchReport>, String> {
    reports_with_turbulence(case, u, p, None, detailed)
}
pub fn reports_with_turbulence(
    case: &IncompressibleCase,
    u: &CellField<Vec3>,
    p: &CellField<f64>,
    turbulence: Option<&crate::turbulence::TurbulenceFields>,
    detailed: bool,
) -> Result<Vec<WallPatchReport>, String> {
    let equilibrium = case.solver.turbulence.wall_treatment
        == crate::turbulence::WallTreatment::EquilibriumKEpsilon;
    if equilibrium && turbulence.is_none() {
        return Err("Wall-function reports require the matching turbulence fields".into());
    }
    if let Some(t) = turbulence {
        t.k.ensure_mesh(&case.mesh).map_err(|e| format!("{e:?}"))?;
    }
    let mesh = &case.mesh;
    u.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
    p.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
    let rho = case.material.density;
    let nu = case.material.kinematic_viscosity;
    if !rho.is_finite() || rho <= 0.0 || !nu.is_finite() || nu < 0.0 {
        return Err("Invalid density or viscosity for wall diagnostics".into());
    }
    let mut out = Vec::new();
    for (name, bc) in &case.boundaries {
        let velocity = match bc {
            IncompressibleBoundaryCondition::NoSlipWall => Vec3::ZERO,
            IncompressibleBoundaryCondition::MovingWall { velocity } => *velocity,
            _ => continue,
        };
        let Some(patch) = mesh.boundary_patches().iter().find(|q| q.name == *name) else {
            continue;
        };
        let mut r = WallPatchReport {
            patch: name.clone(),
            force_unit: if mesh.dimension() == crate::MeshDimension::TwoD {
                "N/m".into()
            } else {
                "N".into()
            },
            faces: patch.face_indices.len(),
            invalid_faces: 0,
            measure: 0.0,
            y_plus_min: None,
            y_plus_mean: None,
            y_plus_max: None,
            fraction_measure_above_y_plus_one: None,
            shear_stress_mean: 0.0,
            shear_stress_max: 0.0,
            pressure_force: Vec3::ZERO,
            shear_force: Vec3::ZERO,
            total_force: Vec3::ZERO,
            samples: Vec::new(),
        };
        let mut weighted_y = 0.0;
        let mut above = 0.0;
        for &i in &patch.face_indices {
            let f = &mesh.faces()[i];
            let normal = f.area_vector / f.area;
            let y = (f.center - mesh.cells()[f.owner].center).dot(normal).abs();
            let relative = u[f.owner] - velocity;
            let tangent = relative - normal * relative.dot(normal);
            let nut = if equilibrium {
                crate::turbulence::smooth_wall(turbulence.unwrap().k[f.owner], y, nu)?.1
            } else {
                0.0
            };
            let tau = tangent * (rho * (nu + nut) / y);
            let magnitude = tau.norm();
            let friction = ((nu + nut) * tangent.norm() / y).sqrt();
            let yplus = (nu > 0.0).then(|| friction * y / nu);
            if y <= 0.0
                || !y.is_finite()
                || !magnitude.is_finite()
                || !p[f.owner].is_finite()
                || yplus.is_some_and(|v| !v.is_finite())
            {
                r.invalid_faces += 1;
                continue;
            }
            let pressure_force = f.area_vector * (rho * p[f.owner]);
            let shear_force = tau * f.area;
            if [
                pressure_force.x,
                pressure_force.y,
                pressure_force.z,
                shear_force.x,
                shear_force.y,
                shear_force.z,
            ]
            .iter()
            .any(|x| !x.is_finite())
            {
                r.invalid_faces += 1;
                continue;
            }
            r.pressure_force += pressure_force;
            r.shear_force += shear_force;
            r.measure += f.area;
            r.shear_stress_mean += magnitude * f.area;
            r.shear_stress_max = r.shear_stress_max.max(magnitude);
            if let Some(v) = yplus {
                r.y_plus_min = Some(r.y_plus_min.map_or(v, |x| x.min(v)));
                r.y_plus_max = Some(r.y_plus_max.map_or(v, |x| x.max(v)));
                weighted_y += v * f.area;
                if v > 1.0 {
                    above += f.area;
                }
            }
            if detailed {
                r.samples.push(WallFaceSample {
                    face: i,
                    owner: f.owner,
                    point: f.center,
                    normal_distance: y,
                    y_plus: yplus,
                    shear_stress: tau,
                    friction_velocity: friction,
                    pressure_force,
                    shear_force,
                });
            }
        }
        if r.measure > 0.0 {
            r.shear_stress_mean /= r.measure;
            if nu > 0.0 {
                r.y_plus_mean = Some(weighted_y / r.measure);
                r.fraction_measure_above_y_plus_one = Some(above / r.measure);
            }
        }
        r.total_force = r.pressure_force + r.shear_force;
        if [
            r.measure,
            r.shear_stress_mean,
            r.shear_stress_max,
            r.pressure_force.x,
            r.pressure_force.y,
            r.pressure_force.z,
            r.shear_force.x,
            r.shear_force.y,
            r.shear_force.z,
            r.total_force.x,
            r.total_force.y,
            r.total_force.z,
        ]
        .iter()
        .any(|x| !x.is_finite())
        {
            return Err("Wall integral overflow".into());
        }
        if [
            r.y_plus_min,
            r.y_plus_mean,
            r.y_plus_max,
            r.fraction_measure_above_y_plus_one,
        ]
        .into_iter()
        .flatten()
        .any(|v| !v.is_finite())
        {
            return Err("Wall statistics overflow".into());
        }
        out.push(r);
    }
    Ok(out)
}
pub fn summary(reports: &[WallPatchReport]) -> String {
    if reports.is_empty() {
        return "Wall diagnostics unavailable: no solved no-slip / moving-wall patches.".into();
    }
    let mut text=String::from("WALL-RESOLVED ESTIMATES · first-cell tangential gradient\nShear stress: Pa · forces: N (2D: N per unit depth).\nPressure force uses adjacent-cell gauge pressure. No wall functions.\n");
    for r in reports {
        let y = match (r.y_plus_min, r.y_plus_mean, r.y_plus_max) {
            (Some(a), Some(b), Some(c)) => format!("{a:.4e} / {b:.4e} / {c:.4e}"),
            _ => "N/A (zero molecular viscosity or no valid faces)".into(),
        };
        text.push_str(&format!("\n{} · {} faces · {} invalid\ny+ min / area-mean / max: {y}\nArea fraction y+ > 1: {}\nShear mean / max: {:.4e} / {:.4e} Pa\nForce on wall (Fx, Fy, Fz): {:.6e}, {:.6e}, {:.6e}\n",r.patch,r.faces,r.invalid_faces,r.fraction_measure_above_y_plus_one.map(|v|format!("{:.2}%",100.0*v)).unwrap_or_else(||"N/A".into()),r.shear_stress_mean,r.shear_stress_max,r.total_force.x,r.total_force.y,r.total_force.z));
    }
    text
}
pub fn csv(reports: &[WallPatchReport], faces: bool) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    if faces {
        s.push_str("patch,face,owner,x_m,y_m,z_m,normal_distance_m,y_plus,friction_velocity_m_s,tau_x_Pa,tau_y_Pa,tau_z_Pa,pressure_force_x,pressure_force_y,pressure_force_z,shear_force_x,shear_force_y,shear_force_z,force_unit\n");
        for r in reports {
            for f in &r.samples {
                let _=writeln!(s,"{},{},{},{:.16e},{:.16e},{:.16e},{:.16e},{},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{}",quote(&r.patch),f.face,f.owner,f.point.x,f.point.y,f.point.z,f.normal_distance,optional(f.y_plus),f.friction_velocity,f.shear_stress.x,f.shear_stress.y,f.shear_stress.z,f.pressure_force.x,f.pressure_force.y,f.pressure_force.z,f.shear_force.x,f.shear_force.y,f.shear_force.z,r.force_unit);
            }
        }
    } else {
        s.push_str("patch,faces,invalid_faces,measure,y_plus_min,y_plus_mean,y_plus_max,fraction_measure_y_plus_above_one,shear_mean_Pa,shear_max_Pa,force_x,force_y,force_z,force_unit\n");
        for r in reports {
            let _ = writeln!(
                s,
                "{},{},{},{:.16e},{},{},{},{},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{}",
                quote(&r.patch),
                r.faces,
                r.invalid_faces,
                r.measure,
                optional(r.y_plus_min),
                optional(r.y_plus_mean),
                optional(r.y_plus_max),
                optional(r.fraction_measure_above_y_plus_one),
                r.shear_stress_mean,
                r.shear_stress_max,
                r.total_force.x,
                r.total_force.y,
                r.total_force.z,
                r.force_unit
            );
        }
    }
    s
}
fn quote(s: &str) -> String {
    format!("\"{}\"", s.replace('"', "\"\""))
}
fn optional(v: Option<f64>) -> String {
    v.map(|v| format!("{v:.16e}")).unwrap_or_default()
}
