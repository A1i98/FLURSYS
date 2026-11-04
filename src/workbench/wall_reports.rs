//! First-cell, wall-resolved shear/y+ estimates; not a wall function.
//! Pressure force uses the adjacent-cell kinematic gauge pressure. Forces act
//! on the wall; two-dimensional integrated forces are per unit depth.
use crate::{CellField, IncompressibleBoundaryCondition, IncompressibleCase, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ThermalWallReport {
    pub patch: String,
    pub measure: f64,
    pub inward_heat: Option<f64>,
    pub mean_inward_heat_flux: Option<f64>,
    pub heat_unit: String,
    pub method: String,
}

pub fn thermal_reports(
    case: &IncompressibleCase,
    temperature: &CellField<f64>,
) -> Result<Vec<ThermalWallReport>, String> {
    let mesh = &case.mesh;
    temperature
        .ensure_mesh(mesh)
        .map_err(|e| format!("{e:?}"))?;
    let thermal = &case.solver.thermal;
    if !thermal.enabled {
        return Ok(Vec::new());
    }
    thermal.validate_for_case(case)?;
    let mut result = Vec::new();
    for (name, boundary) in &case.boundaries {
        if !matches!(
            boundary,
            IncompressibleBoundaryCondition::NoSlipWall
                | IncompressibleBoundaryCondition::MovingWall { .. }
        ) {
            continue;
        }
        let patch = mesh
            .boundary_patches()
            .iter()
            .find(|p| p.name == *name)
            .ok_or_else(|| format!("Missing wall patch {name}"))?;
        let condition = *thermal
            .boundaries
            .get(name)
            .ok_or_else(|| format!("Missing thermal boundary for {name}"))?;
        let scripted = case
            .solver
            .scripting
            .bindings
            .get(name)
            .is_some_and(|bindings| {
                bindings
                    .iter()
                    .any(|binding| binding.quantity == crate::scripting::Quantity::Temperature)
            })
            || case
                .solver
                .boundary_profiles
                .get(name)
                .is_some_and(|profile| {
                    profile.quantity
                        == crate::unstructured_incompressible::BoundaryQuantity::Temperature
                });
        let supported = !scripted
            && match condition {
                crate::thermal::Boundary::Adiabatic | crate::thermal::Boundary::HeatFlux { .. } => {
                    true
                }
                crate::thermal::Boundary::FixedTemperature { .. } => {
                    thermal.wall_treatment == crate::thermal::ThermalWallTreatment::Molecular
                }
                _ => false,
            };
        let mut measure = 0.0;
        let mut heat = 0.0;
        for &index in &patch.face_indices {
            let face = &mesh.faces()[index];
            measure += face.area;
            if !supported {
                continue;
            }
            heat += match condition {
                crate::thermal::Boundary::Adiabatic => 0.0,
                crate::thermal::Boundary::HeatFlux { inward_flux } => inward_flux * face.area,
                crate::thermal::Boundary::FixedTemperature { temperature: wall } => {
                    let displacement = face.center - mesh.cells()[face.owner].center;
                    let (factor, _) = crate::numerics::diffusion_geometry(
                        face.area_vector,
                        displacement,
                        crate::NonOrthogonalCorrection::Limited,
                        index,
                    )
                    .map_err(|e| format!("{e:?}"))?;
                    thermal.conductivity * factor * (wall - temperature[face.owner])
                }
                _ => unreachable!(),
            };
        }
        if !measure.is_finite() || measure <= 0.0 || !heat.is_finite() {
            return Err(format!("Invalid thermal wall measure or heat on {name}"));
        }
        result.push(ThermalWallReport {
            patch: name.clone(),
            measure,
            inward_heat: supported.then_some(heat),
            mean_inward_heat_flux: supported.then_some(heat / measure),
            heat_unit: if mesh.dimension() == crate::MeshDimension::TwoD {
                "W/m"
            } else {
                "W"
            }
            .into(),
            method: if supported {
                "solver boundary discretization, evaluated at saved field"
            } else {
                "unsupported boundary, wall treatment, or script"
            }
            .into(),
        });
    }
    Ok(result)
}
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
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct WallPatchReport {
    #[serde(default)]
    pub heat_flux_outward_mean: Option<f64>,
    #[serde(default)]
    pub heat_transfer_outward: Option<f64>,
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
    /// Moments about the global origin; shift with M(c)=M(0)-c cross F.
    #[serde(default)]
    pub pressure_moment: Vec3,
    #[serde(default)]
    pub shear_moment: Vec3,
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
    let equilibrium = matches!(
        case.solver.turbulence.wall_treatment,
        crate::turbulence::WallTreatment::EquilibriumKEpsilon
            | crate::turbulence::WallTreatment::EquilibriumKOmega
            | crate::turbulence::WallTreatment::VelocityBlendedKOmega
    );
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
            heat_flux_outward_mean: None,
            heat_transfer_outward: None,
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
            pressure_moment: Vec3::ZERO,
            shear_moment: Vec3::ZERO,
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
            let nut = if case.solver.turbulence.wall_treatment
                == crate::turbulence::WallTreatment::VelocityBlendedKOmega
            {
                crate::turbulence::velocity_blended_wall_gradient(
                    tangent.norm(),
                    relative.norm() / y,
                    y,
                    nu,
                )?
                .1
            } else if equilibrium {
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
            r.pressure_moment += f.center.cross(pressure_force);
            r.shear_moment += f.center.cross(shear_force);
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
            r.pressure_moment.x,
            r.pressure_moment.y,
            r.pressure_moment.z,
            r.shear_moment.x,
            r.shear_moment.y,
            r.shear_moment.z,
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
    let mut text = String::from(
        "WALL-RESOLVED ESTIMATES · first-cell tangential gradient\nShear stress: Pa · forces: N (2D: N per unit depth).\nPressure force uses adjacent-cell gauge pressure. No wall functions.\n",
    );
    for r in reports {
        let y = match (r.y_plus_min, r.y_plus_mean, r.y_plus_max) {
            (Some(a), Some(b), Some(c)) => format!("{a:.4e} / {b:.4e} / {c:.4e}"),
            _ => "N/A (zero molecular viscosity or no valid faces)".into(),
        };
        text.push_str(&format!("\n{} · {} faces · {} invalid\ny+ min / area-mean / max: {y}\nArea fraction y+ > 1: {}\nShear mean / max: {:.4e} / {:.4e} Pa\nForce on wall (Fx, Fy, Fz): {:.6e}, {:.6e}, {:.6e}\n",r.patch,r.faces,r.invalid_faces,r.fraction_measure_above_y_plus_one.map(|v|format!("{:.2}%",100.0*v)).unwrap_or_else(||"N/A".into()),r.shear_stress_mean,r.shear_stress_max,r.total_force.x,r.total_force.y,r.total_force.z));
    }
    for r in reports {
        let m = r.pressure_moment + r.shear_moment;
        text.push_str(&format!(
            "\n{} · moment about global origin: ({:.6e}, {:.6e}, {:.6e}) {}\n",
            r.patch,
            m.x,
            m.y,
            m.z,
            if r.force_unit == "N/m" {
                "N (per unit depth)"
            } else {
                "N m"
            }
        ));
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
                let _ = writeln!(
                    s,
                    "{},{},{},{:.16e},{:.16e},{:.16e},{:.16e},{},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{}",
                    quote(&r.patch),
                    f.face,
                    f.owner,
                    f.point.x,
                    f.point.y,
                    f.point.z,
                    f.normal_distance,
                    optional(f.y_plus),
                    f.friction_velocity,
                    f.shear_stress.x,
                    f.shear_stress.y,
                    f.shear_stress.z,
                    f.pressure_force.x,
                    f.pressure_force.y,
                    f.pressure_force.z,
                    f.shear_force.x,
                    f.shear_force.y,
                    f.shear_force.z,
                    r.force_unit
                );
            }
        }
    } else {
        s.push_str("patch,faces,invalid_faces,measure,y_plus_min,y_plus_mean,y_plus_max,fraction_measure_y_plus_above_one,shear_mean_Pa,shear_max_Pa,force_x,force_y,force_z,pressure_moment_x,pressure_moment_y,pressure_moment_z,viscous_moment_x,viscous_moment_y,viscous_moment_z,force_unit\n");
        for r in reports {
            let _ = writeln!(
                s,
                "{},{},{},{:.16e},{},{},{},{},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{:.16e},{}",
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
                r.pressure_moment.x,
                r.pressure_moment.y,
                r.pressure_moment.z,
                r.shear_moment.x,
                r.shear_moment.y,
                r.shear_moment.z,
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

/// Reference measure is area in 3D, or length per unit depth in 2D.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ForceReference {
    pub density: f64,
    pub speed: f64,
    pub measure: f64,
    pub length: f64,
    pub center: Vec3,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ForceCoefficients {
    pub pressure: Vec3,
    pub viscous: Vec3,
    pub total: Vec3,
    pub moment: Vec3,
}
impl WallPatchReport {
    pub fn coefficients(&self, reference: ForceReference) -> Result<ForceCoefficients, String> {
        if [
            reference.density,
            reference.speed,
            reference.measure,
            reference.length,
        ]
        .iter()
        .any(|v| !v.is_finite() || *v <= 0.0)
            || [reference.center.x, reference.center.y, reference.center.z]
                .iter()
                .any(|v| !v.is_finite())
        {
            return Err("Force reference requires finite positive density, speed, measure and length, and a finite center".into());
        }
        let scale = 0.5 * reference.density * reference.speed.powi(2) * reference.measure;
        let moment_scale = scale * reference.length;
        if !scale.is_finite() || scale <= 0.0 || !moment_scale.is_finite() || moment_scale <= 0.0 {
            return Err("Force reference normalization overflow or underflow".into());
        }
        Ok(ForceCoefficients {
            pressure: self.pressure_force / scale,
            viscous: self.shear_force / scale,
            total: self.total_force / scale,
            moment: (self.pressure_moment + self.shear_moment
                - reference.center.cross(self.total_force))
                / moment_scale,
        })
    }
}

/// Wall-shear zero crossings along an explicitly supplied straight wall segment.
/// Returns downstream distances from start; no crossing is an empty vector.
/// Detailed wall samples must be requested when constructing the report.
pub fn reattachment_distances(
    report: &WallPatchReport,
    start: Vec3,
    direction: Vec3,
    transverse_tolerance: f64,
) -> Result<Vec<f64>, String> {
    let length = direction.norm();
    if !length.is_finite()
        || length <= 0.0
        || !transverse_tolerance.is_finite()
        || transverse_tolerance <= 0.0
        || [start.x, start.y, start.z].iter().any(|v| !v.is_finite())
    {
        return Err("Require a finite start, nonzero direction and positive wall tolerance".into());
    }
    if report.samples.is_empty() {
        return Err("Reattachment requires detailed wall samples".into());
    }
    let axis = direction / length;
    let mut samples: Vec<_> = report
        .samples
        .iter()
        .filter_map(|s| {
            let d = s.point - start;
            let x = d.dot(axis);
            let transverse = (d - axis * x).norm();
            let shear = s.shear_stress.dot(axis);
            (x >= 0.0 && transverse <= transverse_tolerance && shear.is_finite())
                .then_some((x, shear))
        })
        .collect();
    samples.sort_by(|a, b| a.0.total_cmp(&b.0));
    Ok(samples
        .windows(2)
        .filter_map(|w| {
            let ((x0, t0), (x1, t1)) = (w[0], w[1]);
            (x1 > x0 && t0 <= 0.0 && t1 > 0.0).then(|| x0 + (x1 - x0) * (-t0) / (t1 - t0))
        })
        .collect())
}

#[cfg(test)]
mod thermal_report_tests {
    use super::*;
    use crate::{
        BoundaryPatch, BoundaryType, CellDefinition, IncompressibleMaterial,
        IncompressibleSolverOptions, MeshDimension, Point, UnstructuredMesh,
    };

    #[test]
    fn wall_heat_uses_the_solver_discretization_and_marks_unsupported_conditions() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(2., 0., 0.),
                Point::new(2.5, 1., 0.),
                Point::new(0.5, 1., 0.),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2, 3])],
        )
        .unwrap();
        let left = mesh.faces().iter().position(|f| f.center.x < 0.5).unwrap();
        let rest = (0..mesh.face_count()).filter(|i| *i != left).collect();
        let mesh = mesh
            .with_boundary_patches(vec![
                BoundaryPatch {
                    name: "left".into(),
                    face_indices: vec![left],
                    boundary_type: BoundaryType::Wall,
                },
                BoundaryPatch {
                    name: "rest".into(),
                    face_indices: rest,
                    boundary_type: BoundaryType::Wall,
                },
            ])
            .unwrap();
        let mut solver = IncompressibleSolverOptions::default();
        solver.thermal.enabled = true;
        solver.thermal.conductivity = 2.;
        solver.thermal.boundaries.insert(
            "left".into(),
            crate::thermal::Boundary::FixedTemperature { temperature: 301. },
        );
        solver
            .thermal
            .boundaries
            .insert("rest".into(), crate::thermal::Boundary::Adiabatic);
        let mut case = IncompressibleCase::steady(
            mesh,
            vec![
                ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
                ("rest".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ],
            IncompressibleMaterial {
                density: 1.,
                kinematic_viscosity: 0.01,
            },
            solver,
        );
        let field = CellField::filled(&case.mesh, 300.);
        let reports = thermal_reports(&case, &field).unwrap();
        let left = reports.iter().find(|r| r.patch == "left").unwrap();
        assert!((left.inward_heat.unwrap() - 2.).abs() < 1e-12);
        assert_eq!(left.heat_unit, "W/m");
        assert_eq!(
            reports
                .iter()
                .find(|r| r.patch == "rest")
                .unwrap()
                .inward_heat,
            Some(0.)
        );
        case.solver.thermal.boundaries.insert(
            "left".into(),
            crate::thermal::Boundary::Convection {
                coefficient: 1.,
                ambient_temperature: 301.,
            },
        );
        assert_eq!(
            thermal_reports(&case, &field)
                .unwrap()
                .iter()
                .find(|r| r.patch == "left")
                .unwrap()
                .inward_heat,
            None
        );
    }
}
