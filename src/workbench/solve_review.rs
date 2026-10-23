//! Pre-run review and finite-field health diagnostics; no convergence claims.
use super::{diagnostics::mesh_preflight, plane_monitor::FlowPlaneMonitor};
use crate::{IncompressibleCase, IncompressibleInitialConditions, MeshDimension};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CaseReview {
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}
impl CaseReview {
    pub fn blocked(&self) -> bool {
        !self.errors.is_empty()
    }
    pub fn summary(&self) -> String {
        let mut lines = vec![if self.blocked() {
            "BLOCKED"
        } else {
            "PRE-RUN REVIEW"
        }
        .into()];
        lines.extend(self.errors.iter().map(|s| format!("ERROR · {s}")));
        lines.extend(self.warnings.iter().map(|s| format!("NOTE · {s}")));
        if lines.len() == 1 {
            lines.push("Checks passed. This does not establish numerical accuracy.".into());
        }
        lines.join("\n")
    }
}
pub fn review(case: &IncompressibleCase, planes: &[FlowPlaneMonitor]) -> CaseReview {
    let mut r = CaseReview::default();
    if let Err(e) = case.resolve_boundaries() {
        r.errors.push(format!("Case settings: {e:?}"));
    }
    let mesh = mesh_preflight(&case.mesh);
    if mesh.blocked() {
        r.errors.push(mesh.summary());
    }
    r.warnings.extend(mesh.warnings);
    for plane in planes.iter().filter(|p| p.enabled) {
        if case.mesh.dimension() != MeshDimension::ThreeD {
            r.warnings.push(format!(
                "Plane '{}': volume recording is skipped for this 2D mesh.",
                plane.name
            ));
            continue;
        }
        match super::plane_monitor::intersect(&case.mesh,plane) {
            Ok(cells) if cells.is_empty()=>r.errors.push(format!("Plane '{}': no intersection with fluid volume. Disable recording or edit its position.",plane.name)),
            Err(e)=>r.errors.push(format!("Plane '{}': {e}",plane.name)),
            _=>{}
        }
    }
    r
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SolveHealth {
    pub iteration: usize,
    pub max_speed: f64,
    pub max_speed_cell: usize,
    pub pressure_min: f64,
    pub pressure_max: f64,
    pub invalid_cells: usize,
    pub inflow: f64,
    pub outflow: f64,
    pub relative_imbalance: Option<f64>,
    /// Fraction of outlet faces with inward flux, separate from reverse flux fraction.
    pub outlet_reverse_faces: usize,
    pub outlet_faces: usize,
    pub outlet_reverse_flux: f64,
    pub outlet_absolute_flux: f64,
}
impl SolveHealth {
    pub fn summary(&self) -> String {
        format!("FIELD HEALTH · sample {}\nMax |U| {:.6e} m/s · cell {}\np/ρ [{:.6e}, {:.6e}] m²/s² · invalid cells {}\nFace-flux inflow {:.6e} · outflow {:.6e}\nBoundary imbalance {}\nOutlet reverse-flow faces {}/{} · reverse flux fraction {}\nFluxes are conservative face values; 2D flux is per unit depth. Not a convergence certificate.",
            self.iteration,self.max_speed,self.max_speed_cell,self.pressure_min,self.pressure_max,self.invalid_cells,self.inflow,self.outflow,
            self.relative_imbalance.map(|v|format!("{:.4e}%",100.0*v)).unwrap_or("N/A (zero throughflow)".into()),
            self.outlet_reverse_faces,self.outlet_faces,
            (self.outlet_absolute_flux>0.0).then(||format!("{:.4e}%",100.0*self.outlet_reverse_flux/self.outlet_absolute_flux)).unwrap_or("N/A".into()))
    }
}
pub fn health(
    case: &IncompressibleCase,
    initial: &IncompressibleInitialConditions,
    iteration: usize,
) -> Option<SolveHealth> {
    let IncompressibleInitialConditions::Restart {
        velocity,
        pressure,
        face_flux,
        ..
    } = initial
    else {
        return None;
    };
    let mesh = &case.mesh;
    if velocity.values().len() != mesh.cell_count()
        || pressure.values().len() != mesh.cell_count()
        || face_flux.values().len() != mesh.face_count()
    {
        return None;
    }
    let mut r = SolveHealth {
        iteration,
        max_speed: 0.0,
        max_speed_cell: 0,
        pressure_min: f64::INFINITY,
        pressure_max: f64::NEG_INFINITY,
        invalid_cells: 0,
        inflow: 0.0,
        outflow: 0.0,
        relative_imbalance: None,
        outlet_reverse_faces: 0,
        outlet_faces: 0,
        outlet_reverse_flux: 0.0,
        outlet_absolute_flux: 0.0,
    };
    for (i, (v, p)) in velocity.values().iter().zip(pressure.values()).enumerate() {
        let speed = v.norm();
        if !speed.is_finite() || !p.is_finite() {
            r.invalid_cells += 1;
            continue;
        }
        if speed > r.max_speed {
            r.max_speed = speed;
            r.max_speed_cell = i;
        }
        r.pressure_min = r.pressure_min.min(*p);
        r.pressure_max = r.pressure_max.max(*p);
    }
    if !r.pressure_min.is_finite() {
        return None;
    }
    for patch in mesh.boundary_patches() {
        let outlet = case.boundaries.iter().any(|(name, bc)| {
            name == &patch.name
                && matches!(
                    bc,
                    crate::IncompressibleBoundaryCondition::PressureOutlet { .. }
                        | crate::IncompressibleBoundaryCondition::PressureOutletBackflow { .. }
                )
        });
        for &i in &patch.face_indices {
            let q = face_flux[i];
            if !q.is_finite() {
                return None;
            }
            if q < 0.0 {
                r.inflow -= q;
            } else {
                r.outflow += q;
            }
            if outlet {
                r.outlet_faces += 1;
                r.outlet_absolute_flux += q.abs();
                if q < 0.0 {
                    r.outlet_reverse_faces += 1;
                    r.outlet_reverse_flux -= q;
                }
            }
        }
    }
    let flow = r.inflow.max(r.outflow);
    r.relative_imbalance = (flow > 0.0).then(|| (r.outflow - r.inflow).abs() / flow);
    Some(r)
}

impl SolveHealth {
    pub fn csv(&self) -> String {
        format!("iteration,max_speed_m_s,max_speed_cell,p_over_rho_min_m2_s2,p_over_rho_max_m2_s2,invalid_cells,inflow,outflow,relative_imbalance,outlet_reverse_faces,outlet_faces,outlet_reverse_flux,outlet_absolute_flux\n{},{:.12e},{},{:.12e},{:.12e},{},{:.12e},{:.12e},{},{},{},{:.12e},{:.12e}\n",
            self.iteration,self.max_speed,self.max_speed_cell,self.pressure_min,self.pressure_max,self.invalid_cells,self.inflow,self.outflow,
            self.relative_imbalance.map(|v|format!("{v:.12e}")).unwrap_or_default(),self.outlet_reverse_faces,self.outlet_faces,self.outlet_reverse_flux,self.outlet_absolute_flux)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        CellDefinition, CellField, FaceField, IncompressibleBoundaryCondition as Bc,
        IncompressibleMaterial, IncompressibleSolverOptions, Point, UnstructuredMesh, Vec3,
    };
    fn case() -> IncompressibleCase {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0., 0., 0.),
                Point::new(1., 0., 0.),
                Point::new(1., 1., 0.),
                Point::new(0., 1., 0.),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2, 3])],
        )
        .unwrap();
        let indices = mesh
            .faces()
            .iter()
            .enumerate()
            .filter(|(_, f)| f.neighbour.is_none())
            .map(|(i, _)| i)
            .collect();
        let mesh = mesh
            .with_boundary_patches(vec![crate::BoundaryPatch {
                name: "out".into(),
                face_indices: indices,
                boundary_type: crate::BoundaryType::PressureOutlet,
            }])
            .unwrap();
        let boundaries = mesh
            .boundary_patches()
            .iter()
            .map(|p| {
                (
                    p.name.clone(),
                    Bc::PressureOutletBackflow {
                        pressure: 0.,
                        velocity: Vec3::ZERO,
                    },
                )
            })
            .collect();
        IncompressibleCase::steady(
            mesh,
            boundaries,
            IncompressibleMaterial {
                density: 1.,
                kinematic_viscosity: 0.01,
            },
            IncompressibleSolverOptions::default(),
        )
    }
    fn fields(case: &IncompressibleCase, flux: Vec<f64>) -> IncompressibleInitialConditions {
        IncompressibleInitialConditions::Restart {
            velocity: CellField::from_values(&case.mesh, vec![Vec3::new(3., 4., 0.)]).unwrap(),
            pressure: CellField::from_values(&case.mesh, vec![2.]).unwrap(),
            face_flux: FaceField::from_values(&case.mesh, flux).unwrap(),
            time: 0.,
            last_dt: None,
            temperature: None,
            turbulence: None,
        }
    }
    #[test]
    fn conservative_health_separates_face_count_from_reverse_flux() {
        let c = case();
        let r = health(&c, &fields(&c, vec![-1., 3., -2., 0.]), 17).unwrap();
        assert_eq!(r.max_speed, 5.);
        assert_eq!(r.max_speed_cell, 0);
        assert_eq!(r.inflow, 3.);
        assert_eq!(r.outflow, 3.);
        assert_eq!(r.relative_imbalance, Some(0.));
        assert_eq!(r.outlet_reverse_faces, 2);
        assert_eq!(r.outlet_faces, 4);
        assert_eq!(r.outlet_reverse_flux / r.outlet_absolute_flux, 0.5);
        assert_eq!(r.iteration, 17);
        assert!(r.csv().contains("max_speed_m_s"));
    }
    #[test]
    fn zero_throughflow_has_no_relative_percentage() {
        let c = case();
        let r = health(&c, &fields(&c, vec![0.; 4]), 1).unwrap();
        assert_eq!(r.relative_imbalance, None);
        assert!(r.summary().contains("N/A"));
    }
    #[test]
    fn nonfinite_flux_is_never_reported_as_healthy() {
        let c = case();
        assert!(health(&c, &fields(&c, vec![f64::NAN, 0., 0., 0.]), 1).is_none());
    }
    #[test]
    fn two_dimensional_plane_recording_is_explicitly_skipped() {
        let c = case();
        let p = FlowPlaneMonitor {
            id: "xy".into(),
            name: "XY".into(),
            origin: Vec3::ZERO,
            u: Vec3::new(1., 0., 0.),
            v: Vec3::new(0., 1., 0.),
            enabled: true,
            every: 10,
            display: Default::default(),
        };
        let r = review(&c, &[p]);
        assert!(r.warnings.iter().any(|s| s.contains("skipped")));
    }
}
