//! Explicit planar fluid domains and opt-in channel boundary preparation.
use super::*;
use crate::{EdgeGeometry, GeometryFaceRepresentation};

fn invalid(message: impl Into<String>) -> WorkbenchError {
    WorkbenchError::InvalidGrouping {
        message: message.into(),
    }
}

impl WorkbenchSession {
    /// Stable exterior entities of the active fluid domain, in deterministic order.
    pub fn boundary_geometry_targets(
        &self,
    ) -> Result<Vec<GeometrySelectionTarget>, WorkbenchError> {
        let mut targets = match self.mesh_dimension {
            MeshDimension::TwoD => {
                let face = self
                    .fluid_face()
                    .ok_or_else(|| invalid("Select a planar fluid domain first"))?;
                let Some(crate::GeometryFace {
                    representation:
                        GeometryFaceRepresentation::Planar {
                            outer_loop,
                            inner_loops,
                        },
                    ..
                }) = self.geometry.face(face)
                else {
                    return Err(invalid("The fluid domain must be a planar face"));
                };
                outer_loop
                    .iter()
                    .chain(inner_loops.iter().flatten())
                    .map(|edge| GeometrySelectionTarget::Edge(edge.edge))
                    .collect::<Vec<_>>()
            }
            MeshDimension::ThreeD => {
                let mut bodies = self.geometry.bodies().filter(|body| {
                    self.geometry
                        .cad_body(body.id)
                        .is_none_or(|cad| cad.visible)
                });
                let body = bodies
                    .next()
                    .ok_or_else(|| invalid("Create a fluid volume first"))?;
                if bodies.next().is_some() {
                    return Err(invalid(
                        "Keep one visible fluid body for boundary selection",
                    ));
                }
                body.faces
                    .iter()
                    .copied()
                    .map(GeometrySelectionTarget::Face)
                    .collect::<Vec<_>>()
            }
        };
        targets.sort();
        targets.dedup();
        Ok(targets)
    }

    /// Create or replace an explicit, disjoint boundary zone. This does not assign physics.
    pub fn set_boundary_group(
        &mut self,
        name: &str,
        targets: Vec<GeometrySelectionTarget>,
        replace: bool,
    ) -> Result<(), WorkbenchError> {
        let name = name.trim();
        let allowed = self.boundary_geometry_targets()?;
        if targets.is_empty() {
            return Err(invalid("Select at least one boundary entity"));
        }
        for target in &targets {
            if !allowed.contains(target) {
                return Err(invalid(format!(
                    "{target:?} is not an exterior entity of the active fluid domain"
                )));
            }
            if let Some(group) = self
                .named_selections
                .iter()
                .find(|group| (!replace || group.name != name) && group.targets.contains(target))
            {
                return Err(invalid(format!(
                    "{target:?} already belongs to {:?}; edit that group first",
                    group.name
                )));
            }
        }
        let mut staged = self.named_selections.clone();
        if replace {
            if staged.get(name).is_none() {
                return Err(invalid("Select an existing boundary group to replace"));
            }
            staged.delete(name);
        }
        staged.create(name, targets, |_| true)?;
        self.named_selections = staged;
        self.invalidate_mesh();
        Ok(())
    }

    /// Lift the fluid profile and its circular holes into a 3D channel.
    /// The spanwise caps are explicit no-slip walls, not implicit symmetry planes.
    pub fn prepare_extruded_channel(
        &mut self,
        face: FaceId,
        depth: f64,
    ) -> Result<crate::ExtrudeEntities, WorkbenchError> {
        if !depth.is_finite() || depth <= 0.0 {
            return Err(invalid("Channel depth must be finite and positive"));
        }
        if self.geometry.bodies().next().is_some() || self.geometry.cad_bodies().next().is_some() {
            return Err(invalid(
                "Start from an unextruded fluid sketch; a body already exists",
            ));
        }
        let mut staged = Self::from_project(&self.to_project("3D channel preparation"))?;
        staged.prepare_channel_boundaries(face)?;
        let GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } = &staged.geometry.face(face).unwrap().representation
        else {
            unreachable!()
        };
        let edges = outer_loop
            .iter()
            .chain(inner_loops.iter().flatten())
            .map(|e| e.edge)
            .collect::<Vec<_>>();
        let extrusion = staged.geometry.extrude_planar_face(face, depth)?;
        let mut selections = NamedSelectionStore::default();
        for group in staged.named_selections.iter() {
            let faces = group
                .edges()
                .iter()
                .map(|edge| {
                    let index = edges.iter().position(|e| e == edge).unwrap();
                    GeometrySelectionTarget::Face(extrusion.side_faces[index])
                })
                .collect::<Vec<_>>();
            selections.create(&group.name, faces, |_| true)?;
        }
        selections.create(
            "span_walls",
            vec![
                GeometrySelectionTarget::Face(face),
                GeometrySelectionTarget::Face(extrusion.top_face),
            ],
            |_| true,
        )?;
        let groups = selections
            .iter()
            .map(|g| {
                (
                    g.name.clone(),
                    g.targets
                        .iter()
                        .filter_map(|t| {
                            if let GeometrySelectionTarget::Face(f) = t {
                                Some(*f)
                            } else {
                                None
                            }
                        })
                        .collect::<Vec<_>>(),
                )
            })
            .collect::<Vec<_>>();
        GmshGeometryExporter::extruded(&staged.geometry, extrusion.body, groups, "fluid")?;
        self.geometry = staged.geometry;
        self.named_selections = selections;
        self.boundaries = staged.boundaries;
        self.boundaries.insert(
            "span_walls".into(),
            IncompressibleBoundaryCondition::NoSlipWall,
        );
        self.fluid_face_selection = Some(face);
        self.mesh_dimension = MeshDimension::ThreeD;
        self.mesh_recipe = MeshRecipe::default();
        self.invalidate_mesh();
        Ok(extrusion)
    }
    pub fn set_coupling_tolerance(&mut self, tolerance: f64) -> Result<(), WorkbenchError> {
        if !tolerance.is_finite() || tolerance <= 0.0 {
            return Err(invalid(
                "Momentum/velocity relative tolerance must be finite and positive",
            ));
        }
        if self.solver.coupling_relative_tolerance != tolerance {
            self.solver.coupling_relative_tolerance = tolerance;
            self.solution = None;
            self.status = SolveStatus::Idle;
        }
        Ok(())
    }
    pub(super) fn invalidate_mesh(&mut self) {
        self.mesh = None;
        self.mesh_render_cache = None;
        self.mesh_selection = None;
        self.mesh_hover = None;
        self.solution = None;
        self.status = SolveStatus::Idle;
    }

    pub fn select_fluid_face(&mut self, face: FaceId) -> Result<(), WorkbenchError> {
        let f = self
            .geometry
            .face(face)
            .ok_or_else(|| invalid("Select a planar fluid face"))?;
        if !matches!(f.representation, GeometryFaceRepresentation::Planar { .. })
            || self.geometry.cad_owner(face).is_some()
        {
            return Err(invalid(
                "A 2D fluid domain must be an independent planar sketch, not a solid face",
            ));
        }
        let frame = self.geometry.planar_face_frame(face)?;
        if frame.normal.z.abs() < 1.0 - 1e-9 || frame.origin.z.abs() > 1e-9 {
            return Err(invalid(
                "The 2D solver requires a fluid profile in the XY plane at Z=0",
            ));
        }
        if self.fluid_face_selection != Some(face) {
            self.fluid_face_selection = Some(face);
            self.invalidate_mesh();
        }
        Ok(())
    }

    /// Horizontal rectangular channel, with any existing inner loops retained
    /// as obstacle walls. Conditions are explicit defaults and remain editable.
    pub fn prepare_channel_boundaries(&mut self, face: FaceId) -> Result<(), WorkbenchError> {
        let f = self
            .geometry
            .face(face)
            .ok_or_else(|| invalid("Select the rectangle with the circular hole"))?;
        let GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } = &f.representation
        else {
            return Err(invalid("Select a planar channel face"));
        };
        if outer_loop.len() != 4 || self.geometry.cad_owner(face).is_some() {
            return Err(invalid(
                "Channel preparation requires one axis-aligned rectangular sketch",
            ));
        }
        let mut lines = Vec::new();
        for e in outer_loop {
            let Some(crate::GeometryEdge {
                geometry: EdgeGeometry::Line { start, end },
                ..
            }) = self.geometry.edge(e.edge)
            else {
                return Err(invalid(
                    "The outer channel boundary must have four straight edges",
                ));
            };
            let a = self.geometry.vertex(*start).unwrap().position;
            let b = self.geometry.vertex(*end).unwrap().position;
            if a.z.abs() > 1e-9 || b.z.abs() > 1e-9 {
                return Err(invalid("The 2D channel must lie on the XY plane"));
            }
            lines.push((e.edge, a, b));
        }
        let min_x = lines
            .iter()
            .map(|(_, a, _)| a.x)
            .fold(f64::INFINITY, f64::min);
        let max_x = lines
            .iter()
            .map(|(_, a, _)| a.x)
            .fold(f64::NEG_INFINITY, f64::max);
        let eps = (max_x - min_x).abs().max(1.0) * 1e-9;
        let mut inlet = Vec::new();
        let mut outlet = Vec::new();
        let mut walls = Vec::new();
        for (edge, a, b) in lines {
            if (a.x - min_x).abs() <= eps && (b.x - min_x).abs() <= eps {
                inlet.push(edge);
            } else if (a.x - max_x).abs() <= eps && (b.x - max_x).abs() <= eps {
                outlet.push(edge);
            } else if (a.y - b.y).abs() <= eps {
                walls.push(edge);
            } else {
                return Err(invalid("Channel edges must be horizontal or vertical"));
            }
        }
        if inlet.len() != 1 || outlet.len() != 1 || walls.len() != 2 {
            return Err(invalid(
                "The outer channel is not an axis-aligned rectangle",
            ));
        }
        let mut groups = vec![("inlet", inlet), ("outlet", outlet), ("walls", walls)];
        let obstacles = inner_loops
            .iter()
            .flatten()
            .map(|e| e.edge)
            .collect::<Vec<_>>();
        if !obstacles.is_empty() {
            groups.push(("obstacle", obstacles));
        }
        let mut selections = self.named_selections.clone();
        for (name, edges) in &groups {
            let mut targets = edges
                .iter()
                .map(|id| GeometrySelectionTarget::Edge(*id))
                .collect::<Vec<_>>();
            targets.sort();
            if let Some(existing) = selections.get(name) {
                if existing.targets != targets {
                    return Err(invalid(format!("Named Selection {name:?} already contains different entities; rename or remove it first")));
                }
            } else {
                selections.create(name, targets, |_| true)?;
            }
        }
        // Validate grouping before publishing any state or clearing an old mesh.
        let export_groups = selections
            .iter()
            .map(|s| (s.name.as_str(), s.edges()))
            .collect::<Vec<_>>();
        GmshGeometryExporter::planar(&self.geometry, face, export_groups, "fluid")?;
        self.select_fluid_face(face)?;
        self.named_selections = selections;
        self.mesh_dimension = MeshDimension::TwoD;
        for (name, _) in groups {
            self.boundaries.entry(name.into()).or_insert(match name {
                "inlet" => IncompressibleBoundaryCondition::VelocityInlet {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
                "outlet" => IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
                _ => IncompressibleBoundaryCondition::NoSlipWall,
            });
        }
        self.invalidate_mesh();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn drawn_channel() -> WorkbenchSession {
        let mut session = WorkbenchSession::new();
        let mut editor = crate::GeometryEditorState::new();
        editor.snap_enabled = false;
        editor.transform.set_viewport(800.0, 500.0);
        editor.set_tool(crate::GeometryTool::Rectangle);
        for p in [(0.0, 0.0), (2.0, 1.0)] {
            editor
                .click(
                    session.geometry_mut(),
                    editor.transform.world_to_screen(p),
                    false,
                )
                .unwrap();
        }
        editor.set_tool(crate::GeometryTool::Circle);
        for p in [(0.5, 0.5), (0.65, 0.5)] {
            editor
                .click(
                    session.geometry_mut(),
                    editor.transform.world_to_screen(p),
                    false,
                )
                .unwrap();
        }
        session.geometry_changed();
        session
    }

    #[test]
    fn manual_boundary_groups_are_explicit_disjoint_and_transactional() {
        let mut s = WorkbenchSession::new();
        let r = s.add_rectangle(1.0, 1.0).unwrap();
        let edge = GeometrySelectionTarget::Edge;
        assert_eq!(s.boundary_geometry_targets().unwrap().len(), 4);
        assert!(s.patch_names().is_empty());
        s.set_boundary_group("top", vec![edge(r.top)], false)
            .unwrap();
        assert!(s.boundary_assignment("top").is_none());
        let snapshot = |session: &WorkbenchSession| {
            let mut value = serde_json::to_value(session.to_project("manual")).unwrap();
            value.as_object_mut().unwrap().remove("project_id");
            value
        };
        let before = snapshot(&s);
        assert!(s
            .set_boundary_group("overlap", vec![edge(r.top)], false)
            .is_err());
        assert!(s
            .set_boundary_group(
                "wrong-kind",
                vec![GeometrySelectionTarget::Face(r.face)],
                false
            )
            .is_err());
        assert!(s.set_boundary_group("top", vec![], true).is_err());
        assert_eq!(before, snapshot(&s));
        s.set_boundary_group(
            "fixed",
            vec![edge(r.left), edge(r.bottom), edge(r.right)],
            false,
        )
        .unwrap();
        assert!(s.mesh_generation_inputs().is_ok());
        assert!(s.to_project("manual").boundaries.is_empty());
        s.assign_boundary(
            "top",
            IncompressibleBoundaryCondition::MovingWall {
                velocity: Vec3::new(1.0, 0.0, 0.0),
            },
        )
        .unwrap();
        s.rename_named_selection("top", "moving_surface").unwrap();
        assert!(s.boundary_assignment("moving_surface").is_some());
        let project = s.to_project("manual");
        let restored = WorkbenchSession::from_project(&project).unwrap();
        assert_eq!(restored.patch_names(), s.patch_names());
        assert!(restored.boundary_assignment("fixed").is_none());
        assert!(s
            .set_boundary_group("fixed", vec![edge(r.top)], true)
            .is_err());
        s.set_boundary_group("fixed", vec![edge(r.left), edge(r.bottom)], true)
            .unwrap();
        assert!(s.mesh_generation_inputs().is_err());
    }

    #[test]
    fn manual_boundary_groups_support_3d_surface_zones() {
        let mut s = WorkbenchSession::new();
        s.add_box(1.0, 1.0, 1.0).unwrap();
        s.set_mesh_configuration(MeshDimension::ThreeD, 0.2, 0.1, 0.3, 1)
            .unwrap();
        let faces = s.boundary_geometry_targets().unwrap();
        assert_eq!(faces.len(), 6);
        assert!(faces
            .iter()
            .all(|t| matches!(t, GeometrySelectionTarget::Face(_))));
        s.set_boundary_group("external_surfaces", faces, false)
            .unwrap();
        assert!(s.mesh_generation_inputs().is_ok());
        assert!(s.to_project("manual volume").boundaries.is_empty());
    }

    #[test]
    #[ignore = "requires the Gmsh executable"]
    fn manual_boundary_groups_mesh_and_solve_closed_flow() {
        let mut s = WorkbenchSession::new();
        let r = s.add_rectangle(1.0, 1.0).unwrap();
        let edge = GeometrySelectionTarget::Edge;
        s.set_boundary_group("driven", vec![edge(r.top)], false)
            .unwrap();
        s.set_boundary_group(
            "stationary",
            vec![edge(r.left), edge(r.right), edge(r.bottom)],
            false,
        )
        .unwrap();
        s.set_mesh_configuration(MeshDimension::TwoD, 0.1, 0.05, 0.1, 1)
            .unwrap();
        let (export, options) = s.mesh_generation_inputs().unwrap();
        let generated = s.mesher().generate(&export.document, &options).unwrap();
        assert_eq!(generated.mesh.boundary_patches().len(), 2);
        let cells = generated.mesh.cell_count();
        s.install_mesh(generated);
        assert!(
            s.readiness().is_err(),
            "Meshing must not assign boundary physics"
        );
        s.assign_boundary(
            "driven",
            IncompressibleBoundaryCondition::MovingWall {
                velocity: Vec3::new(1.0, 0.0, 0.0),
            },
        )
        .unwrap();
        assert!(
            s.readiness().is_err(),
            "Every patch must be assigned explicitly"
        );
        s.assign_boundary("stationary", IncompressibleBoundaryCondition::NoSlipWall)
            .unwrap();
        s.set_material(1.0, 0.01).unwrap();
        s.set_solver_controls(1500, 0.7, 0.3, 1e-8).unwrap();
        s.readiness().unwrap();
        let solution = crate::solve_incompressible(&s.prepare_case().unwrap()).unwrap();
        eprintln!(
            "manual-closed-flow: cells={cells}, iterations={}, converged={}, continuity={:.3e}",
            solution.report.outer_iterations,
            solution.report.converged(),
            solution.report.final_continuity_rms
        );
        assert!(solution.report.converged());
        assert!(solution.report.final_continuity_rms.is_finite());
    }

    #[test]
    fn drawn_channel_preparation_preserves_hole_and_persists_conditions() {
        let mut s = drawn_channel();
        let face = s.fluid_face().unwrap();
        s.prepare_channel_boundaries(face).unwrap();
        assert_eq!(s.patch_names().len(), 4);
        assert_eq!(s.named_selections.get("obstacle").unwrap().edges().len(), 4);
        s.assign_boundary(
            "inlet",
            IncompressibleBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(0.2, 0.0, 0.0),
            },
        )
        .unwrap();
        s.prepare_channel_boundaries(face).unwrap();
        assert!(
            matches!(s.boundary_assignment("inlet"), Some(IncompressibleBoundaryCondition::VelocityInlet { velocity }) if velocity.x == 0.2)
        );
        let project = s.to_project("Drawn cylinder");
        project.validate().unwrap();
        let loaded: WorkbenchProject =
            serde_json::from_str(&serde_json::to_string(&project).unwrap()).unwrap();
        let restored = WorkbenchSession::from_project(&loaded).unwrap();
        assert_eq!(restored.fluid_face(), Some(face));
        assert_eq!(restored.patch_names(), s.patch_names());
        assert_eq!(
            restored.boundary_assignment("inlet"),
            s.boundary_assignment("inlet")
        );
        restored.mesh_generation_inputs().unwrap();
    }

    #[test]
    fn missing_and_overlapping_boundary_groups_fail_before_meshing() {
        let mut s = drawn_channel();
        assert!(s
            .mesh_generation_inputs()
            .unwrap_err()
            .to_string()
            .contains("Missing edges"));
        s.prepare_channel_boundaries(s.fluid_face().unwrap())
            .unwrap();
        let edges = s.named_selections.get("obstacle").unwrap().targets.clone();
        s.create_named_selection("duplicate", edges).unwrap();
        assert!(s
            .mesh_generation_inputs()
            .unwrap_err()
            .to_string()
            .contains("must not overlap"));
    }

    #[test]
    fn circle_cut_rejects_tangency_overlap_and_outside_without_mutation() {
        let mut s = drawn_channel();
        let face = s.fluid_face().unwrap();
        let before = serde_json::to_value(s.geometry()).unwrap();
        for (center, radius) in [
            (Vec3::new(0.05, 0.5, 0.0), 0.05),
            (Vec3::new(0.6, 0.5, 0.0), 0.1),
            (Vec3::new(0.5, 0.5, 0.0), 0.01),
            (Vec3::new(3.0, 0.5, 0.0), 0.1),
        ] {
            assert!(s
                .geometry_mut()
                .add_circle_hole(face, center, radius)
                .is_err());
            assert_eq!(serde_json::to_value(s.geometry()).unwrap(), before);
        }
        let mut solid = GeometryTopology::new();
        let rectangle = solid.add_rectangle(2.0, 1.0).unwrap();
        solid.extrude_planar_face(rectangle.face, 0.2).unwrap();
        let before = serde_json::to_value(&solid).unwrap();
        assert!(solid
            .add_circle_hole(rectangle.face, Vec3::new(0.5, 0.5, 0.0), 0.1)
            .is_err());
        assert_eq!(serde_json::to_value(&solid).unwrap(), before);
    }

    #[test]
    fn multiple_profiles_require_explicit_domain_selection() {
        let mut s = WorkbenchSession::new();
        let a = s.geometry_mut().add_rectangle(2.0, 1.0).unwrap();
        s.geometry_mut().add_rectangle(3.0, 2.0).unwrap();
        assert_eq!(s.fluid_face(), None);
        s.select_fluid_face(a.face).unwrap();
        assert_eq!(s.fluid_face(), Some(a.face));
    }

    #[test]
    #[ignore = "Requires local Gmsh; exercises the actual draw-to-solve pipeline"]
    fn gmsh_cylinder_draw_mesh_boundary_solve() {
        let mut s = drawn_channel();
        s.prepare_channel_boundaries(s.fluid_face().unwrap())
            .unwrap();
        s.assign_boundary(
            "inlet",
            IncompressibleBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(0.1, 0.0, 0.0),
            },
        )
        .unwrap();
        s.set_material(1.0, 0.1).unwrap();
        s.set_solver_controls(1000, 0.5, 0.3, 1e-7).unwrap();
        s.set_mesh_configuration(MeshDimension::TwoD, 0.15, 0.04, 0.2, 1)
            .unwrap();
        let (export, options) = s.mesh_generation_inputs().unwrap();
        let generated = s.mesher().generate(&export.document, &options).unwrap();
        let cells = generated.mesh.cell_count();
        let area: f64 = generated.mesh.cells().iter().map(|c| c.volume).sum();
        assert!((area - (2.0 - std::f64::consts::PI * 0.15_f64.powi(2))).abs() < 0.01);
        assert!(generated
            .mesh
            .cells()
            .iter()
            .all(|c| (c.center - Vec3::new(0.5, 0.5, 0.0)).norm() > 0.14));
        s.install_mesh(generated);
        s.readiness().unwrap();
        let solution = crate::solve_incompressible(&s.prepare_case().unwrap()).unwrap();
        eprintln!("drawn-cylinder: cells={cells}, area={area:.8}, iterations={}, converged={}, continuity={:.3e}, inflow={:.8e}, outflow={:.8e}, net_flux={:.3e}", solution.report.outer_iterations, solution.report.converged(), solution.report.final_continuity_rms, solution.report.total_inflow, solution.report.total_outflow, solution.report.net_boundary_flux);
        eprintln!(
            "momentum_relative={:.3e}, velocity_relative={:.3e}",
            solution.report.simple.final_momentum_relative_residual,
            solution.report.simple.final_velocity_relative_change
        );
        assert!(solution.report.converged());
        assert!(
            solution.report.outer_iterations > 1,
            "Pressure projection alone must not declare steady convergence"
        );
        assert!(solution.report.simple.final_momentum_relative_residual < 1e-6);
        assert!(solution.report.simple.final_velocity_relative_change < 1e-6);
        assert!(solution.report.net_boundary_flux.abs() < 1e-5);
        s.complete_solve(Ok(solution));
        s.rename_named_selection("obstacle", "cylinder").unwrap();
        assert!(!s.has_mesh());
        assert!(s.solution().is_none());
        assert!(s.boundary_assignment("cylinder").is_some());
    }
}
