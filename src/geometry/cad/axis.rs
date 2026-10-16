//! Axis references use current geometry, so dimensional edits move the axis too.
use super::*;
impl GeometryTopology {
    pub fn cad_reference_axis(
        &self,
        reference: CadAxisReference,
    ) -> Result<(Vec3, Vec3), GeometryError> {
        match reference {
            CadAxisReference::ProfileX(face) | CadAxisReference::ProfileY(face) => {
                let frame =
                    if let Some(sketch) = self.sketches().find(|s| s.profile_face == Some(face)) {
                        sketch.frame
                    } else {
                        self.sketch_frame_for_face(face)?
                    };
                Ok((
                    frame.origin,
                    if matches!(reference, CadAxisReference::ProfileX(_)) {
                        frame.x_axis
                    } else {
                        frame.y_axis
                    },
                ))
            }
            CadAxisReference::Edge(id) => {
                let edge = self
                    .edge(id)
                    .ok_or_else(|| error("Axis edge no longer exists"))?;
                let (start, end) = match edge.geometry {
                    EdgeGeometry::Line { start, end } => (start, end),
                    EdgeGeometry::Kernel { start, end, .. } => {
                        let curve = self
                            .cad_edge(id)
                            .ok_or_else(|| error("Missing axis curve"))?;
                        let a = self
                            .vertex(start)
                            .ok_or_else(|| error("Missing axis endpoint"))?
                            .position;
                        let b = self
                            .vertex(end)
                            .ok_or_else(|| error("Missing axis endpoint"))?
                            .position;
                        let d = b - a;
                        if d.norm() < 1e-12
                            || curve.path.iter().any(|p| {
                                (*p - a).cross(d).norm() > 1e-8 * d.norm().max(1.0) * d.norm()
                            })
                        {
                            return Err(error("Choose a straight edge; arcs and curved edges cannot define a revolution axis"));
                        }
                        (start, end)
                    }
                    _ => return Err(error("Choose a straight line, not an arc")),
                };
                let a = self
                    .vertex(start)
                    .ok_or_else(|| error("Missing axis endpoint"))?
                    .position;
                let b = self
                    .vertex(end)
                    .ok_or_else(|| error("Missing axis endpoint"))?
                    .position;
                if (b - a).norm() < 1e-12 {
                    return Err(error("Axis line has zero length"));
                }
                Ok((a, b - a))
            }
        }
    }
    pub(super) fn add_axis_dependencies(
        &self,
        d: &mut CadDependencies,
        reference: CadAxisReference,
    ) -> Result<(), GeometryError> {
        self.cad_reference_axis(reference)?;
        match reference {
            CadAxisReference::ProfileX(face) | CadAxisReference::ProfileY(face) => {
                if !d.profiles.contains(&face) {
                    d.profiles.push(face);
                }
            }
            CadAxisReference::Edge(edge) => {
                if let Some(body) = self.cad_edge_owner(edge) {
                    if !d.bodies.contains(&body) {
                        d.bodies.push(body);
                    }
                } else {
                    let face = self
                        .sketches()
                        .filter_map(|s| s.profile_face)
                        .find(|face| {
                            self.face(*face).is_some_and(|f| match &f.representation {
                                GeometryFaceRepresentation::Planar {
                                    outer_loop,
                                    inner_loops,
                                } => outer_loop
                                    .iter()
                                    .chain(inner_loops.iter().flatten())
                                    .any(|e| e.edge == edge),
                                _ => false,
                            })
                        })
                        .ok_or_else(|| error("Axis line must belong to a saved sketch or solid"))?;
                    if !d.profiles.contains(&face) {
                        d.profiles.push(face);
                    }
                }
            }
        }
        Ok(())
    }
    pub fn cad_revolve_with_reference(
        &mut self,
        face: FaceId,
        reference: CadAxisReference,
        degrees: f64,
        reversed: bool,
    ) -> Result<BodyId, GeometryError> {
        let (pivot, mut axis) = self.cad_reference_axis(reference)?;
        let mut dependencies = CadDependencies {
            bodies: Vec::new(),
            profiles: vec![face],
            replayable_pad: false,
        };
        self.add_axis_dependencies(&mut dependencies, reference)?;
        if reversed {
            axis = axis * -1.0;
        }
        let id = self.cad_revolve(face, pivot, axis, degrees)?;
        let body = self.cad_bodies.get_mut(&id).unwrap();
        body.dependencies = Some(dependencies);
        body.recipe = Some(CadRecipe::Revolve {
            face,
            pivot,
            axis,
            degrees,
            axis_reference: Some(reference),
            axis_reversed: reversed,
        });
        Ok(id)
    }
}

impl GeometryTopology {
    /// Check that a selectable region is a closed planar wire before a feature
    /// task sends it to the solid kernel. Open construction lines are excluded.
    pub fn cad_validate_profile(&self, face: FaceId) -> Result<(), GeometryError> {
        match &self.require_face(face)?.representation {
            GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } => {
                if outer_loop.is_empty() {
                    return Err(error("Choose a closed region, not an open line"));
                }
                self.validate_loop(outer_loop)?;
                for inner in inner_loops {
                    self.validate_loop(inner)?;
                }
                self.planar_face_frame(face)?;
                Ok(())
            }
            _ => Err(error(
                "Create a closed sketch region on the solid face first",
            )),
        }
    }
}
