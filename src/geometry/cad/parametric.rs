//! Transactional dimension editing for circle/rectangle sketches and feature graphs.
//! Unsupported consumers stop regeneration rather than keeping stale solids.
use super::*;
use std::collections::BTreeSet;

impl GeometryTopology {
    pub fn cad_sketch_parameters(
        &self,
        id: CadSketchId,
    ) -> Result<(f64, f64, CadSketchProfile), GeometryError> {
        let s = self.sketch(id).ok_or_else(|| error("Sketch not found"))?;
        let face = s
            .profile_face
            .ok_or_else(|| error("Sketch has no profile"))?;
        let profile = s
            .profile
            .clone()
            .ok_or_else(|| error("Sketch has no dimensional profile"))?;
        let vertices = self.profile_vertices(face)?;
        if vertices.iter().any(|id| {
            (self.vertices[id].position - s.frame.origin)
                .dot(s.frame.normal)
                .abs()
                > 1e-7
        }) {
            return Err(error(
                "Sketch geometry is outside its saved plane; resolve external edits first",
            ));
        }
        let local = |p: Vec3| {
            let d = p - s.frame.origin;
            (d.dot(s.frame.x_axis), d.dot(s.frame.y_axis))
        };
        let edges = self.profile_edge_ids(face)?;
        let tolerance = 1e-7;
        let (x, y) = match profile {
            CadSketchProfile::Circle { radius } => {
                let center_id = edges
                    .iter()
                    .find_map(|id| {
                        if let EdgeGeometry::CircularArc { center, .. } = self.edges[id].geometry {
                            Some(center)
                        } else {
                            None
                        }
                    })
                    .ok_or_else(|| error("Circle topology is inconsistent"))?;
                let center = self.vertices[&center_id].position;
                if edges.iter().any(|id| match self.edges[id].geometry {
                    EdgeGeometry::CircularArc {
                        start,
                        center: other,
                        end,
                    } => {
                        other != center_id
                            || [start, end].iter().any(|v| {
                                ((self.vertices[v].position - center).norm() - radius).abs()
                                    > tolerance * radius.max(1.0)
                            })
                    }
                    _ => true,
                }) {
                    return Err(error("Circle geometry differs from its saved radius; resolve external edits first"));
                }
                local(center)
            }
            CadSketchProfile::Rectangle { width, height } => {
                let bounds = vertices
                    .iter()
                    .map(|id| local(self.vertices[id].position))
                    .fold(
                        (
                            f64::INFINITY,
                            f64::INFINITY,
                            f64::NEG_INFINITY,
                            f64::NEG_INFINITY,
                        ),
                        |a, b| (a.0.min(b.0), a.1.min(b.1), a.2.max(b.0), a.3.max(b.1)),
                    );
                if (bounds.2 - bounds.0 - width).abs() > tolerance * width.max(1.0)
                    || (bounds.3 - bounds.1 - height).abs() > tolerance * height.max(1.0)
                {
                    return Err(error("Rectangle geometry differs from its saved dimensions; resolve external edits first"));
                }
                (bounds.0, bounds.1)
            }
        };
        if s.locked_position.is_some_and(|p| {
            !p[0].is_finite()
                || !p[1].is_finite()
                || (p[0] - x).abs() > 1e-7
                || (p[1] - y).abs() > 1e-7
        }) || s.locked_profile.as_ref().is_some_and(|p| p != &profile)
        {
            return Err(error("Saved sketch constraints do not match its geometry"));
        }
        Ok((x, y, profile))
    }

    /// Retains sketch, vertex, edge, face, feature and body IDs. On any error,
    /// neither geometry nor allocator/revision state is published.
    pub fn cad_edit_sketch(
        &mut self,
        id: CadSketchId,
        x: f64,
        y: f64,
        profile: CadSketchProfile,
    ) -> Result<Vec<BodyId>, GeometryError> {
        if !x.is_finite() || !y.is_finite() {
            return Err(error("Sketch position must be finite"));
        }
        let (x, y, profile) = self.cad_resolve_sketch_parameters(id, x, y, profile)?;
        let (old_x, old_y, old) = self.cad_sketch_parameters(id)?;
        let record = self.sketch(id).unwrap().clone();
        let face = record.profile_face.unwrap();
        let vertices = self.profile_vertices(face)?;
        let edges = self.profile_edge_ids(face)?;
        if edges.len() != 4 {
            return Err(error(
                "Dimension editing requires a four-edge circle or rectangle",
            ));
        }
        let positive = |v: f64| v.is_finite() && v > GEOMETRY_EPSILON;
        let (sx, sy) = match (&old, &profile) {
            (CadSketchProfile::Circle { radius: a }, CadSketchProfile::Circle { radius: b })
                if positive(*a) && positive(*b) =>
            {
                (*b / *a, *b / *a)
            }
            (
                CadSketchProfile::Rectangle {
                    width: a,
                    height: b,
                },
                CadSketchProfile::Rectangle {
                    width: c,
                    height: d,
                },
            ) if positive(*a) && positive(*b) && positive(*c) && positive(*d) => (*c / *a, *d / *b),
            _ => {
                return Err(error(
                    "Keep the profile type and use finite, positive dimensions",
                ))
            }
        };
        if edges.iter().any(|id| {
            !matches!(
                (&old, &self.edges[id].geometry),
                (
                    CadSketchProfile::Circle { .. },
                    EdgeGeometry::CircularArc { .. }
                ) | (
                    CadSketchProfile::Rectangle { .. },
                    EdgeGeometry::Line { .. }
                )
            )
        }) {
            return Err(error(
                "Profile geometry no longer matches its dimensional definition",
            ));
        }
        // Shared vertices/edges must not move another independent profile.
        if self.faces.values().any(|f| {
            f.id != face
                && match &f.representation {
                    GeometryFaceRepresentation::Planar {
                        outer_loop,
                        inner_loops,
                    } => outer_loop
                        .iter()
                        .chain(inner_loops.iter().flatten())
                        .any(|e| edges.contains(&e.edge)),
                    _ => false,
                }
        }) || self.edges.values().any(|e| {
            !edges.contains(&e.id) && edge_vertex_ids(e).iter().any(|v| vertices.contains(v))
        }) {
            return Err(error(
                "Profile shares geometry with another entity; detach it before dimensional editing",
            ));
        }
        let mut next = self.clone();
        let f = record.frame;
        for vertex in vertices {
            let d = next.vertices[&vertex].position - f.origin;
            let px = x + (d.dot(f.x_axis) - old_x) * sx;
            let py = y + (d.dot(f.y_axis) - old_y) * sy;
            let position = f.origin + f.x_axis * px + f.y_axis * py;
            finite(position)?;
            next.vertices.get_mut(&vertex).unwrap().position = position;
        }
        next.sketches.get_mut(&id).unwrap().profile = Some(profile);
        let result = next.regenerate_features(id)?;
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(result)
    }

    pub fn cad_edit_pad(
        &mut self,
        id: CadFeatureId,
        distance: f64,
    ) -> Result<Vec<BodyId>, GeometryError> {
        if !distance.is_finite() || distance.abs() <= GEOMETRY_EPSILON {
            return Err(error("Pad depth must be finite and nonzero"));
        }
        let mut next = self.clone();
        let feature = next
            .extrude_features
            .get_mut(&id)
            .ok_or_else(|| error("Pad not found"))?;
        let sketch = feature.sketch;
        feature.distance = Some(distance);
        let result_body = feature.body;
        let source_face = feature.source_face;
        let additive = feature.additive;
        let tool = if additive {
            next.cad_body(result_body)
                .and_then(|b| b.dependencies.as_ref())
                .and_then(|d| {
                    d.bodies.iter().copied().find(|id| {
                        next.cad_body(*id).is_some_and(|b| {
                            b.dependencies
                                .as_ref()
                                .is_some_and(|d| d.profiles.contains(&source_face))
                        })
                    })
                })
                .ok_or_else(|| error("Additive Pad has no saved extrusion tool"))?
        } else {
            result_body
        };
        if let Some(recipe) = next
            .cad_bodies
            .get_mut(&tool)
            .and_then(|b| b.recipe.as_mut())
        {
            fn set_depth(recipe: &mut CadRecipe, face: FaceId, depth: f64) -> bool {
                match recipe {
                    CadRecipe::Extrude { face: f, distance } if *f == face => {
                        *distance = depth;
                        true
                    }
                    CadRecipe::Modified { source, .. } => set_depth(source, face, depth),
                    _ => false,
                }
            }
            if !set_depth(recipe, source_face, distance) {
                return Err(error("Feature has no editable Pad depth recipe"));
            }
        }
        let result = next.regenerate_features(sketch)?;
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(result)
    }

    pub(super) fn profile_edge_ids(&self, face: FaceId) -> Result<BTreeSet<EdgeId>, GeometryError> {
        match &self.require_face(face)?.representation {
            GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } if inner_loops.is_empty() => Ok(outer_loop.iter().map(|e| e.edge).collect()),
            _ => Err(error(
                "Dimension editing currently requires a single closed profile without holes",
            )),
        }
    }

    pub(super) fn profile_vertices(
        &self,
        face: FaceId,
    ) -> Result<BTreeSet<VertexId>, GeometryError> {
        Ok(self
            .profile_edge_ids(face)?
            .iter()
            .flat_map(|id| edge_vertex_ids(&self.edges[id]))
            .collect())
    }
}

fn edge_vertex_ids(edge: &GeometryEdge) -> Vec<VertexId> {
    match edge.geometry {
        EdgeGeometry::Line { start, end } | EdgeGeometry::Kernel { start, end, .. } => {
            vec![start, end]
        }
        EdgeGeometry::CircularArc { start, center, end } => vec![start, center, end],
    }
}
