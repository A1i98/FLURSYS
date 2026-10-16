//! Stable per-region records grouped by a shared sketch support.
use super::*;
impl GeometryTopology {
    pub fn cad_sketch_owner(&self, id: CadSketchId) -> Result<CadSketchId, GeometryError> {
        let s = self
            .sketch(id)
            .ok_or_else(|| error("Sketch region not found"))?;
        Ok(s.region_owner.unwrap_or(id))
    }
    pub fn cad_sketch_region_count(&self, id: CadSketchId) -> usize {
        let Ok(owner) = self.cad_sketch_owner(id) else {
            return 0;
        };
        self.sketches()
            .filter(|s| s.id == owner || s.region_owner == Some(owner))
            .count()
    }
    /// Regions retain their own editable dimensions and feature references.
    /// Intersections are not implicitly fused; each closed contour is selectable.
    pub fn cad_add_sketch_region(
        &mut self,
        owner: CadSketchId,
        x: f64,
        y: f64,
        profile: CadSketchProfile,
    ) -> Result<(CadSketch, FaceId), GeometryError> {
        let mut next = self.clone();
        let owner = next.cad_sketch_owner(owner)?;
        let root = next.sketch(owner).unwrap().clone();
        let region = next.insert_sketch(root.plane, root.frame, root.host_face)?;
        let s = next.sketches.get_mut(&region.id).unwrap();
        s.region_owner = Some(owner);
        s.attachment_offset = root.attachment_offset;
        let face = match profile {
            CadSketchProfile::Rectangle { width, height } => {
                next.materialize_sketch_rectangle_at(region.id, x, y, width, height)?
            }
            CadSketchProfile::Circle { radius } => {
                next.materialize_sketch_circle_at(region.id, x, y, radius)?
            }
        };
        next.validate()?;
        let record = next.sketch(region.id).unwrap().clone();
        *self = next;
        Ok((record, face))
    }
}

impl GeometryTopology {
    /// Give a directly drawn XY region persistent sketch intent without
    /// replacing its face/edge/vertex IDs or changing the existing geometry.
    pub fn cad_register_xy_region(&mut self, face: FaceId) -> Result<CadSketchId, GeometryError> {
        if let Some(s) = self.sketches().find(|s| s.profile_face == Some(face)) {
            return Ok(s.id);
        }
        self.cad_validate_profile(face)?;
        if self.cad_owner(face).is_some() {
            return Err(error("Generated solid faces require an attached sketch"));
        }
        let points = self
            .profile_vertices(face)?
            .into_iter()
            .map(|id| self.vertices[&id].position)
            .collect::<Vec<_>>();
        if points.iter().any(|p| p.z.abs() > 1e-9) {
            return Err(error("Direct drawing registration requires the XY plane"));
        }
        let edges = self.profile_edge_ids(face)?.into_iter().collect::<Vec<_>>();
        let mut profile = None;
        if edges.len() == 4
            && edges
                .iter()
                .all(|id| matches!(self.edges[id].geometry, EdgeGeometry::CircularArc { .. }))
        {
            if let EdgeGeometry::CircularArc { start, center, .. } = self.edges[&edges[0]].geometry
            {
                let c = self.vertices[&center].position;
                let radius = (self.vertices[&start].position - c).norm();
                if edges.iter().all(|id| match self.edges[id].geometry {
                    EdgeGeometry::CircularArc {
                        start,
                        center: other,
                        end,
                    } => {
                        other == center
                            && [start, end].iter().all(|v| {
                                ((self.vertices[v].position - c).norm() - radius).abs()
                                    <= 1e-8 * radius.max(1.0)
                            })
                    }
                    _ => false,
                }) {
                    profile = Some(CadSketchProfile::Circle { radius });
                }
            }
        } else if edges.len() == 4
            && points.len() == 4
            && edges
                .iter()
                .all(|id| matches!(self.edges[id].geometry, EdgeGeometry::Line { .. }))
        {
            let xmin = points.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
            let xmax = points.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
            let ymin = points.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
            let ymax = points.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
            let width = xmax - xmin;
            let height = ymax - ymin;
            if width > GEOMETRY_EPSILON
                && height > GEOMETRY_EPSILON
                && points.iter().all(|p| {
                    ((p.x - xmin).abs() < 1e-9 || (p.x - xmax).abs() < 1e-9)
                        && ((p.y - ymin).abs() < 1e-9 || (p.y - ymax).abs() < 1e-9)
                })
                && edges.iter().all(|id| match self.edges[id].geometry {
                    EdgeGeometry::Line { start, end } => {
                        let d = self.vertices[&end].position - self.vertices[&start].position;
                        d.x.abs() < 1e-9 || d.y.abs() < 1e-9
                    }
                    _ => false,
                })
            {
                profile = Some(CadSketchProfile::Rectangle { width, height });
            }
        }
        let mut next = self.clone();
        let id = next.create_sketch_on_plane(CadSketchPlane::Xy)?.id;
        let s = next.sketches.get_mut(&id).unwrap();
        s.profile_face = Some(face);
        s.profile = profile;
        next.validate()?;
        *self = next;
        Ok(id)
    }
}
