//! Persisted sketch visibility; consumed profiles must not cap a Pocket visually.
use super::*;

impl GeometryTopology {
    pub fn cad_set_sketch_visible(
        &mut self,
        id: CadSketchId,
        visible: bool,
    ) -> Result<(), GeometryError> {
        let face = self
            .sketch(id)
            .and_then(|s| s.profile_face)
            .ok_or_else(|| error("Sketch has no profile"))?;
        if visible {
            self.hidden_profiles.remove(&face);
        } else {
            self.hidden_profiles.insert(face);
        }
        self.bump_revision();
        Ok(())
    }

    pub fn is_face_visible(&self, id: FaceId) -> bool {
        !self.hidden_profiles.contains(&id)
            && self
                .cad_owner(id)
                .and_then(|b| self.cad_body(b))
                .is_none_or(|b| b.visible)
    }

    pub fn is_edge_visible(&self, id: EdgeId) -> bool {
        let mut hidden = false;
        for face in self.faces.values() {
            if let GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } = &face.representation
            {
                if outer_loop
                    .iter()
                    .chain(inner_loops.iter().flatten())
                    .any(|e| e.edge == id)
                {
                    if self.is_face_visible(face.id) {
                        return true;
                    }
                    hidden = true;
                }
            }
        }
        !hidden
    }

    pub fn is_vertex_visible(&self, id: VertexId) -> bool {
        let mut hidden = false;
        for edge in self.edges.values() {
            let vertices = match edge.geometry {
                EdgeGeometry::Line { start, end } => vec![start, end],
                EdgeGeometry::CircularArc { start, center, end } => vec![start, center, end],
            };
            if vertices.contains(&id) {
                if self.is_edge_visible(edge.id) {
                    return true;
                }
                hidden = true;
            }
        }
        !hidden
    }
}
