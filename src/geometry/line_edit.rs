//! Transactional edits of independent sketch lines. Solid/profile boundaries
//! must go through their parametric feature rather than being detached here.

use super::*;

impl GeometryTopology {
    pub(crate) fn replace_open_line(
        &mut self,
        id: EdgeId,
        segments: &[(Vec3, Vec3)],
    ) -> Result<Vec<EdgeId>, GeometryError> {
        let edge = self.edge(id).ok_or(GeometryError::InvalidPrimitive {
            message: "The selected line no longer exists".into(),
        })?;
        let EdgeGeometry::Line { start, end } = edge.geometry else {
            return Err(GeometryError::InvalidPrimitive {
                message: "This tool requires a straight sketch line".into(),
            });
        };
        // Reuse the canonical dependency guards, even when retaining the ID.
        let mut next = self.clone();
        next.remove_edge(id).map_err(|error| match error {
            GeometryError::EntityInUse { .. } => GeometryError::InvalidPrimitive {
                message: "This line belongs to a profile or solid. Edit its source sketch; Trim, Extend and Split currently edit independent XY lines".into(),
            },
            other => other,
        })?;
        if segments.is_empty() {
            return Err(GeometryError::InvalidPrimitive {
                message: "The edit must retain at least one line segment".into(),
            });
        }
        let mut result = Vec::with_capacity(segments.len());
        for (index, &(a, b)) in segments.iter().enumerate() {
            let mut vertex_at = |point: Vec3| -> Result<VertexId, GeometryError> {
                let existing = next
                    .vertices()
                    .find(|v| {
                        next.is_vertex_visible(v.id)
                            && next.cad_vertex_owner(v.id).is_none()
                            && (v.position - point).norm() <= 1.0e-9
                    })
                    .map(|v| v.id);
                match existing {
                    Some(vertex) => Ok(vertex),
                    None => next.add_vertex(point),
                }
            };
            let a = vertex_at(a)?;
            let b = vertex_at(b)?;
            next.validate_line(a, b)?;
            let segment = if index == 0 { id } else { next.allocate_edge() };
            next.edges.insert(
                segment,
                GeometryEdge {
                    id: segment,
                    geometry: EdgeGeometry::Line { start: a, end: b },
                },
            );
            result.push(segment);
        }
        for vertex in [start, end] {
            if !next
                .edges()
                .any(|edge| edge_vertices(&edge.geometry).contains(&vertex))
            {
                next.remove_vertex(vertex)?;
            }
        }
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(result)
    }
}
