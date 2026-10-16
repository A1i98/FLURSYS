//! Kernel sub-elements share the application's normal selection IDs. Picking
//! uses bounded curve samples; measurements integrate the exact curve derivative.
use super::*;
use truck_modeling::{BoundedCurve, ParameterDivision1D, ParametricCurve};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadEdge {
    pub id: EdgeId,
    pub start: VertexId,
    pub end: VertexId,
    pub faces: Vec<FaceId>,
    pub path: Vec<Vec3>,
    pub length: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadVertex {
    pub id: VertexId,
    pub faces: Vec<FaceId>,
}

impl GeometryTopology {
    /// Upgrade documents saved before kernel sub-element selection was available.
    pub fn ensure_cad_subelements(&mut self) -> Result<bool, GeometryError> {
        let ids = self
            .cad_bodies
            .iter()
            .filter_map(|(id, b)| (b.edges.is_empty() || b.vertices.is_empty()).then_some(*id))
            .collect::<Vec<_>>();
        if ids.is_empty() {
            return Ok(false);
        }
        let mut next = self.clone();
        for id in ids {
            next.refresh_subelements(id)?;
        }
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(true)
    }

    pub fn cad_edge(&self, id: EdgeId) -> Option<&CadEdge> {
        self.cad_bodies
            .values()
            .find_map(|body| body.edges.iter().find(|edge| edge.id == id))
    }
    pub fn cad_edge_owner(&self, id: EdgeId) -> Option<BodyId> {
        self.cad_bodies
            .iter()
            .find_map(|(body_id, body)| body.edges.iter().any(|e| e.id == id).then_some(*body_id))
    }
    pub fn cad_vertex_owner(&self, id: VertexId) -> Option<BodyId> {
        self.cad_bodies.iter().find_map(|(body_id, body)| {
            body.vertices.iter().any(|v| v.id == id).then_some(*body_id)
        })
    }
    pub(super) fn discard_subelements(&mut self, body: &CadBody) {
        for edge in body.edges.iter() {
            self.edges.remove(&edge.id);
        }
        for vertex in body.vertices.iter() {
            self.vertices.remove(&vertex.id);
        }
    }
    pub(super) fn refresh_subelements(&mut self, id: BodyId) -> Result<(), GeometryError> {
        let solid = self.cad_solid(id)?;
        let old = self.cad_bodies[&id].clone();
        let owned_edges = old.edges.iter().map(|e| e.id).collect::<BTreeSet<_>>();
        let owned_vertices = old.vertices.iter().map(|v| v.id).collect::<BTreeSet<_>>();
        if self.edges.values().any(|e| {
            !owned_edges.contains(&e.id)
                && super::super::edge_vertices(&e.geometry)
                    .iter()
                    .any(|v| owned_vertices.contains(v))
        }) {
            return Err(error("External sketch geometry references kernel vertices; use an independent projected sketch before changing this body"));
        }
        let kernel_vertices = solid.vertex_iter().collect::<Vec<_>>();
        let mut vertex_faces = std::collections::HashMap::new();
        let mut edge_faces = std::collections::HashMap::new();
        for (face, display) in solid.face_iter().zip(old.face_meshes.iter()) {
            for edge in face.boundary_iters().into_iter().flatten() {
                edge_faces
                    .entry(edge.id())
                    .or_insert_with(BTreeSet::new)
                    .insert(display.face);
                for vertex in [edge.front(), edge.back()] {
                    vertex_faces
                        .entry(vertex.id())
                        .or_insert_with(BTreeSet::new)
                        .insert(display.face);
                }
            }
        }
        let mut points = std::collections::HashMap::new();
        let mut vertices = Vec::new();
        let mut positions = Vec::new();
        let mut reused_vertices = BTreeSet::new();
        for vertex in kernel_vertices {
            let faces = vertex_faces
                .get(&vertex.id())
                .map(|s| s.iter().copied().collect::<Vec<_>>())
                .unwrap_or_default();
            let candidates = old
                .vertices
                .iter()
                .filter(|v| v.faces == faces)
                .collect::<Vec<_>>();
            let multiplicity = vertex_faces
                .values()
                .filter(|s| s.iter().copied().collect::<Vec<_>>() == faces)
                .count();
            let stable = if candidates.len() == 1 && multiplicity == 1 {
                Some(candidates[0].id)
            } else {
                None
            };
            let vertex_id = stable.unwrap_or_else(|| self.allocate_vertex());
            reused_vertices.insert(vertex_id);
            points.insert(vertex.id(), vertex_id);
            positions.push((vertex_id, vec3(vertex.point())));
            vertices.push(CadVertex {
                id: vertex_id,
                faces,
            });
        }
        let diagonal = positions
            .iter()
            .map(|(_, p)| *p)
            .fold(None::<(Vec3, Vec3)>, |bounds, p| {
                Some(match bounds {
                    None => (p, p),
                    Some((a, b)) => (
                        Vec3::new(a.x.min(p.x), a.y.min(p.y), a.z.min(p.z)),
                        Vec3::new(b.x.max(p.x), b.y.max(p.y), b.z.max(p.z)),
                    ),
                })
            })
            .map_or(1.0, |(a, b)| (b - a).norm());
        let mut edges = Vec::new();
        let mut reused_edges = BTreeSet::new();
        for edge in solid.edge_iter() {
            let faces = edge_faces[&edge.id()].iter().copied().collect::<Vec<_>>();
            let start = points[&edge.front().id()];
            let end = points[&edge.back().id()];
            let candidates = old
                .edges
                .iter()
                .filter(|e| {
                    e.faces == faces
                        && ((e.start == start && e.end == end)
                            || (e.start == end && e.end == start))
                })
                .collect::<Vec<_>>();
            let edge_id = if candidates.len() == 1 && reused_edges.insert(candidates[0].id) {
                candidates[0].id
            } else {
                self.allocate_edge()
            };
            reused_edges.insert(edge_id);
            let curve = edge.oriented_curve();
            let range = curve.range_tuple();
            let (_, path) = curve.parameter_division(range, (diagonal * 0.0005).max(1e-6));
            if path.len() < 2 || path.len() > 8192 {
                return Err(error("Kernel edge exceeds picking resolution limits"));
            }
            let path = path.into_iter().map(vec3).collect::<Vec<_>>();
            for p in &path {
                finite(*p)?;
            }
            let n = 1024;
            let h = (range.1 - range.0) / n as f64;
            let mut integral = 0.0;
            for i in 0..=n {
                let speed = curve.der(range.0 + h * i as f64).magnitude();
                integral += speed
                    * if i == 0 || i == n {
                        1.0
                    } else if i % 2 == 0 {
                        2.0
                    } else {
                        4.0
                    };
            }
            let length = integral * h.abs() / 3.0;
            if !length.is_finite() || length <= 0.0 {
                return Err(error("Invalid exact edge curve length"));
            }
            edges.push(CadEdge {
                id: edge_id,
                start,
                end,
                faces,
                path,
                length,
            });
        }
        // Publish after every curve has been prepared successfully.
        self.discard_subelements(&old);
        for (id, position) in positions {
            self.vertices.insert(id, GeometryVertex { id, position });
        }
        for edge in &edges {
            self.edges.insert(
                edge.id,
                GeometryEdge {
                    id: edge.id,
                    geometry: EdgeGeometry::Kernel {
                        id: edge.id,
                        start: edge.start,
                        end: edge.end,
                    },
                },
            );
        }
        let body = self.cad_bodies.get_mut(&id).unwrap();
        body.edges = Arc::new(edges);
        body.vertices = Arc::new(vertices);
        Ok(())
    }
}
