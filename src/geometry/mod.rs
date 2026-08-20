//! Stable, solver-independent geometry topology for preprocessing.
//!
//! Entity IDs are model-local opaque handles. They are distinct from Gmsh tags
//! and mesh indices, and are allocated monotonically without reuse.

use crate::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

const GEOMETRY_EPSILON: f64 = 1.0e-12;

macro_rules! geometry_id {
    ($name:ident) => {
        #[derive(
            Clone,
            Copy,
            Debug,
            Default,
            PartialEq,
            Eq,
            PartialOrd,
            Ord,
            Hash,
            Serialize,
            Deserialize,
        )]
        pub struct $name(u64);

        impl $name {
            pub const fn get(self) -> u64 {
                self.0
            }
        }
    };
}

geometry_id!(VertexId);
geometry_id!(EdgeId);
geometry_id!(FaceId);
geometry_id!(BodyId);
geometry_id!(GeometryRevision);

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryVertex {
    pub id: VertexId,
    pub position: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum EdgeGeometry {
    Line {
        start: VertexId,
        end: VertexId,
    },
    CircularArc {
        start: VertexId,
        center: VertexId,
        end: VertexId,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryEdge {
    pub id: EdgeId,
    pub geometry: EdgeGeometry,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OrientedEdge {
    pub edge: EdgeId,
    pub reversed: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum GeometryFaceRepresentation {
    Planar {
        outer_loop: Vec<OrientedEdge>,
        inner_loops: Vec<Vec<OrientedEdge>>,
    },
    PrimitiveSurface,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryFace {
    pub id: FaceId,
    pub representation: GeometryFaceRepresentation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GeometryBody {
    pub id: BodyId,
    pub faces: Vec<FaceId>,
    pub representation: GeometryBodyRepresentation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum GeometryBodyRepresentation {
    Topology,
    Box {
        length: f64,
        width: f64,
        height: f64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RectangleEntities {
    pub vertices: [VertexId; 4],
    pub bottom: EdgeId,
    pub right: EdgeId,
    pub top: EdgeId,
    pub left: EdgeId,
    pub face: FaceId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CircleHoleEntities {
    pub center: VertexId,
    pub boundary: [EdgeId; 4],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoxEntities {
    pub body: BodyId,
    pub x_min: FaceId,
    pub x_max: FaceId,
    pub y_min: FaceId,
    pub y_max: FaceId,
    pub z_min: FaceId,
    pub z_max: FaceId,
}

#[derive(Clone, Debug, PartialEq)]
pub enum GeometryError {
    EntityNotFound {
        entity: &'static str,
        id: u64,
    },
    NonFiniteGeometry,
    DegenerateEdge,
    InvalidLoop {
        message: String,
    },
    EntityInUse {
        entity: &'static str,
        id: u64,
        used_by: &'static str,
    },
    InvalidPrimitive {
        message: String,
    },
}

impl std::fmt::Display for GeometryError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EntityNotFound { entity, id } => {
                write!(formatter, "{entity} {id} does not exist")
            }
            Self::NonFiniteGeometry => write!(formatter, "geometry coordinates must be finite"),
            Self::DegenerateEdge => write!(formatter, "geometry edge is degenerate"),
            Self::InvalidLoop { message } => write!(formatter, "invalid geometry loop: {message}"),
            Self::EntityInUse {
                entity,
                id,
                used_by,
            } => {
                write!(
                    formatter,
                    "{entity} {id} is still referenced by a {used_by}"
                )
            }
            Self::InvalidPrimitive { message } => {
                write!(formatter, "invalid geometry primitive: {message}")
            }
        }
    }
}

impl std::error::Error for GeometryError {}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct GeometryTopology {
    revision: GeometryRevision,
    next_vertex_id: u64,
    next_edge_id: u64,
    next_face_id: u64,
    next_body_id: u64,
    vertices: BTreeMap<VertexId, GeometryVertex>,
    edges: BTreeMap<EdgeId, GeometryEdge>,
    faces: BTreeMap<FaceId, GeometryFace>,
    bodies: BTreeMap<BodyId, GeometryBody>,
}

impl Default for GeometryTopology {
    fn default() -> Self {
        Self::new()
    }
}

impl GeometryTopology {
    pub const fn new() -> Self {
        Self {
            revision: GeometryRevision(0),
            next_vertex_id: 1,
            next_edge_id: 1,
            next_face_id: 1,
            next_body_id: 1,
            vertices: BTreeMap::new(),
            edges: BTreeMap::new(),
            faces: BTreeMap::new(),
            bodies: BTreeMap::new(),
        }
    }

    pub const fn revision(&self) -> GeometryRevision {
        self.revision
    }

    pub fn vertices(&self) -> impl Iterator<Item = &GeometryVertex> {
        self.vertices.values()
    }

    pub fn edges(&self) -> impl Iterator<Item = &GeometryEdge> {
        self.edges.values()
    }

    pub fn faces(&self) -> impl Iterator<Item = &GeometryFace> {
        self.faces.values()
    }

    pub fn bodies(&self) -> impl Iterator<Item = &GeometryBody> {
        self.bodies.values()
    }

    pub fn vertex(&self, id: VertexId) -> Option<&GeometryVertex> {
        self.vertices.get(&id)
    }

    pub fn edge(&self, id: EdgeId) -> Option<&GeometryEdge> {
        self.edges.get(&id)
    }

    pub fn face(&self, id: FaceId) -> Option<&GeometryFace> {
        self.faces.get(&id)
    }

    pub fn body(&self, id: BodyId) -> Option<&GeometryBody> {
        self.bodies.get(&id)
    }

    pub fn add_vertex(&mut self, position: Vec3) -> Result<VertexId, GeometryError> {
        if !(position.x.is_finite() && position.y.is_finite() && position.z.is_finite()) {
            return Err(GeometryError::NonFiniteGeometry);
        }
        let id = self.allocate_vertex();
        self.vertices.insert(id, GeometryVertex { id, position });
        self.bump_revision();
        Ok(id)
    }

    pub fn add_line(&mut self, start: VertexId, end: VertexId) -> Result<EdgeId, GeometryError> {
        self.validate_line(start, end)?;
        let id = self.allocate_edge();
        self.edges.insert(
            id,
            GeometryEdge {
                id,
                geometry: EdgeGeometry::Line { start, end },
            },
        );
        self.bump_revision();
        Ok(id)
    }

    pub fn add_circular_arc(
        &mut self,
        start: VertexId,
        center: VertexId,
        end: VertexId,
    ) -> Result<EdgeId, GeometryError> {
        let start_position = self.vertex_position(start)?;
        let center_position = self.vertex_position(center)?;
        let end_position = self.vertex_position(end)?;
        let radius_start = (start_position - center_position).norm();
        let radius_end = (end_position - center_position).norm();
        if start == end
            || radius_start <= GEOMETRY_EPSILON
            || (radius_start - radius_end).abs() > GEOMETRY_EPSILON * radius_start.max(1.0)
        {
            return Err(GeometryError::DegenerateEdge);
        }
        let id = self.allocate_edge();
        self.edges.insert(
            id,
            GeometryEdge {
                id,
                geometry: EdgeGeometry::CircularArc { start, center, end },
            },
        );
        self.bump_revision();
        Ok(id)
    }

    pub fn add_planar_face(
        &mut self,
        outer_loop: Vec<OrientedEdge>,
        inner_loops: Vec<Vec<OrientedEdge>>,
    ) -> Result<FaceId, GeometryError> {
        self.validate_loop(&outer_loop)?;
        for loop_edges in &inner_loops {
            self.validate_loop(loop_edges)?;
        }
        let id = self.allocate_face();
        self.faces.insert(
            id,
            GeometryFace {
                id,
                representation: GeometryFaceRepresentation::Planar {
                    outer_loop,
                    inner_loops,
                },
            },
        );
        self.bump_revision();
        Ok(id)
    }

    pub fn add_body(&mut self, faces: Vec<FaceId>) -> Result<BodyId, GeometryError> {
        self.add_body_with_representation(faces, GeometryBodyRepresentation::Topology)
    }

    fn add_body_with_representation(
        &mut self,
        faces: Vec<FaceId>,
        representation: GeometryBodyRepresentation,
    ) -> Result<BodyId, GeometryError> {
        if faces.is_empty() || faces.iter().collect::<BTreeSet<_>>().len() != faces.len() {
            return Err(GeometryError::InvalidPrimitive {
                message: "body must reference at least one unique face".into(),
            });
        }
        for &face in &faces {
            self.require_face(face)?;
        }
        let id = self.allocate_body();
        self.bodies.insert(
            id,
            GeometryBody {
                id,
                faces,
                representation,
            },
        );
        self.bump_revision();
        Ok(id)
    }

    pub fn add_rectangle(
        &mut self,
        width: f64,
        height: f64,
    ) -> Result<RectangleEntities, GeometryError> {
        if !(width.is_finite()
            && width > GEOMETRY_EPSILON
            && height.is_finite()
            && height > GEOMETRY_EPSILON)
        {
            return Err(GeometryError::InvalidPrimitive {
                message: "rectangle width and height must be finite and positive".into(),
            });
        }
        let vertices = [
            self.add_vertex(Vec3::new(0.0, 0.0, 0.0))?,
            self.add_vertex(Vec3::new(width, 0.0, 0.0))?,
            self.add_vertex(Vec3::new(width, height, 0.0))?,
            self.add_vertex(Vec3::new(0.0, height, 0.0))?,
        ];
        let bottom = self.add_line(vertices[0], vertices[1])?;
        let right = self.add_line(vertices[1], vertices[2])?;
        let top = self.add_line(vertices[2], vertices[3])?;
        let left = self.add_line(vertices[3], vertices[0])?;
        let face = self.add_planar_face(
            vec![
                OrientedEdge {
                    edge: bottom,
                    reversed: false,
                },
                OrientedEdge {
                    edge: right,
                    reversed: false,
                },
                OrientedEdge {
                    edge: top,
                    reversed: false,
                },
                OrientedEdge {
                    edge: left,
                    reversed: false,
                },
            ],
            Vec::new(),
        )?;
        Ok(RectangleEntities {
            vertices,
            bottom,
            right,
            top,
            left,
            face,
        })
    }

    pub fn add_circle_hole(
        &mut self,
        face: FaceId,
        center: Vec3,
        radius: f64,
    ) -> Result<CircleHoleEntities, GeometryError> {
        if !(center.x.is_finite()
            && center.y.is_finite()
            && center.z.is_finite()
            && radius.is_finite()
            && radius > GEOMETRY_EPSILON)
        {
            return Err(GeometryError::InvalidPrimitive {
                message: "circle center and radius must be finite and positive".into(),
            });
        }
        let outer = match &self.require_face(face)?.representation {
            GeometryFaceRepresentation::Planar { outer_loop, .. } => outer_loop.clone(),
            GeometryFaceRepresentation::PrimitiveSurface => {
                return Err(GeometryError::InvalidPrimitive {
                    message: "circle holes require a planar face".into(),
                })
            }
        };
        let center_id = self.add_vertex(center)?;
        let vertices = [
            self.add_vertex(Vec3::new(center.x + radius, center.y, center.z))?,
            self.add_vertex(Vec3::new(center.x, center.y + radius, center.z))?,
            self.add_vertex(Vec3::new(center.x - radius, center.y, center.z))?,
            self.add_vertex(Vec3::new(center.x, center.y - radius, center.z))?,
        ];
        let boundary = [
            self.add_circular_arc(vertices[0], center_id, vertices[1])?,
            self.add_circular_arc(vertices[1], center_id, vertices[2])?,
            self.add_circular_arc(vertices[2], center_id, vertices[3])?,
            self.add_circular_arc(vertices[3], center_id, vertices[0])?,
        ];
        let inner: Vec<OrientedEdge> = boundary
            .iter()
            .rev()
            .copied()
            .map(|edge| OrientedEdge {
                edge,
                reversed: true,
            })
            .collect();
        let old = self.require_face(face)?.clone();
        if let GeometryFaceRepresentation::Planar { inner_loops, .. } = old.representation {
            self.validate_loop(&inner)?;
            let mut updated_holes = inner_loops;
            updated_holes.push(inner);
            self.faces.insert(
                face,
                GeometryFace {
                    id: face,
                    representation: GeometryFaceRepresentation::Planar {
                        outer_loop: outer,
                        inner_loops: updated_holes,
                    },
                },
            );
            self.bump_revision();
        }
        Ok(CircleHoleEntities {
            center: center_id,
            boundary,
        })
    }

    pub fn add_rectangle_with_circle(
        &mut self,
        width: f64,
        height: f64,
        center_x: f64,
        center_y: f64,
        radius: f64,
    ) -> Result<(RectangleEntities, CircleHoleEntities), GeometryError> {
        if !(center_x.is_finite()
            && center_y.is_finite()
            && radius.is_finite()
            && radius > GEOMETRY_EPSILON
            && center_x - radius > 0.0
            && center_x + radius < width
            && center_y - radius > 0.0
            && center_y + radius < height)
        {
            return Err(GeometryError::InvalidPrimitive {
                message: "circle must be finite, positive, and strictly inside the rectangle"
                    .into(),
            });
        }
        let rectangle = self.add_rectangle(width, height)?;
        let hole =
            self.add_circle_hole(rectangle.face, Vec3::new(center_x, center_y, 0.0), radius)?;
        Ok((rectangle, hole))
    }

    pub fn add_box(
        &mut self,
        length: f64,
        width: f64,
        height: f64,
    ) -> Result<BoxEntities, GeometryError> {
        if !(length.is_finite()
            && length > GEOMETRY_EPSILON
            && width.is_finite()
            && width > GEOMETRY_EPSILON
            && height.is_finite()
            && height > GEOMETRY_EPSILON)
        {
            return Err(GeometryError::InvalidPrimitive {
                message: "box dimensions must be finite and positive".into(),
            });
        }
        let faces = [
            self.add_primitive_face()?,
            self.add_primitive_face()?,
            self.add_primitive_face()?,
            self.add_primitive_face()?,
            self.add_primitive_face()?,
            self.add_primitive_face()?,
        ];
        let body = self.add_body_with_representation(
            faces.to_vec(),
            GeometryBodyRepresentation::Box {
                length,
                width,
                height,
            },
        )?;
        Ok(BoxEntities {
            body,
            x_min: faces[0],
            x_max: faces[1],
            y_min: faces[2],
            y_max: faces[3],
            z_min: faces[4],
            z_max: faces[5],
        })
    }

    pub fn remove_vertex(&mut self, id: VertexId) -> Result<GeometryVertex, GeometryError> {
        self.require_vertex(id)?;
        if self
            .edges
            .values()
            .any(|edge| edge_vertices(&edge.geometry).contains(&id))
        {
            return Err(GeometryError::EntityInUse {
                entity: "vertex",
                id: id.get(),
                used_by: "edge",
            });
        }
        let removed = self.vertices.remove(&id).expect("validated vertex exists");
        self.bump_revision();
        Ok(removed)
    }

    pub fn remove_edge(&mut self, id: EdgeId) -> Result<GeometryEdge, GeometryError> {
        self.require_edge(id)?;
        if self
            .faces
            .values()
            .any(|face| face_edges(&face.representation).contains(&id))
        {
            return Err(GeometryError::EntityInUse {
                entity: "edge",
                id: id.get(),
                used_by: "face",
            });
        }
        let removed = self.edges.remove(&id).expect("validated edge exists");
        self.bump_revision();
        Ok(removed)
    }

    pub fn remove_face(&mut self, id: FaceId) -> Result<GeometryFace, GeometryError> {
        self.require_face(id)?;
        if self.bodies.values().any(|body| body.faces.contains(&id)) {
            return Err(GeometryError::EntityInUse {
                entity: "face",
                id: id.get(),
                used_by: "body",
            });
        }
        let removed = self.faces.remove(&id).expect("validated face exists");
        self.bump_revision();
        Ok(removed)
    }

    pub fn remove_body(&mut self, id: BodyId) -> Result<GeometryBody, GeometryError> {
        self.body(id).ok_or(GeometryError::EntityNotFound {
            entity: "body",
            id: id.get(),
        })?;
        let removed = self.bodies.remove(&id).expect("validated body exists");
        self.bump_revision();
        Ok(removed)
    }

    pub fn validate(&self) -> Result<(), GeometryError> {
        for edge in self.edges.values() {
            match edge.geometry {
                EdgeGeometry::Line { start, end } => self.validate_line(start, end)?,
                EdgeGeometry::CircularArc { start, center, end } => {
                    let start_position = self.vertex_position(start)?;
                    let center_position = self.vertex_position(center)?;
                    let end_position = self.vertex_position(end)?;
                    let start_radius = (start_position - center_position).norm();
                    let end_radius = (end_position - center_position).norm();
                    if start == end
                        || start_radius <= GEOMETRY_EPSILON
                        || (start_radius - end_radius).abs()
                            > GEOMETRY_EPSILON * start_radius.max(1.0)
                    {
                        return Err(GeometryError::DegenerateEdge);
                    }
                }
            }
        }
        for face in self.faces.values() {
            if let GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } = &face.representation
            {
                self.validate_loop(outer_loop)?;
                for loop_edges in inner_loops {
                    self.validate_loop(loop_edges)?;
                }
            }
        }
        for body in self.bodies.values() {
            if body.faces.is_empty()
                || body.faces.iter().collect::<BTreeSet<_>>().len() != body.faces.len()
            {
                return Err(GeometryError::InvalidPrimitive {
                    message: "body must reference at least one unique face".into(),
                });
            }
            for &face in &body.faces {
                self.require_face(face)?;
            }
        }
        Ok(())
    }

    fn add_primitive_face(&mut self) -> Result<FaceId, GeometryError> {
        let id = self.allocate_face();
        self.faces.insert(
            id,
            GeometryFace {
                id,
                representation: GeometryFaceRepresentation::PrimitiveSurface,
            },
        );
        self.bump_revision();
        Ok(id)
    }

    fn validate_line(&self, start: VertexId, end: VertexId) -> Result<(), GeometryError> {
        let start_position = self.vertex_position(start)?;
        let end_position = self.vertex_position(end)?;
        if start == end || (end_position - start_position).norm() <= GEOMETRY_EPSILON {
            return Err(GeometryError::DegenerateEdge);
        }
        Ok(())
    }

    fn validate_loop(&self, loop_edges: &[OrientedEdge]) -> Result<(), GeometryError> {
        if loop_edges.is_empty() {
            return Err(GeometryError::InvalidLoop {
                message: "loop has no edges".into(),
            });
        }
        let mut first = None;
        let mut previous_end = None;
        let mut used = BTreeSet::new();
        for oriented in loop_edges {
            if !used.insert(oriented.edge) {
                return Err(GeometryError::InvalidLoop {
                    message: "loop repeats an edge".into(),
                });
            }
            let (start, end) = self.oriented_endpoints(*oriented)?;
            if let Some(previous) = previous_end {
                if previous != start {
                    return Err(GeometryError::InvalidLoop {
                        message: "edge endpoints do not connect".into(),
                    });
                }
            } else {
                first = Some(start);
            }
            previous_end = Some(end);
        }
        if previous_end != first {
            return Err(GeometryError::InvalidLoop {
                message: "loop is open".into(),
            });
        }
        Ok(())
    }

    fn oriented_endpoints(
        &self,
        oriented: OrientedEdge,
    ) -> Result<(VertexId, VertexId), GeometryError> {
        let edge = self.require_edge(oriented.edge)?;
        let (start, end) = match edge.geometry {
            EdgeGeometry::Line { start, end } | EdgeGeometry::CircularArc { start, end, .. } => {
                (start, end)
            }
        };
        Ok(if oriented.reversed {
            (end, start)
        } else {
            (start, end)
        })
    }

    fn vertex_position(&self, id: VertexId) -> Result<Vec3, GeometryError> {
        Ok(self.require_vertex(id)?.position)
    }

    fn require_vertex(&self, id: VertexId) -> Result<&GeometryVertex, GeometryError> {
        self.vertex(id).ok_or(GeometryError::EntityNotFound {
            entity: "vertex",
            id: id.get(),
        })
    }

    fn require_edge(&self, id: EdgeId) -> Result<&GeometryEdge, GeometryError> {
        self.edge(id).ok_or(GeometryError::EntityNotFound {
            entity: "edge",
            id: id.get(),
        })
    }

    fn require_face(&self, id: FaceId) -> Result<&GeometryFace, GeometryError> {
        self.face(id).ok_or(GeometryError::EntityNotFound {
            entity: "face",
            id: id.get(),
        })
    }

    fn allocate_vertex(&mut self) -> VertexId {
        let id = VertexId(self.next_vertex_id);
        self.next_vertex_id += 1;
        id
    }
    fn allocate_edge(&mut self) -> EdgeId {
        let id = EdgeId(self.next_edge_id);
        self.next_edge_id += 1;
        id
    }
    fn allocate_face(&mut self) -> FaceId {
        let id = FaceId(self.next_face_id);
        self.next_face_id += 1;
        id
    }
    fn allocate_body(&mut self) -> BodyId {
        let id = BodyId(self.next_body_id);
        self.next_body_id += 1;
        id
    }
    fn bump_revision(&mut self) {
        self.revision.0 += 1;
    }
}

fn edge_vertices(geometry: &EdgeGeometry) -> Vec<VertexId> {
    match *geometry {
        EdgeGeometry::Line { start, end } => vec![start, end],
        EdgeGeometry::CircularArc { start, center, end } => vec![start, center, end],
    }
}

fn face_edges(representation: &GeometryFaceRepresentation) -> Vec<EdgeId> {
    match representation {
        GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } => outer_loop
            .iter()
            .chain(inner_loops.iter().flatten())
            .map(|edge| edge.edge)
            .collect(),
        GeometryFaceRepresentation::PrimitiveSurface => Vec::new(),
    }
}
