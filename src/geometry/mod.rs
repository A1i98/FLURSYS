//! Stable, solver-independent geometry topology for preprocessing.
//!
//! Entity IDs are model-local opaque handles. They are distinct from Gmsh tags
//! and mesh indices, and are allocated monotonically without reuse.

use crate::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

mod picking;

pub use picking::{FaceRayHit, Ray3};

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
geometry_id!(CadSketchId);
geometry_id!(CadFeatureId);

/// Canonical coordinate planes available for creating a sketch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CadSketchPlane {
    Xy,
    Yz,
    Zx,
    Face,
}

/// A finite right-handed local coordinate system for a canonical sketch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadPlaneFrame {
    pub origin: Vec3,
    pub x_axis: Vec3,
    pub y_axis: Vec3,
    pub normal: Vec3,
}

/// Persisted sketch metadata owned by [`GeometryTopology`]. Geometry editing
/// creates its profile topology separately; the sketch record establishes the
/// stable plane/source identity for subsequent feature commands.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadSketch {
    pub id: CadSketchId,
    pub plane: CadSketchPlane,
    pub frame: CadPlaneFrame,
    #[serde(default)]
    pub host_face: Option<FaceId>,
    #[serde(default)]
    pub profile: Option<CadSketchProfile>,
    #[serde(default)]
    pub profile_face: Option<FaceId>,
}

/// Persisted 2D profile intent on a canonical sketch plane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum CadSketchProfile {
    Rectangle { width: f64, height: f64 },
}

/// Persistent source-to-result relation for the first canonical CAD feature.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CadExtrudeFeature {
    pub id: CadFeatureId,
    pub sketch: CadSketchId,
    pub source_face: FaceId,
    pub body: BodyId,
}

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

/// A canonical CAD surface polygon suitable for a renderer or selection overlay.
/// Its `face` is always the stable topology ID used by picking and boundaries.
#[derive(Clone, Debug, PartialEq)]
pub struct RenderableFace {
    pub face: FaceId,
    pub body: Option<BodyId>,
    pub vertices: Vec<Vec3>,
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
    /// A prism-like body generated from one canonical planar profile.
    Extrude {
        source_face: FaceId,
        distance: f64,
        top_face: FaceId,
        side_faces: Vec<FaceId>,
    },
}

/// A finite rigid transform that can be applied atomically to one body.
///
/// Construct values through [`Self::translation`] or [`Self::rotation`]; the
/// private operation keeps invalid axes and non-finite coordinates out of the
/// public transform representation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigidBodyTransform {
    operation: RigidBodyTransformOperation,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum RigidBodyTransformOperation {
    Translation {
        displacement: Vec3,
    },
    Rotation {
        pivot: Vec3,
        unit_axis: Vec3,
        angle_radians: f64,
    },
}

impl RigidBodyTransform {
    pub fn translation(displacement: Vec3) -> Result<Self, GeometryError> {
        if !is_finite_vec3(displacement) {
            return Err(GeometryError::NonFiniteGeometry);
        }
        Ok(Self {
            operation: RigidBodyTransformOperation::Translation { displacement },
        })
    }

    pub fn rotation(pivot: Vec3, axis: Vec3, angle_radians: f64) -> Result<Self, GeometryError> {
        if !is_finite_vec3(pivot) || !is_finite_vec3(axis) || !angle_radians.is_finite() {
            return Err(GeometryError::NonFiniteGeometry);
        }
        let axis_length = axis.norm();
        if !(axis_length.is_finite() && axis_length > GEOMETRY_EPSILON) {
            return Err(GeometryError::InvalidPrimitive {
                message: "rotation axis must have finite non-zero length".into(),
            });
        }
        let unit_axis = axis / axis_length;
        Ok(Self {
            operation: RigidBodyTransformOperation::Rotation {
                pivot,
                unit_axis,
                angle_radians,
            },
        })
    }

    fn apply(self, position: Vec3) -> Vec3 {
        match self.operation {
            RigidBodyTransformOperation::Translation { displacement } => position + displacement,
            RigidBodyTransformOperation::Rotation {
                pivot,
                unit_axis,
                angle_radians,
            } => {
                let relative = position - pivot;
                let cosine = angle_radians.cos();
                let sine = angle_radians.sin();
                pivot
                    + relative * cosine
                    + unit_axis.cross(relative) * sine
                    + unit_axis * (unit_axis.dot(relative) * (1.0 - cosine))
            }
        }
    }
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExtrudeEntities {
    pub body: BodyId,
    pub source_face: FaceId,
    pub top_face: FaceId,
    pub side_faces: Vec<FaceId>,
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
    /// Transforming this body would also move a vertex owned by another body.
    UnsafeBodyTransform {
        body: BodyId,
        vertex: VertexId,
        other_body: BodyId,
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
            Self::UnsafeBodyTransform {
                body,
                vertex,
                other_body,
            } => write!(
                formatter,
                "cannot transform body {}: vertex {} is also owned by body {}",
                body.get(),
                vertex.get(),
                other_body.get()
            ),
        }
    }
}

impl std::error::Error for GeometryError {}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct GeometryTopology {
    revision: GeometryRevision,
    next_vertex_id: u64,
    next_edge_id: u64,
    next_face_id: u64,
    next_body_id: u64,
    #[serde(default = "first_cad_id")]
    next_sketch_id: u64,
    #[serde(default = "first_cad_id")]
    next_feature_id: u64,
    vertices: BTreeMap<VertexId, GeometryVertex>,
    edges: BTreeMap<EdgeId, GeometryEdge>,
    faces: BTreeMap<FaceId, GeometryFace>,
    bodies: BTreeMap<BodyId, GeometryBody>,
    #[serde(default)]
    sketches: BTreeMap<CadSketchId, CadSketch>,
    #[serde(default)]
    extrude_features: BTreeMap<CadFeatureId, CadExtrudeFeature>,
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
            next_sketch_id: 1,
            next_feature_id: 1,
            vertices: BTreeMap::new(),
            edges: BTreeMap::new(),
            faces: BTreeMap::new(),
            bodies: BTreeMap::new(),
            sketches: BTreeMap::new(),
            extrude_features: BTreeMap::new(),
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

    pub fn sketches(&self) -> impl Iterator<Item = &CadSketch> {
        self.sketches.values()
    }

    pub fn sketch(&self, id: CadSketchId) -> Option<&CadSketch> {
        self.sketches.get(&id)
    }

    pub fn extrude_features(&self) -> impl Iterator<Item = &CadExtrudeFeature> {
        self.extrude_features.values()
    }

    pub fn materialize_sketch_rectangle(
        &mut self,
        sketch: CadSketchId,
        width: f64,
        height: f64,
    ) -> Result<FaceId, GeometryError> {
        if !(width.is_finite()
            && height.is_finite()
            && width > GEOMETRY_EPSILON
            && height > GEOMETRY_EPSILON)
        {
            return Err(GeometryError::InvalidPrimitive {
                message: "sketch rectangle width and height must be finite and positive".into(),
            });
        }
        let sketch_record = self
            .sketch(sketch)
            .cloned()
            .ok_or(GeometryError::EntityNotFound {
                entity: "sketch",
                id: sketch.get(),
            })?;
        if sketch_record.profile_face.is_some() {
            return Err(GeometryError::InvalidPrimitive {
                message: "sketch profile is already materialized".into(),
            });
        }
        let corners = [
            sketch_record.frame.origin,
            sketch_record.frame.origin + sketch_record.frame.x_axis * width,
            sketch_record.frame.origin
                + sketch_record.frame.x_axis * width
                + sketch_record.frame.y_axis * height,
            sketch_record.frame.origin + sketch_record.frame.y_axis * height,
        ];
        let vertices = corners
            .into_iter()
            .map(|corner| self.add_vertex(corner))
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let edges = [
            self.add_line(vertices[0], vertices[1])?,
            self.add_line(vertices[1], vertices[2])?,
            self.add_line(vertices[2], vertices[3])?,
            self.add_line(vertices[3], vertices[0])?,
        ];
        let face = self.add_planar_face(
            edges
                .into_iter()
                .map(|edge| OrientedEdge {
                    edge,
                    reversed: false,
                })
                .collect(),
            Vec::new(),
        )?;
        let record = self
            .sketches
            .get_mut(&sketch)
            .expect("validated sketch exists");
        record.profile = Some(CadSketchProfile::Rectangle { width, height });
        record.profile_face = Some(face);
        self.bump_revision();
        Ok(face)
    }

    pub fn extrude_sketch_face(
        &mut self,
        sketch: CadSketchId,
        source_face: FaceId,
        distance: f64,
    ) -> Result<(CadExtrudeFeature, ExtrudeEntities), GeometryError> {
        let sketch_record = self.sketch(sketch).ok_or(GeometryError::EntityNotFound {
            entity: "sketch",
            id: sketch.get(),
        })?;
        if let Some(host_face) = sketch_record.host_face {
            if host_face != source_face {
                return Err(GeometryError::InvalidPrimitive {
                    message: "face-hosted sketch can extrude only its selected host face".into(),
                });
            }
        }
        let entities = self.extrude_planar_face(source_face, distance)?;
        let id = CadFeatureId(self.next_feature_id);
        self.next_feature_id += 1;
        let feature = CadExtrudeFeature {
            id,
            sketch,
            source_face,
            body: entities.body,
        };
        self.extrude_features.insert(id, feature.clone());
        self.bump_revision();
        Ok((feature, entities))
    }

    pub fn create_sketch_on_plane(
        &mut self,
        plane: CadSketchPlane,
    ) -> Result<CadSketch, GeometryError> {
        let frame = match plane {
            CadSketchPlane::Xy => CadPlaneFrame {
                origin: Vec3::ZERO,
                x_axis: Vec3::new(1.0, 0.0, 0.0),
                y_axis: Vec3::new(0.0, 1.0, 0.0),
                normal: Vec3::new(0.0, 0.0, 1.0),
            },
            CadSketchPlane::Yz => CadPlaneFrame {
                origin: Vec3::ZERO,
                x_axis: Vec3::new(0.0, 1.0, 0.0),
                y_axis: Vec3::new(0.0, 0.0, 1.0),
                normal: Vec3::new(1.0, 0.0, 0.0),
            },
            CadSketchPlane::Zx => CadPlaneFrame {
                origin: Vec3::ZERO,
                x_axis: Vec3::new(0.0, 0.0, 1.0),
                y_axis: Vec3::new(1.0, 0.0, 0.0),
                normal: Vec3::new(0.0, 1.0, 0.0),
            },
            CadSketchPlane::Face => {
                return Err(GeometryError::InvalidPrimitive {
                    message: "use create_sketch_on_face for a face-hosted sketch".into(),
                });
            }
        };
        self.insert_sketch(plane, frame, None)
    }

    pub fn create_sketch_on_face(&mut self, face: FaceId) -> Result<CadSketch, GeometryError> {
        let frame = self.planar_face_frame(face)?;
        self.insert_sketch(CadSketchPlane::Face, frame, Some(face))
    }

    fn insert_sketch(
        &mut self,
        plane: CadSketchPlane,
        frame: CadPlaneFrame,
        host_face: Option<FaceId>,
    ) -> Result<CadSketch, GeometryError> {
        validate_plane_frame(frame)?;
        let id = CadSketchId(self.next_sketch_id);
        self.next_sketch_id += 1;
        let sketch = CadSketch {
            id,
            plane,
            frame,
            host_face,
            profile: None,
            profile_face: None,
        };
        self.sketches.insert(id, sketch.clone());
        self.bump_revision();
        Ok(sketch)
    }

    fn planar_face_frame(&self, face: FaceId) -> Result<CadPlaneFrame, GeometryError> {
        let GeometryFaceRepresentation::Planar { outer_loop, .. } =
            &self.require_face(face)?.representation
        else {
            return Err(GeometryError::InvalidPrimitive {
                message: "sketches can be attached only to planar faces".into(),
            });
        };
        let points = outer_loop
            .iter()
            .map(|oriented| {
                let (start, end) = match self.require_edge(oriented.edge)?.geometry {
                    EdgeGeometry::Line { start, end }
                    | EdgeGeometry::CircularArc { start, end, .. } => (start, end),
                };
                self.vertex_position(if oriented.reversed { end } else { start })
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        let origin = *points
            .first()
            .ok_or_else(|| GeometryError::InvalidPrimitive {
                message: "planar face has no boundary points".into(),
            })?;
        let x_axis = (points.get(1).copied().unwrap_or(origin) - origin)
            .normalized()
            .ok_or_else(|| GeometryError::InvalidPrimitive {
                message: "planar face has a degenerate first edge".into(),
            })?;
        let normal = (1..points.len().saturating_sub(1))
            .find_map(|index| {
                (points[index] - origin)
                    .cross(points[index + 1] - origin)
                    .normalized()
            })
            .ok_or_else(|| GeometryError::InvalidPrimitive {
                message: "planar face has no finite normal".into(),
            })?;
        let y_axis =
            normal
                .cross(x_axis)
                .normalized()
                .ok_or_else(|| GeometryError::InvalidPrimitive {
                    message: "planar face has an invalid local frame".into(),
                })?;
        Ok(CadPlaneFrame {
            origin,
            x_axis,
            y_axis,
            normal,
        })
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

    /// Returns the canonical planar and extrusion-derived surfaces that can be
    /// rasterised without consulting a mesh or legacy project model.
    pub fn renderable_faces(&self) -> Vec<RenderableFace> {
        let mut faces = self
            .faces
            .values()
            .filter_map(|face| {
                let GeometryFaceRepresentation::Planar { outer_loop, .. } = &face.representation
                else {
                    return None;
                };
                self.loop_vertices(outer_loop)
                    .map(|vertices| RenderableFace {
                        face: face.id,
                        body: self.body_owning_face(face.id),
                        vertices,
                    })
            })
            .collect::<Vec<_>>();
        for body in self.bodies.values() {
            let GeometryBodyRepresentation::Extrude {
                source_face,
                distance,
                top_face,
                side_faces,
            } = &body.representation
            else {
                continue;
            };
            let Some(source) = self.face(*source_face) else {
                continue;
            };
            let GeometryFaceRepresentation::Planar {
                outer_loop,
                inner_loops,
            } = &source.representation
            else {
                continue;
            };
            let Some(outer) = self.loop_vertices(outer_loop) else {
                continue;
            };
            let Some(normal) = polygon_normal(&outer) else {
                continue;
            };
            let displacement = normal * *distance;
            faces.push(RenderableFace {
                face: *top_face,
                body: Some(body.id),
                vertices: outer.iter().map(|point| *point + displacement).collect(),
            });
            let mut boundaries = vec![outer];
            let Some(holes) = inner_loops
                .iter()
                .map(|loop_edges| self.loop_vertices(loop_edges))
                .collect::<Option<Vec<_>>>()
            else {
                continue;
            };
            boundaries.extend(holes);
            for (face, (start, end)) in side_faces.iter().copied().zip(
                boundaries
                    .iter()
                    .flat_map(|loop_vertices| loop_vertex_pairs(loop_vertices)),
            ) {
                faces.push(RenderableFace {
                    face,
                    body: Some(body.id),
                    vertices: vec![start, end, end + displacement, start + displacement],
                });
            }
        }
        faces
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

    /// Commits a stable extrusion topology from a selected planar profile.
    /// The source profile remains the bottom cap and generated faces are
    /// independent of renderer triangles and Gmsh tags.
    pub fn extrude_planar_face(
        &mut self,
        source_face: FaceId,
        distance: f64,
    ) -> Result<ExtrudeEntities, GeometryError> {
        if !(distance.is_finite() && distance.abs() > GEOMETRY_EPSILON) {
            return Err(GeometryError::InvalidPrimitive {
                message: "extrusion distance must be finite and non-zero".into(),
            });
        }
        let source = self.require_face(source_face)?.clone();
        let GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } = source.representation
        else {
            return Err(GeometryError::InvalidPrimitive {
                message: "extrusion requires a planar source face".into(),
            });
        };
        let top_face = self.add_primitive_face()?;
        let side_count = outer_loop.len() + inner_loops.iter().map(Vec::len).sum::<usize>();
        let mut side_faces = Vec::with_capacity(side_count);
        for _ in 0..side_count {
            side_faces.push(self.add_primitive_face()?);
        }
        let mut faces = vec![source_face, top_face];
        faces.extend(side_faces.iter().copied());
        let body = self.add_body_with_representation(
            faces,
            GeometryBodyRepresentation::Extrude {
                source_face,
                distance,
                top_face,
                side_faces: side_faces.clone(),
            },
        )?;
        Ok(ExtrudeEntities {
            body,
            source_face,
            top_face,
            side_faces,
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
        if self
            .sketches
            .values()
            .any(|sketch| sketch.host_face == Some(id))
        {
            return Err(GeometryError::EntityInUse {
                entity: "face",
                id: id.get(),
                used_by: "sketch",
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
        if self
            .extrude_features
            .values()
            .any(|feature| feature.body == id)
        {
            return Err(GeometryError::EntityInUse {
                entity: "body",
                id: id.get(),
                used_by: "feature",
            });
        }
        let removed = self.bodies.remove(&id).expect("validated body exists");
        self.bump_revision();
        Ok(removed)
    }

    /// Applies a validated rigid transform without allocating or replacing any
    /// topology entities. The operation is all-or-nothing: shared vertices and
    /// non-finite results are rejected before coordinates are changed.
    pub fn transform_body(
        &mut self,
        id: BodyId,
        transform: RigidBodyTransform,
    ) -> Result<(), GeometryError> {
        let body = self.body(id).ok_or(GeometryError::EntityNotFound {
            entity: "body",
            id: id.get(),
        })?;
        self.validate_body_transform_support(body, transform)?;
        let vertices = self.body_vertex_ids(body)?;
        if vertices.is_empty() {
            return Err(GeometryError::InvalidPrimitive {
                message: format!("body {} has no transformable vertex topology", id.get()),
            });
        }
        for (&other_body, other) in &self.bodies {
            if other_body == id {
                continue;
            }
            let other_vertices = self.body_vertex_ids(other)?;
            if let Some(vertex) = vertices.intersection(&other_vertices).next() {
                return Err(GeometryError::UnsafeBodyTransform {
                    body: id,
                    vertex: *vertex,
                    other_body,
                });
            }
        }
        let updated = vertices
            .iter()
            .map(|&vertex| {
                let position = transform.apply(self.vertex_position(vertex)?);
                if !is_finite_vec3(position) {
                    return Err(GeometryError::NonFiniteGeometry);
                }
                Ok((vertex, position))
            })
            .collect::<Result<Vec<_>, GeometryError>>()?;
        for (vertex, position) in updated {
            self.vertices
                .get_mut(&vertex)
                .expect("validated vertex exists")
                .position = position;
        }
        self.bump_revision();
        Ok(())
    }

    pub fn translate_body(&mut self, id: BodyId, displacement: Vec3) -> Result<(), GeometryError> {
        self.transform_body(id, RigidBodyTransform::translation(displacement)?)
    }

    pub fn rotate_body(
        &mut self,
        id: BodyId,
        pivot: Vec3,
        axis: Vec3,
        angle_radians: f64,
    ) -> Result<(), GeometryError> {
        self.transform_body(
            id,
            RigidBodyTransform::rotation(pivot, axis, angle_radians)?,
        )
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
            if let GeometryBodyRepresentation::Extrude {
                source_face,
                distance,
                top_face,
                side_faces,
            } = &body.representation
            {
                if !distance.is_finite()
                    || distance.abs() <= GEOMETRY_EPSILON
                    || !body.faces.contains(source_face)
                    || !body.faces.contains(top_face)
                    || side_faces.is_empty()
                    || side_faces.iter().any(|face| !body.faces.contains(face))
                    || !matches!(
                        self.require_face(*source_face)?.representation,
                        GeometryFaceRepresentation::Planar { .. }
                    )
                {
                    return Err(GeometryError::InvalidPrimitive {
                        message: "invalid extrusion body topology".into(),
                    });
                }
            }
        }
        for (&id, sketch) in &self.sketches {
            if sketch.id != id {
                return Err(GeometryError::InvalidPrimitive {
                    message: "canonical sketch map key does not match its stable ID".into(),
                });
            }
            validate_plane_frame(sketch.frame)?;
            match (sketch.plane, sketch.host_face) {
                (CadSketchPlane::Face, Some(face)) => {
                    if !matches!(
                        self.require_face(face)?.representation,
                        GeometryFaceRepresentation::Planar { .. }
                    ) {
                        return Err(GeometryError::InvalidPrimitive {
                            message: "face-hosted sketch requires a planar host face".into(),
                        });
                    }
                }
                (CadSketchPlane::Face, None) => {
                    return Err(GeometryError::InvalidPrimitive {
                        message: "face-hosted sketch is missing its host face".into(),
                    });
                }
                (_, Some(_)) => {
                    return Err(GeometryError::InvalidPrimitive {
                        message: "standard sketch plane cannot have a host face".into(),
                    });
                }
                (_, None) => {}
            }
        }
        if self.next_sketch_id <= self.sketches.keys().map(|id| id.get()).max().unwrap_or(0) {
            return Err(GeometryError::InvalidPrimitive {
                message: "canonical sketch allocator would reuse a stable ID".into(),
            });
        }
        for (&id, feature) in &self.extrude_features {
            if feature.id != id
                || self.sketch(feature.sketch).is_none()
                || !matches!(
                    self.body(feature.body).map(|body| &body.representation),
                    Some(GeometryBodyRepresentation::Extrude { source_face, .. })
                        if *source_face == feature.source_face
                )
            {
                return Err(GeometryError::InvalidPrimitive {
                    message: "canonical extrusion feature has a dangling source or body".into(),
                });
            }
        }
        if self.next_feature_id
            <= self
                .extrude_features
                .keys()
                .map(|id| id.get())
                .max()
                .unwrap_or(0)
        {
            return Err(GeometryError::InvalidPrimitive {
                message: "canonical feature allocator would reuse a stable ID".into(),
            });
        }
        Ok(())
    }

    fn validate_body_transform_support(
        &self,
        body: &GeometryBody,
        _transform: RigidBodyTransform,
    ) -> Result<(), GeometryError> {
        match body.representation {
            GeometryBodyRepresentation::Topology => {
                if body.faces.iter().any(|&face| {
                    !matches!(
                        self.face(face)
                            .expect("body faces are validated")
                            .representation,
                        GeometryFaceRepresentation::Planar { .. }
                    )
                }) {
                    return Err(GeometryError::InvalidPrimitive {
                        message: format!(
                            "body {} contains a primitive face without transformable topology",
                            body.id.get()
                        ),
                    });
                }
            }
            GeometryBodyRepresentation::Extrude { .. } => {}
            GeometryBodyRepresentation::Box { .. } => {
                return Err(GeometryError::InvalidPrimitive {
                    message: format!(
                        "body {} is a parametric box without editable vertex topology",
                        body.id.get()
                    ),
                });
            }
        }
        Ok(())
    }

    fn body_vertex_ids(&self, body: &GeometryBody) -> Result<BTreeSet<VertexId>, GeometryError> {
        let mut vertices = BTreeSet::new();
        for &face in &body.faces {
            for edge in face_edges(&self.require_face(face)?.representation) {
                vertices.extend(edge_vertices(&self.require_edge(edge)?.geometry));
            }
        }
        Ok(vertices)
    }

    fn body_owning_face(&self, face: FaceId) -> Option<BodyId> {
        self.bodies
            .values()
            .find(|body| body.faces.contains(&face))
            .map(|body| body.id)
    }

    fn loop_vertices(&self, loop_edges: &[OrientedEdge]) -> Option<Vec<Vec3>> {
        loop_edges
            .iter()
            .map(|oriented| {
                let edge = self.edge(oriented.edge)?;
                let (start, end) = match edge.geometry {
                    EdgeGeometry::Line { start, end }
                    | EdgeGeometry::CircularArc { start, end, .. } => (start, end),
                };
                self.vertex(if oriented.reversed { end } else { start })
                    .map(|vertex| vertex.position)
            })
            .collect()
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

fn first_cad_id() -> u64 {
    1
}

fn validate_plane_frame(frame: CadPlaneFrame) -> Result<(), GeometryError> {
    if !is_finite_vec3(frame.origin)
        || !is_finite_vec3(frame.x_axis)
        || !is_finite_vec3(frame.y_axis)
        || !is_finite_vec3(frame.normal)
        || (frame.x_axis.norm() - 1.0).abs() > GEOMETRY_EPSILON
        || (frame.y_axis.norm() - 1.0).abs() > GEOMETRY_EPSILON
        || (frame.normal.norm() - 1.0).abs() > GEOMETRY_EPSILON
        || frame.x_axis.dot(frame.y_axis).abs() > GEOMETRY_EPSILON
        || frame.x_axis.dot(frame.normal).abs() > GEOMETRY_EPSILON
        || frame.y_axis.dot(frame.normal).abs() > GEOMETRY_EPSILON
        || (frame.x_axis.cross(frame.y_axis) - frame.normal).norm() > GEOMETRY_EPSILON
    {
        return Err(GeometryError::InvalidPrimitive {
            message: "canonical sketch plane frame must be finite and orthonormal".into(),
        });
    }
    Ok(())
}

fn polygon_normal(vertices: &[Vec3]) -> Option<Vec3> {
    let origin = *vertices.first()?;
    (1..vertices.len().saturating_sub(1)).find_map(|index| {
        (vertices[index] - origin)
            .cross(vertices[index + 1] - origin)
            .normalized()
    })
}

fn loop_vertex_pairs(vertices: &[Vec3]) -> impl Iterator<Item = (Vec3, Vec3)> + '_ {
    vertices
        .iter()
        .copied()
        .zip(vertices.iter().copied().cycle().skip(1))
        .take(vertices.len())
}

fn is_finite_vec3(vector: Vec3) -> bool {
    vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()
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
