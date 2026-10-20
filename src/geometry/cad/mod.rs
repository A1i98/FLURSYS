//! Native B-rep modeling adapter. Stable application face IDs correspond to
//! kernel faces, never to renderer triangles. Commands prepare a complete result
//! before publishing it, so failed operations do not change the document.
use super::*;
use std::sync::Arc;
use truck_meshalgo::prelude::*;
use truck_modeling::{builder, EuclideanSpace, InnerSpace, Point3, Rad, Solid, Vector3, Wire};
mod advanced;
mod axis;
mod constraints;
mod datum;
mod exchange;
mod finishing;
mod history;
mod naming;
mod operations;
mod parametric;
mod pocket;
mod primitives;
mod regenerate;
mod regions;
mod sketch;
mod drawing;
mod subelements;
mod subtractive;
pub use subelements::{CadEdge, CadVertex};
mod visibility;
pub use primitives::CadPrimitive;

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CadDependencies {
    pub bodies: Vec<BodyId>,
    pub profiles: Vec<FaceId>,
    pub replayable_pad: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadBody {
    pub name: String,
    pub operation: String,
    pub visible: bool,
    #[serde(default)]
    pub dependencies: Option<CadDependencies>,
    #[serde(default)]
    pub recipe: Option<CadRecipe>,
    /// Truck's compressed exact B-rep; the display mesh is never used for CSG.
    pub brep: Arc<serde_json::Value>,
    pub face_meshes: Arc<Vec<CadFaceMesh>>,
    #[serde(default)]
    pub edges: Arc<Vec<CadEdge>>,
    #[serde(default)]
    pub vertices: Arc<Vec<CadVertex>>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CadFaceMesh {
    pub face: FaceId,
    #[serde(default)]
    pub semantic_name: Option<String>,
    #[serde(default)]
    pub plane: Option<CadPlaneFrame>,
    pub triangles: Vec<[Vec3; 3]>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CadBoolean {
    Union,
    Difference,
    Intersection,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum CadTransform {
    Translate(Vec3),
    Rotate {
        pivot: Vec3,
        axis: Vec3,
        degrees: f64,
    },
    Mirror {
        pivot: Vec3,
        normal: Vec3,
    },
    ScaleAxes {
        pivot: Vec3,
        factors: Vec3,
    },
    Scale {
        pivot: Vec3,
        factor: f64,
    },
}
/// An associative axis reference resolved from current sketch/B-rep geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CadAxisReference {
    Edge(EdgeId),
    ProfileX(FaceId),
    ProfileY(FaceId),
}
/// Persisted modeling intent, independent of display text and tessellation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum CadRecipe {
    Finish {
        source: BodyId,
        kind: u8,
        amount: f64,
        corners: u8,
        opening: u8,
    },
    Primitive {
        shape: CadPrimitive,
        position: Vec3,
    },
    Extrude {
        face: FaceId,
        distance: f64,
    },
    Revolve {
        face: FaceId,
        pivot: Vec3,
        axis: Vec3,
        degrees: f64,
        #[serde(default)]
        axis_reference: Option<CadAxisReference>,
        #[serde(default)]
        axis_reversed: bool,
    },
    Loft {
        sections: Vec<FaceId>,
    },
    Sweep {
        face: FaceId,
        displacement: Vec3,
    },
    Taper {
        face: FaceId,
        distance: f64,
        factor: f64,
    },
    Boolean {
        left: BodyId,
        right: BodyId,
        operation: CadBoolean,
        tolerance: f64,
    },
    Transform {
        source: BodyId,
        transform: CadTransform,
    },
    Modified {
        source: Box<CadRecipe>,
        transform: CadTransform,
    },
    PocketTool {
        face: FaceId,
        depth: f64,
        through_all: bool,
        #[serde(default)]
        stop_face: Option<FaceId>,
        tolerance: f64,
    },
}
fn error(message: impl Into<String>) -> GeometryError {
    GeometryError::InvalidPrimitive {
        message: message.into(),
    }
}
fn point(v: Vec3) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}
fn vector(v: Vec3) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}
fn vec3(v: Point3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}
fn finite(v: Vec3) -> Result<(), GeometryError> {
    if is_finite_vec3(v) {
        Ok(())
    } else {
        Err(GeometryError::NonFiniteGeometry)
    }
}

impl GeometryTopology {
    pub(super) fn validate_cad_bodies(&self) -> Result<(), GeometryError> {
        for sketch in self.sketches.values() {
            if !sketch.attachment_offset.is_finite() {
                return Err(error("Non-finite sketch attachment offset"));
            }
            if sketch.locked_position.is_some() || sketch.locked_profile.is_some() {
                self.cad_sketch_parameters(sketch.id)?;
            }
        }
        for (&id, cad) in &self.cad_bodies {
            let body = self.body(id).ok_or_else(|| error("Orphaned CAD body"))?;
            if let Some(dependencies) = &cad.dependencies {
                if dependencies
                    .bodies
                    .iter()
                    .any(|parent| *parent == id || self.cad_body(*parent).is_none())
                    || dependencies
                        .profiles
                        .iter()
                        .any(|profile| self.face(*profile).is_none())
                {
                    return Err(error("CAD operation has a dangling dependency"));
                }
            }

            if body.faces.len() != cad.face_meshes.len()
                || body
                    .faces
                    .iter()
                    .copied()
                    .ne(cad.face_meshes.iter().map(|m| m.face))
            {
                return Err(error("CAD face identity mismatch"));
            }
            if !cad.edges.is_empty() || !cad.vertices.is_empty() {
                let face_ids = body.faces.iter().copied().collect::<BTreeSet<_>>();
                for edge in cad.edges.iter() {
                    if edge.path.len()<2 || !edge.length.is_finite() || edge.length<=0.0
                        || edge.faces.is_empty() || edge.faces.iter().any(|f|!face_ids.contains(f))
                        || !self.edges.get(&edge.id).is_some_and(|e|matches!(e.geometry,EdgeGeometry::Kernel{id,start,end} if id==edge.id && start==edge.start && end==edge.end)) {
                        return Err(error("Invalid CAD edge identity or curve cache"));
                    }
                    for p in &edge.path {
                        finite(*p)?;
                    }
                }
                for vertex in cad.vertices.iter() {
                    if vertex.faces.is_empty() || vertex.faces.iter().any(|f| !face_ids.contains(f))
                    {
                        return Err(error("Invalid CAD vertex adjacency"));
                    }
                    finite(self.require_vertex(vertex.id)?.position)?;
                }
            }
            let solid = self.cad_solid(id)?;
            if solid.face_iter().count() != cad.face_meshes.len() {
                return Err(error("B-rep and display face counts differ"));
            }
            for (mesh, face) in cad.face_meshes.iter().zip(solid.face_iter()) {
                if kernel_plane(face).is_some() != mesh.plane.is_some() {
                    return Err(error("CAD planar face cache differs from B-rep"));
                }
                if let Some(plane) = mesh.plane {
                    validate_plane_frame(plane)?;
                }

                if mesh.triangles.is_empty() {
                    return Err(error("Empty CAD display face"));
                }
                for tri in &mesh.triangles {
                    for v in tri {
                        finite(*v)?;
                    }
                }
            }
        }
        Ok(())
    }
    pub(super) fn cad_face_frame(&self, face: FaceId) -> Result<CadPlaneFrame, GeometryError> {
        let id = self
            .cad_owner(face)
            .ok_or_else(|| error("Face has no CAD body"))?;
        self.cad_bodies[&id]
            .face_meshes
            .iter()
            .find(|m| m.face == face)
            .and_then(|m| m.plane)
            .ok_or_else(|| {
                error(
                    "Sketch attachment requires an exact planar face; select a plane or planar cap",
                )
            })
    }
    pub fn cad_bodies(&self) -> impl Iterator<Item = (BodyId, &CadBody)> {
        self.cad_bodies.iter().map(|(id, b)| (*id, b))
    }
    pub fn cad_body(&self, id: BodyId) -> Option<&CadBody> {
        self.cad_bodies.get(&id)
    }
    pub fn cad_owner(&self, face: FaceId) -> Option<BodyId> {
        self.body_owning_face(face)
            .filter(|id| self.cad_bodies.contains_key(id))
    }
    pub fn cad_set_visible(&mut self, id: BodyId, visible: bool) -> Result<(), GeometryError> {
        self.cad_bodies
            .get_mut(&id)
            .ok_or_else(|| error("Select a CAD body"))?
            .visible = visible;
        self.bump_revision();
        Ok(())
    }
    pub fn cad_rename(&mut self, id: BodyId, name: &str) -> Result<(), GeometryError> {
        if name.trim().is_empty() || name.len() > 128 {
            return Err(error("Body name must contain 1..128 characters"));
        }
        self.cad_bodies
            .get_mut(&id)
            .ok_or_else(|| error("Select a CAD body"))?
            .name = name.trim().into();
        self.bump_revision();
        Ok(())
    }
    pub fn cad_add_primitive(
        &mut self,
        shape: CadPrimitive,
        position: Vec3,
        name: &str,
    ) -> Result<BodyId, GeometryError> {
        finite(position)?;
        let solid = builder::translated(&primitives::primitive(shape)?, vector(position));
        let description = match shape {
            CadPrimitive::Box {
                length,
                width,
                height,
            } => format!("Box · {length} × {width} × {height}"),
            CadPrimitive::Cylinder { radius, height } => {
                format!("Cylinder · R {radius} · height {height}")
            }
            CadPrimitive::Sphere { radius } => format!("Sphere · R {radius}"),
            CadPrimitive::Cone { radius, height } => format!("Cone · R {radius} · height {height}"),
            CadPrimitive::Torus {
                major_radius,
                minor_radius,
            } => format!("Torus · major R {major_radius} · minor R {minor_radius}"),
        };
        let id = self.cad_insert(solid, name, &description)?;
        self.cad_bodies.get_mut(&id).unwrap().recipe =
            Some(CadRecipe::Primitive { shape, position });
        self.name_constructed_faces(id)?;
        Ok(id)
    }
    pub(super) fn cad_solid(&self, id: BodyId) -> Result<Solid, GeometryError> {
        let body = self
            .cad_bodies
            .get(&id)
            .ok_or_else(|| error("Select a Truck CAD body in the model tree"))?;
        Solid::deserialize(body.brep.as_ref()).map_err(|e| error(format!("Invalid B-rep: {e}")))
    }
    fn cad_insert(
        &mut self,
        mut solid: Solid,
        name: &str,
        operation: &str,
    ) -> Result<BodyId, GeometryError> {
        let (mut brep, mut meshes) = prepare(&solid)?;
        let origin = meshes[0][0][0];
        let signed_volume = meshes
            .iter()
            .flatten()
            .map(|t| (t[0] - origin).dot((t[1] - origin).cross(t[2] - origin)) / 6.0)
            .sum::<f64>();
        if !signed_volume.is_finite() || signed_volume.abs() < 1e-24 {
            return Err(error(
                "Operation produced a zero-volume or invalid solid; check profile and axis",
            ));
        }
        if signed_volume < 0.0 {
            solid.not();
            (brep, meshes) = prepare(&solid)?;
        }

        let ids = (0..meshes.len())
            .map(|_| self.add_primitive_face())
            .collect::<Result<Vec<_>, _>>()?;
        let id =
            self.add_body_with_representation(ids.clone(), GeometryBodyRepresentation::Topology)?;
        self.cad_bodies.insert(
            id,
            CadBody {
                name: name.into(),
                operation: operation.into(),
                visible: true,
                dependencies: Some(CadDependencies::default()),
                recipe: None,
                edges: Arc::new(Vec::new()),
                vertices: Arc::new(Vec::new()),
                brep: Arc::new(brep),
                face_meshes: Arc::new(
                    ids.into_iter()
                        .zip(meshes)
                        .zip(solid.face_iter().map(kernel_plane))
                        .map(|((face, triangles), plane)| CadFaceMesh {
                            face,
                            semantic_name: None,
                            triangles,
                            plane,
                        })
                        .collect(),
                ),
            },
        );
        self.refresh_subelements(id)?;
        Ok(id)
    }
    fn cad_profile(&self, id: FaceId) -> Result<truck_modeling::Face, GeometryError> {
        if let Some(record) = self.sketches().find(|s| s.profile_face == Some(id) && s.drawing_region.is_some()) {
            return drawing::exact_face(record.frame, record.drawing_region.as_ref().ok_or_else(|| error("Missing drawing region"))?);
        }
        let face = self
            .face(id)
            .ok_or_else(|| error("Select a closed planar profile"))?;
        let GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } = &face.representation
        else {
            return Err(error("Select a planar sketch profile"));
        };
        let mut vertices = BTreeMap::new();
        for v in self.vertices() {
            vertices.insert(v.id, builder::vertex(point(v.position)));
        }
        let mut wires = Vec::new();
        for edges in std::iter::once(outer_loop).chain(inner_loops) {
            let mut wire = Wire::new();
            for oriented in edges {
                let edge = self
                    .edge(oriented.edge)
                    .ok_or_else(|| error("Missing profile edge"))?;
                let mut edge = match edge.geometry {
                    EdgeGeometry::Kernel { .. } => {
                        return Err(error(
                            "Project kernel edges into a sketch before constructing a profile",
                        ))
                    }
                    EdgeGeometry::Line { start, end } => {
                        builder::line(&vertices[&start], &vertices[&end])
                    }
                    EdgeGeometry::CircularArc { start, center, end } => {
                        let a = self.vertex_position(start)?;
                        let b = self.vertex_position(end)?;
                        let c = self.vertex_position(center)?;
                        let mid = (a - c) + (b - c);
                        if mid.norm() < 1e-12 {
                            return Err(error("Split semicircle edges before extrusion"));
                        }
                        let mid = c + mid * ((a - c).norm() / mid.norm());
                        builder::circle_arc(&vertices[&start], &vertices[&end], point(mid))
                    }
                };
                if oriented.reversed {
                    edge.invert();
                }
                wire.push_back(edge);
            }
            wires.push(wire);
        }
        builder::try_attach_plane(&wires).map_err(|e| error(format!("Invalid profile: {e}")))
    }
}
fn apply_transform(s: &Solid, t: CadTransform) -> Result<Solid, GeometryError> {
    Ok(match t {
        CadTransform::Translate(v) => {
            finite(v)?;
            builder::translated(s, vector(v))
        }
        CadTransform::Rotate {
            pivot,
            axis,
            degrees,
        } => {
            finite(pivot)?;
            finite(axis)?;
            if !degrees.is_finite() || axis.norm() < 1e-12 {
                return Err(error("Rotation requires a finite angle and nonzero axis"));
            }
            builder::rotated(
                s,
                point(pivot),
                vector(axis).normalize(),
                Rad(degrees.to_radians()),
            )
        }
        CadTransform::Mirror { pivot, normal } => {
            finite(pivot)?;
            finite(normal)?;
            if normal.norm() < 1e-12 {
                return Err(error("Mirror plane requires a nonzero normal"));
            }
            let n = normal * (1.0 / normal.norm());
            let reflect = |v: Vec3| v - n * (2.0 * v.dot(n));
            let x = reflect(Vec3::new(1.0, 0.0, 0.0));
            let y = reflect(Vec3::new(0.0, 1.0, 0.0));
            let z = reflect(Vec3::new(0.0, 0.0, 1.0));
            let t = pivot - reflect(pivot);
            let matrix = truck_modeling::Matrix4::new(
                x.x, x.y, x.z, 0.0, y.x, y.y, y.z, 0.0, z.x, z.y, z.z, 0.0, t.x, t.y, t.z, 1.0,
            );
            let mut result = builder::transformed(s, matrix);
            // Reflection reverses handedness; restore outward shell orientation.
            result.not();
            result
        }
        CadTransform::ScaleAxes { pivot, factors } => {
            finite(pivot)?;
            finite(factors)?;
            for value in [factors.x, factors.y, factors.z] {
                primitives::positive(value, "Axis scale")?;
            }
            builder::scaled(s, point(pivot), vector(factors))
        }
        CadTransform::Scale { pivot, factor } => {
            finite(pivot)?;
            primitives::positive(factor, "Scale factor")?;
            builder::scaled(s, point(pivot), Vector3::new(factor, factor, factor))
        }
    })
}
fn prepare(solid: &Solid) -> Result<(serde_json::Value, Vec<Vec<[Vec3; 3]>>), GeometryError> {
    prepare_with_tolerance(solid, None)
}
fn prepare_with_tolerance(
    solid: &Solid,
    requested: Option<f64>,
) -> Result<(serde_json::Value, Vec<Vec<[Vec3; 3]>>), GeometryError> {
    if solid.boundaries().is_empty() || solid.face_iter().count() == 0 {
        return Err(error("Operation produced an empty solid"));
    }
    Solid::try_new(solid.boundaries().clone())
        .map_err(|e| error(format!("Invalid closed solid: {e}")))?;
    let brep = serde_json::to_value(solid).map_err(|e| error(e.to_string()))?;
    let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for v in solid.vertex_iter() {
        let p = v.point();
        finite(vec3(p))?;
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }
    let diagonal = (max - min).magnitude();
    if diagonal < 1e-5 {
        return Err(error("Solid is smaller than the kernel modeling tolerance"));
    }
    let tolerance = requested.unwrap_or_else(|| (diagonal * 0.002).max(1e-5));
    if requested.is_some()
        && (!tolerance.is_finite() || tolerance < 1e-6 || tolerance > diagonal * 0.1)
    {
        return Err(error("Tessellation tolerance must be at least 1e-6 and at most 10% of the vertex bounding-box diagonal"));
    }
    let mut faces = Vec::new();
    let mut count = 0;
    for shell in solid.boundaries() {
        let mesh = shell.triangulation(tolerance);
        for face in mesh.face_iter() {
            let mut poly = face
                .surface()
                .ok_or_else(|| error("Surface tessellation failed"))?;
            if !face.orientation() {
                poly.invert();
            }
            poly.triangulate();
            let triangles = poly
                .tri_faces()
                .iter()
                .map(|f| f.map(|v| vec3(poly.positions()[v.pos])))
                .collect::<Vec<_>>();
            if triangles.is_empty() {
                return Err(error("Surface tessellation produced no triangles"));
            }
            count += triangles.len();
            if count > 250_000 {
                return Err(error("CAD display mesh exceeds 250000 triangles"));
            }
            faces.push(triangles);
        }
    }
    Ok((brep, faces))
}

fn kernel_plane(face: &truck_modeling::Face) -> Option<CadPlaneFrame> {
    let truck_modeling::Surface::Plane(plane) = face.surface() else {
        return None;
    };
    let x = plane.u_axis();
    if x.magnitude() < 1e-12 {
        return None;
    }
    let x_axis = vec3(Point3::from_vec(x.normalize()));
    let mut normal = vec3(Point3::from_vec(plane.normal()));
    if !face.orientation() {
        normal = normal * (-1.0);
    }
    Some(CadPlaneFrame {
        origin: vec3(plane.origin()),
        x_axis,
        y_axis: normal.cross(x_axis),
        normal,
    })
}
