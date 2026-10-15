//! Faceted CAD-to-mesh export. Exact B-rep remains available through STEP;
//! this exporter explicitly uses the validated display tessellation, retaining
//! a one-to-many mapping from stable CAD faces to mesher surfaces.
use super::geometry::*;
use crate::{BodyId, FaceId, GeometryError, GeometryTopology, MeshDimension};
use std::collections::BTreeMap;
use std::fmt::Write;
fn error(s: impl Into<String>) -> GeometryError {
    GeometryError::InvalidPrimitive { message: s.into() }
}
impl GmshGeometryExporter {
    pub fn cad_faceted(
        geometry: &GeometryTopology,
        body: BodyId,
        groups: Vec<(String, Vec<FaceId>)>,
        fluid: &str,
    ) -> Result<GeometryGmshExport, GeometryError> {
        let data = geometry
            .cad_body(body)
            .ok_or_else(|| error("Select a CAD body"))?;
        if data.brep["boundaries"].as_array().map(|a| a.len()) != Some(1) {
            return Err(error("Faceted meshing currently requires one closed shell; export STEP for multi-shell solids"));
        }
        if fluid.is_empty() || fluid.contains(['"', '\n', '\r']) {
            return Err(error("Invalid fluid name"));
        }
        let mut minimum = crate::Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut maximum = crate::Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for v in data.face_meshes.iter().flat_map(|f| &f.triangles).flatten() {
            minimum.x = minimum.x.min(v.x);
            minimum.y = minimum.y.min(v.y);
            minimum.z = minimum.z.min(v.z);
            maximum.x = maximum.x.max(v.x);
            maximum.y = maximum.y.max(v.y);
            maximum.z = maximum.z.max(v.z);
        }
        let weld = ((maximum - minimum).norm() * 1e-6).max(1e-6);
        let mut points = BTreeMap::<(i64, i64, i64), Vec<usize>>::new();
        let mut coordinates = Vec::<crate::Vec3>::new();
        let mut edges = BTreeMap::<(usize, usize), (usize, usize)>::new();
        let mut face_groups = BTreeMap::<FaceId, Vec<usize>>::new();
        let mut source=String::from("// FLURSYS faceted CAD approximation; exact analytic geometry is retained in the project B-rep.\n");
        let mut next_edge = 1;
        let mut surfaces = Vec::new();
        for face in data.face_meshes.iter() {
            for triangle in &face.triangles {
                if (triangle[1] - triangle[0])
                    .cross(triangle[2] - triangle[0])
                    .norm()
                    <= weld * weld
                {
                    continue;
                }
                let mut vertex_tags = Vec::new();
                for v in triangle {
                    let relative = (*v - minimum) * (1.0 / weld);
                    let key = (
                        relative.x.floor() as i64,
                        relative.y.floor() as i64,
                        relative.z.floor() as i64,
                    );
                    let mut existing = None;
                    'search: for dx in -1..=1 {
                        for dy in -1..=1 {
                            for dz in -1..=1 {
                                if let Some(tags) =
                                    points.get(&(key.0 + dx, key.1 + dy, key.2 + dz))
                                {
                                    for &tag in tags {
                                        if (coordinates[tag - 1] - *v).norm() <= weld {
                                            existing = Some(tag);
                                            break 'search;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    let tag = if let Some(tag) = existing {
                        tag
                    } else {
                        let tag = coordinates.len() + 1;
                        coordinates.push(*v);
                        points.entry(key).or_default().push(tag);
                        writeln!(
                            source,
                            "Point({tag}) = {{{:.17}, {:.17}, {:.17}}};",
                            v.x, v.y, v.z
                        )
                        .unwrap();
                        tag
                    };
                    vertex_tags.push(tag);
                }
                if vertex_tags
                    .iter()
                    .collect::<std::collections::BTreeSet<_>>()
                    .len()
                    != 3
                {
                    continue;
                }
                let mut edge_tags = Vec::new();
                for i in 0..3 {
                    let (a, b) = (vertex_tags[i], vertex_tags[(i + 1) % 3]);
                    let key = (a.min(b), a.max(b));
                    let entry = edges.entry(key).or_insert_with(|| {
                        let tag = next_edge;
                        next_edge += 1;
                        writeln!(source, "Line({tag}) = {{{}, {}}};", key.0, key.1).unwrap();
                        (tag, 0)
                    });
                    entry.1 += 1;
                    edge_tags.push(if a < b {
                        entry.0 as i64
                    } else {
                        -(entry.0 as i64)
                    });
                }
                let tag = surfaces.len() + 1;
                writeln!(
                    source,
                    "Curve Loop({tag}) = {{{}, {}, {}}};\nPlane Surface({tag}) = {{{tag}}};",
                    edge_tags[0], edge_tags[1], edge_tags[2]
                )
                .unwrap();
                surfaces.push(tag);
                face_groups.entry(face.face).or_default().push(tag);
            }
        }
        if edges.values().any(|e| e.1 != 2) {
            return Err(error(
                "CAD tessellation is not watertight; use exact STEP export for meshing",
            ));
        }
        let all = surfaces
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join(",");
        writeln!(source,"Surface Loop(1) = {{{all}}};\nVolume(1) = {{1}};\nPhysical Volume(\"{fluid}\") = {{1}};").unwrap();
        for (name, faces) in groups {
            if name.is_empty() || name.contains(['"', '\n', '\r']) {
                return Err(error("Invalid boundary group name"));
            }
            let mut tags = Vec::new();
            for face in faces {
                tags.extend(
                    face_groups
                        .get(&face)
                        .ok_or_else(|| {
                            error(format!(
                                "Boundary {name} refers to a face outside the selected CAD body"
                            ))
                        })?
                        .iter()
                        .copied(),
                );
            }
            tags.sort_unstable();
            tags.dedup();
            if tags.is_empty() {
                return Err(error("Empty CAD boundary group"));
            }
            writeln!(
                source,
                "Physical Surface(\"{name}\") = {{{}}};",
                tags.iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            )
            .unwrap();
        }
        let mut map = GeometryToGmshMap::default();
        map.body_tags.insert(body, 1);
        for (face, tags) in &face_groups {
            if let Some(&tag) = tags.first() {
                map.face_tags.insert(*face, tag);
            }
        }
        map.face_tag_groups = face_groups;
        Ok(GeometryGmshExport {
            document: super::GmshGeoDocument::from_source(MeshDimension::ThreeD, source),
            map,
        })
    }
}
