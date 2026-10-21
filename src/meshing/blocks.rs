//! Exact axis-aligned box recovery with conservative CAD face recognition.
use super::geometry::*;
use super::GmshGeoDocument;
use crate::{BodyId, FaceId, GeometryError, GeometryTopology, MeshDimension, Vec3};
use std::collections::BTreeSet;
use std::fmt::Write;
fn error(message: impl Into<String>) -> GeometryError {
    GeometryError::InvalidPrimitive {
        message: message.into(),
    }
}
impl GmshGeometryExporter {
    /// Recover only a six-face axis-aligned box. Do not approximate arbitrary
    /// solids by their bounding box: each complete face area must also match.
    pub fn cad_hex_box(
        geometry: &GeometryTopology,
        body: BodyId,
        groups: Vec<(String, Vec<FaceId>)>,
        fluid: &str,
    ) -> Result<GeometryGmshExport, GeometryError> {
        let data = geometry
            .cad_body(body)
            .ok_or_else(|| error("Select a CAD box"))?;
        if data.face_meshes.len() != 6
            || data.brep["boundaries"].as_array().map(|a| a.len()) != Some(1)
        {
            return Err(error(
                "Hex block requires exactly six planar box faces and one shell",
            ));
        }
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in data.face_meshes.iter().flat_map(|f| &f.triangles).flatten() {
            if ![p.x, p.y, p.z].iter().all(|v| v.is_finite()) {
                return Err(error("Non-finite CAD coordinates"));
            }
            lo.x = lo.x.min(p.x);
            lo.y = lo.y.min(p.y);
            lo.z = lo.z.min(p.z);
            hi.x = hi.x.max(p.x);
            hi.y = hi.y.max(p.y);
            hi.z = hi.z.max(p.z);
        }
        let d = hi - lo;
        if ![d.x, d.y, d.z].iter().all(|v| v.is_finite() && *v > 0.0) {
            return Err(error("Degenerate CAD box"));
        }
        let tolerance = d.x.min(d.y).min(d.z) * 1e-7;
        let coords = |p: Vec3| [p.x, p.y, p.z];
        let low = coords(lo);
        let high = coords(hi);
        let expected = [
            d.y * d.z,
            d.y * d.z,
            d.x * d.z,
            d.x * d.z,
            d.x * d.y,
            d.x * d.y,
        ];
        let mut sides = BTreeSet::new();
        let mut map = GeometryToGmshMap::default();
        for face in data.face_meshes.iter() {
            let side=(0..6).find(|s|face.triangles.iter().flatten().all(|p|(coords(*p)[s/2]-if s%2==0 {low[s/2]}else{high[s/2]}).abs()<=tolerance)).ok_or_else(||error("Hex block supports axis-aligned planar boxes only; this CAD face is not a box side"))?;
            let area = face
                .triangles
                .iter()
                .map(|t| 0.5 * (t[1] - t[0]).cross(t[2] - t[0]).norm())
                .sum::<f64>();
            if !sides.insert(side)
                || !area.is_finite()
                || (area - expected[side]).abs() > expected[side] * 1e-6
            {
                return Err(error("CAD faces do not cover six complete box sides; holes, cuts and split faces require a different mesh method"));
            }
            map.face_tags.insert(face.face, side + 1);
        }
        let mut source =
            String::from("// FLURSYS recognized CAD box; physical face identities retained.\n");
        let points = [
            lo,
            Vec3::new(hi.x, lo.y, lo.z),
            Vec3::new(hi.x, hi.y, lo.z),
            Vec3::new(lo.x, hi.y, lo.z),
            Vec3::new(lo.x, lo.y, hi.z),
            Vec3::new(hi.x, lo.y, hi.z),
            hi,
            Vec3::new(lo.x, hi.y, hi.z),
        ];
        for (i, p) in points.iter().enumerate() {
            writeln!(
                source,
                "Point({}) = {{{:.17}, {:.17}, {:.17}}};",
                i + 1,
                p.x,
                p.y,
                p.z
            )
            .unwrap();
        }
        for (i, (a, b)) in [
            (1, 2),
            (2, 3),
            (3, 4),
            (4, 1),
            (5, 6),
            (6, 7),
            (7, 8),
            (8, 5),
            (1, 5),
            (2, 6),
            (3, 7),
            (4, 8),
        ]
        .iter()
        .enumerate()
        {
            writeln!(source, "Line({}) = {{{a}, {b}}};", i + 1).unwrap();
        }
        for (i, edges) in [
            "-4,12,8,-9",
            "2,11,-6,-10",
            "1,10,-5,-9",
            "3,12,-7,-11",
            "-4,-3,-2,-1",
            "5,6,7,8",
        ]
        .iter()
        .enumerate()
        {
            let n = i + 1;
            writeln!(
                source,
                "Curve Loop({n}) = {{{edges}}};\nPlane Surface({n}) = {{{n}}};"
            )
            .unwrap();
        }
        source.push_str("Surface Loop(7000) = {1,2,3,4,5,6};\nVolume(7000) = {7000};\n");
        let mut assigned = BTreeSet::new();
        for (name, faces) in groups {
            if name.is_empty()
                || name.chars().any(|c| c == '"' || c.is_control())
                || faces.is_empty()
            {
                return Err(error("Invalid box boundary group"));
            }
            let mut tags = vec![];
            for face in faces {
                if !assigned.insert(face) {
                    return Err(error("Box face assigned more than once"));
                }
                tags.push(
                    map.face_tag(face)
                        .ok_or_else(|| error("Boundary references a face outside the box"))?
                        .to_string(),
                );
            }
            writeln!(
                source,
                "Physical Surface(\"{name}\") = {{{}}};",
                tags.join(",")
            )
            .unwrap();
        }
        if assigned.len() != 6 {
            return Err(error(
                "Assign all six box faces before generating a hex mesh",
            ));
        }
        if fluid.is_empty() || fluid.chars().any(|c| c == '"' || c.is_control()) {
            return Err(error("Invalid fluid region name"));
        }
        writeln!(source, "Physical Volume(\"{fluid}\") = {{7000}};").unwrap();
        map.body_tags.insert(body, 7000);
        Ok(GeometryGmshExport {
            document: GmshGeoDocument::from_source(MeshDimension::ThreeD, source),
            map,
        })
    }
}
