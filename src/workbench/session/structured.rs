//! Topology-aware mapped meshes. Physical boundary curves retain their tags.
use super::*;
use crate::workbench::MeshMethod;
use crate::{EdgeGeometry, GeometryFaceRepresentation, OrientedEdge, VertexId};

fn invalid(message: impl Into<String>) -> WorkbenchError {
    WorkbenchError::InvalidGrouping {
        message: message.into(),
    }
}
fn signed(tag: usize, reversed: bool) -> i64 {
    if reversed {
        -(tag as i64)
    } else {
        tag as i64
    }
}
impl WorkbenchSession {
    fn loop_data(
        &self,
        export: &GeometryGmshExport,
        edges: &[OrientedEdge],
    ) -> Result<Vec<(i64, VertexId)>, WorkbenchError> {
        edges
            .iter()
            .map(|e| {
                let edge = self
                    .geometry
                    .edge(e.edge)
                    .ok_or_else(|| invalid("Missing mapped edge"))?;
                let (start, end) = match edge.geometry {
                    EdgeGeometry::Line { start, end }
                    | EdgeGeometry::CircularArc { start, end, .. } => (start, end),
                    _ => {
                        return Err(invalid(
                            "Mapped meshing requires analytic curves, not faceted CAD edges",
                        ));
                    }
                };
                Ok((
                    signed(
                        export
                            .map
                            .edge_tag(e.edge)
                            .ok_or_else(|| invalid("Mapped edge was not exported"))?,
                        e.reversed,
                    ),
                    if e.reversed { end } else { start },
                ))
            })
            .collect()
    }
    pub(super) fn apply_topology_method(
        &self,
        mut export: GeometryGmshExport,
    ) -> Result<GeometryGmshExport, WorkbenchError> {
        let t = &self.mesh_recipe.tuning;
        t.validate().map_err(invalid)?;
        if t.method == MeshMethod::Unstructured {
            return Ok(export);
        }
        let structured = matches!(
            t.method,
            MeshMethod::MappedQuad
                | MeshMethod::OGrid
                | MeshMethod::HexBlock
                | MeshMethod::SweepPrism
                | MeshMethod::SweepHex
        );
        if structured && self.mesh_recipe.edge_divisions.iter().any(|c| c.enabled) {
            return Err(invalid(
                "Mapped/block/sweep methods use their I/J/K counts. Disable scoped edge division controls first.",
            ));
        }
        if structured
            && (self.mesh_recipe.local_sizes.iter().any(|c| c.enabled)
                || self.mesh_recipe.refinements.iter().any(|c| c.enabled)
                || self.mesh_recipe.boundary_layers.iter().any(|c| c.enabled)
                || self.mesh_recipe.regions.iter().any(|c| c.enabled))
        {
            return Err(invalid(
                "Structured methods use explicit cell divisions. Disable size, distance, box and inflation controls first; use mapped progression for grading.",
            ));
        }
        let [ni, nj, nk] = t.divisions;
        let [ri, rj] = t.progression;
        if matches!(t.method, MeshMethod::SweepPrism | MeshMethod::SweepHex) {
            if self.mesh_dimension != MeshDimension::ThreeD
                || self.box_body.is_some()
                || self.geometry.cad_bodies().any(|(_, b)| b.visible)
            {
                return Err(invalid(
                    "Surface sweep requires an analytic profile extrusion, not an imported/faceted CAD solid or canonical box",
                ));
            }
            let body = self
                .geometry
                .bodies()
                .find(|b| {
                    matches!(
                        b.representation,
                        crate::GeometryBodyRepresentation::Extrude { .. }
                    )
                })
                .ok_or_else(|| invalid("No analytic extrusion available for surface sweep"))?;
            let crate::GeometryBodyRepresentation::Extrude { source_face, .. } =
                body.representation
            else {
                unreachable!()
            };
            if t.method == MeshMethod::SweepHex {
                let GeometryFaceRepresentation::Planar {
                    outer_loop,
                    inner_loops,
                } = &self.geometry.face(source_face).unwrap().representation
                else {
                    return Err(invalid("Sweep source must be planar"));
                };
                if outer_loop.len() != 4
                    || !inner_loops.is_empty()
                    || outer_loop.iter().any(|e| {
                        !matches!(
                            self.geometry.edge(e.edge).map(|e| &e.geometry),
                            Some(EdgeGeometry::Line { .. })
                        )
                    })
                {
                    return Err(invalid(
                        "Hex sweep requires a four-line source profile without holes",
                    ));
                }
                let edges = self.loop_data(&export, outer_loop)?;
                let p = edges
                    .iter()
                    .map(|(_, v)| self.geometry.vertex(*v).unwrap().position)
                    .collect::<Vec<_>>();
                let normal = (p[1] - p[0]).cross(p[2] - p[1]);
                if normal.norm() <= 0.0
                    || (0..4).any(|i| {
                        (p[(i + 1) % 4] - p[i])
                            .cross(p[(i + 2) % 4] - p[(i + 1) % 4])
                            .dot(normal)
                            <= 0.0
                    })
                {
                    return Err(invalid("Hex sweep source must be convex"));
                }
                let mut source = String::new();
                for (i, edge) in edges.iter().enumerate().take(4) {
                    let (n, r) = if i % 2 == 0 { (ni, ri) } else { (nj, rj) };
                    let tag = if i >= 2 { -edge.0 } else { edge.0 };
                    source.push_str(&format!(
                        "Transfinite Curve {{{tag}}} = {} Using Progression {r:.17};\n",
                        n + 1
                    ));
                }
                source.push_str("Transfinite Surface {3000};\nRecombine Surface {3000};\n");
                export.document = export.document.with_appended_source(source);
            }
            return Ok(export);
        }
        if t.method == MeshMethod::HexBlock {
            if self.mesh_dimension != MeshDimension::ThreeD
                || (self.box_body.is_none()
                    && !self
                        .geometry
                        .cad_bodies()
                        .any(|(id, b)| b.visible && export.map.body_tag(id) == Some(7000)))
            {
                return Err(invalid(
                    "Hex block requires a canonical box or a recognized six-face axis-aligned CAD box. General solids are not supported.",
                ));
            }
            if ri != 1.0 || rj != 1.0 {
                return Err(invalid(
                    "Hex block currently uses uniform spacing; set both progression ratios to 1",
                ));
            }
            // Classify OCC box curves by their bounding boxes rather than relying
            // on OCC's internal curve-number ordering.
            let mut source = String::from(
                "\n// Axis-aligned conformal hexahedral block.\nfl_curves[] = Curve{:};\n",
            );
            source.push_str(&format!("For fl_i In {{0:#fl_curves[]-1}}\n fl_b[] = BoundingBox Curve {{fl_curves[fl_i]}};\n fl_dx = fl_b[3]-fl_b[0]; fl_dy = fl_b[4]-fl_b[1]; fl_dz = fl_b[5]-fl_b[2];\n If (fl_dx > fl_dy && fl_dx > fl_dz)\n Transfinite Curve {{fl_curves[fl_i]}} = {};\n ElseIf (fl_dy > fl_dz)\n Transfinite Curve {{fl_curves[fl_i]}} = {};\n Else\n Transfinite Curve {{fl_curves[fl_i]}} = {};\n EndIf\nEndFor\nTransfinite Surface {{:}};\nRecombine Surface {{:}};\nTransfinite Volume {{7000}};\n",ni+1,nj+1,nk+1));
            export.document = export.document.with_appended_source(source);
            return Ok(export);
        }
        if self.mesh_dimension != MeshDimension::TwoD {
            return Err(invalid(
                "Quad and O-grid methods require a 2D planar fluid domain",
            ));
        }
        if t.method == MeshMethod::QuadDominant {
            export.document = export.document.with_appended_source(
                "\nMesh.RecombinationAlgorithm = 1;\nRecombine Surface {:};\n",
            );
            return Ok(export);
        }
        let face = self
            .fluid_face()
            .ok_or_else(|| invalid("Select a planar fluid face first"))?;
        let GeometryFaceRepresentation::Planar {
            outer_loop,
            inner_loops,
        } = &self
            .geometry
            .face(face)
            .ok_or_else(|| invalid("Missing fluid face"))?
            .representation
        else {
            return Err(invalid("Mapped meshing requires an analytic planar face"));
        };
        if t.method == MeshMethod::MappedQuad && outer_loop.len() != 4 && inner_loops.is_empty() {
            if let Some(mapped) = self.mapped_orthogonal_partition(&export, face, outer_loop)? {
                return Ok(mapped);
            }
        }
        if outer_loop.len() != 4
            || outer_loop.iter().any(|e| {
                !matches!(
                    self.geometry.edge(e.edge).map(|e| &e.geometry),
                    Some(EdgeGeometry::Line { .. })
                )
            })
        {
            return Err(invalid(
                "Mapped/O-grid requires a convex four-line outer boundary; partition the geometry into blocks first",
            ));
        }
        let outer = self.loop_data(&export, outer_loop)?;
        let pos = |id: VertexId| self.geometry.vertex(id).unwrap().position;
        let p = outer.iter().map(|(_, v)| pos(*v)).collect::<Vec<_>>();
        let normal = (p[1] - p[0]).cross(p[2] - p[1]);
        let norm = normal.norm();
        if norm <= 0.0
            || (0..4).any(|i| {
                (p[(i + 1) % 4] - p[i])
                    .cross(p[(i + 2) % 4] - p[(i + 1) % 4])
                    .dot(normal)
                    <= 0.0
            })
        {
            return Err(invalid(
                "Mapped outer boundary must be nondegenerate and convex",
            ));
        }
        let mut source = String::new();
        if t.method == MeshMethod::MappedQuad {
            if !inner_loops.is_empty() {
                return Err(invalid(
                    "A mapped quad face cannot contain holes. Choose O-grid for a single circular hole.",
                ));
            }
            for (i, edge) in outer.iter().enumerate().take(4) {
                let (n, r) = if i % 2 == 0 { (ni, ri) } else { (nj, rj) };
                // Opposite edges use the same physical grading direction.
                let tag = if i >= 2 { -edge.0 } else { edge.0 };
                source.push_str(&format!(
                    "Transfinite Curve {{{tag}}} = {} Using Progression {r:.17};\n",
                    n + 1
                ));
            }
            let surface = export
                .map
                .face_tag(face)
                .ok_or_else(|| invalid("Missing mapped surface"))?;
            source.push_str(&format!(
                "Transfinite Surface {{{surface}}};\nRecombine Surface {{{surface}}};\n"
            ));
        } else {
            if ri != 1.0 {
                return Err(invalid(
                    "O-grid tangential spacing is uniform; set I progression to 1",
                ));
            }
            if inner_loops.len() != 1 || inner_loops[0].len() != 4 {
                return Err(invalid(
                    "O-grid requires exactly one circular hole composed of four arcs",
                ));
            }
            let hole = &inner_loops[0];
            let mut center = None;
            for e in hole {
                let Some(EdgeGeometry::CircularArc { center: c, .. }) =
                    self.geometry.edge(e.edge).map(|e| &e.geometry)
                else {
                    return Err(invalid(
                        "O-grid hole must use exact circular arcs, not faceted edges",
                    ));
                };
                if center.is_some_and(|old| old != *c) {
                    return Err(invalid("O-grid arcs must share one circle centre"));
                }
                center = Some(*c);
            }
            let c = pos(center.unwrap());
            let mut radius: Option<f64> = None;
            for e in hole {
                let EdgeGeometry::CircularArc { start, end, .. } =
                    self.geometry.edge(e.edge).unwrap().geometry
                else {
                    unreachable!()
                };
                let a = pos(start) - c;
                let b = pos(end) - c;
                if radius.is_some_and(|r| (r - a.norm()).abs() > 1e-8 * r) {
                    return Err(invalid("O-grid arcs must have equal radii"));
                }
                radius = Some(a.norm());
                if (a.norm() - b.norm()).abs() > 1e-8 * a.norm()
                    || a.dot(b).abs() > 1e-8 * a.norm() * b.norm()
                {
                    return Err(invalid(
                        "O-grid requires four quarter-circle arcs with equal radius",
                    ));
                }
            }
            if (0..4).any(|i| (p[(i + 1) % 4] - p[i]).cross(c - p[i]).dot(normal) <= 0.0) {
                return Err(invalid(
                    "O-grid circle centre must be strictly inside the convex outer boundary",
                ));
            }
            let radius = radius.unwrap();
            if (0..4).any(|i| {
                let edge = p[(i + 1) % 4] - p[i];
                edge.cross(c - p[i]).dot(normal) / (edge.norm() * norm) <= radius * (1.0 + 1e-8)
            }) {
                return Err(invalid(
                    "O-grid circle must lie strictly inside every outer boundary edge",
                ));
            }
            let mut inner = self.loop_data(&export, hole)?;
            let q = inner.iter().map(|(_, v)| pos(*v)).collect::<Vec<_>>();
            if (q[1] - q[0]).cross(q[2] - q[1]).dot(normal) < 0.0 {
                inner = (0..4)
                    .rev()
                    .map(|i| (-inner[i].0, inner[(i + 1) % 4].1))
                    .collect();
            }
            // Pick a cyclic correspondence, never independently pair vertices.
            let shift = (0..4)
                .min_by(|a, b| {
                    let score = |s: usize| {
                        (0..4)
                            .map(|i| (pos(inner[(i + s) % 4].1) - p[i]).norm().powi(2))
                            .sum::<f64>()
                    };
                    score(*a).total_cmp(&score(*b))
                })
                .unwrap();
            inner.rotate_left(shift);
            // Reject severely twisted blocks instead of emitting inverted cells.
            if (0..4).any(|i| (pos(inner[i].1) - c).dot(p[i] - c) <= 0.0) {
                return Err(invalid(
                    "O-grid circle arcs are not aligned with the outer corners; rotate/partition the circular profile",
                ));
            }
            for i in 0..4 {
                let j = (i + 1) % 4;
                let block = [pos(inner[i].1), p[i], p[j], pos(inner[j].1)];
                if (0..4).any(|k| {
                    (block[(k + 1) % 4] - block[k])
                        .cross(block[(k + 2) % 4] - block[(k + 1) % 4])
                        .dot(normal)
                        <= 0.0
                }) {
                    return Err(invalid(
                        "O-grid corner correspondence creates a degenerate/twisted block; reposition or repartition the profile",
                    ));
                }
            }
            let original = export
                .map
                .face_tag(face)
                .ok_or_else(|| invalid("Missing O-grid surface"))?;
            let original_source = export.document.to_geo_string()?;
            // Replace only the fluid surface; preserve all physical curve groups.
            let mut base = original_source
                .lines()
                .filter(|l| !l.starts_with("Physical Surface("))
                .collect::<Vec<_>>()
                .join("\n");
            base.push_str(&format!("\nDelete {{ Surface{{{original}}}; }}\n"));
            // Allocate fresh tags above every exported entity tag.
            let max_tag = original_source
                .lines()
                .filter_map(|l| {
                    l.split_once('(')
                        .and_then(|(_, s)| s.split_once(')'))
                        .and_then(|(s, _)| s.parse::<usize>().ok())
                })
                .max()
                .unwrap_or(10000)
                + 100;
            for i in 0..4 {
                let a = export.map.vertex_tag(inner[i].1).unwrap();
                let b = export.map.vertex_tag(outer[i].1).unwrap();
                let spoke = max_tag + i;
                base.push_str(&format!("Line({spoke}) = {{{a}, {b}}};\nTransfinite Curve {{{spoke}}} = {} Using Progression {rj:.17};\n",nj+1));
            }
            for i in 0..4 {
                let j = (i + 1) % 4;
                let loop_tag = max_tag + 10 + i;
                let surf = max_tag + 20 + i;
                base.push_str(&format!("Curve Loop({loop_tag}) = {{{}, {}, -{}, {}}};\nPlane Surface({surf}) = {{{loop_tag}}};\nTransfinite Curve {{{}, {}}} = {};\nTransfinite Surface {{{surf}}};\nRecombine Surface {{{surf}}};\n",max_tag+i,outer[i].0,max_tag+j,-inner[i].0,outer[i].0,inner[i].0,ni+1));
            }
            base.push_str(&format!(
                "Physical Surface(\"fluid\") = {{{}, {}, {}, {}}};\n",
                max_tag + 20,
                max_tag + 21,
                max_tag + 22,
                max_tag + 23
            ));
            export
                .map
                .set_face_tags(face, (0..4).map(|i| max_tag + 20 + i).collect());
            export.document = crate::GmshGeoDocument::from_source(MeshDimension::TwoD, base);
        }
        export.document = export.document.with_appended_source(source);
        Ok(export)
    }
}

impl WorkbenchSession {
    fn mapped_orthogonal_partition(
        &self,
        export: &GeometryGmshExport,
        face: crate::FaceId,
        boundary: &[OrientedEdge],
    ) -> Result<Option<GeometryGmshExport>, WorkbenchError> {
        use std::collections::{BTreeMap, BTreeSet};
        use std::fmt::Write;
        let mut polygon = Vec::new();
        let mut original = Vec::new();
        for edge in boundary {
            let Some(EdgeGeometry::Line { start, end }) =
                self.geometry.edge(edge.edge).map(|e| &e.geometry)
            else {
                return Ok(None);
            };
            let (a, b) = if edge.reversed {
                (*end, *start)
            } else {
                (*start, *end)
            };
            let pa = self
                .geometry
                .vertex(a)
                .ok_or_else(|| invalid("Missing block vertex"))?
                .position;
            let pb = self
                .geometry
                .vertex(b)
                .ok_or_else(|| invalid("Missing block vertex"))?
                .position;
            if pa.z.abs() > 1e-12
                || pb.z.abs() > 1e-12
                || ((pa.x - pb.x).abs() > 1e-12 && (pa.y - pb.y).abs() > 1e-12)
            {
                return Ok(None);
            }
            polygon.push(pa);
            original.push((edge.edge, a, pa, pb));
        }
        if self
            .mesh_recipe
            .boundary_layers
            .iter()
            .any(|control| control.enabled)
        {
            return Err(invalid("Mapped blocks grade wall spacing from Minimum size; disable separate inflation fields"));
        }
        let axis = |component: usize| {
            let mut coordinates: Vec<f64> = polygon
                .iter()
                .map(|p| if component == 0 { p.x } else { p.y })
                .collect();
            coordinates.sort_by(f64::total_cmp);
            coordinates.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
            let mut split = Vec::new();
            for pair in coordinates.windows(2) {
                split.push(pair[0]);
                split.push(0.5 * (pair[0] + pair[1]));
            }
            split.push(*coordinates.last().unwrap());
            split
        };
        let xs = axis(0);
        let ys = axis(1);
        if xs.len() > 129 || ys.len() > 129 {
            return Err(invalid(
                "Mapped orthogonal partition exceeds 64 coordinate intervals",
            ));
        }
        let inside = |x: f64, y: f64| {
            let mut result = false;
            for i in 0..polygon.len() {
                let a = polygon[i];
                let b = polygon[(i + 1) % polygon.len()];
                if (a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x {
                    result = !result;
                }
            }
            result
        };
        let nx = xs.len();
        let point = |x: usize, y: usize| 1000 + y * nx + x;
        let mut used = BTreeSet::new();
        let mut blocks = Vec::new();
        for y in 0..ys.len() - 1 {
            for x in 0..xs.len() - 1 {
                if inside(0.5 * (xs[x] + xs[x + 1]), 0.5 * (ys[y] + ys[y + 1])) {
                    let corners = [
                        point(x, y),
                        point(x + 1, y),
                        point(x + 1, y + 1),
                        point(x, y + 1),
                    ];
                    used.extend(corners);
                    blocks.push(corners);
                }
            }
        }
        if blocks.is_empty() {
            return Err(invalid("Mapped partition has no fluid blocks"));
        }
        let mut lines: BTreeMap<(usize, usize), (usize, usize)> = BTreeMap::new();
        for corners in &blocks {
            for i in 0..4 {
                let (a, b) = (corners[i], corners[(i + 1) % 4]);
                let key = (a.min(b), a.max(b));
                let next = 2000 + lines.len();
                let value = lines.entry(key).or_insert((next, 0));
                value.1 += 1;
            }
        }
        let position = |tag: usize| {
            let index = tag - 1000;
            Vec3::new(xs[index % nx], ys[index / nx], 0.)
        };
        let mut source = String::from("SetFactory(\"Built-in\");\nMesh.MshFileVersion = 4.1;\n");
        let mut map = export.map.clone();
        for &tag in &used {
            let p = position(tag);
            writeln!(
                source,
                "Point({tag}) = {{{:.17}, {:.17}, 0, {:.17}}};",
                p.x, p.y, self.global_size
            )
            .unwrap();
        }
        let counts = [
            self.mesh_recipe.tuning.divisions[0].div_ceil(2).max(1),
            self.mesh_recipe.tuning.divisions[1].div_ceil(2).max(1),
        ];
        let start_sizes: [f64; 2] = std::array::from_fn(|axis| {
            let coordinates = if axis == 0 { &xs } else { &ys };
            let shortest = coordinates
                .windows(2)
                .map(|p| p[1] - p[0])
                .fold(f64::INFINITY, f64::min);
            self.mesh_recipe.tuning.mapped_start_sizes[axis]
                .unwrap_or(self.min_size)
                .min(shortest / counts[axis] as f64)
        });
        for (&(a, b), &(tag, _)) in &lines {
            let pa = position(a);
            let pb = position(b);
            writeln!(source, "Line({tag}) = {{{a},{b}}};").unwrap();
            let horizontal = (pa.y - pb.y).abs() < 1e-12;
            let index = if horizontal {
                (a - 1000) % nx
            } else {
                (a - 1000) / nx
            };
            let count = counts[usize::from(!horizontal)] as usize;
            let length = (pb - pa).norm();
            let target = start_sizes[usize::from(!horizontal)];
            let first = target.min(length / count as f64);
            let mut lo = 1.;
            let mut hi = 2.;
            let span = |ratio: f64| {
                if (ratio - 1.).abs() < 1e-12 {
                    first * count as f64
                } else {
                    first * (ratio.powi(count as i32) - 1.) / (ratio - 1.)
                }
            };
            while span(hi) < length && hi < 16. {
                hi *= 2.;
            }
            for _ in 0..64 {
                let mid = 0.5 * (lo + hi);
                if span(mid) < length {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let ratio = 0.5 * (lo + hi);
            let signed = if index % 2 == 0 {
                tag as i64
            } else {
                -(tag as i64)
            };
            writeln!(
                source,
                "Transfinite Curve {{{signed}}} = {} Using Progression {ratio:.17};",
                count + 1
            )
            .unwrap();
        }
        let mut surfaces = Vec::new();
        for (index, corners) in blocks.iter().enumerate() {
            let tag = 3000 + index;
            let mut loop_tags = Vec::new();
            for i in 0..4 {
                let (a, b) = (corners[i], corners[(i + 1) % 4]);
                let curve = lines[&(a.min(b), a.max(b))].0;
                loop_tags.push(if a < b { curve as i64 } else { -(curve as i64) });
            }
            writeln!(source,"Curve Loop({tag}) = {{{}}};\nPlane Surface({tag}) = {{{tag}}};\nTransfinite Surface {{{tag}}} = {{{},{},{},{}}};\nRecombine Surface {{{tag}}};",loop_tags.iter().map(ToString::to_string).collect::<Vec<_>>().join(","),corners[0],corners[1],corners[2],corners[3]).unwrap();
            surfaces.push(tag);
        }
        for &(edge, vertex, pa, pb) in &original {
            let mut tags = Vec::new();
            for (&(a, b), &(tag, uses)) in &lines {
                if uses != 1 {
                    continue;
                }
                let on = |p: Vec3| {
                    (pb - pa).cross(p - pa).norm() < 1e-10 * (pb - pa).norm()
                        && (p - pa).dot(p - pb) <= 1e-20
                };
                if on(position(a)) && on(position(b)) {
                    tags.push(tag);
                }
            }
            if tags.is_empty() {
                return Err(invalid("Mapped boundary edge was lost during partition"));
            }
            map.set_edge_tags(edge, tags);
            if let Some(&tag) = used
                .iter()
                .find(|&&tag| (position(tag) - pa).norm() < 1e-12)
            {
                map.set_vertex_tag(vertex, tag);
            }
        }
        let mut physical = 1;
        for (name, edges) in self.boundary_groups_2d()? {
            let tags: BTreeSet<_> = edges
                .into_iter()
                .flat_map(|edge| map.edge_tags(edge))
                .collect();
            if tags.is_empty() {
                return Err(invalid("Mapped boundary group has no curves"));
            }
            writeln!(
                source,
                "Physical Curve(\"{name}\", {physical}) = {{{}}};",
                tags.iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(",")
            )
            .unwrap();
            physical += 1;
        }
        writeln!(
            source,
            "Physical Surface(\"fluid\", {physical}) = {{{}}};",
            surfaces
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(",")
        )
        .unwrap();
        map.set_face_tags(face, surfaces);
        Ok(Some(GeometryGmshExport {
            document: crate::GmshGeoDocument::from_source(MeshDimension::TwoD, source),
            map,
        }))
    }
}
