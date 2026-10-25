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
