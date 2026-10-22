//! Parametric sketch publication and exact curve transfer to the B-rep kernel.
//! Freeform edge construction; see THIRD_PARTY_NOTICES.txt for attribution.
use super::*;
use cad_geom::{Loop2, Region2, Seg2, Vec2};

fn world(frame: CadPlaneFrame, p: Vec2) -> Vec3 {
    frame.origin + frame.x_axis * (p.x * 0.001) + frame.y_axis * (p.y * 0.001)
}

pub(super) fn exact_face(
    frame: CadPlaneFrame,
    region: &Region2,
) -> Result<truck_modeling::Face, GeometryError> {
    let mut wires = Vec::new();
    for (index, original) in std::iter::once(&region.outer)
        .chain(region.holes.iter())
        .enumerate()
    {
        let lp = if (original.signed_area() > 0.0) != (index == 0) {
            original.reversed()
        } else {
            original.clone()
        };
        if lp.segs.is_empty() || lp.segs.len() > 20_000 || !lp.is_closed(1e-6) {
            return Err(error("Drawing profile is not a closed loop"));
        }
        let verts: Vec<_> = lp
            .segs
            .iter()
            .map(|s| builder::vertex(point(world(frame, s.start()))))
            .collect();
        let mut wire = Wire::new();
        for (i, s) in lp.segs.iter().enumerate() {
            let (a, b) = (&verts[i], &verts[(i + 1) % verts.len()]);
            match *s {
                Seg2::Line { .. } => wire.push_back(builder::line(a, b)),
                Seg2::Arc { sweep, .. } => {
                    if sweep.abs() < 1e-12 {
                        return Err(error("Zero sweep in profile"));
                    }
                    if sweep.abs() >= std::f64::consts::PI * 1.5 {
                        let mid = builder::vertex(point(world(frame, s.point_at(0.5))));
                        wire.push_back(builder::circle_arc(
                            a,
                            &mid,
                            point(world(frame, s.point_at(0.25))),
                        ));
                        wire.push_back(builder::circle_arc(
                            &mid,
                            b,
                            point(world(frame, s.point_at(0.75))),
                        ));
                    } else {
                        wire.push_back(builder::circle_arc(
                            a,
                            b,
                            point(world(frame, s.point_at(0.5))),
                        ));
                    }
                }
                Seg2::Cubic { p1, p2, .. } => wire.push_back(builder::bezier(
                    a,
                    b,
                    vec![point(world(frame, p1)), point(world(frame, p2))],
                )),
                Seg2::Conic { apex, w, .. } => {
                    let (pa, pb, px) = (a.point(), b.point(), point(world(frame, apex)));
                    let ctrl = vec![
                        truck_modeling::Vector4::new(pa.x, pa.y, pa.z, 1.0),
                        truck_modeling::Vector4::new(px.x * w, px.y * w, px.z * w, w),
                        truck_modeling::Vector4::new(pb.x, pb.y, pb.z, 1.0),
                    ];
                    let curve = truck_modeling::NurbsCurve::new(truck_modeling::BSplineCurve::new(
                        truck_modeling::KnotVec::bezier_knot(2),
                        ctrl,
                    ));
                    wire.push_back(truck_modeling::Edge::new(
                        a,
                        b,
                        truck_modeling::Curve::NurbsCurve(curve),
                    ));
                }
                Seg2::Bezier { n, p } => {
                    let inner = p
                        .get(1..usize::from(n))
                        .ok_or_else(|| error("Invalid Bezier degree"))?;
                    wire.push_back(builder::bezier(
                        a,
                        b,
                        inner.iter().map(|p| point(world(frame, *p))).collect(),
                    ));
                }
            }
        }
        wires.push(wire);
    }
    builder::try_attach_plane(&wires)
        .map_err(|e| error(format!("Invalid exact drawing profile: {e}")))
}

impl GeometryTopology {
    /// Open sketch wires remain visible in the shared CAD viewport, in metres.
    pub fn cad_open_drawing_paths(&self) -> Vec<Vec<Vec3>> {
        self.sketches()
            .filter(|s| s.profile_face.is_none())
            .filter_map(|s| s.drawing.as_ref().map(|d| (s.frame, d)))
            .flat_map(|(frame, d)| {
                (0..d.curves.len())
                    .filter(|i| !d.curves[*i].construction)
                    .map(move |i| d.polyline(i).into_iter().map(|p| world(frame, p)).collect())
            })
            .collect()
    }

    /// Commit a whole sketch atomically. Existing profile IDs remain attached to
    /// the same source-curve boundaries; incompatible dependent edits fail.
    pub fn cad_set_drawing(
        &mut self,
        id: CadSketchId,
        mut drawing: cad_sketch::Sketch,
    ) -> Result<Vec<FaceId>, GeometryError> {
        crate::sketching::validate(&drawing).map_err(error)?;
        let report = cad_sketch::solve(&mut drawing);
        if !report.ok() {
            return Err(error("The drawing constraints are inconsistent"));
        }
        let profiles = cad_sketch::find_profiles(&drawing);
        if profiles.len() > 256 {
            return Err(error("A sketch may publish at most 256 regions"));
        }
        let mut next = self.clone();
        let owner = next.cad_sketch_owner(id)?;
        let root = next
            .sketch(owner)
            .ok_or_else(|| error("Sketch not found"))?
            .clone();
        let old = next
            .sketches()
            .filter(|s| s.id == owner || s.region_owner == Some(owner))
            .cloned()
            .collect::<Vec<_>>();
        let mut used = BTreeSet::new();
        let mut faces = Vec::new();
        for profile in profiles {
            let mut key = vec![profile.outer_curves.clone()];
            key.extend(profile.hole_curves.clone());
            for ids in &mut key {
                ids.sort();
            }
            if key.len() > 1 {
                key[1..].sort();
            }
            let existing = old
                .iter()
                .find(|s| s.drawing_boundary == key && !used.contains(&s.id));
            let record = if let Some(s) = existing {
                s.clone()
            } else if faces.is_empty() && (root.profile_face.is_none() || root.drawing.is_none()) {
                root.clone()
            } else {
                let mut s = next.insert_sketch(root.plane, root.frame, root.host_face)?;
                s.region_owner = Some(owner);
                s
            };
            let mut loops = Vec::new();
            for lp in std::iter::once(&profile.region.outer).chain(profile.region.holes.iter()) {
                loops.push(next.drawing_loop(root.frame, lp)?);
            }
            let outer = loops.remove(0);
            let temporary = next.add_planar_face(outer, loops)?;
            let face = if let Some(face) = record.profile_face {
                let mut replacement = next
                    .faces
                    .remove(&temporary)
                    .ok_or_else(|| error("Profile transfer failed"))?;
                let previous = next
                    .faces
                    .get(&face)
                    .ok_or_else(|| error("Old profile missing"))?
                    .representation
                    .clone();
                let old_edges = face_edges(&previous);
                let new_edges = face_edges(&replacement.representation);
                let compatible = record.drawing_boundary == key
                    && match (&previous, &replacement.representation) {
                        (
                            GeometryFaceRepresentation::Planar {
                                outer_loop: a,
                                inner_loops: aa,
                            },
                            GeometryFaceRepresentation::Planar {
                                outer_loop: b,
                                inner_loops: bb,
                            },
                        ) => {
                            a.len() == b.len()
                                && aa.len() == bb.len()
                                && aa.iter().zip(bb).all(|(a, b)| a.len() == b.len())
                        }
                        _ => false,
                    }
                    && old_edges.iter().zip(&new_edges).all(|(a, b)| {
                        std::mem::discriminant(&next.edges[a].geometry)
                            == std::mem::discriminant(&next.edges[b].geometry)
                    });
                if compatible {
                    for (old, new) in old_edges.iter().zip(new_edges) {
                        let edge = next
                            .edges
                            .remove(&new)
                            .ok_or_else(|| error("Replacement edge missing"))?;
                        next.edges
                            .get_mut(old)
                            .ok_or_else(|| error("Referenced edge missing"))?
                            .geometry = edge.geometry;
                    }
                    replacement.representation = previous;
                } else if next.cad_bodies.values().any(|body| {
                    body.recipe
                        .as_ref()
                        .is_some_and(|r| recipe_uses_edges(r, &old_edges))
                }) {
                    return Err(error("The edit changes the topology of an edge used as a feature axis; reassign the axis before editing"));
                }
                next.faces
                    .get_mut(&face)
                    .ok_or_else(|| error("Old profile missing"))?
                    .representation = replacement.representation;
                face
            } else {
                temporary
            };
            let mut record = record;
            record.profile_face = Some(face);
            record.profile = None;
            record.drawing_region = Some(profile.region);
            record.drawing_boundary = key;
            used.insert(record.id);
            next.sketches.insert(record.id, record);
            faces.push(face);
        }
        // Removing a region with a dependent solid would silently rebind the
        // feature, so protect it before publishing anything.
        for record in &old {
            if used.contains(&record.id) {
                continue;
            }
            if let Some(face) = record.profile_face {
                if next.cad_bodies().any(|(_, b)| {
                    b.dependencies
                        .as_ref()
                        .is_some_and(|d| d.profiles.contains(&face))
                }) {
                    return Err(error("A dependent feature uses a removed/replaced region; retain its source curves or remove the feature first"));
                }
                next.faces.remove(&face);
                next.hidden_profiles.remove(&face);
            }
            if record.id == owner {
                let s = next
                    .sketches
                    .get_mut(&owner)
                    .ok_or_else(|| error("Sketch not found"))?;
                s.profile_face = None;
                s.drawing_region = None;
                s.drawing_boundary.clear();
            } else {
                next.sketches.remove(&record.id);
            }
        }
        let owner_record = next
            .sketches
            .get_mut(&owner)
            .ok_or_else(|| error("Sketch owner missing"))?;
        owner_record.drawing = Some(drawing);
        owner_record.drawing_dof = Some(report.dof);
        next.regenerate_from(faces.iter().copied().collect(), BTreeSet::new())?;
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(faces)
    }

    fn drawing_loop(
        &mut self,
        frame: CadPlaneFrame,
        lp: &Loop2,
    ) -> Result<Vec<OrientedEdge>, GeometryError> {
        if lp.segs.is_empty() || !lp.is_closed(1e-6) {
            return Err(error("Sketch loop is open"));
        }
        let mut pieces = Vec::new();
        for seg in &lp.segs {
            match *seg {
                Seg2::Line { a, b } => pieces.push((a, b, None)),
                Seg2::Arc { center, sweep, .. } => {
                    let count =
                        (sweep.abs() / std::f64::consts::FRAC_PI_2).ceil().max(1.0) as usize;
                    for i in 0..count {
                        pieces.push((
                            seg.point_at(i as f64 / count as f64),
                            seg.point_at((i + 1) as f64 / count as f64),
                            Some(center),
                        ));
                    }
                }
                _ => {
                    let points = seg.polyline(1e-4);
                    for pair in points.windows(2) {
                        pieces.push((pair[0], pair[1], None));
                    }
                }
            }
        }
        if pieces.len() > 20_000 {
            return Err(error("Display boundary exceeds 20000 segments"));
        }
        let verts = pieces
            .iter()
            .map(|(a, _, _)| self.add_vertex(world(frame, *a)))
            .collect::<Result<Vec<_>, _>>()?;
        let mut edges = Vec::new();
        for (i, (_, _, center)) in pieces.iter().enumerate() {
            let (a, b) = (verts[i], verts[(i + 1) % verts.len()]);
            let edge = if let Some(c) = center {
                let c = self.add_vertex(world(frame, *c))?;
                self.add_circular_arc(a, c, b)?
            } else {
                self.add_line(a, b)?
            };
            edges.push(OrientedEdge {
                edge,
                reversed: false,
            });
        }
        Ok(edges)
    }
}

fn recipe_uses_edges(recipe: &CadRecipe, edges: &[EdgeId]) -> bool {
    match recipe {
        CadRecipe::Revolve {
            axis_reference: Some(CadAxisReference::Edge(e)),
            ..
        } => edges.contains(e),
        CadRecipe::Modified { source, .. } => recipe_uses_edges(source, edges),
        _ => false,
    }
}

impl GeometryTopology {
    /// Import an evaluated sketch in its explicit world frame (all coordinates in metres).
    pub fn cad_import_drawing_frame(
        &mut self,
        frame: CadPlaneFrame,
        drawing: cad_sketch::Sketch,
    ) -> Result<CadSketchId, GeometryError> {
        let mut next = self.clone();
        let sketch = next.insert_sketch(CadSketchPlane::Xy, frame, None)?;
        next.cad_set_drawing(sketch.id, drawing)?;
        *self = next;
        Ok(sketch.id)
    }
}
