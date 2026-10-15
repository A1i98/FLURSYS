//! Sketch profiles and their persistent source-to-result feature links.
use super::*;

impl GeometryTopology {
    pub(crate) fn register_xy_sketch_face(
        &mut self,
        face: FaceId,
    ) -> Result<CadSketchId, GeometryError> {
        self.planar_face_frame(face)?;
        let sketch = self.create_sketch_on_plane(CadSketchPlane::Xy)?;
        self.sketches.get_mut(&sketch.id).unwrap().profile_face = Some(face);
        self.bump_revision();
        Ok(sketch.id)
    }

    /// Adopt a newly drawn XY profile into the persistent design tree.
    pub(crate) fn register_xy_sketch_profile(
        &mut self,
        face: FaceId,
        profile: CadSketchProfile,
    ) -> Result<CadSketchId, GeometryError> {
        self.planar_face_frame(face)?;
        let sketch = self.create_sketch_on_plane(CadSketchPlane::Xy)?;
        let record = self.sketches.get_mut(&sketch.id).unwrap();
        record.profile_face = Some(face);
        record.profile = Some(profile);
        self.bump_revision();
        Ok(sketch.id)
    }

    /// An exact circle in the sketch frame, represented by four quarter arcs.
    /// Publication is atomic, including allocator and revision changes.
    pub fn materialize_sketch_circle_at(
        &mut self,
        sketch: CadSketchId,
        x: f64,
        y: f64,
        radius: f64,
    ) -> Result<FaceId, GeometryError> {
        if !x.is_finite() || !y.is_finite() || !radius.is_finite() || radius <= GEOMETRY_EPSILON {
            return Err(error("Circle requires a finite center and positive radius"));
        }
        let mut next = self.clone();
        let record = next
            .sketch(sketch)
            .ok_or_else(|| error("Sketch not found"))?
            .clone();
        if record.profile_face.is_some() {
            return Err(error("Sketch already has a profile"));
        }
        let f = record.frame;
        let center = f.origin + f.x_axis * x + f.y_axis * y;
        let center_id = next.add_vertex(center)?;
        let mut vertices = Vec::new();
        for delta in [f.x_axis, f.y_axis, f.x_axis * -1.0, f.y_axis * -1.0] {
            vertices.push(next.add_vertex(center + delta * radius)?);
        }
        let mut edges = Vec::new();
        for i in 0..4 {
            edges.push(OrientedEdge {
                edge: next.add_circular_arc(vertices[i], center_id, vertices[(i + 1) % 4])?,
                reversed: false,
            });
        }
        let face = next.add_planar_face(edges, Vec::new())?;
        let record = next.sketches.get_mut(&sketch).unwrap();
        record.profile = Some(CadSketchProfile::Circle { radius });
        record.profile_face = Some(face);
        next.bump_revision();
        *self = next;
        Ok(face)
    }

    /// Pad a profile, optionally joining its host body. Failed joins leave the
    /// complete document unchanged. The sketch and feature survive save/load.
    pub fn cad_pad(
        &mut self,
        face: FaceId,
        distance: f64,
        additive: bool,
        tolerance: f64,
    ) -> Result<BodyId, GeometryError> {
        let sketch = self
            .sketches()
            .find(|s| s.profile_face == Some(face))
            .cloned();
        let mut next = self.clone();
        let tool = next.cad_extrude(face, distance)?;
        let body = if additive {
            let host = sketch
                .as_ref()
                .and_then(|s| s.host_face)
                .and_then(|f| next.cad_owner(f))
                .ok_or_else(|| error("Additive Pad requires a sketch on a solid's planar face"))?;
            next.cad_boolean(host, tool, CadBoolean::Union, tolerance)?
        } else {
            tool
        };
        if let Some(sketch) = sketch {
            let id = CadFeatureId(next.next_feature_id);
            next.next_feature_id += 1;
            next.extrude_features.insert(
                id,
                CadExtrudeFeature {
                    id,
                    sketch: sketch.id,
                    source_face: face,
                    body,
                    distance: Some(distance),
                    additive,
                },
            );
        }
        next.hidden_profiles.insert(face);
        next.bump_revision();
        *self = next;
        Ok(body)
    }
}
