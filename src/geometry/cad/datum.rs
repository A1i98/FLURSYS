//! Offset sketch planes retain their signed distance from the principal/host plane.
use super::*;

impl GeometryTopology {
    pub fn cad_create_offset_sketch(
        &mut self,
        plane: CadSketchPlane,
        host: Option<FaceId>,
        offset: f64,
    ) -> Result<CadSketch, GeometryError> {
        if !offset.is_finite() {
            return Err(error("Sketch offset must be finite"));
        }
        let mut frame = if plane == CadSketchPlane::Face {
            self.sketch_frame_for_face(host.ok_or_else(|| error("Select a planar host face"))?)?
        } else {
            if host.is_some() {
                return Err(error(
                    "A principal-plane sketch cannot also have a face host",
                ));
            }
            GeometryTopology::new().create_sketch_on_plane(plane)?.frame
        };
        frame.origin = frame.origin + frame.normal * offset;
        finite(frame.origin)?;
        let sketch = self.insert_sketch(plane, frame, host)?;
        self.sketches.get_mut(&sketch.id).unwrap().attachment_offset = offset;
        Ok(self.sketches[&sketch.id].clone())
    }

    /// Offset and dimensions form one candidate: dependent solids never see
    /// an intermediate offset with old dimensions published to the document.
    pub fn cad_edit_sketch_with_offset(
        &mut self,
        id: CadSketchId,
        x: f64,
        y: f64,
        profile: CadSketchProfile,
        offset: f64,
    ) -> Result<Vec<BodyId>, GeometryError> {
        if !offset.is_finite() {
            return Err(error("Sketch offset must be finite"));
        }
        let mut next = self.clone();
        let sketch = next
            .sketch(id)
            .ok_or_else(|| error("Sketch not found"))?
            .clone();
        if offset!=sketch.attachment_offset && next.cad_sketch_region_count(id)>1 {
            return Err(error("This sketch has multiple regions sharing one support. Independent region plane offsets are not allowed"));
        }
        let delta = sketch.frame.normal * (offset - sketch.attachment_offset);
        finite(delta)?;
        let origin = sketch.frame.origin + delta;
        finite(origin)?;
        if offset != sketch.attachment_offset {
            if let Some(face) = sketch.profile_face {
                for vertex in next.profile_vertices(face)? {
                    let p = next.vertices[&vertex].position + delta;
                    finite(p)?;
                    next.vertices.get_mut(&vertex).unwrap().position = p;
                }
            }
            let record = next.sketches.get_mut(&id).unwrap();
            record.frame.origin = origin;
            record.attachment_offset = offset;
        }
        let rebuilt = next.cad_edit_sketch(id, x, y, profile)?;
        *self = next;
        Ok(rebuilt)
    }
}
