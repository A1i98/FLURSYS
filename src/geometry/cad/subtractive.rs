//! Compound subtractive tools publish their entire construction atomically.
use super::*;
impl GeometryTopology {
    pub fn cad_hole(
        &mut self,
        host: FaceId,
        x: f64,
        y: f64,
        diameter: f64,
        depth: f64,
        through_all: bool,
        tolerance: f64,
    ) -> Result<BodyId, GeometryError> {
        primitives::positive(diameter, "Hole diameter")?;
        if !x.is_finite() || !y.is_finite() {
            return Err(error("Hole center must be finite"));
        }
        if self.cad_owner(host).is_none() {
            return Err(error("Hole requires a planar solid face"));
        }
        let mut next = self.clone();
        let sketch = next.create_sketch_on_face(host)?;
        let face = next.materialize_sketch_circle_at(sketch.id, x, y, diameter * 0.5)?;
        let body = next.cad_pocket(face, depth, through_all, tolerance)?;
        next.cad_rename(body, "Hole")?;
        next.validate()?;
        *self = next;
        Ok(body)
    }
    pub fn cad_revolved_cut(
        &mut self,
        face: FaceId,
        pivot: Vec3,
        axis: Vec3,
        degrees: f64,
        reference: Option<CadAxisReference>,
        reversed: bool,
        tolerance: f64,
    ) -> Result<BodyId, GeometryError> {
        let host = self
            .sketches()
            .find(|s| s.profile_face == Some(face))
            .and_then(|s| s.host_face)
            .ok_or_else(|| error("Revolved cut requires a sketch attached to a solid"))?;
        let base = self
            .cad_owner(host)
            .ok_or_else(|| error("Revolved cut support has no solid"))?;
        let mut next = self.clone();
        let tool = if let Some(reference) = reference {
            next.cad_revolve_with_reference(face, reference, degrees, reversed)?
        } else {
            next.cad_revolve(face, pivot, axis, degrees)?
        };
        let body = next.cad_boolean(base, tool, CadBoolean::Difference, tolerance)?;
        let before = super::pocket::display_volume(next.cad_body(base).unwrap());
        let after = super::pocket::display_volume(next.cad_body(body).unwrap());
        if after >= before * (1. - 1e-8) {
            return Err(error("Revolved cut removes no resolvable material; check axis, profile, angle and tolerance"));
        }
        next.cad_rename(body, "Revolved cut")?;
        next.hidden_profiles.insert(face);
        next.validate()?;
        *self = next;
        Ok(body)
    }
}
