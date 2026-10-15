//! Subtractive extrusion of a face-hosted sketch. Failed kernel operations
//! leave the whole source document, including visibility and IDs, unchanged.
use super::*;

impl GeometryTopology {
    pub fn cad_pocket(
        &mut self,
        face: FaceId,
        depth: f64,
        through_all: bool,
        tolerance: f64,
    ) -> Result<BodyId, GeometryError> {
        primitives::positive(depth, "Pocket depth")?;
        if !tolerance.is_finite() || tolerance <= 1e-6 || tolerance > 1.0 {
            return Err(error(
                "Pocket tolerance must be greater than 1e-6 and at most 1 model unit",
            ));
        }
        let sketch = self
            .sketches()
            .find(|s| s.profile_face == Some(face))
            .cloned()
            .ok_or_else(|| error("Pocket requires a completed sketch on a solid's planar face"))?;
        let host = sketch
            .host_face
            .ok_or_else(|| error("Select a sketch attached to the solid, not a principal plane"))?;
        let base = self
            .cad_owner(host)
            .ok_or_else(|| error("Pocket host must be a kernel solid"))?;
        let host_frame = self.sketch_frame_for_face(host)?;
        let profile_frame = self.planar_face_frame(face)?;
        let normal = host_frame.normal;
        if normal.dot(profile_frame.normal).abs() < 1.0 - 1e-7
            || (profile_frame.origin - host_frame.origin).dot(normal).abs() > tolerance
        {
            return Err(error("Pocket sketch must lie on its saved host plane"));
        }
        let origin = sketch.frame.origin;
        let extent = self
            .cad_body(base)
            .unwrap()
            .face_meshes
            .iter()
            .flat_map(|f| f.triangles.iter().flatten())
            .map(|p| (*p - origin).dot(normal) * -1.0)
            .fold(0.0f64, f64::max);
        let clearance = (tolerance * 4.0).max(extent * 1e-6).max(1e-5);
        let depth = if through_all {
            let (min, max) = self
                .cad_body(base)
                .unwrap()
                .face_meshes
                .iter()
                .flat_map(|f| f.triangles.iter().flatten())
                .fold(
                    (
                        Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                        Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                    ),
                    |(a, b), p| {
                        (
                            Vec3::new(a.x.min(p.x), a.y.min(p.y), a.z.min(p.z)),
                            Vec3::new(b.x.max(p.x), b.y.max(p.y), b.z.max(p.z)),
                        )
                    },
                );
            // Extend beyond the display-derived bounds by more than the
            // tessellation chord tolerance, including on curved back surfaces.
            extent + clearance.max((max - min).norm() * 0.01)
        } else {
            depth
        };
        primitives::positive(depth, "Resolved Pocket depth")?;
        // Start outside the material to avoid a coincident Boolean cap; the
        // blind end remains exactly `depth` from the original sketch plane.
        let profile = builder::translated(&self.cad_profile(face)?, vector(normal * clearance));
        let solid = builder::tsweep(&profile, vector(normal * (-depth - clearance)));
        let before = display_volume(self.cad_body(base).unwrap());
        let mut next = self.clone();
        let tool = next.cad_insert(solid, "Pocket tool", "Subtractive extrusion tool")?;
        next.cad_bodies.get_mut(&tool).unwrap().dependencies = Some(CadDependencies {
            bodies: vec![base],
            profiles: vec![face],
            replayable_pad: false,
        });
        let body = next.cad_boolean(base, tool, CadBoolean::Difference, tolerance)?;
        let after = display_volume(next.cad_body(body).unwrap());
        if after >= before * (1.0 - 1e-8) {
            return Err(error(
                "No resolvable material was removed. Check profile location, depth and tolerance",
            ));
        }
        let id = CadFeatureId(next.next_feature_id);
        next.next_feature_id += 1;
        next.pocket_features.insert(
            id,
            CadPocketFeature {
                id,
                sketch: sketch.id,
                source_face: face,
                base,
                tool,
                body,
                depth,
                through_all,
                tolerance,
            },
        );
        let result = next.cad_bodies.get_mut(&body).unwrap();
        result.name = format!("Pocket {}", id.get());
        result.operation = format!(
            "Pocket Sketch {} · {}",
            sketch.id.get(),
            if through_all {
                "Through all".into()
            } else {
                format!("Depth {depth}")
            }
        );
        next.hidden_profiles.insert(face);
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(body)
    }
}

fn display_volume(body: &CadBody) -> f64 {
    let origin = body.face_meshes[0].triangles[0][0];
    body.face_meshes
        .iter()
        .flat_map(|f| f.triangles.iter())
        .map(|t| (t[0] - origin).dot((t[1] - origin).cross(t[2] - origin)) / 6.0)
        .sum::<f64>()
        .abs()
}
