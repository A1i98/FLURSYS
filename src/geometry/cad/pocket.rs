//! Subtractive extrusion of a face-hosted sketch. Failed kernel operations
//! leave the whole source document, including visibility and IDs, unchanged.
use super::*;

impl GeometryTopology {
    /// Edit an existing subtractive feature, keeping its sketch, tool and result
    /// identities. The entire downstream replay is transactional.
    pub fn cad_edit_pocket(
        &mut self,
        id: CadFeatureId,
        depth: f64,
        through_all: bool,
        tolerance: f64,
    ) -> Result<Vec<BodyId>, GeometryError> {
        self.cad_edit_pocket_to_face(id, depth, through_all, None, tolerance)
    }

    pub fn cad_edit_pocket_to_face(
        &mut self,
        id: CadFeatureId,
        depth: f64,
        through_all: bool,
        stop_face: Option<FaceId>,
        tolerance: f64,
    ) -> Result<Vec<BodyId>, GeometryError> {
        primitives::positive(depth, "Pocket depth")?;
        if !tolerance.is_finite() || tolerance <= 1e-6 || tolerance > 1.0 {
            return Err(error("Pocket tolerance must be in (1e-6,1]"));
        }
        let mut next = self.clone();
        let feature = next
            .pocket_features
            .get(&id)
            .ok_or_else(|| error("Pocket feature not found"))?
            .clone();
        if !matches!(
            next.cad_bodies[&feature.tool].recipe,
            None | Some(CadRecipe::PocketTool { .. })
        ) || !matches!(
            next.cad_bodies[&feature.body].recipe,
            None | Some(CadRecipe::Boolean {
                operation: CadBoolean::Difference,
                ..
            })
        ) {
            return Err(error(
                "Pocket was transformed; edit the untransformed source feature instead",
            ));
        }
        next.cad_bodies.get_mut(&feature.tool).unwrap().recipe = Some(CadRecipe::PocketTool {
            face: feature.source_face,
            depth,
            through_all,
            stop_face,
            tolerance,
        });
        next.cad_bodies.get_mut(&feature.body).unwrap().recipe = Some(CadRecipe::Boolean {
            left: feature.base,
            right: feature.tool,
            operation: CadBoolean::Difference,
            tolerance,
        });
        let saved = next.pocket_features.get_mut(&id).unwrap();
        saved.depth = depth;
        saved.through_all = through_all;
        saved.stop_face = stop_face;
        saved.tolerance = tolerance;
        let rebuilt = next.regenerate_features(feature.sketch)?;
        let before = display_volume(next.cad_body(feature.base).unwrap());
        let after = display_volume(next.cad_body(feature.body).unwrap());
        if after >= before * (1.0 - 1e-8) {
            return Err(error("Edited Pocket removes no resolvable material"));
        }
        let result = next.cad_bodies.get_mut(&feature.body).unwrap();
        result.operation = format!(
            "Pocket Sketch {} · {}",
            feature.sketch.get(),
            if through_all {
                "Through all".into()
            } else if let Some(face) = stop_face {
                format!("Up to Face #{}", face.get())
            } else {
                format!("Depth {depth}")
            }
        );
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(rebuilt)
    }

    pub fn cad_pocket(
        &mut self,
        face: FaceId,
        depth: f64,
        through_all: bool,
        tolerance: f64,
    ) -> Result<BodyId, GeometryError> {
        self.cad_pocket_to_face(face, depth, through_all, None, tolerance)
    }

    pub fn cad_pocket_to_face(
        &mut self,
        face: FaceId,
        depth: f64,
        through_all: bool,
        stop_face: Option<FaceId>,
        tolerance: f64,
    ) -> Result<BodyId, GeometryError> {
        let (solid, depth, base, sketch_id) =
            self.pocket_tool_solid(face, depth, through_all, stop_face, tolerance)?;
        let before = display_volume(self.cad_body(base).unwrap());
        let mut next = self.clone();
        let tool = next.cad_insert(solid, "Pocket tool", "Subtractive extrusion tool")?;
        next.cad_bodies.get_mut(&tool).unwrap().dependencies = Some(CadDependencies {
            bodies: vec![base],
            profiles: vec![face],
            replayable_pad: false,
        });
        next.cad_bodies.get_mut(&tool).unwrap().recipe = Some(CadRecipe::PocketTool {
            face,
            depth,
            through_all,
            stop_face,
            tolerance,
        });
        {
            let data = next.cad_bodies.get_mut(&tool).unwrap();
            for (i, mesh) in Arc::make_mut(&mut data.face_meshes).iter_mut().enumerate() {
                mesh.semantic_name = Some(format!("pocket-tool:{face:?}:face:{i}"));
            }
        }
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
                sketch: sketch_id,
                source_face: face,
                base,
                tool,
                body,
                depth,
                through_all,
                stop_face,
                tolerance,
            },
        );
        let result = next.cad_bodies.get_mut(&body).unwrap();
        result.name = format!("Pocket {}", id.get());
        result.operation = format!(
            "Pocket Sketch {} · {}",
            sketch_id.get(),
            if through_all {
                "Through all".into()
            } else if stop_face.is_some() {
                format!("Up to Face #{}", stop_face.map(|face| face.get()).unwrap_or_default())
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
    pub(super) fn pocket_tool_solid(
        &self,
        face: FaceId,
        depth: f64,
        through_all: bool,
        stop_face: Option<FaceId>,
        tolerance: f64,
    ) -> Result<(Solid, f64, BodyId, CadSketchId), GeometryError> {
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
        if through_all && stop_face.is_some() {
            return Err(error("Pocket end condition must be either Through all or Up to face"));
        }
        let depth = if let Some(stop) = stop_face {
            if self.cad_owner(stop) != Some(base) {
                return Err(error("Pocket stop face must belong to the same source body"));
            }
            let stop_frame = self.sketch_frame_for_face(stop)?;
            if stop_frame.normal.dot(normal).abs() < 1.0 - 1.0e-7 {
                return Err(error("Up to face currently requires a planar face parallel to the sketch plane"));
            }
            let travel = -normal;
            let distance = (stop_frame.origin - origin).dot(travel);
            if !distance.is_finite() || distance <= tolerance {
                return Err(error("Pocket stop face must lie ahead of the sketch along the cut direction"));
            }
            // `pocket_tool_solid` starts outside the host by `clearance` and
            // sweeps back across that offset, so this depth places the end
            // cap on the selected stop plane.
            distance
        } else if through_all {
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
        Ok((solid, depth, base, sketch.id))
    }
}

pub(super) fn display_volume(body: &CadBody) -> f64 {
    let origin = body.face_meshes[0].triangles[0][0];
    body.face_meshes
        .iter()
        .flat_map(|f| f.triangles.iter())
        .map(|t| (t[0] - origin).dot((t[1] - origin).cross(t[2] - origin)) / 6.0)
        .sum::<f64>()
        .abs()
}
