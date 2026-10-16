//! Additional exact solid operations. Construction completes before publication.
use super::*;

impl GeometryTopology {
    /// Rebuild only the display cache; exact B-rep and stable face IDs are retained.
    /// STL, OBJ and faceted volume meshing then use this same approximation.
    pub fn cad_retessellate(&mut self, id: BodyId, tolerance: f64) -> Result<(), GeometryError> {
        let solid = self.cad_solid(id)?;
        let (_, meshes) = prepare_with_tolerance(&solid, Some(tolerance))?;
        let body = self
            .cad_body(id)
            .ok_or_else(|| error("Select a CAD body"))?;
        if meshes.len() != body.face_meshes.len() {
            return Err(error("Tessellation changed face count"));
        }
        let display = body
            .face_meshes
            .iter()
            .zip(meshes)
            .map(|(old, triangles)| CadFaceMesh {
                face: old.face,
                semantic_name: old.semantic_name.clone(),
                plane: old.plane,
                triangles,
            })
            .collect();
        self.cad_bodies.get_mut(&id).unwrap().face_meshes = Arc::new(display);
        self.bump_revision();
        Ok(())
    }

    /// Validate exact boundary geometry against its topological connectivity.
    /// Display triangles are not the source for this check.
    pub fn cad_check_solid(&self, id: BodyId) -> Result<(), GeometryError> {
        let solid = self.cad_solid(id)?;
        Solid::try_new(solid.boundaries().clone())
            .map_err(|e| error(format!("Solid topology: {e}")))?;
        if !solid.is_geometric_consistent() {
            return Err(error(
                "Exact edge/surface geometry is inconsistent with solid topology",
            ));
        }
        Ok(())
    }

    /// Piecewise ruled loft through ordered sections, including inner loops.
    /// Section order is explicit; correspondence follows each wire's edge order.
    pub fn cad_loft_sections(&mut self, sections: &[FaceId]) -> Result<BodyId, GeometryError> {
        if !(2..=32).contains(&sections.len())
            || sections.iter().copied().collect::<BTreeSet<_>>().len() != sections.len()
        {
            return Err(error("Loft requires 2..32 different ordered profiles"));
        }
        let profiles = sections
            .iter()
            .map(|id| self.cad_profile(*id))
            .collect::<Result<Vec<_>, _>>()?;
        let mut shell = truck_modeling::Shell::new();
        for pair in profiles.windows(2) {
            let a = pair[0].boundaries();
            let b = pair[1].boundaries();
            if a.len() != b.len() {
                return Err(error("All loft sections need matching boundary loops"));
            }
            for (a, b) in a.iter().zip(&b) {
                let mut sides = builder::try_wire_homotopy(a, b)
                    .map_err(|e| error(format!("Loft correspondence: {e}")))?;
                shell.append(&mut sides);
            }
        }
        let mut first = profiles[0].clone();
        first.invert();
        shell.push(first);
        shell.push(profiles.last().unwrap().clone());
        let solid = Solid::try_new(vec![shell]).map_err(|e| error(format!("Loft closure: {e}")))?;
        let mut candidate = self.clone();
        let id = candidate.cad_insert(
            solid,
            "Section loft",
            &format!("Ruled loft · {} sections", sections.len()),
        )?;
        candidate.cad_bodies.get_mut(&id).unwrap().dependencies = Some(CadDependencies {
            bodies: Vec::new(),
            profiles: sections.to_vec(),
            replayable_pad: false,
        });
        candidate.cad_bodies.get_mut(&id).unwrap().recipe = Some(CadRecipe::Loft {
            sections: sections.to_vec(),
        });
        candidate.name_constructed_faces(id)?;
        candidate.validate()?;
        *self = candidate;
        Ok(id)
    }

    /// Oblique translational sweep. A tangential vector cannot enclose volume.
    pub fn cad_sweep_vector(
        &mut self,
        face: FaceId,
        displacement: Vec3,
    ) -> Result<BodyId, GeometryError> {
        finite(displacement)?;
        let normal = self.planar_face_frame(face)?.normal;
        if displacement.dot(normal).abs() <= 1e-6 {
            return Err(error("Sweep vector must leave the sketch plane"));
        }
        let solid = builder::tsweep(&self.cad_profile(face)?, vector(displacement));
        let mut candidate = self.clone();
        let id = candidate.cad_insert(
            solid,
            "Vector sweep",
            &format!("Vector sweep · {displacement:?}"),
        )?;
        candidate.cad_bodies.get_mut(&id).unwrap().dependencies = Some(CadDependencies {
            bodies: Vec::new(),
            profiles: vec![face],
            replayable_pad: false,
        });
        candidate.cad_bodies.get_mut(&id).unwrap().recipe =
            Some(CadRecipe::Sweep { face, displacement });
        candidate.name_constructed_faces(id)?;
        candidate.validate()?;
        *self = candidate;
        Ok(id)
    }

    /// Ruled tapered extrusion, scaling the top profile about a point in its plane.
    pub fn cad_tapered_extrude(
        &mut self,
        face: FaceId,
        distance: f64,
        factor: f64,
    ) -> Result<BodyId, GeometryError> {
        primitives::positive(distance.abs(), "Taper depth")?;
        primitives::positive(factor, "Top profile scale")?;
        let frame = self.planar_face_frame(face)?;
        let mut bottom = self.cad_profile(face)?;
        let top = builder::translated(
            &builder::scaled(
                &bottom,
                point(frame.origin),
                Vector3::new(factor, factor, factor),
            ),
            vector(frame.normal * distance),
        );
        let mut shell = truck_modeling::Shell::new();
        for (a, b) in bottom.boundaries().iter().zip(top.boundaries()) {
            let mut sides = builder::try_wire_homotopy(a, &b)
                .map_err(|e| error(format!("Taper correspondence: {e}")))?;
            shell.append(&mut sides);
        }
        bottom.invert();
        shell.push(bottom);
        shell.push(top);
        let solid =
            Solid::try_new(vec![shell]).map_err(|e| error(format!("Taper closure: {e}")))?;
        let mut candidate = self.clone();
        let id = candidate.cad_insert(
            solid,
            "Tapered extrusion",
            &format!("Taper · depth {distance} · top scale {factor}"),
        )?;
        candidate.cad_bodies.get_mut(&id).unwrap().dependencies = Some(CadDependencies {
            bodies: Vec::new(),
            profiles: vec![face],
            replayable_pad: false,
        });
        candidate.cad_bodies.get_mut(&id).unwrap().recipe = Some(CadRecipe::Taper {
            face,
            distance,
            factor,
        });
        candidate.name_constructed_faces(id)?;
        candidate.validate()?;
        *self = candidate;
        Ok(id)
    }

    /// Pattern around an arbitrary global axis; source is included in count.
    pub fn cad_radial_pattern(
        &mut self,
        id: BodyId,
        count: usize,
        pivot: Vec3,
        axis: Vec3,
        step_degrees: f64,
    ) -> Result<Vec<BodyId>, GeometryError> {
        finite(pivot)?;
        finite(axis)?;
        if !(2..=64).contains(&count)
            || axis.norm() < 1e-12
            || !step_degrees.is_finite()
            || step_degrees.abs() < 1e-9
        {
            return Err(error(
                "Radial pattern requires 2..64 instances, a nonzero axis and angular step",
            ));
        }
        // Avoid duplicate instances such as a 360-degree step.
        for i in 1..count {
            if (step_degrees * i as f64)
                .rem_euclid(360.0)
                .min(360.0 - (step_degrees * i as f64).rem_euclid(360.0))
                < 1e-8
            {
                return Err(error(
                    "Angular step produces overlapping duplicate instances",
                ));
            }
        }
        let mut candidate = self.clone();
        let mut ids = vec![id];
        for i in 1..count {
            ids.push(candidate.cad_transform(
                id,
                CadTransform::Rotate {
                    pivot,
                    axis,
                    degrees: step_degrees * i as f64,
                },
                true,
            )?);
        }
        candidate.validate()?;
        *self = candidate;
        Ok(ids)
    }

    /// Wavefront OBJ of the cached display tessellation, grouped by stable face.
    /// Coordinates use model units; this is an approximate polygon export.
    pub fn cad_export_obj(&self, id: BodyId) -> Result<String, GeometryError> {
        use std::fmt::Write;
        let body = self
            .cad_body(id)
            .ok_or_else(|| error("Select a CAD body"))?;
        let mut output = String::from("# FLURSYS display tessellation; model units\n");
        let mut index = 1usize;
        for mesh in body.face_meshes.iter() {
            writeln!(output, "g face_{}", mesh.face.get()).unwrap();
            for triangle in &mesh.triangles {
                for p in triangle {
                    writeln!(output, "v {} {} {}", p.x, p.y, p.z).unwrap();
                }
                writeln!(output, "f {} {} {}", index, index + 1, index + 2).unwrap();
                index += 3;
            }
        }
        Ok(output)
    }
}
