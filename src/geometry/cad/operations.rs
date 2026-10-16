use super::*;

impl GeometryTopology {
    pub fn cad_extrude(&mut self, face: FaceId, distance: f64) -> Result<BodyId, GeometryError> {
        primitives::positive(distance.abs(), "Extrude distance")?;
        let profile = self.cad_profile(face)?;
        let frame = self.planar_face_frame(face)?;
        let solid = builder::tsweep(&profile, vector(frame.normal * distance));
        let id = self.cad_insert(solid, "Extrude", &format!("Pad · depth {distance}"))?;
        self.cad_bodies.get_mut(&id).unwrap().dependencies = Some(CadDependencies {
            bodies: Vec::new(),
            profiles: vec![face],
            replayable_pad: true,
        });
        self.cad_bodies.get_mut(&id).unwrap().recipe = Some(CadRecipe::Extrude { face, distance });
        self.name_constructed_faces(id)?;
        Ok(id)
    }
    pub fn cad_revolve(
        &mut self,
        face: FaceId,
        pivot: Vec3,
        axis: Vec3,
        degrees: f64,
    ) -> Result<BodyId, GeometryError> {
        finite(pivot)?;
        finite(axis)?;
        if axis.norm() < 1e-12 || !degrees.is_finite() || degrees <= 0.0 || degrees > 360.0 {
            return Err(error(
                "Revolve requires a nonzero axis and angle in (0,360]",
            ));
        }
        let profile = self.cad_profile(face)?;
        let unit_axis = vector(axis).normalize();
        let wires = profile.boundaries();
        let axis_edge = if wires.len() == 1 {
            wires[0].iter().position(|edge| {
                let a = edge.front().point() - point(pivot);
                let b = edge.back().point() - point(pivot);
                a.cross(unit_axis).magnitude() < 1e-6 && b.cross(unit_axis).magnitude() < 1e-6
            })
        } else {
            None
        };
        let solid = if let Some(index) = axis_edge {
            if (degrees - 360.0).abs() > 1e-9 {
                return Err(error(
                    "A profile touching the axis currently requires a full 360° revolution",
                ));
            }
            let wire: Wire = wires[0]
                .iter()
                .cycle()
                .skip(index + 1)
                .take(wires[0].len() - 1)
                .cloned()
                .collect();
            Solid::try_new(vec![builder::cone(
                &wire,
                unit_axis,
                Rad(std::f64::consts::TAU),
            )])
            .map_err(|e| error(e.to_string()))?
        } else {
            builder::rsweep(&profile, point(pivot), unit_axis, Rad(degrees.to_radians()))
        };
        let id = self.cad_insert(solid, "Revolve", &format!("Revolve · {degrees}°"))?;
        self.cad_bodies.get_mut(&id).unwrap().dependencies = Some(CadDependencies {
            bodies: Vec::new(),
            profiles: vec![face],
            replayable_pad: false,
        });
        self.cad_bodies.get_mut(&id).unwrap().recipe = Some(CadRecipe::Revolve {
            face,
            pivot,
            axis,
            degrees,
            axis_reference: None,
            axis_reversed: false,
        });
        self.name_constructed_faces(id)?;
        Ok(id)
    }
    /// Ruled loft between two closed planar profiles with matching edge counts.
    pub fn cad_loft(&mut self, first: FaceId, second: FaceId) -> Result<BodyId, GeometryError> {
        self.cad_loft_sections(&[first, second])
    }
    pub fn cad_boolean(
        &mut self,
        a: BodyId,
        b: BodyId,
        operation: CadBoolean,
        tolerance: f64,
    ) -> Result<BodyId, GeometryError> {
        if a == b {
            return Err(error("Select two different CAD bodies"));
        }
        if !tolerance.is_finite() || tolerance <= truck_modeling::TOLERANCE || tolerance > 1.0 {
            return Err(error(
                "Boolean tolerance must be greater than 1e-6 and at most 1 model unit",
            ));
        }
        let left = self.cad_solid(a)?;
        let mut right = self.cad_solid(b)?;
        if operation == CadBoolean::Difference {
            right.not();
        }
        let result = match operation {
            CadBoolean::Union => truck_shapeops::or(&left, &right, tolerance),
            _ => truck_shapeops::and(&left, &right, tolerance),
        }
        .ok_or_else(|| {
            error("Boolean intersection failed; check contact, overlapping surfaces and tolerance")
        })?;
        let id = self.cad_insert(
            result,
            &format!("{operation:?}"),
            &format!("{operation:?} Body {} / {}", a.get(), b.get()),
        )?;
        self.cad_bodies
            .get_mut(&id)
            .unwrap()
            .dependencies
            .as_mut()
            .unwrap()
            .bodies = vec![a, b];
        self.cad_bodies.get_mut(&id).unwrap().recipe = Some(CadRecipe::Boolean {
            left: a,
            right: b,
            operation,
            tolerance,
        });
        self.cad_bodies.get_mut(&a).unwrap().visible = false;
        self.cad_bodies.get_mut(&b).unwrap().visible = false;
        self.name_constructed_faces(id)?;
        Ok(id)
    }
    pub fn cad_transform(
        &mut self,
        id: BodyId,
        transform: CadTransform,
        copy: bool,
    ) -> Result<BodyId, GeometryError> {
        if !copy
            && self.cad_bodies.iter().any(|(other, body)| {
                *other != id
                    && body
                        .dependencies
                        .as_ref()
                        .is_some_and(|d| d.bodies.contains(&id))
            })
        {
            return Err(error("This body has dependent features; transform a copy or edit the source sketch so the feature graph can rebuild"));
        }
        if !copy
            && self.sketches.values().any(|s| {
                s.host_face.is_some_and(|face| {
                    self.bodies
                        .get(&id)
                        .is_some_and(|body| body.faces.contains(&face))
                })
            })
        {
            return Err(error(
                "Body has face-hosted sketches; moving dependent sketches is not supported yet",
            ));
        }
        let original = self.cad_solid(id)?;
        let solid = apply_transform(&original, transform)?;
        let name = self.cad_bodies[&id].name.clone();
        if copy {
            let copy_id =
                self.cad_insert(solid, &format!("{name} copy"), &format!("{transform:?}"))?;
            self.cad_bodies
                .get_mut(&copy_id)
                .unwrap()
                .dependencies
                .as_mut()
                .unwrap()
                .bodies = vec![id];
            self.cad_bodies.get_mut(&copy_id).unwrap().recipe = Some(CadRecipe::Transform {
                source: id,
                transform,
            });
            self.name_constructed_faces(copy_id)?;
            return Ok(copy_id);
        }
        let data = prepare(&solid)?;
        let ids = self.bodies[&id].faces.clone();
        if ids.len() != data.1.len() {
            return Err(error("Rigid transform unexpectedly changed face count"));
        }
        let body = self.cad_bodies.get_mut(&id).unwrap();
        body.brep = Arc::new(data.0);
        let planes = solid.face_iter().map(kernel_plane).collect::<Vec<_>>();
        body.face_meshes = Arc::new(
            ids.into_iter()
                .zip(data.1)
                .zip(planes)
                .map(|((face, triangles), plane)| CadFaceMesh {
                    semantic_name: body
                        .face_meshes
                        .iter()
                        .find(|m| m.face == face)
                        .and_then(|m| m.semantic_name.clone()),
                    face,
                    triangles,
                    plane,
                })
                .collect(),
        );
        body.operation = format!("{transform:?}");
        body.recipe = body.recipe.take().map(|source| CadRecipe::Modified {
            source: Box::new(source),
            transform,
        });
        if let Some(dependencies) = body.dependencies.as_mut() {
            dependencies.replayable_pad = false;
        }
        self.bump_revision();
        self.name_constructed_faces(id)?;
        self.refresh_subelements(id)?;
        Ok(id)
    }
    pub fn cad_pattern(
        &mut self,
        id: BodyId,
        count: usize,
        step: Vec3,
        angle_degrees: f64,
    ) -> Result<Vec<BodyId>, GeometryError> {
        finite(step)?;
        if !(2..=64).contains(&count) || !angle_degrees.is_finite() {
            return Err(error("Pattern requires 2..64 instances and a finite angle"));
        }
        if angle_degrees == 0.0 && step.norm() < 1e-8 {
            return Err(error("Linear pattern step must be nonzero"));
        }
        let original = self.cad_solid(id)?;
        let mut candidate = self.clone();
        let mut ids = vec![id];
        for i in 1..count {
            let transform = if angle_degrees == 0.0 {
                CadTransform::Translate(step * i as f64)
            } else {
                CadTransform::Rotate {
                    pivot: step,
                    axis: Vec3::new(0.0, 0.0, 1.0),
                    degrees: angle_degrees * i as f64,
                }
            };
            let instance = candidate.cad_insert(
                apply_transform(&original, transform)?,
                "Pattern instance",
                &format!("Pattern of Body {} instance {i}", id.get()),
            )?;
            candidate
                .cad_bodies
                .get_mut(&instance)
                .unwrap()
                .dependencies
                .as_mut()
                .unwrap()
                .bodies = vec![id];
            candidate.cad_bodies.get_mut(&instance).unwrap().recipe = Some(CadRecipe::Transform {
                source: id,
                transform,
            });
            candidate.name_constructed_faces(instance)?;
            ids.push(instance);
        }
        *self = candidate;
        Ok(ids)
    }
}
