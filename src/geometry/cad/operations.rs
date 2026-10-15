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
        Ok(id)
    }
    /// Ruled loft between two closed planar profiles with matching edge counts.
    pub fn cad_loft(&mut self, first: FaceId, second: FaceId) -> Result<BodyId, GeometryError> {
        if first == second {
            return Err(error("Loft requires two different profiles"));
        }
        let mut a = self.cad_profile(first)?;
        let b = self.cad_profile(second)?;
        let wa = a.boundaries();
        let wb = b.boundaries();
        if wa.len() != wb.len() {
            return Err(error(
                "Loft profiles must have the same number of boundary loops",
            ));
        }
        let mut shell = truck_modeling::Shell::new();
        for (a, b) in wa.iter().zip(&wb) {
            let mut sides = builder::try_wire_homotopy(a, b)
                .map_err(|e| error(format!("Loft edge correspondence: {e}")))?;
            shell.append(&mut sides);
        }
        a.invert();
        shell.push(a);
        shell.push(b);
        let solid = Solid::try_new(vec![shell])
            .map_err(|e| error(format!("Loft is not a closed solid: {e}")))?;
        let id = self.cad_insert(
            solid,
            "Loft",
            &format!("Ruled loft Face {} → {}", first.get(), second.get()),
        )?;
        self.cad_bodies.get_mut(&id).unwrap().dependencies = Some(CadDependencies {
            bodies: Vec::new(),
            profiles: vec![first, second],
            replayable_pad: false,
        });
        Ok(id)
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
        self.cad_bodies.get_mut(&a).unwrap().visible = false;
        self.cad_bodies.get_mut(&b).unwrap().visible = false;
        Ok(id)
    }
    pub fn cad_transform(
        &mut self,
        id: BodyId,
        transform: CadTransform,
        copy: bool,
    ) -> Result<BodyId, GeometryError> {
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
                    face,
                    triangles,
                    plane,
                })
                .collect(),
        );
        body.operation = format!("{transform:?}");
        if let Some(dependencies) = body.dependencies.as_mut() {
            dependencies.replayable_pad = false;
        }
        self.bump_revision();
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
            ids.push(instance);
        }
        *self = candidate;
        Ok(ids)
    }
}
