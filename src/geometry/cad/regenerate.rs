//! Dependency-ordered replay on the caller's private candidate document.
//! Stable face IDs are retained only when the topology is compatible.
use super::*;

impl GeometryTopology {
    fn replay_recipe(&mut self, recipe: &CadRecipe) -> Result<BodyId, GeometryError> {
        match recipe {
            CadRecipe::Finish {
                source,
                kind,
                amount,
                corners,
                opening,
            } => self.cad_finish_prism(*source, *kind, *amount, *corners, *opening),
            CadRecipe::Primitive { shape, position } => {
                self.cad_add_primitive(*shape, *position, "Primitive")
            }
            CadRecipe::Extrude { face, distance } => self.cad_extrude(*face, *distance),
            CadRecipe::Revolve {
                face,
                pivot,
                axis,
                degrees,
                axis_reference,
                axis_reversed,
            } => {
                if let Some(reference) = axis_reference {
                    self.cad_revolve_with_reference(*face, *reference, *degrees, *axis_reversed)
                } else {
                    self.cad_revolve(*face, *pivot, *axis, *degrees)
                }
            }
            CadRecipe::Loft { sections } => self.cad_loft_sections(sections),
            CadRecipe::Sweep { face, displacement } => self.cad_sweep_vector(*face, *displacement),
            CadRecipe::Taper {
                face,
                distance,
                factor,
            } => self.cad_tapered_extrude(*face, *distance, *factor),
            CadRecipe::Boolean {
                left,
                right,
                operation,
                tolerance,
            } => self.cad_boolean(*left, *right, *operation, *tolerance),
            CadRecipe::Transform { source, transform } => {
                self.cad_transform(*source, *transform, true)
            }
            CadRecipe::Modified { source, transform } => {
                let temporary = self.replay_recipe(source)?;
                self.cad_transform(temporary, *transform, false)
            }
            CadRecipe::PocketTool {
                face,
                depth,
                through_all,
                stop_face,
                tolerance,
            } => {
                let (solid, _, _, _) =
                    self.pocket_tool_solid(*face, *depth, *through_all, *stop_face, *tolerance)?;
                let id = self.cad_insert(solid, "Pocket tool", "Regenerated subtractive tool")?;
                for (i, mesh) in
                    Arc::make_mut(&mut self.cad_bodies.get_mut(&id).unwrap().face_meshes)
                        .iter_mut()
                        .enumerate()
                {
                    mesh.semantic_name = Some(format!("pocket-tool:{face:?}:face:{i}"));
                }
                Ok(id)
            }
        }
    }

    pub(super) fn regenerate_features(
        &mut self,
        sketch: CadSketchId,
    ) -> Result<Vec<BodyId>, GeometryError> {
        let dirty_profiles = self
            .sketch(sketch)
            .and_then(|s| s.profile_face)
            .into_iter()
            .collect();
        self.regenerate_from(dirty_profiles, BTreeSet::new())
    }

    /// Change the parameters of a saved operation and rebuild its descendants
    /// atomically. Reparenting is deliberately excluded from this editor.
    pub fn cad_edit_body_recipe(
        &mut self,
        body: BodyId,
        recipe: CadRecipe,
    ) -> Result<Vec<BodyId>, GeometryError> {
        let old = self
            .cad_body(body)
            .and_then(|b| b.recipe.as_ref())
            .ok_or_else(|| error("Body has no saved recipe"))?;
        let compatible = match (old, &recipe) {
            (
                CadRecipe::Finish {
                    source: a, kind: b, ..
                },
                CadRecipe::Finish {
                    source: c, kind: d, ..
                },
            ) => a == c && b == d,
            (CadRecipe::Primitive { shape: a, .. }, CadRecipe::Primitive { shape: b, .. }) => {
                std::mem::discriminant(a) == std::mem::discriminant(b)
            }
            (CadRecipe::Revolve { face: a, .. }, CadRecipe::Revolve { face: b, .. })
            | (CadRecipe::Sweep { face: a, .. }, CadRecipe::Sweep { face: b, .. })
            | (CadRecipe::Taper { face: a, .. }, CadRecipe::Taper { face: b, .. }) => a == b,
            (CadRecipe::Loft { .. }, CadRecipe::Loft { sections }) => sections.len() >= 2,
            (CadRecipe::Transform { source: a, .. }, CadRecipe::Transform { source: b, .. }) => {
                a == b
            }
            (CadRecipe::Modified { source: a, .. }, CadRecipe::Modified { source: b, .. }) => {
                a == b
            }
            (
                CadRecipe::Boolean {
                    left: a,
                    right: b,
                    operation: c,
                    ..
                },
                CadRecipe::Boolean {
                    left: d,
                    right: e,
                    operation: f,
                    ..
                },
            ) => a == d && b == e && c == f,
            _ => false,
        };
        if !compatible {
            return Err(error(
                "Use the typed feature editor or retain this operation's original inputs",
            ));
        }
        if !matches!(&recipe, CadRecipe::Modified { .. })
            && (self.extrude_features.values().any(|f| f.body == body)
                || self
                    .pocket_features
                    .values()
                    .any(|f| f.body == body || f.tool == body))
        {
            return Err(error("Use the Pad or Pocket editor for this feature"));
        }
        let mut next = self.clone();
        if let CadRecipe::Revolve {
            face,
            axis_reference,
            ..
        } = &recipe
        {
            let mut dependencies = CadDependencies {
                bodies: Vec::new(),
                profiles: vec![*face],
                replayable_pad: false,
            };
            if let Some(reference) = axis_reference {
                next.add_axis_dependencies(&mut dependencies, *reference)?;
            }
            next.cad_bodies.get_mut(&body).unwrap().dependencies = Some(dependencies);
        }
        if let CadRecipe::Loft { sections } = &recipe {
            if sections.len() < 2 {
                return Err(error("A Loft requires at least two ordered profiles"));
            }
            let dependencies = next
                .cad_bodies
                .get_mut(&body)
                .unwrap()
                .dependencies
                .get_or_insert(CadDependencies {
                    bodies: Vec::new(),
                    profiles: Vec::new(),
                    replayable_pad: false,
                });
            dependencies.profiles = sections.clone();
        }
        next.cad_bodies.get_mut(&body).unwrap().recipe = Some(recipe);
        let rebuilt = next.regenerate_from(BTreeSet::new(), [body].into_iter().collect())?;
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(rebuilt)
    }

    pub(super) fn regenerate_from(
        &mut self,
        mut dirty_profiles: BTreeSet<FaceId>,
        mut affected: BTreeSet<BodyId>,
    ) -> Result<Vec<BodyId>, GeometryError> {
        if self.cad_bodies.values().any(|b| b.dependencies.is_none()) {
            return Err(error("Legacy geometry has no dependency metadata; rebuild its features before parametric editing"));
        }
        loop {
            let before = (dirty_profiles.len(), affected.len());
            for (id, body) in &self.cad_bodies {
                let d = body.dependencies.as_ref().unwrap();
                if d.profiles.iter().any(|f| dirty_profiles.contains(f))
                    || d.bodies.iter().any(|b| affected.contains(b))
                {
                    affected.insert(*id);
                }
            }
            for s in self.sketches.values() {
                if s.host_face
                    .and_then(|f| self.cad_owner(f))
                    .is_some_and(|b| affected.contains(&b))
                {
                    if let Some(f) = s.profile_face {
                        dirty_profiles.insert(f);
                    }
                }
            }
            if before == (dirty_profiles.len(), affected.len()) {
                break;
            }
        }
        // Pocket already had a typed feature record before generic recipes.
        // Recover intent from that record, never by parsing its display label.
        for feature in self.pocket_features.values().cloned().collect::<Vec<_>>() {
            if affected.contains(&feature.tool) && self.cad_bodies[&feature.tool].recipe.is_none() {
                self.cad_bodies.get_mut(&feature.tool).unwrap().recipe =
                    Some(CadRecipe::PocketTool {
                        face: feature.source_face,
                        depth: feature.depth,
                        through_all: feature.through_all,
                        stop_face: feature.stop_face,
                        tolerance: feature.tolerance,
                    });
            }
            if affected.contains(&feature.body) && self.cad_bodies[&feature.body].recipe.is_none() {
                self.cad_bodies.get_mut(&feature.body).unwrap().recipe = Some(CadRecipe::Boolean {
                    left: feature.base,
                    right: feature.tool,
                    operation: CadBoolean::Difference,
                    tolerance: feature.tolerance,
                });
            }
        }
        // Migrate old ordinary Pads from their existing typed feature record.
        for id in &affected {
            if self.cad_bodies[id].recipe.is_none() {
                let feature = self
                    .extrude_features
                    .values()
                    .find(|f| f.body == *id && !f.additive);
                let recipe = feature.filter(|_|self.cad_bodies[id].dependencies.as_ref().unwrap().replayable_pad)
                    .and_then(|f|f.distance.map(|distance|CadRecipe::Extrude {face:f.source_face,distance}))
                    .ok_or_else(||error(format!("Body {} has no saved construction recipe; recreate this feature before editing",id.get())))?;
                self.cad_bodies.get_mut(id).unwrap().recipe = Some(recipe);
            }
        }
        let visibility = self
            .cad_bodies
            .iter()
            .map(|(id, b)| (*id, b.visible))
            .collect::<BTreeMap<_, _>>();
        let mut pending = affected;
        let mut rebuilt = Vec::new();
        while !pending.is_empty() {
            let ready = pending
                .iter()
                .copied()
                .find(|id| {
                    let d = self.cad_bodies[id].dependencies.as_ref().unwrap();
                    !d.bodies.iter().any(|b| pending.contains(b))
                        && !d.profiles.iter().any(|face| {
                            self.sketches
                                .values()
                                .find(|s| s.profile_face == Some(*face))
                                .and_then(|s| s.host_face)
                                .and_then(|host| self.cad_owner(host))
                                .is_some_and(|b| pending.contains(&b))
                        })
                })
                .ok_or_else(|| error("Feature dependencies contain a cycle"))?;
            let old = self.cad_bodies[&ready].clone();
            let recipe = old.recipe.as_ref().unwrap();
            let temporary = self.replay_recipe(recipe)?;
            self.adopt_regenerated_body(ready, temporary, &old)?;
            pending.remove(&ready);
            rebuilt.push(ready);
        }
        for (id, visible) in visibility {
            self.cad_bodies.get_mut(&id).unwrap().visible = visible;
        }
        Ok(rebuilt)
    }

    fn adopt_regenerated_body(
        &mut self,
        id: BodyId,
        temporary: BodyId,
        old: &CadBody,
    ) -> Result<(), GeometryError> {
        let old_faces = self.bodies[&id].faces.clone();
        let mut replacement = self
            .cad_bodies
            .remove(&temporary)
            .ok_or_else(|| error("Missing replay result"))?;
        let temporary_body = self.bodies.remove(&temporary).unwrap();
        if old_faces.len() != replacement.face_meshes.len() {
            return Err(error(
                "Regeneration changed face topology; automatic face mapping is not safe",
            ));
        }
        let attached = self
            .sketches
            .values()
            .filter(|s| s.host_face.is_some_and(|f| old_faces.contains(&f)))
            .cloned()
            .collect::<Vec<_>>();
        let old_names = old
            .face_meshes
            .iter()
            .filter_map(|m| m.semantic_name.as_ref().map(|n| (n.clone(), m.face)))
            .collect::<BTreeMap<_, _>>();
        let semantic_mapping = old_names.len() == old_faces.len()
            && replacement.face_meshes.iter().all(|m| {
                m.semantic_name
                    .as_ref()
                    .is_some_and(|n| old_names.contains_key(n))
            })
            && replacement
                .face_meshes
                .iter()
                .filter_map(|m| m.semantic_name.as_ref())
                .collect::<BTreeSet<_>>()
                .len()
                == old_faces.len();
        let boolean = old.recipe.as_ref().is_some_and(requires_provenance);
        if boolean && !semantic_mapping {
            return Err(error("Boolean face provenance is ambiguous or has split/merged. Automatic reference reassignment was rejected"));
        }
        if !attached.is_empty()
            && !semantic_mapping
            && !attachment_safe(old.recipe.as_ref().unwrap())
        {
            return Err(error("Attached sketches require unique persistent face names; recreate the feature before rebuilding"));
        }
        let ids = if semantic_mapping {
            replacement
                .face_meshes
                .iter()
                .map(|m| old_names[m.semantic_name.as_ref().unwrap()])
                .collect::<Vec<_>>()
        } else {
            old_faces.clone()
        };
        for (mesh, face) in replacement.face_meshes.iter().zip(&ids) {
            let previous = old.face_meshes.iter().find(|m| m.face == *face).unwrap();
            if previous.plane.is_some() != mesh.plane.is_some()
                || matches!((previous.plane,mesh.plane),(Some(a),Some(b)) if a.normal.dot(b.normal)<0.999999)
            {
                return Err(error("Regeneration changed the referenced face orientation; attachment mapping needs review"));
            }
        }
        replacement.face_meshes = Arc::new(
            replacement
                .face_meshes
                .iter()
                .cloned()
                .zip(&ids)
                .map(|(mut m, face)| {
                    m.face = *face;
                    m
                })
                .collect(),
        );
        self.bodies.get_mut(&id).unwrap().faces = ids;
        self.discard_subelements(&replacement);
        replacement.edges = old.edges.clone();
        replacement.vertices = old.vertices.clone();
        replacement.name = old.name.clone();
        replacement.visible = old.visible;
        replacement.dependencies = old.dependencies.clone();
        replacement.recipe = old.recipe.clone();
        for face in temporary_body.faces {
            self.faces.remove(&face);
        }
        self.cad_bodies.insert(id, replacement);
        self.refresh_subelements(id)?;
        let mut moved = BTreeSet::new();
        for s in attached {
            let mut frame = self.sketch_frame_for_face(s.host_face.unwrap())?;
            frame.origin = frame.origin + frame.normal * s.attachment_offset;
            finite(frame.origin)?;
            if let Some(face) = s.profile_face {
                for vertex in self.profile_vertices(face)? {
                    if !moved.insert(vertex) {
                        return Err(error("Attached sketches share vertices; detach shared geometry before rebuilding"));
                    }
                    let d = self.vertices[&vertex].position - s.frame.origin;
                    self.vertices.get_mut(&vertex).unwrap().position = frame.origin
                        + frame.x_axis * d.dot(s.frame.x_axis)
                        + frame.y_axis * d.dot(s.frame.y_axis)
                        + frame.normal * d.dot(s.frame.normal);
                }
            }
            self.sketches.get_mut(&s.id).unwrap().frame = frame;
        }
        Ok(())
    }
}
fn attachment_safe(recipe: &CadRecipe) -> bool {
    match recipe {
        CadRecipe::Extrude { .. } | CadRecipe::Sweep { .. } | CadRecipe::Taper { .. } => true,
        CadRecipe::Modified { source, .. } => attachment_safe(source),
        _ => false,
    }
}

fn requires_provenance(recipe: &CadRecipe) -> bool {
    match recipe {
        CadRecipe::Boolean { .. } => true,
        CadRecipe::Modified { source, .. } => requires_provenance(source),
        _ => false,
    }
}
