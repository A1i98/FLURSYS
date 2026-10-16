//! Safe terminal-feature deletion. Descendants are checked by topology guards;
//! deleting a parent never leaves attached sketches or Boolean inputs dangling.
use super::*;

impl GeometryTopology {
    pub fn cad_delete_sketch(&mut self, id: CadSketchId) -> Result<(), GeometryError> {
        if self.sketches().any(|s| s.region_owner == Some(id)) {
            return Err(error(
                "Delete the additional regions before deleting their owning sketch",
            ));
        }
        if self.extrude_features.values().any(|f| f.sketch == id)
            || self.pocket_features.values().any(|f| f.sketch == id)
        {
            return Err(error(
                "Sketch is used by a feature. Delete the dependent feature first",
            ));
        }
        let mut next = self.clone();
        let sketch = next
            .sketches
            .remove(&id)
            .ok_or_else(|| error("Sketch not found"))?;
        if let Some(face) = sketch.profile_face {
            let edges = match &next.require_face(face)?.representation {
                GeometryFaceRepresentation::Planar {
                    outer_loop,
                    inner_loops,
                } => outer_loop
                    .iter()
                    .chain(inner_loops.iter().flatten())
                    .map(|e| e.edge)
                    .collect::<BTreeSet<_>>(),
                _ => return Err(error("Sketch profile is not planar")),
            };
            let vertices = edges
                .iter()
                .flat_map(|id| match next.edges[id].geometry {
                    EdgeGeometry::Line { start, end } | EdgeGeometry::Kernel { start, end, .. } => {
                        vec![start, end]
                    }
                    EdgeGeometry::CircularArc { start, center, end } => vec![start, center, end],
                })
                .collect::<BTreeSet<_>>();
            next.remove_face(face)?;
            for edge in edges {
                match next.remove_edge(edge) {
                    Ok(_) | Err(GeometryError::EntityInUse { .. }) => {}
                    Err(e) => return Err(e),
                }
            }
            for vertex in vertices {
                match next.remove_vertex(vertex) {
                    Ok(_) | Err(GeometryError::EntityInUse { .. }) => {}
                    Err(e) => return Err(e),
                }
            }
        }
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(())
    }

    pub fn cad_delete_feature_body(&mut self, body: BodyId) -> Result<(), GeometryError> {
        if self.cad_body(body).is_none() {
            return Err(error("Feature deletion requires a kernel body"));
        }
        let mut next = self.clone();
        let profile;
        if let Some(feature) = next
            .extrude_features
            .values()
            .find(|f| f.body == body)
            .cloned()
        {
            if feature.additive {
                return Err(error("Delete the additive feature through Undo until its full removal recipe is supported"));
            }
            next.extrude_features.remove(&feature.id);
            next.remove_body(body)?;
            profile = feature.source_face;
        } else if let Some(feature) = next
            .pocket_features
            .values()
            .find(|f| f.body == body)
            .cloned()
        {
            next.pocket_features.remove(&feature.id);
            next.remove_body(body)?;
            next.remove_body(feature.tool)?;
            if !next.cad_bodies().any(|(_, b)| {
                b.dependencies
                    .as_ref()
                    .is_some_and(|d| d.bodies.contains(&feature.base))
            }) {
                next.cad_set_visible(feature.base, true)?;
            }
            profile = feature.source_face;
        } else {
            let finishing_source = match next.cad_body(body).and_then(|b| b.recipe.as_ref()) {
                Some(CadRecipe::Finish { source, .. }) => Some(*source),
                _ => None,
            };
            next.remove_body(body)?;
            if let Some(source) = finishing_source {
                if !next.cad_bodies().any(|(_, b)| {
                    b.dependencies
                        .as_ref()
                        .is_some_and(|d| d.bodies.contains(&source))
                }) {
                    next.cad_set_visible(source, true)?;
                }
            }
            next.validate()?;
            *self = next;
            return Ok(());
        }
        if !next.cad_bodies().any(|(_, b)| {
            b.dependencies
                .as_ref()
                .is_some_and(|d| d.profiles.contains(&profile))
        }) {
            next.hidden_profiles.remove(&profile);
        }
        next.bump_revision();
        next.validate()?;
        *self = next;
        Ok(())
    }
}
