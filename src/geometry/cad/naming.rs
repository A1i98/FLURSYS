//! Persistent construction roles and conservative Boolean face provenance.
//! Ambiguous or split source surfaces deliberately receive no semantic name.
use super::*;

impl GeometryTopology {
    pub(super) fn name_constructed_faces(&mut self, id: BodyId) -> Result<(), GeometryError> {
        let recipe = self.cad_bodies[&id].recipe.clone();
        let count = self.cad_bodies[&id].face_meshes.len();
        let profile_edges = |face: FaceId| -> Option<Vec<EdgeId>> {
            match &self.face(face)?.representation {
                GeometryFaceRepresentation::Planar {
                    outer_loop,
                    inner_loops,
                } => Some(
                    outer_loop
                        .iter()
                        .chain(inner_loops.iter().flatten())
                        .map(|e| e.edge)
                        .collect(),
                ),
                _ => None,
            }
        };
        let names: Option<Vec<String>> = match recipe {
            Some(CadRecipe::Finish{kind,corners,opening,..}) => {
                let names=if kind==2 {
                    (0..count).map(|i|format!("shell:opening:{opening}:face:{i}")).collect::<Vec<_>>()
                }else{
                    let mut names=vec![format!("finish:{kind}:start")];
                    for corner in 0..4 {
                        if corners & (1<<corner)!=0 {names.push(format!("finish:{kind}:corner:{corner}"));}
                        names.push(format!("finish:{kind}:side:{corner}"));
                    }
                    names.push(format!("finish:{kind}:end"));names
                };
                (names.len()==count).then_some(names)
            },
            Some(CadRecipe::Primitive { shape, .. }) => {
                let kind = match shape {
                    CadPrimitive::Box { .. } => "box",
                    CadPrimitive::Cylinder { .. } => "cylinder",
                    CadPrimitive::Sphere { .. } => "sphere",
                    CadPrimitive::Cone { .. } => "cone",
                    CadPrimitive::Torus { .. } => "torus",
                };
                Some(
                    (0..count)
                        .map(|i| format!("primitive:{kind}:face:{i}"))
                        .collect(),
                )
            }
            Some(CadRecipe::Extrude { face, .. }) | Some(CadRecipe::Sweep { face, .. }) => {
                profile_edges(face).map(|edges| {
                    std::iter::once(format!("profile:{face:?}:start"))
                        .chain(
                            edges
                                .into_iter()
                                .map(|edge| format!("profile:{face:?}:side:{edge:?}")),
                        )
                        .chain(std::iter::once(format!("profile:{face:?}:end")))
                        .collect()
                })
            }
            Some(CadRecipe::Taper { face, .. }) => profile_edges(face).map(|edges| {
                edges
                    .into_iter()
                    .map(|edge| format!("profile:{face:?}:side:{edge:?}"))
                    .chain([
                        format!("profile:{face:?}:start"),
                        format!("profile:{face:?}:end"),
                    ])
                    .collect()
            }),
            Some(CadRecipe::Loft { sections }) => {
                let mut names = Vec::new();
                for pair in sections.windows(2) {
                    if let Some(edges) = profile_edges(pair[0]) {
                        names.extend(
                            edges
                                .into_iter()
                                .map(|e| format!("loft:{:?}:{:?}:side:{e:?}", pair[0], pair[1])),
                        );
                    }
                }
                if let (Some(first), Some(last)) = (sections.first(), sections.last()) {
                    names.extend([
                        format!("loft:start:{first:?}"),
                        format!("loft:end:{last:?}"),
                    ]);
                }
                Some(names)
            }
            Some(CadRecipe::Transform { source, .. }) => Some(
                self.cad_bodies[&source]
                    .face_meshes
                    .iter()
                    .map(|m| format!("copy:{source:?}:face:{:?}", m.face))
                    .collect(),
            ),
            Some(CadRecipe::Boolean { left, right, .. }) => {
                let mut sources = Vec::new();
                for body in [left, right] {
                    let solid = self.cad_solid(body)?;
                    for (face, mesh) in solid
                        .face_iter()
                        .zip(self.cad_bodies[&body].face_meshes.iter())
                    {
                        sources.push((
                            serde_json::to_value(face.surface())
                                .map_err(|e| error(e.to_string()))?,
                            format!("source:{body:?}:face:{:?}", mesh.face),
                        ));
                    }
                }
                let solid = self.cad_solid(id)?;
                let mut labels = Vec::new();
                for face in solid.face_iter() {
                    let surface =
                        serde_json::to_value(face.surface()).map_err(|e| error(e.to_string()))?;
                    let matches = sources
                        .iter()
                        .filter(|(s, _)| *s == surface)
                        .collect::<Vec<_>>();
                    labels.push(if matches.len() == 1 {
                        Some(matches[0].1.clone())
                    } else {
                        None
                    });
                }
                // A cut can split one input face into multiple patches. The
                // shared surface alone cannot tell those patches apart.
                let duplicates = labels
                    .iter()
                    .flatten()
                    .filter(|label| {
                        labels
                            .iter()
                            .flatten()
                            .filter(|other| *other == *label)
                            .count()
                            > 1
                    })
                    .cloned()
                    .collect::<BTreeSet<_>>();
                let data = self.cad_bodies.get_mut(&id).unwrap();
                for (mesh, label) in Arc::make_mut(&mut data.face_meshes).iter_mut().zip(labels) {
                    mesh.semantic_name = label.filter(|name| !duplicates.contains(name));
                }
                return Ok(());
            }
            _ => None,
        };
        if let Some(names) = names.filter(|n| n.len() == count) {
            let data = self.cad_bodies.get_mut(&id).unwrap();
            for (mesh, name) in Arc::make_mut(&mut data.face_meshes).iter_mut().zip(names) {
                mesh.semantic_name = Some(name);
            }
        }
        Ok(())
    }
}
