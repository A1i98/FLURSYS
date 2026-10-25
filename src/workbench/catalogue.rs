//! Versioned case files with embedded defaults and editable local overrides.
#[path = "builtin_cases.rs"]
mod builtin_cases;
use super::{examples::*, project::*};
use crate::MeshDimension;
pub fn normalize_project_name(input: &str) -> Result<String, String> {
    let name = input.trim();
    if name.is_empty() || name.chars().count() > 100 || name.chars().any(char::is_control) {
        return Err("Project name must contain 1–100 characters and no control characters.".into());
    }
    Ok(name.into())
}
pub fn rename_project(project: &mut WorkbenchProject, input: &str) -> Result<(), String> {
    let name = normalize_project_name(input)?;
    let cad = project
        .cad_document
        .as_ref()
        .map(|text| {
            let mut doc: serde_json::Value =
                serde_json::from_str(text).map_err(|e| e.to_string())?;
            doc["name"] = serde_json::Value::String(name.clone());
            serde_json::to_string(&doc).map_err(|e| e.to_string())
        })
        .transpose()?;
    project.name = name;
    project.cad_document = cad;
    Ok(())
}
pub fn example_slug(id: ExampleProjectId) -> &'static str {
    match id {
        ExampleProjectId::Blank => "blank-2d",
        ExampleProjectId::LidDrivenCavity2D => "lid-driven-cavity",
        ExampleProjectId::LaminarChannel2D => "laminar-channel",
        ExampleProjectId::CylinderFlow2D => "cylinder-flow",
        ExampleProjectId::SkewedMeshVerification2D => "skewed-mesh-verification",
        ExampleProjectId::Channel3D => "channel-3d",
    }
}
pub fn load_case_file(path: &std::path::Path) -> Result<CaseTemplateManifest, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut entry: CaseTemplateManifest =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    entry.project_template.runs.clear();
    entry.project_template.mesh_asset = None;
    entry
        .validate()
        .map_err(|e| format!("{}: {e}", path.display()))?;
    Ok(entry)
}
pub fn builtin_templates() -> Result<Vec<CaseTemplateManifest>, String> {
    let mut templates = builtin_cases::CASES
        .iter()
        .map(|text| serde_json::from_str::<CaseTemplateManifest>(text).map_err(|e| e.to_string()))
        .collect::<Result<Vec<_>, _>>()?;
    let mut roots =
        vec![std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/cases")];
    if let Ok(exe) = std::env::current_exe() {
        if let Some(parent) = exe.parent() {
            roots.push(parent.join("examples/cases"));
        }
    }
    if let Some(root) = std::env::var_os("FLURSYS_EXAMPLES_DIR") {
        roots.push(root.into());
    }
    let mut visited = std::collections::BTreeSet::new();
    for root in roots {
        let Ok(identity) = std::fs::canonicalize(&root) else {
            continue;
        };
        if !visited.insert(identity) {
            continue;
        }
        for entry in super::discover_templates(&root) {
            let mut entry = entry.map_err(|e| e.to_string())?;
            entry.project_template.runs.clear();
            entry.project_template.mesh_asset = None;
            if let Some(existing) = templates.iter_mut().find(|t| t.id == entry.id) {
                *existing = entry;
            } else {
                templates.push(entry);
            }
        }
    }
    for entry in &mut templates {
        if entry.project_template.cad_document.is_none() {
            entry.project_template.cad_document = cad_document(&entry.project_template)?;
        }
        #[cfg(feature = "gui")]
        seed_cad_geometry_hash(&mut entry.project_template)?;
        entry
            .validate()
            .map_err(|e| format!("Case {}: {e}", entry.id))?;
    }
    Ok(templates)
}

pub fn cad_document(project: &WorkbenchProject) -> Result<Option<String>, String> {
    use crate::{
        EdgeGeometry, GeometryBodyRepresentation as Body, GeometryFaceRepresentation as Face,
    };
    use serde_json::json;
    let geometry = &project.geometry;
    if geometry.vertices().next().is_none() && geometry.bodies().next().is_none() {
        return Ok(None);
    }
    let feature = if project.mesh.dimension == MeshDimension::ThreeD {
        let body = geometry.bodies().next().ok_or("Recipe volume missing")?;
        let Body::Box {
            length,
            width,
            height,
        } = body.representation
        else {
            return Err("Recipe CAD conversion requires an analytic box".into());
        };
        json!({"id":1,"name":"Fluid volume","suppressed":false,"type":"box","corner":[0,0,0],"length":(length*1000.0).to_string(),"width":(width*1000.0).to_string(),"height":(height*1000.0).to_string()})
    } else {
        let face = geometry
            .face(project.fluid_face.ok_or("Recipe fluid face missing")?)
            .ok_or("Recipe fluid face missing")?;
        let Face::Planar {
            outer_loop,
            inner_loops,
        } = &face.representation
        else {
            return Err("Recipe profile is not planar".into());
        };
        let edges = outer_loop
            .iter()
            .chain(inner_loops.iter().flatten())
            .map(|e| e.edge)
            .collect::<std::collections::BTreeSet<_>>();
        let mut ids = std::collections::BTreeSet::new();
        for id in &edges {
            match geometry.edge(*id).ok_or("Recipe edge missing")?.geometry {
                EdgeGeometry::Line { start, end } => {
                    ids.insert(start);
                    ids.insert(end);
                }
                EdgeGeometry::CircularArc { start, center, end } => {
                    ids.insert(start);
                    ids.insert(center);
                    ids.insert(end);
                }
                _ => return Err("Unsupported recipe sketch edge".into()),
            }
        }
        let indices = ids
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, i))
            .collect::<std::collections::BTreeMap<_, _>>();
        let points=ids.iter().map(|id| {
            let p=geometry.vertex(*id).ok_or("Recipe vertex missing")?.position;
            Ok(json!({"id":format!("p{}",id.get()),"pos":[p.x*1000.0,p.y*1000.0],"fixed":false}))
        }).collect::<Result<Vec<_>,String>>()?;
        let curves=edges.iter().map(|id| {
            let name=format!("edge{}",id.get());
            Ok(match geometry.edge(*id).ok_or("Recipe edge missing")?.geometry {
                EdgeGeometry::Line{start,end}=>json!({"id":name,"type":"line","a":indices[&start],"b":indices[&end],"construction":false}),
                EdgeGeometry::CircularArc{start,center,end}=>json!({"id":name,"type":"arc","c":indices[&center],"a":indices[&start],"b":indices[&end],"construction":false}),
                _=>return Err("Unsupported recipe curve".into()),
            })
        }).collect::<Result<Vec<_>,String>>()?;
        json!({"id":1,"name":"Fluid profile","suppressed":false,"type":"sketch","plane":{"type":"origin","name":"XY"},
            "sketch":{"points":points,"curves":curves,"constraints":[],"counters":{"p":ids.last().map_or(0,|id|id.get()),"l":edges.len(),"a":edges.len(),"k":0}}})
    };
    Ok(Some(json!({"format":"solidcraft/30","name":project.name,"units":"mm","params":[],"features":[feature],"next_id":2}).to_string()))
}
#[cfg(feature = "gui")]
pub fn seed_cad_geometry_hash(project: &mut WorkbenchProject) -> Result<(), String> {
    if project.cad_geometry_hash.is_some()
        || (project.geometry.vertices().next().is_none()
            && project.geometry.bodies().next().is_none())
    {
        return Ok(());
    }
    let Some(text) = project.cad_document.as_ref() else {
        return Ok(());
    };
    let doc = cad_engine::doc::Document::from_json(text).map_err(|e| e.to_string())?;
    let session = cad_engine::Session::new(doc);
    let model = cad_engine::doc::world_state(&session.doc, &session.model.state());
    if model.bodies.is_empty() && model.sketches.is_empty() {
        return Err("Recipe CAD document evaluated to empty geometry".into());
    }
    project.cad_geometry_hash = Some(model.host_geometry_hash().map_err(|e| e.to_string())?);
    Ok(())
}

#[cfg(test)]
mod tests {
    #[test]
    fn file_library_roundtrips_and_exports_portably() {
        let cases = super::builtin_templates().expect("load library");
        for id in [
            "couette-planar",
            "natural-convection-ra-1000",
            "square-cylinder-re100",
            "taylor-green-2d",
            "sudden-expansion-re40",
            "sod-shock-tube",
        ] {
            let entry = cases.iter().find(|e| e.id == id).expect("case file");
            let session = crate::workbench::WorkbenchSession::from_project(&entry.project_template)
                .expect("canonical project");
            let project = session.to_project("roundtrip");
            assert_eq!(
                project.solver.initial_profile,
                entry.project_template.solver.initial_profile
            );
            assert_eq!(
                project.solver.compressible,
                entry.project_template.solver.compressible
            );
        }
        let directory = tempfile::tempdir().expect("directory");
        let path =
            crate::workbench::save_case_template(directory.path(), &cases[0]).expect("export");
        let loaded = super::load_case_file(&path).expect("import");
        assert_eq!(loaded.id, cases[0].id);
        assert!(loaded.project_template.runs.is_empty());
        assert!(loaded.project_template.mesh_asset.is_none());
    }
}
