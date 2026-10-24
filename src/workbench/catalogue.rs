//! Built-in, executable Workbench recipes; no source-tree files required.
#[path = "script_examples.rs"]
mod script_examples;
use super::{examples::*, project::*, WorkbenchSession};
use crate::{
    unstructured_incompressible::{AdaptiveTimeStep, TimeIntegration},
    MeshDimension,
};
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
fn manifest(
    id: &str,
    title: &str,
    category: &str,
    description: &str,
    session: WorkbenchSession,
    notes: &str,
) -> CaseTemplateManifest {
    let mut project = session.to_project(title);
    project.fluid_face = session.fluid_face();
    project.project_id = format!("template-{id}");
    CaseTemplateManifest {
        format_version: WORKBENCH_PROJECT_FORMAT_VERSION,
        id: id.into(),
        name: title.into(),
        category: category.into(),
        description: description.into(),
        capabilities: vec![
            "Workbench".into(),
            format!("{:?}", session.mesh_dimension()),
            "Editable".into(),
        ],
        expectations: TemplateExpectations {
            notes: notes.into(),
            ..Default::default()
        },
        project_template: project,
    }
}
pub fn builtin_templates() -> Result<Vec<CaseTemplateManifest>, String> {
    let mut templates = Vec::new();
    for d in example_descriptors() {
        let session = build_example(d.id).map_err(|e| e.to_string())?;
        let e = expectations(d.id);
        let mut entry = manifest(
            example_slug(d.id),
            d.title,
            d.category,
            d.short_description,
            session,
            &format!(
                "{} Recipe only; no reference-accuracy certification is implied.",
                d.expected_behavior
            ),
        );
        entry.capabilities = d.capabilities.iter().map(|s| s.to_string()).collect();
        entry.expectations.closed_domain = e.closed_domain;
        entry.expectations.positive_streamwise_flow = e.expected_positive_streamwise_flow;
        entry.expectations.curved_boundary = e.expects_curved_boundary;
        entry.expectations.non_ideal_mesh_quality = e.expects_non_ideal_quality;
        templates.push(entry);
    }
    let mut blank = WorkbenchProject::blank("New 3D analysis");
    blank.mesh.dimension = MeshDimension::ThreeD;
    templates.insert(
        1,
        manifest(
            "blank-3d",
            "New 3D analysis",
            "New project",
            "Empty 3D fluid-volume project.",
            WorkbenchSession::from_project(&blank).map_err(|e| e.to_string())?,
            "Create a fluid volume and define named boundaries before meshing.",
        ),
    );
    let mut transient =
        build_example(ExampleProjectId::LaminarChannel2D).map_err(|e| e.to_string())?;
    transient
        .set_flow_models(
            TimeIntegration::AdaptiveBackward {
                dt: 0.001,
                steps: 100,
                controls: AdaptiveTimeStep::default(),
            },
            transient.solver_options().turbulence.clone(),
        )
        .map_err(|e| e.to_string())?;
    transient
        .set_pressure_coupling(crate::simple::PressureCoupling {
            algorithm: crate::simple::CouplingAlgorithm::Pimple,
            ..Default::default()
        })
        .map_err(|e| e.to_string())?;
    let mut entry=manifest("transient-channel","Transient channel · adaptive BDF2","Experimental transient",
        "Fixed-mesh channel with Courant-controlled variable-step BDF2 and PIMPLE.",transient,
        "Experimental recipe. Inspect accepted physical steps and Courant; not benchmark validated.");
    entry.capabilities.extend([
        "Adaptive BDF2".into(),
        "PIMPLE".into(),
        "Experimental".into(),
    ]);
    templates.push(entry);
    let mut rans = build_example(ExampleProjectId::LaminarChannel2D).map_err(|e| e.to_string())?;
    rans.set_material(1.0, 1e-5).map_err(|e| e.to_string())?;
    let mut turbulence = rans.solver_options().turbulence.clone();
    turbulence.model = crate::turbulence::TurbulenceModel::KOmegaSst;
    turbulence.inlets.insert(
        "inlet".into(),
        crate::turbulence::TurbulenceInlet::IntensityDiameter {
            intensity: 0.05,
            hydraulic_diameter: 2.0,
            length_ratio: 0.07,
        },
    );
    rans.set_flow_models(TimeIntegration::Steady, turbulence)
        .map_err(|e| e.to_string())?;
    let mut entry=manifest("sst-channel","Channel · k–ω SST setup","Experimental RANS",
        "Wall-resolved SST starter with intensity/hydraulic-diameter inlet data; not a validated benchmark.",
        rans,"Draft setup. Review near-wall resolution/y+, domain development length and solver tolerances before relying on results.");
    entry.capabilities.extend([
        "k–ω SST".into(),
        "Intensity / diameter".into(),
        "Experimental".into(),
    ]);
    templates.push(entry);
    templates.extend(script_examples::templates()?);
    for entry in &mut templates {
        entry.project_template.cad_document = cad_document(&entry.project_template)?;
        #[cfg(feature = "gui")]
        seed_cad_geometry_hash(&mut entry.project_template)?;
        entry.validate().map_err(|e| e.to_string())?;
    }
    Ok(templates)
}

fn cad_document(project: &WorkbenchProject) -> Result<Option<String>, String> {
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
