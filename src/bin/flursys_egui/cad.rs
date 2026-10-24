use super::*;
pub struct Published {
    pub geometry: GeometryTopology,
    pub document: String,
    pub planes: Vec<plane_monitor::FlowPlaneMonitor>,
    pub hash: String,
}
pub fn services() -> cad_workbench::Services {
    cad_workbench::Services {
        pick_open: Some(Box::new(|extensions| {
            let extensions = extensions.iter().map(|s| s.to_string()).collect::<Vec<_>>();
            Box::pin(async move {
                rfd::AsyncFileDialog::new()
                    .add_filter("CAD files", &extensions)
                    .pick_file()
                    .await
                    .map(|f| f.path().to_string_lossy().to_string())
            })
        })),
        pick_save: Some(Box::new(|name, extensions| {
            let name = name.to_string();
            let extensions = extensions.iter().map(|s| s.to_string()).collect::<Vec<_>>();
            Box::pin(async move {
                rfd::AsyncFileDialog::new()
                    .set_file_name(name)
                    .add_filter("CAD files", &extensions)
                    .save_file()
                    .await
                    .map(|f| f.path().to_string_lossy().to_string())
            })
        })),
        ..Default::default()
    }
}
pub fn app(
    document: Option<&str>,
    render: Option<&egui_wgpu::RenderState>,
) -> Result<cad_workbench::SolveApp, String> {
    let doc = document
        .map(cad_engine::doc::Document::from_json)
        .transpose()
        .map_err(|e| e.to_string())?
        .unwrap_or_default();
    let mut app = cad_workbench::SolveApp::new(cad_engine::Session::new(doc), services());
    app.home.open = false;
    app.custom_titlebar = false;
    app.ui.show_timeline = true;
    app.preferences.timeline_left = false;
    if let Some(rs) = render {
        app.set_wgpu(rs, 24, 8, 1);
    }
    app.fit_view();
    Ok(app)
}
pub fn publish(document: String) -> Result<Published, String> {
    let doc = cad_engine::doc::Document::from_json(&document).map_err(|e| e.to_string())?;
    let session = cad_engine::Session::new(doc);
    let model = cad_engine::doc::world_state(&session.doc, &session.model.state());
    let hash = model.host_geometry_hash().map_err(|e| e.to_string())?;
    let mut geometry = GeometryTopology::new();
    if !model.bodies.is_empty() {
        for body in &model.bodies {
            let scaled = cad_kernel::transform_matrix(
                &body.body,
                [
                    [0.001, 0., 0., 0.],
                    [0., 0.001, 0., 0.],
                    [0., 0., 0.001, 0.],
                    [0., 0., 0., 1.],
                ],
            )
            .map_err(|e| e.to_string())?;
            geometry
                .cad_import_brep(
                    &scaled.host_brep_json().map_err(|e| e.to_string())?,
                    &body.name,
                )
                .map_err(|e| e.to_string())?;
        }
    } else {
        for sketch in &model.sketches {
            if sketch.profiles.is_empty() {
                continue;
            }
            let p = sketch.plane;
            let v = |p: cad_engine::geom::Vec3| Vec3::new(p.x, p.y, p.z);
            geometry
                .cad_import_drawing_frame(
                    CadPlaneFrame {
                        origin: v(p.origin) * 0.001,
                        x_axis: v(p.x),
                        y_axis: v(p.y),
                        normal: v(p.x.cross(p.y)),
                    },
                    sketch.sketch.clone(),
                )
                .map_err(|e| e.to_string())?;
        }
    }
    let mut planes = Vec::new();
    let mut add = |id: String, name: String, p: cad_engine::geom::Plane| {
        let v = |a: cad_engine::geom::Vec3| Vec3::new(a.x, a.y, a.z);
        planes.push(plane_monitor::FlowPlaneMonitor {
            id,
            name,
            origin: v(p.origin) * 0.001,
            u: v(p.x),
            v: v(p.y),
            enabled: false,
            every: 10,
            display: Default::default(),
        });
    };
    for (name, p) in [
        ("XY", cad_engine::geom::Plane::XY),
        ("XZ", cad_engine::geom::Plane::XZ),
        ("YZ", cad_engine::geom::Plane::YZ),
    ] {
        add(format!("origin-{name}"), name.into(), p);
    }
    for c in &model.construct {
        if let cad_engine::doc::construct::ConstructGeom::Plane(p) = c.geom {
            add(format!("datum-{}", c.feature), c.name.clone(), p);
        }
    }
    for s in &model.sketches {
        add(
            format!("sketch-{}", s.feature),
            format!("{} · sketch plane", s.name),
            s.plane,
        );
    }
    Ok(Published {
        geometry,
        document,
        planes,
        hash,
    })
}

/// Preserve workspaces predating an embedded CAD document. Exact imported bodies
/// become portable STEP-backed native features; canonical CFD IDs remain untouched.
pub fn restore_document(project: &WorkbenchProject) -> Result<Option<String>, String> {
    if let Some(doc) = &project.cad_document {
        return Ok(Some(doc.clone()));
    }
    if let Ok(doc) = workbench::catalogue::cad_document(project) {
        return Ok(doc);
    }
    let bodies: Vec<_> = project
        .geometry
        .cad_bodies()
        .filter(|(_, b)| b.visible)
        .map(|(id, b)| (id, b.name.clone()))
        .collect();
    if bodies.is_empty() {
        return Ok(None);
    }
    let mut geometry = project.geometry.clone();
    let mut document = cad_engine::doc::Document::new(&project.name);
    for (id, name) in bodies {
        let scaled = geometry
            .cad_transform(
                id,
                flursys::CadTransform::Scale {
                    pivot: Vec3::ZERO,
                    factor: 1000.,
                },
                true,
            )
            .map_err(|e| e.to_string())?;
        let step = geometry
            .cad_export_step(scaled)
            .map_err(|e| e.to_string())?;
        document
            .add_feature(
                cad_engine::doc::FeatureKind::Import {
                    file: format!("{name}.step"),
                    step: step.into(),
                    components: vec![],
                    skip: vec![],
                },
                Some(&name),
            )
            .map_err(|e| e.to_string())?;
    }
    Ok(Some(document.to_json()))
}
