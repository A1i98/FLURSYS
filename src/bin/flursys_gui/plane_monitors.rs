//! CAD plane monitor configuration and live cell-centred vector views.
use super::*;
use flursys::workbench::plane_monitor::{self, FlowPlaneMonitor};

pub(super) fn refresh_catalog(
    ui: &MainWindow,
    state: &mut AppState,
    planes: Vec<FlowPlaneMonitor>,
    selected: Option<String>,
) {
    let previous = state.workbench_project.plane_monitors.clone();
    let next = planes
        .into_iter()
        .map(|mut p| {
            if let Some(old) = previous.iter().find(|q| q.id == p.id) {
                p.enabled = old.enabled;
                p.every = old.every;
            }
            p
        })
        .collect::<Vec<_>>();
    for p in &next {
        if previous
            .iter()
            .find(|old| old.id == p.id)
            .is_some_and(|old| old.origin != p.origin || old.u != p.u || old.v != p.v)
        {
            state.post.plane_samples.remove(&p.id);
        }
    }
    if next != previous {
        state.workbench_project.plane_monitors = next;
        state.mark_workbench_dirty();
    }
    let items = state
        .workbench_project
        .plane_monitors
        .iter()
        .map(|p| SharedString::from(p.name.clone()))
        .collect::<Vec<_>>();
    ui.set_flow_plane_names(ModelRc::new(VecModel::from(items)));
    ui.set_cad_flow_plane_selected(selected.is_some());
    if let Some(id) = selected {
        if let Some(i) = state
            .workbench_project
            .plane_monitors
            .iter()
            .position(|p| p.id == id)
        {
            ui.set_flow_plane_index(i as i32);
        }
    }
    load(ui, state);
}
fn selected(ui: &MainWindow, state: &AppState) -> Option<FlowPlaneMonitor> {
    state
        .workbench_project
        .plane_monitors
        .get(ui.get_flow_plane_index().max(0) as usize)
        .cloned()
}
fn load(ui: &MainWindow, state: &AppState) {
    if let Some(p) = selected(ui, state) {
        ui.set_flow_plane_every(p.every.min(1_000_000) as i32);
        ui.set_flow_plane_enabled(p.enabled);
        ui.set_flow_plane_details(
            format!(
                "Origin: ({:.4e}, {:.4e}, {:.4e}) m\nNormal: ({:.3}, {:.3}, {:.3})",
                p.origin.x,
                p.origin.y,
                p.origin.z,
                p.normal().x,
                p.normal().y,
                p.normal().z
            )
            .into(),
        );
    }
    show_latest(ui, state);
}
pub(super) fn show_latest(ui: &MainWindow, state: &AppState) {
    let Some(plane) = selected(ui, state) else {
        return;
    };
    if let Some(snapshot) = state.post.plane_samples.get(&plane.id) {
        let key = format!("{}:{}:{:?}", plane.id, snapshot.iteration, snapshot.path);
        if ui.get_flow_plane_render_key() != key {
            if let Ok(image) =
                slint::Image::load_from_svg_data(plane_monitor::vector_svg(snapshot).as_bytes())
            {
                ui.set_flow_plane_image(image);
                ui.set_flow_plane_render_key(key.into());
            }
        }
        ui.set_flow_plane_status(
            format!(
                "{} · iteration {} · {} intersected cells\n{}",
                plane.name,
                snapshot.iteration,
                snapshot.samples.len(),
                snapshot
                    .path
                    .as_ref()
                    .map(|p| p.display().to_string())
                    .unwrap_or_else(|| "Final result preview; not a recorded file".into())
            )
            .into(),
        );
    } else {
        ui.set_flow_plane_image(Default::default());
        ui.set_flow_plane_render_key("".into());
        ui.set_flow_plane_status(
            "No sample yet. Enable recording before Run, or inspect the completed result.".into(),
        );
    }
}
fn export(ui: &MainWindow, state: &AppState, kind: i32) -> Result<(), String> {
    let plane = selected(ui, state).ok_or("Select a monitor plane")?;
    let snapshot = state
        .post
        .plane_samples
        .get(&plane.id)
        .ok_or("No plane sample is available yet")?;
    let extension = match kind {
        1 => "png",
        2 => "svg",
        _ => "csv",
    };
    let Some(path) = rfd::FileDialog::new()
        .add_filter("Plane snapshot", &[extension])
        .set_file_name(format!(
            "plane-iteration-{}.{}",
            snapshot.iteration, extension
        ))
        .save_file()
    else {
        return Ok(());
    };
    match kind {
        1 => {
            let pixels = ui
                .get_flow_plane_image()
                .to_rgba8()
                .ok_or("No rendered plane image")?;
            let mut file = tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
                .map_err(|e| e.to_string())?;
            {
                let mut png = png::Encoder::new(&mut file, pixels.width(), pixels.height());
                png.set_color(png::ColorType::Rgba);
                png.set_depth(png::BitDepth::Eight);
                let mut writer = png.write_header().map_err(|e| e.to_string())?;
                writer
                    .write_image_data(pixels.as_bytes())
                    .map_err(|e| e.to_string())?;
            }
            file.persist(path).map_err(|e| e.to_string())?;
            Ok(())
        }
        2 => result_analysis::atomic_text(&path, &plane_monitor::vector_svg(snapshot)),
        _ => result_analysis::atomic_text(&path, &plane_monitor::csv_text(snapshot)),
    }
}
pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_export(move |kind| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        if let Err(e) = export(&ui, &state.borrow(), kind) {
            ui.set_flow_plane_status(e.into());
        }
    });

    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_select(move |index| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        ui.set_flow_plane_index(index);
        load(&ui, &state.borrow());
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_apply(move||{let Some(ui)=weak.upgrade()else{return;};let mut s=state.borrow_mut();if s.solving{return;}
        let Some(mut plane)=selected(&ui,&s)else{return;};plane.every=ui.get_flow_plane_every().max(1) as usize;plane.enabled=ui.get_flow_plane_enabled();
        if let Err(e)=plane.validate(){ui.set_flow_plane_status(e.into());return;}
        if let Some(p)=s.workbench_project.plane_monitors.iter_mut().find(|p|p.id==plane.id){*p=plane.clone();}
        s.mark_workbench_dirty();ui.set_flow_plane_status(format!("{}: {}. Values are sampled after each completed SIMPLE iteration; iteration is not physical time.",plane.name,if plane.enabled{format!("record every {} iterations",plane.every)}else{"recording disabled".into()}).into());
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_view(move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut s = state.borrow_mut();
        if let Some(plane) = selected(&ui, &s) {
            if let Some(data) = s.active_result().filter(|_| !s.solving) {
                let result = plane_monitor::intersect(data.mesh(), &plane).and_then(|cells| {
                    plane_monitor::sample(
                        &plane,
                        &cells,
                        s.post.iteration,
                        data.velocity(),
                        data.pressure(),
                    )
                });
                match result {
                    Ok(sample) => {
                        s.post.plane_samples.insert(plane.id.clone(), sample);
                    }
                    Err(e) => {
                        ui.set_flow_plane_status(e.into());
                        return;
                    }
                }
            }
        }
        show_latest(&ui, &s);
        ui.set_solution_show_plane(true);
        ui.set_solution_show_log(false);
        apply_workflow_step(&ui, &mut s, 3);
    });
}
