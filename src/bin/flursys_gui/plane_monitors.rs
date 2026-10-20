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
                p.display = old.display.clone();
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
    if let Some(id) = selected.clone() {
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
    ui.set_flow_plane_error("".into());
    if let Some(p) = selected(ui, state) {
        ui.set_flow_plane_field(p.display.field as i32);
        ui.set_flow_plane_vectors(p.display.vectors);
        ui.set_flow_plane_edges(p.display.edges);
        ui.set_flow_plane_normalize(p.display.normalized_vectors);
        ui.set_flow_plane_normal_marks(p.display.normal_markers);
        ui.set_flow_plane_min(
            p.display
                .range
                .map(|r| r[0].to_string())
                .unwrap_or_default()
                .into(),
        );
        ui.set_flow_plane_max(
            p.display
                .range
                .map(|r| r[1].to_string())
                .unwrap_or_default()
                .into(),
        );
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
    ui.set_flow_plane_playing(state.post.plane_playing);
    ui.set_flow_plane_frame(state.post.plane_frame as i32);
    ui.set_flow_plane_count(state.post.plane_frames.len() as i32);
    if let Some(path) = state.post.plane_frames.get(state.post.plane_frame) {
        let style = selected(ui, state).map(|p| p.display).unwrap_or_default();
        let key = format!("replay:{path:?}:{style:?}");
        if ui.get_flow_plane_render_key() != key {
            let rendered = if path.extension().is_some_and(|e| e == "json") {
                std::fs::read(path)
                    .map_err(|e| e.to_string())
                    .and_then(|bytes| {
                        let sample: plane_monitor::PlaneSnapshot =
                            serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
                        slint::Image::load_from_svg_data(
                            plane_monitor::section_svg(&sample, &style).as_bytes(),
                        )
                        .map_err(|e| e.to_string())
                    })
            } else {
                slint::Image::load_from_path(path).map_err(|e| e.to_string())
            };
            match rendered {
                Ok(image) => {
                    ui.set_flow_plane_image(image);
                    ui.set_flow_plane_render_key(key.into());
                }
                Err(e) => {
                    ui.set_flow_plane_status(format!("Cannot load frame: {e}").into());
                    return;
                }
            }
        }
        ui.set_flow_plane_status(
            format!(
                "Frame {}/{} · {} · SIMPLE iteration history{}",
                state.post.plane_frame + 1,
                state.post.plane_frames.len(),
                path.file_stem().unwrap_or_default().to_string_lossy(),
                if path.extension().is_some_and(|e| e == "svg") {
                    " · legacy SVG: recorded appearance"
                } else {
                    ""
                }
            )
            .into(),
        );
        return;
    }
    let error = ui.get_flow_plane_error();
    if !error.is_empty() {
        ui.set_flow_plane_status(error);
        return;
    }
    let Some(plane) = selected(ui, state) else {
        ui.set_flow_plane_image(Default::default());
        ui.set_flow_plane_render_key("".into());
        ui.set_flow_plane_status("Select a CAD section plane".into());
        return;
    };
    if let Some(snapshot) = state.post.plane_samples.get(&plane.id) {
        let key = format!(
            "{}:{}:{:?}:{:?}",
            plane.id, snapshot.iteration, snapshot.path, plane.display
        );
        if ui.get_flow_plane_render_key() != key {
            if let Ok(image) = slint::Image::load_from_svg_data(
                plane_monitor::section_svg(snapshot, &plane.display).as_bytes(),
            ) {
                ui.set_flow_plane_image(image);
                ui.set_flow_plane_render_key(key.into());
            }
        }
        ui.set_flow_plane_status(
            format!(
                "{} · iteration {} · {} cells · {}\n{}",
                plane.name,
                snapshot.iteration,
                snapshot.samples.len(),
                status_label(&state.workbench),
                snapshot
                    .path
                    .as_ref()
                    .map(|p| p.display().to_string())
                    .unwrap_or_else(|| "Selected result preview; not a recorded file".into())
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
fn sample_selected(ui: &MainWindow, state: &mut AppState) -> Result<(), String> {
    let Some(plane) = selected(ui, state) else {
        return Err("Select a CAD section plane".into());
    };
    if state.solving {
        return Ok(());
    }
    let Some(data) = state.active_result() else {
        return Ok(());
    };
    let cells = plane_monitor::intersect(data.mesh(), &plane)?;
    if cells.is_empty() {
        return Err(
            "This plane does not intersect any fluid cells. Check its origin and offset in metres."
                .into(),
        );
    }
    let iteration = state
        .workbench_project
        .runs
        .iter()
        .find(|r| r.id == data.run_id())
        .and_then(|r| r.iterations)
        .unwrap_or(state.post.iteration);
    let sample =
        plane_monitor::sample(&plane, &cells, iteration, data.velocity(), data.pressure())?;
    state.post.plane_samples.insert(plane.id.clone(), sample);
    Ok(())
}
fn export(ui: &MainWindow, state: &AppState, kind: i32) -> Result<(), String> {
    let plane = selected(ui, state).ok_or("Select a monitor plane")?;
    let archived = if let Some(path) = state.post.plane_frames.get(state.post.plane_frame) {
        if path.extension().is_some_and(|e| e == "json") {
            Some(
                serde_json::from_slice::<plane_monitor::PlaneSnapshot>(
                    &std::fs::read(path).map_err(|e| e.to_string())?,
                )
                .map_err(|e| e.to_string())?,
            )
        } else {
            if kind == 0 {
                return Err("Legacy SVG recordings have no replayable field data. Export the adjacent CSV directly.".into());
            }
            if kind == 2 {
                let Some(target) = rfd::FileDialog::new()
                    .set_file_name("plane-frame.svg")
                    .save_file()
                else {
                    return Ok(());
                };
                std::fs::copy(path, target).map_err(|e| e.to_string())?;
                return Ok(());
            }
            None
        }
    } else {
        None
    };
    let snapshot = archived
        .as_ref()
        .or_else(|| state.post.plane_samples.get(&plane.id));
    if snapshot.is_none() && kind != 1 {
        return Err("No plane sample is available yet".into());
    }
    let iteration = snapshot
        .map(|s| s.iteration)
        .unwrap_or(state.post.plane_frame);
    show_latest(ui, state);
    let extension = match kind {
        1 => "png",
        2 => "svg",
        _ => "csv",
    };
    let Some(path) = rfd::FileDialog::new()
        .add_filter("Plane snapshot", &[extension])
        .set_file_name(format!("plane-iteration-{}.{}", iteration, extension))
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
        2 => result_analysis::atomic_text(
            &path,
            &plane_monitor::section_svg(snapshot.ok_or("No field snapshot")?, &plane.display),
        ),
        _ => result_analysis::atomic_text(
            &path,
            &plane_monitor::csv_text(snapshot.ok_or("No field snapshot")?),
        ),
    }
}
fn display(ui: &MainWindow) -> Result<plane_monitor::PlaneDisplay, String> {
    let lo = ui.get_flow_plane_min();
    let hi = ui.get_flow_plane_max();
    let range = if lo.trim().is_empty() && hi.trim().is_empty() {
        None
    } else {
        Some([
            lo.trim()
                .parse::<f64>()
                .map_err(|_| "Enter both numeric range limits")?,
            hi.trim()
                .parse::<f64>()
                .map_err(|_| "Enter both numeric range limits")?,
        ])
    };
    let style = plane_monitor::PlaneDisplay {
        field: ui.get_flow_plane_field().max(0) as usize,
        vectors: ui.get_flow_plane_vectors(),
        edges: ui.get_flow_plane_edges(),
        normalized_vectors: ui.get_flow_plane_normalize(),
        normal_markers: ui.get_flow_plane_normal_marks(),
        range,
    };
    style.validate()?;
    Ok(style)
}
fn frame_files(folder: &Path) -> Vec<PathBuf> {
    let mut paths = std::fs::read_dir(folder)
        .into_iter()
        .flatten()
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| {
            p.file_name()
                .is_some_and(|n| n.to_string_lossy().starts_with("iteration-"))
        })
        .collect::<Vec<_>>();
    let json = paths
        .iter()
        .any(|p| p.extension().is_some_and(|e| e == "json"));
    paths.retain(|p| {
        p.extension()
            .is_some_and(|e| e == if json { "json" } else { "svg" })
    });
    paths.sort();
    paths
}
fn historical_folder(
    workspace: &Path,
    record: &flursys::workbench::RunRecord,
    lower_ms: u128,
    plane: &FlowPlaneMonitor,
) -> Option<PathBuf> {
    let encoded = plane
        .id
        .bytes()
        .map(|b| format!("{b:02x}"))
        .collect::<String>();
    if let Ok(bytes) = std::fs::read(workspace.join(&record.report_path)) {
        if let Ok(report) = serde_json::from_slice::<serde_json::Value>(&bytes) {
            if let Some(relative) = report
                .get("plane_recording_folder")
                .and_then(|v| v.as_str())
            {
                let relative = Path::new(relative);
                if relative
                    .components()
                    .all(|c| matches!(c, std::path::Component::Normal(_)))
                {
                    let folder = workspace.join(relative).join(&encoded);
                    let saved: FlowPlaneMonitor =
                        serde_json::from_slice(&std::fs::read(folder.join("plane.json")).ok()?)
                            .ok()?;
                    if saved.id != plane.id
                        || saved.origin != plane.origin
                        || saved.u != plane.u
                        || saved.v != plane.v
                    {
                        return None;
                    }
                    return (!frame_files(&folder).is_empty()).then_some(folder);
                }
            }
        }
    }
    // Older reports have no explicit link. Require an unambiguous time window,
    // matching plane geometry and final iteration; never choose merely the latest run.
    let expected = format!("iteration-{:08}", record.iterations?);
    let candidates = std::fs::read_dir(workspace.join("monitors"))
        .ok()?
        .filter_map(Result::ok)
        .filter_map(|entry| {
            let name = entry.file_name();
            let name = name.to_str()?;
            let start_ms = name.strip_prefix("solve-")?.parse::<u128>().ok()? / 1_000_000;
            if start_ms <= lower_ms || start_ms > record.created_unix_ms {
                return None;
            }
            let folder = entry.path().join(&encoded);
            let saved: FlowPlaneMonitor =
                serde_json::from_slice(&std::fs::read(folder.join("plane.json")).ok()?).ok()?;
            if saved.id != plane.id
                || saved.origin != plane.origin
                || saved.u != plane.u
                || saved.v != plane.v
            {
                return None;
            }
            let frames = frame_files(&folder);
            if frames.last()?.file_stem()?.to_str()? != expected {
                return None;
            }
            Some(folder)
        })
        .collect::<Vec<_>>();
    (candidates.len() == 1).then(|| candidates[0].clone())
}
pub(super) fn refresh_selected(ui: &MainWindow, state: &mut AppState) {
    state.post.plane_frames.clear();
    state.post.plane_playing = false;
    if let Err(e) = sample_selected(ui, state) {
        ui.set_flow_plane_status(e.into());
        return;
    }
    if !state.solving {
        if let (Some(workspace), Some(plane), Some(data)) = (
            &state.workspace_path,
            selected(ui, state),
            state.active_result(),
        ) {
            if let Some(record) = state
                .workbench_project
                .runs
                .iter()
                .find(|r| r.id == data.run_id())
            {
                let lower = state
                    .workbench_project
                    .runs
                    .iter()
                    .filter(|r| r.created_unix_ms < record.created_unix_ms)
                    .map(|r| r.created_unix_ms)
                    .max()
                    .unwrap_or(0);
                if let Some(folder) = historical_folder(workspace, record, lower, &plane) {
                    state.post.plane_frames = frame_files(&folder);
                    state.post.plane_frame = state.post.plane_frames.len().saturating_sub(1);
                }
            }
        }
    }
    load(ui, state);
}

pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let control_state = shared.clone();
    ui.on_solver_control(move |action| {
        let Some(ui)=weak.upgrade() else{return;};let mut s=control_state.borrow_mut();
        if action==2 && !s.solving {
            let Some(mesh)=s.workbench.mesh() else{s.log("Generate a mesh before loading a checkpoint.");return;};
            let mut dialog=rfd::FileDialog::new().add_filter("Field checkpoint",&["json"]);
            if let Some(workspace)=&s.workspace_path {dialog=dialog.set_directory(workspace.join("monitors"));}
            let Some(path)=dialog.pick_file() else{return;};
            match flursys::workbench::checkpoint::load(&path,&mesh.mesh) {
                Ok((iteration,_))=>{s.workbench_jobs.set_restart(Some(path));s.log(format!("Field restart queued from iteration {iteration}. Next Run uses these velocity/pressure fields with current settings; flux and convergence history are rebuilt."));},
                Err(e)=>{s.workbench_jobs.set_restart(None);s.log(format!("Checkpoint rejected: {e}"));},
            }
        } else if action==1 && s.solving {
            let paused=!s.workbench_jobs.is_paused();s.workbench_jobs.set_paused(paused);ui.set_solver_paused(paused);
            s.log(if paused {"Pause requested; the worker retains all solver fields. Resume continues the same solve."}else{"Solve resumed from retained fields."});
        } else if action==0 && !s.solving {
            let case=match s.workbench.prepare_case(){Ok(c)=>c,Err(e)=>{s.log(e.to_string());return;}};
            s.workbench_jobs.set_restart(None);
            s.workbench.initialize_solution();
            s.active_result=None;
            s.post.reset_result();ui.set_solver_paused(false);
            let velocity=vec![flursys::Vec3::ZERO;case.mesh.cell_count()];
            let pressure=vec![0.0;case.mesh.cell_count()];
            for plane in s.workbench_project.plane_monitors.clone() {
                if let Ok(cells)=plane_monitor::intersect(&case.mesh,&plane) {
                    if let Ok(snapshot)=plane_monitor::sample(&plane,&cells,0,&velocity,&pressure){s.post.plane_samples.insert(plane.id.clone(),snapshot);}
                }
            }
            show_latest(&ui,&s);
            s.log("Initialized uniform U=(0,0,0) m/s and p/rho=0. Boundary conditions are applied when Run starts. Archived runs are retained.");
        }
        refresh_ui(&ui,&s);
    });

    let weak = ui.as_weak();
    let shared_state = shared.clone();
    ui.on_flow_plane_playback(move |action| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut s = shared_state.borrow_mut();
        match action {
            0 => {
                let current = selected(&ui, &s)
                    .and_then(|p| s.post.plane_samples.get(&p.id))
                    .and_then(|p| p.path.as_ref())
                    .and_then(|p| p.parent())
                    .map(Path::to_path_buf)
                    .or_else(|| {
                        let plane = selected(&ui, &s)?;
                        let encoded = plane
                            .id
                            .bytes()
                            .map(|b| format!("{b:02x}"))
                            .collect::<String>();
                        let root = s.workspace_path.as_ref()?.join("monitors");
                        let mut folders = std::fs::read_dir(&root)
                            .ok()?
                            .filter_map(Result::ok)
                            .map(|e| e.path().join(&encoded))
                            .filter(|p| p.join("plane.json").is_file())
                            .collect::<Vec<_>>();
                        folders.sort();
                        folders.pop().or(Some(root))
                    });
                let mut dialog = rfd::FileDialog::new()
                    .set_title("Select recorded plane folder (contains iteration files)");
                if let Some(folder) = current {
                    dialog = dialog.set_directory(folder);
                }
                let Some(folder) = dialog.pick_folder() else {
                    return;
                };
                s.post.plane_frames = frame_files(&folder);
                s.post.plane_frame = 0;
                s.post.plane_playing = false;
                if s.post.plane_frames.is_empty() {
                    ui.set_flow_plane_status("No recorded frames in this folder".into());
                    return;
                }
            }
            1 => {
                if s.post.plane_frames.is_empty() {refresh_selected(&ui,&mut s);}
                if s.post.plane_frames.is_empty() {ui.set_flow_plane_status("No recording linked to this run/plane. Use Open recording to choose its plane folder.".into());return;}
                if s.post.plane_frame + 1 >= s.post.plane_frames.len() {
                    s.post.plane_frame = 0;
                }
                s.post.plane_playing = !s.post.plane_playing && !s.post.plane_frames.is_empty();
                s.post.plane_tick = Some(std::time::Instant::now());
            }
            2 => {
                s.post.plane_playing = false;
                s.post.plane_frame = s.post.plane_frame.saturating_sub(1);
            }
            3 => {
                s.post.plane_playing = false;
                s.post.plane_frame =
                    (s.post.plane_frame + 1).min(s.post.plane_frames.len().saturating_sub(1));
            }
            n if n >= 1000 => {
                s.post.plane_playing = false;
                s.post.plane_frame =
                    ((n - 1000) as usize).min(s.post.plane_frames.len().saturating_sub(1));
            }
            _ => {
                s.post.plane_playing = false;
                s.post.plane_frames.clear();
            }
        }
        show_latest(&ui, &s);
    });

    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_style(move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = state.borrow_mut();
        let style = match display(&ui) {
            Ok(s) => s,
            Err(e) => {
                ui.set_flow_plane_error(e.clone().into());
                ui.set_flow_plane_status(e.into());
                return;
            }
        };
        if let Some(p) = state
            .workbench_project
            .plane_monitors
            .get_mut(ui.get_flow_plane_index().max(0) as usize)
        {
            p.display = style;
            state.mark_workbench_dirty();
        }
        ui.set_flow_plane_error("".into());
        ui.set_flow_plane_render_key("".into());
        show_latest(&ui, &state);
    });

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
        refresh_selected(&ui, &mut state.borrow_mut());
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_apply(move||{let Some(ui)=weak.upgrade()else{return;};let mut s=state.borrow_mut();if s.solving{return;}
        let Some(mut plane)=selected(&ui,&s)else{return;};plane.display=match display(&ui) {Ok(d)=>d,Err(e)=>{ui.set_flow_plane_status(e.into());return;}};plane.every=ui.get_flow_plane_every().max(1) as usize;plane.enabled=ui.get_flow_plane_enabled();
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
        refresh_selected(&ui, &mut s);
        ui.set_post_tab(5);
        apply_workflow_step(&ui, &mut s, 4);
    });
}

#[cfg(test)]
mod recording_link_tests {
    use super::*;
    #[test]
    fn legacy_recording_lookup_requires_unique_matching_run_window_and_plane() {
        let root = tempfile::tempdir().unwrap();
        let plane = FlowPlaneMonitor {
            id: "plane".into(),
            name: "Plane1".into(),
            origin: flursys::Vec3::new(0., 0., 0.005),
            u: flursys::Vec3::new(1., 0., 0.),
            v: flursys::Vec3::new(0., 1., 0.),
            enabled: true,
            every: 10,
            display: Default::default(),
        };
        let encoded = plane
            .id
            .bytes()
            .map(|b| format!("{b:02x}"))
            .collect::<String>();
        let create = |start: u128| {
            let folder = root
                .path()
                .join("monitors")
                .join(format!("solve-{}", start * 1_000_000))
                .join(&encoded);
            std::fs::create_dir_all(&folder).unwrap();
            std::fs::write(
                folder.join("plane.json"),
                serde_json::to_vec(&plane).unwrap(),
            )
            .unwrap();
            std::fs::write(folder.join("iteration-00000020.json"), "{}").unwrap();
            folder
        };
        let mut record = flursys::workbench::WorkbenchProject::blank("fixture")
            .next_run(flursys::RunStatus::Converged);
        record.created_unix_ms = 3000;
        record.iterations = Some(20);
        create(500);
        create(4000);
        let correct = create(2000);
        assert_eq!(
            historical_folder(root.path(), &record, 1000, &plane),
            Some(correct.clone())
        );
        let mut moved = plane.clone();
        moved.origin.z = 0.006;
        assert!(historical_folder(root.path(), &record, 1000, &moved).is_none());
        create(2500);
        assert!(historical_folder(root.path(), &record, 1000, &plane).is_none());
        let report = root.path().join(&record.report_path);
        std::fs::create_dir_all(report.parent().unwrap()).unwrap();
        std::fs::write(report,serde_json::to_vec(&serde_json::json!({"plane_recording_folder":correct.parent().unwrap().strip_prefix(root.path()).unwrap()})).unwrap()).unwrap();
        assert_eq!(
            historical_folder(root.path(), &record, 1000, &plane),
            Some(correct)
        );
    }
}
