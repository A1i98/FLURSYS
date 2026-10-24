//! CAD plane monitor configuration and live cell-centred vector views.
use super::*;
use flursys::workbench::plane_monitor::{self, FlowPlaneMonitor};

pub(super) fn refresh_locations(ui: &MainWindow, state: &AppState) {
    let mut names: Vec<SharedString> = ["3D surface", "XY section", "XZ section", "YZ section"]
        .into_iter()
        .map(SharedString::from)
        .collect();
    let mut indices = Vec::new();
    for (i, p) in state.workbench_project.plane_monitors.iter().enumerate() {
        if matches!(p.name.as_str(), "XY" | "XZ" | "YZ") && p.origin.norm() < 1e-12 {
            continue;
        }
        names.push(p.name.clone().into());
        indices.push(i as i32);
    }
    if ui.get_result_section_names().iter().collect::<Vec<_>>() != names
        || ui
            .get_result_section_plane_indices()
            .iter()
            .collect::<Vec<_>>()
            != indices
    {
        ui.set_result_section_names(ModelRc::new(VecModel::from(names)));
        ui.set_result_section_plane_indices(ModelRc::new(VecModel::from(indices)));
    }
}
fn load_display(ui: &MainWindow, state: &AppState) {
    let Some(p) = selected(ui, state) else { return };
    ui.set_result_plot_index(if p.display.plot == 3 {
        3
    } else if p.display.vectors {
        2
    } else if p.display.plot > 0 {
        1
    } else {
        0
    });
    ui.set_result_contour_levels(p.display.levels as i32);
    ui.set_result_colormap_index(p.display.palette as i32);
    ui.set_post_edges(p.display.edges);
    ui.set_result_vector_density(if p.display.vector_density <= 150 {
        0
    } else if p.display.vector_density >= 1200 {
        2
    } else {
        1
    });
    ui.set_result_vector_scale(p.display.vector_scale.to_string().into());
    ui.set_result_range_min(ui.get_flow_plane_min());
    ui.set_result_range_max(ui.get_flow_plane_max());
}
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
    refresh_locations(ui, state);
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
        ui.set_flow_plane_palette(p.display.palette as i32);
        ui.set_flow_plane_density(if p.display.vector_density <= 150 {
            0
        } else if p.display.vector_density >= 1200 {
            2
        } else {
            1
        });
        ui.set_flow_plane_scale(p.display.vector_scale.to_string().into());
        ui.set_flow_plane_plot(p.display.plot as i32);
        ui.set_flow_plane_levels(p.display.levels as i32);
        ui.set_flow_plane_streamlines(p.display.streamlines);
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
pub(super) fn playback_bounds(ui: &MainWindow, count: usize) -> (usize, usize) {
    let maximum = count.saturating_sub(1);
    let first = (ui.get_flow_plane_first().max(1) as usize - 1).min(maximum);
    let last = if ui.get_flow_plane_last() <= 0 {
        maximum
    } else {
        (ui.get_flow_plane_last() as usize - 1).min(maximum)
    };
    (first.min(last), last)
}
fn validate_recorded_plane(
    ui: &MainWindow,
    state: &AppState,
    sample: &plane_monitor::PlaneSnapshot,
) -> Result<(), String> {
    sample.plane.validate()?;
    let expected = selected(ui, state).ok_or("Select the recorded plane")?;
    if sample.plane.id != expected.id
        || sample.plane.origin != expected.origin
        || sample.plane.u != expected.u
        || sample.plane.v != expected.v
    {
        return Err("Recording belongs to a different plane or its geometry has changed. Select the matching plane/recording.".into());
    }
    Ok(())
}
fn current_snapshot(
    ui: &MainWindow,
    state: &AppState,
) -> Result<std::sync::Arc<plane_monitor::PlaneSnapshot>, String> {
    if let Some(path) = state.post.plane_frames.get(state.post.plane_frame) {
        let mut cached = state.post.section_archive.borrow_mut();
        if let Some((key, sample)) = cached.as_ref() {
            if key == path {
                validate_recorded_plane(ui, state, sample)?;
                return Ok(sample.clone());
            }
        }
        let sample: plane_monitor::PlaneSnapshot =
            serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
                .map_err(|e| format!("Raw frame unavailable: {e}"))?;
        validate_recorded_plane(ui, state, &sample)?;
        let sample = std::sync::Arc::new(sample);
        *cached = Some((path.clone(), sample.clone()));
        return Ok(sample);
    }
    let plane = selected(ui, state).ok_or("Select a section")?;
    state
        .post
        .plane_samples
        .get(&plane.id)
        .cloned()
        .map(std::sync::Arc::new)
        .ok_or_else(|| "No section field data".to_string())
}
pub(super) fn show_latest(ui: &MainWindow, state: &AppState) {
    if state.post.section_probe.borrow().is_none() {
        ui.set_flow_plane_readout("".into());
    }
    if let Some(point) = *state.post.section_probe.borrow() {
        if let Ok(snapshot) = current_snapshot(ui, state) {
            let text=plane_monitor::probe_world(&snapshot,point).map(|sample|format!("Cell {} · U=({:.5e}, {:.5e}, {:.5e}) m/s · |U|={:.5e} m/s · p/ρ={:.5e} m²/s²",sample.geometry.cell,sample.velocity.x,sample.velocity.y,sample.velocity.z,sample.velocity.norm(),sample.pressure)).unwrap_or_else(||"Probe is outside this frame's section".into());
            ui.set_flow_plane_readout(text.into());
        }
    }

    ui.set_flow_plane_playing(state.post.plane_playing);
    ui.set_flow_plane_frame(state.post.plane_frame as i32);
    ui.set_flow_plane_count(state.post.plane_frames.len() as i32);
    if let Some(path) = state.post.plane_frames.get(state.post.plane_frame) {
        let style = selected(ui, state).map(|p| p.display).unwrap_or_default();
        let key = format!(
            "replay:{path:?}:{style:?}:{:?}:{:?}",
            state.post.section_camera.borrow(),
            state.post.section_probe.borrow()
        );
        if ui.get_flow_plane_render_key() != key {
            let rendered = if path.extension().is_some_and(|e| e == "json") {
                current_snapshot(ui, state).and_then(|sample| {
                    let clock = sample
                        .physical_time
                        .map(|t| format!("physical time {t:.6e} s"))
                        .unwrap_or_else(|| "steady iteration, not physical time".into());
                    ui.set_flow_plane_status(
                        format!(
                            "Frame {}/{} · iteration {} · {}",
                            state.post.plane_frame + 1,
                            state.post.plane_frames.len(),
                            sample.iteration,
                            clock
                        )
                        .into(),
                    );
                    slint::Image::load_from_svg_data(
                        plane_monitor::section_interactive_probe_svg(
                            &sample,
                            &style,
                            &state.post.section_camera.borrow(),
                            *state.post.section_probe.borrow(),
                        )
                        .as_bytes(),
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
                    ui.set_flow_plane_image(Default::default());
                    ui.set_flow_plane_render_key("".into());
                    ui.set_flow_plane_readout("".into());
                    ui.set_flow_plane_status(format!("Cannot load frame: {e}").into());
                    return;
                }
            }
        }
        if path.extension().is_some_and(|e| e == "svg") {
            ui.set_flow_plane_status(
                format!(
                    "Frame {}/{} · {} · legacy SVG: recorded appearance",
                    state.post.plane_frame + 1,
                    state.post.plane_frames.len(),
                    path.file_stem().unwrap_or_default().to_string_lossy()
                )
                .into(),
            );
        }
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
            "{}:{}:{:?}:{:?}:{:?}:{:?}",
            plane.id,
            snapshot.iteration,
            snapshot.path,
            plane.display,
            state.post.section_camera.borrow(),
            state.post.section_probe.borrow()
        );
        if ui.get_flow_plane_render_key() != key {
            if let Ok(image) = slint::Image::load_from_svg_data(
                plane_monitor::section_interactive_probe_svg(
                    snapshot,
                    &plane.display,
                    &state.post.section_camera.borrow(),
                    *state.post.section_probe.borrow(),
                )
                .as_bytes(),
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
    let mut sample =
        plane_monitor::sample(&plane, &cells, iteration, data.velocity(), data.pressure())?;
    sample.physical_time = state
        .workbench_project
        .runs
        .iter()
        .find(|r| r.id == data.run_id())
        .and_then(|r| r.physical_time)
        .or_else(|| state.workbench.solution().and_then(|s| s.physical_time));
    state.post.plane_samples.insert(plane.id.clone(), sample);
    Ok(())
}
fn export(ui: &MainWindow, state: &AppState, kind: i32) -> Result<(), String> {
    let plane = selected(ui, state).ok_or("Select a monitor plane")?;
    let archived = if let Some(path) = state.post.plane_frames.get(state.post.plane_frame) {
        if path.extension().is_some_and(|e| e == "json") {
            Some(current_snapshot(ui, state)?)
        } else {
            if matches!(kind, 0 | 3 | 4 | 5) {
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
        .as_deref()
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
        .set_file_name(format!(
            "plane-{}-iteration-{}.{}",
            match kind {
                3 => "statistics",
                4 => "horizontal-profile",
                5 => "vertical-profile",
                _ => "field",
            },
            iteration,
            extension
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
        4 | 5 => result_analysis::atomic_text(
            &path,
            &flursys::workbench::plane_statistics::centreline_csv(
                snapshot.ok_or("No field snapshot")?,
                kind == 5,
                201,
                plane.display.field,
            )?,
        ),
        3 => result_analysis::atomic_text(
            &path,
            &plane_monitor::section_statistics_csv(
                snapshot.ok_or("No field snapshot")?,
                plane.display.field,
            )?,
        ),
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
        palette: ui.get_flow_plane_palette().clamp(0, 3) as usize,
        vector_density: match ui.get_flow_plane_density() {
            0 => 150,
            2 => 1200,
            _ => 600,
        },
        vector_scale: ui
            .get_flow_plane_scale()
            .parse::<f64>()
            .map_err(|_| "Enter a positive vector scale")?,
        plot: ui.get_flow_plane_plot().max(0) as usize,
        levels: ui.get_flow_plane_levels().max(3) as usize,
        streamlines: ui.get_flow_plane_streamlines(),
        field: ui.get_flow_plane_field().max(0) as usize,
        vectors: ui.get_flow_plane_vectors(),
        edges: ui.get_flow_plane_edges(),
        normalized_vectors: ui.get_flow_plane_normalize(),
        normal_markers: ui.get_flow_plane_normal_marks(),
        range,
        ..Default::default()
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
    *state.post.section_archive.borrow_mut() = None;
    *state.post.section_probe.borrow_mut() = None;
    {
        let mut camera = state.post.section_camera.borrow_mut();
        camera.zoom = 1.;
        camera.pan = [0.; 2];
    }
    ui.set_flow_plane_readout("".into());
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
                    state
                        .post
                        .monitors
                        .load(state.post.plane_frames.clone(), plane.clone());
                }
            }
        }
    }
    load(ui, state);
}

pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let display_state = shared.clone();
    ui.on_plane_display_style(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = display_state.borrow_mut();
        ui.set_flow_plane_plot(if ui.get_result_plot_index() == 3 {
            3
        } else if matches!(ui.get_result_plot_index(), 1 | 2) {
            2
        } else {
            0
        });
        ui.set_flow_plane_vectors(matches!(ui.get_result_plot_index(), 2 | 3));
        ui.set_flow_plane_levels(ui.get_result_contour_levels());
        ui.set_flow_plane_edges(ui.get_post_edges());
        ui.set_flow_plane_min(ui.get_result_range_min());
        ui.set_flow_plane_max(ui.get_result_range_max());
        let mut style = match display(&ui) {
            Ok(s) => s,
            Err(e) => {
                ui.set_flow_plane_status(e.into());
                return;
            }
        };
        style.palette = ui.get_result_colormap_index().clamp(0, 3) as usize;
        style.normalized_vectors = ui.get_flow_plane_normalize();
        style.normal_markers = ui.get_flow_plane_normal_marks();
        style.vectors = matches!(ui.get_result_plot_index(), 2 | 3);
        style.vector_scale = if ui.get_result_vector_scale().trim().is_empty() {
            1.0
        } else {
            match ui.get_result_vector_scale().trim().parse::<f64>() {
                Ok(v) if v.is_finite() && v > 0.0 => v,
                _ => {
                    ui.set_flow_plane_status("Vector scale must be positive".into());
                    return;
                }
            }
        };
        style.vector_density = match ui.get_result_vector_density() {
            0 => 150,
            2 => 1200,
            _ => 600,
        };
        if let Some(p) = state
            .workbench_project
            .plane_monitors
            .get_mut(ui.get_flow_plane_index().max(0) as usize)
        {
            p.display = style;
        }
        state.mark_workbench_dirty();
        ui.set_flow_plane_render_key("".into());
        show_latest(&ui, &state);
    });
    let weak = ui.as_weak();
    let control_state = shared.clone();
    ui.on_solver_control(move |action| {
        let Some(ui)=weak.upgrade() else{return;};let mut s=control_state.borrow_mut();
        if action==6 {field_mapping::start_patch(&ui,&mut s);return;}
        if matches!(action,3|4) {
            field_mapping::start(&ui,&mut s,action==4);return;
        }
        if action==5 {
            s.post.mapping.cancel();s.log("Field mapping cancelled; initialization unchanged.");refresh_ui(&ui,&s);return;
        }
        if s.post.mapping.busy() {s.log("Wait for field mapping to finish before changing initialization.");return;}
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
            let values=[ui.get_initial_x(),ui.get_initial_y(),ui.get_initial_z(),ui.get_initial_p()];
            let parsed=values.iter().map(|v|v.trim().parse::<f64>()).collect::<Result<Vec<_>,_>>();
            let values=match parsed {Ok(v) if v.iter().all(|x|x.is_finite())=>v,_=>{s.log("Initialization requires finite velocity and pressure values.");refresh_ui(&ui,&s);return;}};
            let initial_velocity=Vec3::new(values[0],values[1],values[2]);
            s.workbench_jobs.set_initial_fields(case.mesh.id(),flursys::IncompressibleInitialConditions::uniform(initial_velocity,values[3]));
            s.workbench.initialize_solution();
            s.active_result=None;
            s.post.reset_result();s.wb_residual_history.clear();ui.set_solver_paused(false);
            let velocity=vec![initial_velocity;case.mesh.cell_count()];
            let pressure=vec![values[3];case.mesh.cell_count()];
            for plane in s.workbench_project.plane_monitors.clone() {
                if let Ok(cells)=plane_monitor::intersect(&case.mesh,&plane) {
                    if let Ok(snapshot)=plane_monitor::sample(&plane,&cells,0,&velocity,&pressure){s.post.plane_samples.insert(plane.id.clone(),snapshot);}
                }
            }
            show_latest(&ui,&s);
            s.log("Initialized the specified uniform velocity and kinematic pressure. Boundary conditions are applied when Run starts. Archived runs are retained.");
        }
        refresh_ui(&ui,&s);
    });

    let weak = ui.as_weak();
    let nav_state = shared.clone();
    ui.on_flow_plane_navigate(move |action, x, y, w, h| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let state = nav_state.borrow();
        if action == 3 {
            match current_snapshot(&ui, &state) {
                Ok(snapshot) => {
                    let camera = state.post.section_camera.borrow();
                    if plane_monitor::probe_section(&snapshot, &camera, f64::from(x), f64::from(y))
                        .is_some()
                    {
                        *state.post.section_probe.borrow_mut() =
                            Some(camera.world_at(&snapshot, f64::from(x), f64::from(y)));
                    } else {
                        *state.post.section_probe.borrow_mut() = None;
                        ui.set_flow_plane_readout("Outside sampled section".into());
                    }
                }
                Err(e) => ui.set_flow_plane_readout(e.into()),
            }
            show_latest(&ui, &state);
            return;
        }
        {
            let mut camera = state.post.section_camera.borrow_mut();
            camera.viewport = [f64::from(w).max(240.0), f64::from(h).max(160.0)];
            match action {
                0 => {
                    let [width, height] = camera.viewport;
                    camera.pan[0] += f64::from(x) / width;
                    camera.pan[1] += f64::from(y) / height;
                }
                1 => {
                    camera.zoom =
                        (camera.zoom * if y > 0.0 { 1.12 } else { 1.0 / 1.12 }).clamp(0.1, 30.0);
                }
                2 => {
                    camera.zoom = 1.0;
                    camera.pan = [0.0; 2];
                }
                _ => {}
            }
        }
        show_latest(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared_state = shared.clone();
    ui.on_flow_plane_playback(move |action| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut s = shared_state.borrow_mut();
        if action==0 && s.solving {
            ui.set_flow_plane_status("Stop the active solve before opening another recording; live monitor history remains attached to this solve.".into());
            return;
        }
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
                *s.post.section_archive.borrow_mut()=None;
                s.post.plane_frames = frame_files(&folder);
                if let Some(plane)=selected(&ui,&s) {let paths=s.post.plane_frames.clone();s.post.monitors.load(paths,plane);}
                s.post.plane_frame = 0;
                s.post.plane_playing = false;
                if s.post.plane_frames.is_empty() {
                    ui.set_flow_plane_status("No recorded frames in this folder".into());
                    return;
                }
            }
            6 | 7 => {
                if s.post.plane_statistics_job.is_some() {
                    ui.set_flow_plane_status("A time-statistics export is already running".into());return;
                }
                if s.post.plane_frames.len() < 2 {
                    ui.set_flow_plane_status("Open a physical-time recording with at least two frames".into());return;
                }
                let (first, last) = playback_bounds(&ui, s.post.plane_frames.len());
                let paths = s.post.plane_frames[first..=last].to_vec();
                let point=*s.post.section_probe.borrow();
                if action==7 && point.is_none() {ui.set_flow_plane_status("Click a point on the section first".into());return;}
                let Some(target) = rfd::FileDialog::new().set_file_name(if action==7 {"plane-probe-history.csv"}else{"plane-time-statistics.csv"}).save_file() else {return;};
                let (tx, rx) = std::sync::mpsc::channel();
                s.post.plane_statistics_job = Some(rx);
                std::thread::spawn(move || {
                    let result = if action==7 {flursys::workbench::plane_statistics::probe_history_csv(&paths,point.expect("validated probe"))}
                        else {flursys::workbench::plane_statistics::temporal_csv(&paths)};
                    let result = result.and_then(|text| result_analysis::atomic_text(&target, &text))
                        .map(|()| format!("Time statistics saved: {}", target.display()));
                    let _ = tx.send(result);
                });
                ui.set_flow_plane_status("Computing physical-time means, RMS and velocity covariances…".into());
                return;
            }
            5 => {
                if s.post.plane_range_job.is_some() {
                    ui.set_flow_plane_status("Colour-range calculation is already running".into());return;
                }
                let Some(plane) = selected(&ui, &s) else {return;};
                let paths = s.post.plane_frames.clone();
                let (tx, rx) = std::sync::mpsc::channel();
                s.post.plane_range_job = Some(rx);
                std::thread::spawn(move || {
                    let result = flursys::workbench::plane_statistics::recorded_range(&paths, plane.display.field)
                        .map(|range| (plane.id, plane.display.field, range));
                    let _ = tx.send(result);
                });
                ui.set_flow_plane_status("Scanning recording colour range in background…".into());
                return;
            }
            1 => {
                if s.post.plane_frames.is_empty() {refresh_selected(&ui,&mut s);}
                if s.post.plane_frames.is_empty() {ui.set_flow_plane_status("No recording linked to this run/plane. Use Open recording to choose its plane folder.".into());return;}
                let (first, last) = playback_bounds(&ui, s.post.plane_frames.len());
                if s.post.plane_frame < first || s.post.plane_frame >= last {
                    s.post.plane_frame = first;
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
        if selected(&ui, &state)
            .is_some_and(|p| p.display.field != ui.get_flow_plane_field().max(0) as usize)
        {
            ui.set_flow_plane_min("".into());
            ui.set_flow_plane_max("".into());
            ui.set_result_range_min("".into());
            ui.set_result_range_max("".into());
        }
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
    let profile_state = shared.clone();
    ui.on_flow_plane_profile(move |a, b, c, d, count, kind| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let state = profile_state.borrow();
        let result = (|| -> Result<(), String> {
            let parse = |s: SharedString| {
                s.trim()
                    .parse::<f64>()
                    .ok()
                    .filter(|v| v.is_finite())
                    .ok_or_else(|| {
                        "Enter finite profile coordinates in plane u/v metres".to_string()
                    })
            };
            let start = [parse(a)?, parse(b)?];
            let end = [parse(c)?, parse(d)?];
            let snapshot = current_snapshot(&ui, &state)?;
            let field = selected(&ui, &state).ok_or("Select a plane")?.display.field;
            let (text, extension) = if kind == 1 {
                (
                    flursys::workbench::plane_statistics::profile_svg(
                        &snapshot,
                        start,
                        end,
                        count.max(0) as usize,
                        field,
                    )?,
                    "svg",
                )
            } else {
                (
                    flursys::workbench::plane_statistics::profile_csv(
                        &snapshot,
                        start,
                        end,
                        count.max(0) as usize,
                        field,
                    )?,
                    "csv",
                )
            };
            let Some(path) = rfd::FileDialog::new()
                .set_file_name(format!(
                    "plane-profile-iteration-{}.{extension}",
                    snapshot.iteration
                ))
                .add_filter("Profile", &[extension])
                .save_file()
            else {
                return Ok(());
            };
            result_analysis::atomic_text(&path, &text)?;
            ui.set_flow_plane_status(format!("Profile saved: {}", path.display()).into());
            Ok(())
        })();
        if let Err(e) = result {
            ui.set_flow_plane_status(e.into());
        }
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
    let section_state = shared.clone();
    ui.on_result_section_selected(move |index| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = section_state.borrow_mut();
        state.post.playing = false;
        state.post.plane_playing = false;
        state.animation_playing = false;
        ui.set_post_playing(false);
        ui.set_result_slice_axis(index);
        if index >= 4 {
            if let Some(plane_index) = ui
                .get_result_section_plane_indices()
                .row_data((index - 4) as usize)
            {
                ui.set_flow_plane_index(plane_index);
            } else {
                ui.set_result_slice_axis(0);
                return;
            }
            load(&ui, &state);
            load_display(&ui, &state);
            refresh_selected(&ui, &mut state);
        }
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_select(move |index| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        ui.set_flow_plane_index(index);
        ui.set_flow_plane_readout("".into());
        let mut state = state.borrow_mut();
        load(&ui, &state);
        refresh_selected(&ui, &mut state);
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_manage(move |action| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = state.borrow_mut();
        if state.solving || state.meshing {
            return;
        }
        if action == 0 {
            for p in &mut state.workbench_project.plane_monitors {
                p.enabled = false;
            }
        } else if action == 1 {
            let Some(p) = state
                .workbench_project
                .plane_monitors
                .get_mut(ui.get_flow_plane_index().max(0) as usize)
            else {
                return;
            };
            p.enabled = false;
            p.every = 10;
            p.display = Default::default();
        } else {
            return;
        }
        state.mark_workbench_dirty();
        load(&ui, &state);
        ui.set_flow_plane_status(
            "Monitor settings updated; existing recorded files were retained.".into(),
        );
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_apply(move||{let Some(ui)=weak.upgrade()else{return;};let mut s=state.borrow_mut();if s.solving{return;}
        let Some(mut plane)=selected(&ui,&s)else{return;};plane.every=ui.get_flow_plane_every().max(1) as usize;plane.enabled=ui.get_flow_plane_enabled();
        if let Err(e)=plane.validate(){ui.set_flow_plane_status(e.into());return;}
        if let Some(p)=s.workbench_project.plane_monitors.iter_mut().find(|p|p.id==plane.id){*p=plane.clone();}
        s.mark_workbench_dirty();ui.set_flow_plane_status(format!("{}: {}. Steady mode samples completed SIMPLE iterations; transient mode samples accepted physical time steps.",plane.name,if plane.enabled{format!("record every {} iterations",plane.every)}else{"recording disabled".into()}).into());
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_flow_plane_view(move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut s = state.borrow_mut();
        refresh_selected(&ui, &mut s);
        if s.current_step == 3 {
            ui.set_solution_show_plane(true);
        } else {
            refresh_locations(&ui, &s);
            if let Some(i) = ui
                .get_result_section_plane_indices()
                .iter()
                .position(|p| p == ui.get_flow_plane_index())
            {
                ui.set_result_slice_axis(4 + i as i32);
            }
            load_display(&ui, &s);
            ui.set_post_tab(0);
            apply_workflow_step(&ui, &mut s, 4);
        }
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
