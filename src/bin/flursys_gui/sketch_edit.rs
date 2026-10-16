//! Local-plane sketch edit drafts. Pointer motion updates parameters only;
//! exact geometry and dependent solids are rebuilt by the CAD worker on Preview.
use super::*;
use flursys::{CadSketchId, CadSketchProfile, GeometryTopology};

type Parameters = (f64, f64, CadSketchProfile, flursys::CadSketchConstraints);

pub(super) struct SketchEdit {
    id: CadSketchId,
    source: GeometryTopology,
    workspace: Option<PathBuf>,
    project_id: String,
    host: Option<flursys::FaceId>,
    plane: CadSketchPlane,
    x: f64,
    y: f64,
    profile: CadSketchProfile,
    constraints: flursys::CadSketchConstraints,
    handle: Option<usize>,
    drag_start: Option<Parameters>,
    undo: Vec<Parameters>,
    redo: Vec<Parameters>,
}

fn parameters(d: &SketchEdit) -> Parameters {
    (d.x, d.y, d.profile.clone(), d.constraints.clone())
}
fn remember(d: &mut SketchEdit, before: Parameters) {
    if before != parameters(d) {
        if d.undo.len() == 64 {
            d.undo.remove(0);
        }
        d.undo.push(before);
        d.redo.clear();
    }
}
fn finish_drag(d: &mut SketchEdit) {
    if let Some(before) = d.drag_start.take() {
        remember(d, before);
    }
    d.handle = None;
}
pub(super) fn undo(ui: &MainWindow, state: &mut AppState, redo: bool) -> bool {
    if valid(state).is_none() {
        return false;
    }
    if state.cad_job.is_some() || cad::preview(state).is_some() {
        return true;
    }
    let d = state.sketch_edit.as_mut().unwrap();
    finish_drag(d);
    let item = if redo { d.redo.pop() } else { d.undo.pop() };
    if let Some((x, y, profile, constraints)) = item {
        let current = parameters(d);
        if redo {
            d.undo.push(current);
        } else {
            d.redo.push(current);
        }
        d.x = x;
        d.y = y;
        d.profile = profile;
        d.constraints = constraints;
        sync_parameters(ui, d);
    }
    true
}
pub(super) fn fit(state: &mut AppState) -> bool {
    let Some(d) = valid(state) else { return false };
    let bounds = match d.profile {
        CadSketchProfile::Circle { radius } => {
            (d.x - radius, d.y - radius, d.x + radius, d.y + radius)
        }
        CadSketchProfile::Rectangle { width, height } => (d.x, d.y, d.x + width, d.y + height),
    };
    state.geometry_editor.transform.fit(Some(bounds));
    state.geometry_editor.transform.pixels_per_unit =
        (state.geometry_editor.transform.pixels_per_unit * 0.75).max(ViewTransform::MIN_ZOOM);
    true
}

pub(super) fn valid(state: &AppState) -> Option<&SketchEdit> {
    state.sketch_edit.as_ref().filter(|d| {
        d.source == *state.workbench.geometry()
            && d.workspace == state.workspace_path
            && d.project_id == state.workbench_project.project_id
    })
}

pub(super) fn begin(ui: &MainWindow, state: &mut AppState) -> Result<String, String> {
    let g = state.workbench.geometry();
    let s = g
        .sketches()
        .nth(ui.get_cad_sketch_index().max(0) as usize)
        .ok_or("Select a sketch")?;
    let (x, y, profile) = g.cad_sketch_parameters(s.id).map_err(|e| e.to_string())?;
    super::cad_constraints::load(ui, g, s.id);
    ui.set_cad_region_owner("".into());
    let (w, h) = match profile {
        CadSketchProfile::Circle { radius } => (radius * 2.0, radius * 2.0),
        CadSketchProfile::Rectangle { width, height } => (width, height),
    };
    let bounds = match profile {
        CadSketchProfile::Circle { radius } => (x - radius, y - radius, x + radius, y + radius),
        _ => (x, y, x + w, y + h),
    };
    let mut draft = SketchEdit {
        id: s.id,
        source: g.clone(),
        workspace: state.workspace_path.clone(),
        project_id: state.workbench_project.project_id.clone(),
        host: s.host_face,
        plane: s.plane,
        x,
        y,
        profile,
        constraints: s.constraints.clone(),
        handle: None,
        drag_start: None,
        undo: Vec::new(),
        redo: Vec::new(),
    };
    state.geometry_editor.cancel();
    state.geometry_editor.transform.fit(Some(bounds));
    state.geometry_editor.transform.pixels_per_unit =
        (state.geometry_editor.transform.pixels_per_unit * 0.75).max(ViewTransform::MIN_ZOOM);
    state.face_sketch_host = None;
    state.cad_measure_points.clear();
    state.show_geometry_3d = false;
    state.show_mesh = false;
    ui.set_geometry_cad_view_3d(false);
    ui.set_cad_measure_active(false);
    ui.set_cad_sketch_offset(super::cad_units::display_length(
        ui,
        draft.source.sketch(draft.id).unwrap().attachment_offset,
    ));
    ui.set_cad_edit_sketch_id(draft.id.get() as i32);
    ui.set_cad_sketch_kind(
        if matches!(draft.profile, CadSketchProfile::Circle { .. }) {
            0
        } else {
            1
        },
    );
    ui.set_cad_sketch_plane(match draft.plane {
        CadSketchPlane::Xy => 0,
        CadSketchPlane::Yz => 1,
        CadSketchPlane::Zx => 2,
        CadSketchPlane::Face => 3,
    });
    ui.set_cad_edit_body_id("".into());
    ui.set_cad_sketch_host("".into());
    ui.set_cad_sketch_editing(true);
    ui.set_cad_task_open(true);
    ui.set_geometry_sidebar_tab(0);
    ui.set_cad_workbench(0);
    ui.set_cad_operation(212);
    ui.set_cad_task_title("Edit sketch".into());
    sync_parameters(ui, &mut draft);
    state.sketch_edit = Some(draft);
    Ok("Drag the origin handle to move, or resize using the other handles. Preview rebuilds the solids; Cancel preserves the source model.".into())
}

pub(super) fn cancel(ui: &MainWindow, state: &mut AppState) {
    state.sketch_edit = None;
    state.cad_preview = None;
    state.geometry_editor.cancel();
    state.show_geometry_3d = true;
    ui.set_geometry_cad_view_3d(true);
    ui.set_cad_sketch_editing(false);
    ui.set_cad_edit_sketch_id(0);
}

fn handles(d: &SketchEdit) -> Vec<(f64, f64)> {
    match d.profile {
        CadSketchProfile::Circle { radius } => vec![(d.x, d.y), (d.x + radius, d.y)],
        CadSketchProfile::Rectangle { width, height } => vec![
            (d.x, d.y),
            (d.x + width, d.y + height),
            (d.x + width, d.y + height * 0.5),
            (d.x + width * 0.5, d.y + height),
        ],
    }
}

fn sync_parameters(ui: &MainWindow, d: &mut SketchEdit) {
    if let Ok((x, y, profile)) = d.source.cad_resolve_sketch_parameters_using(
        d.id,
        d.x,
        d.y,
        d.profile.clone(),
        Some(&d.constraints),
    ) {
        d.x = x;
        d.y = y;
        d.profile = profile;
    }
    super::cad_constraints::show(ui, &d.constraints);
    let dof = d
        .source
        .cad_sketch_degrees_of_freedom_using(d.id, Some(&d.constraints))
        .unwrap_or(0);
    ui.set_cad_constraint_status(
        format!(
            "{dof} DOF · {}",
            if dof == 0 {
                "fully constrained"
            } else {
                "under constrained"
            }
        )
        .into(),
    );
    ui.set_cad_pos_x(super::cad_units::display_length(ui, d.x));
    ui.set_cad_pos_y(super::cad_units::display_length(ui, d.y));
    match d.profile {
        CadSketchProfile::Circle { radius } => {
            ui.set_cad_size_x(super::cad_units::display_length(ui, radius))
        }
        CadSketchProfile::Rectangle { width, height } => {
            ui.set_cad_size_x(super::cad_units::display_length(ui, width));
            ui.set_cad_size_y(super::cad_units::display_length(ui, height));
        }
    }
}

pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_cad_sketch_drag(move |kind, x, y, w, h| {
        let Some(ui) = weak.upgrade() else { return };
        let screen = preview_image_point(x, y, w, h);
        if screen.is_none() {
            if kind == 2 {
                let mut state = state.borrow_mut();
                if let Some(d) = state.sketch_edit.as_mut() {
                    finish_drag(d);
                }
                refresh_ui(&ui, &state);
            }
            return;
        }
        let screen = screen.unwrap();
        let mut state = state.borrow_mut();
        if state.cad_job.is_some() || cad::preview(&state).is_some() || valid(&state).is_none() {
            return;
        }
        let transform = state.geometry_editor.transform;
        let grid = state.geometry_editor.grid_spacing();
        let snap = state.geometry_editor.snap_enabled;
        let d = state.sketch_edit.as_mut().unwrap();
        if kind == 0 {
            finish_drag(d);
            d.handle = handles(d)
                .iter()
                .enumerate()
                .filter_map(|(i, p)| {
                    let p = transform.world_to_screen(*p);
                    let distance = (screen.0 - p.0).hypot(screen.1 - p.1);
                    (distance <= 12.0).then_some((i, distance))
                })
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|p| p.0);
            if d.handle.is_some() {
                d.drag_start = Some(parameters(d));
            }
        } else if let Some(handle) = d.handle {
            let (mut x, mut y) = transform.screen_to_world(screen);
            if snap {
                x = (x / grid).round() * grid;
                y = (y / grid).round() * grid;
            }
            let minimum = 1e-8;
            if handle == 0 {
                d.x = x;
                d.y = y;
            } else {
                d.profile = match d.profile {
                    CadSketchProfile::Circle { .. } => CadSketchProfile::Circle {
                        radius: (x - d.x)
                            .hypot(if d.constraints.centered {
                                2.0 * y.abs()
                            } else {
                                y - d.y
                            })
                            .max(minimum),
                    },
                    CadSketchProfile::Rectangle { width, height } => CadSketchProfile::Rectangle {
                        width: if handle == 1 || handle == 2 {
                            (if d.constraints.centered {
                                2.0 * x.abs()
                            } else {
                                x - d.x
                            })
                            .max(minimum)
                        } else {
                            width
                        },
                        height: if handle == 1 || handle == 3 {
                            (y - d.y).max(minimum)
                        } else {
                            height
                        },
                    },
                };
            }
            sync_parameters(&ui, d);
        }
        if kind == 2 {
            finish_drag(d);
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let state = shared.clone();
    ui.on_cad_sketch_parameters_edited(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = state.borrow_mut();
        if valid(&state).is_none() || state.cad_job.is_some() || cad::preview(&state).is_some() {
            return;
        }
        let parse = |s: SharedString| super::cad_units::length(&ui, s, "Sketch dimension").ok();
        let Some((x, y, a)) = parse(ui.get_cad_pos_x())
            .zip(parse(ui.get_cad_pos_y()))
            .zip(parse(ui.get_cad_size_x()))
            .map(|((x, y), a)| (x, y, a))
            .filter(|(_, _, a)| *a > 1e-8)
        else {
            return;
        };
        let d = state.sketch_edit.as_mut().unwrap();
        let profile = match d.profile {
            CadSketchProfile::Circle { .. } => CadSketchProfile::Circle { radius: a },
            CadSketchProfile::Rectangle { .. } => {
                let Some(height) = parse(ui.get_cad_size_y()).filter(|v| *v > 1e-8) else {
                    return;
                };
                CadSketchProfile::Rectangle { width: a, height }
            }
        };
        let constraints = super::cad_constraints::read(
            &ui,
            x,
            y,
            a,
            match &profile {
                CadSketchProfile::Rectangle { height, .. } => *height,
                _ => 0.0,
            },
        );
        let resolved =
            d.source
                .cad_resolve_sketch_parameters_using(d.id, x, y, profile, Some(&constraints));
        let (x, y, profile) = match resolved {
            Ok(v) => v,
            Err(e) => {
                ui.set_cad_feedback(e.to_string().into());
                return;
            }
        };
        let before = parameters(d);
        d.x = x;
        d.y = y;
        d.profile = profile;
        d.constraints = constraints;
        sync_parameters(&ui, d);
        remember(d, before);
        refresh_ui(&ui, &state);
    });
}

pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    let draft = valid(state);
    ui.set_cad_direct_sketch_editing(draft.is_some());
    let mut dimensions = Vec::new();
    if let Some(d) = draft {
        let project = |p| state.geometry_editor.transform.world_to_screen(p);
        match d.profile {
            CadSketchProfile::Circle { radius } => {
                let p = project((d.x + radius * 0.5, d.y));
                dimensions.push(SketchDimension {
                    text: format!(
                        "R {} {}",
                        super::cad_units::display_length(ui, radius),
                        super::cad_units::length_symbol(ui)
                    )
                    .into(),
                    x: p.0 as f32,
                    y: (p.1 - 16.0) as f32,
                });
            }
            CadSketchProfile::Rectangle { width, height } => {
                let a = project((d.x + width * 0.5, d.y));
                let b = project((d.x + width, d.y + height * 0.5));
                dimensions.push(SketchDimension {
                    text: format!(
                        "W {} {}",
                        super::cad_units::display_length(ui, width),
                        super::cad_units::length_symbol(ui)
                    )
                    .into(),
                    x: a.0 as f32,
                    y: (a.1 + 30.0) as f32,
                });
                dimensions.push(SketchDimension {
                    text: format!(
                        "H {} {}",
                        super::cad_units::display_length(ui, height),
                        super::cad_units::length_symbol(ui)
                    )
                    .into(),
                    x: (b.0 + 40.0) as f32,
                    y: b.1 as f32,
                });
            }
        }
    }
    ui.set_cad_sketch_dimensions(ModelRc::new(VecModel::from(dimensions)));

    if state.sketch_edit.is_some() && draft.is_none() {
        ui.set_cad_sketch_editing(false);
        ui.set_cad_edit_sketch_id(0);
    }

    if let Some(d) = draft {
        ui.set_cad_sketch_plane_label(
            match d.plane {
                CadSketchPlane::Xy => "XY",
                CadSketchPlane::Yz => "YZ",
                CadSketchPlane::Zx => "ZX",
                CadSketchPlane::Face => "Face plane",
            }
            .into(),
        );
        ui.set_cad_sketch_edit_status(
            format!(
                "Sketch {} · {} DOF",
                d.id.get(),
                d.source
                    .cad_sketch_degrees_of_freedom_using(d.id, Some(&d.constraints))
                    .unwrap_or(0)
            )
            .into(),
        );
    }
}

pub(super) fn render(state: &AppState) -> Option<slint::Image> {
    let d = valid(state)?;
    let editor = &state.geometry_editor;
    let (width, height) = (PREVIEW_WIDTH, PREVIEW_HEIGHT);
    let mut pixels = if let Some(host) = d.host {
        super::render::render_face_sketch(state.workbench.geometry(), host, editor, None, None)
            .to_rgba8()?
            .as_bytes()
            .to_vec()
    } else {
        let mut pixels = vec![0u8; (width * height * 4) as usize];
        super::render::fill(&mut pixels, [9, 16, 22, 255]);
        let grid = editor.grid_spacing();
        let (left, bottom) = editor.transform.screen_to_world((0.0, height as f64));
        let (right, top) = editor.transform.screen_to_world((width as f64, 0.0));
        if editor.grid_enabled {
            for i in (left / grid).floor() as i32..=(right / grid).ceil() as i32 {
                let x = editor.transform.world_to_screen((i as f64 * grid, 0.0)).0 as i32;
                super::render::draw_line(
                    &mut pixels,
                    width,
                    height,
                    (x, 0),
                    (x, height as i32),
                    [25, 44, 55, 255],
                );
            }
            for i in (bottom / grid).floor() as i32..=(top / grid).ceil() as i32 {
                let y = editor.transform.world_to_screen((0.0, i as f64 * grid)).1 as i32;
                super::render::draw_line(
                    &mut pixels,
                    width,
                    height,
                    (0, y),
                    (width as i32, y),
                    [25, 44, 55, 255],
                );
            }
        }
        pixels
    };
    let project = |p| {
        let p = editor.transform.world_to_screen(p);
        (p.0 as i32, p.1 as i32)
    };
    let origin = project((0.0, 0.0));
    super::render::draw_line(
        &mut pixels,
        width,
        height,
        (0, origin.1),
        (width as i32, origin.1),
        [125, 63, 66, 255],
    );
    super::render::draw_line(
        &mut pixels,
        width,
        height,
        (origin.0, 0),
        (origin.0, height as i32),
        [54, 112, 88, 255],
    );
    let points = match d.profile {
        CadSketchProfile::Circle { radius } => (0..128)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 128.0;
                (d.x + radius * a.cos(), d.y + radius * a.sin())
            })
            .collect::<Vec<_>>(),
        CadSketchProfile::Rectangle { width, height } => vec![
            (d.x, d.y),
            (d.x + width, d.y),
            (d.x + width, d.y + height),
            (d.x, d.y + height),
        ],
    };
    for i in 0..points.len() {
        super::render::draw_sketch_line(
            &mut pixels,
            width,
            height,
            project(points[i]),
            project(points[(i + 1) % points.len()]),
            [255, 211, 119, 255],
        );
    }
    let color = [125, 161, 180, 255];
    match d.profile {
        CadSketchProfile::Circle { radius } => super::render::draw_line(
            &mut pixels,
            width,
            height,
            project((d.x, d.y)),
            project((d.x + radius, d.y)),
            color,
        ),
        CadSketchProfile::Rectangle {
            width: w,
            height: h,
        } => {
            let a = project((d.x, d.y));
            let b = project((d.x + w, d.y));
            let c = project((d.x + w, d.y + h));
            super::render::draw_line(
                &mut pixels,
                width,
                height,
                (a.0, a.1.saturating_add(18)),
                (b.0, b.1.saturating_add(18)),
                color,
            );
            super::render::draw_line(
                &mut pixels,
                width,
                height,
                (b.0.saturating_add(18), b.1),
                (c.0.saturating_add(18), c.1),
                color,
            );
            for p in [a, b] {
                super::render::draw_line(
                    &mut pixels,
                    width,
                    height,
                    p,
                    (p.0, p.1.saturating_add(23)),
                    color,
                );
            }
            for p in [b, c] {
                super::render::draw_line(
                    &mut pixels,
                    width,
                    height,
                    p,
                    (p.0.saturating_add(23), p.1),
                    color,
                );
            }
        }
    }
    for (i, p) in handles(d).into_iter().enumerate() {
        let p = project(p);
        let color = if d.handle == Some(i) {
            [136, 239, 192, 255]
        } else {
            [255, 222, 157, 255]
        };
        for x in -4i32..=4 {
            for y in -4i32..=4 {
                if x.abs() == 4 || y.abs() == 4 {
                    let (px, py) = (p.0 as i64 + x as i64, p.1 as i64 + y as i64);
                    if px >= 0 && py >= 0 && px < width as i64 && py < height as i64 {
                        super::render::set_pixel(
                            &mut pixels,
                            width,
                            height,
                            px as u32,
                            py as u32,
                            color,
                        );
                    }
                }
            }
        }
    }
    Some(super::render::image_from_rgba(width, height, pixels))
}
