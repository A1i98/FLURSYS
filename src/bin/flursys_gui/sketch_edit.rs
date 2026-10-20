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
