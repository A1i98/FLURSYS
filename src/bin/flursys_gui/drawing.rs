//! Interactive sketch drafting over the complete imported constraint model.
//! Drafts are isolated; B-rep publication runs through the existing CAD worker.
use super::*;
use flursys::sketching::{self, core, geom::Vec2, ToolParameters};
use flursys::GeometryTopology;

pub(super) struct Draft {
    source: GeometryTopology,
    workspace: Option<PathBuf>,
    project: String,
    pub id: Option<flursys::CadSketchId>,
    plane: CadSketchPlane,
    host: Option<flursys::FaceId>,
    pub model: core::Sketch,
    view: ViewTransform,
    tool: i32,
    pending: Vec<Vec2>,
    cursor: Vec2,
    curves: Vec<usize>,
    points: Vec<usize>,
    undo: Vec<core::Sketch>,
    redo: Vec<core::Sketch>,
    drag: Option<(usize, core::Sketch)>,
    pub report: core::SolveReport,
    feedback: String,
}
fn valid(state: &AppState) -> Option<&Draft> {
    state.drawing.as_ref().filter(|d| {
        d.source == *state.workbench.geometry()
            && d.workspace == state.workspace_path
            && d.project == state.workbench_project.project_id
    })
}

pub(super) fn begin(
    ui: &MainWindow,
    state: &mut AppState,
    face: bool,
    existing: bool,
) -> Result<String, String> {
    if state.cad_job.is_some() || cad::preview(state).is_some() {
        return Err("Finish the current CAD operation first".into());
    }
    let g = state.workbench.geometry();
    let record = if existing {
        let s = g
            .sketches()
            .nth(ui.get_cad_sketch_index().max(0) as usize)
            .ok_or("Select a sketch")?;
        let owner = g.cad_sketch_owner(s.id).map_err(|e| e.to_string())?;
        Some(g.sketch(owner).ok_or("Missing sketch owner")?.clone())
    } else {
        None
    };
    let host = if let Some(s) = &record {
        s.host_face
    } else if face {
        Some(
            state
                .wb_selected_targets
                .iter()
                .find_map(|t| {
                    if let GeometrySelectionTarget::Face(f) = t {
                        Some(*f)
                    } else {
                        None
                    }
                })
                .ok_or("Select a planar face")?,
        )
    } else {
        None
    };
    if let Some(f) = host {
        g.sketch_frame_for_face(f).map_err(|e| e.to_string())?;
    }
    let plane = record
        .as_ref()
        .map(|s| s.plane)
        .unwrap_or(if host.is_some() {
            CadSketchPlane::Face
        } else {
            match ui.get_cad_sketch_plane() {
                1 => CadSketchPlane::Yz,
                2 => CadSketchPlane::Zx,
                _ => CadSketchPlane::Xy,
            }
        });
    let mut model = record
        .as_ref()
        .and_then(|s| s.drawing.clone())
        .unwrap_or_default();
    if let Some(s) = &record {
        if s.drawing.is_none() {
            let (x, y, p) = g.cad_sketch_parameters(s.id).map_err(|e| e.to_string())?;
            match p {
                flursys::CadSketchProfile::Rectangle { width, height } => sketching::draw(
                    &mut model,
                    3,
                    &[
                        Vec2::new(x * 1000.0, y * 1000.0),
                        Vec2::new((x + width) * 1000.0, (y + height) * 1000.0),
                    ],
                    &ToolParameters::default(),
                )?,
                flursys::CadSketchProfile::Circle { radius } => sketching::draw(
                    &mut model,
                    6,
                    &[
                        Vec2::new(x * 1000.0, y * 1000.0),
                        Vec2::new((x + radius) * 1000.0, y * 1000.0),
                    ],
                    &ToolParameters::default(),
                )?,
            }
            if s.constraints != flursys::CadSketchConstraints::default()
                || s.locked_position.is_some()
                || s.locked_profile.is_some()
            {
                return Err("Edit this legacy sketch with its existing dimension controls; conversion must preserve its saved constraints".into());
            }
        }
    }
    let report = core::solve(&mut model);
    let mut view = ViewTransform::default();
    view.set_viewport(PREVIEW_WIDTH as f64, PREVIEW_HEIGHT as f64);
    let bounds = model.bounds();
    view.fit(
        bounds
            .map(|(a, b)| (a.x, a.y, b.x, b.y))
            .or(Some((-10.0, -10.0, 30.0, 20.0))),
    );
    state.drawing = Some(Draft {
        source: g.clone(),
        workspace: state.workspace_path.clone(),
        project: state.workbench_project.project_id.clone(),
        id: record.map(|s| s.id),
        plane,
        host,
        model,
        view,
        tool: 0,
        pending: Vec::new(),
        cursor: Vec2::ZERO,
        curves: Vec::new(),
        points: Vec::new(),
        undo: Vec::new(),
        redo: Vec::new(),
        drag: None,
        report,
        feedback:
            "Choose a drawing tool. Ctrl-click selects multiple entities; Enter finishes a chain."
                .into(),
    });
    ui.set_cad_task_open(true);
    ui.set_cad_operation(220);
    ui.set_cad_task_title("Parametric sketch".into());
    ui.set_drawing_tool(0);
    state.show_geometry_3d = false;
    state.show_mesh = false;
    Ok(
        "Parametric sketch editor opened. Finish publishes the sketch and its closed regions."
            .into(),
    )
}
fn params(ui: &MainWindow) -> Result<ToolParameters, String> {
    let number = |s: SharedString| {
        s.trim()
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| "Enter finite numeric parameters".to_string())
    };
    Ok(ToolParameters {
        value: number(ui.get_drawing_value())?,
        secondary: number(ui.get_drawing_secondary())?,
        count: ui.get_drawing_count().clamp(2, 128) as usize,
        text: ui.get_drawing_text().to_string(),
    })
}
fn remember(d: &mut Draft, before: core::Sketch) {
    if before != d.model {
        if d.undo.len() == 64 {
            d.undo.remove(0);
        }
        d.undo.push(before);
        d.redo.clear();
    }
}
fn transact(d: &mut Draft, f: impl FnOnce(&mut core::Sketch) -> Result<(), String>) {
    let old = d.model.clone();
    match sketching::transaction(&mut d.model, f) {
        Ok(report) => {
            d.report = report;
            remember(d, old);
            d.feedback = "Sketch updated".into();
        }
        Err(e) => d.feedback = e,
    }
}
fn commit_chain(d: &mut Draft, p: &ToolParameters) {
    let pts = d.pending.clone();
    let tool = d.tool;
    transact(d, |sk| sketching::draw(sk, tool, &pts, p));
    if d.feedback == "Sketch updated" {
        d.pending.clear();
    }
}
fn screen_world(d: &Draft, x: f32, y: f32, w: f32, h: f32) -> Option<Vec2> {
    let point = render::preview_image_point(x, y, w, h)?;
    let p = d.view.screen_to_world(point);
    Some(Vec2::new(p.0, p.1))
}
fn snapped(d: &Draft, p: Vec2, snap: bool) -> Vec2 {
    if !snap {
        return p;
    }
    let tol = 8.0 / d.view.pixels_per_unit;
    d.model
        .points
        .iter()
        .map(|q| q.pos)
        .chain(d.pending.iter().copied())
        .filter(|q| q.dist(p) < tol)
        .min_by(|a, b| a.dist(p).total_cmp(&b.dist(p)))
        .unwrap_or(p)
}
fn curve_path(sk: &core::Sketch, index: usize, tolerance: f64) -> Vec<Vec2> {
    sk.exact_segs(index)
        .map(|segs| segs.iter().flat_map(|s| s.polyline(tolerance)).collect())
        .unwrap_or_else(|| sk.polyline(index))
}
fn pick(d: &mut Draft, p: Vec2, add: bool) {
    let tol = 8.0 / d.view.pixels_per_unit;
    if let Some(i) = d
        .model
        .points
        .iter()
        .enumerate()
        .filter(|(_, q)| q.pos.dist(p) < tol)
        .min_by(|(_, a), (_, b)| a.pos.dist(p).total_cmp(&b.pos.dist(p)))
        .map(|(i, _)| i)
    {
        if !add {
            d.points.clear();
            d.curves.clear();
        }
        if let Some(j) = d.points.iter().position(|q| *q == i) {
            d.points.remove(j);
        } else {
            d.points.push(i);
        }
        return;
    }
    let nearest = (0..d.model.curves.len())
        .map(|i| {
            let distance = curve_path(&d.model, i, 0.35 / d.view.pixels_per_unit)
                .windows(2)
                .map(|s| {
                    render::distance_to_segment((p.x, p.y), (s[0].x, s[0].y), (s[1].x, s[1].y))
                })
                .fold(f64::INFINITY, f64::min);
            (i, distance)
        })
        .filter(|(_, distance)| *distance < tol)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(i, _)| i);
    if !add {
        d.curves.clear();
        d.points.clear();
    }
    if let Some(i) = nearest {
        if let Some(j) = d.curves.iter().position(|q| *q == i) {
            d.curves.remove(j);
        } else {
            d.curves.push(i);
        }
    }
}

pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_drawing_change_support(move |index| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if valid(&state).is_none() || state.cad_job.is_some() {
            return;
        }
        if let Some(d) = state.drawing.as_mut() {
            if d.id.is_none() && d.host.is_none() {
                d.plane = match index {
                    1 => CadSketchPlane::Yz,
                    2 => CadSketchPlane::Zx,
                    _ => CadSketchPlane::Xy,
                };
            }
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_drawing_select_tool(move |tool| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if valid(&state).is_none() || state.cad_job.is_some() {
            return;
        }
        if let Some(d) = state.drawing.as_mut() {
            d.tool = tool.clamp(0, 18);
            d.pending.clear();
            if let Some((_, before)) = d.drag.take() {
                d.model = before;
            }
            if tool == 15 {
                ui.set_drawing_value("0.5".into());
            }
            d.feedback = match d.tool {
                2 | 11 | 12 => {
                    "Click points; Enter finishes the chain; Escape cancels the current chain"
                        .into()
                }
                5 | 7 | 8 | 9 | 10 | 15 => "Click three defining points".into(),
                16 | 17 => "Click the insertion point".into(),
                0 => "Click a curve or point. Ctrl-click adds or removes. Drag a point to edit."
                    .into(),
                _ => "Click two defining points".into(),
            };
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_drawing_pointer(move |kind, x, y, w, h, add| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if valid(&state).is_none() || state.cad_job.is_some() {
            return;
        }
        let parameters = params(&ui);
        let d = state.drawing.as_mut().unwrap();
        let Some(raw) = screen_world(d, x, y, w, h) else {
            return;
        };
        let p = snapped(d, raw, ui.get_drawing_snap());
        d.cursor = p;
        if kind == 0 && d.tool == 0 {
            let tol = 8.0 / d.view.pixels_per_unit;
            if !add {
                if let Some(i) = d
                    .model
                    .points
                    .iter()
                    .enumerate()
                    .filter(|(_, q)| q.pos.dist(raw) < tol && !q.fixed)
                    .min_by(|(_, a), (_, b)| a.pos.dist(raw).total_cmp(&b.pos.dist(raw)))
                    .map(|(i, _)| i)
                {
                    d.drag = Some((i, d.model.clone()));
                }
            }
        }
        if kind == 1 {
            if let Some((i, _)) = &d.drag {
                let mut budget = core::Budget::frame();
                core::move_point_towards(&mut d.model, *i, raw, 10.0, Some(&mut budget), |_, _| {
                    true
                });
            }
        }
        if kind == 2 {
            if let Some((_, before)) = d.drag.take() {
                if before == d.model {
                    pick(d, raw, add);
                } else {
                    remember(d, before);
                    d.report = core::solve(&mut d.model);
                }
            } else if d.tool == 0 {
                pick(d, raw, add);
            } else {
                d.pending.push(p);
                let need = match d.tool {
                    2 | 11 | 12 => usize::MAX,
                    5 | 7 | 8 | 9 | 10 | 15 => 3,
                    16 | 17 => 1,
                    _ => 2,
                };
                if d.pending.len() >= need
                    || (d.tool == 2 && d.pending.len() > 3 && p.dist(d.pending[0]) < 1e-7)
                {
                    match parameters {
                        Ok(p) => commit_chain(d, &p),
                        Err(e) => d.feedback = e,
                    }
                }
            }
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_drawing_pan(move |dx, dy, w, h| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if valid(&state).is_none() {
            return;
        }
        let d = state.drawing.as_mut().unwrap();
        let scale = f64::from(viewport::preview_scale(w, h));
        d.view.center_x -= f64::from(dx) / scale / d.view.pixels_per_unit;
        d.view.center_y += f64::from(dy) / scale / d.view.pixels_per_unit;
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_drawing_zoom(move |delta, x, y, w, h| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if valid(&state).is_none() {
            return;
        }
        let d = state.drawing.as_mut().unwrap();
        if let Some(p) = render::preview_image_point(x, y, w, h) {
            d.view.zoom_at(p, delta as f64);
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_drawing_action(move |action| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if valid(&state).is_none() || state.cad_job.is_some() {
            return;
        }
        if action == 101 {
            state.drawing = None;
            ui.set_cad_task_open(false);
            state.show_geometry_3d = true;
            ui.set_geometry_cad_view_3d(true);
            refresh_ui(&ui, &state);
            return;
        }
        if action == 100 {
            if let Err(e) = submit(&ui, &mut state) {
                if let Some(d) = state.drawing.as_mut() {
                    d.feedback = e;
                }
            }
            refresh_ui(&ui, &state);
            return;
        }
        let d = state.drawing.as_mut().unwrap();
        match action {
            102 => match params(&ui) {
                Ok(p) => commit_chain(d, &p),
                Err(e) => d.feedback = e,
            },
            103 => {
                d.pending.clear();
                if let Some((_, before)) = d.drag.take() {
                    d.model = before;
                }
            }
            113 => {}
            104 | 105 => {
                let item = if action == 104 {
                    d.undo.pop()
                } else {
                    d.redo.pop()
                };
                if let Some(sk) = item {
                    if action == 104 {
                        d.redo.push(d.model.clone());
                    } else {
                        d.undo.push(d.model.clone());
                    }
                    d.model = sk;
                    d.curves.clear();
                    d.points.clear();
                    d.pending.clear();
                    d.report = core::solve(&mut d.model);
                }
            }
            106 => {
                d.view
                    .fit(d.model.bounds().map(|(a, b)| (a.x, a.y, b.x, b.y)));
            }
            107 => {
                d.curves = (0..d.model.curves.len()).collect();
                d.points.clear();
            }
            108 => {
                let index = ui.get_drawing_constraint_index().max(0) as usize;
                transact(d, |sk| {
                    if index >= sk.constraints.len() {
                        return Err("Select a constraint".into());
                    }
                    sk.constraints.remove(index);
                    Ok(())
                });
            }
            109 => {
                let index = ui.get_drawing_constraint_index().max(0) as usize;
                match params(&ui) {
                    Ok(p) => transact(d, |sk| {
                        let c = sk.constraints.get_mut(index).ok_or("Select a constraint")?;
                        if !c.kind.is_dimension() {
                            return Err("Select a driving dimension".into());
                        }
                        c.kind.set_value(if c.kind.is_angle() {
                            p.value.to_radians()
                        } else {
                            p.value
                        });
                        Ok(())
                    }),
                    Err(e) => d.feedback = e,
                }
            }
            110 => {
                let cs = d.curves.clone();
                let ps = d.points.clone();
                match params(&ui) {
                    Ok(p) => {
                        let kind = ui.get_drawing_constraint_kind();
                        transact(d, |sk| sketching::constraint(sk, kind, &cs, &ps, p.value));
                    }
                    Err(e) => d.feedback = e,
                }
            }
            _ => {
                let cs = d.curves.clone();
                let ps = d.points.clone();
                let at = if matches!(action, 38 | 40 | 41) {
                    Vec2::new(
                        ui.get_drawing_pivot_x().parse::<f64>().unwrap_or(f64::NAN),
                        ui.get_drawing_pivot_y().parse::<f64>().unwrap_or(f64::NAN),
                    )
                } else {
                    d.cursor
                };
                if !at.is_finite() {
                    d.feedback = "Enter a finite pivot / pattern center".into();
                    refresh_ui(&ui, &state);
                    return;
                }
                match params(&ui) {
                    Ok(p) => {
                        transact(d, |sk| sketching::edit(sk, action, &cs, &ps, at, &p));
                        if d.feedback == "Sketch updated" {
                            d.curves.clear();
                            d.points.clear();
                        }
                    }
                    Err(e) => d.feedback = e,
                }
            }
        }
        refresh_ui(&ui, &state);
    });
}
fn submit(ui: &MainWindow, state: &mut AppState) -> Result<(), String> {
    let d = valid(state).ok_or("Sketch draft no longer matches the project")?;
    if !d.pending.is_empty() {
        return Err("Finish or cancel the current drawing chain before publishing".into());
    }
    sketching::validate(&d.model)?;
    super::cad::submit_drawing(ui, state, d.id, d.plane, d.host, d.model.clone())
}

pub(super) fn sync(ui: &MainWindow, state: &AppState) {
    let Some(d) = valid(state) else {
        ui.set_drawing_active(false);
        return;
    };
    let (plane_name, horizontal, vertical) = match d.plane {
        CadSketchPlane::Xy => ("XY", "X", "Y"),
        CadSketchPlane::Yz => ("YZ", "Y", "Z"),
        CadSketchPlane::Zx => ("ZX", "Z", "X"),
        CadSketchPlane::Face => ("Face sketch", "u", "v"),
    };
    ui.set_drawing_coordinates(
        format!(
            "{horizontal}  {:.3}    {vertical}  {:.3}    mm",
            d.cursor.x, d.cursor.y
        )
        .into(),
    );
    ui.set_drawing_active(true);
    ui.set_drawing_busy(state.cad_job.is_some());
    ui.set_drawing_tool(d.tool);
    ui.set_drawing_status(
        format!(
            "{} points · {} curves · {} constraints · {} DOF · residual {:.2e}",
            d.model.points.len(),
            d.model.curves.len(),
            d.model.constraints.len(),
            d.report.dof,
            d.report.max_residual
        )
        .into(),
    );
    ui.set_drawing_feedback(d.feedback.clone().into());
    ui.set_drawing_plane(format!("{plane_name} · millimetres").into());
    ui.set_drawing_support(match d.plane {
        CadSketchPlane::Yz => 1,
        CadSketchPlane::Zx => 2,
        CadSketchPlane::Face => 3,
        _ => 0,
    });
    ui.set_drawing_can_change_support(d.id.is_none() && d.host.is_none());
    ui.set_drawing_selection(
        format!(
            "{} curves · {} points selected",
            d.curves.len(),
            d.points.len()
        )
        .into(),
    );
    ui.set_drawing_constraints(ModelRc::new(VecModel::from(
        d.model
            .constraints
            .iter()
            .map(|c| {
                SharedString::from(format!(
                    "{} · {}{}",
                    c.id,
                    c.kind.name(),
                    c.kind
                        .value()
                        .map(|v| format!(
                            " = {:.5}",
                            if c.kind.is_angle() { v.to_degrees() } else { v }
                        ))
                        .unwrap_or_default()
                ))
            })
            .collect::<Vec<_>>(),
    )));
    let dimensions = dimension_layouts(d)
        .into_iter()
        .map(|(index, layout)| {
            let p = d.view.world_to_screen((layout.text.x, layout.text.y));
            let c = &d.model.constraints[index];
            SketchDimension {
                x: p.0 as f32,
                y: p.1 as f32,
                text: {
                    let value = c.kind.value().unwrap_or(0.0);
                    if c.kind.is_angle() {
                        format!("{:.2}°", value.to_degrees())
                    } else {
                        match c.kind {
                            core::ConstraintKind::Radius { .. } => format!("R{value:.3} mm"),
                            core::ConstraintKind::Diameter { .. } => format!("⌀{value:.3} mm"),
                            _ => format!("{value:.3} mm"),
                        }
                    }
                }
                .into(),
            }
        })
        .collect::<Vec<_>>();
    ui.set_drawing_dimensions(ModelRc::new(VecModel::from(dimensions)));
    ui.set_drawing_image(render_draft(
        d,
        ui.get_drawing_grid(),
        params(ui).ok().as_ref(),
    ));
}
fn dimension_layouts(d: &Draft) -> Vec<(usize, core::DimLayout)> {
    let px = 1.0 / d.view.pixels_per_unit;
    d.model
        .constraints
        .iter()
        .enumerate()
        .filter_map(|(i, c)| {
            core::dim_frame(&d.model, &c.kind).map(|frame| {
                (
                    i,
                    core::dim_layout_reaching(
                        &frame,
                        c.text,
                        px,
                        Vec2::new(28.0 * px, 8.0 * px),
                        core::dim_reach(&d.model, &c.kind),
                    ),
                )
            })
        })
        .collect()
}
fn render_draft(d: &Draft, grid: bool, parameters: Option<&ToolParameters>) -> slint::Image {
    let (w, h) = (PREVIEW_WIDTH, PREVIEW_HEIGHT);
    let mut pixels = vec![0u8; (w * h * 4) as usize];
    render::fill(&mut pixels, [46, 52, 64, 255]);
    let screen = |p: Vec2| {
        let p = d.view.world_to_screen((p.x, p.y));
        (p.0.round() as i32, p.1.round() as i32)
    };
    if grid {
        let step = 10.0_f64.powf((40.0 / d.view.pixels_per_unit).log10().floor());
        let a = d.view.screen_to_world((0.0, h as f64));
        let b = d.view.screen_to_world((w as f64, 0.0));
        for i in (a.0 / step).floor() as i64..=(b.0 / step).ceil() as i64 {
            let x = screen(Vec2::new(i as f64 * step, 0.0)).0;
            render::draw_line(&mut pixels, w, h, (x, 0), (x, h as i32), [59, 65, 76, 255]);
        }
        for i in (a.1 / step).floor() as i64..=(b.1 / step).ceil() as i64 {
            let y = screen(Vec2::new(0.0, i as f64 * step)).1;
            render::draw_line(&mut pixels, w, h, (0, y), (w as i32, y), [59, 65, 76, 255]);
        }
    }
    let origin = screen(Vec2::ZERO);
    render::draw_line(
        &mut pixels,
        w,
        h,
        (0, origin.1),
        (w as i32, origin.1),
        [116, 63, 66, 255],
    );
    render::draw_line(
        &mut pixels,
        w,
        h,
        (origin.0, 0),
        (origin.0, h as i32),
        [54, 100, 77, 255],
    );
    for (i, c) in d.model.curves.iter().enumerate() {
        let color = if d.curves.contains(&i) {
            [255, 205, 105, 255]
        } else if c.construction || c.centerline {
            [112, 151, 190, 255]
        } else if d.report.curve_determined.get(i) == Some(&true) {
            [91, 204, 142, 255]
        } else {
            [92, 150, 255, 255]
        };
        for (j, pair) in curve_path(&d.model, i, 0.35 / d.view.pixels_per_unit)
            .windows(2)
            .enumerate()
        {
            if (c.construction || c.centerline) && j % 3 == 1 {
                continue;
            }
            let a = screen(pair[0]);
            let b = screen(pair[1]);
            render::draw_line(&mut pixels, w, h, a, b, color);
        }
    }
    for (i, p) in d.model.points.iter().enumerate() {
        let color = if d.points.contains(&i) {
            [255, 205, 105, 255]
        } else if p.fixed {
            [231, 117, 110, 255]
        } else {
            [170, 215, 233, 255]
        };
        render::draw_ellipse(&mut pixels, w, h, screen(p.pos), 3, 3, color);
    }
    for (_, layout) in dimension_layouts(d) {
        for path in layout.lines {
            for pair in path.windows(2) {
                render::draw_line(
                    &mut pixels,
                    w,
                    h,
                    screen(pair[0]),
                    screen(pair[1]),
                    [220, 138, 132, 255],
                );
            }
        }
        for (tip, dir) in layout.arrows {
            let head = 6.0 / d.view.pixels_per_unit;
            for side in [-1.0, 1.0] {
                render::draw_line(
                    &mut pixels,
                    w,
                    h,
                    screen(tip),
                    screen(tip - dir * head + dir.perp() * (side * head * 0.4)),
                    [220, 138, 132, 255],
                );
            }
        }
    }
    let mut preview = d.pending.clone();
    if !preview.is_empty() {
        preview.push(d.cursor);
    }
    for pair in preview.windows(2) {
        render::draw_line(
            &mut pixels,
            w,
            h,
            screen(pair[0]),
            screen(pair[1]),
            [242, 196, 113, 255],
        );
    }
    if let Some(parameters) = parameters {
        if !preview.is_empty() {
            let mut sk = core::Sketch::new();
            if sketching::draw(&mut sk, d.tool, &preview, parameters).is_ok() {
                for i in 0..sk.curves.len() {
                    for pair in curve_path(&sk, i, 0.35 / d.view.pixels_per_unit).windows(2) {
                        render::draw_line(
                            &mut pixels,
                            w,
                            h,
                            screen(pair[0]),
                            screen(pair[1]),
                            [242, 196, 113, 255],
                        );
                    }
                }
            }
        }
    }
    render::image_from_rgba(w, h, pixels)
}

pub(super) fn worker_feedback(state: &mut AppState, message: &str) {
    if let Some(d) = state.drawing.as_mut() {
        d.feedback = message.into();
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;
    use slint::platform::{
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
        Platform, WindowAdapter,
    };
    struct SoftwarePlatform(Rc<MinimalSoftwareWindow>);
    impl Platform for SoftwarePlatform {
        fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, slint::PlatformError> {
            Ok(self.0.clone())
        }
        fn duration_since_start(&self) -> Duration {
            Duration::from_secs(1)
        }
    }
    fn click(ui: &MainWindow, state: &Rc<RefCell<AppState>>, p: Vec2, add: bool) {
        let (x, y) = state
            .borrow()
            .drawing
            .as_ref()
            .unwrap()
            .view
            .world_to_screen((p.x, p.y));
        ui.invoke_drawing_pointer(
            0,
            x as f32,
            y as f32,
            PREVIEW_WIDTH as f32,
            PREVIEW_HEIGHT as f32,
            add,
        );
        ui.invoke_drawing_pointer(
            2,
            x as f32,
            y as f32,
            PREVIEW_WIDTH as f32,
            PREVIEW_HEIGHT as f32,
            add,
        );
    }
    fn finish(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
        ui.invoke_drawing_action(100);
        assert!(state.borrow().cad_job.is_some());
        let until = std::time::Instant::now() + Duration::from_secs(20);
        while state.borrow().cad_job.is_some() {
            cad::poll(ui, &mut state.borrow_mut());
            assert!(std::time::Instant::now() < until);
            std::thread::sleep(Duration::from_millis(10));
        }
        refresh_ui(ui, &state.borrow());
        assert!(!ui.get_drawing_active(), "{}", ui.get_drawing_feedback());
    }
    #[test]
    fn sketch_ui_draws_constrains_undoes_publishes_and_reopens_exact_profiles() {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(Box::new(SoftwarePlatform(window.clone()))).unwrap();
        let ui = MainWindow::new().unwrap();
        let state = Rc::new(RefCell::new(AppState::new()));
        bind_callbacks(&ui, &state);
        cad::bind(&ui, &state);
        ui.invoke_cad_command(21);
        assert!(ui.get_drawing_active());
        ui.set_drawing_snap(false);
        ui.invoke_drawing_select_tool(3);
        click(&ui, &state, Vec2::new(0., 0.), false);
        click(&ui, &state, Vec2::new(20., 10.), false);
        assert_eq!(
            state.borrow().drawing.as_ref().unwrap().model.curves.len(),
            4
        );
        ui.invoke_drawing_action(104);
        assert_eq!(
            state.borrow().drawing.as_ref().unwrap().model.curves.len(),
            0
        );
        ui.invoke_drawing_action(105);
        assert_eq!(
            state.borrow().drawing.as_ref().unwrap().model.curves.len(),
            4
        );
        ui.invoke_drawing_select_tool(0);
        click(&ui, &state, Vec2::new(10., 0.), false);
        assert_eq!(state.borrow().drawing.as_ref().unwrap().curves, vec![0]);
        ui.set_drawing_constraint_kind(13);
        ui.set_drawing_value("20".into());
        ui.invoke_drawing_action(110);
        assert!(state
            .borrow()
            .drawing
            .as_ref()
            .unwrap()
            .model
            .constraints
            .iter()
            .any(|c| matches!(c.kind, core::ConstraintKind::Length { .. })));
        ui.invoke_drawing_select_tool(6);
        click(&ui, &state, Vec2::new(10., 5.), false);
        click(&ui, &state, Vec2::new(12., 5.), false);
        ui.invoke_drawing_action(106);
        ui.show().unwrap();
        for (width, height) in [(1440u32, 1000u32), (1100, 760)] {
            window.set_size(slint::PhysicalSize::new(width, height));
            window.request_redraw();
            let mut pixels = vec![slint::Rgb8Pixel::default(); (width * height) as usize];
            window.draw_if_needed(|r| {
                r.render(&mut pixels, width as usize);
            });
            assert!(pixels.iter().filter(|p| p.r > 100).count() > 100);
            let file =
                std::fs::File::create(format!("/tmp/flursys-sketch-transfer-{width}.png")).unwrap();
            let mut enc = png::Encoder::new(file, width, height);
            enc.set_color(png::ColorType::Rgb);
            let mut writer = enc.write_header().unwrap();
            writer
                .write_image_data(
                    &pixels
                        .iter()
                        .flat_map(|p| [p.r, p.g, p.b])
                        .collect::<Vec<_>>(),
                )
                .unwrap();
        }
        // Real Slint pointer events must reach the drawing surface, not just callbacks.
        use slint::platform::{PointerEventButton, WindowEvent};
        ui.invoke_drawing_select_tool(1);
        for (x, y) in [(500., 350.), (570., 410.)] {
            let position = slint::LogicalPosition::new(x, y);
            ui.window().dispatch_event(WindowEvent::PointerPressed {
                position,
                button: PointerEventButton::Left,
            });
            ui.window().dispatch_event(WindowEvent::PointerReleased {
                position,
                button: PointerEventButton::Left,
            });
        }
        assert_eq!(
            state.borrow().drawing.as_ref().unwrap().model.curves.len(),
            6,
            "Slint pointer routing must create the line"
        );
        ui.invoke_drawing_action(104);
        finish(&ui, &state);
        let (id, face) = {
            let s = state.borrow();
            let root = s
                .workbench
                .geometry()
                .sketches()
                .find(|s| s.drawing.is_some())
                .unwrap();
            (root.id, root.profile_face.unwrap())
        };
        assert!(state
            .borrow()
            .wb_selected_targets
            .contains(&GeometrySelectionTarget::Face(face)));
        let saved = serde_json::to_string(state.borrow().workbench.geometry()).unwrap();
        let restored: GeometryTopology = serde_json::from_str(&saved).unwrap();
        assert!(restored.sketch(id).unwrap().drawing.is_some());
        let b = state
            .borrow_mut()
            .workbench
            .geometry_mut()
            .cad_pad(face, 0.005, false, 1e-7)
            .unwrap();
        let index = state
            .borrow()
            .workbench
            .geometry()
            .sketches()
            .position(|s| s.id == id)
            .unwrap();
        ui.set_cad_sketch_index(index as i32);
        ui.invoke_cad_command(39);
        assert!(ui.get_drawing_active());
        assert_eq!(
            state.borrow().drawing.as_ref().unwrap().model.curves.len(),
            5
        );
        ui.invoke_drawing_action(101);
        assert!(state.borrow().workbench.geometry().cad_body(b).is_some());
        ui.hide().unwrap();
    }
}
