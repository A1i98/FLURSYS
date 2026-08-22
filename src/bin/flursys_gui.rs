use flursys::runtime::{SolverCommand, SolverController, SolverState, SolverUpdate};
use flursys::{
    BoundaryConditionKind, BoundaryFace, ExtrudedMesh3D, FieldUpdate, GeneratedMesh,
    GeometryEditorState, GeometrySelectionTarget, GeometrySketch, GeometryTool, GmshMesher,
    IncompressibleBoundaryCondition, IncompressibleSolution, IncompressibleSolveError,
    MeshDimension, MeshQualityMetric, MeshSelection, Project, ProjectCoupling, SketchAxis,
    SketchEntityKind, SketchProfileKind, SolveStatus, StructuredMesh2D, ThermalBoundaryCondition,
    Vec3, ViewTransform, WorkbenchSession,
};
use slint::{ComponentHandle, ModelRc, SharedString, Timer, TimerMode, VecModel};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::mpsc::Receiver;
use std::time::Duration;

slint::include_modules!();
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MeshColorMode {
    Neutral,
    Patches,
    Quality,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MeshDisplayMode {
    Wireframe,
    Surface,
    SurfaceEdges,
}

impl MeshDisplayMode {
    const fn draws_surface(self) -> bool {
        !matches!(self, Self::Wireframe)
    }

    const fn draws_edges(self) -> bool {
        !matches!(self, Self::Surface)
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Wireframe => "wireframe",
            Self::Surface => "surface",
            Self::SurfaceEdges => "surface + edges",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MeshPickMode {
    Face,
    Cell,
}

impl MeshPickMode {
    const fn label(self) -> &'static str {
        match self {
            Self::Face => "Face",
            Self::Cell => "Cell",
        }
    }
}

struct AppState {
    controller: SolverController,
    project: Project,
    project_loaded: bool,
    logs: VecDeque<String>,
    last_update: Option<SolverUpdate>,
    residual_history: VecDeque<ResidualSample>,
    frames: VecDeque<FieldUpdate>,
    frame_index: usize,
    animation_playing: bool,
    show_mesh: bool,
    show_geometry_3d: bool,
    geometry_yaw: f32,
    geometry_pitch: f32,
    geometry_zoom: f32,
    geometry_view_axis: i32,
    geometry_drag_anchor: Option<(f32, f32, f32, f32)>,
    selected_boundary_face: BoundaryFace,
    preflight_summary: String,
    last_animation_tick: std::time::Instant,
    draft_sketch: Option<GeometrySketch>,
    show_sketch_editor: bool,
    sketch_tool: SketchTool,
    sketch_points: Vec<(f64, f64)>,
    sketch_hover: Option<(f64, f64)>,
    sketch_undo: Vec<GeometrySketch>,
    sketch_redo: Vec<GeometrySketch>,
    workbench: WorkbenchSession,
    geometry_editor: GeometryEditorState,
    geometry_pan_anchor: Option<(f64, f64)>,
    mesh_view: ViewTransform,
    mesh_display_mode: MeshDisplayMode,
    mesh_pick_mode: MeshPickMode,
    mesh_color_mode: MeshColorMode,
    mesh_quality_metric: MeshQualityMetric,
    mesh_quality_threshold: f64,
    tree_rows: Vec<ProjectTreeRowData>,
    tree_dirty: bool,
    selected_tree: Option<TreeSelection>,
    wb_selected_targets: Vec<GeometrySelectionTarget>,
    patch_names: Vec<String>,
    meshing: bool,
    solving: bool,
    mesh_rx: Option<Receiver<Result<GeneratedMesh, flursys::MeshingError>>>,
    solve_rx: Option<Receiver<Result<IncompressibleSolution, IncompressibleSolveError>>>,
    gmsh_probe_rx: Option<Receiver<Result<String, String>>>,
    gmsh_status: String,
    current_step: usize,
}

#[derive(Clone, Copy, Debug)]
enum SketchTool {
    Select,
    Line,
    Rectangle,
    Square,
    Circle,
    Dimension,
    Trim,
}

#[derive(Clone, Copy)]
struct ResidualSample {
    continuity: f64,
    momentum: f64,
    pressure: f64,
}

impl AppState {
    fn new() -> Self {
        Self {
            controller: SolverController::spawn(),
            project: Project::default(),
            project_loaded: true,
            logs: VecDeque::from([
                "New editable project ready. Save it when you are ready to keep it.".to_string(),
            ]),
            last_update: None,
            residual_history: VecDeque::new(),
            frames: VecDeque::new(),
            frame_index: 0,
            animation_playing: false,
            show_mesh: false,
            show_geometry_3d: true,
            geometry_yaw: 0.65,
            geometry_pitch: 0.48,
            geometry_zoom: 1.0,
            geometry_view_axis: 3,
            geometry_drag_anchor: None,
            selected_boundary_face: BoundaryFace::Left,
            preflight_summary: "Run validation before starting the solver.".to_string(),
            last_animation_tick: std::time::Instant::now(),
            draft_sketch: None,
            show_sketch_editor: false,
            sketch_tool: SketchTool::Select,
            sketch_points: Vec::new(),
            sketch_hover: None,
            sketch_undo: Vec::new(),
            sketch_redo: Vec::new(),
            workbench: WorkbenchSession::new(),
            geometry_editor: {
                let mut editor = GeometryEditorState::new();
                editor
                    .transform
                    .set_viewport(f64::from(PREVIEW_WIDTH), f64::from(PREVIEW_HEIGHT));
                editor
            },
            geometry_pan_anchor: None,
            mesh_view: {
                let mut view = ViewTransform::default();
                view.set_viewport(f64::from(PREVIEW_WIDTH), f64::from(PREVIEW_HEIGHT));
                view
            },
            mesh_display_mode: MeshDisplayMode::SurfaceEdges,
            mesh_pick_mode: MeshPickMode::Face,
            mesh_color_mode: MeshColorMode::Neutral,
            mesh_quality_metric: MeshQualityMetric::AspectRatio,
            mesh_quality_threshold: 10.0,
            tree_rows: Vec::new(),
            tree_dirty: true,
            selected_tree: None,
            wb_selected_targets: Vec::new(),
            patch_names: Vec::new(),
            meshing: false,
            solving: false,
            mesh_rx: None,
            solve_rx: None,
            gmsh_probe_rx: None,
            gmsh_status: "Gmsh: checking…".to_string(),
            current_step: 0,
        }
    }

    fn spawn_gmsh_probe(&mut self) {
        let (sender, receiver) = std::sync::mpsc::channel();
        self.gmsh_probe_rx = Some(receiver);
        let mesher = GmshMesher::auto();
        std::thread::spawn(move || {
            let _ = sender.send(match mesher.version() {
                Ok(version) => Ok(format!("Gmsh {} found", version.value)),
                Err(error) => Err(format!("Gmsh unavailable: {error}")),
            });
        });
    }

    /// Polls worker-thread results (Gmsh probe, mesh generation, solve).
    /// Returns true when any state visible to the UI changed.
    fn drain_workbench_jobs(&mut self) -> bool {
        let mut changed = false;
        if let Some(receiver) = self.gmsh_probe_rx.take() {
            if let Ok(result) = receiver.try_recv() {
                match result {
                    Ok(message) => {
                        self.gmsh_status = message.clone();
                        self.log(message);
                    }
                    Err(message) => {
                        self.log(message.clone());
                        self.gmsh_status = message;
                    }
                }
                changed = true;
            } else {
                self.gmsh_probe_rx = Some(receiver);
            }
        }
        if let Some(receiver) = self.mesh_rx.take() {
            match receiver.try_recv() {
                Ok(Ok(generated)) => {
                    self.meshing = false;
                    let report = format!(
                        "Gmsh mesh installed: {} nodes, {} cells, {} patches.",
                        generated.report.node_count,
                        generated.report.cell_count,
                        generated.report.patch_count
                    );
                    self.workbench.install_mesh(generated);
                    if let Some(cache) = self.workbench.mesh_render_cache() {
                        let (min, max) = cache.bounds();
                        if cache.dimension() == MeshDimension::TwoD {
                            self.mesh_view.fit(Some((min.x, min.y, max.x, max.y)));
                        }
                    }
                    self.patch_names = self.workbench.patch_names();
                    self.tree_dirty = true;
                    self.selected_tree = None;
                    self.wb_selected_targets.clear();
                    self.log(report);
                    changed = true;
                }
                Ok(Err(error)) => {
                    self.meshing = false;
                    self.log(format!("Mesh generation failed: {error}"));
                    self.log("Adjust the mesh panel settings and try again.");
                    changed = true;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    self.mesh_rx = Some(receiver);
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.meshing = false;
                    self.log("Mesh worker stopped without a result.");
                    changed = true;
                }
            }
        }
        if let Some(receiver) = self.solve_rx.take() {
            match receiver.try_recv() {
                Ok(outcome) => {
                    self.solving = false;
                    match &outcome {
                        Ok(solution) => self.log(format!(
                            "Workbench solve finished: converged={}, outer iterations={}, final continuity={:.3e}.",
                            solution.report.converged(),
                            solution.report.outer_iterations,
                            solution
                                .report
                                .continuity_history
                                .last()
                                .copied()
                                .unwrap_or(f64::NAN)
                        )),
                        Err(error) => self.log(format!("Workbench solve failed: {error}")),
                    }
                    self.workbench.complete_solve(outcome);
                    self.tree_dirty = true;
                    changed = true;
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    self.solve_rx = Some(receiver);
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.solving = false;
                    self.workbench
                        .complete_solve(Err(flursys::IncompressibleSolveError::Case(
                            flursys::IncompressibleCaseError::InvalidInitialConditions,
                        )));
                    self.log("Solve worker stopped without reporting a status.");
                    self.tree_dirty = true;
                    changed = true;
                }
            }
        }
        changed
    }

    fn log(&mut self, line: impl Into<String>) {
        if self.logs.len() == 150 {
            self.logs.pop_front();
        }
        self.logs.push_back(line.into());
    }

    fn push_update(&mut self, update: SolverUpdate) {
        if update.iteration > 0 {
            push_bounded(
                &mut self.residual_history,
                ResidualSample {
                    continuity: update.continuity_residual,
                    momentum: update.momentum_residual,
                    pressure: update.pressure_residual,
                },
                400,
            );
        }
        if let Some(field) = update.field_update.clone() {
            push_bounded(&mut self.frames, field, 120);
            if !self.animation_playing {
                self.frame_index = self.frames.len().saturating_sub(1);
            }
        }
        self.last_update = Some(update);
    }
}

fn push_bounded<T>(values: &mut VecDeque<T>, value: T, limit: usize) {
    if values.len() == limit {
        values.pop_front();
    }
    values.push_back(value);
}

fn require_project(state: &mut AppState) -> bool {
    if state.project_loaded {
        true
    } else {
        state.log("Open or create a project before editing geometry.");
        false
    }
}

fn apply_axis_constraint(state: &mut AppState, axis: SketchAxis) {
    let before = state.draft_sketch.clone();
    let result = match &mut state.draft_sketch {
        Some(sketch) => sketch.apply_selected_axis_constraint(axis),
        None => Err("start a sketch and select a line first".to_string()),
    };
    match result {
        Ok(()) => {
            if let Some(before) = before {
                state.sketch_undo.push(before);
                state.sketch_redo.clear();
            }
            state.log(match axis {
                SketchAxis::Horizontal => "Applied horizontal constraint.",
                SketchAxis::Vertical => "Applied vertical constraint.",
            });
        }
        Err(error) => state.log(error),
    }
}

fn main() -> Result<(), slint::PlatformError> {
    let ui = MainWindow::new()?;
    let state = Rc::new(RefCell::new(AppState::new()));
    write_project_to_ui(&ui, &state.borrow().project);
    {
        let mut state = state.borrow_mut();
        state.spawn_gmsh_probe();
        push_workbench_defaults(&ui, &state.workbench);
        rebuild_tree_rows(&mut state);
    }
    refresh_ui(&ui, &state.borrow());

    bind_callbacks(&ui, &state);
    let timer = Timer::default();
    let weak_ui = ui.as_weak();
    let poll_state = state.clone();
    timer.start(TimerMode::Repeated, Duration::from_millis(16), move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = poll_state.borrow_mut();
        let mut changed = false;
        while let Ok(update) = state.controller.try_recv() {
            if let Some(message) = &update.message {
                state.log(format!("{:?}: {message}", update.state));
            } else if matches!(update.state, SolverState::Completed | SolverState::Stopped) {
                state.log(format!("Solver state: {:?}", update.state));
            }
            state.push_update(update);
            changed = true;
        }
        if state.drain_workbench_jobs() {
            changed = true;
        }
        if state.animation_playing
            && state.frames.len() > 1
            && state.last_animation_tick.elapsed() >= Duration::from_millis(120)
        {
            state.frame_index = (state.frame_index + 1) % state.frames.len();
            state.last_animation_tick = std::time::Instant::now();
            changed = true;
        }
        if changed {
            refresh_ui(&ui, &state);
        }
    });

    ui.run()
}

fn bind_callbacks(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    // ---- Phase 9D stable geometry editor ----
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_tool(move |tool| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = editor_state.borrow_mut();
        state.geometry_editor.set_tool(match tool {
            1 => GeometryTool::Line,
            2 => GeometryTool::Rectangle,
            3 => GeometryTool::Circle,
            _ => GeometryTool::Select,
        });
        let active_tool = state.geometry_editor.active_tool;
        state.log(format!("Geometry tool: {:?}.", active_tool));
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_move(move |x, y, w, h| {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        let p = geometry_editor_point(x, y, w, h);
        let geometry = state.workbench.geometry().clone();
        state.geometry_editor.cursor_moved(&geometry, p);
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_pointer(move |x, y, w, h, additive| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = editor_state.borrow_mut();
        let p = geometry_editor_point(x, y, w, h);
        let mut editor = std::mem::take(&mut state.geometry_editor);
        let result = editor.click(state.workbench.geometry_mut(), p, additive);
        state.geometry_editor = editor;
        match result {
            Ok(true) => {
                state.workbench.geometry_changed();
                state.wb_selected_targets.clear();
                state.tree_dirty = true;
                state.log("Geometry updated; dependent mesh and solution were invalidated.");
            }
            Ok(false) => {
                state.wb_selected_targets = state.geometry_editor.selection.clone();
                state.selected_tree = state
                    .wb_selected_targets
                    .first()
                    .and_then(|target| tree_selection_for_target(*target));
            }
            Err(error) => state.log(format!("Geometry edit rejected: {error}")),
        }
        rebuild_tree_rows(&mut state);
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_wheel(move |x, y, w, delta| {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        state.geometry_editor.transform.zoom_at(
            geometry_editor_point(x, y, w, PREVIEW_HEIGHT as f32),
            f64::from(delta),
        );
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_fit(move || {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        let geometry = state.workbench.geometry().clone();
        state.geometry_editor.fit_view(&geometry);
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_toggle_snap(move || {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        state.geometry_editor.snap_enabled = !state.geometry_editor.snap_enabled;
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_toggle_grid(move || {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        state.geometry_editor.grid_enabled = !state.geometry_editor.grid_enabled;
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_pan_begin(move |x, y, width, height| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = editor_state.borrow_mut();
        state.geometry_pan_anchor = Some(geometry_editor_point(x, y, width, height));
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_pan_move(move |x, y, width, height| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = editor_state.borrow_mut();
        let point = geometry_editor_point(x, y, width, height);
        if let Some(previous) = state.geometry_pan_anchor.replace(point) {
            state
                .geometry_editor
                .transform
                .pan_pixels(point.0 - previous.0, point.1 - previous.1);
        }
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_shortcut(move |action| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = editor_state.borrow_mut();
        match action {
            0 => state.geometry_editor.cancel(),
            1 => state.geometry_editor.set_tool(GeometryTool::Line),
            2 => state.geometry_editor.set_tool(GeometryTool::Rectangle),
            3 => state.geometry_editor.set_tool(GeometryTool::Circle),
            4 => delete_editor_selection(&mut state),
            5 => {
                let geometry = state.workbench.geometry().clone();
                state.geometry_editor.fit_view(&geometry);
            }
            6 => {
                let mut editor = std::mem::take(&mut state.geometry_editor);
                let changed = editor.undo(state.workbench.geometry_mut());
                state.geometry_editor = editor;
                if changed {
                    state.workbench.geometry_changed();
                    state.wb_selected_targets.clear();
                    rebuild_tree_rows(&mut state);
                }
            }
            7 => {
                let mut editor = std::mem::take(&mut state.geometry_editor);
                let changed = editor.redo(state.workbench.geometry_mut());
                state.geometry_editor = editor;
                if changed {
                    state.workbench.geometry_changed();
                    state.wb_selected_targets.clear();
                    rebuild_tree_rows(&mut state);
                }
            }
            _ => {}
        }
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_undo(move || {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        let mut editor = std::mem::take(&mut state.geometry_editor);
        let changed = editor.undo(state.workbench.geometry_mut());
        state.geometry_editor = editor;
        if changed {
            state.workbench.geometry_changed();
            state.wb_selected_targets.clear();
            state.log("Geometry undo restored stable topology IDs.");
            rebuild_tree_rows(&mut state);
        }
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_redo(move || {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        let mut editor = std::mem::take(&mut state.geometry_editor);
        let changed = editor.redo(state.workbench.geometry_mut());
        state.geometry_editor = editor;
        if changed {
            state.workbench.geometry_changed();
            state.wb_selected_targets.clear();
            state.log("Geometry redo restored stable topology IDs.");
            rebuild_tree_rows(&mut state);
        }
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let editor_state = state.clone();
    ui.on_geometry_delete_selection(move || {
        let Some(ui) = weak_ui.upgrade() else { return };
        let mut state = editor_state.borrow_mut();
        delete_editor_selection(&mut state);
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let new_project_state = state.clone();
    ui.on_new_project(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = new_project_state.borrow_mut();
        if let Err(error) = state.controller.send(SolverCommand::Stop) {
            state.log(error);
        }
        state.project = Project::default();
        state.project_loaded = true;
        state.last_update = None;
        state.residual_history.clear();
        state.frames.clear();
        state.frame_index = 0;
        state.animation_playing = false;
        state.show_mesh = false;
        state.show_geometry_3d = true;
        state.draft_sketch = None;
        state.show_sketch_editor = false;
        state.sketch_points.clear();
        state.sketch_hover = None;
        state.sketch_undo.clear();
        state.sketch_redo.clear();
        state.preflight_summary = "Run validation before starting the solver.".to_string();
        state.log("Created a new editable project.");
        write_project_to_ui(&ui, &state.project);
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let workflow_state = state.clone();
    ui.on_select_step(move |step| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = workflow_state.borrow_mut();
        apply_workflow_step(&ui, &mut state, step);
    });

    let weak_ui = ui.as_weak();
    let case_state = state.clone();
    ui.on_select_case(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = case_state.borrow_mut();
        sync_project_from_ui(&ui, &mut state.project);
        state.project.case = project_case_from_index(ui.get_case_index());
        state.project_loaded = true;
        let selected_case = case_name(&state.project.case);
        state.project.name = selected_case.to_string();
        state.show_geometry_3d = true;
        state.show_mesh = false;
        state.animation_playing = false;
        state.log(format!("Geometry selected: {selected_case}."));
        write_project_to_ui(&ui, &state.project);
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let parts_state = state.clone();
    ui.on_add_part(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = parts_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        sync_project_from_ui(&ui, &mut state.project);
        match geometry_part_from_ui(&ui) {
            Ok(part) if state.project.preprocessing.geometry.parts.len() < 128 => {
                state.log(format!("Added 3D solid: {}.", part.name));
                state.project.preprocessing.geometry.parts.push(part);
                state.show_geometry_3d = true;
                state.show_mesh = false;
            }
            Ok(_) => state.log("A project can contain at most 128 geometry parts."),
            Err(error) => state.log(error),
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let feature_state = state.clone();
    ui.on_build_feature(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = feature_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        sync_project_from_ui(&ui, &mut state.project);
        let sketch = state
            .draft_sketch
            .take()
            .map(Ok)
            .unwrap_or_else(|| sketch_from_ui(&ui));
        match sketch.and_then(|sketch| {
            feature_from_ui(&ui).and_then(|(name, feature)| {
                state
                    .project
                    .preprocessing
                    .geometry
                    .add_sketch_feature(sketch, name, feature)
            })
        }) {
            Ok(output) => {
                state.log(format!("Built CAD feature output: {output}."));
                state.show_geometry_3d = true;
                state.show_mesh = false;
                state.show_sketch_editor = false;
                write_project_to_ui(&ui, &state.project);
            }
            Err(error) => state.log(error),
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let sketch_state = state.clone();
    ui.on_start_sketch(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = sketch_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        match sketch_from_ui(&ui) {
            Ok(mut sketch) => {
                sketch.entities.clear();
                sketch.dimensions.clear();
                state.draft_sketch = Some(sketch);
                state.show_sketch_editor = true;
                state.sketch_tool = SketchTool::Select;
                state.sketch_points.clear();
                state.sketch_hover = None;
                state.sketch_undo.clear();
                state.sketch_redo.clear();
                state.log("Started an editable 2D sketch on the XY plane.");
            }
            Err(error) => state.log(error),
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let sketch_state = state.clone();
    ui.on_select_sketch_tool(move |tool| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = sketch_state.borrow_mut();
        state.sketch_points.clear();
        state.sketch_hover = None;
        state.sketch_tool = match tool {
            8 => SketchTool::Line,
            1 => SketchTool::Rectangle,
            2 => SketchTool::Square,
            3 => SketchTool::Circle,
            4 => SketchTool::Dimension,
            5 => SketchTool::Trim,
            6 => {
                apply_axis_constraint(&mut state, SketchAxis::Horizontal);
                SketchTool::Select
            }
            7 => {
                apply_axis_constraint(&mut state, SketchAxis::Vertical);
                SketchTool::Select
            }
            _ => SketchTool::Select,
        };
        state.show_sketch_editor = state.draft_sketch.is_some();
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let sketch_state = state.clone();
    ui.on_sketch_click(move |mouse_x, mouse_y, width, height| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = sketch_state.borrow_mut();
        let Some(canvas_point) = sketch_viewport_point(mouse_x, mouse_y, width, height) else {
            return;
        };
        let point = snap_to_existing_sketch_geometry(
            state.draft_sketch.as_ref(),
            &state.sketch_points,
            snap_sketch_point(canvas_point),
        );
        let before = state.draft_sketch.clone();
        state.sketch_hover = Some(point);
        let result = apply_sketch_click(&mut state, point);
        if result.is_ok() && state.draft_sketch != before {
            if let Some(before) = before {
                state.sketch_undo.push(before);
                state.sketch_redo.clear();
            }
        }
        if let Err(error) = result {
            state.log(error);
        } else if matches!(state.sketch_tool, SketchTool::Select) {
            if let Some(value) = state
                .draft_sketch
                .as_ref()
                .and_then(selected_entity_dimension)
            {
                ui.set_driving_dimension(SharedString::from(format!("{value:.3}")));
            }
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let sketch_preview_state = state.clone();
    ui.on_sketch_preview(move |mouse_x, mouse_y, width, height| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = sketch_preview_state.borrow_mut();
        if state.draft_sketch.is_none() || state.sketch_points.is_empty() {
            return;
        }
        let Some(canvas_point) = sketch_viewport_point(mouse_x, mouse_y, width, height) else {
            return;
        };
        let point = snap_to_existing_sketch_geometry(
            state.draft_sketch.as_ref(),
            &state.sketch_points,
            snap_sketch_point(canvas_point),
        );
        if state.sketch_hover != Some(point) {
            state.sketch_hover = Some(point);
            refresh_ui(&ui, &state);
        }
    });

    let weak_ui = ui.as_weak();
    let sketch_state = state.clone();
    ui.on_sketch_undo(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = sketch_state.borrow_mut();
        if let (Some(previous), Some(current)) =
            (state.sketch_undo.pop(), state.draft_sketch.take())
        {
            state.sketch_redo.push(current);
            state.draft_sketch = Some(previous);
            state.log("Sketch undo.");
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let sketch_state = state.clone();
    ui.on_sketch_redo(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = sketch_state.borrow_mut();
        if let (Some(next), Some(current)) = (state.sketch_redo.pop(), state.draft_sketch.take()) {
            state.sketch_undo.push(current);
            state.draft_sketch = Some(next);
            state.log("Sketch redo.");
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let sketch_state = state.clone();
    ui.on_apply_driving_dimension(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = sketch_state.borrow_mut();
        let before = state.draft_sketch.clone();
        let result = match &mut state.draft_sketch {
            Some(sketch) => {
                parse_positive(ui.get_driving_dimension().as_str(), "driving dimension")
                    .and_then(|value| sketch.set_selected_dimension(value))
            }
            None => Err("start a sketch and select an entity first".to_string()),
        };
        if result.is_ok() {
            if let Some(before) = before {
                state.sketch_undo.push(before);
                state.sketch_redo.clear();
            }
        } else if let Err(error) = result {
            state.log(error);
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let parts_state = state.clone();
    ui.on_duplicate_last_part(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = parts_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        if state.project.preprocessing.geometry.parts.len() >= 128 {
            state.log("A project can contain at most 128 geometry parts.");
        } else if let Some(source) = state.project.preprocessing.geometry.parts.last().cloned() {
            let mut duplicate = source.clone();
            duplicate.name =
                next_copy_name(&source.name, &state.project.preprocessing.geometry.parts);
            // A small offset makes a duplicate visible and immediately editable.
            duplicate.x += 0.1;
            duplicate.y += 0.1;
            state.project.preprocessing.geometry.parts.push(duplicate);
            state.log(format!("Duplicated 3D solid: {}.", source.name));
        } else {
            state.log("There is no custom 3D solid to duplicate.");
        }
        state.show_geometry_3d = true;
        state.show_mesh = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let parts_state = state.clone();
    ui.on_remove_last_part(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = parts_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        if let Some(part) = state.project.preprocessing.geometry.parts.pop() {
            if let Some(feature) = state
                .project
                .preprocessing
                .geometry
                .features
                .pop_if(|feature| feature.output_part == part.name)
            {
                if let Some(sketch_index) = state
                    .project
                    .preprocessing
                    .geometry
                    .sketches
                    .iter()
                    .position(|sketch| sketch.id == feature.sketch_id)
                {
                    state
                        .project
                        .preprocessing
                        .geometry
                        .sketches
                        .remove(sketch_index);
                }
            }
            state.log(format!("Removed 3D solid: {}.", part.name));
        } else {
            state.log("There is no custom 3D solid to remove.");
        }
        state.show_geometry_3d = true;
        state.show_mesh = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let start_state = state.clone();
    ui.on_start(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = start_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        sync_project_from_ui(&ui, &mut state.project);
        match preflight_report(&state.project) {
            Ok(report) => state.preflight_summary = report,
            Err(error) => {
                state.preflight_summary = format!("BLOCKED\n{error}");
                state.log(error);
                refresh_ui(&ui, &state);
                return;
            }
        }
        state.residual_history.clear();
        state.frames.clear();
        state.frame_index = 0;
        state.animation_playing = false;
        match state.project.simulation_config("results/gui-run") {
            Ok(config) => match state
                .controller
                .send(SolverCommand::Start(Box::new(config)))
            {
                Ok(()) => state.log("Solver start requested."),
                Err(error) => state.log(error),
            },
            Err(error) => state.log(error),
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let validate_state = state.clone();
    ui.on_validate_case(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = validate_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        sync_project_from_ui(&ui, &mut state.project);
        match preflight_report(&state.project) {
            Ok(report) => {
                state.preflight_summary = report;
                state.log("Case validation passed.");
            }
            Err(error) => {
                state.preflight_summary = format!("BLOCKED\n{error}");
                state.log(error);
            }
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let pause_state = state.clone();
    ui.on_pause(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = pause_state.borrow_mut();
        if let Err(error) = state.controller.send(SolverCommand::Pause) {
            state.log(error);
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let resume_state = state.clone();
    ui.on_resume(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = resume_state.borrow_mut();
        if let Err(error) = state.controller.send(SolverCommand::Resume) {
            state.log(error);
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let stop_state = state.clone();
    ui.on_stop(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = stop_state.borrow_mut();
        if let Err(error) = state.controller.send(SolverCommand::Stop) {
            state.log(error);
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let load_state = state.clone();
    ui.on_load_project(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        match Project::load(ui.get_project_path().as_str()) {
            Ok(project) => {
                let mut state = load_state.borrow_mut();
                state.project = project;
                state.project_loaded = true;
                state.log("Project loaded.");
                write_project_to_ui(&ui, &state.project);
                refresh_ui(&ui, &state);
            }
            Err(error) => {
                let mut state = load_state.borrow_mut();
                state.log(error);
                refresh_ui(&ui, &state);
            }
        }
    });

    let weak_ui = ui.as_weak();
    let save_state = state.clone();
    ui.on_save_project(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = save_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        sync_project_from_ui(&ui, &mut state.project);
        match state.project.save(ui.get_project_path().as_str()) {
            Ok(()) => state.log("Project saved."),
            Err(error) => state.log(error),
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let mesh_state = state.clone();
    ui.on_show_mesh(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        state.show_mesh = true;
        state.show_geometry_3d = false;
        state.animation_playing = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let field_state = state.clone();
    ui.on_show_field(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = field_state.borrow_mut();
        state.show_mesh = false;
        state.show_geometry_3d = false;
        state.frame_index = state.frames.len().saturating_sub(1);
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let play_state = state.clone();
    ui.on_animation_play_pause(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = play_state.borrow_mut();
        if state.frames.len() > 1 {
            state.show_mesh = false;
            state.animation_playing = !state.animation_playing;
            state.last_animation_tick = std::time::Instant::now();
        } else {
            state.log("Animation needs at least two field snapshots.");
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let next_state = state.clone();
    ui.on_animation_next(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = next_state.borrow_mut();
        if !state.frames.is_empty() {
            state.show_mesh = false;
            state.animation_playing = false;
            state.frame_index = (state.frame_index + 1) % state.frames.len();
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let geometry_state = state.clone();
    ui.on_show_geometry_3d(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = geometry_state.borrow_mut();
        if !require_project(&mut state) {
            refresh_ui(&ui, &state);
            return;
        }
        sync_project_from_ui(&ui, &mut state.project);
        state.show_geometry_3d = true;
        state.show_mesh = false;
        state.show_sketch_editor = false;
        state.animation_playing = false;
        state.log("Showing the saved geometry and mesh extrusion preview.");
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let rotate_state = state.clone();
    ui.on_rotate_geometry_3d(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = rotate_state.borrow_mut();
        sync_project_from_ui(&ui, &mut state.project);
        state.show_geometry_3d = true;
        state.show_mesh = false;
        state.animation_playing = false;
        state.geometry_yaw = (state.geometry_yaw + 0.45) % std::f32::consts::TAU;
        state.geometry_view_axis = 3;
        state.geometry_drag_anchor = None;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let orbit_state = state.clone();
    ui.on_geometry_drag(move |mouse_x, mouse_y, pressed_x, pressed_y| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = orbit_state.borrow_mut();
        let new_gesture = state.geometry_drag_anchor.is_none_or(|(x, y, _, _)| {
            (x - pressed_x).abs() > f32::EPSILON || (y - pressed_y).abs() > f32::EPSILON
        });
        if new_gesture {
            state.geometry_drag_anchor = Some((
                pressed_x,
                pressed_y,
                state.geometry_yaw,
                state.geometry_pitch,
            ));
        }
        if let Some((start_x, start_y, start_yaw, start_pitch)) = state.geometry_drag_anchor {
            state.geometry_yaw = start_yaw + (mouse_x - start_x) * 0.012;
            state.geometry_pitch = (start_pitch + (mouse_y - start_y) * 0.008).clamp(-0.85, 0.85);
            state.geometry_view_axis = 3;
        }
        state.show_geometry_3d = true;
        state.show_mesh = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let zoom_state = state.clone();
    ui.on_geometry_zoom(move |delta_y| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = zoom_state.borrow_mut();
        state.geometry_zoom = (state.geometry_zoom * (-delta_y * 0.0015).exp()).clamp(0.62, 1.28);
        state.show_geometry_3d = true;
        state.show_mesh = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let view_axis_state = state.clone();
    ui.on_select_geometry_view_axis(move |axis| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = view_axis_state.borrow_mut();
        let axis = axis.clamp(0, 3);
        let (yaw, pitch) = geometry_view_angles(axis);
        state.geometry_yaw = yaw;
        state.geometry_pitch = pitch;
        state.geometry_view_axis = axis;
        state.geometry_drag_anchor = None;
        state.show_geometry_3d = true;
        state.show_mesh = false;
        state.animation_playing = false;
        state.log(format!(
            "Selected {} geometry view.",
            geometry_view_label(axis)
        ));
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let reset_geometry_state = state.clone();
    ui.on_reset_geometry_view(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = reset_geometry_state.borrow_mut();
        (state.geometry_yaw, state.geometry_pitch) = geometry_view_angles(3);
        state.geometry_zoom = 1.0;
        state.geometry_view_axis = 3;
        state.geometry_drag_anchor = None;
        state.show_geometry_3d = true;
        state.show_mesh = false;
        state.log("Geometry camera reset.");
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let boundary_state = state.clone();
    ui.on_apply_boundary(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = boundary_state.borrow_mut();
        sync_project_from_ui(&ui, &mut state.project);
        let face = boundary_face_from_index(ui.get_boundary_face_index());
        state.selected_boundary_face = face;
        if ui.get_boundary_kind_index() == 2 && face != BoundaryFace::Right {
            state.log("The active 2D solver accepts a pressure outlet only on the right boundary.");
            refresh_ui(&ui, &state);
            return;
        }
        let value = parse_number(ui.get_boundary_value().as_str(), 0.0);
        let kind = match ui.get_boundary_kind_index() {
            1 => BoundaryConditionKind::Velocity {
                u: value,
                v: 0.0,
                w: 0.0,
            },
            2 => BoundaryConditionKind::PressureOutlet { pressure: value },
            3 => BoundaryConditionKind::Wall {
                u: value,
                v: 0.0,
                w: 0.0,
            },
            4 => BoundaryConditionKind::Symmetry,
            _ => BoundaryConditionKind::CaseDefault,
        };
        if let Some(boundary) = state.project.preprocessing.boundary_mut(face) {
            boundary.kind = kind;
            state.log(format!("{} boundary updated.", face.label()));
        } else {
            state.log(format!(
                "{} boundary is missing from the project.",
                face.label()
            ));
        }
        state.show_geometry_3d = true;
        state.show_mesh = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let boundary_preview_state = state.clone();
    ui.on_show_boundary_face(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = boundary_preview_state.borrow_mut();
        sync_project_from_ui(&ui, &mut state.project);
        state.selected_boundary_face = boundary_face_from_index(ui.get_boundary_face_index());
        state.show_geometry_3d = true;
        state.show_mesh = false;
        state.animation_playing = false;
        let label = state.selected_boundary_face.label();
        state.log(format!("Highlighted {label} boundary in the mesh preview."));
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let pick_boundary_state = state.clone();
    ui.on_pick_boundary(move |x, y, preview_width, preview_height| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let Some(point) = preview_image_point(x, y, preview_width, preview_height) else {
            return;
        };
        let mut state = pick_boundary_state.borrow_mut();
        if state.show_mesh {
            let point = geometry_editor_point(x, y, preview_width, preview_height);
            if let Some(cache) = state.workbench.mesh_render_cache() {
                let selected = match (cache.dimension(), state.mesh_pick_mode) {
                    (MeshDimension::TwoD, MeshPickMode::Face) => cache.pick_face_2d(
                        state.mesh_view.screen_to_world(point),
                        8.0 / state.mesh_view.pixels_per_unit,
                    ),
                    (MeshDimension::TwoD, MeshPickMode::Cell) => {
                        cache.pick_cell_2d(state.mesh_view.screen_to_world(point))
                    }
                    (MeshDimension::ThreeD, MeshPickMode::Face) => {
                        let (origin, direction) = mesh_screen_ray(point);
                        cache.pick_face_3d(origin, direction)
                    }
                    (MeshDimension::ThreeD, MeshPickMode::Cell) => None,
                };
                state.workbench.set_mesh_selection(selected);
                refresh_ui(&ui, &state);
                return;
            }
        }
        if state.show_geometry_3d {
            if let Some(axis) = pick_orientation_axis_3d(
                &state.project,
                state.geometry_yaw,
                state.geometry_pitch,
                state.geometry_zoom,
                point,
            ) {
                let (yaw, pitch) = geometry_view_angles(axis);
                state.geometry_yaw = yaw;
                state.geometry_pitch = pitch;
                state.geometry_view_axis = axis;
                state.geometry_drag_anchor = None;
                state.log(format!(
                    "Selected {} axis view from triad.",
                    geometry_view_label(axis)
                ));
                refresh_ui(&ui, &state);
                return;
            }
        }
        let selected = if state.show_mesh {
            pick_boundary_2d(&state.project, point)
        } else if state.show_geometry_3d {
            pick_boundary_3d(
                &state.project,
                state.geometry_yaw,
                state.geometry_pitch,
                state.geometry_zoom,
                point,
            )
        } else {
            None
        };
        if let Some(face) = selected {
            state.selected_boundary_face = face;
            write_boundary_to_ui(&ui, &state.project, face);
            state.log(format!("Selected {} boundary from preview.", face.label()));
            refresh_ui(&ui, &state);
        }
    });

    let weak_ui = ui.as_weak();
    let thermal_state = state.clone();
    ui.on_apply_thermal_boundary(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = thermal_state.borrow_mut();
        sync_project_from_ui(&ui, &mut state.project);
        let temperature = parse_number(
            ui.get_thermal_boundary_value().as_str(),
            state.project.physics.thermal.initial_temperature,
        );
        let boundary = if ui.get_thermal_boundary_kind_index() == 0 {
            ThermalBoundaryCondition::Adiabatic
        } else {
            ThermalBoundaryCondition::FixedTemperature { temperature }
        };
        match ui.get_thermal_face_index() {
            1 => state.project.physics.thermal.right = boundary,
            2 => state.project.physics.thermal.bottom = boundary,
            3 => state.project.physics.thermal.top = boundary,
            _ => state.project.physics.thermal.left = boundary,
        }
        state.log("Thermal boundary condition updated.");
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let results_state = state.clone();
    ui.on_select_result_field(move |index| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        ui.set_result_field_index(index.clamp(0, 3));
        let mut state = results_state.borrow_mut();
        state.show_mesh = false;
        state.show_geometry_3d = false;
        state.animation_playing = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let results_state = state.clone();
    ui.on_apply_result_style(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = results_state.borrow_mut();
        state.show_mesh = false;
        state.show_geometry_3d = false;
        state.animation_playing = false;
        state.log(format!(
            "Applied result style: {} / {} / {} levels.",
            result_plot_label(ui.get_result_plot_index()),
            result_colormap_label(ui.get_result_colormap_index()),
            ui.get_result_contour_levels().clamp(3, 32),
        ));
        refresh_ui(&ui, &state);
    });

    // ---- Workbench pipeline ----
    let weak_ui = ui.as_weak();
    let workflow_state = state.clone();
    ui.on_select_tree_row(move |index| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = workflow_state.borrow_mut();
        let Some(row) = state
            .tree_rows
            .get(usize::try_from(index.max(0)).unwrap_or(usize::MAX))
            .cloned()
        else {
            return;
        };
        match row.kind {
            TREE_KIND_STAGE => apply_workflow_step(&ui, &mut state, row.payload),
            TREE_KIND_BODY | TREE_KIND_FACE | TREE_KIND_EDGE | TREE_KIND_VERTEX => {
                if let Some(target) = find_geometry_target(&state.workbench, row.kind, row.payload)
                {
                    if let Some(existing) =
                        state.wb_selected_targets.iter().position(|t| *t == target)
                    {
                        state.wb_selected_targets.remove(existing);
                        state.log(format!(
                            "Deselected {target:?} from the geometry selection."
                        ));
                    } else {
                        state.wb_selected_targets.push(target);
                        state.log(format!("Added {target:?} to the geometry selection."));
                    }
                    state.geometry_editor.selection = state.wb_selected_targets.clone();
                }
                state.selected_tree = Some(TreeSelection {
                    kind: row.kind,
                    payload: row.payload,
                });
                ui.set_inspector_mode(0);
                rebuild_tree_rows(&mut state);
                refresh_ui(&ui, &state);
            }
            TREE_KIND_NAMED_SELECTION => {
                let names: Vec<String> = state
                    .workbench
                    .named_selections()
                    .iter()
                    .map(|selection| selection.name.clone())
                    .collect();
                let Some(name) = names
                    .get(usize::try_from(row.payload.max(0)).unwrap_or(usize::MAX))
                    .cloned()
                else {
                    return;
                };
                state.selected_tree = Some(TreeSelection {
                    kind: row.kind,
                    payload: row.payload,
                });
                state.wb_selected_targets.clear();
                if let Some(selection) = state.workbench.named_selections().get(&name) {
                    ui.set_ns_edit_name(SharedString::from(selection.name.as_str()));
                    ui.set_ns_members(SharedString::from(describe_targets(&selection.targets)));
                }
                ui.set_inspector_mode(1);
                rebuild_tree_rows(&mut state);
                refresh_ui(&ui, &state);
            }
            TREE_KIND_PATCH => {
                let Some(name) = state
                    .patch_names
                    .get(usize::try_from(row.payload.max(0)).unwrap_or(usize::MAX))
                    .cloned()
                else {
                    return;
                };
                state.selected_tree = Some(TreeSelection {
                    kind: row.kind,
                    payload: row.payload,
                });
                ui.set_patch_index(
                    row.payload
                        .clamp(0, (state.patch_names.len().max(1) - 1) as i32),
                );
                push_boundary_fields(&ui, &state.workbench, &name);
                // A patch selection is a mesh-view selection: keep the boundary
                // inspector open while showing the exact mesh faces it owns.
                state.show_mesh = true;
                state.show_geometry_3d = false;
                ui.set_current_step(1);
                state.current_step = 1;
                ui.set_inspector_mode(3);
                rebuild_tree_rows(&mut state);
                refresh_ui(&ui, &state);
            }
            _ => {}
        }
    });

    let weak_ui = ui.as_weak();
    let ns_state = state.clone();
    ui.on_create_named_selection(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = ns_state.borrow_mut();
        let name = ui.get_ns_edit_name().trim().to_string();
        if name.is_empty() {
            state.log("Enter a name before creating a Named Selection.");
            refresh_ui(&ui, &state);
            return;
        }
        if state.wb_selected_targets.is_empty() {
            state.log("Select one or more geometry entities in the project tree first.");
            refresh_ui(&ui, &state);
            return;
        }
        let count = state.wb_selected_targets.len();
        let targets = state.wb_selected_targets.clone();
        match state.workbench.create_named_selection(&name, targets) {
            Ok(()) => {
                state.log(format!(
                    "Created Named Selection '{name}' with {count} entities."
                ));
                ui.set_ns_edit_name(SharedString::from(""));
                state.tree_dirty = true;
                rebuild_tree_rows(&mut state);
                refresh_ui(&ui, &state);
            }
            Err(error) => state.log(format!("Cannot create Named Selection: {error}")),
        }
    });

    let weak_ui = ui.as_weak();
    let ns_state = state.clone();
    ui.on_rename_named_selection(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = ns_state.borrow_mut();
        let Some(old_name) = selected_named_selection_name(&state) else {
            state.log("Select a Named Selection in the project tree first.");
            refresh_ui(&ui, &state);
            return;
        };
        let new_name = ui.get_ns_edit_name().trim().to_string();
        match state.workbench.rename_named_selection(&old_name, &new_name) {
            Ok(()) => {
                state.log(format!(
                    "Renamed Named Selection '{old_name}' to '{new_name}'."
                ));
                ui.set_ns_edit_name(SharedString::from(new_name.as_str()));
                state.tree_dirty = true;
                rebuild_tree_rows(&mut state);
                refresh_ui(&ui, &state);
            }
            Err(error) => state.log(format!("Cannot rename '{old_name}': {error}")),
        }
    });

    let weak_ui = ui.as_weak();
    let ns_state = state.clone();
    ui.on_delete_named_selection(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = ns_state.borrow_mut();
        let Some(name) = selected_named_selection_name(&state) else {
            state.log("Select a Named Selection in the project tree first.");
            refresh_ui(&ui, &state);
            return;
        };
        if state.workbench.delete_named_selection(&name) {
            state.log(format!(
                "Deleted Named Selection '{name}'. Its boundary assignment is dropped."
            ));
            state.selected_tree = None;
            state.tree_dirty = true;
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
        } else {
            state.log(format!("Named Selection '{name}' no longer exists."));
        }
    });

    let weak_ui = ui.as_weak();
    let mesh_state = state.clone();
    ui.on_generate_mesh_wb(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_state.borrow_mut();
        if state.meshing || state.solving {
            state.log("A Gmsh generation or solve is already running; wait for it to finish.");
            refresh_ui(&ui, &state);
            return;
        }
        let dimension = if ui.get_mesh_dimension_index() == 0 {
            MeshDimension::TwoD
        } else {
            MeshDimension::ThreeD
        };
        let global_size = parse_number(ui.get_wb_global_size().as_str(), 0.0);
        let min_size = parse_number(ui.get_wb_min_size().as_str(), 0.0);
        let max_size = parse_number(ui.get_wb_max_size().as_str(), 0.0);
        if let Err(error) =
            state
                .workbench
                .set_mesh_configuration(dimension, global_size, min_size, max_size, 1)
        {
            state.log(format!("Invalid mesh configuration: {error}"));
            refresh_ui(&ui, &state);
            return;
        }
        match state.workbench.mesh_generation_inputs() {
            Ok((export, options)) => {
                let (sender, receiver) = std::sync::mpsc::channel();
                state.mesh_rx = Some(receiver);
                state.meshing = true;
                std::thread::spawn(move || {
                    let mesher = GmshMesher::auto();
                    let result = mesher.generate(&export.document, &options);
                    let _ = sender.send(result);
                });
                state.log(format!(
                    "Gmsh generation started ({:?}, global size {global_size}).",
                    dimension
                ));
            }
            Err(error) => state.log(format!("Cannot start meshing: {error}")),
        }
        rebuild_tree_rows(&mut state);
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let bc_state = state.clone();
    ui.on_apply_boundary_wb(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = bc_state.borrow_mut();
        let index = ui
            .get_patch_index()
            .clamp(0, (state.patch_names.len().max(1) - 1) as i32) as usize;
        let Some(patch) = state.patch_names.get(index).cloned() else {
            state.log("Generate a mesh before assigning boundary conditions.");
            return;
        };
        let condition = match ui.get_bc_kind_index() {
            0 => IncompressibleBoundaryCondition::NoSlipWall,
            1 => IncompressibleBoundaryCondition::MovingWall {
                velocity: Vec3::new(
                    parse_number(ui.get_bc_value_x().as_str(), 0.0),
                    parse_number(ui.get_bc_value_y().as_str(), 0.0),
                    parse_number(ui.get_bc_value_z().as_str(), 0.0),
                ),
            },
            2 => IncompressibleBoundaryCondition::VelocityInlet {
                velocity: Vec3::new(
                    parse_number(ui.get_bc_value_x().as_str(), 0.0),
                    parse_number(ui.get_bc_value_y().as_str(), 0.0),
                    parse_number(ui.get_bc_value_z().as_str(), 0.0),
                ),
            },
            _ => IncompressibleBoundaryCondition::PressureOutlet {
                pressure: parse_number(ui.get_bc_pressure().as_str(), 0.0),
            },
        };
        match state.workbench.assign_boundary(&patch, condition) {
            Ok(()) => {
                state.log(format!(
                    "Assigned {:?} to patch '{patch}'.",
                    bc_kind_label(condition)
                ));
                push_boundary_fields(&ui, &state.workbench, &patch);
                rebuild_tree_rows(&mut state);
                refresh_ui(&ui, &state);
            }
            Err(error) => state.log(format!("Cannot assign boundary condition: {error}")),
        }
    });

    let weak_ui = ui.as_weak();
    let bc_state = state.clone();
    ui.on_unassign_boundary_wb(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = bc_state.borrow_mut();
        let index = ui
            .get_patch_index()
            .clamp(0, (state.patch_names.len().max(1) - 1) as i32) as usize;
        let Some(patch) = state.patch_names.get(index).cloned() else {
            state.log("Generate a mesh first.");
            return;
        };
        if state.workbench.unassign_boundary(&patch) {
            state.log(format!("Cleared the assignment on patch '{patch}'."));
            push_boundary_fields(&ui, &state.workbench, &patch);
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
        } else {
            state.log(format!("Patch '{patch}' had no assigned condition."));
        }
    });

    let weak_ui = ui.as_weak();
    let solver_state_wb = state.clone();
    ui.on_apply_solver_settings_wb(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = solver_state_wb.borrow_mut();
        match sync_solver_panel(&ui, &mut state.workbench) {
            Ok(()) => state.log("Applied material and SIMPLE solver settings."),
            Err(error) => state.log(error),
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let run_state = state.clone();
    ui.on_run_workbench(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = run_state.borrow_mut();
        if state.meshing || state.solving {
            state.log("A job is already running; wait for it to finish.");
            refresh_ui(&ui, &state);
            return;
        }
        if let Err(error) = sync_solver_panel(&ui, &mut state.workbench) {
            state.log(error);
            refresh_ui(&ui, &state);
            return;
        }
        if let Err(reason) = state.workbench.readiness() {
            state.log(format!("Run blocked: {reason}"));
            refresh_ui(&ui, &state);
            return;
        }
        let case = match state.workbench.prepare_case() {
            Ok(case) => case,
            Err(error) => {
                state.log(format!("Run blocked: {error}"));
                refresh_ui(&ui, &state);
                return;
            }
        };
        let cells = case.mesh.cell_count();
        state.workbench.mark_solving();
        let (sender, receiver) = std::sync::mpsc::channel();
        state.solve_rx = Some(receiver);
        state.solving = true;
        std::thread::spawn(move || {
            let outcome = flursys::solve_incompressible(&case);
            let _ = sender.send(outcome);
        });
        state.log(format!(
            "Unstructured incompressible solve started on {cells} cells."
        ));
        rebuild_tree_rows(&mut state);
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let vtk_state = state.clone();
    ui.on_export_vtk_wb(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = vtk_state.borrow_mut();
        let path = std::path::PathBuf::from("results/workbench-run/channel.vtk");
        match state.workbench.export_vtk(&path) {
            Ok(()) => {
                state.log(format!(
                    "Exported legacy VTK unstructured grid to {}.",
                    path.display()
                ));
                ui.set_visualization_title(SharedString::from("WORKBENCH RUN EXPORTED"));
            }
            Err(error) => state.log(format!("VTK export failed: {error}")),
        }
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let gmsh_state = state.clone();
    ui.on_refresh_gmsh_status(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = gmsh_state.borrow_mut();
        state.spawn_gmsh_probe();
        state.log("Checking for the gmsh executable…");
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let mesh_controls = state.clone();
    ui.on_mesh_display(move |index| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_controls.borrow_mut();
        state.mesh_display_mode = match index {
            0 => MeshDisplayMode::Wireframe,
            1 => MeshDisplayMode::Surface,
            _ => MeshDisplayMode::SurfaceEdges,
        };
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let mesh_controls = state.clone();
    ui.on_mesh_select_mode(move |index| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_controls.borrow_mut();
        state.mesh_pick_mode = if index == 1 {
            MeshPickMode::Cell
        } else {
            MeshPickMode::Face
        };
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let mesh_controls = state.clone();
    ui.on_mesh_color(move |index| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_controls.borrow_mut();
        state.mesh_color_mode = match index {
            1 => MeshColorMode::Patches,
            2 => MeshColorMode::Quality,
            _ => MeshColorMode::Neutral,
        };
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let mesh_controls = state.clone();
    ui.on_mesh_quality(move |index| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_controls.borrow_mut();
        state.mesh_quality_metric = match index {
            1 => MeshQualityMetric::Skewness,
            2 => MeshQualityMetric::NonOrthogonality,
            3 => MeshQualityMetric::CellMeasure,
            _ => MeshQualityMetric::AspectRatio,
        };
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let mesh_threshold_state = state.clone();
    ui.on_mesh_threshold(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_threshold_state.borrow_mut();
        state.mesh_quality_threshold = parse_number(
            ui.get_mesh_quality_threshold().as_str(),
            state.mesh_quality_threshold,
        );
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let bad_cell_state = state.clone();
    ui.on_select_bad_cell(move |list_index| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = bad_cell_state.borrow_mut();
        let Some(cell_index) = state.workbench.mesh_render_cache().and_then(|cache| {
            cache
                .quality()
                .bad_cells(state.mesh_quality_metric, state.mesh_quality_threshold)
                .get(usize::try_from(list_index.max(0)).unwrap_or(usize::MAX))
                .copied()
        }) else {
            return;
        };
        let mesh_id = state.workbench.mesh().map(|mesh| mesh.mesh.id());
        if let Some(mesh_id) = mesh_id {
            state
                .workbench
                .set_mesh_selection(Some(MeshSelection::cell(mesh_id, cell_index)));
            state.mesh_pick_mode = MeshPickMode::Cell;
            refresh_ui(&ui, &state);
        }
    });
    let weak_ui = ui.as_weak();
    let mesh_hover_state = state.clone();
    ui.on_mesh_hover(move |x, y, width, height| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = mesh_hover_state.borrow_mut();
        if state.show_mesh {
            let screen = geometry_editor_point(x, y, width, height);
            let hover = state.workbench.mesh_render_cache().and_then(|cache| {
                match (cache.dimension(), state.mesh_pick_mode) {
                    (MeshDimension::TwoD, MeshPickMode::Face) => cache.pick_face_2d(
                        state.mesh_view.screen_to_world(screen),
                        8.0 / state.mesh_view.pixels_per_unit,
                    ),
                    (MeshDimension::TwoD, MeshPickMode::Cell) => {
                        cache.pick_cell_2d(state.mesh_view.screen_to_world(screen))
                    }
                    (MeshDimension::ThreeD, MeshPickMode::Face) => {
                        let (origin, direction) = mesh_screen_ray(screen);
                        cache.pick_face_3d(origin, direction)
                    }
                    (MeshDimension::ThreeD, MeshPickMode::Cell) => None,
                }
            });
            state.workbench.set_mesh_hover(hover);
            refresh_ui(&ui, &state);
        }
    });
}

fn apply_workflow_step(ui: &MainWindow, state: &mut AppState, step: i32) {
    sync_project_from_ui(ui, &mut state.project);
    let step = step.clamp(0, 4);
    ui.set_current_step(step);
    state.current_step = step as usize;
    state.selected_tree = None;
    state.wb_selected_targets.clear();
    ui.set_inspector_mode(inspector_mode_for(TREE_KIND_STAGE, step as usize) as i32);
    match step {
        0 | 2 => {
            state.show_geometry_3d = true;
            state.show_mesh = false;
            state.animation_playing = false;
        }
        1 => {
            state.show_geometry_3d = false;
            state.show_mesh = true;
            state.animation_playing = false;
        }
        4 => {
            state.show_geometry_3d = false;
            state.show_mesh = false;
            state.frame_index = state.frames.len().saturating_sub(1);
        }
        _ => {}
    }
    rebuild_tree_rows(state);
    refresh_ui(ui, state);
}

/// Rebuilds the Rust-side project tree rows from the session state.
fn rebuild_tree_rows(state: &mut AppState) {
    let bodies: Vec<u64> = state
        .workbench
        .geometry()
        .bodies()
        .map(|body| body.id.get())
        .collect();
    let faces: Vec<u64> = state
        .workbench
        .geometry()
        .faces()
        .map(|face| face.id.get())
        .collect();
    let vertices: Vec<u64> = state
        .workbench
        .geometry()
        .vertices()
        .map(|vertex| vertex.id.get())
        .collect();
    let edges: Vec<u64> = state
        .workbench
        .geometry()
        .edges()
        .map(|edge| edge.id.get())
        .collect();
    let mesh_cells = state
        .workbench
        .mesh()
        .map(|generated| generated.report.cell_count);
    let named_selections: Vec<(String, usize)> = state
        .workbench
        .named_selections()
        .iter()
        .map(|selection| (selection.name.clone(), selection.targets.len()))
        .collect();
    let patches: Vec<(String, bool)> = state
        .patch_names
        .iter()
        .map(|name| {
            (
                name.clone(),
                state.workbench.boundary_assignment(name).is_some(),
            )
        })
        .collect();
    let solved = matches!(
        state.workbench.status(),
        SolveStatus::Converged | SolveStatus::MaxIterations
    );
    state.tree_rows = build_project_tree_rows(
        &bodies,
        &faces,
        &vertices,
        &edges,
        mesh_cells,
        &named_selections,
        &patches,
        &status_label(&state.workbench),
        solved,
        state.current_step,
        state.selected_tree,
    );
}

fn find_geometry_target(
    session: &WorkbenchSession,
    _kind: i32,
    payload: i32,
) -> Option<GeometrySelectionTarget> {
    session
        .geometry()
        .vertices()
        .find(|vertex| vertex.id.get() == payload as u64)
        .map(|vertex| GeometrySelectionTarget::Vertex(vertex.id))
        .or_else(|| {
            session
                .geometry()
                .edges()
                .find(|edge| edge.id.get() == payload as u64)
                .map(|edge| GeometrySelectionTarget::Edge(edge.id))
        })
        .or_else(|| {
            session
                .geometry()
                .faces()
                .find(|face| face.id.get() == payload as u64)
                .map(|face| GeometrySelectionTarget::Face(face.id))
        })
        .or_else(|| {
            session
                .geometry()
                .bodies()
                .find(|body| body.id.get() == payload as u64)
                .map(|body| GeometrySelectionTarget::Body(body.id))
        })
}

fn tree_selection_for_target(target: GeometrySelectionTarget) -> Option<TreeSelection> {
    match target {
        GeometrySelectionTarget::Vertex(id) => Some(TreeSelection {
            kind: TREE_KIND_VERTEX,
            payload: id.get() as i32,
        }),
        GeometrySelectionTarget::Body(id) => Some(TreeSelection {
            kind: TREE_KIND_BODY,
            payload: id.get() as i32,
        }),
        GeometrySelectionTarget::Face(id) => Some(TreeSelection {
            kind: TREE_KIND_FACE,
            payload: id.get() as i32,
        }),
        GeometrySelectionTarget::Edge(id) => Some(TreeSelection {
            kind: TREE_KIND_EDGE,
            payload: id.get() as i32,
        }),
    }
}

fn selected_patch_index(selection: Option<TreeSelection>) -> Option<usize> {
    selection
        .filter(|entry| entry.kind == TREE_KIND_PATCH && entry.payload >= 0)
        .map(|entry| entry.payload as usize)
}

fn describe_targets(targets: &[GeometrySelectionTarget]) -> String {
    targets
        .iter()
        .map(|target| match target {
            GeometrySelectionTarget::Vertex(id) => format!("Vertex {}", id.get()),
            GeometrySelectionTarget::Edge(id) => format!("Edge {}", id.get()),
            GeometrySelectionTarget::Face(id) => format!("Face {}", id.get()),
            GeometrySelectionTarget::Body(id) => format!("Body {}", id.get()),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn geometry_target_inspector(
    topology: &flursys::GeometryTopology,
    target: GeometrySelectionTarget,
) -> String {
    match target {
        GeometrySelectionTarget::Vertex(id) => topology.vertex(id).map_or_else(
            || format!("Vertex {} (deleted)", id.get()),
            |vertex| {
                format!(
                    "Vertex {}\nX {:.6} · Y {:.6} · Z {:.6}",
                    id.get(),
                    vertex.position.x,
                    vertex.position.y,
                    vertex.position.z
                )
            },
        ),
        GeometrySelectionTarget::Edge(id) => topology.edge(id).map_or_else(
            || format!("Edge {} (deleted)", id.get()),
            |edge| match edge.geometry {
                flursys::EdgeGeometry::Line { start, end } => {
                    let length = topology
                        .vertex(start)
                        .zip(topology.vertex(end))
                        .map_or(0.0, |(a, b)| (b.position - a.position).norm());
                    format!(
                        "Edge {} · Line\nEndpoints: {} → {}\nLength {:.6}",
                        id.get(),
                        start.get(),
                        end.get(),
                        length
                    )
                }
                flursys::EdgeGeometry::CircularArc { start, center, end } => {
                    format!(
                        "Edge {} · Circular arc\nStart {} · Center {} · End {}",
                        id.get(),
                        start.get(),
                        center.get(),
                        end.get()
                    )
                }
            },
        ),
        GeometrySelectionTarget::Face(id) => topology.face(id).map_or_else(
            || format!("Face {} (deleted)", id.get()),
            |face| match &face.representation {
                flursys::GeometryFaceRepresentation::Planar {
                    outer_loop,
                    inner_loops,
                } => format!(
                    "Face {} · Planar\n{} outer edges · {} holes",
                    id.get(),
                    outer_loop.len(),
                    inner_loops.len()
                ),
                flursys::GeometryFaceRepresentation::PrimitiveSurface => {
                    format!("Face {} · Primitive surface", id.get())
                }
            },
        ),
        GeometrySelectionTarget::Body(id) => topology.body(id).map_or_else(
            || format!("Body {} (deleted)", id.get()),
            |body| format!("Body {}\n{} faces", id.get(), body.faces.len()),
        ),
    }
}

fn selected_named_selection_name(state: &AppState) -> Option<String> {
    let selection = state.selected_tree?;
    if selection.kind != TREE_KIND_NAMED_SELECTION {
        return None;
    }
    state
        .workbench
        .named_selections()
        .iter()
        .nth(usize::try_from(selection.payload.max(0)).unwrap_or(usize::MAX))
        .map(|named| named.name.clone())
}

fn selection_memberships(
    session: &WorkbenchSession,
    target: &GeometrySelectionTarget,
) -> Vec<String> {
    session
        .named_selections()
        .iter()
        .filter(|selection| selection.targets.contains(target))
        .map(|selection| selection.name.clone())
        .collect()
}

fn status_label(session: &WorkbenchSession) -> String {
    match session.status() {
        SolveStatus::Idle => "idle".to_string(),
        SolveStatus::Solving => "running".to_string(),
        SolveStatus::Converged => "converged".to_string(),
        SolveStatus::MaxIterations => "iteration limit".to_string(),
        SolveStatus::Failed(message) => format!("failed: {message}"),
    }
}

fn bc_kind_label(condition: IncompressibleBoundaryCondition) -> &'static str {
    match condition {
        IncompressibleBoundaryCondition::NoSlipWall => "No-slip wall",
        IncompressibleBoundaryCondition::MovingWall { .. } => "Moving wall",
        IncompressibleBoundaryCondition::VelocityInlet { .. } => "Velocity inlet",
        IncompressibleBoundaryCondition::PressureOutlet { .. } => "Pressure outlet",
    }
}

fn push_workbench_defaults(ui: &MainWindow, session: &WorkbenchSession) {
    let (global_size, min_size, max_size) = session.mesh_sizes();
    ui.set_mesh_dimension_index(if matches!(session.mesh_dimension(), MeshDimension::TwoD) {
        0
    } else {
        1
    });
    ui.set_wb_global_size(SharedString::from(format!("{global_size:.4}")));
    ui.set_wb_min_size(SharedString::from(format!("{min_size:.4}")));
    ui.set_wb_max_size(SharedString::from(format!("{max_size:.4}")));
    let material = session.material();
    ui.set_material_density(SharedString::from(format!("{}", material.density)));
    ui.set_material_viscosity(SharedString::from(format!(
        "{}",
        material.kinematic_viscosity
    )));
    let solver = session.solver_options();
    ui.set_wb_max_iterations(solver.max_outer_iterations as i32);
    ui.set_wb_velocity_relaxation(SharedString::from(format!(
        "{}",
        solver.velocity_relaxation
    )));
    ui.set_wb_pressure_relaxation(SharedString::from(format!(
        "{}",
        solver.pressure_relaxation
    )));
    ui.set_wb_continuity_tolerance(SharedString::from(format!(
        "{:e}",
        solver.continuity_absolute_tolerance
    )));
}

fn push_boundary_fields(ui: &MainWindow, session: &WorkbenchSession, patch: &str) {
    match session.boundary_assignment(patch).copied() {
        None => {
            ui.set_bc_kind_index(2);
            ui.set_bc_value_x(SharedString::from("0.1"));
            ui.set_bc_value_y(SharedString::from("0"));
            ui.set_bc_value_z(SharedString::from("0"));
            ui.set_bc_pressure(SharedString::from("0"));
            ui.set_bc_assigned(SharedString::from("Unassigned"));
        }
        Some(IncompressibleBoundaryCondition::NoSlipWall) => {
            ui.set_bc_kind_index(0);
            ui.set_bc_assigned(SharedString::from("Assigned: no-slip wall"));
        }
        Some(IncompressibleBoundaryCondition::MovingWall { velocity }) => {
            ui.set_bc_kind_index(1);
            set_velocity_fields(ui, velocity);
            ui.set_bc_assigned(SharedString::from("Assigned: moving wall"));
        }
        Some(IncompressibleBoundaryCondition::VelocityInlet { velocity }) => {
            ui.set_bc_kind_index(2);
            set_velocity_fields(ui, velocity);
            ui.set_bc_assigned(SharedString::from("Assigned: velocity inlet"));
        }
        Some(IncompressibleBoundaryCondition::PressureOutlet { pressure }) => {
            ui.set_bc_kind_index(3);
            ui.set_bc_pressure(SharedString::from(format!("{pressure:.4}")));
            ui.set_bc_assigned(SharedString::from("Assigned: pressure outlet"));
        }
    }
}

fn set_velocity_fields(ui: &MainWindow, velocity: Vec3) {
    ui.set_bc_value_x(SharedString::from(format!("{:.4}", velocity.x)));
    ui.set_bc_value_y(SharedString::from(format!("{:.4}", velocity.y)));
    ui.set_bc_value_z(SharedString::from(format!("{:.4}", velocity.z)));
}

/// Validates the material/SIMPLE panel inputs against the session.
fn sync_solver_panel(ui: &MainWindow, session: &mut WorkbenchSession) -> Result<(), String> {
    let density = parse_number(ui.get_material_density().as_str(), f64::NAN);
    let viscosity = parse_number(ui.get_material_viscosity().as_str(), f64::NAN);
    session
        .set_material(density, viscosity)
        .map_err(|error| format!("Invalid material: {error}"))?;
    session
        .set_solver_controls(
            ui.get_wb_max_iterations().max(1) as usize,
            parse_number(ui.get_wb_velocity_relaxation().as_str(), f64::NAN),
            parse_number(ui.get_wb_pressure_relaxation().as_str(), f64::NAN),
            parse_number(ui.get_wb_continuity_tolerance().as_str(), f64::NAN),
        )
        .map_err(|error| format!("Invalid solver controls: {error}"))?;
    Ok(())
}

fn mesh_inspector_summary(
    mesh: &flursys::UnstructuredMesh,
    cache: &flursys::MeshRenderCache,
    metric: MeshQualityMetric,
    threshold: f64,
) -> String {
    let quality = cache.quality();
    let (minimum, maximum) = quality.color_range(metric).unwrap_or((0.0, 0.0));
    let bad_cells = quality.bad_cells(metric, threshold);
    format!(
        "nodes     {:>8}\ncells     {:>8}\nfaces     {:>8}\npatches   {:>8}\n{}\nrange     {:.3e} .. {:.3e}\nthreshold {:.3e}\nbad cells {:>8}",
        mesh.points().len(),
        mesh.cell_count(),
        mesh.face_count(),
        mesh.boundary_patches().len(),
        metric.label(),
        minimum,
        maximum,
        threshold,
        bad_cells.len(),
    )
}

fn mesh_summary_text(session: &WorkbenchSession) -> String {
    let Some(generated) = session.mesh() else {
        return "No mesh generated yet.".to_string();
    };
    let statistics = generated.mesh.statistics();
    let quality = &statistics.quality;
    format!(
        "nodes     {:>8}\ncells     {:>8}\nfaces     {:>8}\npatches   {:>8}\norder     {:>8}\nnon-orth  {:>7.1} deg\nskewness  {:>8.3}\naspect    {:>8.2}",
        generated.report.node_count,
        statistics.cell_count,
        statistics.face_count,
        statistics.boundary_patches.len(),
        format!("#{}", session.element_order()),
        quality.max_non_orthogonality_degrees,
        quality.max_skewness,
        quality.max_aspect_ratio,
    )
}

fn solution_summary_text(session: &WorkbenchSession) -> String {
    match session.solution() {
        None => match session.status() {
            SolveStatus::Idle => "No solution yet.".to_string(),
            SolveStatus::Solving => "Solving… watch the log for progress.".to_string(),
            SolveStatus::Failed(message) => format!("Last solve failed:\n{message}"),
            SolveStatus::Converged | SolveStatus::MaxIterations => {
                "Solution is being summarised…".to_string()
            }
        },
        Some(solution) => {
            let report = &solution.report;
            let history: Vec<String> = report
                .continuity_history
                .iter()
                .rev()
                .take(6)
                .rev()
                .enumerate()
                .map(|(offset, value)| {
                    format!(
                        "  iter {:>5}  continuity {:.3e}",
                        report.outer_iterations.saturating_sub(5) + offset + 1,
                        value
                    )
                })
                .collect();
            format!(
                "status      {}\niterations  {}\ninitial ω   {:.3e}\nfinal ω     {:.3e}\ninflow      {:.5}\noutflow     {:.5}\nnet flux    {:+.3e}\n{}",
                status_label(session),
                report.outer_iterations,
                report.initial_continuity_rms,
                report.final_continuity_rms,
                report.total_inflow.abs(),
                report.total_outflow,
                report.net_boundary_flux,
                if history.is_empty() {
                    String::new()
                } else {
                    format!("recent continuity:\n{}", history.join("\n"))
                }
            )
        }
    }
}

fn next_copy_name(source: &str, parts: &[flursys::GeometryPart]) -> String {
    for copy_index in 1..=128 {
        let candidate = format!("{source} copy {copy_index}");
        if !parts.iter().any(|part| part.name == candidate) {
            return candidate;
        }
    }
    format!("{source} copy")
}

fn sketch_viewport_point(
    mouse_x: f32,
    mouse_y: f32,
    width: f32,
    height: f32,
) -> Option<(f64, f64)> {
    let (pixel_x, pixel_y) = preview_image_point(mouse_x, mouse_y, width, height)?;
    let scale = 140.0_f64;
    Some((
        (pixel_x - f64::from(PREVIEW_WIDTH) * 0.5) / scale,
        (f64::from(PREVIEW_HEIGHT) * 0.5 - pixel_y) / scale,
    ))
}

/// Maps the actual Slint viewport coordinates into the fixed-size editor
/// render buffer. The domain transform itself remains centralized in the
/// editor and is never duplicated in UI callbacks.
fn geometry_editor_point(x: f32, y: f32, width: f32, height: f32) -> (f64, f64) {
    let width = width.max(1.0);
    let height = height.max(1.0);
    (
        f64::from(x / width * PREVIEW_WIDTH as f32),
        f64::from(y / height * PREVIEW_HEIGHT as f32),
    )
}

/// Shared command path for the toolbar and Delete shortcut. A snapshot is
/// recorded once before the first successful deletion, so undo restores the
/// exact stable IDs present before the command.
fn delete_editor_selection(state: &mut AppState) {
    let targets = state.geometry_editor.selection.clone();
    if targets.is_empty() {
        state.log("Select geometry before deleting.");
        return;
    }
    let geometry = state.workbench.geometry().clone();
    state.geometry_editor.snapshot_before_delete(&geometry);
    let mut deleted = 0;
    for target in targets {
        match state.workbench.delete_geometry_target(target) {
            Ok(()) => deleted += 1,
            Err(error) => state.log(format!("Cannot delete {target:?}: {error}")),
        }
    }
    if deleted > 0 {
        state.geometry_editor.selection.clear();
        state.wb_selected_targets.clear();
        state.log(format!("Deleted {deleted} geometry entities."));
        rebuild_tree_rows(state);
    } else {
        state.geometry_editor.discard_last_undo_snapshot();
    }
}

fn snap_sketch_point((x, y): (f64, f64)) -> (f64, f64) {
    const GRID: f64 = 0.1;
    ((x / GRID).round() * GRID, (y / GRID).round() * GRID)
}

fn snap_to_existing_sketch_geometry(
    sketch: Option<&GeometrySketch>,
    pending_points: &[(f64, f64)],
    point: (f64, f64),
) -> (f64, f64) {
    const SNAP_DISTANCE: f64 = 0.15;
    let mut nearest: Option<((f64, f64), f64)> = None;
    if let Some(sketch) = sketch {
        for entity in &sketch.entities {
            let candidates: &[(f64, f64)] = match &entity.kind {
                SketchEntityKind::Line { x1, y1, x2, y2 } => &[(*x1, *y1), (*x2, *y2)],
                SketchEntityKind::Circle {
                    center_x, center_y, ..
                } => &[(*center_x, *center_y)],
            };
            for candidate in candidates {
                let distance = (candidate.0 - point.0).hypot(candidate.1 - point.1);
                if distance <= SNAP_DISTANCE
                    && nearest.is_none_or(|(_, best_distance)| distance < best_distance)
                {
                    nearest = Some((*candidate, distance));
                }
            }
        }
    }
    if let Some((candidate, _)) = nearest {
        return candidate;
    }
    let Some(&(anchor_x, anchor_y)) = pending_points.first() else {
        return point;
    };
    let x = if (point.0 - anchor_x).abs() <= SNAP_DISTANCE {
        anchor_x
    } else {
        point.0
    };
    let y = if (point.1 - anchor_y).abs() <= SNAP_DISTANCE {
        anchor_y
    } else {
        point.1
    };
    (x, y)
}

fn selected_entity_dimension(sketch: &GeometrySketch) -> Option<f64> {
    let id = sketch.selected_entity?;
    let entity = sketch.entities.iter().find(|entity| entity.id == id)?;
    match entity.kind {
        SketchEntityKind::Line { x1, y1, x2, y2 } => Some((x2 - x1).hypot(y2 - y1)),
        SketchEntityKind::Circle { radius, .. } => Some(radius),
    }
}

fn apply_sketch_click(state: &mut AppState, point: (f64, f64)) -> Result<(), String> {
    let Some(sketch) = &mut state.draft_sketch else {
        return Err("start a sketch before using sketch tools".to_string());
    };
    match state.sketch_tool {
        SketchTool::Select => {
            sketch.select_entity_near(point.0, point.1, 0.15);
            Ok(())
        }
        SketchTool::Trim => sketch.trim_line_near(point.0, point.1),
        SketchTool::Line
        | SketchTool::Rectangle
        | SketchTool::Square
        | SketchTool::Circle
        | SketchTool::Dimension => {
            state.sketch_points.push(point);
            if state.sketch_points.len() < 2 {
                return Ok(());
            }
            let (start, end) = (state.sketch_points[0], state.sketch_points[1]);
            state.sketch_points.clear();
            match state.sketch_tool {
                SketchTool::Line => sketch.add_line(start.0, start.1, end.0, end.1)?,
                SketchTool::Rectangle => {
                    let width = (end.0 - start.0).abs();
                    let height = (end.1 - start.1).abs();
                    sketch.add_rectangle(
                        (start.0 + end.0) * 0.5,
                        (start.1 + end.1) * 0.5,
                        width,
                        height,
                    )?;
                    sketch.profile = SketchProfileKind::Rectangle { width, height };
                }
                SketchTool::Square => {
                    let side = (end.0 - start.0).abs().max((end.1 - start.1).abs());
                    sketch.add_rectangle(
                        (start.0 + end.0) * 0.5,
                        (start.1 + end.1) * 0.5,
                        side,
                        side,
                    )?;
                    sketch.profile = SketchProfileKind::Rectangle {
                        width: side,
                        height: side,
                    };
                }
                SketchTool::Circle => {
                    let radius = (end.0 - start.0).hypot(end.1 - start.1);
                    sketch.add_circle(start.0, start.1, radius)?;
                    sketch.profile = SketchProfileKind::Circle { radius };
                }
                SketchTool::Dimension => {
                    sketch.add_distance_dimension(start.0, start.1, end.0, end.1)?
                }
                SketchTool::Select | SketchTool::Trim => unreachable!("handled above"),
            }
            Ok(())
        }
    }
}

fn sketch_tool_label(tool: SketchTool) -> &'static str {
    match tool {
        SketchTool::Select => "Select",
        SketchTool::Line => "Line: pick start then end",
        SketchTool::Rectangle => "Rectangle: pick two corners",
        SketchTool::Square => "Square: pick two opposite corners",
        SketchTool::Circle => "Circle: pick centre then radius",
        SketchTool::Dimension => "Dimension: pick two points",
        SketchTool::Trim => "Trim: click a line near an intersection",
    }
}

#[path = "flursys_gui/bindings.rs"]
mod bindings;
use bindings::*;
fn refresh_ui(ui: &MainWindow, state: &AppState) {
    ui.set_project_loaded(state.project_loaded);
    if !state.project_loaded {
        ui.set_geometry_parts_summary(SharedString::from(
            "No project is open. Choose a domain or load a .flursys.json project.",
        ));
        ui.set_visualization_title(SharedString::from("NO ACTIVE PROJECT"));
        ui.set_animation_status(SharedString::from(
            "Choose a domain or load a project before creating geometry.",
        ));
        ui.set_visualization_image(render_empty_preview());
        ui.set_log_text(SharedString::from(
            state.logs.iter().cloned().collect::<Vec<_>>().join("\n"),
        ));
        return;
    }
    ui.set_sketch_editing(state.show_sketch_editor && state.draft_sketch.is_some());
    ui.set_geometry_view_axis(state.geometry_view_axis);
    if let Some(sketch) = &state.draft_sketch {
        let dimensions = sketch
            .dimensions
            .iter()
            .map(|dimension| format!("{}={:.3} mm", dimension.name, dimension.value))
            .collect::<Vec<_>>()
            .join(" · ");
        ui.set_sketch_image(render_sketch_2d(
            sketch,
            state.sketch_points.first().copied().zip(state.sketch_hover),
        ));
        let pending_start = state
            .sketch_points
            .first()
            .map(|(x, y)| format!(" · start ({x:.2}, {y:.2}) mm → choose end"))
            .unwrap_or_default();
        ui.set_sketch_status(SharedString::from(format!(
            "{}{} · {} entities · {} constraints · active axis: {:?}{}",
            sketch_tool_label(state.sketch_tool),
            pending_start,
            sketch.entities.len(),
            sketch.constraints.len(),
            sketch.selected_axis,
            if dimensions.is_empty() {
                String::new()
            } else {
                format!(" · {dimensions}")
            },
        )));
    } else {
        ui.set_sketch_status(SharedString::from("Start a sketch to edit a 2D profile."));
    }
    ui.set_geometry_parts_summary(SharedString::from(geometry_parts_summary(&state.project)));
    ui.set_geometry_model_tree(SharedString::from(geometry_model_tree(&state.project)));
    ui.set_boundary_summary(SharedString::from(boundary_summary(&state.project)));
    ui.set_preflight_summary(SharedString::from(state.preflight_summary.as_str()));
    let update = state.last_update.as_ref();
    let solver_state = update.map_or(SolverState::Idle, |update| update.state);
    ui.set_status(SharedString::from(format!("{:?}", solver_state)));
    if let Some(update) = update {
        ui.set_solver_physical_time(update.physical_time as f32);
        ui.set_solver_time_step(update.time_step as f32);
        ui.set_solver_momentum_cfl(update.momentum_cfl as f32);
        ui.set_solver_viscous_number(update.viscous_diffusion_number as f32);
        ui.set_solver_max_speed(update.max_speed as f32);
        let time_label = match state.project.solver.coupling {
            ProjectCoupling::Projection => "Time",
            ProjectCoupling::Simple => "Pseudo time",
        };
        ui.set_solver_stability_summary(SharedString::from(format!(
            "{time_label:<10}{:>10.4e}\ndt         {:>10.4e}\nCFL        {:>10.4e}\nDν         {:>10.4e}\n|U|max     {:>10.4e}",
            update.physical_time,
            update.time_step,
            update.momentum_cfl,
            update.viscous_diffusion_number,
            update.max_speed,
        )));
        ui.set_residual_summary(SharedString::from(format!(
            "Iteration        {:>10}\nElapsed          {:>10.3} s\nLast iteration   {:>10.3} ms\nPressure PCG     {:>10} iters\nContinuity       {:>10.3e}\nMomentum         {:>10.3e}\nPressure         {:>10.3e}\nConverged        {}",
            update.iteration,
            update.elapsed_seconds,
            update.iteration_seconds * 1_000.0,
            update.pressure_iterations,
            update.continuity_residual,
            update.momentum_residual,
            update.pressure_residual,
            update.converged
        )));
        ui.set_force_summary(SharedString::from(format!(
            "Drag coefficient   {:>12.6}\nLift coefficient   {:>12.6}",
            update.drag_coefficient, update.lift_coefficient
        )));
        ui.set_continuity_level(residual_level(update.continuity_residual));
        ui.set_momentum_level(residual_level(update.momentum_residual));
        ui.set_pressure_level(residual_level(update.pressure_residual));
        if let Some(field) = &update.field_update {
            let max_speed = field.speed.iter().copied().fold(0.0_f64, f64::max);
            ui.set_field_summary(SharedString::from(format!(
                "Cell-centred field snapshot\nGrid: {} × {}\nMax speed: {:.6}\nPressure / vorticity samples: {} / {}\nTemperature: {}\nSolid cells: {}",
                field.nx,
                field.ny,
                max_speed,
                field.pressure.len(),
                field.vorticity.len(),
                if field.temperature.is_some() { "available" } else { "off" },
                field.solid.iter().filter(|solid| **solid).count()
            )));
        }
    }
    ui.set_log_text(SharedString::from(
        state.logs.iter().cloned().collect::<Vec<_>>().join("\n"),
    ));
    ui.set_residual_image(render_residual_chart(&state.residual_history));
    ui.set_mesh_inspection(SharedString::from(mesh_inspection(&state.project)));

    // ---- Workbench pipeline ----
    let tree_model: VecModel<ProjectTreeRow> = state
        .tree_rows
        .iter()
        .enumerate()
        .map(|(index, row)| ProjectTreeRow {
            index: index as i32,
            depth: row.depth as i32,
            label: SharedString::from(row.label.as_str()),
            note: SharedString::from(row.note.as_str()),
            kind: row.kind,
            payload: row.payload,
            active: row.active,
        })
        .collect();
    ui.set_project_tree_model(ModelRc::new(tree_model));

    let patch_items: Vec<SharedString> = state
        .patch_names
        .iter()
        .map(|name| SharedString::from(name.as_str()))
        .collect();
    ui.set_patch_model(ModelRc::new(VecModel::from(patch_items)));
    if !state.patch_names.is_empty() {
        ui.set_patch_index(
            ui.get_patch_index()
                .clamp(0, state.patch_names.len() as i32 - 1),
        );
    }

    let busy = state.meshing || state.solving;
    ui.set_wb_busy(busy);
    ui.set_wb_gmsh_status(SharedString::from(state.gmsh_status.as_str()));
    let mesh_summary = state
        .workbench
        .mesh()
        .zip(state.workbench.mesh_render_cache())
        .map_or_else(
            || mesh_summary_text(&state.workbench),
            |(generated, cache)| {
                mesh_inspector_summary(
                    &generated.mesh,
                    cache,
                    state.mesh_quality_metric,
                    state.mesh_quality_threshold,
                )
            },
        );
    ui.set_wb_mesh_summary(SharedString::from(mesh_summary));
    let bad_cell_indices = state
        .workbench
        .mesh_render_cache()
        .map(|cache| {
            cache
                .quality()
                .bad_cells(state.mesh_quality_metric, state.mesh_quality_threshold)
        })
        .unwrap_or_default();
    let bad_cell_items: Vec<SharedString> = bad_cell_indices
        .iter()
        .take(8)
        .map(|index| SharedString::from(format!("Cell {index}")))
        .collect();
    ui.set_bad_cell_model(ModelRc::new(VecModel::from(bad_cell_items)));
    ui.set_mesh_display_index(match state.mesh_display_mode {
        MeshDisplayMode::Wireframe => 0,
        MeshDisplayMode::Surface => 1,
        MeshDisplayMode::SurfaceEdges => 2,
    });
    ui.set_mesh_selection_index(match state.mesh_pick_mode {
        MeshPickMode::Face => 0,
        MeshPickMode::Cell => 1,
    });
    ui.set_wb_solution_summary(SharedString::from(solution_summary_text(&state.workbench)));
    ui.set_can_run_workbench(!busy && state.workbench.readiness().is_ok());
    ui.set_geometry_editor_image(render_geometry_editor(
        state.workbench.geometry(),
        &state.geometry_editor,
    ));
    ui.set_geometry_active_tool(match state.geometry_editor.active_tool {
        GeometryTool::Select => 0,
        GeometryTool::Line => 1,
        GeometryTool::Rectangle => 2,
        GeometryTool::Circle => 3,
    });
    ui.set_geometry_snap(state.geometry_editor.snap_enabled);
    ui.set_geometry_grid(state.geometry_editor.grid_enabled);
    ui.set_geometry_editor_status(SharedString::from(
        match state.geometry_editor.active_tool {
            GeometryTool::Select => match state.geometry_editor.hover_target {
                Some(target) => format!("Hover: {target:?} · click to select"),
                None if state.workbench.geometry().vertices().next().is_none() => {
                    "Start by drawing a Line, Rectangle, or Circle.".to_string()
                }
                None => "SELECT · click an entity; vertex > edge > face priority".to_string(),
            },
            GeometryTool::Line => "LINE · click start, then end · Esc cancels".to_string(),
            GeometryTool::Rectangle => {
                "RECTANGLE · click opposite corners · Esc cancels".to_string()
            }
            GeometryTool::Circle => {
                "CIRCLE · click centre then radius inside a planar face".to_string()
            }
        },
    ));

    match state.workbench.mesh() {
        Some(generated) => ui.set_status_cells(SharedString::from(format!(
            "{} cells",
            generated.report.cell_count
        ))),
        None => ui.set_status_cells(SharedString::from("— cells")),
    }
    if let Some(solution) = state.workbench.solution() {
        ui.set_status_iteration(SharedString::from(format!(
            "iter {}",
            solution.report.outer_iterations
        )));
        ui.set_status_continuity(SharedString::from(format!(
            "continuity {:.1e}",
            solution.report.final_continuity_rms
        )));
    } else if let Some(update) = update {
        ui.set_status_iteration(SharedString::from(format!("iter {}", update.iteration)));
        ui.set_status_continuity(SharedString::from(format!(
            "continuity {:.1e}",
            update.continuity_residual
        )));
    }

    let inspector_mode = ui.get_inspector_mode();
    ui.set_inspector_title(SharedString::from(match inspector_mode {
        0 => "GEOMETRY",
        1 => "NAMED SELECTIONS",
        2 => "MESH GENERATION",
        3 => "BOUNDARY CONDITIONS",
        4 => "SOLVER SETTINGS",
        _ => "RESULTS",
    }));
    let selected_target = state.selected_tree.and_then(|selection| {
        find_geometry_target(&state.workbench, selection.kind, selection.payload)
    });
    match (inspector_mode, selected_target) {
        (0, _) if state.workbench.mesh_selection().is_some() => {
            if let (Some(generated), Some(selection)) =
                (state.workbench.mesh(), state.workbench.mesh_selection())
            {
                let text = match selection.target() {
                    flursys::MeshSelectionTarget::Face(index) => generated.mesh.faces().get(index).map_or_else(
                        || format!("Mesh face {index} is no longer valid."),
                        |face| format!("MESH FACE {index}\nowner cell {}\nneighbour {}\ncentre {:.5}, {:.5}, {:.5}\nmeasure {:.5}\narea vector {:.5}, {:.5}, {:.5}", face.owner, face.neighbour.map_or_else(|| "boundary".to_string(), |value| value.to_string()), face.center.x, face.center.y, face.center.z, face.area, face.area_vector.x, face.area_vector.y, face.area_vector.z),
                    ),
                    flursys::MeshSelectionTarget::Cell(index) => generated.mesh.cells().get(index).map_or_else(
                        || format!("Mesh cell {index} is no longer valid."),
                        |cell| format!("MESH CELL {index}\ncentre {:.5}, {:.5}, {:.5}\nmeasure {:.5}\nfaces {}\nneighbours {}", cell.center.x, cell.center.y, cell.center.z, cell.volume, cell.faces.len(), cell.neighbours.len()),
                    ),
                };
                ui.set_inspector_title(SharedString::from("MESH ENTITY"));
                ui.set_inspector_info(SharedString::from(text));
            }
        }
        (_, Some(target)) if inspector_mode == 0 => {
            let members = selection_memberships(&state.workbench, &target);
            ui.set_ns_members(SharedString::from(if members.is_empty() {
                String::new()
            } else {
                members.join(", ")
            }));
            ui.set_inspector_info(SharedString::from(format!(
                "{}\nStable topology id — survives edits and feeds Named Selections.\n{} entities selected for the next group.",
                geometry_target_inspector(state.workbench.geometry(), target),
                state.wb_selected_targets.len()
            )));
        }
        (mode, _) => {
            let bodies = state.workbench.geometry().bodies().count();
            let faces = state.workbench.geometry().faces().count();
            let edges = state.workbench.geometry().edges().count();
            let overview = format!(
                "Fluid domain: {bodies} body · {faces} face · {edges} edges.\nPick a workflow stage on the left or in the top bar."
            );
            ui.set_inspector_info(SharedString::from(match mode {
                1 => "Group geometry entities into reusable Named Selections; each group becomes one Gmsh physical group and boundary patch.".to_string(),
                2 => "Generate the unstructured mesh with Gmsh once the Named Selections cover every exterior edge/face you need as a boundary patch.".to_string(),
                3 => "Assign physical conditions to every mesh patch; the run stays blocked until all patches are covered.".to_string(),
                4 => "Material properties and SIMPLE coupling controls for the unstructured solver.".to_string(),
                _ => overview,
            }));
            if mode != 1 {
                ui.set_ns_members(SharedString::from(""));
            }
        }
    }

    if state.show_geometry_3d && !state.show_sketch_editor {
        ui.set_visualization_title(SharedString::from("3D GEOMETRY & MESH"));
        let (length, height) = project_case_domain(&state.project.case);
        let base = StructuredMesh2D::new(
            state.project.solver.nx,
            state.project.solver.ny,
            length,
            height,
        )
        .expect("project domain and UI grid are validated before rendering");
        let mesh = ExtrudedMesh3D::new(
            base,
            state.project.preprocessing.mesh.cells_z,
            state.project.preprocessing.geometry.extrusion_depth,
        )
        .expect("project extrusion settings are validated before rendering");
        let z_exaggeration = preview_z_exaggeration(&mesh);
        ui.set_animation_status(SharedString::from(format!(
            "{} cells · {} layers · selected: {} · visual Z ×{:.1} · preview samples ≤28 × 28 × 16 · yaw {:.0}° · pitch {:.0}° · zoom {:.0}%",
            mesh.cell_count(),
            mesh.nz,
            state.selected_boundary_face.label(),
            z_exaggeration,
            state.geometry_yaw.to_degrees(),
            state.geometry_pitch.to_degrees(),
            state.geometry_zoom * 100.0,
        )));
        ui.set_visualization_image(render_geometry_3d(
            &state.project,
            state.geometry_yaw,
            state.geometry_pitch,
            state.geometry_zoom,
            Some(state.selected_boundary_face),
        ));
    } else if state.show_mesh {
        if let Some(generated) = state.workbench.mesh() {
            if let Some(cache) = state.workbench.mesh_render_cache() {
                ui.set_visualization_title(SharedString::from("UNSTRUCTURED MESH"));
                ui.set_animation_status(SharedString::from(format!(
                    "{} nodes · {} faces · {} cells · {} · {} · view {} · select {}",
                    generated.mesh.points().len(),
                    generated.mesh.face_count(),
                    generated.mesh.cell_count(),
                    state.mesh_quality_metric.label(),
                    match state.mesh_color_mode {
                        MeshColorMode::Neutral => "neutral",
                        MeshColorMode::Patches => "patches",
                        MeshColorMode::Quality => "quality",
                    },
                    state.mesh_display_mode.label(),
                    state.mesh_pick_mode.label(),
                )));
                ui.set_visualization_image(render_workbench_mesh(
                    &generated.mesh,
                    cache,
                    &state.mesh_view,
                    state.mesh_display_mode,
                    state.mesh_color_mode,
                    state.mesh_quality_metric,
                    state.workbench.mesh_selection(),
                    state.workbench.mesh_hover(),
                    selected_patch_index(state.selected_tree),
                ));
            }
        } else {
            ui.set_visualization_title(SharedString::from("NO MESH GENERATED"));
            ui.set_animation_status(SharedString::from(
                "Configure mesh settings and choose Generate Mesh.",
            ));
            ui.set_visualization_image(render_empty_image());
        }
    } else if let Some(field) = state.frames.get(state.frame_index) {
        let selected = ui.get_result_field_index();
        let plot = ui.get_result_plot_index().clamp(0, 3);
        let colormap = ui.get_result_colormap_index().clamp(0, 3);
        let contour_levels = ui.get_result_contour_levels().clamp(3, 32) as usize;
        let (title, image) = match selected {
            1 => (
                "PRESSURE FIELD",
                render_scalar_field(field, &field.pressure, true, colormap, plot, contour_levels),
            ),
            2 => (
                "VORTICITY FIELD",
                render_scalar_field(
                    field,
                    &field.vorticity,
                    true,
                    colormap,
                    plot,
                    contour_levels,
                ),
            ),
            3 => match &field.temperature {
                Some(temperature) => (
                    "TEMPERATURE FIELD",
                    render_scalar_field(field, temperature, false, colormap, plot, contour_levels),
                ),
                None => ("TEMPERATURE UNAVAILABLE", render_empty_image()),
            },
            _ => (
                "SPEED FIELD",
                render_scalar_field(field, &field.speed, false, colormap, plot, contour_levels),
            ),
        };
        ui.set_visualization_title(SharedString::from(title));
        ui.set_animation_status(SharedString::from(format!(
            "Frame {} / {} · {} · {} · {} levels{}{}",
            state.frame_index + 1,
            state.frames.len(),
            result_plot_label(plot),
            result_colormap_label(colormap),
            contour_levels,
            if state.animation_playing {
                " · playing"
            } else {
                ""
            },
            if selected == 3 && field.temperature.is_none() {
                " · enable Energy in Setup"
            } else {
                ""
            }
        )));
        ui.set_visualization_image(image);
    } else {
        ui.set_visualization_title(SharedString::from("SPEED FIELD"));
        ui.set_animation_status(SharedString::from("Frame 0 / 0"));
        ui.set_visualization_image(render_empty_image());
    }
}

const PREVIEW_WIDTH: u32 = 520;
const PREVIEW_HEIGHT: u32 = 320;

#[path = "flursys_gui/render.rs"]
mod render;
use render::*;
fn case_name(case: &flursys::ProjectCase) -> &'static str {
    match case {
        flursys::ProjectCase::LidDrivenCavity { .. } => "Lid-driven cavity",
        flursys::ProjectCase::Cylinder { .. } => "Cylinder flow",
        flursys::ProjectCase::BackwardFacingStep { .. } => "Backward-facing step",
        flursys::ProjectCase::Channel { .. } => "Channel (Poiseuille)",
    }
}

fn parse_number(value: &str, fallback: f64) -> f64 {
    value
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .unwrap_or(fallback)
}

fn residual_level(residual: f64) -> f32 {
    if !residual.is_finite() || residual <= 0.0 {
        return 0.0;
    }
    ((-residual.log10()).clamp(0.0, 10.0) / 10.0) as f32
}

fn geometry_view_angles(axis: i32) -> (f32, f32) {
    match axis {
        0 => (std::f32::consts::FRAC_PI_2, 0.0),
        1 => (0.0, 0.0),
        2 => (0.0, std::f32::consts::FRAC_PI_2),
        _ => (0.65, 0.48),
    }
}

fn geometry_view_label(axis: i32) -> &'static str {
    match axis {
        0 => "X-axis",
        1 => "Y-axis",
        2 => "Z-axis",
        _ => "isometric",
    }
}

fn result_plot_label(index: i32) -> &'static str {
    match index {
        1 => "contours",
        2 => "contours + vectors",
        3 => "vectors",
        _ => "filled field",
    }
}

fn result_colormap_label(index: i32) -> &'static str {
    match index {
        1 => "Viridis",
        2 => "Plasma",
        3 => "Blue-red",
        _ => "Turbo",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flursys::cases::{BackwardStepCase, CylinderCase};
    use flursys::{
        GeometryPart, GeometryPartKind, GeometrySketch, ProjectCase, SketchPlane, SketchProfileKind,
    };

    #[test]
    fn residual_indicators_are_bounded() {
        assert_eq!(residual_level(f64::NAN), 0.0);
        assert_eq!(residual_level(1.0), 0.0);
        assert_eq!(residual_level(1.0e-10), 1.0);
    }

    #[test]
    fn axis_views_map_to_stable_orthographic_camera_angles() {
        assert_eq!(geometry_view_angles(0), (std::f32::consts::FRAC_PI_2, 0.0));
        assert_eq!(geometry_view_angles(1), (0.0, 0.0));
        assert_eq!(geometry_view_angles(2), (0.0, std::f32::consts::FRAC_PI_2));
        assert_eq!(geometry_view_label(3), "isometric");
    }

    #[test]
    fn orientation_triad_endpoints_select_the_corresponding_axis() {
        let project = Project::default();
        let (length, domain_height) = project_case_domain(&project.case);
        let mesh = ExtrudedMesh3D::new(
            StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, domain_height)
                .unwrap(),
            project.preprocessing.mesh.cells_z,
            project.preprocessing.geometry.extrusion_depth,
        )
        .unwrap();
        let camera = MeshCamera::fit(&mesh, 0.65, 0.48, 1.0);
        for (axis, endpoint) in orientation_triad_endpoints(camera).into_iter().enumerate() {
            assert_eq!(
                pick_orientation_triad((f64::from(endpoint.0), f64::from(endpoint.1)), camera),
                Some(axis as i32)
            );
            assert_eq!(
                pick_orientation_axis_3d(
                    &project,
                    0.65,
                    0.48,
                    1.0,
                    (f64::from(endpoint.0), f64::from(endpoint.1)),
                ),
                Some(axis as i32)
            );
        }
    }

    #[test]
    fn sketch_viewport_mapping_accounts_for_contain_scale_and_letterboxing() {
        assert_eq!(
            sketch_viewport_point(800.0, 320.0, 1040.0, 640.0),
            Some((1.0, 0.0))
        );
        assert_eq!(sketch_viewport_point(260.0, 20.0, 520.0, 520.0), None);
    }

    #[test]
    fn gui_starts_with_a_new_editable_project() {
        let state = AppState::new();
        assert!(state.project_loaded);
        assert_eq!(state.project.name, "Lid-driven cavity");
    }

    #[test]
    fn speed_preview_accepts_a_cell_field() {
        let field = FieldUpdate {
            nx: 2,
            ny: 2,
            pressure: vec![0.0; 4],
            speed: vec![0.0, 0.5, 1.0, 0.25],
            velocity_x: vec![0.0, 0.5, 1.0, 0.25],
            velocity_y: vec![0.0; 4],
            vorticity: vec![0.0; 4],
            solid: vec![false, false, false, true],
            temperature: None,
        };
        for (colormap, plot) in [(0, 0), (1, 1), (2, 2), (3, 3)] {
            let _image = render_scalar_field(&field, &field.speed, false, colormap, plot, 8);
        }
    }

    #[test]
    fn geometry_preview_accepts_a_project_mesh() {
        let _image = render_geometry_3d(&Project::default(), 0.0, 0.0, 1.0, None);
    }

    #[test]
    fn geometry_preview_renders_parametric_parts() {
        let mut project = Project::default();
        project.preprocessing.geometry.parts.push(GeometryPart {
            name: "test-cylinder".to_string(),
            kind: GeometryPartKind::Cylinder {
                radius: 0.5,
                height: 1.0,
                segments: 32,
            },
            x: 0.0,
            y: 0.0,
            z: 0.5,
        });
        let _image = render_geometry_3d(&project, 0.35, -0.2, 1.1, Some(BoundaryFace::Top));
    }

    #[test]
    fn parametric_parts_follow_mesh_camera_pitch() {
        let project = Project::default();
        let (length, domain_height) = project_case_domain(&project.case);
        let mesh = ExtrudedMesh3D::new(
            StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, domain_height)
                .unwrap(),
            project.preprocessing.mesh.cells_z,
            project.preprocessing.geometry.extrusion_depth,
        )
        .unwrap();
        let part = GeometryPart {
            name: "camera-part".to_string(),
            kind: GeometryPartKind::Box {
                length: 0.4,
                width: 0.4,
                height: 0.4,
            },
            x: 0.5 * length,
            y: 0.5 * domain_height,
            z: 0.5 * mesh.depth,
        };
        let mut level = vec![0_u8; (PREVIEW_WIDTH * PREVIEW_HEIGHT * 4) as usize];
        let mut pitched = level.clone();
        draw_part_with_camera(
            &mut level,
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
            &part,
            MeshCamera::fit(&mesh, 0.4, 0.0, 1.0),
            part_color(0),
        );
        draw_part_with_camera(
            &mut pitched,
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
            &part,
            MeshCamera::fit(&mesh, 0.4, 0.7, 1.0),
            part_color(0),
        );
        assert!(level.chunks_exact(4).any(|pixel| pixel == part_color(0)));
        assert!(pitched.chunks_exact(4).any(|pixel| pixel == part_color(0)));
        assert_ne!(level, pitched);
    }

    #[test]
    fn geometry_model_tree_lists_sketches_features_and_solids() {
        let mut project = Project::default();
        project
            .preprocessing
            .geometry
            .add_sketch_feature(
                GeometrySketch::from_profile(
                    "inlet-profile".to_string(),
                    SketchPlane::Xy,
                    SketchProfileKind::Rectangle {
                        width: 2.0,
                        height: 1.0,
                    },
                    0.0,
                    0.0,
                    0.0,
                ),
                "inlet-extrude".to_string(),
                flursys::GeometryFeatureKind::Extrude { depth: 0.5 },
            )
            .unwrap();

        let tree = geometry_model_tree(&project);

        assert!(tree.contains("Sketches (1)"));
        assert!(tree.contains("inlet-profile [XY]"));
        assert!(tree.contains("Features (1)"));
        assert!(tree.contains("inlet-extrude [Extrude]"));
        assert!(tree.contains("Regions (6)"));
        assert!(tree.contains("inlet-extrude:side-4 [Side]"));
        assert!(tree.contains("Solids (1)"));
        assert!(tree.contains("inlet-extrude solid [Box]"));
    }

    #[test]
    fn geometry_preview_accepts_advanced_primitives() {
        let mut project = Project::default();
        for (name, kind) in [
            (
                "cone",
                GeometryPartKind::Cone {
                    radius: 0.4,
                    height: 1.0,
                    segments: 32,
                },
            ),
            (
                "sphere",
                GeometryPartKind::Sphere {
                    radius: 0.4,
                    segments: 24,
                },
            ),
            (
                "torus",
                GeometryPartKind::Torus {
                    major_radius: 0.6,
                    minor_radius: 0.2,
                    segments: 32,
                },
            ),
        ] {
            project.preprocessing.geometry.parts.push(GeometryPart {
                name: name.to_string(),
                kind,
                x: project.preprocessing.geometry.parts.len() as f64,
                y: 0.0,
                z: 0.5,
            });
        }
        let _image = render_geometry_3d(&project, 0.35, -0.2, 1.1, None);
    }

    #[test]
    fn sketch_canvas_renders_axes_and_construction_geometry() {
        let sketch = GeometrySketch::from_profile(
            "canvas".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Circle { radius: 0.5 },
            0.0,
            0.0,
            0.0,
        );
        let _image = render_sketch_2d(&sketch, Some(((0.0, 0.0), (1.0, 0.5))));
    }

    #[test]
    fn line_tool_creates_a_line_after_two_distinct_canvas_clicks() {
        let mut state = AppState::new();
        let mut sketch = GeometrySketch::from_profile(
            "line-test".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Rectangle {
                width: 1.0,
                height: 1.0,
            },
            0.0,
            0.0,
            0.0,
        );
        sketch.entities.clear();
        state.draft_sketch = Some(sketch);
        state.sketch_tool = SketchTool::Line;

        apply_sketch_click(&mut state, (-1.0, -0.5)).unwrap();
        apply_sketch_click(&mut state, (1.0, 0.5)).unwrap();

        assert!(matches!(
            state.draft_sketch.unwrap().entities.as_slice(),
            [flursys::SketchEntity {
                kind: flursys::SketchEntityKind::Line {
                    x1: -1.0,
                    y1: -0.5,
                    x2: 1.0,
                    y2: 0.5,
                },
                ..
            }]
        ));
    }

    #[test]
    fn sketch_snap_prefers_existing_endpoints_then_infers_axis_from_start() {
        let mut sketch = GeometrySketch::from_profile(
            "snap-test".to_string(),
            SketchPlane::Xy,
            SketchProfileKind::Rectangle {
                width: 1.0,
                height: 1.0,
            },
            0.0,
            0.0,
            0.0,
        );
        sketch.entities.clear();
        sketch.add_line(0.0, 0.0, 2.0, 0.0).unwrap();

        assert_eq!(
            snap_to_existing_sketch_geometry(Some(&sketch), &[], (2.1, 0.1)),
            (2.0, 0.0)
        );
        assert_eq!(
            snap_to_existing_sketch_geometry(Some(&sketch), &[(1.0, 1.0)], (1.1, 3.0)),
            (1.0, 3.0)
        );
    }

    #[test]
    fn case_obstacle_classification_matches_the_builtin_geometry() {
        let cylinder = ProjectCase::from(CylinderCase::default());
        assert!(project_case_is_solid(&cylinder, 5.0, 5.0));
        assert!(!project_case_is_solid(&cylinder, 0.1, 0.1));
        let step = ProjectCase::from(BackwardStepCase::default());
        assert!(project_case_is_solid(&step, 0.1, 0.1));
    }

    #[test]
    fn mesh_display_modes_are_distinct_and_surface_capable() {
        assert!(!MeshDisplayMode::Wireframe.draws_surface());
        assert!(MeshDisplayMode::Surface.draws_surface());
        assert!(MeshDisplayMode::SurfaceEdges.draws_surface());
        assert!(!MeshDisplayMode::Surface.draws_edges());
        assert!(MeshDisplayMode::SurfaceEdges.draws_edges());
    }

    #[test]
    fn mesh_selection_mode_keeps_face_and_cell_picking_unambiguous() {
        assert_eq!(MeshPickMode::Face.label(), "Face");
        assert_eq!(MeshPickMode::Cell.label(), "Cell");
    }

    #[test]
    fn mesh_inspector_summary_uses_installed_unstructured_mesh_quality() {
        let mesh = flursys::UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                flursys::Point::new(0.0, 0.0, 0.0),
                flursys::Point::new(1.0, 0.0, 0.0),
                flursys::Point::new(1.0, 1.0, 0.0),
                flursys::Point::new(0.0, 1.0, 0.0),
            ],
            vec![flursys::CellDefinition::polygon(vec![0, 1, 2, 3])],
        )
        .unwrap();
        let cache = flursys::MeshRenderCache::build(&mesh).unwrap();

        let summary = mesh_inspector_summary(&mesh, &cache, MeshQualityMetric::AspectRatio, 1.1);

        assert!(summary.contains("nodes"));
        assert!(summary.contains("cells"));
        assert!(summary.contains("Aspect ratio"));
        assert!(summary.contains("bad cells"));
        assert!(!summary.contains("dx / dy / dz"));
    }

    #[test]
    fn preflight_accepts_the_default_case() {
        assert!(preflight_report(&Project::default())
            .expect("the default case should be runnable")
            .contains("READY"));
    }

    #[test]
    fn boundary_summary_and_2d_picker_use_the_project_faces() {
        let project = Project::default();
        assert!(boundary_summary(&project).contains("Left"));
        assert_eq!(
            pick_boundary_2d(&project, (125.0, 160.0)),
            Some(BoundaryFace::Left)
        );
    }

    fn workbench_tree_fixture(patches: &[(String, bool)]) -> Vec<ProjectTreeRowData> {
        build_project_tree_rows(
            &[1],
            &[1],
            &[1, 2, 3, 4],
            &[1, 2, 3, 4],
            None,
            &[("inlet".to_string(), 1)],
            patches,
            "idle",
            false,
            0,
            None,
        )
    }

    #[test]
    fn project_tree_lists_stages_entities_groups_and_boundaries() {
        let rows =
            workbench_tree_fixture(&[("inlet".to_string(), true), ("outlet".to_string(), false)]);

        let labels: Vec<&str> = rows.iter().map(|row| row.label.as_str()).collect();
        assert_eq!(
            labels,
            vec![
                "Geometry",
                "Body 1",
                "Face 1",
                "Vertex 1",
                "Vertex 2",
                "Vertex 3",
                "Vertex 4",
                "Edge 1",
                "Edge 2",
                "Edge 3",
                "Edge 4",
                "Named Selections",
                "inlet",
                "Mesh",
                "Setup",
                "Boundaries",
                "inlet",
                "outlet",
                "Solution",
                "Results",
            ]
        );
        let inlet_patch = &rows[16];
        assert_eq!(inlet_patch.kind, TREE_KIND_PATCH);
        assert_eq!(inlet_patch.payload, 0);
        assert_eq!(inlet_patch.note, "Assigned");
        let outlet_patch = &rows[17];
        assert_eq!(outlet_patch.payload, 1);
        assert_eq!(outlet_patch.note, "Unassigned");
        // Nothing is selected and the current step is Geometry.
        assert!(rows[0].active);
        assert!(!rows.iter().skip(1).any(|row| row.active));
    }

    #[test]
    fn tree_selection_highlights_exactly_one_row_per_kind() {
        let rows = build_project_tree_rows(
            &[1],
            &[1],
            &[1, 2, 3, 4],
            &[1, 2, 3, 4],
            None,
            &[("inlet".to_string(), 1)],
            &[],
            "idle",
            false,
            0,
            Some(TreeSelection {
                kind: TREE_KIND_EDGE,
                payload: 2,
            }),
        );
        let active: Vec<&ProjectTreeRowData> = rows.iter().filter(|row| row.active).collect();
        assert_eq!(active.len(), 1);
        assert_eq!(active[0].label, "Edge 2");
    }

    #[test]
    fn inspector_mode_follows_stage_and_selection() {
        assert_eq!(inspector_mode_for(TREE_KIND_STAGE, 0), 0);
        assert_eq!(inspector_mode_for(TREE_KIND_STAGE, 1), 2);
        assert_eq!(inspector_mode_for(TREE_KIND_STAGE, 2), 3);
        assert_eq!(inspector_mode_for(TREE_KIND_STAGE, 3), 4);
        assert_eq!(inspector_mode_for(TREE_KIND_STAGE, 4), 5);
        assert_eq!(inspector_mode_for(TREE_KIND_EDGE, 2), 0);
        assert_eq!(inspector_mode_for(TREE_KIND_NAMED_SELECTION, 2), 1);
        assert_eq!(inspector_mode_for(TREE_KIND_PATCH, 2), 3);
    }

    #[test]
    fn demo_session_reports_unassigned_patches_in_the_project_tree() {
        let mut session = WorkbenchSession::demo_channel(4.0, 1.0)
            .expect("the built-in demo channel is a valid workflow");
        session.unassign_boundary("outlet");
        let bodies: Vec<u64> = session.geometry().bodies().map(|b| b.id.get()).collect();
        let faces: Vec<u64> = session.geometry().faces().map(|f| f.id.get()).collect();
        let vertices: Vec<u64> = session.geometry().vertices().map(|v| v.id.get()).collect();
        let edges: Vec<u64> = session.geometry().edges().map(|e| e.id.get()).collect();
        let named_selections: Vec<(String, usize)> = session
            .named_selections()
            .iter()
            .map(|selection| (selection.name.clone(), selection.targets.len()))
            .collect();
        let patches: Vec<(String, bool)> = ["inlet", "outlet", "walls"]
            .iter()
            .map(|name| {
                (
                    name.to_string(),
                    session.boundary_assignment(name).is_some(),
                )
            })
            .collect();
        let rows = build_project_tree_rows(
            &bodies,
            &faces,
            &vertices,
            &edges,
            None,
            &named_selections,
            &patches,
            "idle",
            false,
            0,
            None,
        );
        let outlet = rows
            .iter()
            .find(|row| row.kind == TREE_KIND_PATCH && row.label == "outlet")
            .expect("outlet patch listed");
        assert_eq!(outlet.note, "Unassigned");
        assert!(session.readiness().is_err());
    }
}
