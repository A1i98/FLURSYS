use flursys::runtime::{SolverCommand, SolverController, SolverState, SolverUpdate};
use flursys::{
    BoundaryConditionKind, BoundaryFace, ExtrudedMesh3D, FieldUpdate, Project, StructuredMesh2D,
    ThermalBoundaryCondition,
};
use slint::{ComponentHandle, SharedString, Timer, TimerMode};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

slint::include_modules!();
struct AppState {
    controller: SolverController,
    project: Project,
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
    geometry_drag_anchor: Option<(f32, f32, f32, f32)>,
    selected_boundary_face: BoundaryFace,
    preflight_summary: String,
    last_animation_tick: std::time::Instant,
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
            logs: VecDeque::from(["FLURSYS Slint workbench ready.".to_string()]),
            last_update: None,
            residual_history: VecDeque::new(),
            frames: VecDeque::new(),
            frame_index: 0,
            animation_playing: false,
            show_mesh: false,
            show_geometry_3d: true,
            geometry_yaw: 0.0,
            geometry_pitch: 0.0,
            geometry_zoom: 1.0,
            geometry_drag_anchor: None,
            selected_boundary_face: BoundaryFace::Left,
            preflight_summary: "Run validation before starting the solver.".to_string(),
            last_animation_tick: std::time::Instant::now(),
        }
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

fn main() -> Result<(), slint::PlatformError> {
    let ui = MainWindow::new()?;
    let state = Rc::new(RefCell::new(AppState::new()));
    write_project_to_ui(&ui, &state.borrow().project);
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
    let weak_ui = ui.as_weak();
    let workflow_state = state.clone();
    ui.on_select_step(move |step| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = workflow_state.borrow_mut();
        sync_project_from_ui(&ui, &mut state.project);
        let step = step.clamp(0, 4);
        ui.set_current_step(step);
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
        refresh_ui(&ui, &state);
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
        state.project.ensure_preprocessing_defaults();
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
    let parts_state = state.clone();
    ui.on_remove_last_part(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = parts_state.borrow_mut();
        if let Some(part) = state.project.preprocessing.geometry.parts.pop() {
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
        sync_project_from_ui(&ui, &mut state.project);
        state.show_geometry_3d = true;
        state.show_mesh = false;
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
        let new_gesture = state.geometry_drag_anchor.map_or(true, |(x, y, _, _)| {
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
    let reset_geometry_state = state.clone();
    ui.on_reset_geometry_view(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = reset_geometry_state.borrow_mut();
        state.geometry_yaw = 0.0;
        state.geometry_pitch = 0.0;
        state.geometry_zoom = 1.0;
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
}

#[path = "flursys_gui/bindings.rs"]
mod bindings;
use bindings::*;
fn refresh_ui(ui: &MainWindow, state: &AppState) {
    ui.set_geometry_parts_summary(SharedString::from(geometry_parts_summary(&state.project)));
    ui.set_boundary_summary(SharedString::from(boundary_summary(&state.project)));
    ui.set_preflight_summary(SharedString::from(state.preflight_summary.as_str()));
    let update = state.last_update.as_ref();
    let solver_state = update.map_or(SolverState::Idle, |update| update.state);
    ui.set_status(SharedString::from(format!("{:?}", solver_state)));
    if let Some(update) = update {
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
    if state.show_geometry_3d {
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
        ui.set_visualization_title(SharedString::from("MESH PREVIEW"));
        let (length, height) = project_case_domain(&state.project.case);
        let mesh = StructuredMesh2D::new(
            state.project.solver.nx,
            state.project.solver.ny,
            length,
            height,
        )
        .expect("project domain and UI grid are validated before rendering");
        ui.set_animation_status(SharedString::from(format!(
            "{} × {} · {} cells · dx {:.3e} · dy {:.3e} · aspect {:.3}",
            mesh.nx,
            mesh.ny,
            mesh.cell_count(),
            mesh.dx,
            mesh.dy,
            mesh.dx.max(mesh.dy) / mesh.dx.min(mesh.dy),
        )));
        ui.set_visualization_image(render_mesh(
            &state.project,
            Some(state.selected_boundary_face),
        ));
    } else if let Some(field) = state.frames.get(state.frame_index) {
        let selected = ui.get_result_field_index();
        let (title, image) = match selected {
            1 => (
                "PRESSURE FIELD",
                render_scalar_field(field, &field.pressure, true),
            ),
            2 => (
                "VORTICITY FIELD",
                render_scalar_field(field, &field.vorticity, true),
            ),
            3 => match &field.temperature {
                Some(temperature) => (
                    "TEMPERATURE FIELD",
                    render_scalar_field(field, temperature, false),
                ),
                None => ("TEMPERATURE UNAVAILABLE", render_empty_image()),
            },
            _ => ("SPEED FIELD", render_speed_field(field)),
        };
        ui.set_visualization_title(SharedString::from(title));
        ui.set_animation_status(SharedString::from(format!(
            "Frame {} / {}{}{}",
            state.frame_index + 1,
            state.frames.len(),
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

#[cfg(test)]
mod tests {
    use super::*;
    use flursys::cases::{BackwardStepCase, CylinderCase};
    use flursys::{GeometryPart, GeometryPartKind, ProjectCase};

    #[test]
    fn residual_indicators_are_bounded() {
        assert_eq!(residual_level(f64::NAN), 0.0);
        assert_eq!(residual_level(1.0), 0.0);
        assert_eq!(residual_level(1.0e-10), 1.0);
    }

    #[test]
    fn speed_preview_accepts_a_cell_field() {
        let field = FieldUpdate {
            nx: 2,
            ny: 2,
            pressure: vec![0.0; 4],
            speed: vec![0.0, 0.5, 1.0, 0.25],
            vorticity: vec![0.0; 4],
            solid: vec![false, false, false, true],
            temperature: None,
        };
        let _image = render_speed_field(&field);
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
    fn case_obstacle_classification_matches_the_builtin_geometry() {
        let cylinder = ProjectCase::from(CylinderCase::default());
        assert!(project_case_is_solid(&cylinder, 5.0, 5.0));
        assert!(!project_case_is_solid(&cylinder, 0.1, 0.1));
        let step = ProjectCase::from(BackwardStepCase::default());
        assert!(project_case_is_solid(&step, 0.1, 0.1));
    }

    #[test]
    fn mesh_inspection_reports_real_structured_metrics() {
        let summary = mesh_inspection(&Project::default());
        assert!(summary.contains("2D cells"));
        assert!(summary.contains("dx / dy / dz"));
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
}
