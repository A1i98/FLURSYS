use flursys::runtime::{SolverCommand, SolverController, SolverState, SolverUpdate};
use flursys::{FieldUpdate, Project, ProjectCoupling, ProjectPressureSolver};
use slint::{
    ComponentHandle, Image, Rgba8Pixel, SharedPixelBuffer, SharedString, Timer, TimerMode,
};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

slint::slint! {
    import { Button, ComboBox, LineEdit, SpinBox, TextEdit } from "std-widgets.slint";

    component PanelTitle inherits Text {
        color: rgb(219, 231, 242);
        font-size: 14px;
        font-weight: 700;
    }

    component MetricCard inherits Rectangle {
        in property <string> label;
        in property <string> value;
        background: rgb(23, 35, 46);
        border-color: rgb(48, 72, 90);
        border-width: 1px;
        border-radius: 3px;
        VerticalLayout {
            padding: 8px;
            Text { text: root.label; color: rgb(131, 160, 182); font-size: 11px; }
            Text { text: root.value; color: rgb(240, 195, 109); font-size: 18px; font-weight: 700; }
        }
    }

    export component MainWindow inherits Window {
        title: "FLURSYS | CFD Workbench";
        width: 1440px;
        height: 900px;
        background: rgb(13, 21, 28);

        in-out property <string> status: "Idle";
        in-out property <string> project-path: "case.flursys.json";
        in-out property <string> project-name: "Lid-driven cavity";
        in-out property <string> case-name: "Lid-driven cavity";
        in-out property <int> nx: 64;
        in-out property <int> ny: 64;
        in-out property <string> dt-text: "0.001";
        in-out property <int> iterations: 10000;
        in-out property <int> threads: 0;
        in-out property <int> coupling-index: 0;
        in-out property <int> pressure-solver-index: 0;
        in-out property <string> velocity-relaxation: "0.7";
        in-out property <string> pressure-relaxation: "0.3";
        in-out property <string> residual-summary: "Waiting for a solver run.";
        in-out property <image> residual-image;
        in-out property <float> continuity-level: 0.0;
        in-out property <float> momentum-level: 0.0;
        in-out property <float> pressure-level: 0.0;
        in-out property <string> force-summary: "Cd: —\nCl: —";
        in-out property <string> field-summary: "No field snapshot received.";
        in-out property <image> visualization-image;
        in-out property <string> visualization-title: "MESH PREVIEW";
        in-out property <string> animation-status: "Frame 0 / 0";
        in-out property <string> log-text: "FLURSYS Slint workbench ready.";

        callback start();
        callback pause();
        callback resume();
        callback stop();
        callback load-project();
        callback save-project();
        callback show-mesh();
        callback show-field();
        callback animation-play-pause();
        callback animation-next();

        VerticalLayout {
            spacing: 0px;
            Rectangle {
                height: 56px;
                background: rgb(22, 38, 51);
                border-color: rgb(49, 78, 98);
                border-width: 1px;
                HorizontalLayout {
                    padding: 12px;
                    spacing: 10px;
                    Text { text: "FLURSYS"; color: rgb(232, 241, 248); font-size: 22px; font-weight: 800; }
                    Rectangle { width: 1px; background: rgb(58, 83, 99); }
                    Text { text: "CFD WORKBENCH"; color: rgb(131, 160, 182); font-size: 12px; }
                    Rectangle { horizontal-stretch: 1; }
                    Text { text: "SOLVER STATUS"; color: rgb(131, 160, 182); font-size: 11px; }
                    Text { text: root.status; color: rgb(240, 195, 109); font-size: 13px; font-weight: 700; }
                    Button { text: "START"; clicked => { root.start(); } }
                    Button { text: "PAUSE"; clicked => { root.pause(); } }
                    Button { text: "RESUME"; clicked => { root.resume(); } }
                    Button { text: "STOP"; clicked => { root.stop(); } }
                }
            }

            HorizontalLayout {
                spacing: 8px;
                padding: 8px;
                Rectangle {
                    width: 320px;
                    background: rgb(17, 29, 38);
                    border-color: rgb(41, 66, 82);
                    border-width: 1px;
                    VerticalLayout {
                        padding: 12px;
                        spacing: 9px;
                        PanelTitle { text: "PROJECT & CASE"; }
                        Text { text: "Project file"; color: rgb(131, 160, 182); font-size: 11px; }
                        LineEdit { text <=> root.project-path; }
                        HorizontalLayout {
                            Button { text: "LOAD"; clicked => { root.load-project(); } }
                            Button { text: "SAVE"; clicked => { root.save-project(); } }
                        }
                        Text { text: "Project name"; color: rgb(131, 160, 182); font-size: 11px; }
                        LineEdit { text <=> root.project-name; }
                        Text { text: "Case"; color: rgb(131, 160, 182); font-size: 11px; }
                        Rectangle {
                            height: 34px;
                            background: rgb(23, 43, 56);
                            border-color: rgb(54, 89, 108);
                            border-width: 1px;
                            Text { text: root.case-name; color: rgb(219, 231, 242); vertical-alignment: center; }
                        }
                        Text { text: "Imported cases retain their geometry and boundary data. Use a project file to define and exchange cases."; color: rgb(131, 160, 182); font-size: 11px; wrap: word-wrap; }
                        Rectangle { height: 1px; background: rgb(41, 66, 82); }
                        PanelTitle { text: "MESH & SOLVER"; }
                        HorizontalLayout {
                            Text { text: "Nx"; color: rgb(169, 189, 204); vertical-alignment: center; }
                            SpinBox { value <=> root.nx; minimum: 4; maximum: 1024; }
                            Text { text: "Ny"; color: rgb(169, 189, 204); vertical-alignment: center; }
                            SpinBox { value <=> root.ny; minimum: 4; maximum: 1024; }
                        }
                        Text { text: "Pseudo-time step"; color: rgb(131, 160, 182); font-size: 11px; }
                        LineEdit { text <=> root.dt-text; }
                        Text { text: "Iterations"; color: rgb(131, 160, 182); font-size: 11px; }
                        SpinBox { value <=> root.iterations; minimum: 1; maximum: 10000000; }
                        Text { text: "CPU threads (0 = auto)"; color: rgb(131, 160, 182); font-size: 11px; }
                        SpinBox { value <=> root.threads; minimum: 0; maximum: 256; }
                        Text { text: "Coupling"; color: rgb(131, 160, 182); font-size: 11px; }
                        ComboBox { model: ["SIMPLE-style steady", "Projection transient"]; current-index <=> root.coupling-index; }
                        Text { text: "Pressure solver"; color: rgb(131, 160, 182); font-size: 11px; }
                        ComboBox { model: ["PCG + Jacobi", "SOR"]; current-index <=> root.pressure-solver-index; }
                    }
                }

                VerticalLayout {
                    spacing: 8px;
                    Rectangle {
                        vertical-stretch: 3;
                        background: rgb(17, 29, 38);
                        border-color: rgb(41, 66, 82);
                        border-width: 1px;
                        VerticalLayout {
                            padding: 12px;
                            PanelTitle { text: "SOLUTION MONITOR"; }
                            Text { text: "Residual monitor"; color: rgb(131, 160, 182); font-size: 12px; }
                            Rectangle {
                                vertical-stretch: 1;
                                background: rgb(11, 18, 24);
                                border-color: rgb(35, 59, 74);
                                border-width: 1px;
                                Image { source: root.residual-image; width: parent.width; height: parent.height; image-fit: contain; }
                            }
                            Text { text: "CONTINUITY"; color: rgb(131, 160, 182); font-size: 10px; }
                            Rectangle {
                                height: 7px; background: rgb(8, 14, 19);
                                Rectangle { width: parent.width * root.continuity-level; background: rgb(77, 168, 184); }
                            }
                            Text { text: "MOMENTUM"; color: rgb(131, 160, 182); font-size: 10px; }
                            Rectangle {
                                height: 7px; background: rgb(8, 14, 19);
                                Rectangle { width: parent.width * root.momentum-level; background: rgb(116, 189, 135); }
                            }
                            Text { text: "PRESSURE"; color: rgb(131, 160, 182); font-size: 10px; }
                            Rectangle {
                                height: 7px; background: rgb(8, 14, 19);
                                Rectangle { width: parent.width * root.pressure-level; background: rgb(240, 195, 109); }
                            }
                            Text { text: "The worker publishes sampled diagnostics; rendering remains independent from numerical iterations."; color: rgb(111, 138, 157); font-size: 11px; }
                        }
                    }
                    Rectangle {
                        vertical-stretch: 2;
                        background: rgb(17, 29, 38);
                        border-color: rgb(41, 66, 82);
                        border-width: 1px;
                        VerticalLayout {
                            padding: 12px;
                            PanelTitle { text: "CONVERGENCE & FORCES"; }
                            HorizontalLayout {
                                MetricCard { label: "CONTINUITY"; value: root.status == "Running" ? "LIVE" : "STANDBY"; }
                                MetricCard { label: "ITERATION"; value: root.iterations; }
                                MetricCard { label: "WORKERS"; value: root.threads == 0 ? "AUTO" : root.threads; }
                            }
                            Text { text: root.force-summary; color: rgb(214, 227, 236); font-family: "monospace"; wrap: word-wrap; }
                        }
                    }
                }

                Rectangle {
                    width: 330px;
                    background: rgb(17, 29, 38);
                    border-color: rgb(41, 66, 82);
                    border-width: 1px;
                    VerticalLayout {
                        padding: 12px;
                        spacing: 9px;
                        PanelTitle { text: "FIELD VIEW"; }
                        Rectangle {
                            height: 260px;
                            background: rgb(11, 18, 24);
                            border-color: rgb(35, 59, 74);
                            border-width: 1px;
                            Image { source: root.visualization-image; width: parent.width; height: parent.height; image-fit: contain; }
                        }
                        Text { text: root.visualization-title + " · " + root.animation-status; color: rgb(169, 197, 213); font-size: 11px; }
                        HorizontalLayout {
                            Button { text: "MESH"; clicked => { root.show-mesh(); } }
                            Button { text: "FIELD"; clicked => { root.show-field(); } }
                            Button { text: "PLAY / PAUSE"; clicked => { root.animation-play-pause(); } }
                            Button { text: "NEXT"; clicked => { root.animation-next(); } }
                        }
                        Text { text: root.field-summary; color: rgb(111, 138, 157); font-size: 11px; wrap: word-wrap; }
                        PanelTitle { text: "SOLVER LOG"; }
                        TextEdit { text: root.log-text; read-only: true; vertical-stretch: 1; }
                    }
                }
            }
        }
    }
}

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
            show_mesh: true,
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
    let start_state = state.clone();
    ui.on_start(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = start_state.borrow_mut();
        sync_project_from_ui(&ui, &mut state.project);
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
}

fn sync_project_from_ui(ui: &MainWindow, project: &mut Project) {
    project.name = ui.get_project_name().to_string();
    project.solver.nx = ui.get_nx().max(4) as usize;
    project.solver.ny = ui.get_ny().max(4) as usize;
    project.solver.dt = parse_number(ui.get_dt_text().as_str(), project.solver.dt);
    project.solver.max_iterations = ui.get_iterations().max(1) as usize;
    project.solver.threads = ui.get_threads().max(0) as usize;
    project.solver.coupling = if ui.get_coupling_index() == 0 {
        ProjectCoupling::Simple
    } else {
        ProjectCoupling::Projection
    };
    project.solver.pressure_solver = if ui.get_pressure_solver_index() == 0 {
        ProjectPressureSolver::Pcg
    } else {
        ProjectPressureSolver::Sor
    };
    project.solver.velocity_relaxation = parse_number(
        ui.get_velocity_relaxation().as_str(),
        project.solver.velocity_relaxation,
    );
    project.solver.pressure_relaxation = parse_number(
        ui.get_pressure_relaxation().as_str(),
        project.solver.pressure_relaxation,
    );
}

fn write_project_to_ui(ui: &MainWindow, project: &Project) {
    ui.set_project_name(SharedString::from(project.name.as_str()));
    ui.set_case_name(SharedString::from(case_name(&project.case)));
    ui.set_nx(project.solver.nx as i32);
    ui.set_ny(project.solver.ny as i32);
    ui.set_dt_text(SharedString::from(format!("{:.6}", project.solver.dt)));
    ui.set_iterations(project.solver.max_iterations as i32);
    ui.set_threads(project.solver.threads as i32);
    ui.set_coupling_index(match project.solver.coupling {
        ProjectCoupling::Simple => 0,
        ProjectCoupling::Projection => 1,
    });
    ui.set_pressure_solver_index(match project.solver.pressure_solver {
        ProjectPressureSolver::Pcg => 0,
        ProjectPressureSolver::Sor => 1,
    });
    ui.set_velocity_relaxation(SharedString::from(format!(
        "{:.3}",
        project.solver.velocity_relaxation
    )));
    ui.set_pressure_relaxation(SharedString::from(format!(
        "{:.3}",
        project.solver.pressure_relaxation
    )));
}

fn refresh_ui(ui: &MainWindow, state: &AppState) {
    let update = state.last_update.as_ref();
    let solver_state = update.map_or(SolverState::Idle, |update| update.state);
    ui.set_status(SharedString::from(format!("{:?}", solver_state)));
    if let Some(update) = update {
        ui.set_residual_summary(SharedString::from(format!(
            "Iteration        {:>10}\nElapsed          {:>10.3} s\nContinuity       {:>10.3e}\nMomentum         {:>10.3e}\nPressure         {:>10.3e}\nConverged        {}",
            update.iteration,
            update.elapsed_seconds,
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
                "Cell-centred field snapshot\nGrid: {} × {}\nMax speed: {:.6}\nPressure samples: {}\nSolid cells: {}",
                field.nx,
                field.ny,
                max_speed,
                field.pressure.len(),
                field.solid.iter().filter(|solid| **solid).count()
            )));
        }
    }
    ui.set_log_text(SharedString::from(
        state.logs.iter().cloned().collect::<Vec<_>>().join("\n"),
    ));
    ui.set_residual_image(render_residual_chart(&state.residual_history));
    if state.show_mesh {
        ui.set_visualization_title(SharedString::from("MESH PREVIEW"));
        ui.set_animation_status(SharedString::from(format!(
            "{} × {} structured grid",
            state.project.solver.nx, state.project.solver.ny
        )));
        ui.set_visualization_image(render_mesh(
            state.project.solver.nx,
            state.project.solver.ny,
        ));
    } else if let Some(field) = state.frames.get(state.frame_index) {
        ui.set_visualization_title(SharedString::from("SPEED FIELD"));
        ui.set_animation_status(SharedString::from(format!(
            "Frame {} / {}{}",
            state.frame_index + 1,
            state.frames.len(),
            if state.animation_playing {
                " · playing"
            } else {
                ""
            }
        )));
        ui.set_visualization_image(render_speed_field(field));
    } else {
        ui.set_visualization_title(SharedString::from("SPEED FIELD"));
        ui.set_animation_status(SharedString::from("Frame 0 / 0"));
        ui.set_visualization_image(render_empty_image());
    }
}

const PREVIEW_WIDTH: u32 = 520;
const PREVIEW_HEIGHT: u32 = 320;

fn render_empty_image() -> Image {
    let mut pixels = vec![0_u8; (PREVIEW_WIDTH * PREVIEW_HEIGHT * 4) as usize];
    fill(&mut pixels, [11, 18, 24, 255]);
    image_from_rgba(PREVIEW_WIDTH, PREVIEW_HEIGHT, pixels)
}

fn render_mesh(nx: usize, ny: usize) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [11, 18, 24, 255]);
    let columns = nx.clamp(4, 80);
    let rows = ny.clamp(4, 80);
    for x in 0..width {
        let line = x % (width / columns as u32).max(1) == 0;
        for y in 0..height {
            if line || y % (height / rows as u32).max(1) == 0 {
                set_pixel(&mut pixels, width, height, x, y, [52, 87, 106, 255]);
            }
        }
    }
    image_from_rgba(width, height, pixels)
}

fn render_speed_field(field: &FieldUpdate) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let max_speed = field.speed.iter().copied().fold(1.0e-12_f64, f64::max);
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    for y in 0..height {
        let j = ((height - 1 - y) as usize * field.ny / height as usize).min(field.ny - 1);
        for x in 0..width {
            let i = (x as usize * field.nx / width as usize).min(field.nx - 1);
            let index = i + field.nx * j;
            let color = if field.solid[index] {
                [22, 25, 29, 255]
            } else {
                speed_color((field.speed[index] / max_speed).clamp(0.0, 1.0) as f32)
            };
            set_pixel(&mut pixels, width, height, x, y, color);
        }
    }
    image_from_rgba(width, height, pixels)
}

fn render_residual_chart(history: &VecDeque<ResidualSample>) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [11, 18, 24, 255]);
    for divisor in 1..5 {
        let y = height * divisor / 5;
        for x in 0..width {
            set_pixel(&mut pixels, width, height, x, y, [29, 48, 61, 255]);
        }
    }
    if history.len() > 1 {
        draw_series(
            &mut pixels,
            width,
            height,
            history,
            |sample| sample.continuity,
            [77, 168, 184, 255],
        );
        draw_series(
            &mut pixels,
            width,
            height,
            history,
            |sample| sample.momentum,
            [116, 189, 135, 255],
        );
        draw_series(
            &mut pixels,
            width,
            height,
            history,
            |sample| sample.pressure,
            [240, 195, 109, 255],
        );
    }
    image_from_rgba(width, height, pixels)
}

fn draw_series(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    history: &VecDeque<ResidualSample>,
    value: impl Fn(&ResidualSample) -> f64,
    color: [u8; 4],
) {
    let count = history.len() - 1;
    let mut previous = None;
    for (index, sample) in history.iter().enumerate() {
        let x = (index as u32 * (width - 1) / count as u32) as i32;
        let level = residual_level(value(sample));
        let y = ((1.0 - level) * (height - 1) as f32) as i32;
        if let Some((old_x, old_y)) = previous {
            draw_line(pixels, width, height, (old_x, old_y), (x, y), color);
        }
        previous = Some((x, y));
    }
}

fn draw_line(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    start: (i32, i32),
    end: (i32, i32),
    color: [u8; 4],
) {
    let (mut x0, mut y0) = start;
    let (x1, y1) = end;
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut error = dx + dy;
    loop {
        if x0 >= 0 && y0 >= 0 {
            set_pixel(pixels, width, height, x0 as u32, y0 as u32, color);
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let twice_error = 2 * error;
        if twice_error >= dy {
            error += dy;
            x0 += sx;
        }
        if twice_error <= dx {
            error += dx;
            y0 += sy;
        }
    }
}

fn speed_color(value: f32) -> [u8; 4] {
    let r = (255.0 * value.sqrt()) as u8;
    let g = (255.0 * (1.0 - (2.0 * value - 1.0).abs())) as u8;
    let b = (255.0 * (1.0 - value).sqrt()) as u8;
    [r, g, b, 255]
}

fn fill(pixels: &mut [u8], color: [u8; 4]) {
    for pixel in pixels.chunks_exact_mut(4) {
        pixel.copy_from_slice(&color);
    }
}

fn set_pixel(pixels: &mut [u8], width: u32, height: u32, x: u32, y: u32, color: [u8; 4]) {
    if x >= width || y >= height {
        return;
    }
    let index = ((y * width + x) * 4) as usize;
    pixels[index..index + 4].copy_from_slice(&color);
}

fn image_from_rgba(width: u32, height: u32, pixels: Vec<u8>) -> Image {
    let mut buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    buffer.make_mut_bytes().copy_from_slice(&pixels);
    Image::from_rgba8(buffer)
}

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
            solid: vec![false, false, false, true],
        };
        let _image = render_speed_field(&field);
    }
}
