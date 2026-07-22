use flursys::cases::{BackwardStepCase, CavityCase, CylinderCase};
use flursys::runtime::{SolverCommand, SolverController, SolverState, SolverUpdate};
use flursys::{
    BoundaryConditionKind, BoundaryFace, FieldUpdate, GeometryPart, GeometryPartKind, Project,
    ProjectCase, ProjectCoupling, ProjectPressureSolver,
};
use slint::{
    ComponentHandle, Image, Rgba8Pixel, SharedPixelBuffer, SharedString, Timer, TimerMode,
};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::time::Duration;

slint::slint! {
    import { Button, ComboBox, LineEdit, SpinBox, TextEdit } from "std-widgets.slint";

    component SectionTitle inherits Text {
        color: rgb(228, 238, 246);
        font-size: 22px;
        font-weight: 700;
    }

    component SectionHint inherits Text {
        color: rgb(135, 160, 178);
        font-size: 12px;
        wrap: word-wrap;
    }

    component Card inherits Rectangle {
        background: rgb(20, 34, 45);
        border-color: rgb(48, 74, 92);
        border-width: 1px;
        border-radius: 5px;
    }

    component MetricCard inherits Card {
        in property <string> label;
        in property <string> value;
        VerticalLayout {
            padding: 12px;
            Text { text: root.label; color: rgb(135, 160, 178); font-size: 11px; }
            Text { text: root.value; color: rgb(240, 195, 109); font-size: 20px; font-weight: 700; }
        }
    }

    component WorkflowItem inherits Rectangle {
        in property <string> number;
        in property <string> title;
        in property <string> detail;
        in property <bool> active;
        callback select();
        height: 66px;
        background: root.active ? rgb(30, 61, 78) : rgb(17, 29, 38);
        border-color: root.active ? rgb(77, 168, 184) : rgb(36, 58, 73);
        border-width: 1px;
        border-radius: 4px;
        HorizontalLayout {
            padding: 10px;
            spacing: 10px;
            Text { text: root.number; color: root.active ? rgb(240, 195, 109) : rgb(109, 137, 155); font-size: 17px; font-weight: 800; vertical-alignment: center; }
            VerticalLayout {
                Text { text: root.title; color: root.active ? rgb(232, 241, 248) : rgb(180, 199, 211); font-size: 13px; font-weight: 700; }
                Text { text: root.detail; color: rgb(123, 151, 169); font-size: 10px; }
            }
        }
        TouchArea { clicked => { root.select(); } }
    }

    export component MainWindow inherits Window {
        title: "FLURSYS | CFD Workbench";
        width: 1440px;
        height: 900px;
        background: rgb(11, 19, 26);

        in-out property <int> current-step: 0;
        in-out property <int> case-index: 0;
        in-out property <int> part-kind-index: 0;
        in-out property <string> part-name: "Part 1";
        in-out property <string> part-size-x: "1.0";
        in-out property <string> part-size-y: "1.0";
        in-out property <string> part-size-z: "1.0";
        in-out property <string> part-pos-x: "0.0";
        in-out property <string> part-pos-y: "0.0";
        in-out property <string> part-pos-z: "0.0";
        in-out property <string> geometry-parts-summary: "No custom solids in this project.";
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
        in-out property <string> extrusion-depth: "0.25";
        in-out property <int> mesh-nz: 8;
        in-out property <int> boundary-face-index: 0;
        in-out property <int> boundary-kind-index: 0;
        in-out property <string> boundary-value: "0.0";
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

        callback start(); callback pause(); callback resume(); callback stop();
        callback load-project(); callback save-project(); callback show-mesh(); callback show-field();
        callback animation-play-pause(); callback animation-next(); callback show-geometry-3d();
        callback rotate-geometry-3d(); callback apply-boundary(); callback select-step(int);
        callback select-case(); callback add-part(); callback remove-last-part();

        VerticalLayout {
            spacing: 0px;
            Rectangle {
                height: 60px;
                background: rgb(20, 37, 49);
                border-color: rgb(50, 79, 99);
                border-width: 1px;
                HorizontalLayout {
                    padding: 14px;
                    spacing: 11px;
                    Text { text: "FLURSYS"; color: rgb(233, 242, 248); font-size: 23px; font-weight: 800; }
                    Rectangle { width: 1px; background: rgb(65, 94, 112); }
                    Text { text: "CFD WORKBENCH"; color: rgb(136, 164, 182); font-size: 12px; }
                    Rectangle { horizontal-stretch: 1; }
                    Text { text: "PROJECT"; color: rgb(136, 164, 182); font-size: 10px; }
                    Text { text: root.project-name; color: rgb(215, 229, 238); font-size: 12px; }
                    Rectangle { width: 1px; background: rgb(65, 94, 112); }
                    Text { text: "SOLVER"; color: rgb(136, 164, 182); font-size: 10px; }
                    Text { text: root.status; color: rgb(240, 195, 109); font-size: 12px; font-weight: 700; }
                }
            }

            HorizontalLayout {
                spacing: 0px;
                Rectangle {
                    width: 248px;
                    background: rgb(14, 25, 34);
                    border-color: rgb(37, 58, 73);
                    border-width: 1px;
                    VerticalLayout {
                        padding: 12px;
                        spacing: 8px;
                        Text { text: "SIMULATION WORKFLOW"; color: rgb(132, 161, 180); font-size: 10px; font-weight: 700; }
                        WorkflowItem { number: "01"; title: "Geometry"; detail: "Choose domain and project"; active: root.current-step == 0; select => { root.select-step(0); } }
                        WorkflowItem { number: "02"; title: "Mesh"; detail: "Structured volume controls"; active: root.current-step == 1; select => { root.select-step(1); } }
                        WorkflowItem { number: "03"; title: "Setup"; detail: "Boundaries and physics"; active: root.current-step == 2; select => { root.select-step(2); } }
                        WorkflowItem { number: "04"; title: "Run"; detail: "Solver and convergence"; active: root.current-step == 3; select => { root.select-step(3); } }
                        WorkflowItem { number: "05"; title: "Results"; detail: "Fields and animation"; active: root.current-step == 4; select => { root.select-step(4); } }
                        Rectangle { vertical-stretch: 1; }
                        Rectangle { height: 1px; background: rgb(37, 58, 73); }
                        Text { text: "Active numerical core"; color: rgb(136, 164, 182); font-size: 10px; }
                        Text { text: "2D structured FVM\n3D workbench preview"; color: rgb(190, 209, 220); font-size: 11px; wrap: word-wrap; }
                    }
                }

                Rectangle {
                    horizontal-stretch: 1;
                    background: rgb(11, 19, 26);

                    if root.current-step == 0 : Rectangle {
                        width: parent.width; height: parent.height;
                        VerticalLayout {
                            padding: 26px; spacing: 16px;
                            SectionTitle { text: "01  Geometry"; }
                            SectionHint { text: "Start every simulation by choosing a supported geometry and saving a portable project definition."; }
                            HorizontalLayout {
                                spacing: 16px;
                                Card {
                                    width: 300px;
                                    VerticalLayout {
                                        padding: 18px; spacing: 10px;
                                        Text { text: "CASE LIBRARY"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                        Text { text: "Select the physical domain"; color: rgb(226, 237, 245); font-size: 16px; font-weight: 700; }
                                        ComboBox { model: ["Lid-driven cavity", "Cylinder flow Re=100", "Backward-facing step"]; current-index <=> root.case-index; }
                                        Button { text: "USE THIS GEOMETRY"; clicked => { root.select-case(); } }
                                        Rectangle { height: 1px; background: rgb(47, 74, 91); }
                                        Text { text: "Project name"; color: rgb(140, 167, 185); font-size: 11px; }
                                        LineEdit { text <=> root.project-name; }
                                        Text { text: "Project file"; color: rgb(140, 167, 185); font-size: 11px; }
                                        LineEdit { text <=> root.project-path; }
                                        HorizontalLayout { Button { text: "LOAD"; clicked => { root.load-project(); } } Button { text: "SAVE"; clicked => { root.save-project(); } } }
                                    }
                                }
                                Card {
                                    width: 355px;
                                    VerticalLayout {
                                        padding: 18px; spacing: 8px;
                                        Text { text: "3D PART DESIGNER"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                        Text { text: "Create parametric solids"; color: rgb(226, 237, 245); font-size: 16px; font-weight: 700; }
                                        ComboBox { model: ["Box", "Cylinder"]; current-index <=> root.part-kind-index; }
                                        Text { text: "Part name"; color: rgb(140, 167, 185); font-size: 11px; }
                                        LineEdit { text <=> root.part-name; }
                                        Text { text: "Dimensions  (X / Y / Z; cylinder uses radius / radius / height)"; color: rgb(140, 167, 185); font-size: 10px; }
                                        HorizontalLayout {
                                            LineEdit { text <=> root.part-size-x; }
                                            LineEdit { text <=> root.part-size-y; }
                                            LineEdit { text <=> root.part-size-z; }
                                        }
                                        Text { text: "Centre position  (X / Y / Z)"; color: rgb(140, 167, 185); font-size: 10px; }
                                        HorizontalLayout {
                                            LineEdit { text <=> root.part-pos-x; }
                                            LineEdit { text <=> root.part-pos-y; }
                                            LineEdit { text <=> root.part-pos-z; }
                                        }
                                        HorizontalLayout {
                                            Button { text: "ADD SOLID"; clicked => { root.add-part(); } }
                                            Button { text: "REMOVE LAST"; clicked => { root.remove-last-part(); } }
                                        }
                                        Text { text: root.geometry-parts-summary; color: rgb(126, 153, 170); font-size: 10px; wrap: word-wrap; vertical-stretch: 1; }
                                    }
                                }
                                Card {
                                    horizontal-stretch: 1;
                                    VerticalLayout {
                                        padding: 18px; spacing: 10px;
                                        Text { text: "GEOMETRY PREVIEW"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                        Text { text: root.case-name + " · " + root.geometry-parts-summary; color: rgb(229, 239, 246); font-size: 13px; font-weight: 700; wrap: word-wrap; }
                                        Rectangle { vertical-stretch: 1; background: rgb(9, 16, 22); border-color: rgb(40, 66, 82); border-width: 1px; Image { source: root.visualization-image; width: parent.width; height: parent.height; image-fit: contain; } }
                                        HorizontalLayout { Button { text: "3D PREVIEW"; clicked => { root.show-geometry-3d(); } } Button { text: "ROTATE"; clicked => { root.rotate-geometry-3d(); } } }
                                    }
                                }
                            }
                        }
                    }

                    if root.current-step == 1 : Rectangle {
                        width: parent.width; height: parent.height;
                        VerticalLayout {
                            padding: 26px; spacing: 16px;
                            SectionTitle { text: "02  Mesh"; }
                            SectionHint { text: "Define the active structured solver grid and retain extrusion layers for the 3D workbench model."; }
                            HorizontalLayout {
                                spacing: 16px;
                                Card { width: 400px; VerticalLayout { padding: 18px; spacing: 11px;
                                    Text { text: "STRUCTURED GRID"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Text { text: "Planar cells"; color: rgb(140, 167, 185); font-size: 11px; }
                                    HorizontalLayout { Text { text: "Nx"; color: rgb(205, 220, 229); vertical-alignment: center; } SpinBox { value <=> root.nx; minimum: 4; maximum: 1024; } Text { text: "Ny"; color: rgb(205, 220, 229); vertical-alignment: center; } SpinBox { value <=> root.ny; minimum: 4; maximum: 1024; } }
                                    Text { text: "Extrusion layers (Nz)"; color: rgb(140, 167, 185); font-size: 11px; }
                                    SpinBox { value <=> root.mesh-nz; minimum: 1; maximum: 1024; }
                                    Text { text: "Extrusion depth"; color: rgb(140, 167, 185); font-size: 11px; }
                                    LineEdit { text <=> root.extrusion-depth; }
                                    Text { text: "The active solver uses Nx × Ny. Nz is retained in the project and drives the 3D mesh preview."; color: rgb(126, 153, 170); font-size: 11px; wrap: word-wrap; }
                                    Button { text: "GENERATE MESH PREVIEW"; clicked => { root.show-mesh(); } }
                                } }
                                Card { horizontal-stretch: 1; VerticalLayout { padding: 18px; spacing: 10px;
                                    Text { text: "MESH INSPECTION"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Rectangle { vertical-stretch: 1; background: rgb(9, 16, 22); border-color: rgb(40, 66, 82); border-width: 1px; Image { source: root.visualization-image; width: parent.width; height: parent.height; image-fit: contain; } }
                                    Text { text: root.visualization-title + " · " + root.animation-status; color: rgb(163, 188, 203); font-size: 11px; }
                                    HorizontalLayout { Button { text: "2D GRID"; clicked => { root.show-mesh(); } } Button { text: "3D VOLUME"; clicked => { root.show-geometry-3d(); } } }
                                } }
                            }
                        }
                    }

                    if root.current-step == 2 : Rectangle {
                        width: parent.width; height: parent.height;
                        VerticalLayout {
                            padding: 26px; spacing: 16px;
                            SectionTitle { text: "03  Setup"; }
                            SectionHint { text: "Assign boundary conditions to named faces. Changes are saved in the project and supported planar conditions reach the solver."; }
                            HorizontalLayout {
                                spacing: 16px;
                                Card { width: 420px; VerticalLayout { padding: 18px; spacing: 10px;
                                    Text { text: "BOUNDARY CONDITIONS"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Text { text: "Boundary face"; color: rgb(140, 167, 185); font-size: 11px; }
                                    ComboBox { model: ["Left / inlet", "Right / outlet", "Bottom", "Top", "Front", "Back"]; current-index <=> root.boundary-face-index; }
                                    Text { text: "Condition"; color: rgb(140, 167, 185); font-size: 11px; }
                                    ComboBox { model: ["Case default", "Velocity", "Pressure outlet", "Wall", "Symmetry"]; current-index <=> root.boundary-kind-index; }
                                    Text { text: "Value  (u for velocity/wall; p for outlet)"; color: rgb(140, 167, 185); font-size: 11px; }
                                    LineEdit { text <=> root.boundary-value; }
                                    Button { text: "APPLY BOUNDARY CONDITION"; clicked => { root.apply-boundary(); } }
                                    Rectangle { height: 1px; background: rgb(47, 74, 91); }
                                    Text { text: "Front and back are retained for the future 3D solver. The active 2D solver accepts a pressure outlet on the right face."; color: rgb(126, 153, 170); font-size: 11px; wrap: word-wrap; }
                                } }
                                Card { horizontal-stretch: 1; VerticalLayout { padding: 18px; spacing: 10px;
                                    Text { text: "BOUNDARY & VOLUME VIEW"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Rectangle { vertical-stretch: 1; background: rgb(9, 16, 22); border-color: rgb(40, 66, 82); border-width: 1px; Image { source: root.visualization-image; width: parent.width; height: parent.height; image-fit: contain; } }
                                    Text { text: "Cyan: left · Gold: right · Green: top · Magenta: bottom"; color: rgb(163, 188, 203); font-size: 11px; }
                                    HorizontalLayout { Button { text: "SHOW 3D"; clicked => { root.show-geometry-3d(); } } Button { text: "ROTATE"; clicked => { root.rotate-geometry-3d(); } } }
                                } }
                            }
                        }
                    }

                    if root.current-step == 3 : Rectangle {
                        width: parent.width; height: parent.height;
                        VerticalLayout {
                            padding: 26px; spacing: 16px;
                            SectionTitle { text: "04  Run"; }
                            SectionHint { text: "Choose the numerical method, start the worker, and follow convergence without blocking the interface."; }
                            HorizontalLayout {
                                spacing: 16px;
                                Card { width: 400px; VerticalLayout { padding: 18px; spacing: 10px;
                                    Text { text: "SOLVER CONTROLS"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Text { text: "Coupling"; color: rgb(140, 167, 185); font-size: 11px; } ComboBox { model: ["SIMPLE-style steady", "Projection transient"]; current-index <=> root.coupling-index; }
                                    Text { text: "Pressure solver"; color: rgb(140, 167, 185); font-size: 11px; } ComboBox { model: ["PCG + Jacobi", "SOR"]; current-index <=> root.pressure-solver-index; }
                                    Text { text: "Pseudo-time step"; color: rgb(140, 167, 185); font-size: 11px; } LineEdit { text <=> root.dt-text; }
                                    Text { text: "Iterations"; color: rgb(140, 167, 185); font-size: 11px; } SpinBox { value <=> root.iterations; minimum: 1; maximum: 10000000; }
                                    Text { text: "CPU threads (0 = auto)"; color: rgb(140, 167, 185); font-size: 11px; } SpinBox { value <=> root.threads; minimum: 0; maximum: 256; }
                                    HorizontalLayout { Button { text: "START"; clicked => { root.start(); } } Button { text: "PAUSE"; clicked => { root.pause(); } } Button { text: "RESUME"; clicked => { root.resume(); } } Button { text: "STOP"; clicked => { root.stop(); } } }
                                } }
                                Card { horizontal-stretch: 1; VerticalLayout { padding: 18px; spacing: 10px;
                                    Text { text: "LIVE CONVERGENCE"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Rectangle { vertical-stretch: 1; background: rgb(9, 16, 22); border-color: rgb(40, 66, 82); border-width: 1px; Image { source: root.residual-image; width: parent.width; height: parent.height; image-fit: contain; } }
                                    HorizontalLayout { MetricCard { label: "STATUS"; value: root.status; } MetricCard { label: "ITERATIONS"; value: root.iterations; } MetricCard { label: "WORKERS"; value: root.threads == 0 ? "AUTO" : root.threads; } }
                                } }
                            }
                        }
                    }

                    if root.current-step == 4 : Rectangle {
                        width: parent.width; height: parent.height;
                        VerticalLayout {
                            padding: 26px; spacing: 16px;
                            SectionTitle { text: "05  Results"; }
                            SectionHint { text: "Inspect generated field snapshots, animate sampled results, and review the run log."; }
                            HorizontalLayout {
                                spacing: 16px;
                                Card { horizontal-stretch: 1; VerticalLayout { padding: 18px; spacing: 10px;
                                    Text { text: root.visualization-title; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Rectangle { vertical-stretch: 1; background: rgb(9, 16, 22); border-color: rgb(40, 66, 82); border-width: 1px; Image { source: root.visualization-image; width: parent.width; height: parent.height; image-fit: contain; } }
                                    Text { text: root.animation-status; color: rgb(163, 188, 203); font-size: 11px; }
                                    HorizontalLayout { Button { text: "FIELD"; clicked => { root.show-field(); } } Button { text: "MESH"; clicked => { root.show-mesh(); } } Button { text: "PLAY / PAUSE"; clicked => { root.animation-play-pause(); } } Button { text: "NEXT"; clicked => { root.animation-next(); } } }
                                } }
                                Card { width: 390px; VerticalLayout { padding: 18px; spacing: 10px;
                                    Text { text: "RUN SUMMARY"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    Text { text: root.residual-summary; color: rgb(213, 227, 236); font-family: "monospace"; font-size: 12px; }
                                    Text { text: root.force-summary; color: rgb(213, 227, 236); font-family: "monospace"; font-size: 12px; }
                                    Text { text: root.field-summary; color: rgb(140, 167, 185); font-size: 11px; wrap: word-wrap; }
                                    Text { text: "SOLVER LOG"; color: rgb(240, 195, 109); font-size: 11px; font-weight: 700; }
                                    TextEdit { text: root.log-text; read-only: true; vertical-stretch: 1; }
                                } }
                            }
                        }
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
    show_geometry_3d: bool,
    geometry_rotation: f32,
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
            geometry_rotation: 0.0,
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
        state.geometry_rotation = (state.geometry_rotation + 0.45) % std::f32::consts::TAU;
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
    project.preprocessing.geometry.extrusion_depth = parse_number(
        ui.get_extrusion_depth().as_str(),
        project.preprocessing.geometry.extrusion_depth,
    );
    project.preprocessing.mesh.cells_z = ui.get_mesh_nz().max(1) as usize;
}

fn write_project_to_ui(ui: &MainWindow, project: &Project) {
    ui.set_project_name(SharedString::from(project.name.as_str()));
    ui.set_case_name(SharedString::from(case_name(&project.case)));
    ui.set_case_index(project_case_index(&project.case));
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
    ui.set_extrusion_depth(SharedString::from(format!(
        "{:.3}",
        project.preprocessing.geometry.extrusion_depth
    )));
    ui.set_mesh_nz(project.preprocessing.mesh.cells_z as i32);
    ui.set_geometry_parts_summary(SharedString::from(geometry_parts_summary(project)));
    ui.set_part_name(SharedString::from(format!(
        "Part {}",
        project.preprocessing.geometry.parts.len() + 1
    )));
    write_boundary_to_ui(ui, project, BoundaryFace::Left);
}

fn geometry_part_from_ui(ui: &MainWindow) -> Result<GeometryPart, String> {
    let name = ui.get_part_name().trim().to_string();
    if name.is_empty() {
        return Err("3D part name cannot be empty".to_string());
    }
    let x_size = parse_positive(ui.get_part_size_x().as_str(), "X/radius")?;
    let y_size = parse_positive(ui.get_part_size_y().as_str(), "Y")?;
    let z_size = parse_positive(ui.get_part_size_z().as_str(), "Z/height")?;
    let x = parse_finite(ui.get_part_pos_x().as_str(), "position X")?;
    let y = parse_finite(ui.get_part_pos_y().as_str(), "position Y")?;
    let z = parse_finite(ui.get_part_pos_z().as_str(), "position Z")?;
    let kind = if ui.get_part_kind_index() == 1 {
        GeometryPartKind::Cylinder {
            radius: x_size,
            height: z_size,
            segments: 32,
        }
    } else {
        GeometryPartKind::Box {
            length: x_size,
            width: y_size,
            height: z_size,
        }
    };
    Ok(GeometryPart {
        name,
        kind,
        x,
        y,
        z,
    })
}

fn parse_positive(value: &str, label: &str) -> Result<f64, String> {
    let value = parse_finite(value, label)?;
    if value > 0.0 {
        Ok(value)
    } else {
        Err(format!("{label} must be positive"))
    }
}

fn parse_finite(value: &str, label: &str) -> Result<f64, String> {
    value
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| format!("{label} must be a finite number"))
}

fn geometry_parts_summary(project: &Project) -> String {
    let parts = &project.preprocessing.geometry.parts;
    match parts.len() {
        0 => "No custom solids in this project.".to_string(),
        1 => format!("1 solid: {}", parts[0].summary()),
        count => format!("{count} solids · latest: {}", parts[count - 1].summary()),
    }
}

fn project_case_from_index(index: i32) -> ProjectCase {
    match index {
        1 => ProjectCase::from(CylinderCase::default()),
        2 => ProjectCase::from(BackwardStepCase::default()),
        _ => ProjectCase::from(CavityCase::default()),
    }
}

fn project_case_index(case: &ProjectCase) -> i32 {
    match case {
        ProjectCase::LidDrivenCavity { .. } => 0,
        ProjectCase::Cylinder { .. } => 1,
        ProjectCase::BackwardFacingStep { .. } => 2,
    }
}

fn write_boundary_to_ui(ui: &MainWindow, project: &Project, face: BoundaryFace) {
    ui.set_boundary_face_index(boundary_face_index(face));
    let Some(boundary) = project.preprocessing.boundary(face) else {
        return;
    };
    let (kind_index, value) = match &boundary.kind {
        BoundaryConditionKind::CaseDefault => (0, 0.0),
        BoundaryConditionKind::Velocity { u, .. } => (1, *u),
        BoundaryConditionKind::PressureOutlet { pressure } => (2, *pressure),
        BoundaryConditionKind::Wall { u, .. } => (3, *u),
        BoundaryConditionKind::Symmetry => (4, 0.0),
    };
    ui.set_boundary_kind_index(kind_index);
    ui.set_boundary_value(SharedString::from(format!("{value:.6}")));
}

fn boundary_face_from_index(index: i32) -> BoundaryFace {
    match index {
        1 => BoundaryFace::Right,
        2 => BoundaryFace::Bottom,
        3 => BoundaryFace::Top,
        4 => BoundaryFace::Front,
        5 => BoundaryFace::Back,
        _ => BoundaryFace::Left,
    }
}

fn boundary_face_index(face: BoundaryFace) -> i32 {
    match face {
        BoundaryFace::Left => 0,
        BoundaryFace::Right => 1,
        BoundaryFace::Bottom => 2,
        BoundaryFace::Top => 3,
        BoundaryFace::Front => 4,
        BoundaryFace::Back => 5,
    }
}

fn refresh_ui(ui: &MainWindow, state: &AppState) {
    ui.set_geometry_parts_summary(SharedString::from(geometry_parts_summary(&state.project)));
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
    if state.show_geometry_3d {
        ui.set_visualization_title(SharedString::from("3D GEOMETRY & MESH"));
        ui.set_animation_status(SharedString::from(format!(
            "{} layers · boundary setup retained in project",
            state.project.preprocessing.mesh.cells_z
        )));
        ui.set_visualization_image(render_geometry_3d(&state.project, state.geometry_rotation));
    } else if state.show_mesh {
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

fn render_geometry_3d(project: &Project, rotation: f32) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [11, 18, 24, 255]);

    let front_left = 74_i32;
    let front_right = 360_i32;
    let front_top = 62_i32;
    let front_bottom = 264_i32;
    let depth = 62.0;
    let offset_x = (rotation.cos() * depth) as i32;
    let offset_y = (-42.0 - rotation.sin().abs() * 14.0) as i32;
    let front = [
        (front_left, front_bottom),
        (front_right, front_bottom),
        (front_right, front_top),
        (front_left, front_top),
    ];
    let back = front.map(|(x, y)| (x + offset_x, y + offset_y));
    let mesh = &project.preprocessing.mesh;
    let nx = project.solver.nx.clamp(4, 28) as i32;
    let ny = project.solver.ny.clamp(4, 24) as i32;
    let nz = mesh.cells_z.clamp(1, 16) as i32;

    for column in 0..=nx {
        let x = front_left + (front_right - front_left) * column / nx;
        draw_line(
            &mut pixels,
            width,
            height,
            (x, front_top),
            (x, front_bottom),
            [45, 82, 103, 255],
        );
    }
    for row in 0..=ny {
        let y = front_top + (front_bottom - front_top) * row / ny;
        draw_line(
            &mut pixels,
            width,
            height,
            (front_left, y),
            (front_right, y),
            [45, 82, 103, 255],
        );
    }
    for layer in 0..=nz {
        let ratio = layer as f32 / nz as f32;
        for &(from, to) in &[(front[0], back[0]), (front[1], back[1])] {
            let x = from.0 + ((to.0 - from.0) as f32 * ratio) as i32;
            let y = from.1 + ((to.1 - from.1) as f32 * ratio) as i32;
            draw_line(
                &mut pixels,
                width,
                height,
                (x, y),
                (x + front_right - front_left, y),
                [34, 62, 79, 255],
            );
        }
    }
    for index in 0..4 {
        draw_line(
            &mut pixels,
            width,
            height,
            front[index],
            front[(index + 1) % 4],
            [196, 215, 226, 255],
        );
        draw_line(
            &mut pixels,
            width,
            height,
            back[index],
            back[(index + 1) % 4],
            [94, 126, 143, 255],
        );
        draw_line(
            &mut pixels,
            width,
            height,
            front[index],
            back[index],
            [94, 126, 143, 255],
        );
    }

    draw_line(
        &mut pixels,
        width,
        height,
        front[0],
        front[3],
        [77, 168, 184, 255],
    );
    draw_line(
        &mut pixels,
        width,
        height,
        front[1],
        front[2],
        [240, 195, 109, 255],
    );
    draw_line(
        &mut pixels,
        width,
        height,
        front[2],
        front[3],
        [116, 189, 135, 255],
    );
    draw_line(
        &mut pixels,
        width,
        height,
        front[0],
        front[1],
        [205, 115, 175, 255],
    );

    if project.preprocessing.geometry.parts.is_empty() {
        match &project.case {
            ProjectCase::Cylinder { .. } => draw_ellipse(
                &mut pixels,
                width,
                height,
                (front_left + 105, (front_top + front_bottom) / 2),
                24,
                24,
                [240, 195, 109, 255],
            ),
            ProjectCase::BackwardFacingStep { .. } => {
                let x = front_left + 90;
                let y = front_bottom - 55;
                draw_line(
                    &mut pixels,
                    width,
                    height,
                    (front_left, y),
                    (x, y),
                    [240, 195, 109, 255],
                );
                draw_line(
                    &mut pixels,
                    width,
                    height,
                    (x, y),
                    (x, front_bottom),
                    [240, 195, 109, 255],
                );
            }
            ProjectCase::LidDrivenCavity { .. } => {}
        }
    } else {
        let scale = geometry_scene_scale(&project.preprocessing.geometry.parts);
        for (index, part) in project.preprocessing.geometry.parts.iter().enumerate() {
            let color = part_color(index);
            match &part.kind {
                GeometryPartKind::Box {
                    length,
                    width,
                    height,
                } => draw_part_box(
                    &mut pixels,
                    PREVIEW_WIDTH,
                    PREVIEW_HEIGHT,
                    part,
                    *length,
                    *width,
                    *height,
                    rotation,
                    scale,
                    color,
                ),
                GeometryPartKind::Cylinder { radius, height, .. } => draw_part_cylinder(
                    &mut pixels,
                    PREVIEW_WIDTH,
                    PREVIEW_HEIGHT,
                    part,
                    *radius,
                    *height,
                    rotation,
                    scale,
                    color,
                ),
            }
        }
    }

    image_from_rgba(width, height, pixels)
}

fn geometry_scene_scale(parts: &[GeometryPart]) -> f64 {
    let extent = parts.iter().fold(1.0_f64, |extent, part| {
        let size = match &part.kind {
            GeometryPartKind::Box {
                length,
                width,
                height,
            } => 0.5 * length.max(*width).max(*height),
            GeometryPartKind::Cylinder { radius, height, .. } => radius.max(0.5 * height),
        };
        extent
            .max(part.x.abs() + size)
            .max(part.y.abs() + size)
            .max(part.z.abs() + size)
    });
    (110.0 / extent).clamp(16.0, 72.0)
}

fn part_color(index: usize) -> [u8; 4] {
    const COLORS: [[u8; 4]; 5] = [
        [77, 168, 184, 255],
        [240, 195, 109, 255],
        [116, 189, 135, 255],
        [205, 115, 175, 255],
        [134, 147, 241, 255],
    ];
    COLORS[index % COLORS.len()]
}

fn scene_point(x: f64, y: f64, z: f64, rotation: f32, scale: f64) -> (i32, i32) {
    let angle = f64::from(rotation) + 0.75;
    let horizontal = x * angle.cos() - y * angle.sin();
    let depth = x * angle.sin() + y * angle.cos();
    (
        (260.0 + horizontal * scale) as i32,
        (218.0 - z * scale - depth * scale * 0.36) as i32,
    )
}

#[allow(clippy::too_many_arguments)]
fn draw_part_box(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    length: f64,
    width: f64,
    height: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    let hx = 0.5 * length;
    let hy = 0.5 * width;
    let hz = 0.5 * height;
    let corners = [
        (-hx, -hy, -hz),
        (hx, -hy, -hz),
        (hx, hy, -hz),
        (-hx, hy, -hz),
        (-hx, -hy, hz),
        (hx, -hy, hz),
        (hx, hy, hz),
        (-hx, hy, hz),
    ]
    .map(|(x, y, z)| scene_point(part.x + x, part.y + y, part.z + z, rotation, scale));
    for (from, to) in [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ] {
        draw_line(
            pixels,
            image_width,
            image_height,
            corners[from],
            corners[to],
            color,
        );
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_part_cylinder(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    radius: f64,
    height: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    let mut previous_bottom = None;
    let mut previous_top = None;
    for step in 0..=32 {
        let angle = step as f64 * std::f64::consts::TAU / 32.0;
        let x = part.x + radius * angle.cos();
        let y = part.y + radius * angle.sin();
        let bottom = scene_point(x, y, part.z - 0.5 * height, rotation, scale);
        let top = scene_point(x, y, part.z + 0.5 * height, rotation, scale);
        if let Some(previous) = previous_bottom {
            draw_line(pixels, image_width, image_height, previous, bottom, color);
        }
        if let Some(previous) = previous_top {
            draw_line(pixels, image_width, image_height, previous, top, color);
        }
        if step % 8 == 0 {
            draw_line(pixels, image_width, image_height, bottom, top, color);
        }
        previous_bottom = Some(bottom);
        previous_top = Some(top);
    }
}

fn draw_ellipse(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    center: (i32, i32),
    radius_x: i32,
    radius_y: i32,
    color: [u8; 4],
) {
    let mut previous = None;
    for step in 0..=48 {
        let angle = step as f32 * std::f32::consts::TAU / 48.0;
        let point = (
            center.0 + (radius_x as f32 * angle.cos()) as i32,
            center.1 + (radius_y as f32 * angle.sin()) as i32,
        );
        if let Some(last) = previous {
            draw_line(pixels, width, height, last, point, color);
        }
        previous = Some(point);
    }
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

    #[test]
    fn geometry_preview_accepts_a_project_mesh() {
        let _image = render_geometry_3d(&Project::default(), 0.0);
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
        let _image = render_geometry_3d(&project, 0.35);
    }
}
