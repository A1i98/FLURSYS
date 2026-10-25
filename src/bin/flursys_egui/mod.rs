//! Desktop CFD workbench.
mod cad;
mod case_library;
mod compressible_flow;
mod continuation;
mod export;
mod initialization;
mod migration;
mod monitoring;
mod panels;
mod performance;
mod post;
mod property_editors;
mod transport;
mod viewport;
mod widgets;
use egui::{Color32, RichText};
use flursys::app::{JobProgress, WorkbenchJobEvent, WorkbenchJobExecutor};
use flursys::workbench::{self, force_analysis, mesh_storage, plane_monitor};
use flursys::*;
use serde::{Deserialize, Serialize};
use std::{
    path::{Path, PathBuf},
    sync::mpsc::{self, Receiver},
    time::{Duration, Instant},
};
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
enum Stage {
    Geometry,
    Mesh,
    Setup,
    Solution,
    Results,
}
impl Stage {
    const ALL: [Self; 5] = [
        Self::Geometry,
        Self::Mesh,
        Self::Setup,
        Self::Solution,
        Self::Results,
    ];
    fn label(self) -> &'static str {
        match self {
            Self::Geometry => "Geometry",
            Self::Mesh => "Mesh",
            Self::Setup => "Setup",
            Self::Solution => "Solution",
            Self::Results => "Results",
        }
    }
}
#[derive(Serialize, Deserialize)]
#[serde(default)]
struct Preferences {
    gmsh_path: String,
    light: bool,
    scale: f32,
    geometry_edges: bool,
    geometry_shading: bool,
    ambient_occlusion: bool,
    edge_width: f32,
    navigation_speed: f32,
    browser: bool,
    properties: bool,
    messages: bool,
    live_refresh_seconds: f64,
}
impl Default for Preferences {
    fn default() -> Self {
        Self {
            gmsh_path: String::new(),
            light: false,
            scale: 1.0,
            geometry_edges: true,
            geometry_shading: true,
            ambient_occlusion: true,
            edge_width: 1.2,
            navigation_speed: 1.0,
            browser: true,
            properties: true,
            messages: false,
            live_refresh_seconds: 2.0,
        }
    }
}
struct App {
    migration: migration::State,
    monitoring: monitoring::State,
    gas_flow: compressible_flow::State,
    field_loader: performance::FieldLoader,
    project: WorkbenchProject,
    session: WorkbenchSession,
    workspace: Option<PathBuf>,
    workspace_dialog: Option<(WorkspaceDialogAction, Receiver<Option<PathBuf>>)>,
    dirty: bool,
    stage: Stage,
    page: usize,
    jobs: WorkbenchJobExecutor,
    cad: cad_workbench::SolveApp,
    render: Option<egui_wgpu::RenderState>,
    publishing: Option<(Stage, Receiver<Result<cad::Published, String>>)>,
    loading: Option<
        Receiver<Result<(PathBuf, WorkbenchProject, WorkbenchSession, Option<String>), String>>,
    >,
    log: Vec<String>,
    feedback: String,
    show_log: bool,
    show_preferences: bool,
    preferences_page: usize,
    show_examples: bool,
    show_advanced: bool,
    pending_replace: Option<Pending>,
    preferences: Preferences,
    editor: WorkbenchProject,
    json_editor: String,
    selected: Vec<GeometrySelectionTarget>,
    pick: usize,
    boundary_name: String,
    boundary_patch: String,
    boundary_kind: usize,
    boundary_velocity: Vec3,
    boundary_pressure: f64,
    parametric_angle: f64,
    parametric_start_angle: f64,
    parametric_end_angle: f64,
    parametric_angle_step: f64,
    parametric_iterations: usize,
    parametric_job: Option<Receiver<(usize, Result<String, String>)>>,
    parametric_total: usize,
    parametric_completed: usize,
    parametric_failures: usize,
    inlet_axis: usize,
    inlet_center: f64,
    inlet_half_width: f64,
    camera: viewport::View,
    mesh_cache: Option<MeshRenderCache>,
    geometry_faces: Vec<flursys::RenderableFace>,
    geometry_edges: Vec<(EdgeId, Vec<Vec3>)>,
    result: Option<ResultDataset>,
    result_cache: Option<ResultRenderCache>,
    result_field: ResultFieldKind,
    result_edges: bool,
    result_plot: usize,
    result_levels: usize,
    result_palette: usize,
    vector_scale: f64,
    vector_count: usize,
    vector_normalized: bool,
    result_range: Option<[f64; 2]>,
    streamlines: Vec<StreamlinePath>,
    seed_streamlines: bool,
    result_vectors: bool,
    result_run: String,
    histories: [Vec<(f64, Option<f64>)>; 4],
    solve_histories: [Vec<(f64, Option<f64>)>; 4],
    continuation_run: Option<String>,
    result_iteration: usize,
    segment_start: usize,
    checkpoint_every: usize,
    iteration: usize,
    limit: usize,
    wall_reports: Vec<workbench::wall_reports::WallPatchReport>,
    force_history: Vec<force_analysis::Sample>,
    force_series: Vec<workbench::series_plot::Series>,
    recording: Option<PathBuf>,
    solve_recording: Option<PathBuf>,
    plane_index: usize,
    plane_snapshot: Option<plane_monitor::PlaneSnapshot>,
    plane_frames: Vec<PathBuf>,
    plane_frame: usize,
    plane_playing: bool,
    field_frames: Vec<PathBuf>,
    field_frame: usize,
    field_playing: bool,
    plane_tick: Instant,
    selected_probe: Option<ResultProbe>,
    probes: Vec<ResultProbe>,
    probe_enabled: bool,
    probe_position: Vec3,
    probe_quantity: usize,
    probe_series: Vec<workbench::series_plot::Series>,
    probe_history_job: Option<(
        String,
        Receiver<Result<Vec<workbench::series_plot::Series>, String>>,
    )>,
    show_probe_plot: bool,
    curve_visible: [bool; 4],
    curve_log: bool,
    curve_normalized: bool,
    flow_playing: bool,
    flow_phase: f64,
    post_tick: Instant,
    live_refresh: Instant,
    live_fields: bool,
    live_iteration: usize,
    playback_fps: f64,
    viewport_rect: Option<egui::Rect>,
    export: Option<export::Job>,
    gif_options: export::Options,
    show_gif_options: bool,
    gpu_devices: Vec<flursys::compute::DeviceInfo>,
    drag_select: Option<egui::Pos2>,
    mesh_quality: usize,
    quality_range: Option<[f64; 2]>,
    quality_edges: bool,
    quality_above: Option<f64>,
    mesh_cut: bool,
    mesh_cut_axis: usize,
    mesh_cut_fraction: f64,
    examples: Vec<workbench::CaseTemplateManifest>,
    example_search: String,
    example_category: String,
    example_selected: Option<usize>,
    example_preset: String,
    case_probes_seeded: bool,
    case_comparison_time: f64,
    case_comparison_report: String,
    save_error: Option<String>,
    initialized: bool,
    show_initialization: bool,
    initial_velocity: Vec3,
    initial_pressure: f64,
    browser_open: bool,
    browser_requested: bool,
    properties_open: bool,
    first_layout: bool,
    archived_report: Option<serde_json::Value>,
    materials: Vec<flursys::database::Entry>,
    material_id: String,
    material_temperature: f64,
    material_pressure: f64,
    capture: Option<PathBuf>,
    capture_requested: bool,
    capture_prepared: Option<Instant>,
    created: Instant,
    published_document: Option<String>,
    saving_after_publish: bool,
    frame_scan: Option<(String, Receiver<Vec<PathBuf>>)>,
    mesh_wireframe: bool,
    show_geometry: bool,
    geometry_gpu: viewport::GeometryGpu,
    transport: transport::Transport,
}
enum Pending {
    New,
    Open(PathBuf),
    Example(usize),
    Quit,
}
#[derive(Clone, Copy)]
enum WorkspaceDialogAction {
    Open,
    Save,
}
pub fn run() -> eframe::Result {
    eframe::run_native(
        "FLURSYS",
        eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default()
                .with_inner_size([1440., 940.])
                .with_min_inner_size([900., 620.]),
            renderer: eframe::Renderer::Wgpu,
            depth_buffer: 24,
            stencil_buffer: 8,
            ..Default::default()
        },
        Box::new(|cc| Ok(Box::new(App::new(cc)))),
    )
}
impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        cad_workbench::fonts::setup_bundled_fonts(&cc.egui_ctx);
        cad_workbench::fonts::load_system_fonts_in_background(&cc.egui_ctx);
        egui_extras::install_image_loaders(&cc.egui_ctx);
        cad_workbench::frame::set_left_docking(&cc.egui_ctx, true);
        let preferences = Self::preferences_path()
            .and_then(|p| std::fs::read(p).ok())
            .and_then(|b| serde_json::from_slice(&b).ok())
            .unwrap_or_default();
        let project = WorkbenchProject::default();
        let session = WorkbenchSession::from_project(&project).expect("valid blank project");
        let render = cc.wgpu_render_state.clone();
        let cad = cad::app(None, render.as_ref()).expect("empty CAD document");
        if let Some(rs) = &render {
            viewport::install_geometry_renderer(rs);
        }
        let mut app = Self {
            migration: migration::State::new(),
            monitoring: monitoring::State::default(),
            gas_flow: compressible_flow::State::default(),
            field_loader: performance::FieldLoader::default(),
            editor: project.clone(),
            project,
            session,
            workspace: None,
            workspace_dialog: None,
            dirty: false,
            stage: Stage::Geometry,
            page: 0,
            jobs: WorkbenchJobExecutor::new(),
            cad,
            render,
            publishing: None,
            loading: None,
            log: vec![],
            feedback: "Create a design or open an existing workspace.".into(),
            show_log: false,
            show_preferences: false,
            preferences_page: 0,
            show_examples: false,
            show_advanced: false,
            pending_replace: None,
            preferences,
            json_editor: String::new(),
            selected: vec![],
            pick: 0,
            boundary_name: String::new(),
            boundary_patch: String::new(),
            boundary_kind: 0,
            boundary_velocity: Vec3::ZERO,
            boundary_pressure: 0.,
            parametric_angle: 4.,
            parametric_start_angle: -2.,
            parametric_end_angle: 12.,
            parametric_angle_step: 2.,
            parametric_iterations: 100,
            parametric_job: None,
            parametric_total: 0,
            parametric_completed: 0,
            parametric_failures: 0,
            inlet_axis: 1,
            inlet_center: 0.5,
            inlet_half_width: 0.5,
            camera: Default::default(),
            mesh_cache: None,
            geometry_faces: vec![],
            geometry_edges: vec![],
            result: None,
            result_cache: None,
            result_field: ResultFieldKind::VelocityMagnitude,
            result_edges: false,
            result_plot: 0,
            result_levels: 12,
            result_palette: 1,
            vector_scale: 1.,
            vector_count: 600,
            vector_normalized: false,
            result_range: None,
            streamlines: vec![],
            seed_streamlines: false,
            result_vectors: true,
            result_run: String::new(),
            histories: Default::default(),
            solve_histories: Default::default(),
            continuation_run: None,
            result_iteration: 0,
            segment_start: 0,
            checkpoint_every: 100,
            iteration: 0,
            limit: 0,
            wall_reports: vec![],
            force_history: vec![],
            force_series: Vec::new(),
            recording: None,
            solve_recording: None,
            plane_index: 0,
            plane_snapshot: None,
            plane_frames: vec![],
            plane_frame: 0,
            plane_playing: false,
            field_frames: Vec::new(),
            field_frame: 0,
            field_playing: false,
            plane_tick: Instant::now(),
            selected_probe: None,
            probes: Vec::new(),
            probe_enabled: true,
            probe_position: Vec3::ZERO,
            probe_quantity: 0,
            probe_series: Vec::new(),
            probe_history_job: None,
            show_probe_plot: false,
            curve_visible: [true; 4],
            curve_log: true,
            curve_normalized: false,
            flow_playing: false,
            flow_phase: 0.,
            post_tick: Instant::now(),
            live_refresh: Instant::now(),
            live_fields: true,
            live_iteration: usize::MAX,
            playback_fps: 10.,
            viewport_rect: None,
            export: None,
            gif_options: export::Options::default(),
            show_gif_options: false,
            gpu_devices: Vec::new(),
            drag_select: None,
            mesh_quality: 0,
            quality_range: None,
            quality_edges: false,
            quality_above: None,
            mesh_cut: false,
            mesh_cut_axis: 0,
            mesh_cut_fraction: 0.5,
            examples: vec![],
            example_search: String::new(),
            example_category: String::new(),
            example_selected: None,
            example_preset: String::new(),
            case_probes_seeded: false,
            case_comparison_time: 0.,
            case_comparison_report: String::new(),
            save_error: None,
            initialized: false,
            show_initialization: false,
            initial_velocity: Vec3::ZERO,
            initial_pressure: 0.0,
            browser_open: true,
            browser_requested: false,
            properties_open: true,
            first_layout: true,
            archived_report: None,
            materials: vec![],
            material_id: String::new(),
            material_temperature: 293.15,
            material_pressure: 101325.,
            capture: if cfg!(debug_assertions) {
                std::env::var_os("FLURSYS_EGUI_CAPTURE").map(PathBuf::from)
            } else {
                None
            },
            capture_requested: false,
            capture_prepared: None,
            created: Instant::now(),
            published_document: None,
            saving_after_publish: false,
            frame_scan: None,
            mesh_wireframe: true,
            show_geometry: true,
            geometry_gpu: Default::default(),
            transport: Default::default(),
        };
        app.apply_preferences(&cc.egui_ctx);
        if let Some(path) = std::env::args_os().nth(1) {
            app.load(PathBuf::from(path));
        }
        app
    }
    fn preferences_path() -> Option<PathBuf> {
        std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|p| PathBuf::from(p).join(".config")))
            .map(|p| p.join("flursys/interface-egui.json"))
    }
    fn apply_preferences(&mut self, ctx: &egui::Context) {
        self.preferences.live_refresh_seconds = if self.preferences.live_refresh_seconds.is_finite()
        {
            self.preferences.live_refresh_seconds.clamp(0.1, 60.0)
        } else {
            2.0
        };

        self.cad.ui.dark = !self.preferences.light;
        cad_workbench::theme::set_dark(!self.preferences.light);
        cad_workbench::theme::apply(ctx);
        ctx.global_style_mut(|style| {
            style.spacing.item_spacing = egui::vec2(8., 8.);
            style.spacing.button_padding = egui::vec2(10., 6.);
            style.spacing.interact_size.y = 28.;
            style.visuals.window_corner_radius = egui::CornerRadius::same(10);
        });
        self.cad.preferences.ambient_occlusion = self.preferences.ambient_occlusion;
        if self.preferences.scale >= 0.7 && self.preferences.scale <= 2.0 {
            ctx.set_zoom_factor(self.preferences.scale);
        }
        GmshMesher::set_executable_override(
            (!self.preferences.gmsh_path.is_empty())
                .then(|| PathBuf::from(&self.preferences.gmsh_path)),
        );
    }
    fn message(&mut self, text: impl Into<String>) {
        let text = text.into();
        if self.capture.is_some() {
            eprintln!("Preview: {text}");
        }
        self.feedback = text.clone();
        self.append_log(text);
    }
    fn append_log(&mut self, text: impl AsRef<str>) {
        self.log.extend(text.as_ref().lines().map(str::to_owned));
        if self.log.len() > 10000 {
            self.log.drain(..self.log.len() - 10000);
        }
    }
    fn busy(&self) -> bool {
        self.jobs.is_running()
            || self.gas_flow.busy()
            || self.migration.mapping_active()
            || self.publishing.is_some()
            || self.loading.is_some()
            || self.workspace_dialog.is_some()
            || self.transport.receiver.is_some()
    }
    fn sync_project(&mut self) {
        let mut p = self.session.to_project(self.project.name.clone());
        p.project_id = self.project.project_id.clone();
        p.scalar_transport = self.project.scalar_transport.clone();
        p.runs = self.project.runs.clone();
        p.mesh_asset = self.project.mesh_asset.clone();
        p.cad_document = self.project.cad_document.clone();
        p.cad_geometry_hash = self.project.cad_geometry_hash.clone();
        p.plane_monitors = self.project.plane_monitors.clone();
        self.project = p;
    }
    fn set_project(
        &mut self,
        mut p: WorkbenchProject,
        s: WorkbenchSession,
        path: Option<PathBuf>,
    ) -> Result<(), String> {
        if p.cad_document.is_none() {
            let mut recipe = p.clone();
            recipe.fluid_face = s.fluid_face();
            p.cad_document = cad::restore_document(&recipe)?;
        }
        self.cad = cad::app(p.cad_document.as_deref(), self.render.as_ref())?;
        self.transport = Default::default();
        self.transport.options = p.scalar_transport.clone().unwrap_or_default();
        self.published_document = Some(self.cad.session.doc.to_json());
        self.project = p;
        (self.initial_velocity, self.initial_pressure) = self
            .project
            .boundaries
            .iter()
            .find_map(|b| match b.condition {
                workbench::PhysicalBoundaryCondition::FarField { velocity, pressure } => {
                    Some((velocity, pressure))
                }
                _ => None,
            })
            .unwrap_or((Vec3::ZERO, 0.0));
        self.session = s;
        self.workspace = path;
        self.dirty = false;
        self.editor = self.project.clone();
        self.jobs = WorkbenchJobExecutor::new();
        self.continuation_run = None;
        self.recording = None;
        self.solve_recording = None;
        self.solve_histories = Default::default();
        self.iteration = 0;
        self.limit = 0;
        self.result = None;
        self.result_cache = None;
        self.result_run.clear();
        self.result_iteration = 0;
        self.probes.clear();
        self.case_probes_seeded = false;
        self.case_comparison_report.clear();
        self.probe_series.clear();
        self.probe_history_job = None;
        self.show_probe_plot = false;
        self.frame_scan = None;
        self.live_fields = true;
        self.wall_reports.clear();
        self.force_history.clear();
        self.histories = Default::default();
        self.selected.clear();
        self.plane_snapshot = None;
        self.plane_frames.clear();
        self.plane_playing = false;
        self.field_frames.clear();
        self.field_playing = false;
        self.field_loader = performance::FieldLoader::default();
        self.force_series.clear();
        self.migration = migration::State::new();
        self.monitoring = monitoring::State::default();
        self.gas_flow = compressible_flow::State::default();
        self.apply_case_display();
        self.initialized = false;
        self.refresh_geometry();
        self.refresh_mesh();
        self.restore_continuation();
        self.stage = if self.session.mesh().is_some() {
            Stage::Mesh
        } else {
            Stage::Geometry
        };
        self.page = 0;
        Ok(())
    }
    fn refresh_geometry(&mut self) {
        self.geometry_faces = self.session.geometry().renderable_faces();
        self.geometry_edges = self
            .session
            .geometry()
            .edges()
            .filter_map(|e| {
                viewport::edge_path(self.session.geometry(), &e.geometry).map(|p| (e.id, p))
            })
            .collect();
        self.camera.fit_geometry(self.session.geometry());
    }
    fn refresh_mesh(&mut self) {
        self.mesh_cache = self
            .session
            .mesh()
            .and_then(|m| MeshRenderCache::build(&m.mesh).ok());
        if let Some(m) = self.session.mesh() {
            self.camera.fit_mesh(&m.mesh);
        }
    }
    fn request(&mut self, p: Pending) {
        if self.busy() {
            self.message("Finish or stop the active job before replacing the project.");
            return;
        }
        if self.dirty || self.cad.session.is_dirty() {
            self.pending_replace = Some(p);
        } else {
            self.replace(p);
        }
    }
    fn choose_workspace_folder(&mut self, action: WorkspaceDialogAction) {
        if self.workspace_dialog.is_some() {
            return;
        }
        let title = match action {
            WorkspaceDialogAction::Open => "Open FLURSYS workspace",
            WorkspaceDialogAction::Save => "Save FLURSYS workspace",
        };
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let folder =
                pollster::block_on(rfd::AsyncFileDialog::new().set_title(title).pick_folder())
                    .map(|handle| handle.path().to_path_buf());
            let _ = tx.send(folder);
        });
        self.workspace_dialog = Some((action, rx));
        self.message(match action {
            WorkspaceDialogAction::Open => "Choose a workspace folder…",
            WorkspaceDialogAction::Save => "Choose where to save the workspace…",
        });
    }
    fn poll_workspace_dialog(&mut self) {
        let Some((action, receiver)) = &self.workspace_dialog else {
            return;
        };
        let action = *action;
        let received = match receiver.try_recv() {
            Ok(path) => Some(Ok(path)),
            Err(mpsc::TryRecvError::Disconnected) => {
                Some(Err("The workspace picker stopped unexpectedly.".to_string()))
            }
            Err(mpsc::TryRecvError::Empty) => None,
        };
        if let Some(result) = received {
            self.workspace_dialog = None;
            match result {
                Ok(Some(path)) => match action {
                    WorkspaceDialogAction::Open => self.request(Pending::Open(path)),
                    WorkspaceDialogAction::Save => match self.save_to(path) {
                        Ok(true) => {
                            if let Some(pending) = self.pending_replace.take() {
                                self.replace(pending);
                            }
                        }
                        Ok(false) => {}
                        Err(error) => self.message(error),
                    },
                },
                Ok(None) => self.message("Workspace selection cancelled."),
                Err(error) => self.message(error),
            }
        }
    }
    fn replace(&mut self, p: Pending) {
        match p {
            Pending::Open(path) => self.load(path),
            Pending::New => {
                let p = WorkbenchProject::default();
                match WorkbenchSession::from_project(&p)
                    .map_err(|e| e.to_string())
                    .and_then(|s| self.set_project(p, s, None))
                {
                    Ok(()) => {}
                    Err(e) => self.message(e),
                }
            }
            Pending::Example(i) => {
                if let Some(template) = self.examples.get(i) {
                    let p = match workbench::case_format::instantiate(
                        template,
                        (!self.example_preset.is_empty()).then_some(self.example_preset.as_str()),
                    ) {
                        Ok(p) => p,
                        Err(e) => {
                            self.message(e);
                            return;
                        }
                    };
                    match WorkbenchSession::from_project(&p)
                        .map_err(|e| e.to_string())
                        .and_then(|s| self.set_project(p, s, None))
                    {
                        Ok(()) => {
                            self.stage = Stage::Mesh;
                            self.dirty = true;
                        }
                        Err(e) => self.message(e),
                    }
                }
            }
            Pending::Quit => self.save_error = Some("quit-confirmed".into()),
        }
    }
    fn load(&mut self, path: PathBuf) {
        let path = if path.is_file() {
            path.parent().unwrap_or(Path::new(".")).to_path_buf()
        } else {
            path
        };
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            let result = (|| -> Result<_, String> {
                let project = flursys::load_workspace(&path).map_err(|e| e.to_string())?;
                let mut session =
                    WorkbenchSession::from_project(&project).map_err(|e| e.to_string())?;
                let warning = mesh_storage::restore(&path, &project, &mut session).err();
                Ok((path, project, session, warning))
            })();
            let _ = tx.send(result);
        });
        self.loading = Some(rx);
        self.message("Loading workspace and saved mesh…");
    }
    fn save(&mut self) -> Result<bool, String> {
        if self.cad.session.active_sketch.is_some() || self.cad.applying.is_some() {
            return Err("Finish the CAD operation before saving the workspace.".into());
        }
        let document = self.cad.session.doc.to_json();
        if self.published_document.as_ref() != Some(&document) && self.publishing.is_none() {
            let (tx, rx) = mpsc::channel();
            std::thread::spawn(move || {
                let _ = tx.send(cad::publish(document));
            });
            self.publishing = Some((self.stage, rx));
            self.saving_after_publish = true;
            self.message("Preparing CAD geometry before saving…");
            return Ok(false);
        }
        if self.project.cad_document.is_some() || self.cad.session.is_dirty() {
            self.project.cad_document = Some(document);
        }
        self.sync_project();
        let Some(path) = self.workspace.clone() else {
            self.choose_workspace_folder(WorkspaceDialogAction::Save);
            return Ok(false);
        };
        self.save_to(path)
    }
    fn save_to(&mut self, path: PathBuf) -> Result<bool, String> {
        if let Some(mesh) = self.session.mesh() {
            self.project.mesh_asset = Some(mesh_storage::save(&path, &self.project, mesh)?);
        }
        workbench::save_workspace(&path, &self.project).map_err(|e| e.to_string())?;
        self.workspace = Some(path);
        self.dirty = false;
        self.cad.session.mark_saved();
        self.message("Workspace saved, including mesh and CAD document.");
        Ok(true)
    }
    fn stage(&mut self, next: Stage) {
        if next == Stage::Geometry
            && (self.jobs.is_running() || self.transport.receiver.is_some() || self.gas_flow.busy())
        {
            self.message("Stop the active job before editing CAD.");
            return;
        }
        if self.busy() && self.publishing.is_some() {
            return;
        }
        if self.stage == Stage::Geometry && next != Stage::Geometry {
            if self.cad.session.active_sketch.is_some()
                || self.cad.dialog.is_some()
                || self.cad.applying.is_some()
            {
                self.message("Finish or cancel the active CAD operation first.");
                return;
            }
            let document = self.cad.session.doc.to_json();
            if self.published_document.as_ref() != Some(&document)
                || self.session.geometry().vertices().next().is_none()
            {
                let (tx, rx) = mpsc::channel();
                std::thread::spawn(move || {
                    let _ = tx.send(cad::publish(document));
                });
                self.publishing = Some((next, rx));
                self.message("Preparing canonical CFD geometry…");
                return;
            }
        }
        self.stage = next;
        self.page = 0;
        self.editor = self.project.clone();
        if next == Stage::Results
            && self.result.is_none()
            && self.project.solver.compressible.is_none()
        {
            self.ensure_result();
        }
    }
    fn apply_editor(&mut self) -> Result<(), String> {
        if self.busy() {
            return Err("Wait for the active operation before applying settings.".into());
        }
        let mesh = self.session.mesh().cloned();
        let mut p = self.editor.clone();
        p.geometry = self.project.geometry.clone();
        p.runs = self.project.runs.clone();
        p.cad_document = self.project.cad_document.clone();
        p.plane_monitors = self.project.plane_monitors.clone();
        let mut s = WorkbenchSession::from_project(&p).map_err(|e| e.to_string())?;
        let mut old_mesh_settings = self.project.mesh.clone();
        old_mesh_settings.threads = p.mesh.threads;
        let mesh_changed = p.mesh != old_mesh_settings || p.mesh_recipe != self.project.mesh_recipe;
        let equations_changed = p.solver.time.is_transient()
            != self.project.solver.time.is_transient()
            || p.solver.turbulence.model != self.project.solver.turbulence.model
            || p.solver.thermal.enabled != self.project.solver.thermal.enabled
            || p.solver.thermal.reduced_pressure != self.project.solver.thermal.reduced_pressure
            || p.solver.initial_profile != self.project.solver.initial_profile
            || p.solver.compressible.is_some() != self.project.solver.compressible.is_some();
        if !mesh_changed {
            if let Some(m) = mesh {
                s.install_mesh(m);
            }
        }
        self.project = p;
        self.session = s;
        self.dirty = true;
        self.refresh_mesh();
        if mesh_changed || equations_changed {
            self.reset_solve_session();
            self.persist_continuation()?;
        }
        self.message("Settings applied. Existing runs are retained.");
        Ok(())
    }
    fn generate_mesh(&mut self) {
        if self.busy() {
            return;
        }
        let result = self
            .apply_editor()
            .and_then(|_| {
                self.session
                    .mesh_generation_inputs()
                    .map_err(|e| e.to_string())
            })
            .and_then(|(e, o)| self.jobs.start_mesh(e, o).map_err(str::to_string));
        if let Err(e) = result {
            self.message(e);
        } else {
            self.message("Mesh generation started.");
        }
    }
    fn start_solve(&mut self) {
        if self.editor.solver.compressible.is_some() {
            self.start_compressible();
            return;
        }
        if self.jobs.is_running() {
            if self.jobs.is_paused() {
                self.jobs.set_paused(false);
                self.message("Solve resumed.");
            }
            return;
        }
        let result = (|| -> Result<(), String> {
            self.apply_editor()?;
            let path = self.workspace.clone().ok_or(
                "Save the workspace before running so results and checkpoints can be recorded.",
            )?;
            if let Some(mesh) = self.session.mesh() {
                self.project.mesh_asset = Some(mesh_storage::save(&path, &self.project, mesh)?);
                self.editor.mesh_asset = self.project.mesh_asset.clone();
            }
            workbench::save_workspace(&path, &self.project).map_err(|e| e.to_string())?;
            let case = self.session.prepare_case().map_err(|e| e.to_string())?;
            let review = workbench::solve_review::review(&case, &self.project.plane_monitors);
            if review.blocked() {
                return Err(review.summary());
            }
            let folder = self.solve_recording.clone().unwrap_or_else(|| {
                path.join("monitors").join(format!(
                    "solve-{}",
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap_or_default()
                        .as_nanos()
                ))
            });
            self.segment_start = self.jobs.completed_iterations();
            if self.segment_start == 0 {
                self.solve_histories = Default::default();
                self.histories = Default::default();
            }
            self.iteration = self.segment_start;
            self.limit = self.segment_start.saturating_add(
                self.project
                    .solver
                    .time
                    .parameters()
                    .map(|(_, steps)| steps)
                    .unwrap_or(self.project.solver.max_outer_iterations),
            );
            self.configure_engineering()?;
            self.jobs.set_checkpoint_interval(self.checkpoint_every);
            self.jobs
                .start_solve_with_planes(case, self.project.plane_monitors.clone(), folder.clone())
                .map_err(str::to_string)?;
            self.solve_recording = Some(folder.clone());
            self.recording = Some(folder);
            if let Err(e) = self.persist_continuation() {
                self.message(e);
            }
            self.result_run = "live".into();
            self.result = None;
            self.result_cache = None;
            self.probes.clear();
            self.streamlines.clear();
            self.plane_frames.clear();
            self.plane_playing = false;
            self.live_iteration = usize::MAX;
            self.session.mark_solving();
            self.stage = Stage::Solution;
            self.page = 0;
            self.message("Solve started; continuation retains accepted fields and numbering.");
            Ok(())
        })();
        if let Err(e) = result {
            self.message(e);
        }
    }
    fn activate_solution(&mut self) {
        self.archived_report = None;
        if let Some((m, s)) = self.session.mesh().zip(self.session.solution()) {
            match ResultDataset::from_values(
                "current",
                &m.mesh,
                s.pressure.values().to_vec(),
                s.velocity.values().to_vec(),
            )
            .and_then(|d| d.with_temperature(s.temperature.as_ref().map(|t| t.values().to_vec())))
            {
                Ok(d) => {
                    let d = d.with_turbulence(s.turbulence.as_ref());
                    self.result_cache = ResultRenderCache::build(&d).ok();
                    self.result = Some(d);
                    self.result_run = "current".into();
                    self.result_iteration = self.jobs.completed_iterations();
                }
                Err(e) => self.feedback = e.to_string(),
            };
            if let Ok(c) = self.session.prepare_case() {
                self.wall_reports = workbench::wall_reports::reports_with_turbulence(
                    &c,
                    &s.velocity,
                    &s.pressure,
                    s.turbulence.as_ref(),
                    true,
                )
                .unwrap_or_default();
            }
        }
    }
    fn poll(&mut self) {
        self.poll_workspace_dialog();
        self.poll_compressible();
        self.poll_mapping();
        self.poll_transport();
        if let Some((id, rx)) = &self.frame_scan {
            if let Ok(frames) = rx.try_recv() {
                let id = id.clone();
                self.frame_scan = None;
                if self
                    .project
                    .plane_monitors
                    .get(self.plane_index.saturating_sub(1))
                    .is_some_and(|p| p.id == id)
                {
                    self.plane_frames = frames;
                    self.plane_frame = self.plane_frames.len().saturating_sub(1);
                    self.read_plane_frame();
                }
            }
        }

        if let Some(rx) = &self.loading {
            let received = match rx.try_recv() {
                Ok(result) => Some(result),
                Err(mpsc::TryRecvError::Disconnected) => Some(Err(
                    "Workspace loading stopped unexpectedly; the current project remains open."
                        .into(),
                )),
                Err(mpsc::TryRecvError::Empty) => None,
            };
            if let Some(result) = received {
                self.loading = None;
                match result {
                    Ok((path, p, s, warning)) => match self.set_project(p, s, Some(path)) {
                        Ok(()) => {
                            if let Some(warning) = warning {
                                self.message(format!("Workspace opened; saved mesh was not restored: {warning}. Regenerate the mesh before solving."));
                            } else {
                                self.message("Workspace and saved mesh loaded.");
                            }
                        }
                        Err(e) => self.message(e),
                    },
                    Err(e) => self.message(e),
                }
            }
        }
        if let Some((next, rx)) = &self.publishing {
            let received = match rx.try_recv() {
                Ok(result) => Some(result),
                Err(mpsc::TryRecvError::Disconnected) => Some(Err(
                    "CAD preparation stopped unexpectedly; retry after checking the model.".into(),
                )),
                Err(mpsc::TryRecvError::Empty) => None,
            };
            if let Some(result) = received {
                let next = *next;
                self.publishing = None;
                match result {
                    Ok(published) => {
                        if self.project.cad_geometry_hash.as_ref() != Some(&published.hash) {
                            let mut p = self.session.to_project(self.project.name.clone());
                            p.geometry = published.geometry;
                            p.mesh.dimension = if p.geometry.bodies().next().is_some() {
                                MeshDimension::ThreeD
                            } else {
                                MeshDimension::TwoD
                            };
                            p.fluid_face = None;
                            p.named_selections.clear();
                            p.boundaries.clear();
                            p.mesh_recipe = Default::default();
                            match WorkbenchSession::from_project(&p) {
                                Ok(s) => {
                                    self.session = s;
                                    self.selected.clear();
                                    self.jobs = WorkbenchJobExecutor::new();
                                    self.mesh_cache = None;
                                    self.result = None;
                                    self.result_cache = None;
                                }
                                Err(e) => {
                                    self.message(e.to_string());
                                    return;
                                }
                            }
                        }
                        for mut plane in published.planes {
                            if let Some(old) = self
                                .project
                                .plane_monitors
                                .iter()
                                .find(|p| p.id == plane.id)
                            {
                                plane.enabled = old.enabled;
                                plane.every = old.every;
                                plane.display = old.display.clone();
                            }
                            if let Some(existing) = self
                                .project
                                .plane_monitors
                                .iter_mut()
                                .find(|p| p.id == plane.id)
                            {
                                *existing = plane;
                            } else {
                                self.project.plane_monitors.push(plane);
                            }
                        }
                        self.published_document = Some(published.document.clone());
                        self.project.cad_document = Some(published.document);
                        self.project.cad_geometry_hash = Some(published.hash);
                        self.sync_project();
                        self.editor = self.project.clone();
                        self.dirty = true;
                        self.refresh_geometry();
                        self.stage = next;
                        self.page = 0;
                        self.message(
                            "CAD geometry ready for CFD; define fluid domain and boundary groups.",
                        );
                        if self.saving_after_publish {
                            self.saving_after_publish = false;
                            match self.save() {
                                Ok(true) => {
                                    if let Some(p) = self.pending_replace.take() {
                                        self.replace(p);
                                    }
                                }
                                Ok(false) => {}
                                Err(e) => self.message(e),
                            }
                        }
                    }
                    Err(e) => {
                        self.saving_after_publish = false;
                        self.message(e);
                    }
                }
            }
        }
        for event in self.jobs.drain() {
            match event {
                WorkbenchJobEvent::SolveProgress(JobProgress::StartedAt { iteration }) => {
                    self.append_log(format!("Solve starting after iteration {iteration}"));
                    self.append_log(format!(
                        "Cells: {} · model: {:?} · time: {:?} · additional iterations: {}",
                        self.session.mesh().map_or(0, |m| m.mesh.cell_count()),
                        self.project.solver.turbulence.model,
                        self.project.solver.time,
                        self.project.solver.max_outer_iterations,
                    ));
                    self.append_log(format!(
                        "Relaxation: velocity {} · pressure {} · continuity tolerance {:.3e} · coupling tolerance {:.3e}",
                        self.project.solver.velocity_relaxation, self.project.solver.pressure_relaxation,
                        self.project.solver.continuity_absolute_tolerance, self.project.solver.coupling_relative_tolerance,
                    ));
                    self.append_log("  Iteration       Continuity RMS     Momentum relative     Velocity change      Pressure linear");
                }
                WorkbenchJobEvent::MeshCompleted(result) => match result {
                    Ok(m) => {
                        self.session.install_mesh(m);
                        self.reset_solve_session();
                        if let Err(e) = self.persist_continuation() {
                            self.message(e);
                        }
                        self.show_geometry = false;
                        self.refresh_mesh();
                        self.sync_project();
                        self.editor = self.project.clone();
                        self.dirty = true;
                        self.message("Mesh generated. Inspect quality and assign physical boundary conditions.");
                    }
                    Err(e) => self.message(e.to_string()),
                },
                WorkbenchJobEvent::SolveProgress(JobProgress::Iteration {
                    current,
                    max,
                    residual,
                }) => {
                    self.iteration = current;
                    self.limit = max;
                    if let Some(r) = residual.filter(|_| current > self.segment_start) {
                        continuation::insert_sample(&mut self.solve_histories[0], current, Some(r));
                    }
                }
                WorkbenchJobEvent::SolveProgress(JobProgress::Metrics { iteration, values }) => {
                    self.append_log(format!(
                        "{iteration:>11} {0:>20.8e} {1:>21.8e} {2:>19.8e} {3:>20.8e}",
                        values[0], values[1], values[2], values[3],
                    ));
                    for (i, v) in values.into_iter().enumerate().skip(1) {
                        if iteration <= self.segment_start {
                            continue;
                        }
                        continuation::insert_sample(
                            &mut self.solve_histories[i],
                            iteration,
                            Some(v),
                        );
                    }
                }
                WorkbenchJobEvent::SolveProgress(JobProgress::Message(m)) => self.message(m),
                WorkbenchJobEvent::SolveCompleted(result) => {
                    self.iteration = self.jobs.completed_iterations();
                    match &result {
                        Ok(solution) => self.message(format!(
                            "Solve {:?} at iteration {} · continuity RMS {:.8e} · inflow {:.8e} · outflow {:.8e} · net flux {:.8e}",
                            solution.report.status, self.iteration,
                            solution.report.final_continuity_rms, solution.report.total_inflow,
                            solution.report.total_outflow, solution.report.net_boundary_flux,
                        )),
                        Err(error) => {
                            self.show_log = true;
                            self.message(format!("ERROR after iteration {}: {error}", self.iteration));
                        }
                    }
                    for series in &mut self.solve_histories {
                        series.retain(|(x, _)| *x <= self.iteration as f64);
                    }
                    self.session.complete_solve(result);
                    if self.export.is_none() {
                        self.activate_solution();
                    }
                    if let Some(path) = self.workspace.clone() {
                        self.sync_project();
                        match workbench::finalize_workspace_run_continuing(
                            &path,
                            &mut self.project,
                            &self.session,
                            self.continuation_run.as_deref(),
                        ) {
                            Ok(Some(run)) => {
                                self.append_log(format!(
                                    "Run report: {}",
                                    path.join(&run.report_path).display()
                                ));
                                self.continuation_run = Some(run.id.clone());
                                self.histories = self.solve_histories.clone();
                                if let Err(e) = self.persist_continuation() {
                                    self.message(e);
                                }
                                let metadata = self.save_run_display_metadata(&run);
                                self.result_run = run.id.clone();
                                match metadata {
                                    Ok(()) => self
                                        .message(format!("Run {} saved: {:?}", run.id, run.status)),
                                    Err(e) => self.message(format!(
                                        "Run {} saved, but display metadata failed: {e}",
                                        run.id
                                    )),
                                }
                            }
                            Ok(None) => {}
                            Err(e) => self.message(e.to_string()),
                        }
                    }
                    self.recording = self.solve_recording.clone();
                    self.refresh_field_frames();
                    self.editor = self.project.clone();
                }
                WorkbenchJobEvent::WallReports {
                    iteration,
                    physical_time,
                    reports,
                } => {
                    let sample = force_analysis::Sample {
                        index: iteration,
                        time: physical_time,
                        density: self.session.material().density,
                        reports: reports.clone(),
                    };
                    if let Some(old) = self
                        .force_history
                        .last_mut()
                        .filter(|s| s.index == iteration && s.time == physical_time)
                    {
                        *old = sample;
                    } else {
                        self.force_history.push(sample);
                    }
                    self.wall_reports = reports;
                }
                WorkbenchJobEvent::PlaneSample(s) => {
                    if self.live_fields
                        && !self.plane_playing
                        && !self.field_playing
                        && (self.result.is_none()
                            || self.result_run == "live"
                            || self.result_run == "current")
                        && self
                            .project
                            .plane_monitors
                            .get(self.plane_index.saturating_sub(1))
                            .is_some_and(|p| p.id == s.plane.id)
                        && self.plane_index > 0
                    {
                        let mut s = s;
                        if let Some(current) = &self.plane_snapshot {
                            if current.plane.id == s.plane.id {
                                s.plane.display = current.plane.display.clone();
                            }
                        }
                        self.plane_snapshot = Some(s);
                    }
                }
                WorkbenchJobEvent::TimeStep {
                    index,
                    time,
                    dt,
                    courant,
                    rejected,
                } => {
                    self.monitoring.time_state = Some((index, time, dt, courant, rejected));
                }
                WorkbenchJobEvent::Engineering(sample) => self.accept_engineering(sample),
                WorkbenchJobEvent::Health(health) => self.append_log(health.summary()),
                WorkbenchJobEvent::Performance(p) => self.session.set_performance(p),
                WorkbenchJobEvent::MeshProgress(JobProgress::Meshing { fraction, stage }) => {
                    self.feedback = format!("Meshing {:.0}% · {stage}", fraction * 100.);
                }
                _ => {}
            }
        }
        if self.plane_playing
            && self.plane_tick.elapsed().as_secs_f64() > 1.0 / self.playback_fps.max(1.)
            && !self.plane_frames.is_empty()
        {
            self.plane_tick = Instant::now();
            self.plane_frame = (self.plane_frame + 1) % self.plane_frames.len();
            self.read_plane_frame();
        }
    }
    fn read_plane_frame(&mut self) -> bool {
        if let Some(path) = self.plane_frames.get(self.plane_frame) {
            match std::fs::read(path)
                .map_err(|e| e.to_string())
                .and_then(|b| {
                    serde_json::from_slice::<plane_monitor::PlaneSnapshot>(&b)
                        .map_err(|e| e.to_string())
                }) {
                Ok(mut s) => {
                    if let Some(current) = self
                        .plane_snapshot
                        .as_ref()
                        .filter(|current| current.plane.id == s.plane.id)
                    {
                        s.plane.display = current.plane.display.clone();
                    }
                    for probe in &mut self.probes {
                        if let Some(sample) = s
                            .samples
                            .iter()
                            .find(|v| v.geometry.cell == probe.cell_index)
                        {
                            probe.pressure = sample.pressure;
                            probe.velocity = sample.velocity;
                            probe.velocity_magnitude = sample.velocity.norm();
                            probe.run_id = format!("{}@{}", self.result_run, s.iteration);
                        }
                    }
                    self.plane_snapshot = Some(s);
                    return true;
                }
                Err(e) => self.message(e),
            }
        }
        false
    }
}
impl eframe::App for App {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll();
        self.poll_angle_design_points();
        self.poll_field_frame(ctx);
        self.seed_case_probes();
        self.post_logic(ctx);
        if self.capture.is_some() {
            for event in ctx.input(|i| i.events.clone()) {
                if let egui::Event::Screenshot { image, .. } = event {
                    if let Some(path) = self.capture.take() {
                        let result = (|| -> Result<(), String> {
                            let file = std::fs::File::create(path).map_err(|e| e.to_string())?;
                            let mut encoder =
                                png::Encoder::new(file, image.size[0] as u32, image.size[1] as u32);
                            encoder.set_color(png::ColorType::Rgba);
                            encoder.set_depth(png::BitDepth::Eight);
                            encoder
                                .write_header()
                                .map_err(|e| e.to_string())?
                                .write_image_data(image.as_raw())
                                .map_err(|e| e.to_string())
                        })();
                        if let Err(e) = result {
                            eprintln!("Preview capture: {e}");
                        }
                        self.save_error = Some("quit-confirmed".into());
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                }
            }
            if self.capture_prepared.is_none()
                && self.created.elapsed() > Duration::from_secs(2)
                && !self.busy()
            {
                if let Ok(run) = std::env::var("FLURSYS_EGUI_CAPTURE_RUN") {
                    if let Err(e) = self.open_run(&run) {
                        eprintln!("Preview archive: {e}");
                    }
                }
                if let Ok(stage) = std::env::var("FLURSYS_EGUI_CAPTURE_STAGE") {
                    match stage.as_str() {
                        "mesh" => {
                            self.stage = Stage::Mesh;
                        }
                        "quality" => {
                            self.stage = Stage::Mesh;
                            self.page = 2;
                            self.show_geometry = false;
                        }
                        "display" => {
                            self.stage = Stage::Solution;
                            self.page = 7;
                            self.ensure_result();
                        }
                        "setup" => {
                            self.stage = Stage::Setup;
                        }
                        "solution" => {
                            self.stage = Stage::Solution;
                        }
                        "results" => {
                            self.stage = Stage::Results;
                        }
                        "initialize" => {
                            self.stage = Stage::Solution;
                            self.show_initialization = true;
                        }
                        "reports" => {
                            self.stage = Stage::Solution;
                            self.page = 6;
                        }
                        "thermal" => {
                            self.stage = Stage::Setup;
                            self.page = 2;
                        }
                        "models" => {
                            self.stage = Stage::Solution;
                            self.page = 3;
                        }
                        "methods" => {
                            self.stage = Stage::Solution;
                            self.page = 4;
                        }
                        "profiles" => {
                            self.stage = Stage::Results;
                            self.page = 3;
                            self.ensure_result();
                        }
                        "settings" => {
                            self.stage = Stage::Mesh;
                            self.show_preferences = true;
                        }
                        _ => {}
                    }
                }
                self.capture_prepared = Some(Instant::now());
            }
            if !self.capture_requested
                && self
                    .capture_prepared
                    .is_some_and(|t| t.elapsed() > Duration::from_millis(800))
            {
                self.capture_requested = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
            }
            ctx.request_repaint_after(Duration::from_millis(50));
        }
        if self.stage == Stage::Geometry {
            self.cad.logic(ctx);
        }
        if self.busy()
            || self.plane_playing
            || self.field_playing
            || self.flow_playing
            || self.export.is_some()
            || self.probe_history_job.is_some()
        {
            ctx.request_repaint_after(Duration::from_millis(33));
        }
        if ctx.input(|i| i.viewport().close_requested())
            && self.save_error.as_deref() != Some("quit-confirmed")
        {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.request(Pending::Quit);
        }
        if self.save_error.as_deref() == Some("quit-confirmed") {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.export_footer(ui);
        if let Some(job) = &self.export {
            self.camera = job.frozen_camera();
        }
        ui.scope(|ui| {
            // Lock interaction, not rendering: disabled widgets normally fade every
            // painted primitive, including the viewport captured for PNG/GIF.
            if self.export.is_some() {
                ui.visuals_mut().disabled_alpha = 1.0;
            }
            ui.add_enabled_ui(self.export.is_none(), |ui| self.shell(ui));
        });
        self.gif_settings_window(ui.ctx());
        self.request_export_capture(ui.ctx());
    }
    fn raw_input_hook(&mut self, _ctx: &egui::Context, input: &mut egui::RawInput) {
        if self.stage == Stage::Geometry {
            self.cad.raw_input_hook(input);
        }
    }
}
