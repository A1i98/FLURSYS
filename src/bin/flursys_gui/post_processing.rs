//! Result-workspace interaction, illustrative steady-flow animation and exports.
use super::*;
use slint::Model;
#[path = "post_export.rs"]
mod post_export;
use post_export::{save_gif, save_png, write_probes};

#[derive(Default)]
pub(super) struct State {
    pub metric_histories: [Vec<f64>; 3],
    pub continuity_full: Vec<f64>,
    pub plane_samples:
        std::collections::BTreeMap<String, flursys::workbench::plane_monitor::PlaneSnapshot>,
    pub profile: Vec<result_analysis::ProfilePoint>,
    pub profile_label: String,
    pub comparison: profile_comparison::State,
    pub monitors: monitor_history::State,
    pub boundary_summary: String,
    pub wall_report_index: Option<usize>,
    pub wall_report_time: Option<f64>,
    pub wall_reports: Vec<flursys::workbench::wall_reports::WallPatchReport>,
    pub boundary_reports: Vec<flursys::workbench::boundary_reports::BoundaryFlowReport>,
    pub plane_frames: Vec<PathBuf>,
    pub plane_range_job:
        Option<std::sync::mpsc::Receiver<Result<(String, usize, [f64; 2]), String>>>,
    pub plane_statistics_job: Option<std::sync::mpsc::Receiver<Result<String, String>>>,
    pub plane_frame: usize,
    pub section_probe: RefCell<Option<Vec3>>,
    pub section_camera: RefCell<flursys::workbench::plane_monitor::SectionCamera>,
    pub section_archive: RefCell<
        Option<(
            PathBuf,
            std::sync::Arc<flursys::workbench::plane_monitor::PlaneSnapshot>,
        )>,
    >,
    pub plane_playing: bool,
    pub plane_tick: Option<std::time::Instant>,
    pub playing: bool,
    pub phase: f64,
    pub probes: Vec<ResultProbe>,
    traces: Vec<TimedPath>,
    base: RefCell<Option<slint::Image>>,
    last_tick: Option<std::time::Instant>,
    export: Option<std::sync::mpsc::Receiver<Result<PathBuf, String>>>,
    export_status: String,
    pub started: Option<std::time::Instant>,
    pub elapsed: f64,
    pub iteration: usize,
    pub limit: usize,
}
impl State {
    pub fn reset_result(&mut self) {
        self.iteration = 0;
        self.limit = 0;
        self.playing = false;
        self.phase = 0.0;
        self.probes.clear();
        self.profile.clear();
        self.profile_label.clear();
        self.boundary_summary.clear();
        self.boundary_reports.clear();
        self.wall_reports.clear();
        self.wall_report_index = None;
        self.wall_report_time = None;
        self.traces.clear();
        self.metric_histories = Default::default();
        self.continuity_full.clear();
        self.plane_samples.clear();
        self.monitors.clear();
        self.plane_frames.clear();
        *self.section_probe.borrow_mut() = None;
        *self.section_archive.borrow_mut() = None;
        *self.section_camera.borrow_mut() = Default::default();
        self.plane_statistics_job = None;
        self.plane_range_job = None;
        self.plane_playing = false;
        *self.base.get_mut() = None;
    }
    pub fn start_solve(&mut self, continuing: bool) {
        self.started = Some(std::time::Instant::now());
        self.elapsed = 0.0;
        if !continuing {
            self.iteration = 0;
            self.metric_histories = Default::default();
            self.continuity_full.clear();
        }
        self.plane_samples.clear();
        self.monitors.clear();
        self.plane_frames.clear();
        *self.section_probe.borrow_mut() = None;
        *self.section_archive.borrow_mut() = None;
        *self.section_camera.borrow_mut() = Default::default();
        self.plane_statistics_job = None;
        self.plane_range_job = None;
        self.plane_playing = false;
        self.playing = false;
    }
    pub fn finish_solve(&mut self) {
        if let Some(start) = self.started.take() {
            self.elapsed = start.elapsed().as_secs_f64();
        }
    }
}
#[derive(Clone)]
struct TimedPath {
    points: Vec<Vec3>,
    times: Vec<f64>,
}
impl TimedPath {
    fn new(path: &StreamlinePath, field: &StreamlineField) -> Option<Self> {
        if path.points.len() < 2 {
            return None;
        }
        let mut times = vec![0.0];
        let mut time = 0.0;
        for pair in path.points.windows(2) {
            let speed = field.velocity_at((pair[0] + pair[1]) * 0.5)?.norm();
            if speed <= 1e-12 {
                return None;
            }
            time += (pair[1] - pair[0]).norm() / speed;
            times.push(time);
        }
        if !(time > 0.0 && time.is_finite()) {
            return None;
        }
        for t in &mut times {
            *t /= time;
        }
        Some(Self {
            points: path.points.clone(),
            times,
        })
    }
    fn at(&self, phase: f64) -> Vec3 {
        let phase = phase.rem_euclid(1.0);
        let i = self
            .times
            .partition_point(|t| *t <= phase)
            .saturating_sub(1)
            .min(self.times.len() - 2);
        let f = (phase - self.times[i]) / (self.times[i + 1] - self.times[i]).max(1e-30);
        self.points[i] + (self.points[i + 1] - self.points[i]) * f
    }
}
fn markers(traces: &[TimedPath], phase: f64) -> Vec<(f64, f64)> {
    traces
        .iter()
        .flat_map(|p| {
            (0..4).map(move |i| {
                let p = p.at(phase + i as f64 / 4.0);
                (p.x, p.y)
            })
        })
        .collect()
}
pub(super) fn field(index: i32) -> ResultFieldKind {
    match index {
        1 => ResultFieldKind::Pressure,
        2 => ResultFieldKind::VorticityMagnitude,
        3 => ResultFieldKind::Temperature,
        4 => ResultFieldKind::VelocityX,
        5 => ResultFieldKind::VelocityY,
        6 => ResultFieldKind::VelocityZ,
        7 => ResultFieldKind::VelocityDivergence,
        8 => ResultFieldKind::TurbulentK,
        9 => ResultFieldKind::SpecificDissipation,
        10 => ResultFieldKind::TurbulentViscosity,
        11 => ResultFieldKind::QCriterion,
        12 => ResultFieldKind::FlowType,
        13 => ResultFieldKind::StrainRate,
        _ => ResultFieldKind::VelocityMagnitude,
    }
}
fn update_traces(state: &mut AppState) -> Result<(), String> {
    let dataset = state.active_result().ok_or("Select a result run first")?;
    let velocity = StreamlineField::from_dataset(dataset).map_err(|e| e.to_string())?;
    state.post.traces = state
        .streamline_paths
        .iter()
        .filter_map(|p| TimedPath::new(p, &velocity))
        .collect();
    if state.post.traces.is_empty() {
        return Err("No moving streamline paths. Add seeds inside the fluid domain.".into());
    }
    Ok(())
}
fn auto_seed(state: &mut AppState) -> Result<usize, String> {
    let dataset = state.active_result().ok_or("Select a result run first")?;
    let velocity = StreamlineField::from_dataset(dataset).map_err(|e| e.to_string())?;
    let cache = state.result_render_cache().unwrap();
    let (lo, hi) = cache.mesh_cache().bounds();
    let length = (hi.x - lo.x).max(hi.y - lo.y);
    let mut paths = Vec::new();
    for iy in 0..4 {
        for ix in 0..4 {
            let seed = Vec3::new(
                lo.x + (hi.x - lo.x) * (ix as f64 + 0.5) / 4.0,
                lo.y + (hi.y - lo.y) * (iy as f64 + 0.5) / 4.0,
                lo.z,
            );
            if velocity.locate_cell(seed).is_none() {
                continue;
            }
            if let Ok(path) = velocity.integrate(
                seed,
                StreamlineDirection::Forward,
                StreamlineOptions {
                    step_size: length / 250.0,
                    max_steps: 1200,
                    max_length: 5.0 * length,
                    stagnation_speed: 1e-12,
                },
            ) {
                if path.points.len() > 1 {
                    paths.push(path);
                }
            }
        }
    }
    let n = paths.len();
    state.streamline_paths = paths;
    update_traces(state)?;
    Ok(n)
}
fn feedback(ui: &MainWindow, state: &mut AppState, message: String) {
    ui.set_post_feedback(message.clone().into());
    state.log(message);
}

pub(super) fn chart_history(state: &AppState, kind: i32) -> VecDeque<ResidualSample> {
    if state.current_step == 3 && !state.solving {
        if let Some(solution) = state.workbench.solution() {
            let values = match kind {
                1 => &solution.report.simple.momentum_relative_history,
                2 => &solution.report.simple.velocity_change_history,
                3 => &solution.report.simple.pressure_linear_history,
                _ => &solution.report.continuity_history,
            };
            return values
                .iter()
                .rev()
                .take(400)
                .rev()
                .map(|v| ResidualSample {
                    continuity: *v,
                    momentum: 0.0,
                    pressure: 0.0,
                })
                .collect();
        }
    }
    if kind == 0 {
        return state.wb_residual_history.clone();
    }
    state.post.metric_histories[(kind - 1).clamp(0, 2) as usize]
        .iter()
        .rev()
        .take(400)
        .rev()
        .map(|v| ResidualSample {
            continuity: *v,
            momentum: 0.0,
            pressure: 0.0,
        })
        .collect()
}
fn chart_caption(state: &AppState, kind: i32) -> String {
    let name = match kind {
        1 => "Momentum · relative",
        2 => "Velocity change · relative",
        3 => "Pressure linear solve · final residual",
        _ => "Continuity · RMS",
    };
    let samples = chart_history(state, kind);
    let values = samples
        .iter()
        .map(|v| v.continuity)
        .filter(|v| v.is_finite() && *v > 0.0)
        .collect::<Vec<_>>();
    if values.is_empty() && !samples.is_empty() && samples.iter().all(|v| v.continuity == 0.0) {
        return format!(
            "{name} · all {} recorded samples are zero; logarithmic scale is unavailable.",
            samples.len()
        );
    }
    if values.is_empty() {
        return format!("{name} · no positive history available. Additional metrics are recorded on new solves.");
    }
    let lo = values.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = values.iter().copied().fold(0.0, f64::max);
    format!(
        "{name} · logarithmic scale · {lo:.3e} … {hi:.3e} · last {} samples",
        samples.len()
    )
}

pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_profile(move |preset| {
        let Some(ui)=weak.upgrade() else{return};let mut state=shared.borrow_mut();
        let result=(||->Result<(),String>{
            let data=state.active_result().ok_or("Select a completed result first")?;
            if preset==1 || preset==2 {
                let cache=state.result_render_cache().ok_or("No result mesh")?;
                let (lo,hi)=cache.mesh_cache().bounds();
                let (a,b)=if preset==1 {(Vec3::new((lo.x+hi.x)*0.5,lo.y,0.0),Vec3::new((lo.x+hi.x)*0.5,hi.y,0.0))}else{(Vec3::new(lo.x,(lo.y+hi.y)*0.5,0.0),Vec3::new(hi.x,(lo.y+hi.y)*0.5,0.0))};
                ui.set_post_line_x0(a.x.to_string().into());ui.set_post_line_y0(a.y.to_string().into());ui.set_post_line_x1(b.x.to_string().into());ui.set_post_line_y1(b.y.to_string().into());
            }
            let parse=|s:SharedString|s.parse::<f64>().ok().filter(|v|v.is_finite()).ok_or("Endpoints must be finite numbers".to_string());
            let a=Vec3::new(parse(ui.get_post_line_x0())?,parse(ui.get_post_line_y0())?,0.0);
            let b=Vec3::new(parse(ui.get_post_line_x1())?,parse(ui.get_post_line_y1())?,0.0);
            let f=field(ui.get_result_field_index());
            let points=result_analysis::sample_line(data,a,b,ui.get_post_line_samples() as usize,f)?;
            let valid=points.iter().filter(|p|p.value.is_some()).count();
            let values=points.iter().filter_map(|p|p.value).collect::<Vec<_>>();
            if values.is_empty(){return Err("The line has no samples inside the fluid domain".into());}
            let lo=values.iter().copied().fold(f64::INFINITY,f64::min);let hi=values.iter().copied().fold(f64::NEG_INFINITY,f64::max);
            state.post.profile_label=format!("{} · distance from ({:.4e}, {:.4e}) to ({:.4e}, {:.4e})",f.label(),a.x,a.y,b.x,b.y);
            ui.set_post_profile_caption(format!("{}\n{} / {} inside · min {:.5e} · max {:.5e}\nHorizontal: distance 0 … {:.5e}; vertical: selected field value. Gaps = outside fluid.",state.post.profile_label,valid,points.len(),lo,hi,(b-a).norm()).into());
            ui.set_post_profile_image(slint::Image::load_from_svg_data(result_analysis::profile_svg(&points,f.label(),false).as_bytes()).map_err(|e|e.to_string())?);
            let caption=ui.get_post_profile_caption().to_string()+"\n"+&result_analysis::line_measurements(&points);
            ui.set_post_profile_caption(caption.into());
            state.post.profile=points;
            Ok(())
        })();
        if let Err(e)=result{feedback(&ui,&mut state,e);}
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_refresh(move || {
        if let Some(ui) = weak.upgrade() {
            refresh_ui(&ui, &shared.borrow());
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_view_fit(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if let Some(cache) = state.result_render_cache() {
            let (lo, hi) = cache.mesh_cache().bounds();
            state.mesh_view.fit(Some((lo.x, lo.y, hi.x, hi.y)));
        }
        state.mesh_pan_anchor = None;
        state.geometry_drag_anchor = None;
        state.view_3d.fit();
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_view_zoom(move |delta| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state
            .active_result()
            .is_some_and(|r| r.mesh().dimension() == MeshDimension::ThreeD)
            && ui.get_result_slice_axis() == 0
        {
            state.view_3d.zoom_by(delta);
            refresh_ui(&ui, &state);
            return;
        }
        if state
            .active_result()
            .is_none_or(|r| r.mesh().dimension() != MeshDimension::TwoD)
        {
            return;
        }
        state.mesh_view.zoom_at(
            (PREVIEW_WIDTH as f64 / 2.0, PREVIEW_HEIGHT as f64 / 2.0),
            f64::from(delta),
        );
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_view_drag(move |x, y, px, py, w, h| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state
            .active_result()
            .is_some_and(|r| r.mesh().dimension() == MeshDimension::ThreeD)
            && ui.get_result_slice_axis() == 0
        {
            if state
                .geometry_drag_anchor
                .is_none_or(|a| a.0 != px || a.1 != py)
            {
                state.geometry_drag_anchor = Some((px, py, state.view_3d.yaw, state.view_3d.pitch));
            }
            if let Some((sx, sy, yaw, pitch)) = state.geometry_drag_anchor {
                let scale = viewport::preview_scale(w, h);
                state
                    .view_3d
                    .orbit_from(yaw, pitch, (x - sx) / scale, (y - sy) / scale);
                state.geometry_view_axis = 7;
            }
            refresh_ui(&ui, &state);
            return;
        }
        if state
            .active_result()
            .is_none_or(|r| r.mesh().dimension() != MeshDimension::TwoD)
        {
            return;
        }
        if state.mesh_pan_anchor.is_none_or(|a| a.0 != px || a.1 != py) {
            state.mesh_pan_anchor =
                Some((px, py, state.mesh_view.center_x, state.mesh_view.center_y));
        }
        if let Some((_, _, cx, cy)) = state.mesh_pan_anchor {
            let scale = (w.max(1.0) / PREVIEW_WIDTH as f32).min(h.max(1.0) / PREVIEW_HEIGHT as f32);
            state.mesh_view.center_x =
                cx - f64::from((x - px) / scale) / state.mesh_view.pixels_per_unit;
            state.mesh_view.center_y =
                cy + f64::from((y - py) / scale) / state.mesh_view.pixels_per_unit;
        }
        refresh_ui(&ui, &state);
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_select_run(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        let id = state
            .workbench_project
            .runs
            .iter()
            .filter(|r| r.solution_path.is_some())
            .nth(index.max(0) as usize)
            .map(|r| r.id.clone());
        if let Some(id) = id {
            if let Err(e) = state.activate_persisted_result(&id) {
                feedback(&ui, &mut state, e);
            }
            plane_monitors::refresh_selected(&ui, &mut state);
            state.show_mesh = false;
            state.show_geometry_3d = false;
            state.current_step = 4;
            ui.set_inspector_mode(5);
            ui.set_current_step(4);
            rebuild_tree_rows(&mut state);
            refresh_ui(&ui, &state);
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_action(move|action|{
        let Some(ui)=weak.upgrade()else{return};let mut state=shared.borrow_mut();
        let result=(||->Result<String,String>{match action {
            0=>{let n=auto_seed(&mut state)?;Ok(format!("Added {n} streamline seeds. Paths use reconstructed cell-centred velocity."))}
            1=>{state.streamline_paths.clear();state.post.traces.clear();state.post.playing=false;Ok("Streamlines cleared".into())}
            2=>{
                if ui.get_post_animation_mode()==0 {
                    if state.streamline_paths.is_empty(){auto_seed(&mut state)?;}else{update_traces(&mut state)?;}
                }else{
                    if state.active_result().is_none_or(|r|r.mesh().dimension()!=MeshDimension::ThreeD){return Err("Slice animation requires a 3D result".into());}
                    if ui.get_result_slice_axis()==0 {ui.set_result_slice_axis(1);}
                }
                state.post.playing = !state.post.playing;state.post.last_tick=Some(std::time::Instant::now());
                state.show_mesh=false;state.show_geometry_3d=false;
                Ok(if state.post.playing {"Animation playing · steady-field visualization, not a transient CFD solution"}else{"Animation paused"}.into())
            }
            3=>{state.post.playing=false;state.post.phase=0.0;Ok("Animation reset".into())}
            4=>{state.post.probes.clear();state.active_result_probe=None;Ok("Probes cleared".into())}
            5=>{state.post.playing=false;state.streamline_seed_mode=true;state.show_mesh=false;state.show_geometry_3d=false;Ok("Click inside the 2D fluid to place a streamline seed".into())}
            6=>{state.show_mesh = !state.show_mesh;state.show_geometry_3d=false;Ok("Result view updated".into())}
            _=>Err("Unknown result action".into()),
        }})();
        feedback(&ui,&mut state,result.unwrap_or_else(|e|e));refresh_ui(&ui,&state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_seek(move |phase| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        state.post.playing = false;
        state.post.phase = f64::from(phase.clamp(0.0, 1000.0)) / 1000.0;
        if ui.get_post_animation_mode() == 1 {
            if ui.get_result_slice_axis() == 0 {
                ui.set_result_slice_axis(1);
            }
            ui.set_result_slice_percent((1.0 + state.post.phase * 98.0) as i32);
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_select_probe(move |index| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        state.active_result_probe = state.post.probes.get(index.max(0) as usize).cloned();
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_post_export(move |kind| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.post.export.is_some() {
            feedback(&ui, &mut state, "An export is already running".into());
            return;
        }
        let (name, ext) = match kind {
            0 => ("field.png", "png"),
            1 => ("flow-tracers.gif", "gif"),
            2 => ("probes.csv", "csv"),
            3 if state.current_step==3 && ui.get_wb_chart_kind()>=5 => ("section-monitor.csv","csv"),
            3 => ("convergence.csv", "csv"),
            5 => ("run-report.json", "json"),
            6 => ("convergence.png", "png"),
            7 => ("cfd-report.html", "html"),
            8 => ("line-profile.csv", "csv"),
            9 => ("line-profile.svg", "svg"),
            10 => ("boundary-flows.csv", "csv"),
            11 => ("convergence.svg", "svg"),
            12 => ("field-statistics.csv", "csv"),
            13 => ("wall-patches.csv","csv"),14=>("wall-faces.csv","csv"),
            _ => ("solution.vtk", "vtk"),
        };
        if kind == 1 {
            if state
                .active_result()
                .is_none_or(|r| r.mesh().dimension() != MeshDimension::TwoD)
            {
                feedback(
                    &ui,
                    &mut state,
                    "GIF export currently supports 2D steady-flow tracers".into(),
                );
                return;
            }
            if state.post.traces.is_empty() {
                if let Err(e) = auto_seed(&mut state) {
                    feedback(&ui, &mut state, e);
                    return;
                }
            }
        }
        let Some(path) = rfd::FileDialog::new()
            .set_title("Export result")
            .add_filter(ext, &[ext])
            .set_file_name(name)
            .save_file()
        else {
            return;
        };
        let result = (|| -> Result<(), String> {
            match kind {
                0 => save_png(&path, &ui.get_visualization_image()),
                6 => save_png(&path, &ui.get_residual_image()),
                13|14=>{
                    if state.post.wall_reports.is_empty() {return Err("No wall report is available for this run".into());}
                    if kind==14 && state.post.wall_reports.iter().all(|r|r.samples.is_empty()) {return Err("Per-face export requires a completed run with detailed wall samples".into());}
                    result_analysis::atomic_text(&path,&flursys::workbench::wall_reports::csv(&state.post.wall_reports,kind==14))
                },
                12 => result_analysis::atomic_text(&path,&result_analysis::engineering_summary_csv(&state)?),
                7 => result_analysis::atomic_text(&path,&result_analysis::report(&state)?),
                10 => result_analysis::atomic_text(&path,&result_analysis::boundary_csv(&state.post.boundary_reports)?),
                11 => result_analysis::atomic_text(&path,&convergence_plot::build(&state,&convergence_plot::Options::from_ui(&ui)).svg),
                8 | 9 => {
                    if state.post.profile.is_empty(){return Err("Sample a line profile first".into());}
                    let text=if kind==8{result_analysis::profile_csv(&state.post.profile,&state.post.profile_label)}else{result_analysis::profile_svg(&state.post.profile,&state.post.profile_label,false)};
                    result_analysis::atomic_text(&path,&text)
                },
                1 => {
                    let base = state
                        .post
                        .base
                        .borrow()
                        .as_ref()
                        .ok_or("Display a result field before exporting")?
                        .to_rgba8()
                        .ok_or("No rendered pixels")?;
                    let (w, h) = (base.width(), base.height());
                    let pixels = base.as_bytes().to_vec();
                    let traces = state.post.traces.clone();
                    let view = state.mesh_view;
                    let destination = path.clone();
                    let (tx, rx) = std::sync::mpsc::channel();
                    std::thread::spawn(move || {
                        let r = save_gif(&destination, w, h, &pixels, &traces, &view)
                            .map(|_| destination);
                        let _ = tx.send(r);
                    });
                    state.post.export = Some(rx);
                    state.post.export_status = "Encoding GIF…".into();
                    Ok(())
                }
                2 => write_probes(&path, &state.post.probes),
                3 if state.current_step==3 && ui.get_wb_chart_kind()>=5 => result_analysis::atomic_text(&path,&monitor_history::csv(&state,ui.get_flow_plane_index().max(0) as usize)?),
                3 => {
                    let history=result_analysis::histories(&state);
                    let count=history.iter().map(Vec::len).max().unwrap_or(0);
                    let recorded=state.active_result().and_then(|a|state.workbench_project.runs.iter().find(|r|r.id==a.run_id())).and_then(|r|r.iterations);
                    let current=state.workbench.solution().map(|s|s.report.outer_iterations);
                    let last=if state.solving {state.post.iteration}else if state.current_step==3 {current.or(recorded).unwrap_or(count.saturating_sub(1))}else{recorded.or(current).unwrap_or(count.saturating_sub(1))};
                    let first=last.saturating_add(1).saturating_sub(count);
                    let mut data=String::from("iteration,continuity_rms,momentum_relative,velocity_change_relative,pressure_linear_final\n");
                    for row in 0..count {
                        data.push_str(&(first+row).to_string());
                        for series in &history {
                            data.push(',');
                            if let Some(index)=row.checked_sub(count-series.len()) {data.push_str(&format!("{:.17e}",series[index]));}
                        }
                        data.push('\n');
                    }
                    result_analysis::atomic_text(&path, &data)
                }
                _ => {
                    let active = state.active_result().ok_or("Select a saved result first")?;
                    let run = state
                        .workbench_project
                        .runs
                        .iter()
                        .find(|r| r.id == active.run_id())
                        .ok_or("Save the completed run before exporting its VTK artifact")?;
                    let source = state
                        .workspace_path
                        .as_ref()
                        .ok_or("No saved workspace")?
                        .join(if kind == 5 {
                            &run.report_path
                        } else {
                            run.solution_path.as_ref().ok_or("No VTK artifact")?
                        });
                    if source == path
                        || std::fs::canonicalize(&source)
                            .ok()
                            .zip(std::fs::canonicalize(&path).ok())
                            .is_some_and(|(a, b)| a == b)
                    {
                        return Err("Choose a different path from the original run artifact".into());
                    }
                    std::fs::copy(source, &path)
                        .map(|_| ())
                        .map_err(|e| e.to_string())
                }
            }
        })();
        feedback(
            &ui,
            &mut state,
            match result {
                Ok(()) => format!(
                    "Export {}: {}",
                    if kind == 1 { "started" } else { "saved" },
                    path.display()
                ),
                Err(e) => format!("Export failed: {e}"),
            },
        );
        refresh_ui(&ui, &state);
    });
}

pub(super) fn tick(ui: &MainWindow, state: &mut AppState) -> bool {
    ui.set_solver_paused(state.solving && state.workbench_jobs.is_paused());
    if state.post.plane_playing
        && ((state.current_step == 4 && ui.get_result_slice_axis() >= 4)
            || (state.current_step == 3 && ui.get_solution_show_plane()))
    {
        let now = std::time::Instant::now();
        if state.post.plane_tick.is_none_or(|t| {
            now.duration_since(t).as_millis() >= (1000 / ui.get_flow_plane_fps().max(1)) as u128
        }) {
            state.post.plane_tick = Some(now);
            let (first, last) = plane_monitors::playback_bounds(ui, state.post.plane_frames.len());
            if state.post.plane_frame < first || state.post.plane_frame > last {
                state.post.plane_frame = first;
            } else if state.post.plane_frame < last {
                state.post.plane_frame += 1;
            } else if ui.get_flow_plane_loop() && !state.post.plane_frames.is_empty() {
                state.post.plane_frame = first;
            } else {
                state.post.plane_playing = false;
            }
            plane_monitors::show_latest(ui, state);
        }
    }

    let mut changed = false;
    if let Some(result) = state.post.monitors.poll() {
        if let Err(e) = result {
            state.log(format!("Monitor history: {e}"));
        }
        changed = true;
    }
    if let Some(rx) = state.post.plane_range_job.take() {
        match rx.try_recv() {
            Ok(Ok((id, field, range))) => {
                if let Some((index, plane)) = state
                    .workbench_project
                    .plane_monitors
                    .iter_mut()
                    .enumerate()
                    .find(|(_, p)| p.id == id && p.display.field == field)
                {
                    plane.display.range = Some(range);
                    if ui.get_flow_plane_index() == index as i32 {
                        ui.set_flow_plane_min(range[0].to_string().into());
                        ui.set_flow_plane_max(range[1].to_string().into());
                        ui.set_result_range_min(range[0].to_string().into());
                        ui.set_result_range_max(range[1].to_string().into());
                        ui.set_flow_plane_render_key("".into());
                    }
                    state.mark_workbench_dirty();
                    plane_monitors::show_latest(ui, state);
                }
            }
            Ok(Err(e)) => {
                ui.set_flow_plane_status(format!("Colour range failed: {e}").into());
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => state.post.plane_range_job = Some(rx),
            Err(_) => {
                state.log("Colour range worker disconnected");
            }
        }
    }
    if let Some(rx) = state.post.plane_statistics_job.take() {
        match rx.try_recv() {
            Ok(result) => {
                let message = result.unwrap_or_else(|e| format!("Plane statistics failed: {e}"));
                ui.set_flow_plane_status(message.clone().into());
                state.log(message);
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => state.post.plane_statistics_job = Some(rx),
            Err(_) => {
                state.log("Plane statistics worker disconnected");
            }
        }
    }
    if let Some(rx) = state.post.export.take() {
        match rx.try_recv() {
            Ok(result) => {
                state.post.export_status = result
                    .map(|p| format!("GIF saved: {}", p.display()))
                    .unwrap_or_else(|e| format!("GIF export failed: {e}"));
                state.log(state.post.export_status.clone());
                changed = true;
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => state.post.export = Some(rx),
            Err(_) => {
                state.post.export_status = "GIF export worker disconnected".into();
                changed = true;
            }
        }
    }
    if state.solving {
        if let Some(start) = state.post.started {
            let elapsed = start.elapsed().as_secs_f64();
            if elapsed - state.post.elapsed >= 0.25 {
                state.post.elapsed = elapsed;
                changed = true;
            }
        }
    }
    if state.post.playing && state.current_step == 4 {
        let now = std::time::Instant::now();
        let old = state.post.last_tick.unwrap_or(now);
        let dt = now.duration_since(old).as_secs_f64();
        if dt >= 1.0 / 20.0 {
            state.post.phase = (state.post.phase
                + dt * f64::from(ui.get_post_animation_rate()) / 100.0 / 8.0)
                .rem_euclid(1.0);
            state.post.last_tick = Some(now);
            if ui.get_post_animation_mode() == 1 {
                ui.set_result_slice_percent((1.0 + 98.0 * state.post.phase) as i32);
                changed = true;
            } else {
                if let Some(base) = state.post.base.borrow().as_ref() {
                    ui.set_visualization_image(render::overlay_markers(
                        base.clone(),
                        &markers(&state.post.traces, state.post.phase),
                        &state.mesh_view,
                    ));
                }
                decorate_probe(ui, state);
                ui.set_post_phase((state.post.phase * 1000.0) as i32);
            }
        }
    }
    changed
}

pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    let identity = state
        .post
        .wall_report_time
        .map(|t| format!("Sample at physical time {t:.9e} s\n"))
        .or_else(|| {
            state
                .post
                .wall_report_index
                .map(|i| format!("Sample at iteration {i}\n"))
        })
        .unwrap_or_default();
    ui.set_post_wall_summary(
        (identity + &flursys::workbench::wall_reports::summary(&state.post.wall_reports)).into(),
    );
    ui.set_post_has_wall_report(!state.post.wall_reports.is_empty());
    ui.set_post_has_wall_samples(
        state
            .post
            .wall_reports
            .iter()
            .any(|r| !r.samples.is_empty()),
    );
    if matches!(state.current_step, 3 | 4) {
        plane_monitors::show_latest(ui, state);
    }
    if state.current_step == 3 {
        convergence_plot::refresh(ui, state);
    } else {
        ui.set_continuity_chart_caption(chart_caption(state, ui.get_wb_chart_kind()).into());
    }
    if state.current_step == 4 && ui.get_post_plot_mode() == 2 {
        profile_comparison::show_monitors(ui, state);
    }
    ui.set_post_has_result(state.active_result().is_some());
    ui.set_post_is_3d(
        state
            .active_result()
            .is_some_and(|r| r.mesh().dimension() == MeshDimension::ThreeD),
    );
    ui.set_post_playing(state.post.playing);
    ui.set_post_phase((state.post.phase * 1000.0) as i32);
    ui.set_post_export_busy(state.post.export.is_some());
    ui.set_post_export_status(state.post.export_status.clone().into());
    ui.set_post_probe_count(state.post.probes.len() as i32);
    ui.set_post_balance((result_analysis::balance(state)+"\n\n"+if state.solving{""}else if state.post.boundary_summary.is_empty(){"Patch details were not recorded for this archive. New solves save patch flow integrals."}else{&state.post.boundary_summary}).into());
    ui.set_post_convergence_summary(result_analysis::convergence_summary(state).into());
    if state.post.profile.is_empty() {
        ui.set_post_profile_image(render::render_empty_image());
        ui.set_post_profile_caption(
            "Choose a line and sample the selected field. Profiles use the active archived mesh."
                .into(),
        );
    }
    let probes = state
        .post
        .probes
        .iter()
        .map(|p| {
            SharedString::from(format!(
                "Cell {} · |U|={:.4e} · p={:.4e}",
                p.cell_index, p.velocity_magnitude, p.pressure
            ))
        })
        .collect::<Vec<_>>();
    if ui.get_post_probe_model().iter().collect::<Vec<_>>() != probes {
        ui.set_post_probe_model(ModelRc::new(VecModel::from(probes)));
    }
    let runs = state
        .workbench_project
        .runs
        .iter()
        .filter(|r| r.solution_path.is_some())
        .collect::<Vec<_>>();
    let model = runs
        .iter()
        .map(|r| {
            SharedString::from(format!(
                "{} · {} · {} iterations",
                r.id,
                run_status_label(&r.status),
                r.iterations.unwrap_or(0)
            ))
        })
        .collect::<Vec<_>>();
    if ui.get_post_run_model().iter().collect::<Vec<_>>() != model {
        ui.set_post_run_model(ModelRc::new(VecModel::from(model)));
    }
    ui.set_post_run_index(
        state
            .active_result()
            .and_then(|a| runs.iter().position(|r| r.id == a.run_id()))
            .map_or(-1, |i| i as i32),
    );
    let iteration = if state.solving {
        state.post.iteration
    } else {
        state
            .workbench
            .solution()
            .map_or(state.post.iteration, |s| s.report.outer_iterations)
    };
    ui.set_wb_solve_progress(if state.post.limit > 0 {
        (iteration as f32 / state.post.limit as f32).clamp(0.0, 1.0)
    } else {
        0.0
    });
    ui.set_wb_solve_monitor(
        format!(
            "{} · {} {iteration} / {} · elapsed {:.2} s",
            status_label(&state.workbench),
            if ui.get_wb_time_mode() != 0 {
                "physical step"
            } else {
                "iteration"
            },
            if ui.get_wb_time_mode() != 0 {
                state.post.limit.max(ui.get_wb_time_steps() as usize)
            } else {
                state.post.limit.max(ui.get_wb_max_iterations() as usize)
            },
            state.post.elapsed
        )
        .into(),
    );
    if let Some(data) = state.active_result() {
        ui.set_post_animation_mode(if data.mesh().dimension() == MeshDimension::ThreeD {
            1
        } else {
            0
        });
        let mut f = field(ui.get_result_field_index());
        if data.scalar_values(f).is_empty() {
            ui.set_result_field_index(0);
            f = ResultFieldKind::VelocityMagnitude;
        }
        let values = data.scalar_values(f);
        if let (Some((imin, vmin)), Some((imax, vmax))) = (
            values.iter().enumerate().min_by(|a, b| a.1.total_cmp(b.1)),
            values.iter().enumerate().max_by(|a, b| a.1.total_cmp(b.1)),
        ) {
            let pmin = data.mesh().cells()[imin].center;
            let pmax = data.mesh().cells()[imax].center;
            ui.set_post_statistics(format!("{}\nMinimum {vmin:.6e}\nCell {imin} · ({:.4e}, {:.4e}, {:.4e})\nMaximum {vmax:.6e}\nCell {imax} · ({:.4e}, {:.4e}, {:.4e})\nVolume mean {:.6e}\n{} cells · {} nodes",f.label(),pmin.x,pmin.y,pmin.z,pmax.x,pmax.y,pmax.z,data.volume_mean(f).unwrap_or(f64::NAN),data.mesh().cell_count(),data.mesh().points().len()).into());
        }
        ui.set_post_legend(
            data.scalar_range(f)
                .map(|(a, b)| match state.result_display_range() {
                    Some((lo, hi)) => format!(
                        "{} · display {lo:.5e} … {hi:.5e} · data {a:.5e} … {b:.5e}",
                        f.label()
                    ),
                    None if a == b => format!("{} · constant value {a:.5e}", f.label()),
                    None => format!("{} · min {a:.5e} · max {b:.5e}", f.label()),
                })
                .unwrap_or_default()
                .into(),
        );
    }
}

pub(super) fn decorate(ui: &MainWindow, state: &AppState) {
    refresh(ui, state);
    if state.current_step == 4 {
        *state.post.base.borrow_mut() = Some(ui.get_visualization_image());
    }
    if state.current_step == 4
        && !state.show_mesh
        && !state.show_geometry_3d
        && ui.get_post_animation_mode() == 0
        && !state.post.traces.is_empty()
    {
        ui.set_visualization_image(render::overlay_markers(
            ui.get_visualization_image(),
            &markers(&state.post.traces, state.post.phase),
            &state.mesh_view,
        ));
    }
    decorate_probe(ui, state);
}
fn decorate_probe(ui: &MainWindow, state: &AppState) {
    if state.current_step == 4
        && state
            .active_result()
            .is_some_and(|d| d.mesh().dimension() == MeshDimension::TwoD)
    {
        if let Some(probe) = state.active_result_probe() {
            let point = state
                .mesh_view
                .world_to_screen((probe.center.x, probe.center.y));
            ui.set_visualization_image(render::annotate_measurement(
                ui.get_visualization_image(),
                &[(point.0 as i32, point.1 as i32)],
            ));
        }
    }
}

#[cfg(test)]
mod tests {
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
    fn dataset() -> ResultDataset {
        let points = (0..4)
            .flat_map(|y| (0..4).map(move |x| flursys::Point::new(x as f64, y as f64, 0.0)))
            .collect();
        let cells = (0..3)
            .flat_map(|y| {
                (0..3).map(move |x| {
                    let i = y * 4 + x;
                    flursys::CellDefinition::polygon(vec![i, i + 1, i + 5, i + 4])
                })
            })
            .collect();
        let mesh =
            flursys::UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
        ResultDataset::from_values(
            "run-0001",
            &mesh,
            mesh.cells().iter().map(|c| c.center.x).collect(),
            vec![Vec3::new(1.0, 0.0, 0.0); 9],
        )
        .unwrap()
    }
    fn snapshot(ui: &MainWindow, window: &MinimalSoftwareWindow, w: u32, h: u32, name: &str) {
        window.set_size(slint::PhysicalSize::new(w, h));
        window.request_redraw();
        let mut pixels = vec![slint::Rgb8Pixel::default(); (w * h) as usize];
        window.draw_if_needed(|r| {
            r.render(&mut pixels, w as usize);
        });
        let mut ppm = format!("P6\n{w} {h}\n255\n").into_bytes();
        for p in pixels {
            ppm.extend([p.r, p.g, p.b]);
        }
        std::fs::write(format!("/tmp/flursys-post-{w}-{name}.ppm"), ppm).unwrap();
        assert!(ui.get_post_has_result());
    }
    #[test]
    fn archived_report_preserves_saved_settings_and_exports_statistics() {
        let folder = tempfile::tempdir().unwrap();
        let mut state = AppState::new();
        let record = state
            .workbench_project
            .next_run(flursys::RunStatus::Converged);
        let path = folder.path().join(&record.report_path);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path,r#"{"material":{"density":1234.5},"solver_version":"fixture-version","conservation":{"net_mass_flux":0.0}}"#).unwrap();
        state.workbench_project.runs.push(record);
        state.workspace_path = Some(folder.path().to_path_buf());
        state.install_result_dataset(dataset()).unwrap();
        let html = result_analysis::report(&state).unwrap();
        assert!(html.contains("1234.5"));
        assert!(html.contains("fixture-version"));
        assert!(html.contains("Conservative mass balance"));
        assert!(html.contains("Result mesh quality"));
        assert!(!html.contains("Archived run settings unavailable"));
        let csv = result_analysis::engineering_summary_csv(&state).unwrap();
        assert!(csv.starts_with("quantity,minimum,maximum,volume_weighted_mean\n"));
        assert!(csv.lines().count() > 4);
        std::fs::remove_file(path).unwrap();
        assert!(result_analysis::report(&state)
            .unwrap()
            .contains("Archived run settings unavailable"));
    }

    #[test]
    fn workspace_contours_animation_probes_exports_and_layout() {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(Box::new(SoftwarePlatform(window.clone()))).unwrap();
        let ui = MainWindow::new().unwrap();
        let mut state = AppState::new();
        let mut run = state
            .workbench_project
            .next_run(flursys::RunStatus::Converged);
        run.iterations = Some(20);
        run.continuity_residual = Some(1e-12);
        state.workbench_project.runs.push(run);
        state.install_result_dataset(dataset()).unwrap();
        state.current_step = 4;
        let state = Rc::new(RefCell::new(state));
        bind_callbacks(&ui, &state);
        ui.set_current_step(4);
        ui.set_result_field_index(1);
        ui.set_result_plot_index(0);
        refresh_ui(&ui, &state.borrow());
        let filled = ui.get_visualization_image().to_rgba8().unwrap();
        ui.set_result_plot_index(1);
        ui.set_result_contour_levels(3);
        ui.invoke_apply_result_style();
        let contour = ui.get_visualization_image().to_rgba8().unwrap();
        assert_ne!(
            filled.as_bytes(),
            contour.as_bytes(),
            "Contours must render actual isolines"
        );
        ui.set_result_contour_levels(18);
        ui.invoke_apply_result_style();
        assert_ne!(
            contour.as_bytes(),
            ui.get_visualization_image().to_rgba8().unwrap().as_bytes()
        );
        ui.invoke_post_action(0);
        assert!(!state.borrow().post.traces.is_empty());
        let a = markers(&state.borrow().post.traces, 0.0);
        let b = markers(&state.borrow().post.traces, 0.2);
        assert_ne!(a, b);
        ui.invoke_post_action(2);
        assert!(state.borrow().post.playing);
        state.borrow_mut().post.last_tick =
            Some(std::time::Instant::now() - Duration::from_millis(100));
        tick(&ui, &mut state.borrow_mut());
        assert!(state.borrow().post.phase > 0.0);
        assert!(state.borrow().post.playing);
        ui.invoke_post_seek(500.0);
        assert!(!state.borrow().post.playing);
        assert_eq!(state.borrow().post.phase, 0.5);
        state.borrow_mut().probe_active_result_cell(0).unwrap();
        state.borrow_mut().probe_active_result_cell(1).unwrap();
        state.borrow_mut().probe_active_result_cell(1).unwrap();
        refresh_ui(&ui, &state.borrow());
        assert_eq!(ui.get_post_probe_count(), 2);
        let old = state.borrow().mesh_view;
        ui.invoke_post_view_zoom(-100.0);
        assert_ne!(old, state.borrow().mesh_view);
        ui.invoke_post_view_fit();
        let temp = tempfile::tempdir().unwrap();
        let png = temp.path().join("field.png");
        save_png(&png, &ui.get_visualization_image()).unwrap();
        let decoder =
            png::Decoder::new(std::io::BufReader::new(std::fs::File::open(&png).unwrap()));
        let mut reader = decoder.read_info().unwrap();
        assert_eq!(reader.info().width, PREVIEW_WIDTH);
        let mut output = vec![0; reader.output_buffer_size().unwrap()];
        reader.next_frame(&mut output).unwrap();
        write_probes(&temp.path().join("probes.csv"), &state.borrow().post.probes).unwrap();
        assert_eq!(
            std::fs::read_to_string(temp.path().join("probes.csv"))
                .unwrap()
                .lines()
                .count(),
            3
        );
        let path = temp.path().join("tracers.gif");
        let mut view = ViewTransform::default();
        view.set_viewport(64.0, 64.0);
        view.fit(Some((0.0, 0.0, 3.0, 3.0)));
        save_gif(
            &path,
            64,
            64,
            &vec![0; 64 * 64 * 4],
            &state.borrow().post.traces,
            &view,
        )
        .unwrap();
        let mut options = gif::DecodeOptions::new();
        options.set_color_output(gif::ColorOutput::RGBA);
        let mut gif = options
            .read_info(std::fs::File::open(path).unwrap())
            .unwrap();
        let mut frames = 0;
        let mut changed = false;
        let mut first = Vec::new();
        while let Some(frame) = gif.read_next_frame().unwrap() {
            assert_eq!(frame.delay, 5);
            if frames == 0 {
                first = frame.buffer.to_vec();
            } else if frame.buffer.as_ref() != first.as_slice() {
                changed = true;
            }
            frames += 1;
        }
        assert_eq!(frames, 60);
        assert!(changed, "Exported GIF must contain moving tracers");
        if let Some(path) = std::env::var_os("FLURSYS_VERIFY_WORKSPACE") {
            let workspace = PathBuf::from(path);
            let document = flursys::load_workspace(&workspace).unwrap();
            let id = document
                .runs
                .iter()
                .rev()
                .find(|r| r.solution_path.is_some())
                .unwrap()
                .id
                .clone();
            {
                let mut s = state.borrow_mut();
                s.workspace_path = Some(workspace);
                s.workbench = WorkbenchSession::from_project(&document).unwrap();
                s.workbench_project = document;
                s.activate_persisted_result(&id).unwrap();
                rebuild_tree_rows(&mut s);
            }
            ui.invoke_post_action(0);
        }
        ui.set_result_field_index(0);
        ui.set_result_plot_index(2);
        refresh_ui(&ui, &state.borrow());
        ui.show().unwrap();
        for (w, h) in [(1440, 900), (1100, 650)] {
            for tab in 0..4 {
                ui.set_post_tab(tab);
                snapshot(&ui, &window, w, h, &format!("results-{tab}"));
            }
        }
        if std::env::var_os("FLURSYS_VERIFY_WORKSPACE").is_some() {
            let s = state.borrow();
            let base = s.post.base.borrow();
            let pixels = base.as_ref().unwrap().to_rgba8().unwrap();
            save_gif(
                Path::new("/tmp/flursys-cavity-steady-tracers.gif"),
                pixels.width(),
                pixels.height(),
                pixels.as_bytes(),
                &s.post.traces,
                &s.mesh_view,
            )
            .unwrap();
        }
        // A result's archived topology remains inspectable without regenerating a mesh.
        ui.invoke_post_action(6);
        assert_eq!(ui.get_visualization_title().as_str(), "RESULT MESH");
        ui.invoke_post_action(6);
        ui.invoke_select_result_field(2);
        assert!(ui.get_visualization_title().contains("VORTICITY"));
        ui.invoke_select_result_field(7);
        assert!(ui.get_visualization_title().contains("DIVERGENCE"));
        {
            let mut s = state.borrow_mut();
            s.post.metric_histories = [
                vec![1.0, 0.1, 0.01],
                vec![0.5, 0.05, 0.005],
                vec![1e-5, 1e-7, 1e-9],
            ];
        }
        apply_workflow_step(&ui, &mut state.borrow_mut(), 3);
        for (w, h) in [(1440, 900), (1100, 650)] {
            for kind in 0..4 {
                ui.set_wb_chart_kind(kind);
                ui.invoke_post_refresh();
                snapshot(&ui, &window, w, h, &format!("fixture-solution-{kind}"));
            }
        }
        // Spatial section animation uses a volume mesh and never invents physical time.
        let mesh = flursys::UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                flursys::Point::new(0.0, 0.0, 0.0),
                flursys::Point::new(1.0, 0.0, 0.0),
                flursys::Point::new(0.0, 1.0, 0.0),
                flursys::Point::new(0.0, 0.0, 1.0),
            ],
            vec![flursys::CellDefinition::tetrahedron([0, 1, 2, 3])],
        )
        .unwrap();
        state
            .borrow_mut()
            .install_result_dataset(
                ResultDataset::from_values(
                    "volume",
                    &mesh,
                    vec![0.0],
                    vec![Vec3::new(1.0, 0.0, 0.0)],
                )
                .unwrap(),
            )
            .unwrap();
        apply_workflow_step(&ui, &mut state.borrow_mut(), 4);
        ui.set_post_animation_mode(1);
        ui.invoke_post_action(2);
        assert!(state.borrow().post.playing);
        state.borrow_mut().post.last_tick =
            Some(std::time::Instant::now() - Duration::from_millis(200));
        assert!(tick(&ui, &mut state.borrow_mut()));
        assert!(ui.get_result_slice_percent() > 1);
        ui.invoke_post_action(3);
        assert!(!state.borrow().post.playing);
    }
}
