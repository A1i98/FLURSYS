//! Headless orchestration over the same canonical workspaces and jobs as the GUI.
//!
//! This module does not parse command-line arguments, print progress, or depend on
//! a GUI toolkit. Library callers can edit a `WorkbenchSession` and observe typed events.
use crate::{
    app::{JobLifecycle, WorkbenchJobEvent, WorkbenchJobExecutor},
    workbench::{finalize_workspace_run, mesh_storage, save_workspace, solve_review},
    GmshMesher, RunRecord, WorkbenchProject, WorkbenchSession,
};
use serde::Serialize;
/// Version of the headless orchestration contract (independent of file formats).
pub const API_VERSION: u32 = 1;
use std::path::{Path, PathBuf};

pub fn load(directory: &Path) -> Result<(WorkbenchProject, WorkbenchSession), String> {
    let project = crate::load_workspace(directory).map_err(|e| e.to_string())?;
    let mut session = WorkbenchSession::from_project(&project).map_err(|e| e.to_string())?;
    mesh_storage::restore(directory, &project, &mut session)?;
    Ok((project, session))
}

fn sync_intent(project: &mut WorkbenchProject, session: &WorkbenchSession) {
    let mut intent = session.to_project(project.name.clone());
    intent.project_id = project.project_id.clone();
    intent.runs = project.runs.clone();
    intent.cad_document = project.cad_document.clone();
    intent.cad_geometry_hash = project.cad_geometry_hash.clone();
    intent.plane_monitors = project.plane_monitors.clone();
    intent.mesh_asset = project.mesh_asset.clone();
    *project = intent;
}

/// Generate and persist a mesh, preserving canonical project identity and history.
pub fn mesh(
    directory: &Path,
    project: &mut WorkbenchProject,
    session: &mut WorkbenchSession,
    mesher: &GmshMesher,
) -> Result<(), String> {
    let (export, options) = session
        .mesh_generation_inputs()
        .map_err(|e| e.to_string())?;
    let generated = mesher
        .generate(&export.document, &options)
        .map_err(|e| e.to_string())?;
    session.install_mesh(generated);
    project.mesh_asset = Some(mesh_storage::save(
        directory,
        &session.to_project(project.name.clone()),
        session.mesh().ok_or("Mesh installation failed")?,
    )?);
    sync_intent(project, session);
    save_workspace(directory, project).map_err(|e| e.to_string())
}

#[derive(Clone, Debug)]
pub struct RunOptions {
    pub restart: Option<PathBuf>,
    /// Full-field playback cadence; zero disables playback snapshots.
    pub checkpoint_every: usize,
    pub recovery: crate::workbench::checkpoint::RecoveryPolicy,
}
impl Default for RunOptions {
    fn default() -> Self {
        Self {
            restart: None,
            checkpoint_every: 100,
            recovery: Default::default(),
        }
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct RunOutcome {
    pub run: Option<RunRecord>,
    pub failure: Option<String>,
    pub restart_checkpoint: Option<PathBuf>,
}

/// Thread-safe cooperative control shared with a running headless job.
/// Stop is terminal; resume cannot revoke a stop request.
#[derive(Clone, Default)]
pub struct ExecutionControl(
    std::sync::Arc<std::sync::atomic::AtomicU8>,
    std::sync::Arc<std::sync::atomic::AtomicBool>,
);
impl ExecutionControl {
    pub fn checkpoint(&self) {
        self.1.store(true, std::sync::atomic::Ordering::Release);
    }
    pub fn pause(&self) {
        let _ = self.0.compare_exchange(
            0,
            1,
            std::sync::atomic::Ordering::AcqRel,
            std::sync::atomic::Ordering::Acquire,
        );
    }
    pub fn resume(&self) {
        let _ = self.0.compare_exchange(
            1,
            0,
            std::sync::atomic::Ordering::AcqRel,
            std::sync::atomic::Ordering::Acquire,
        );
    }
    pub fn stop(&self) {
        self.0.store(2, std::sync::atomic::Ordering::Release);
    }
    pub fn is_stopped(&self) -> bool {
        self.0.load(std::sync::atomic::Ordering::Acquire) == 2
    }
}

enum FlowOutcome {
    Incompressible(Result<crate::IncompressibleSolution, crate::IncompressibleSolveError>),
    Compressible(Result<crate::compressible::Solution, String>),
}
fn drive(
    jobs: &mut WorkbenchJobExecutor,
    control: &ExecutionControl,
    mut observe: impl FnMut(&WorkbenchJobEvent),
) -> Result<FlowOutcome, String> {
    loop {
        if control.1.swap(false, std::sync::atomic::Ordering::AcqRel) {
            if let Err(e) = jobs.checkpoint_now() {
                observe(&WorkbenchJobEvent::Checkpoint(Err(e)));
            }
        }
        match control.0.load(std::sync::atomic::Ordering::Acquire) {
            2 => {
                jobs.cancel();
            }
            1 => jobs.set_paused(true),
            _ => jobs.set_paused(false),
        }
        for event in jobs.drain() {
            observe(&event);
            match event {
                WorkbenchJobEvent::SolveCompleted(result) => {
                    return Ok(FlowOutcome::Incompressible(result))
                }
                WorkbenchJobEvent::CompressibleCompleted(result) => {
                    return Ok(FlowOutcome::Compressible(result))
                }
                _ => {}
            }
        }
        if jobs.lifecycle() != JobLifecycle::Running {
            return Err("Solve worker disconnected before returning accepted state".into());
        }
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}
pub fn execute(
    case: crate::flow::FlowCase,
    folder: &Path,
    control: &ExecutionControl,
    observe: impl FnMut(&WorkbenchJobEvent),
) -> Result<crate::flow::FlowSolution, String> {
    let mut jobs = WorkbenchJobExecutor::new();
    jobs.start_flow(case, Vec::new(), folder.into(), "standalone".into())?;
    match drive(&mut jobs, control, observe)? {
        FlowOutcome::Incompressible(r) => r
            .map(crate::flow::FlowSolution::Incompressible)
            .map_err(|e| e.to_string()),
        FlowOutcome::Compressible(r) => r.map(crate::flow::FlowSolution::Compressible),
    }
}
fn run_compressible(
    directory: &Path,
    project: &mut WorkbenchProject,
    session: &mut WorkbenchSession,
    options: &RunOptions,
    control: &ExecutionControl,
    mut observe: impl FnMut(&WorkbenchJobEvent),
) -> Result<RunOutcome, String> {
    let crate::flow::FlowCase::Compressible { mesh, settings } =
        session.prepare_flow_case().map_err(|e| e.to_string())?
    else {
        return Err("Missing compressible configuration".into());
    };
    let mut jobs = WorkbenchJobExecutor::new();
    jobs.set_checkpoint_interval(options.checkpoint_every);
    jobs.set_recovery_policy(options.recovery.clone())?;
    let mut continuation = None;
    let mut folder = None;
    let mut histories: [Vec<(f64, Option<f64>)>; 4] = Default::default();
    if let Some(path) = &options.restart {
        jobs.set_restart(Some(path.clone()));
    } else if let Some(run) = project.runs.last() {
        if let Some(path) = &run.solution_path {
            let path = directory.join(path);
            if crate::workbench::flow_frame::is_frame(&path) {
                let frame = crate::workbench::flow_frame::FlowFrame::read(&path)?;
                let state = frame.gas_restart(&mesh, settings.gas)?;
                jobs.set_compressible_state(&mesh, state)?;
                let report = report(directory, run)?;
                histories = report
                    .get("indexed_convergence_history")
                    .and_then(|v| serde_json::from_value(v.clone()).ok())
                    .unwrap_or_default();
                folder = report
                    .get("plane_recording_folder")
                    .and_then(|v| v.as_str())
                    .map(|p| {
                        let path = Path::new(p);
                        if path.is_absolute()
                            || path
                                .components()
                                .any(|c| !matches!(c, std::path::Component::Normal(_)))
                        {
                            Err(String::from(
                                "Recording folder must stay inside the workspace",
                            ))
                        } else {
                            Ok(directory.join(path))
                        }
                    })
                    .transpose()?;
                continuation = Some(run.id.clone());
            }
        }
    }
    let folder = folder.unwrap_or_else(|| {
        directory.join("monitors").join(format!(
            "solve-{}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ))
    });
    let run_id = continuation.clone().unwrap_or_else(|| {
        project
            .next_run(crate::workbench::RunStatus::MaxIterations)
            .id
    });
    save_workspace(directory, project).map_err(|e| e.to_string())?;
    jobs.start_compressible(mesh.clone(), settings.clone(), folder.clone(), run_id)?;
    let outcome = drive(&mut jobs, control, |event| {
        observe(event);
        if let WorkbenchJobEvent::SolveProgress(crate::app::JobProgress::Metrics {
            iteration,
            values,
        }) = event
        {
            for (series, value) in histories.iter_mut().zip(values) {
                let x = *iteration as f64;
                match series.binary_search_by(|p| p.0.total_cmp(&x)) {
                    Ok(i) => series[i].1 = Some(*value),
                    Err(i) => series.insert(i, (x, Some(*value))),
                }
            }
        }
    });
    let result = match outcome {
        Ok(FlowOutcome::Compressible(result)) => result,
        Ok(_) => return Err("Unexpected flow model result".into()),
        Err(e) => Err(e),
    };
    let failure = result.as_ref().err().cloned();
    let state = result
        .as_ref()
        .ok()
        .map(|s| std::sync::Arc::new(s.snapshot(settings.gas)))
        .or_else(|| jobs.retained_compressible(mesh.id()));
    let run = state
        .as_ref()
        .map(|state| {
            crate::workbench::flow_frame::finalize_compressible(
                directory,
                project,
                &mesh,
                state,
                &folder,
                continuation.as_deref(),
                &histories,
                failure.clone(),
            )
        })
        .transpose()?;
    session.complete_compressible(result);
    Ok(RunOutcome {
        run,
        failure,
        restart_checkpoint: folder
            .join("checkpoint.json")
            .exists()
            .then(|| folder.join("checkpoint.json")),
    })
}

/// Run synchronously while reporting typed worker events to the caller.
/// A numerical failure is returned in `RunOutcome` after its report is saved;
/// setup, storage and orchestration errors return `Err`.
pub fn run(
    directory: &Path,
    project: &mut WorkbenchProject,
    session: &mut WorkbenchSession,
    options: &RunOptions,
    observe: impl FnMut(&WorkbenchJobEvent),
) -> Result<RunOutcome, String> {
    run_with_control(
        directory,
        project,
        session,
        options,
        &ExecutionControl::default(),
        observe,
    )
}

/// Same execution path as `run`, with cooperative pause/resume/stop from another thread.
pub fn run_with_control(
    directory: &Path,
    project: &mut WorkbenchProject,
    session: &mut WorkbenchSession,
    options: &RunOptions,
    control: &ExecutionControl,
    mut observe: impl FnMut(&WorkbenchJobEvent),
) -> Result<RunOutcome, String> {
    options.recovery.validate()?;
    if options.checkpoint_every > 1_000_000 {
        return Err("Checkpoint interval must be in 0..1000000".into());
    }
    if project.solver.compressible.is_some() {
        return run_compressible(directory, project, session, options, control, observe);
    }
    session
        .readiness()
        .map_err(|e| format!("Project is not ready: {e}"))?;
    let case = session.prepare_case().map_err(|e| e.to_string())?;
    let review = solve_review::review(&case, &project.plane_monitors);
    if review.blocked() {
        return Err(review.summary());
    }
    if options.checkpoint_every > 1_000_000 {
        return Err("Checkpoint interval must be in 0..1000000".into());
    }
    let mut jobs = WorkbenchJobExecutor::new();
    jobs.set_checkpoint_interval(options.checkpoint_every);
    jobs.set_recovery_policy(options.recovery.clone())?;
    if let Some(path) = &options.restart {
        crate::workbench::checkpoint::load_recoverable(path, &case.mesh)?;
        jobs.set_restart(Some(path.clone()));
    }
    sync_intent(project, session);
    save_workspace(directory, project).map_err(|e| e.to_string())?;
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let recording = PathBuf::from("monitors").join(format!("solve-{stamp}"));
    let physical = case.solver.time.is_transient();
    let mesh = case.mesh.clone();
    let checkpoint_context = format!(
        "material={:?}; solver={:?}; boundaries={:?}",
        case.material, case.solver, case.boundaries
    );
    jobs.start_solve_with_planes(
        case,
        project.plane_monitors.clone(),
        directory.join(&recording),
    )
    .map_err(str::to_string)?;
    session.mark_solving();
    let mut health = None;
    let outcome = drive(&mut jobs, control, |event| {
        observe(event);
        match event {
            WorkbenchJobEvent::Health(report) => health = Some(report.clone()),
            WorkbenchJobEvent::Performance(report) => session.set_performance(report.clone()),
            _ => {}
        }
    });
    let result = match outcome {
        Ok(FlowOutcome::Incompressible(result)) => result,
        Ok(_) => return Err("Unexpected flow model result".into()),
        Err(e) => Err(crate::IncompressibleSolveError::Simple(
            crate::SimpleError::Monitor(e),
        )),
    };
    let failure = result.as_ref().err().map(ToString::to_string);
    let retained_checkpoint = if let Some((index, fields)) =
        jobs.retained_fields(mesh.id()).filter(|_| {
            result.is_ok() || matches!(&result, Err(crate::IncompressibleSolveError::Cancelled))
        }) {
        let relative = recording.join("retained-checkpoint.json");
        crate::workbench::checkpoint::write_restart(
            &directory.join(&relative),
            &crate::workbench::checkpoint::MeshSignature::new(&mesh),
            index,
            &fields,
            physical,
            &checkpoint_context,
        )?;
        Some(relative)
    } else {
        None
    };
    session.complete_solve(result);
    let record = finalize_workspace_run(directory, project, session).map_err(|e| e.to_string())?;
    if let Some(record) = &record {
        let path = directory.join(&record.report_path);
        let mut report: serde_json::Value =
            serde_json::from_slice(&std::fs::read(&path).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        report["plane_recording_folder"] = serde_json::json!(recording);
        report["sampled_field_health"] = serde_json::json!(health);
        if let Some(restart) = &options.restart {
            report["restart_source"] = serde_json::json!(restart);
        }
        let diagnostic = recording.join("failed-last-sample.json");
        if failure.is_some() && directory.join(&diagnostic).is_file() {
            report["unconverged_checkpoint"] = serde_json::json!(diagnostic);
        }
        use std::io::Write;
        let mut temp =
            tempfile::NamedTempFile::new_in(path.parent().ok_or("Report has no parent directory")?)
                .map_err(|e| e.to_string())?;
        serde_json::to_writer_pretty(&mut temp, &report).map_err(|e| e.to_string())?;
        temp.flush().map_err(|e| e.to_string())?;
        temp.as_file().sync_all().map_err(|e| e.to_string())?;
        temp.persist(&path).map_err(|e| e.to_string())?;
    }
    Ok(RunOutcome {
        run: record,
        failure,
        restart_checkpoint: retained_checkpoint,
    })
}

/// Load a run report with workspace-relative path validation.
pub fn report(directory: &Path, run: &RunRecord) -> Result<serde_json::Value, String> {
    let path = &run.report_path;
    if path.is_absolute()
        || path
            .components()
            .any(|c| !matches!(c, std::path::Component::Normal(_)))
    {
        return Err("Run report path must stay inside the workspace".into());
    }
    serde_json::from_slice(&std::fs::read(directory.join(path)).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())
}
/// Recompute load histories for a completed or interrupted run using any references.
pub fn force_history(
    directory: &Path,
    run: &RunRecord,
) -> Result<Vec<crate::workbench::force_analysis::Sample>, String> {
    let report = report(directory, run)?;
    let folder = report
        .get("plane_recording_folder")
        .and_then(|v| v.as_str())
        .ok_or("Run has no recorded load history")?;
    let path = Path::new(folder);
    if path.is_absolute()
        || path
            .components()
            .any(|c| !matches!(c, std::path::Component::Normal(_)))
    {
        return Err("Recording folder must stay inside the workspace".into());
    }
    crate::workbench::force_analysis::load(&directory.join(path).join("force-history.jsonl"))
}
