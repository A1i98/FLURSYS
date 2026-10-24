//! Headless orchestration over the same canonical workspaces and jobs as the GUI.
//!
//! This module does not parse command-line arguments, print progress, or depend on
//! Slint. Library callers can edit a `WorkbenchSession` and observe typed events.
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
    /// Zero disables periodic writes; retained final state is still saved.
    pub checkpoint_every: usize,
}
impl Default for RunOptions {
    fn default() -> Self {
        Self {
            restart: None,
            checkpoint_every: 100,
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
pub struct ExecutionControl(std::sync::Arc<std::sync::atomic::AtomicU8>);
impl ExecutionControl {
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
    if let Some(path) = &options.restart {
        crate::workbench::checkpoint::load(path, &case.mesh)?;
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
    let mut outcome = None;
    let mut health = None;
    while outcome.is_none() {
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
                WorkbenchJobEvent::Health(report) => health = Some(report),
                WorkbenchJobEvent::Performance(report) => session.set_performance(report),
                WorkbenchJobEvent::SolveCompleted(result) => outcome = Some(result),
                _ => {}
            }
        }
        if outcome.is_none() && jobs.lifecycle() != JobLifecycle::Running {
            outcome = Some(Err(crate::IncompressibleSolveError::Simple(
                crate::SimpleError::Monitor(
                    "Solve worker disconnected before returning a result".into(),
                ),
            )));
        }
        if outcome.is_none() {
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
    }
    let result = outcome.ok_or("Solve worker returned no outcome")?;
    let failure = result.as_ref().err().map(ToString::to_string);
    let retained_checkpoint = if let Some((index, fields)) = jobs.retained_fields(mesh.id()) {
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
