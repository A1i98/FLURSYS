use crate::{
    solve_incompressible_with_control, GeneratedMesh, GeometryGmshExport, GmshMeshOptions,
    GmshMesher, IncompressibleCase, IncompressibleSolution, IncompressibleSolveError, MeshingError,
    SolveControl,
};
use std::sync::{
    atomic::{AtomicBool, AtomicUsize, Ordering},
    mpsc::{self, Receiver},
    Arc, Mutex,
};

fn wait_for_resume(paused: &AtomicBool, cancellation: &CancellationToken) -> bool {
    while paused.load(Ordering::Acquire) && !cancellation.is_cancelled() {
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    cancellation.is_cancelled()
}

/// Thread-safe cooperative cancellation shared by an application job and its worker.
#[derive(Clone, Debug, Default)]
pub struct CancellationToken(Arc<AtomicBool>);

impl CancellationToken {
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Release);
    }

    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Acquire)
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum JobProgress {
    Meshing {
        fraction: f64,
        stage: String,
    },
    Metrics {
        iteration: usize,
        values: [f64; 4],
    },
    Started,
    StartedAt {
        iteration: usize,
    },
    Message(String),
    Iteration {
        current: usize,
        max: usize,
        residual: Option<f64>,
    },
    Completed,
    Cancelled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JobLifecycle {
    Idle,
    Running,
    Completed,
    Failed,
    Cancelled,
}

#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // Preserve the public value-based API.
pub enum WorkbenchJobEvent {
    MeshProgress(JobProgress),
    MeshCompleted(Result<GeneratedMesh, MeshingError>),
    SolveProgress(JobProgress),
    SolveCompleted(Result<IncompressibleSolution, IncompressibleSolveError>),
    PlaneSample(crate::workbench::plane_monitor::PlaneSnapshot),
    Health(crate::workbench::solve_review::SolveHealth),
    WallReports {
        iteration: usize,
        physical_time: Option<f64>,
        reports: Vec<crate::workbench::wall_reports::WallPatchReport>,
    },
}

/// Owns worker channels and lifecycle state for the canonical workbench jobs.
/// Only one mesh or solve job may run at a time because both mutate the same
/// session lifecycle once their completion event is applied.
pub struct WorkbenchJobExecutor {
    receiver: Option<Receiver<WorkbenchJobEvent>>,
    cancellation: Option<CancellationToken>,
    lifecycle: JobLifecycle,
    paused: Arc<AtomicBool>,
    restart: Option<std::path::PathBuf>,
    checkpoint_every: usize,
    completed: Arc<AtomicUsize>,
    retained: Arc<Mutex<Option<(crate::MeshId, crate::IncompressibleInitialConditions)>>>,
}

impl Drop for WorkbenchJobExecutor {
    fn drop(&mut self) {
        if let Some(token) = &self.cancellation {
            token.cancel();
        }
    }
}

impl Default for WorkbenchJobExecutor {
    fn default() -> Self {
        Self::new()
    }
}

impl WorkbenchJobExecutor {
    pub fn new() -> Self {
        Self {
            receiver: None,
            cancellation: None,
            lifecycle: JobLifecycle::Idle,
            paused: Arc::new(AtomicBool::new(false)),
            restart: None,
            checkpoint_every: 100,
            completed: Arc::new(AtomicUsize::new(0)),
            retained: Arc::new(Mutex::new(None)),
        }
    }

    pub fn retained_fields(&self, mesh: crate::MeshId) -> Option<(usize, crate::IncompressibleInitialConditions)> {
        self.retained.lock().ok()?.as_ref().filter(|(id,_)|*id==mesh)
            .map(|(_,fields)|(self.completed_iterations(),fields.clone()))
    }
    pub fn completed_iterations(&self) -> usize {
        self.completed.load(Ordering::Acquire)
    }
    pub fn set_checkpoint_interval(&mut self, every: usize) {
        self.checkpoint_every = every.min(1_000_000);
    }
    pub fn set_restart(&mut self, path: Option<std::path::PathBuf>) {
        self.completed.store(0, Ordering::Release);
        self.restart = path;
        *self.retained.lock().unwrap() = None;
    }
    pub fn set_initial_fields(
        &mut self,
        mesh: crate::MeshId,
        fields: crate::IncompressibleInitialConditions,
    ) {
        self.completed.store(0, Ordering::Release);
        self.restart = None;
        *self.retained.lock().unwrap() = Some((mesh, fields));
    }
    pub fn set_continuation_fields(
        &mut self,
        mesh: crate::MeshId,
        fields: crate::IncompressibleInitialConditions,
        completed: usize,
    ) {
        // An explicitly selected checkpoint takes precedence over the visible result.
        if self.restart.is_some() {
            return;
        }
        self.set_initial_fields(mesh, fields);
        self.completed.store(completed, Ordering::Release);
    }
    pub fn set_paused(&self, paused: bool) {
        self.paused.store(paused, Ordering::Release);
    }
    pub fn is_paused(&self) -> bool {
        self.paused.load(Ordering::Acquire)
    }
    pub fn lifecycle(&self) -> JobLifecycle {
        self.lifecycle
    }

    pub fn is_running(&self) -> bool {
        self.lifecycle == JobLifecycle::Running
    }

    pub fn start_mesh(
        &mut self,
        export: GeometryGmshExport,
        options: GmshMeshOptions,
    ) -> Result<(), &'static str> {
        self.start(|sender, cancellation| {
            std::thread::spawn(move || {
                let _ = sender.send(WorkbenchJobEvent::MeshProgress(JobProgress::Started));
                let result = GmshMesher::auto().generate_with_progress(
                    &export.document,
                    &options,
                    || cancellation.is_cancelled(),
                    |fraction, stage| {
                        let _ =
                            sender.send(WorkbenchJobEvent::MeshProgress(JobProgress::Meshing {
                                fraction,
                                stage: stage.into(),
                            }));
                    },
                );
                let cancelled = cancellation.is_cancelled();
                let _ = sender.send(WorkbenchJobEvent::MeshCompleted(result));
                let _ = sender.send(WorkbenchJobEvent::MeshProgress(if cancelled {
                    JobProgress::Cancelled
                } else {
                    JobProgress::Completed
                }));
            });
        })
    }

    pub fn start_solve(&mut self, case: IncompressibleCase) -> Result<(), &'static str> {
        self.start_solve_with_planes(case, Vec::new(), std::path::PathBuf::new())
    }
    pub fn start_solve_with_planes(
        &mut self,
        mut case: IncompressibleCase,
        planes: Vec<crate::workbench::plane_monitor::FlowPlaneMonitor>,
        folder: std::path::PathBuf,
    ) -> Result<(), &'static str> {
        if self.is_running() {
            return Err("a workbench job is already running");
        }
        {
            let mut retained=self.retained.lock().unwrap();
            if let Some((mesh, fields))=retained.as_ref() {
                if *mesh==case.mesh.id() {case.initial_conditions=fields.clone();}
                else {
                    self.completed.store(0,Ordering::Release);
                    *retained=None;
                }
            }
        }
        if let Some(path) = self.restart.as_ref() {
            let (iteration, fields) = crate::workbench::checkpoint::load(path, &case.mesh)
                .map_err(|_| {
                    "checkpoint cannot be loaded for this mesh; choose it again or Initialize"
                })?;
            self.completed.store(iteration, Ordering::Release);
            case.initial_conditions = fields;
            self.restart = None;
        }
        let offset = self.completed_iterations();
        let completed_counter = self.completed.clone();
        let physical = case.solver.time.is_transient();
        let checkpoint_every = if folder.as_os_str().is_empty() {
            0
        } else {
            self.checkpoint_every
        };
        let paused = self.paused.clone();
        let retained = self.retained.clone();
        let retained_mesh = case.mesh.id();
        self.start(|sender, cancellation| {
            std::thread::spawn(move || {
                let case=Arc::new(case);
                let sample_time=Arc::new(Mutex::new(None));
                let completed_time=sample_time.clone();
                let monitor_case=case.clone();let monitor_sender=sender.clone();
                let message_sender=sender.clone();
                let _ = sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::StartedAt { iteration: offset }));
                let progress_sender = sender.clone();
                let control_cancellation = cancellation.clone();
                let metrics_sender = sender.clone();
                let control = SolveControl::new(
                    move || wait_for_resume(&paused, &control_cancellation),
                    move |iteration, max, residual| {
                        let _ = progress_sender.send(WorkbenchJobEvent::SolveProgress(
                            JobProgress::Iteration {
                                current: offset.saturating_add(iteration),
                                max: offset.saturating_add(max),
                                residual: Some(residual),
                            },
                        ));
                    },
                )
                .with_metrics(move |iteration, values| {
                    let _ = metrics_sender.send(WorkbenchJobEvent::SolveProgress(
                        JobProgress::Metrics { iteration: offset.saturating_add(iteration), values },
                    ));
                });
                let volume_monitors = case.mesh.dimension() == crate::MeshDimension::ThreeD;
                if !volume_monitors && planes.iter().any(|p| p.enabled) {
                    let _ = sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(
                        "3D flow-plane recording skipped for this 2D mesh. Use the full-domain results display; saved plane settings are retained for 3D cases.".into()
                    )));
                }
                let plans = planes
                    .into_iter()
                    .filter(|p| p.enabled && volume_monitors)
                    .map(|p| {
                        let cells = crate::workbench::plane_monitor::intersect(&case.mesh, &p)?;
                        if cells.is_empty() {
                            return Err(format!(
                                "Monitor plane '{}' does not intersect the fluid volume",
                                p.name
                            ));
                        }
                        Ok((p, cells))
                    })
                    .collect::<Result<Vec<_>, String>>();
                let plans = match plans {
                    Ok(p) => p,
                    Err(e) => {
                        let _ = sender.send(WorkbenchJobEvent::SolveCompleted(Err(
                            IncompressibleSolveError::Simple(crate::simple::SimpleError::Monitor(
                                e,
                            )),
                        )));
                        return;
                    }
                };
                let plane_sender = sender.clone();
                let plans = Arc::new(plans);
                let observer_plans = plans.clone();
                let recording_folder = folder.clone();
                let checkpoint_mesh = (!folder.as_os_str().is_empty()).then(|| {
                    Arc::new(crate::workbench::checkpoint::MeshSignature::new(&case.mesh))
                });
                let checkpoint_path = folder.join("checkpoint.json");
                let checkpoint_context = format!(
                    "material={:?}; solver={:?}; boundaries={:?}",
                    case.material, case.solver, case.boundaries
                );
                let final_checkpoint_mesh = checkpoint_mesh.clone();
                let final_checkpoint_path = checkpoint_path.clone();
                let final_checkpoint_context = checkpoint_context.clone();
                let iteration_retained=retained.clone();
                let iteration_mesh=final_checkpoint_mesh.clone();let iteration_path=final_checkpoint_path.clone();let iteration_context=final_checkpoint_context.clone();let iteration_sender=sender.clone();
                let completed_retained=retained.clone();
                let step_counter=completed_counter.clone();
                let final_counter=completed_counter.clone();
                let completed_mesh=final_checkpoint_mesh.clone();
                let completed_path=final_checkpoint_path.clone();
                let completed_context=final_checkpoint_context.clone();
                let completed_sender=sender.clone();
                let completed_case=case.clone();

                let control = control.with_message(move |message|{let _=message_sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(message)));});
                let control = control.with_fields(move |iteration, velocity, pressure| {
                    let iteration = offset.saturating_add(iteration);
                    if iteration==1 || iteration%10==0 {
                        if let Ok(walls)=crate::workbench::wall_reports::reports(&monitor_case,velocity,pressure,false) {let _=monitor_sender.send(WorkbenchJobEvent::WallReports{iteration,physical_time:*sample_time.lock().unwrap(),reports:walls});}
                    }
                    for (plane, cells) in observer_plans.iter() {
                        if iteration % plane.every != 0 {
                            continue;
                        }
                        let mut snapshot = crate::workbench::plane_monitor::sample(
                            plane,
                            cells,
                            iteration,
                            velocity.values(),
                            pressure.values(),
                        )?;
                        snapshot.physical_time = *sample_time.lock().unwrap();
                        snapshot.path = Some(crate::workbench::plane_monitor::write_csv(
                            &snapshot,
                            &recording_folder,
                        )?);
                        let _ = plane_sender.send(WorkbenchJobEvent::PlaneSample(snapshot));
                    }
                    Ok(())
                });
                let health_case=case.clone();
                // Retention cadence divides the configured disk cadence, including odd intervals.
                let retention_every=if checkpoint_every==0 {10} else {(1..=10).rev().find(|n|checkpoint_every%n==0).unwrap_or(1)};
                let control=control.with_restart_state(retention_every,move |index,initial| {
                    let local_index=index;
                    let index = offset.saturating_add(index);
                    completed_counter.store(index, Ordering::Release);
                    if let Some(health)=crate::workbench::solve_review::health(&health_case,&initial,index) {
                        let _=iteration_sender.send(WorkbenchJobEvent::Health(health));
                    }
                    if let Some(mesh)=iteration_mesh.as_ref().filter(|_|checkpoint_every>0 && (local_index==1||local_index%checkpoint_every==0)) {
                        if let Err(error)=crate::workbench::checkpoint::write_restart(&iteration_path,mesh,index,&initial,false,&iteration_context) {
                            let _=iteration_sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Restart checkpoint failed: {error}"))));
                        }
                    }
                    *iteration_retained.lock().unwrap()=Some((retained_mesh,initial));Ok(())
                });
                let control=control.with_completed_step(move |index,solution| {
                    let local_index=index;
                    let index = offset.saturating_add(index);
                    step_counter.store(index, Ordering::Release);
                    if matches!(completed_case.solver.turbulence.wall_treatment,crate::turbulence::WallTreatment::EquilibriumKEpsilon|crate::turbulence::WallTreatment::EquilibriumKOmega) {
                        match crate::workbench::wall_reports::reports_with_turbulence(&completed_case,&solution.velocity,&solution.pressure,solution.turbulence.as_ref(),false){
                            Ok(reports)=>{
                                if solution.physical_time.is_none()||index==1||index%10==0 {for r in &reports {if r.y_plus_min.is_some_and(|v|v<30.0)||r.y_plus_max.is_some_and(|v|v>300.0){let _=completed_sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Wall-function mesh review: patch {}, shear-based y+ min={:?}, max={:?}. Equilibrium log-layer accuracy is not established outside the target range; inspect local resolution and separation.",r.patch,r.y_plus_min,r.y_plus_max))));}}}
                                let _=completed_sender.send(WorkbenchJobEvent::WallReports{iteration:index,physical_time:solution.physical_time,reports});
                            },Err(e)=>{let _=completed_sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Wall-function report failed: {e}"))));}
                        }
                    }

                    *completed_time.lock().unwrap()=solution.physical_time;
                    *completed_retained.lock().unwrap()=Some((retained_mesh,solution.restart()));
                    if let Some(mesh)=completed_mesh.as_ref().filter(|_|checkpoint_every>0 && (local_index==1||local_index%checkpoint_every==0)) {
                        if let Err(error)=crate::workbench::checkpoint::write_solution(&completed_path,mesh,index,solution,&completed_context) {
                            let _=completed_sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Complete-state checkpoint failed: {error}"))));
                        }
                    }
                    if let Some(time)=solution.physical_time {let _=completed_sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Completed physical time step {index}: t={time:.9e} s, dt={:.6e} s, nonlinear correctors={}, max Co={:.4e}, rejected steps={}",solution.last_time_step.unwrap_or(0.0),solution.report.outer_iterations,solution.max_courant.unwrap_or(0.0),solution.rejected_time_steps))));}
                    Ok(())
                });
                let mut result = solve_incompressible_with_control(&case, &control);
                if let Ok(solution) = &mut result {
                    if physical { solution.completed_time_steps = offset.saturating_add(solution.completed_time_steps); }
                    else { solution.report.outer_iterations = offset.saturating_add(solution.report.outer_iterations); }
                }
                if let Ok(solution) = &result {
                    *retained.lock().unwrap()=Some((retained_mesh,solution.restart()));
                    final_counter.store(if physical { solution.completed_time_steps } else { solution.report.outer_iterations }, Ordering::Release);
                    if let Some(final_checkpoint_mesh) = final_checkpoint_mesh.as_ref().filter(|_|checkpoint_every>0) {
                        if let Err(error) = crate::workbench::checkpoint::write_solution(
                            &final_checkpoint_path, final_checkpoint_mesh,
                            if physical {solution.completed_time_steps}else{solution.report.outer_iterations},
                            solution,&final_checkpoint_context,
                        ) {
                            let _ = sender.send(WorkbenchJobEvent::SolveProgress(
                                JobProgress::Message(format!("Final checkpoint failed: {error}")),
                            ));
                        }
                    }
                    let iteration = if physical {solution.completed_time_steps}else{solution.report.outer_iterations};
                    for (plane, cells) in plans.iter().filter(|(p, _)| iteration % p.every != 0) {
                        let saved = crate::workbench::plane_monitor::sample(
                            plane,
                            cells,
                            iteration,
                            solution.velocity.values(),
                            solution.pressure.values(),
                        )
                        .and_then(|mut snapshot| {
                            snapshot.physical_time=solution.physical_time;
                            snapshot.path = Some(crate::workbench::plane_monitor::write_csv(
                                &snapshot, &folder,
                            )?);
                            Ok(snapshot)
                        });
                        match saved {
                            Ok(snapshot) => {
                                let _ = sender.send(WorkbenchJobEvent::PlaneSample(snapshot));
                            }
                            Err(error) => {
                                let _ = sender.send(WorkbenchJobEvent::SolveProgress(
                                    JobProgress::Message(format!(
                                        "Final plane snapshot failed: {error}"
                                    )),
                                ));
                            }
                        }
                    }
                }
                {
                let retained_guard=retained.lock().unwrap();
                if let Some((_,initial))=retained_guard.as_ref() {
                    let index=final_counter.load(Ordering::Acquire);
                    if let Some(health)=crate::workbench::solve_review::health(&case,&initial,index) {
                        if !folder.as_os_str().is_empty() {
                            let path=folder.join("field-health.csv");
                            let saved=(|| -> std::io::Result<()> {
                                std::fs::create_dir_all(&folder)?;
                                let temporary=path.with_extension("csv.tmp");
                                std::fs::write(&temporary,health.csv())?;
                                std::fs::rename(temporary,&path)
                            })();
                            if let Err(e)=saved {let _=sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Field-health CSV failed: {e}"))));}
                        }
                        let _=sender.send(WorkbenchJobEvent::Health(health));
                    }
                }
                if result.is_err() {
                    if let Some((_,initial))=retained_guard.as_ref() {
                        if let Some(signature)=final_checkpoint_mesh.as_ref() {
                            let path=final_checkpoint_path.with_file_name("failed-last-sample.json");
                            let index=final_counter.load(Ordering::Acquire);
                            if let Err(e)=crate::workbench::checkpoint::write_restart(&path,signature,index,&initial,physical,&final_checkpoint_context) {
                                let _=sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Failed-solve sample could not be archived: {e}"))));
                            } else {
                                let _=sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("UNCONVERGED retained sample {index}: {}",path.display()))));
                            }
                        }
                    }
                }
                }
                let cancelled = cancellation.is_cancelled();
                let _ = sender.send(WorkbenchJobEvent::SolveCompleted(result));
                let _ = sender.send(WorkbenchJobEvent::SolveProgress(if cancelled {
                    JobProgress::Cancelled
                } else {
                    JobProgress::Completed
                }));
            });
        })
    }

    pub fn cancel(&self) -> bool {
        let Some(token) = &self.cancellation else {
            return false;
        };
        token.cancel();
        true
    }

    pub fn drain(&mut self) -> Vec<WorkbenchJobEvent> {
        let mut events = Vec::new();
        let Some(receiver) = self.receiver.take() else {
            return events;
        };
        loop {
            match receiver.try_recv() {
                Ok(event) => {
                    match &event {
                        WorkbenchJobEvent::MeshCompleted(Ok(_))
                        | WorkbenchJobEvent::SolveCompleted(Ok(_)) => {
                            self.lifecycle = JobLifecycle::Completed;
                        }
                        WorkbenchJobEvent::MeshCompleted(Err(MeshingError::Cancelled))
                        | WorkbenchJobEvent::SolveCompleted(Err(
                            IncompressibleSolveError::Cancelled,
                        )) => {
                            self.lifecycle = JobLifecycle::Cancelled;
                        }
                        WorkbenchJobEvent::MeshCompleted(Err(_))
                        | WorkbenchJobEvent::SolveCompleted(Err(_)) => {
                            self.lifecycle = JobLifecycle::Failed
                        }
                        _ => {}
                    }
                    events.push(event);
                }
                Err(mpsc::TryRecvError::Empty) => {
                    self.receiver = Some(receiver);
                    break;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    if self.lifecycle == JobLifecycle::Running {
                        self.lifecycle = JobLifecycle::Failed;
                    }
                    break;
                }
            }
        }
        if self.lifecycle != JobLifecycle::Running {
            self.cancellation = None;
        }
        events
    }

    fn start(
        &mut self,
        spawn: impl FnOnce(mpsc::Sender<WorkbenchJobEvent>, CancellationToken),
    ) -> Result<(), &'static str> {
        if self.is_running() {
            return Err("a workbench job is already running");
        }
        self.paused.store(false, Ordering::Release);
        let (sender, receiver) = mpsc::channel();
        let cancellation = CancellationToken::default();
        self.receiver = Some(receiver);
        self.cancellation = Some(cancellation.clone());
        self.lifecycle = JobLifecycle::Running;
        spawn(sender, cancellation);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paused_worker_resumes_and_cancels_without_losing_state() {
        for cancel in [false, true] {
            let paused = Arc::new(AtomicBool::new(true));
            let token = CancellationToken::default();
            let (tx, rx) = mpsc::channel();
            let worker_pause = paused.clone();
            let worker_token = token.clone();
            let worker = std::thread::spawn(move || {
                let retained_fields = vec![1.0, 2.0, 3.0];
                let cancelled = wait_for_resume(&worker_pause, &worker_token);
                tx.send((cancelled, retained_fields)).unwrap();
            });
            assert!(rx
                .recv_timeout(std::time::Duration::from_millis(100))
                .is_err());
            if cancel {
                token.cancel();
            } else {
                paused.store(false, Ordering::Release);
            }
            let (cancelled, fields) = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
            assert_eq!(cancelled, cancel);
            assert_eq!(fields, vec![1.0, 2.0, 3.0]);
            worker.join().unwrap();
        }
    }

    #[test]
    fn cancellation_token_is_shared() {
        let token = CancellationToken::default();
        assert!(!token.is_cancelled());
        token.clone().cancel();
        assert!(token.is_cancelled());
    }

    #[test]
    fn cancelled_mesh_completion_transitions_executor() {
        let (sender, receiver) = mpsc::channel();
        let mut executor = WorkbenchJobExecutor {
            receiver: Some(receiver),
            cancellation: Some(CancellationToken::default()),
            lifecycle: JobLifecycle::Running,
            paused: Arc::new(AtomicBool::new(false)),
            restart: None,
            checkpoint_every: 100,
            completed: Arc::new(AtomicUsize::new(0)),
            retained: Arc::new(Mutex::new(None)),
        };
        sender
            .send(WorkbenchJobEvent::MeshCompleted(Err(
                MeshingError::Cancelled,
            )))
            .unwrap();
        drop(sender);

        assert_eq!(executor.drain().len(), 1);
        assert_eq!(executor.lifecycle(), JobLifecycle::Cancelled);
        assert!(!executor.cancel());
    }
}
