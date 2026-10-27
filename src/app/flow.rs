use super::*;
use crate::{
    compressible as gas,
    workbench::{checkpoint, flow_frame::FlowFrame},
    UnstructuredMesh,
};
use std::{path::PathBuf, time::Instant};

#[derive(Default)]
pub(super) struct FlowRuntime {
    pub(super) retained: Arc<Mutex<Option<(crate::MeshId, Arc<gas::Snapshot>)>>>,
    pub(super) writer: Option<Arc<FrameWriter>>,
}
impl FlowRuntime {
    pub(super) fn reset(&mut self) {
        *self.retained.lock().unwrap() = None;
        self.writer = None;
    }
}
pub(super) struct FrameWriter {
    mesh: Arc<UnstructuredMesh>,
    folder: PathBuf,
    run_id: String,
    policy: checkpoint::RecoveryPolicy,
    field_every: usize,
    history_disabled: AtomicBool,
    history_error: Mutex<Option<String>>,
    clock: Mutex<(Instant, Option<usize>)>,
}
impl FrameWriter {
    fn save(
        &self,
        state: &gas::Snapshot,
        force: bool,
    ) -> Result<Option<checkpoint::CheckpointInfo>, String> {
        let mut clock = self.clock.lock().map_err(|_| "Frame writer lock failed")?;
        let step = state.report.accepted_steps;
        let due = force
            || (clock.1.is_none()
                && (self.policy.every_iterations > 0 || self.policy.every_seconds > 0))
            || (self.policy.every_iterations > 0 && step % self.policy.every_iterations == 0)
            || (self.policy.every_seconds > 0
                && clock.0.elapsed().as_secs() >= self.policy.every_seconds);
        let history = self.field_every > 0
            && !self.history_disabled.load(Ordering::Acquire)
            && (step % self.field_every == 0 || force);
        if !due && !history {
            return Ok(None);
        }
        let frame = FlowFrame::compressible(&self.run_id, &self.mesh, state.clone());
        let saved = if due {
            Some(frame.write(&self.folder.join("checkpoint.json"), None)?)
        } else {
            None
        };
        if history {
            let result = frame
                .write(
                    &self
                        .folder
                        .join("fields")
                        .join(format!("iteration-{step:010}.json")),
                    Some(self.policy.max_history_bytes),
                )
                .and_then(|_| checkpoint::prune_history(&self.folder.join("fields"), &self.policy));
            if let Err(error) = result {
                self.history_disabled.store(true, Ordering::Release);
                *self
                    .history_error
                    .lock()
                    .map_err(|_| "History status lock failed")? = Some(error);
            }
        }
        if due {
            *clock = (Instant::now(), Some(step));
        }
        Ok(saved)
    }
}
impl WorkbenchJobExecutor {
    pub fn retained_compressible(&self, mesh: crate::MeshId) -> Option<Arc<gas::Snapshot>> {
        self.flow
            .retained
            .lock()
            .ok()?
            .as_ref()
            .filter(|(id, _)| *id == mesh)
            .map(|(_, state)| state.clone())
    }
    pub fn initialize_compressible(&mut self) {
        self.flow.reset();
        self.completed.store(0, Ordering::Release);
        self.restart = None;
    }
    pub fn set_compressible_state(
        &mut self,
        mesh: &UnstructuredMesh,
        state: gas::Snapshot,
    ) -> Result<(), String> {
        state.validate(mesh, state.gas)?;
        self.completed
            .store(state.report.accepted_steps, Ordering::Release);
        *self
            .flow
            .retained
            .lock()
            .map_err(|_| "Retained state unavailable")? = Some((mesh.id(), Arc::new(state)));
        Ok(())
    }
    pub(super) fn checkpoint_flow_now(&self) -> Result<(), String> {
        let writer = self
            .flow
            .writer
            .clone()
            .ok_or("No running flow checkpoint writer")?;
        let state = self
            .flow
            .retained
            .lock()
            .map_err(|_| "Retained fields unavailable")?
            .as_ref()
            .map(|(_, state)| state.clone())
            .ok_or("Wait for an accepted state")?;
        if self.checkpoint_pending.swap(true, Ordering::AcqRel) {
            return Err("A checkpoint write is already in progress".into());
        }
        let pending = self.checkpoint_pending.clone();
        let result = self.checkpoint_result.clone();
        std::thread::spawn(move || {
            *result.lock().unwrap() = Some(
                writer
                    .save(&state, true)
                    .and_then(|v| v.ok_or("No checkpoint written".into())),
            );
            pending.store(false, Ordering::Release);
        });
        Ok(())
    }
    pub fn start_compressible(
        &mut self,
        mesh: UnstructuredMesh,
        settings: gas::Settings,
        folder: PathBuf,
        run_id: String,
    ) -> Result<(), String> {
        if self.is_running() || self.is_checkpointing() {
            return Err("A solve or checkpoint write is already running".into());
        }
        settings.validate(&mesh)?;
        if let Some(path) = self.restart.take() {
            let frame = FlowFrame::read(&path).or_else(|_| {
                checkpoint::backup_paths(&path)
                    .into_iter()
                    .find_map(|p| FlowFrame::read(&p).ok())
                    .ok_or_else(|| String::from("No valid recovery checkpoint"))
            })?;
            self.set_compressible_state(&mesh, frame.gas_restart(&mesh, settings.gas)?)?;
        }
        let restart = self.retained_compressible(mesh.id());
        if let Some(state) = &restart {
            state.validate(&mesh, settings.gas)?;
        }
        let offset = restart.as_ref().map_or(0, |s| s.report.accepted_steps);
        self.completed.store(offset, Ordering::Release);
        self.recovery_writer = None;
        let mesh = Arc::new(mesh);
        let writer = Arc::new(FrameWriter {
            mesh: mesh.clone(),
            folder,
            run_id,
            policy: self.recovery_policy.clone(),
            field_every: self.checkpoint_every,
            history_disabled: AtomicBool::new(false),
            history_error: Mutex::new(None),
            clock: Mutex::new((Instant::now(), None)),
        });
        self.flow.writer = Some(writer.clone());
        let retained = self.flow.retained.clone();
        let paused = self.paused.clone();
        let completed = self.completed.clone();
        let report_every = self.engineering_every;
        self.start(move |sender, cancellation| {
            std::thread::spawn(move || {
                let _ = sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::StartedAt {
                    iteration: offset,
                }));
                let result = gas::solve_resuming(&mesh, &settings, restart.as_deref(), |state| {
                    completed.store(state.report.accepted_steps, Ordering::Release);
                    *retained.lock().map_err(|_| "Retained state unavailable")? =
                        Some((mesh.id(), state.clone()));
                    match writer.save(&state, false) {
                        Ok(Some(info)) => {
                            let _ = sender.send(WorkbenchJobEvent::Checkpoint(Ok(info)));
                        }
                        Err(e) if writer.policy.stop_on_error => return Err(e),
                        Err(e) => {
                            let _ = sender.send(WorkbenchJobEvent::SolveProgress(
                                JobProgress::Message(format!("Checkpoint write failed: {e}")),
                            ));
                        }
                        _ => {}
                    }
                    if let Some(error) = writer.history_error.lock().map_err(|_| "History status lock failed")?.take() {
                        let _ = sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Message(format!("Field history paused: {error}. Recovery checkpoints remain active."))));
                    }
                    let report = &state.report;
                    if report.accepted_steps % report_every == 0 {
                        let reports = gas::wall_reports(&mesh, &settings, &state, false)?;
                        let sample = crate::workbench::force_analysis::Sample {
                            index: report.accepted_steps,
                            time: Some(report.time),
                            density: settings.initial.density,
                            reports: reports.clone(),
                        };
                        crate::workbench::force_analysis::append(
                            &writer.folder.join("force-history.jsonl"),
                            &sample,
                        )?;
                        let _ = sender.send(WorkbenchJobEvent::WallReports {
                            iteration: report.accepted_steps,
                            physical_time: Some(report.time),
                            reports,
                        });
                    }

                    let _ = sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Iteration {
                        current: report.accepted_steps,
                        max: offset.saturating_add(settings.max_steps),
                        residual: Some(report.residuals[0]),
                    }));
                    let _ = sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Metrics {
                        iteration: report.accepted_steps,
                        values: report.residuals,
                    }));
                    let _ = sender.send(WorkbenchJobEvent::TimeStep {
                        index: report.accepted_steps,
                        time: report.time,
                        dt: Some(report.last_dt),
                        courant: Some(settings.courant),
                        rejected: report.rejected_steps,
                    });
                    Ok(if wait_for_resume(&paused, &cancellation) {
                        gas::StepAction::Stop
                    } else {
                        gas::StepAction::Continue
                    })
                });
                if let Ok(solution) = &result {
                    *retained.lock().unwrap() =
                        Some((mesh.id(), Arc::new(solution.snapshot(settings.gas))));
                }
                let final_state = retained
                    .lock()
                    .ok()
                    .and_then(|v| v.as_ref().map(|(_, state)| state.clone()));
                if let Some(state) = final_state {
                    match writer.save(&state, true) {
                        Ok(Some(info)) => {
                            let _ = sender.send(WorkbenchJobEvent::Checkpoint(Ok(info)));
                        }
                        Err(e) => {
                            let _ = sender.send(WorkbenchJobEvent::Checkpoint(Err(e)));
                        }
                        _ => {}
                    }
                }
                if let Ok(solution) = &result {
                    let _ = sender.send(WorkbenchJobEvent::Performance(
                        solution.report.performance.clone(),
                    ));
                }
                let _ = sender.send(WorkbenchJobEvent::CompressibleCompleted(result));
            });
        })
        .map_err(str::to_string)
    }
}
