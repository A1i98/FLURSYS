use crate::{
    solve_incompressible_with_control, GeneratedMesh, GeometryGmshExport, GmshMeshOptions,
    GmshMesher, IncompressibleCase, IncompressibleSolution, IncompressibleSolveError, MeshingError,
    SolveControl,
};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{self, Receiver},
    Arc,
};

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
    Started,
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
pub enum WorkbenchJobEvent {
    MeshProgress(JobProgress),
    MeshCompleted(Result<GeneratedMesh, MeshingError>),
    SolveProgress(JobProgress),
    SolveCompleted(Result<IncompressibleSolution, IncompressibleSolveError>),
}

/// Owns worker channels and lifecycle state for the canonical workbench jobs.
/// Only one mesh or solve job may run at a time because both mutate the same
/// session lifecycle once their completion event is applied.
pub struct WorkbenchJobExecutor {
    receiver: Option<Receiver<WorkbenchJobEvent>>,
    cancellation: Option<CancellationToken>,
    lifecycle: JobLifecycle,
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
        }
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
                let result =
                    GmshMesher::auto().generate_cancellable(&export.document, &options, || {
                        cancellation.is_cancelled()
                    });
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
        self.start(|sender, cancellation| {
            std::thread::spawn(move || {
                let _ = sender.send(WorkbenchJobEvent::SolveProgress(JobProgress::Started));
                let progress_sender = sender.clone();
                let control_cancellation = cancellation.clone();
                let control = SolveControl::new(
                    move || control_cancellation.is_cancelled(),
                    move |iteration, max, residual| {
                        let _ = progress_sender.send(WorkbenchJobEvent::SolveProgress(
                            JobProgress::Iteration {
                                current: iteration,
                                max,
                                residual: Some(residual),
                            },
                        ));
                    },
                );
                let result = solve_incompressible_with_control(&case, &control);
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
