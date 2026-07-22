//! Asynchronous solver execution for interactive clients.
//!
//! The worker owns the numerical solver. Consumers communicate exclusively through
//! channels, so rendering and event handling never block on an iteration.

use crate::{FieldUpdate, IncompressibleSolver, SimulationConfig, SolverStep};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

#[derive(Clone, Debug)]
pub enum SolverCommand {
    Start(Box<SimulationConfig>),
    Pause,
    Resume,
    Stop,
    SetIterations(usize),
    Shutdown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolverState {
    Idle,
    Running,
    Paused,
    Completed,
    Stopped,
    Failed,
}

#[derive(Clone, Debug)]
pub struct SolverUpdate {
    pub state: SolverState,
    pub iteration: usize,
    pub continuity_residual: f64,
    pub momentum_residual: f64,
    pub pressure_residual: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: f64,
    pub elapsed_seconds: f64,
    pub converged: bool,
    pub field_update: Option<FieldUpdate>,
    pub message: Option<String>,
}

impl SolverUpdate {
    fn status(state: SolverState, message: Option<String>) -> Self {
        Self {
            state,
            iteration: 0,
            continuity_residual: 0.0,
            momentum_residual: 0.0,
            pressure_residual: 0.0,
            drag_coefficient: 0.0,
            lift_coefficient: 0.0,
            elapsed_seconds: 0.0,
            converged: false,
            field_update: None,
            message,
        }
    }

    fn from_step(
        state: SolverState,
        step: SolverStep,
        elapsed_seconds: f64,
        field_update: Option<FieldUpdate>,
    ) -> Self {
        Self {
            state,
            iteration: step.iteration,
            continuity_residual: step.continuity_residual,
            momentum_residual: step.momentum_residual,
            pressure_residual: step.pressure_residual,
            drag_coefficient: step.drag_coefficient,
            lift_coefficient: step.lift_coefficient,
            elapsed_seconds,
            converged: step.converged,
            field_update,
            message: None,
        }
    }
}

pub struct SolverController {
    commands: mpsc::Sender<SolverCommand>,
    updates: Receiver<SolverUpdate>,
    worker: Option<JoinHandle<()>>,
}

impl SolverController {
    pub fn spawn() -> Self {
        let (commands, command_rx) = mpsc::channel();
        let (update_tx, updates) = mpsc::channel();
        let worker = thread::Builder::new()
            .name("flursys-solver".to_string())
            .spawn(move || worker_loop(command_rx, update_tx))
            .expect("cannot create FLURSYS solver worker thread");

        Self {
            commands,
            updates,
            worker: Some(worker),
        }
    }

    pub fn send(&self, command: SolverCommand) -> Result<(), String> {
        self.commands
            .send(command)
            .map_err(|_| "solver worker is no longer running".to_string())
    }

    pub fn try_recv(&self) -> Result<SolverUpdate, mpsc::TryRecvError> {
        self.updates.try_recv()
    }
}

impl Drop for SolverController {
    fn drop(&mut self) {
        let _ = self.commands.send(SolverCommand::Shutdown);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

struct ActiveSolver {
    solver: IncompressibleSolver,
    max_iterations: usize,
    started: Instant,
    last_update: Instant,
    last_field_update: Instant,
    paused: bool,
}

fn worker_loop(commands: Receiver<SolverCommand>, updates: Sender<SolverUpdate>) {
    let mut active: Option<ActiveSolver> = None;
    let mut last_state = SolverState::Idle;

    loop {
        if active.is_none() {
            match commands.recv() {
                Ok(command) => {
                    if !handle_command(command, &mut active, &updates, &mut last_state) {
                        break;
                    }
                }
                Err(_) => break,
            }
            continue;
        }

        while let Ok(command) = commands.try_recv() {
            if !handle_command(command, &mut active, &updates, &mut last_state) {
                return;
            }
        }

        let Some(run) = active.as_mut() else {
            continue;
        };
        if run.paused {
            thread::sleep(Duration::from_millis(8));
            continue;
        }

        match run.solver.advance() {
            Ok(step) => {
                let completed = step.converged || step.iteration >= run.max_iterations;
                let now = Instant::now();
                let publish_update =
                    completed || now.duration_since(run.last_update) >= Duration::from_millis(16);
                if !publish_update {
                    continue;
                }
                let snapshot = completed
                    || now.duration_since(run.last_field_update) >= Duration::from_millis(100);
                let update = SolverUpdate::from_step(
                    if completed {
                        SolverState::Completed
                    } else {
                        SolverState::Running
                    },
                    step,
                    run.started.elapsed().as_secs_f64(),
                    snapshot.then(|| run.solver.field_update()),
                );
                publish(&updates, update);
                run.last_update = now;
                if snapshot {
                    run.last_field_update = now;
                }
                if completed {
                    active = None;
                    last_state = SolverState::Completed;
                }
            }
            Err(error) => {
                publish(
                    &updates,
                    SolverUpdate::status(SolverState::Failed, Some(error)),
                );
                active = None;
                last_state = SolverState::Failed;
            }
        }
    }
}

/// Returns false when the worker must terminate.
fn handle_command(
    command: SolverCommand,
    active: &mut Option<ActiveSolver>,
    updates: &Sender<SolverUpdate>,
    last_state: &mut SolverState,
) -> bool {
    match command {
        SolverCommand::Start(config) => match IncompressibleSolver::new((*config).clone()) {
            Ok(solver) => {
                *active = Some(ActiveSolver {
                    solver,
                    max_iterations: config.max_steps,
                    started: Instant::now(),
                    last_update: Instant::now(),
                    last_field_update: Instant::now(),
                    paused: false,
                });
                *last_state = SolverState::Running;
                publish(updates, SolverUpdate::status(SolverState::Running, None));
            }
            Err(error) => {
                *last_state = SolverState::Failed;
                publish(
                    updates,
                    SolverUpdate::status(SolverState::Failed, Some(error)),
                );
            }
        },
        SolverCommand::Pause => {
            if let Some(run) = active.as_mut() {
                run.paused = true;
                *last_state = SolverState::Paused;
                publish(updates, SolverUpdate::status(SolverState::Paused, None));
            }
        }
        SolverCommand::Resume => {
            if let Some(run) = active.as_mut() {
                run.paused = false;
                *last_state = SolverState::Running;
                publish(updates, SolverUpdate::status(SolverState::Running, None));
            }
        }
        SolverCommand::Stop => {
            *active = None;
            *last_state = SolverState::Stopped;
            publish(updates, SolverUpdate::status(SolverState::Stopped, None));
        }
        SolverCommand::SetIterations(iterations) => {
            if iterations == 0 {
                publish(
                    updates,
                    SolverUpdate::status(
                        *last_state,
                        Some("iteration limit must be positive".to_string()),
                    ),
                );
            } else if let Some(run) = active.as_mut() {
                run.max_iterations = iterations;
            }
        }
        SolverCommand::Shutdown => return false,
    }
    true
}

fn publish(updates: &Sender<SolverUpdate>, update: SolverUpdate) {
    let _ = updates.send(update);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cases::CavityCase;
    use crate::{
        Case, ConvectionScheme, PressureSolverKind, PressureVelocityCoupling,
        SolverBoundaryOverrides,
    };
    use std::path::PathBuf;

    fn config() -> SimulationConfig {
        SimulationConfig {
            case: Case::LidDrivenCavity(CavityCase::default()),
            nx: 16,
            ny: 16,
            dt: 1.0e-3,
            max_steps: 2,
            t_end: 0.0,
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling: PressureVelocityCoupling::Simple,
            pressure_solver: PressureSolverKind::Pcg,
            pressure_max_iters: 800,
            pressure_tolerance: 1.0e-6,
            pressure_omega: 1.7,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            print_every: 100,
            output_every: 100,
            frame_every: 100,
            steady_tolerance: 1.0e-10,
            minimum_steps: 100,
            threads: 1,
            boundary_overrides: SolverBoundaryOverrides::default(),
            output_dir: PathBuf::from("target/runtime-test"),
        }
    }

    #[test]
    fn worker_runs_without_blocking_the_caller() {
        let controller = SolverController::spawn();
        controller
            .send(SolverCommand::Start(Box::new(config())))
            .unwrap();

        let deadline = Instant::now() + Duration::from_secs(2);
        let mut completed = false;
        while Instant::now() < deadline {
            match controller.try_recv() {
                Ok(update) if update.state == SolverState::Completed => {
                    completed = true;
                    break;
                }
                Ok(_) | Err(mpsc::TryRecvError::Empty) => thread::sleep(Duration::from_millis(2)),
                Err(mpsc::TryRecvError::Disconnected) => break,
            }
        }
        assert!(completed, "solver worker did not complete in time");
    }
}
