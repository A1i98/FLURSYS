use flursys::cases::{BackwardStepCase, CavityCase, CylinderCase};
use flursys::{
    Case, ConvectionScheme, IncompressibleSolver, PressureSolverKind, PressureVelocityCoupling,
    SimulationConfig, SolverBoundaryOverrides,
};
use std::path::PathBuf;

fn config(case: Case, nx: usize, ny: usize, name: &str) -> SimulationConfig {
    SimulationConfig {
        case,
        nx,
        ny,
        dt: 1.0e-3,
        max_steps: 1,
        t_end: 1.0e-3,
        convection: ConvectionScheme::FirstOrderUpwind,
        coupling: PressureVelocityCoupling::Projection,
        pressure_solver: PressureSolverKind::Pcg,
        pressure_max_iters: 800,
        pressure_tolerance: 1.0e-6,
        pressure_omega: 1.7,
        velocity_relaxation: 0.7,
        pressure_relaxation: 0.3,
        print_every: 1,
        output_every: 1,
        frame_every: 1,
        steady_tolerance: 1.0e-10,
        minimum_steps: 100,
        threads: 1,
        boundary_overrides: SolverBoundaryOverrides::default(),
        output_dir: PathBuf::from(format!("target/smoke-tests/{name}")),
    }
}

#[test]
fn simple_coupling_advances_a_steady_iteration() {
    let mut cfg = config(
        Case::LidDrivenCavity(CavityCase::default()),
        16,
        16,
        "simple-cavity",
    );
    cfg.coupling = PressureVelocityCoupling::Simple;
    cfg.max_steps = 2;
    cfg.t_end = 0.0;
    cfg.minimum_steps = 1;

    let summary = IncompressibleSolver::new(cfg).unwrap().run().unwrap();
    assert_eq!(summary.steps, 2);
    assert!(summary.max_divergence < 1.0e-5);
    assert!(summary.pressure_residual < 1.0e-5);
}

#[test]
fn cavity_advances_one_step() {
    let mut solver = IncompressibleSolver::new(config(
        Case::LidDrivenCavity(CavityCase::default()),
        16,
        16,
        "cavity",
    ))
    .unwrap();
    let summary = solver.run().unwrap();
    assert!(summary.max_divergence.is_finite());
    assert!(summary.max_divergence < 1.0e-5);
}

#[test]
fn cylinder_advances_one_step() {
    let mut solver = IncompressibleSolver::new(config(
        Case::CylinderRe100(CylinderCase::default()),
        80,
        32,
        "cylinder",
    ))
    .unwrap();
    let summary = solver.run().unwrap();
    assert!(summary.max_divergence.is_finite());
    assert!(summary.max_divergence < 1.0e-5);
}

#[test]
fn backward_step_advances_one_step() {
    let mut solver = IncompressibleSolver::new(config(
        Case::BackwardFacingStep(BackwardStepCase::default()),
        70,
        8,
        "backward-step",
    ))
    .unwrap();
    let summary = solver.run().unwrap();
    assert!(summary.max_divergence.is_finite());
    assert!(summary.max_divergence < 1.0e-5);
}

#[test]
fn multithreaded_pcg_matches_single_thread() {
    let mut single = config(
        Case::LidDrivenCavity(CavityCase::default()),
        32,
        32,
        "parallel-equivalence-single",
    );
    single.max_steps = 3;
    single.t_end = 3.0e-3;
    single.threads = 1;

    let mut parallel = single.clone();
    parallel.threads = 4;
    parallel.output_dir = PathBuf::from("target/smoke-tests/parallel-equivalence-many");

    let single_summary = IncompressibleSolver::new(single).unwrap().run().unwrap();
    let parallel_summary = IncompressibleSolver::new(parallel).unwrap().run().unwrap();

    assert_eq!(single_summary.steps, parallel_summary.steps);
    assert!((single_summary.max_divergence - parallel_summary.max_divergence).abs() < 1.0e-10);
    assert!(
        (single_summary.pressure_residual - parallel_summary.pressure_residual).abs() < 1.0e-10
    );
}
