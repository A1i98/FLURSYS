//! Small numerical regressions; these are not industrial benchmark certification.
use flursys::unstructured_incompressible::{AdaptiveTimeStep, TimeIntegration};
use flursys::*;
fn mesh() -> UnstructuredMesh {
    let n = 4;
    let mut points = vec![];
    for j in 0..=n {
        for i in 0..=n {
            points.push(Point::new(i as f64 / n as f64, j as f64 / n as f64, 0.));
        }
    }
    let mut cells = vec![];
    for j in 0..n {
        for i in 0..n {
            let a = j * (n + 1) + i;
            cells.push(CellDefinition::polygon(vec![
                a,
                a + 1,
                a + n + 2,
                a + n + 1,
            ]));
        }
    }
    let mesh = UnstructuredMesh::from_cells(MeshDimension::TwoD, points, cells).unwrap();
    let mut patches: Vec<_> = ["walls", "lid"]
        .iter()
        .map(|s| BoundaryPatch {
            name: s.to_string(),
            face_indices: vec![],
            boundary_type: BoundaryType::Wall,
        })
        .collect();
    for (i, f) in mesh
        .faces()
        .iter()
        .enumerate()
        .filter(|(_, f)| f.neighbour.is_none())
    {
        patches[usize::from(f.center.y > 1. - 1e-12)]
            .face_indices
            .push(i);
    }
    mesh.with_boundary_patches(patches).unwrap()
}
fn case() -> IncompressibleCase {
    IncompressibleCase::steady(
        mesh(),
        vec![
            ("walls".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "lid".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(0.1, 0., 0.),
                },
            ),
        ],
        IncompressibleMaterial {
            density: 1.,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions {
            max_outer_iterations: 2000,
            coupling_relative_tolerance: 1e-8,
            velocity_relaxation: 0.5,
            ..Default::default()
        },
    )
}
#[test]
fn simplec_agrees_with_simple_on_small_viscous_cavity() {
    let mut c = case();
    let a = solve_incompressible(&c).unwrap();
    assert!(a.report.converged());
    c.solver.pressure_coupling.algorithm = flursys::simple::CouplingAlgorithm::Simplec;
    let b = solve_incompressible(&c).unwrap();
    assert!(b.report.converged());
    let difference = a
        .velocity
        .values()
        .iter()
        .zip(b.velocity.values())
        .map(|(a, b)| (*a - *b).norm())
        .fold(0., f64::max);
    assert!(
        difference < 1e-6,
        "SIMPLE/SIMPLEC difference {difference:e}"
    );
}
#[test]
fn convection_schemes_return_finite_coupled_solutions() {
    for scheme in [
        flursys::momentum::MomentumConvection::LimitedLinear,
        flursys::momentum::MomentumConvection::Central,
        flursys::momentum::MomentumConvection::VanLeer,
    ] {
        let mut c = case();
        c.solver.convection = scheme;
        let s = solve_incompressible(&c).unwrap();
        assert!(s.report.converged(), "{scheme:?}");
        assert!(s.velocity.values().iter().all(|v| v.norm().is_finite()));
        assert!(s.report.final_continuity_rms < 1e-9);
    }
}
fn heating() -> IncompressibleCase {
    let mut c = case();
    c.boundaries = vec![
        ("walls".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ("lid".into(), IncompressibleBoundaryCondition::NoSlipWall),
    ];
    c.solver.thermal = flursys::thermal::Options {
        enabled: true,
        initial_temperature: 300.,
        specific_heat: 10.,
        conductivity: 0.1,
        source_w_m3: 20.,
        relaxation: 1.,
        relative_tolerance: 1e-10,
        boundaries: [
            ("walls".into(), flursys::thermal::Boundary::Adiabatic),
            ("lid".into(), flursys::thermal::Boundary::Adiabatic),
        ]
        .into(),
        ..Default::default()
    };
    c
}
#[test]
fn end_time_shortens_last_step_and_restart_advances_absolute_time() {
    for backward in [false, true] {
        let mut c = heating();
        c.solver.time = if backward {
            TimeIntegration::Backward { dt: 0.1, steps: 10 }
        } else {
            TimeIntegration::Euler { dt: 0.1, steps: 10 }
        };
        c.solver.physical_end_time = Some(0.25);
        let s = solve_incompressible(&c).unwrap();
        assert_eq!(s.completed_time_steps, 3);
        assert!((s.physical_time.unwrap() - 0.25).abs() < 1e-14);
        assert!((s.last_time_step.unwrap() - 0.05).abs() < 1e-14);
        for t in s.temperature.as_ref().unwrap().values() {
            assert!((t - 300.5).abs() < 1e-7, "T={t}");
        }
        let budget = s.budgets.as_ref().unwrap();
        assert!(budget.momentum.as_ref().unwrap().imbalance.norm() < 1e-12);
        assert!(budget.energy.as_ref().unwrap().relative_imbalance.unwrap() < 1e-7);
        c.initial_conditions = s.restart();
        c.solver.physical_end_time = Some(0.4);
        let resumed = solve_incompressible(&c).unwrap();
        assert!((resumed.physical_time.unwrap() - 0.4).abs() < 1e-14);
        for t in resumed.temperature.as_ref().unwrap().values() {
            assert!((t - 300.8).abs() < 1e-7);
        }
        c.initial_conditions = resumed.restart();
        assert!(solve_incompressible(&c).is_err());
    }
}
#[test]
fn adaptive_diffusion_limit_and_final_remainder_are_respected() {
    let mut c = heating();
    c.solver.time = TimeIntegration::AdaptiveEuler {
        dt: 0.1,
        steps: 100,
        controls: AdaptiveTimeStep {
            max_diffusion: Some(0.05),
            min_dt: 1e-5,
            max_dt: 0.1,
            ..Default::default()
        },
    };
    c.solver.physical_end_time = Some(0.123456);
    let s = solve_incompressible(&c).unwrap();
    assert!((s.physical_time.unwrap() - 0.123456).abs() < 1e-14);
    assert!(s.max_diffusion_number.unwrap() <= 0.05 * (1. + 1e-6));
    assert!(s.completed_time_steps > 1);
}
#[test]
fn dfg_recipe_has_reference_geometry_and_load_settings() {
    let s = flursys::workbench::dfg::session().unwrap();
    let p = s.to_project("DFG");
    assert_eq!(p.material.density, 1.);
    assert_eq!(p.material.kinematic_viscosity, 0.001);
    assert_eq!(p.solver.force_analysis.patches, vec!["cylinder"]);
    assert_eq!(p.solver.physical_end_time, Some(30.));
    assert!(s.mesh_generation_inputs().is_ok());
}
#[test]
fn cooperative_stop_is_terminal() {
    let c = flursys::headless::ExecutionControl::default();
    c.pause();
    c.resume();
    assert!(!c.is_stopped());
    c.stop();
    c.resume();
    c.pause();
    assert!(c.is_stopped());
}

#[test]
fn wall_coefficients_shift_moments_and_find_reattachment() {
    let c = case();
    let s = solve_incompressible(&c).unwrap();
    let mut reports =
        flursys::workbench::wall_reports::reports(&c, &s.velocity, &s.pressure, true).unwrap();
    let r = &mut reports[0];
    r.pressure_force = Vec3::new(2., 3., 0.);
    r.shear_force = Vec3::new(1., 0., 0.);
    r.total_force = r.pressure_force + r.shear_force;
    r.pressure_moment = Vec3::new(0., 0., 4.);
    r.shear_moment = Vec3::ZERO;
    let coeff = r
        .coefficients(flursys::workbench::wall_reports::ForceReference {
            density: 2.,
            speed: 2.,
            measure: 0.5,
            length: 2.,
            center: Vec3::new(1., 0., 0.),
        })
        .unwrap();
    assert_eq!(coeff.total, Vec3::new(1.5, 1.5, 0.));
    assert!((coeff.moment.z - 0.25).abs() < 1e-14);
    r.samples.truncate(2);
    assert_eq!(r.samples.len(), 2);
    r.samples[0].point = Vec3::new(1., 0., 0.);
    r.samples[1].point = Vec3::new(2., 0., 0.);
    r.samples[0].shear_stress = Vec3::new(-1., 0., 0.);
    r.samples[1].shear_stress = Vec3::new(3., 0., 0.);
    let xs = flursys::workbench::wall_reports::reattachment_distances(
        r,
        Vec3::ZERO,
        Vec3::new(1., 0., 0.),
        1e-6,
    )
    .unwrap();
    assert_eq!(xs, vec![1.25]);
}
#[test]
fn dfg_recipe_generates_a_mesh_with_all_boundary_patches() {
    let mut session = flursys::workbench::dfg::session().unwrap();
    let (export, options) = session.mesh_generation_inputs().unwrap();
    let generated = GmshMesher::auto()
        .generate(&export.document, &options)
        .unwrap();
    assert!(generated.mesh.cell_count() > 100);
    let names: std::collections::BTreeSet<_> = generated
        .mesh
        .boundary_patches()
        .iter()
        .map(|p| p.name.as_str())
        .collect();
    assert_eq!(names, ["inlet", "outlet", "walls", "cylinder"].into());
    session.install_mesh(generated);
    assert!(session.prepare_case().is_ok());
}

#[test]
fn headless_run_saves_load_history_and_physical_restart() {
    let directory = tempfile::tempdir().unwrap();
    let mut session =
        flursys::workbench::build_example(flursys::workbench::ExampleProjectId::LidDrivenCavity2D)
            .unwrap();
    session
        .set_mesh_configuration(MeshDimension::TwoD, 0.25, 0.25, 0.25, 1)
        .unwrap();
    session
        .set_flow_models(
            TimeIntegration::Euler { dt: 0.01, steps: 2 },
            Default::default(),
        )
        .unwrap();
    session.set_solver_controls(500, 0.5, 0.3, 1e-8).unwrap();
    let mut project = session.to_project("Headless regression");
    flursys::headless::mesh(
        directory.path(),
        &mut project,
        &mut session,
        &GmshMesher::auto(),
    )
    .unwrap();
    let control = flursys::headless::ExecutionControl::default();
    let mut paused = false;
    let mut resumer = None;
    let outcome = flursys::headless::run_with_control(
        directory.path(),
        &mut project,
        &mut session,
        &Default::default(),
        &control,
        |_| {
            if !paused {
                paused = true;
                control.pause();
                let c = control.clone();
                resumer = Some(std::thread::spawn(move || {
                    std::thread::sleep(std::time::Duration::from_millis(30));
                    c.resume();
                }));
            }
        },
    )
    .unwrap();
    if let Some(thread) = resumer {
        thread.join().unwrap();
    }
    assert!(outcome.failure.is_none(), "{:?}", outcome.failure);
    let run = outcome.run.unwrap();
    let report = flursys::headless::report(directory.path(), &run).unwrap();
    assert_eq!(report["completed_time_steps"], 2);
    assert!(report["surface_budgets"]["Ok"]["momentum"].is_object());
    let history = flursys::headless::force_history(directory.path(), &run).unwrap();
    assert_eq!(history.len(), 2);
    assert!(history.iter().all(|s| s.time.is_some()));
    let checkpoint = directory.path().join(outcome.restart_checkpoint.unwrap());
    let (_, initial) =
        flursys::workbench::checkpoint::load(&checkpoint, &session.mesh().unwrap().mesh).unwrap();
    match initial {
        flursys::IncompressibleInitialConditions::Restart { time, .. } => {
            assert!((time - 0.02).abs() < 1e-14)
        }
        _ => panic!("not a physical restart"),
    }

    let output = std::process::Command::new(env!("CARGO_BIN_EXE_flursys"))
        .args(["project", "solve"])
        .arg(directory.path())
        .arg("--resume-latest")
        .args([
            "--dt",
            "0.007",
            "--steps",
            "2",
            "--end-time",
            "0.03",
            "--checkpoint-every",
            "0",
            "--recovery-every",
            "1",
            "--recovery-seconds",
            "0",
            "--history-max-frames",
            "2",
            "--history-max-mib",
            "1",
            "--json",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "CLI failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let result: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert!((result["run"]["physical_time"].as_f64().unwrap() - 0.03).abs() < 1e-14);
    assert!(result["restart_checkpoint"].is_string());
}

#[test]
fn load_history_frequency_and_archive_round_trip() {
    use flursys::workbench::force_analysis as loads;
    let c = case();
    let s = solve_incompressible(&c).unwrap();
    let mut report = flursys::workbench::wall_reports::reports(&c, &s.velocity, &s.pressure, false)
        .unwrap()
        .remove(0);
    report.pressure_moment = Vec3::ZERO;
    report.shear_moment = Vec3::ZERO;
    let settings = loads::Settings {
        length: 0.1,
        ..Default::default()
    };
    let mut samples = Vec::new();
    for i in 0..=400 {
        let time = i as f64 * 0.005;
        let force = Vec3::new(0., 0.5 * (std::f64::consts::TAU * 5. * time).sin(), 0.);
        report.pressure_force = force;
        report.shear_force = Vec3::ZERO;
        report.total_force = force;
        samples.push(loads::Sample {
            index: i,
            time: Some(time),
            density: 1.,
            reports: vec![report.clone()],
        });
    }
    let (f, st) = loads::shedding_frequency(&samples, &settings, 0.2)
        .unwrap()
        .unwrap();
    assert!((f - 5.).abs() < 0.01);
    assert!((st - 0.5).abs() < 0.001);
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("loads.jsonl");
    for sample in &samples {
        loads::append(&path, sample).unwrap();
    }
    loads::append(&path, samples.last().unwrap()).unwrap();
    let loaded = loads::load(&path).unwrap();
    assert_eq!(loaded.len(), samples.len());
    assert!(
        loads::csv(&loaded, &settings)
            .unwrap()
            .starts_with("index,physical_time_s,Cd")
    );
    let mut count = 0;
    loads::visit_samples(&path, |_| {
        count += 1;
        Ok(())
    })
    .unwrap();
    assert_eq!(count, samples.len() + 1);
}
#[test]
fn final_adaptive_remainder_can_be_below_minimum_step() {
    let mut c = heating();
    c.solver.time = TimeIntegration::AdaptiveBackward {
        dt: 0.01,
        steps: 10,
        controls: AdaptiveTimeStep {
            min_dt: 0.01,
            max_dt: 0.02,
            ..Default::default()
        },
    };
    c.solver.physical_end_time = Some(0.015);
    let s = solve_incompressible(&c).unwrap();
    assert_eq!(s.completed_time_steps, 2);
    assert!((s.last_time_step.unwrap() - 0.005).abs() < 1e-14);
    assert!((s.physical_time.unwrap() - 0.015).abs() < 1e-14);
}

#[test]
fn backward_checkpoint_resume_matches_uninterrupted_time_levels() {
    use flursys::workbench::checkpoint;
    let mut full = case();
    full.solver.thermal = heating().solver.thermal;
    full.solver.time = TimeIntegration::Backward { dt: 0.01, steps: 4 };
    let uninterrupted = solve_incompressible(&full).unwrap();
    let mut split = full.clone();
    split.solver.time = TimeIntegration::Backward { dt: 0.01, steps: 2 };
    let first = solve_incompressible(&split).unwrap();
    assert!(first.previous.is_some());
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("checkpoint.json");
    checkpoint::write_solution(
        &path,
        &checkpoint::MeshSignature::new(&split.mesh),
        2,
        &first,
        "BDF2",
    )
    .unwrap();
    assert!(checkpoint::inspect(&path).unwrap().backward_history);
    split.initial_conditions = checkpoint::load(&path, &split.mesh).unwrap().1;
    let resumed = solve_incompressible(&split).unwrap();
    assert!((resumed.physical_time.unwrap() - 0.04).abs() < 1e-14);
    for (a, b) in uninterrupted
        .velocity
        .values()
        .iter()
        .zip(resumed.velocity.values())
    {
        assert!((*a - *b).norm() < 1e-11, "{a:?} != {b:?}");
    }
    for (a, b) in uninterrupted
        .pressure
        .values()
        .iter()
        .zip(resumed.pressure.values())
    {
        assert!((a - b).abs() < 1e-11);
    }
    for (a, b) in uninterrupted
        .temperature
        .as_ref()
        .unwrap()
        .values()
        .iter()
        .zip(resumed.temperature.as_ref().unwrap().values())
    {
        assert!((a - b).abs() < 1e-11);
    }
    for (a, b) in uninterrupted
        .face_flux
        .values()
        .iter()
        .zip(resumed.face_flux.values())
    {
        assert!((a - b).abs() < 1e-11);
    }
}
