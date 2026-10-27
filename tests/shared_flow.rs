use flursys::{
    compressible as gas, BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, Point,
    UnstructuredMesh, Vec3,
};

fn case() -> (UnstructuredMesh, gas::Settings) {
    let mesh = UnstructuredMesh::from_cells(
        MeshDimension::TwoD,
        vec![
            Point::new(0., 0., 0.),
            Point::new(1., 0., 0.),
            Point::new(2., 0., 0.),
            Point::new(0., 1., 0.),
            Point::new(1., 1., 0.),
            Point::new(2., 1., 0.),
        ],
        vec![
            CellDefinition::polygon(vec![0, 1, 4, 3]),
            CellDefinition::polygon(vec![1, 2, 5, 4]),
        ],
    )
    .unwrap();
    let faces = mesh
        .faces()
        .iter()
        .enumerate()
        .filter(|(_, f)| f.neighbour.is_none())
        .map(|(i, _)| i)
        .collect();
    let mesh = mesh
        .with_boundary_patches(vec![BoundaryPatch {
            name: "exterior".into(),
            face_indices: faces,
            boundary_type: BoundaryType::Wall,
        }])
        .unwrap();
    let mut settings = gas::Settings::default();
    settings
        .boundaries
        .insert("exterior".into(), gas::Boundary::Transmissive);
    settings.max_steps = 6;
    settings.max_time_step = 1e-5;
    settings.compute.backend = flursys::compute::Backend::Serial;
    (mesh, settings)
}

#[test]
fn stopped_flow_resumes_with_global_time_and_identical_fields() {
    let (mesh, mut settings) = case();
    settings.heat_source = 200.;
    let full = gas::solve(&mesh, &settings).unwrap();
    let partial = gas::solve_resuming(&mesh, &settings, None, |s| {
        Ok(if s.report.accepted_steps == 2 {
            gas::StepAction::Stop
        } else {
            gas::StepAction::Continue
        })
    })
    .unwrap();
    assert!(partial.report.stopped);
    assert_eq!(partial.report.accepted_steps, 2);
    settings.max_steps = 4;
    let resumed = gas::solve_resuming(
        &mesh,
        &settings,
        Some(&partial.snapshot(settings.gas)),
        |_| Ok(gas::StepAction::Continue),
    )
    .unwrap();
    assert_eq!(resumed.report.accepted_steps, 6);
    assert_eq!(resumed.report.time, full.report.time);
    assert_eq!(resumed.fields, full.fields);
    assert_eq!(
        resumed.report.source_integrals,
        full.report.source_integrals
    );
    assert!(resumed
        .report
        .conservation_defect
        .iter()
        .all(|v| v.abs() < 1e-8));
}

#[test]
fn checkpoint_round_trip_rejects_changed_gas_and_tampering() {
    let (mesh, settings) = case();
    let state = gas::solve(&mesh, &settings).unwrap().snapshot(settings.gas);
    let folder = tempfile::tempdir().unwrap();
    let path = folder.path().join("checkpoint.json");
    let frame = flursys::workbench::flow_frame::FlowFrame::compressible("run-0001", &mesh, state);
    frame.write(&path, None).unwrap();
    let loaded = flursys::workbench::flow_frame::FlowFrame::read(&path).unwrap();
    assert_eq!(
        loaded
            .gas_restart(&mesh, settings.gas)
            .unwrap()
            .report
            .accepted_steps,
        6
    );
    let mut wrong_gas = settings.gas;
    wrong_gas.gamma = 1.3;
    assert!(loaded.gas_restart(&mesh, wrong_gas).is_err());
    assert_eq!(loaded.dataset().unwrap().mesh().cell_count(), 2);
    let text = std::fs::read_to_string(&path)
        .unwrap()
        .replace("run-0001", "run-0002");
    std::fs::write(&path, text).unwrap();
    assert!(flursys::workbench::flow_frame::FlowFrame::read(&path).is_err());
}

#[test]
fn parallel_flux_accumulation_matches_serial() {
    let (mesh, mut settings) = case();
    settings.initial.velocity = Vec3::new(3., 1., 0.);
    let serial = gas::solve(&mesh, &settings).unwrap();
    settings.compute.backend = flursys::compute::Backend::Parallel;
    settings.compute.parallel_min_rows = 1;
    settings.threads = 2;
    let parallel = gas::solve(&mesh, &settings).unwrap();
    assert_eq!(serial.fields, parallel.fields);
    assert_eq!(
        serial.report.conservation_defect,
        parallel.report.conservation_defect
    );
}

#[test]
#[ignore = "requires a working GPU adapter"]
fn gpu_fluxes_match_cpu_for_all_schemes() {
    let (mesh, mut settings) = case();
    settings.initial.velocity = Vec3::new(12., 1., 0.);
    settings.regions.push(gas::InitialRegion {
        minimum: Vec3::new(0., 0., -1.),
        maximum: Vec3::new(1., 1., 1.),
        state: gas::Primitive {
            density: 1.1,
            pressure: 95000.,
            velocity: Vec3::new(9., 0.5, 0.),
        },
    });
    for flux in [
        gas::FluxScheme::Rusanov,
        gas::FluxScheme::Hllc,
        gas::FluxScheme::CentralUpwind,
    ] {
        settings.flux = flux;
        settings.compute.backend = flursys::compute::Backend::Serial;
        let cpu = gas::solve(&mesh, &settings).unwrap();
        settings.compute.backend = flursys::compute::Backend::Gpu;
        settings.compute.gpu_min_rows = 1;
        settings.compute.cpu_fallback = false;
        let gpu = gas::solve(&mesh, &settings).unwrap();
        assert!(gpu.report.performance.actual_backend.contains("GPU"));
        for (a, b) in cpu.fields.iter().zip(&gpu.fields) {
            assert!((a.density - b.density).abs() < 1e-5);
            assert!((a.pressure - b.pressure).abs() / a.pressure < 1e-5);
            assert!((a.velocity - b.velocity).norm() < 1e-4);
        }
    }
}

#[test]
fn headless_stop_and_checkpoint_use_shared_job_lifecycle() {
    let (mesh, mut settings) = case();
    settings.max_steps = 1000000;
    settings.end_time = 100.;
    let control = flursys::headless::ExecutionControl::default();
    let folder = tempfile::tempdir().unwrap();
    let solution = flursys::headless::execute(
        flursys::flow::FlowCase::Compressible {
            mesh: mesh.clone(),
            settings: settings.clone(),
        },
        folder.path(),
        &control,
        |event| {
            if matches!(event, flursys::app::WorkbenchJobEvent::TimeStep{index,..} if *index >= 2) {
                control.stop();
            }
        },
    )
    .unwrap();
    let flursys::flow::FlowSolution::Compressible(solution) = solution else {
        panic!("Wrong solver result")
    };
    assert!(solution.report.stopped);
    assert!(solution.report.accepted_steps >= 2);
    let frame =
        flursys::workbench::flow_frame::FlowFrame::read(&folder.path().join("checkpoint.json"))
            .unwrap();
    assert_eq!(frame.step, solution.report.accepted_steps);
    assert_eq!(
        frame.gas_restart(&mesh, settings.gas).unwrap().fields,
        solution.fields
    );
}
