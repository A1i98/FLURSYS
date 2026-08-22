use flursys::{build_example, verify_solution, ExampleProjectId, GmshMesher, MeshQualityMetric};

fn mesh_example(id: ExampleProjectId) -> flursys::WorkbenchSession {
    let mut session = build_example(id).expect("example factory");
    let (export, options) = session
        .mesh_generation_inputs()
        .expect("production mesh inputs");
    let generated = GmshMesher::auto()
        .generate(&export.document, &options)
        .expect("real Gmsh mesh generation");
    session.install_mesh(generated);
    session
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn every_production_example_generates_a_real_mesh_with_named_patches() {
    for id in [
        ExampleProjectId::LidDrivenCavity2D,
        ExampleProjectId::LaminarChannel2D,
        ExampleProjectId::CylinderFlow2D,
        ExampleProjectId::SkewedMeshVerification2D,
        ExampleProjectId::Channel3D,
    ] {
        let session = mesh_example(id);
        let mesh = &session.mesh().expect("installed mesh").mesh;
        assert!(mesh.cell_count() > 0, "{id:?}");
        assert!(!mesh.points().is_empty(), "{id:?}");
        for selection in session.named_selections().iter() {
            assert!(
                session.patch_names().contains(&selection.name),
                "{id:?}: {}",
                selection.name
            );
            assert!(session.boundary_assignment(&selection.name).is_some());
        }
        let cache = session.mesh_render_cache().expect("quality cache");
        assert!(cache
            .quality()
            .values(MeshQualityMetric::AspectRatio)
            .iter()
            .all(|value| value.is_finite()));
    }
}

#[test]
#[ignore = "requires a real gmsh executable and solves production examples"]
fn channel_showcase_uses_the_full_production_pipeline() {
    let mut session = mesh_example(ExampleProjectId::LaminarChannel2D);
    session
        .readiness()
        .expect("all example patches are configured");
    let case = session.prepare_case().expect("production case");
    let solution = flursys::solve_incompressible(&case).expect("production SIMPLE solve");
    let report = verify_solution(ExampleProjectId::LaminarChannel2D, &solution);
    assert!(report.finite_fields);
    assert!(report
        .checks
        .iter()
        .any(|check| check.name == "mass balance" && check.passed));
    assert!(report
        .checks
        .iter()
        .any(|check| check.name == "streamwise flow" && check.passed));
    session.complete_solve(Ok(solution));
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("examples").join("channel.vtk");
    session.export_vtk(&path).expect("VTK export");
    let text = std::fs::read_to_string(path).unwrap();
    assert!(text.contains("SCALARS pressure"));
    assert!(text.contains("VECTORS velocity"));
}

#[test]
#[ignore = "requires a real gmsh executable and solves production examples"]
fn every_production_flow_example_solves_with_finite_fields() {
    for id in [
        ExampleProjectId::LidDrivenCavity2D,
        ExampleProjectId::CylinderFlow2D,
        ExampleProjectId::SkewedMeshVerification2D,
        ExampleProjectId::Channel3D,
    ] {
        let session = mesh_example(id);
        session.readiness().expect("configured production case");
        let solution = flursys::solve_incompressible(&session.prepare_case().unwrap())
            .unwrap_or_else(|error| panic!("{id:?}: {error}"));
        let report = verify_solution(id, &solution);
        assert!(report.finite_fields, "{id:?}");
        assert!(
            report
                .checks
                .iter()
                .any(|check| check.name == "mass balance" && check.passed),
            "{id:?}"
        );
    }
}
