use flursys::workbench::{case_format, catalogue, WorkbenchSession};
use flursys::{GmshMesher, MeshDimension, Vec3};

const CASES: [&str; 6] = [
    "square-cylinder-re100",
    "sphere-re100",
    "turbulent-flat-plate",
    "turbulent-channel",
    "naca0012-re6000000",
    "cylinder-ogrid-re100",
];

#[test]
fn benchmark_recipes_preserve_domains_load_axes_and_recording() {
    let cases = catalogue::builtin_templates().unwrap();
    for id in CASES {
        let case = cases.iter().find(|c| c.id == id).unwrap();
        for preset in ["coarse", "medium", "fine"] {
            let project = case_format::instantiate(case, Some(preset)).unwrap();
            let session = WorkbenchSession::from_project(&project).unwrap();
            let (export, _) = session.mesh_generation_inputs().unwrap();
            let geo = export.document.to_geo_string().unwrap();
            assert!(geo.contains("Physical"), "{id}/{preset}");
            let forces = &project.solver.force_analysis;
            forces.validate().unwrap();
            assert!(!forces.patches.is_empty(), "{id}");
            if id == "sphere-re100" {
                assert_eq!(project.mesh.dimension, MeshDimension::ThreeD);
                assert!(geo.contains("Volume(1) = {1,2}"));
                assert_eq!(project.plane_monitors.len(), 1);
                assert!((forces.measure - std::f64::consts::PI / 4.).abs() < 1e-12);
            }
            if id == "cylinder-ogrid-re100" {
                assert_eq!(
                    project.mesh_recipe.tuning.method,
                    flursys::workbench::MeshMethod::OGrid
                );
                assert!(geo.contains("Transfinite Surface"));
            }
            if id == "naca0012-re6000000" {
                let angle = 10_f64.to_radians();
                assert!((forces.drag_direction.x - angle.cos()).abs() < 1e-12);
                assert!((forces.lift_direction.x + angle.sin()).abs() < 1e-12);
            }
            if id == "square-cylinder-re100" {
                let settings = &project.solver.scripting;
                let binding = &settings.bindings["inlet"][0];
                let script = settings.compiled(binding).unwrap();
                let startup = script
                    .value(binding, Vec3::new(0., 2., 0.), Some(0.25))
                    .unwrap();
                let settled = script
                    .value(binding, Vec3::new(0., 2., 0.), Some(5.25))
                    .unwrap();
                assert!((startup.x - 0.75).abs() < 1e-12);
                assert!((startup.y - 0.00075).abs() < 1e-12);
                assert_eq!(settled, Vec3::new(0.75, 0., 0.));
            }
        }
    }
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn benchmark_meshes_preserve_patches_and_exclude_the_sphere_volume() {
    let cases = catalogue::builtin_templates().unwrap();
    for id in CASES {
        let case = cases.iter().find(|c| c.id == id).unwrap();
        let project = case_format::instantiate(case, Some("coarse")).unwrap();
        let mut session = WorkbenchSession::from_project(&project).unwrap();
        let (export, options) = session.mesh_generation_inputs().unwrap();
        let generated = GmshMesher::auto()
            .generate(&export.document, &options)
            .unwrap_or_else(|e| panic!("{id}: {e}"));
        assert!(generated.mesh.cell_count() > 0, "{id}");
        if id == "sphere-re100" {
            let centre = Vec3::new(5., 5., 5.);
            assert!(generated
                .mesh
                .cells()
                .iter()
                .all(|c| (c.center - centre).norm() > 0.49));
            let volume: f64 = generated.mesh.cells().iter().map(|c| c.volume).sum();
            let expected = 2500. - std::f64::consts::PI / 6.;
            assert!(
                (volume - expected).abs() < 0.02,
                "sphere fluid volume {volume}"
            );
        }
        eprintln!("{id}: {} cells", generated.mesh.cell_count());
        session.install_mesh(generated);
        for selection in session.named_selections().iter() {
            assert!(
                session.patch_names().contains(&selection.name),
                "{id}: {}",
                selection.name
            );
            assert!(session.boundary_assignment(&selection.name).is_some());
        }
    }
}

#[test]
fn advanced_reference_cases_preserve_operating_points_and_comparison_assets() {
    let cases = catalogue::builtin_templates().unwrap();
    for id in [
        "periodic-hill-re2800",
        "cdv-nozzle",
        "ahmed-body-25deg",
        "supersonic-wedge-mach2",
        "rayleigh-benard-ra10000",
    ] {
        let case = cases.iter().find(|c| c.id == id).unwrap();
        assert!(!case.metadata.sources.is_empty());
        assert!(!case.metadata.comparisons.is_empty());
        for preset in &case.metadata.presets {
            let p = case_format::instantiate(case, Some(&preset.id)).unwrap();
            let s = WorkbenchSession::from_project(&p).unwrap();
            let (export, _) = s.mesh_generation_inputs().unwrap();
            assert!(export
                .document
                .to_geo_string()
                .unwrap()
                .contains("Physical"));
            if let Some(settings) = &preset.compressible {
                assert_eq!(p.solver.compressible.as_ref(), Some(settings));
            }
        }
        if id == "cdv-nozzle" {
            for (id, ratio) in [
                ("subsonic", 0.89),
                ("normal-shock", 0.75),
                ("supersonic", 0.16),
            ] {
                let p = case_format::instantiate(case, Some(id)).unwrap();
                let settings = p.solver.compressible.unwrap();
                let flursys::compressible::Boundary::TotalInlet { pressure: pt, .. } =
                    settings.boundaries["inlet"]
                else {
                    panic!("Reservoir inlet missing")
                };
                let flursys::compressible::Boundary::PressureOutlet { pressure: pb } =
                    settings.boundaries["outlet"]
                else {
                    panic!("Pressure outlet missing")
                };
                assert!((pb / pt - ratio).abs() < 1e-12);
            }
        }
    }
}
