use flursys::{
    solve_incompressible, GmshGeoDocument, GmshMeshOptions, GmshMesher,
    IncompressibleBoundaryCondition, IncompressibleCase, IncompressibleMaterial,
    IncompressibleSolverOptions, MeshDimension, MeshingError, Vec3,
};

#[test]
fn rectangle_geo_is_deterministic_and_preserves_physical_group_names() {
    let geo = GmshGeoDocument::rectangle(
        2.0,
        1.0,
        [
            ("inlet", vec![4]),
            ("outlet", vec![2]),
            ("walls", vec![1, 3]),
        ],
        "fluid",
    )
    .unwrap();

    assert_eq!(geo.dimension(), MeshDimension::TwoD);
    assert_eq!(geo.to_geo_string().unwrap(), geo.to_geo_string().unwrap());
    let text = geo.to_geo_string().unwrap();
    assert!(text.contains("Physical Curve(\"inlet\") = {4};"));
    assert!(text.contains("Physical Curve(\"outlet\") = {2};"));
    assert!(text.contains("Physical Curve(\"walls\") = {1, 3};"));
    assert!(text.contains("Physical Surface(\"fluid\") = {1};"));
}

#[test]
fn rectangle_with_circle_geo_preserves_the_cylinder_boundary() {
    let geo = GmshGeoDocument::rectangle_with_circle(4.0, 2.0, 2.0, 1.0, 0.25).unwrap();
    let text = geo.to_geo_string().unwrap();
    assert_eq!(geo.dimension(), MeshDimension::TwoD);
    assert!(text.contains("Curve Loop(2) = {-8, -7, -6, -5};"));
    assert!(text.contains("Plane Surface(1) = {1, 2};"));
    assert!(text.contains("Physical Curve(\"cylinder\") = {5, 6, 7, 8};"));
}

#[test]
fn box_geo_is_deterministic_and_groups_all_exterior_surfaces() {
    let geo = GmshGeoDocument::rectangular_box(
        2.0,
        1.0,
        0.5,
        [
            ("inlet", vec![1]),
            ("outlet", vec![2]),
            ("walls", vec![3, 4, 5, 6]),
        ],
        "fluid",
    )
    .unwrap();

    assert_eq!(geo.dimension(), MeshDimension::ThreeD);
    let text = geo.to_geo_string().unwrap();
    assert!(text.contains("SetFactory(\"OpenCASCADE\");"));
    assert!(text.contains(
        "Box(1) = {0, 0, 0, 2.00000000000000000, 1.00000000000000000, 0.50000000000000000};"
    ));
    assert!(text.contains("Physical Surface(\"walls\") = {3, 4, 5, 6};"));
    assert!(text.contains("Physical Volume(\"fluid\") = {1};"));
}

#[test]
fn mesh_options_validate_sizes_and_build_dimension_specific_msh4_commands() {
    let two_d = GmshMeshOptions::two_d(0.25).unwrap();
    assert_eq!(
        GmshMesher::auto()
            .command_arguments("case.geo", "case.msh", &two_d)
            .unwrap(),
        vec![
            "case.geo",
            "-2",
            "-format",
            "msh4",
            "-setnumber",
            "Mesh.Binary",
            "0",
            "-o",
            "case.msh",
            "-clscale",
            "1",
            "-clmin",
            "0.25",
            "-clmax",
            "0.25",
            "-order",
            "1",
            "-nt",
            "1",
            "-setnumber",
            "Mesh.MaxNumThreads1D",
            "1",
            "-setnumber",
            "Mesh.MaxNumThreads2D",
            "1",
            "-setnumber",
            "Mesh.MaxNumThreads3D",
            "1",
        ],
    );

    let three_d = GmshMeshOptions::three_d(0.5).unwrap();
    assert!(GmshMesher::auto()
        .command_arguments("case.geo", "case.msh", &three_d)
        .unwrap()
        .contains(&"-3".to_owned()));
    let spaced = GmshMesher::auto()
        .command_arguments(
            "directory with spaces/case.geo",
            "directory with spaces/case.msh",
            &two_d,
        )
        .unwrap();
    assert_eq!(spaced[0], "directory with spaces/case.geo");
    assert_eq!(spaced[8], "directory with spaces/case.msh");

    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert!(matches!(
            GmshMeshOptions::two_d(value),
            Err(MeshingError::InvalidOptions { .. })
        ));
    }
}

#[test]
fn missing_custom_gmsh_executable_returns_a_structured_error() {
    let mesher = GmshMesher::from_executable("/definitely/not/a/gmsh-executable");
    assert!(matches!(
        mesher.version(),
        Err(MeshingError::GmshExecutableNotFound { .. })
    ));
}

#[cfg(unix)]
#[test]
fn failing_custom_gmsh_executable_preserves_process_diagnostics() {
    let mesher = GmshMesher::from_executable("/bin/false");
    assert!(matches!(
        mesher.version(),
        Err(MeshingError::GmshProcessFailed {
            status: Some(1),
            ..
        })
    ));
}

#[cfg(unix)]
#[test]
fn successful_process_without_a_mesh_returns_missing_output_error() {
    use std::os::unix::fs::PermissionsExt;

    let temporary = tempfile::NamedTempFile::new().unwrap();
    std::fs::write(
        temporary.path(),
        "#!/bin/sh\nif [ \"$1\" = \"--version\" ]; then echo 4.15.2; fi\nexit 0\n",
    )
    .unwrap();
    let mut permissions = std::fs::metadata(temporary.path()).unwrap().permissions();
    permissions.set_mode(0o700);
    std::fs::set_permissions(temporary.path(), permissions).unwrap();
    let script = temporary.into_temp_path();
    let geometry =
        GmshGeoDocument::rectangle(1.0, 1.0, [("walls", vec![1, 2, 3, 4])], "fluid").unwrap();

    assert!(matches!(
        GmshMesher::from_executable(&script)
            .generate(&geometry, &GmshMeshOptions::two_d(0.25).unwrap()),
        Err(MeshingError::MissingOutputMesh { .. })
    ));
}

#[cfg(unix)]
#[test]
fn verbose_gmsh_process_does_not_block_on_full_stdout_or_stderr_pipes() {
    use std::os::unix::fs::PermissionsExt;

    let directory = tempfile::tempdir().unwrap();
    let executable = directory.path().join("verbose-gmsh");
    std::fs::write(
        &executable,
        r#"#!/bin/sh
if [ "$1" = "--version" ]; then printf '4.15.2\n'; exit 0; fi
i=0
while [ "$i" -lt 5000 ]; do
  printf 'Gmsh generated diagnostic output line 1234567890\n'
  printf 'Gmsh generated error output line 1234567890\n' >&2
  i=$((i + 1))
done
exit 1
"#,
    )
    .unwrap();
    let mut permissions = std::fs::metadata(&executable).unwrap().permissions();
    permissions.set_mode(0o700);
    std::fs::set_permissions(&executable, permissions).unwrap();
    let geometry =
        GmshGeoDocument::rectangle(1.0, 1.0, [("walls", vec![1, 2, 3, 4])], "fluid").unwrap();

    let error = GmshMesher::from_executable(&executable)
        .generate(&geometry, &GmshMeshOptions::two_d(0.25).unwrap())
        .unwrap_err();
    assert!(
        matches!(error, MeshingError::GmshProcessFailed { stdout, stderr, .. }
        if stdout.len() > 65536 && stderr.len() > 65536)
    );
}

#[cfg(unix)]
#[test]
fn cancelling_meshing_reaps_its_running_child() {
    use std::os::unix::fs::PermissionsExt;
    use std::sync::atomic::{AtomicUsize, Ordering};

    let directory = tempfile::tempdir().unwrap();
    let executable = directory.path().join("running-gmsh");
    std::fs::write(
        &executable,
        r#"#!/bin/sh
if [ "$1" = "--version" ]; then printf '4.15.2\n'; exit 0; fi
while :; do :; done
"#,
    )
    .unwrap();
    let mut permissions = std::fs::metadata(&executable).unwrap().permissions();
    permissions.set_mode(0o700);
    std::fs::set_permissions(&executable, permissions).unwrap();
    let geometry =
        GmshGeoDocument::rectangle(1.0, 1.0, [("walls", vec![1, 2, 3, 4])], "fluid").unwrap();
    let polls = AtomicUsize::new(0);
    assert!(matches!(
        GmshMesher::from_executable(&executable).generate_cancellable(
            &geometry,
            &GmshMeshOptions::two_d(0.25).unwrap(),
            || polls.fetch_add(1, Ordering::Relaxed) >= 2
        ),
        Err(MeshingError::Cancelled)
    ));
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn real_gmsh_generates_a_2d_rectangle_with_ascii_msh4_physical_patches() {
    let geometry = GmshGeoDocument::rectangle(
        2.0,
        1.0,
        [
            ("inlet", vec![4]),
            ("outlet", vec![2]),
            ("walls", vec![1, 3]),
        ],
        "fluid",
    )
    .unwrap();
    let generated = GmshMesher::auto()
        .generate(&geometry, &GmshMeshOptions::two_d(0.25).unwrap())
        .unwrap();
    assert_generated_mesh(
        &generated.mesh,
        &generated.report.mesh_format,
        MeshDimension::TwoD,
        &["inlet", "outlet", "walls"],
    );
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn real_gmsh_finer_rectangle_has_more_cells_and_repeated_generation_has_no_stale_state() {
    let geometry = GmshGeoDocument::rectangle(
        2.0,
        1.0,
        [
            ("inlet", vec![4]),
            ("outlet", vec![2]),
            ("walls", vec![1, 3]),
        ],
        "fluid",
    )
    .unwrap();
    let mesher = GmshMesher::auto();
    let coarse = mesher
        .generate(&geometry, &GmshMeshOptions::two_d(0.5).unwrap())
        .unwrap();
    let fine = mesher
        .generate(&geometry, &GmshMeshOptions::two_d(0.1).unwrap())
        .unwrap();
    assert!(fine.mesh.cell_count() > coarse.mesh.cell_count());
    assert_eq!(coarse.report.patch_count, fine.report.patch_count);
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn real_gmsh_generates_a_2d_rectangle_with_a_cylinder_patch() {
    let geometry = GmshGeoDocument::rectangle_with_circle(4.0, 2.0, 2.0, 1.0, 0.25).unwrap();
    let generated = GmshMesher::auto()
        .generate(&geometry, &GmshMeshOptions::two_d(0.25).unwrap())
        .unwrap();
    assert_generated_mesh(
        &generated.mesh,
        &generated.report.mesh_format,
        MeshDimension::TwoD,
        &["inlet", "outlet", "walls", "cylinder"],
    );
    assert!(generated
        .mesh
        .boundary_patches()
        .iter()
        .any(|patch| patch.name == "cylinder" && !patch.face_indices.is_empty()));
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn real_gmsh_generates_a_3d_box_with_ascii_msh4_physical_patches() {
    let geometry = GmshGeoDocument::rectangular_box(
        2.0,
        1.0,
        0.5,
        [
            ("inlet", vec![1]),
            ("outlet", vec![2]),
            ("walls", vec![3, 4, 5, 6]),
        ],
        "fluid",
    )
    .unwrap();
    let generated = GmshMesher::auto()
        .generate(&geometry, &GmshMeshOptions::three_d(0.5).unwrap())
        .unwrap();
    assert_generated_mesh(
        &generated.mesh,
        &generated.report.mesh_format,
        MeshDimension::ThreeD,
        &["inlet", "outlet", "walls"],
    );
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn generated_gmsh_cavity_solves_through_the_high_level_incompressible_api() {
    let geometry = GmshGeoDocument::rectangle(
        1.0,
        1.0,
        [
            ("bottom", vec![1]),
            ("right", vec![2]),
            ("top", vec![3]),
            ("left", vec![4]),
        ],
        "fluid",
    )
    .unwrap();
    let generated = GmshMesher::auto()
        .generate(&geometry, &GmshMeshOptions::two_d(0.5).unwrap())
        .unwrap();
    let case = IncompressibleCase::steady(
        generated.mesh,
        vec![
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "top".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions {
            max_outer_iterations: 500,
            continuity_absolute_tolerance: 1.0e-8,
            ..IncompressibleSolverOptions::default()
        },
    );
    let solution = solve_incompressible(&case).unwrap();
    assert!(solution.report.converged());
    assert!(solution
        .velocity
        .values()
        .iter()
        .all(|value| value.x.is_finite()));
}

fn assert_generated_mesh(
    mesh: &flursys::UnstructuredMesh,
    mesh_format: &str,
    dimension: MeshDimension,
    names: &[&str],
) {
    let patches = mesh
        .boundary_patches()
        .iter()
        .map(|patch| patch.name.as_str())
        .collect::<Vec<_>>();
    assert_eq!(mesh.dimension(), dimension);
    assert!(mesh_format.starts_with("4."));
    assert!(!mesh.points().is_empty());
    assert!(mesh.cell_count() > 0);
    for name in names {
        assert!(patches.contains(name));
    }
}

#[test]
fn meshing_thread_selection_reaches_each_gmsh_dimension_limit() {
    let mut options = GmshMeshOptions::two_d(0.1).unwrap();
    for threads in [2, 0] {
        options.threads = threads;
        let arguments = GmshMesher::auto()
            .command_arguments("case.geo", "case.msh", &options)
            .unwrap();
        let expected = if threads == 0 {
            std::thread::available_parallelism()
                .map(|n| n.get())
                .unwrap_or(1)
        } else {
            threads
        }
        .to_string();
        assert!(arguments
            .windows(2)
            .any(|a| a[0] == "-nt" && a[1] == expected));
        for dimension in ["1D", "2D", "3D"] {
            assert!(arguments.windows(3).any(|a| a[0] == "-setnumber"
                && a[1] == format!("Mesh.MaxNumThreads{dimension}")
                && a[2] == expected));
        }
    }
}

#[cfg(unix)]
#[test]
fn meshing_thread_limit_is_passed_to_the_child_openmp_environment() {
    use std::os::unix::fs::PermissionsExt;
    let directory = tempfile::tempdir().unwrap();
    let executable = directory.path().join("gmsh-env");
    std::fs::write(&executable, "#!/bin/sh\nif [ \"$1\" = \"--version\" ]; then echo 4.15.2; exit 0; fi\nprintf 'workers=%s limit=%s' \"$OMP_NUM_THREADS\" \"$OMP_THREAD_LIMIT\" >&2\nexit 1\n").unwrap();
    std::fs::set_permissions(&executable, std::fs::Permissions::from_mode(0o700)).unwrap();
    let geometry =
        GmshGeoDocument::rectangle(1.0, 1.0, [("walls", vec![1, 2, 3, 4])], "fluid").unwrap();
    let mut options = GmshMeshOptions::two_d(0.25).unwrap();
    options.threads = 2;
    assert!(
        matches!(GmshMesher::from_executable(&executable).generate(&geometry,&options),
        Err(MeshingError::GmshProcessFailed {stderr,..}) if stderr.contains("workers=2 limit=2"))
    );
}

#[test]
#[ignore = "requires a real gmsh executable on PATH"]
fn real_gmsh_multithreaded_generation_reports_monotonic_stages() {
    let geometry =
        GmshGeoDocument::rectangle(1.0, 1.0, [("walls", vec![1, 2, 3, 4])], "fluid").unwrap();
    let mut options = GmshMeshOptions::two_d(0.1).unwrap();
    options.threads = 2;
    let mut progress = Vec::new();
    let generated = GmshMesher::auto()
        .generate_with_progress(
            &geometry,
            &options,
            || false,
            |fraction, stage| progress.push((fraction, stage.to_owned())),
        )
        .unwrap();
    assert!(generated.mesh.cell_count() > 0);
    assert_eq!(progress.first().unwrap().0, 0.0);
    assert_eq!(progress.last().unwrap().0, 1.0);
    assert!(progress.windows(2).all(|p| p[0].0 <= p[1].0));
    assert!(progress
        .iter()
        .any(|(_, stage)| stage.contains("validating")));
}

#[cfg(unix)]
#[test]
fn streamed_progress_handles_partial_lines_and_does_not_complete_a_failed_mesh() {
    use std::os::unix::fs::PermissionsExt;
    let directory = tempfile::tempdir().unwrap();
    let executable = directory.path().join("gmsh-progress");
    std::fs::write(&executable,"#!/bin/sh\nif [ \"$1\" = \"--version\" ]; then echo 4.15.2; exit 0; fi\nprintf 'Info : Meshing 1'\nsleep 0.05\nprintf 'D\\n'\nsleep 0.05\nprintf 'Info : Done meshing 1D\\nInfo : Meshing 2D\\n'\nsleep 0.05\nexit 1\n").unwrap();
    std::fs::set_permissions(&executable, std::fs::Permissions::from_mode(0o700)).unwrap();
    let geometry =
        GmshGeoDocument::rectangle(1.0, 1.0, [("walls", vec![1, 2, 3, 4])], "fluid").unwrap();
    let mut events = Vec::new();
    let result = GmshMesher::from_executable(&executable).generate_with_progress(
        &geometry,
        &GmshMeshOptions::two_d(0.25).unwrap(),
        || false,
        |fraction, stage| events.push((fraction, stage.to_owned())),
    );
    assert!(matches!(
        result,
        Err(MeshingError::GmshProcessFailed { .. })
    ));
    assert!(events.iter().any(|(_, s)| s == "Meshing edges"));
    assert!(events.iter().any(|(_, s)| s == "Meshing surfaces"));
    assert!(events.iter().all(|(f, _)| *f < 1.0));
}
