use flursys::{
    parse_gmsh, solve_incompressible, BoundaryPatch, BoundaryType, CellDefinition,
    IncompressibleBoundaryCondition, IncompressibleCase, IncompressibleMaterial,
    IncompressibleSolverOptions, MeshDimension, Point, UnstructuredMesh, Vec3,
};

const GMSH_CAVITY: &str = "$MeshFormat\n4.1 0 8\n$EndMeshFormat\n$PhysicalNames\n1\n1 7 \"walls\"\n$EndPhysicalNames\n$Entities\n0 1 1 0\n1 0 0 0 1 1 0 1 7 4 1 2 3 4\n1 0 0 0 1 1 0 0 1 1\n$EndEntities\n$Nodes\n1 4 10 50000\n2 1 0 4\n10 42 1001 50000\n0 0 0\n1 0 0\n1 1 0\n0 1 0\n$EndNodes\n$Elements\n2 6 1 6\n1 1 1 4\n1 10 42\n2 42 1001\n3 1001 50000\n4 50000 10\n2 1 2 2\n5 10 42 1001\n6 10 1001 50000\n$EndElements\n";

fn two_cell_mesh_with_named_patches() -> UnstructuredMesh {
    let mesh = UnstructuredMesh::from_cells(
        MeshDimension::TwoD,
        vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
        ],
        vec![
            CellDefinition::polygon(vec![0, 1, 4, 3]),
            CellDefinition::polygon(vec![1, 2, 5, 4]),
        ],
    )
    .unwrap();
    let mut patches = vec![
        BoundaryPatch {
            name: "left".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
        BoundaryPatch {
            name: "right".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
        BoundaryPatch {
            name: "bottom".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
        BoundaryPatch {
            name: "top".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
    ];
    for (index, face) in mesh.faces().iter().enumerate() {
        if face.neighbour.is_none() {
            let patch = if face.center.x == 0.0 {
                0
            } else if face.center.x == 2.0 {
                1
            } else if face.center.y == 0.0 {
                2
            } else {
                3
            };
            patches[patch].face_indices.push(index);
        }
    }
    mesh.with_boundary_patches(patches).unwrap()
}

fn four_cell_mesh_with_named_patches() -> UnstructuredMesh {
    let mesh = UnstructuredMesh::from_cells(
        MeshDimension::TwoD,
        (0..=2)
            .flat_map(|y| (0..=2).map(move |x| Point::new(x as f64, y as f64, 0.0)))
            .collect(),
        vec![
            CellDefinition::polygon(vec![0, 1, 4, 3]),
            CellDefinition::polygon(vec![1, 2, 5, 4]),
            CellDefinition::polygon(vec![3, 4, 7, 6]),
            CellDefinition::polygon(vec![4, 5, 8, 7]),
        ],
    )
    .unwrap();
    let mut patches = vec![
        BoundaryPatch {
            name: "left".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
        BoundaryPatch {
            name: "right".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
        BoundaryPatch {
            name: "bottom".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
        BoundaryPatch {
            name: "top".into(),
            face_indices: Vec::new(),
            boundary_type: BoundaryType::Wall,
        },
    ];
    for (index, face) in mesh.faces().iter().enumerate() {
        if face.neighbour.is_none() {
            let patch = if face.center.x == 0.0 {
                0
            } else if face.center.x == 2.0 {
                1
            } else if face.center.y == 0.0 {
                2
            } else {
                3
            };
            patches[patch].face_indices.push(index);
        }
    }
    mesh.with_boundary_patches(patches).unwrap()
}

fn skewed_nine_cell_cavity() -> UnstructuredMesh {
    let point = |x: usize, y: usize| x + 4 * y;
    let mesh = UnstructuredMesh::from_cells(
        MeshDimension::TwoD,
        (0..=3)
            .flat_map(|y| {
                (0..=3).map(move |x| Point::new(x as f64 + 0.2 * y as f64, y as f64, 0.0))
            })
            .collect(),
        (0..3)
            .flat_map(|y| {
                (0..3).map(move |x| {
                    CellDefinition::polygon(vec![
                        point(x, y),
                        point(x + 1, y),
                        point(x + 1, y + 1),
                        point(x, y + 1),
                    ])
                })
            })
            .collect(),
    )
    .unwrap();
    let mut patches = ["left", "right", "bottom", "top"].map(|name| BoundaryPatch {
        name: name.into(),
        face_indices: Vec::new(),
        boundary_type: BoundaryType::Wall,
    });
    for (index, face) in mesh.faces().iter().enumerate() {
        if face.neighbour.is_none() {
            let patch = if face.area_vector.x < -0.5 {
                0
            } else if face.area_vector.x > 0.5 {
                1
            } else if face.area_vector.y < 0.0 {
                2
            } else {
                3
            };
            patches[patch].face_indices.push(index);
        }
    }
    mesh.with_boundary_patches(patches.into()).unwrap()
}

fn eight_cell_hexahedral_cavity() -> UnstructuredMesh {
    let point = |x: usize, y: usize, z: usize| x + 3 * (y + 3 * z);
    let mesh = UnstructuredMesh::from_cells(
        MeshDimension::ThreeD,
        (0..=2)
            .flat_map(|z| {
                (0..=2).flat_map(move |y| {
                    (0..=2).map(move |x| Point::new(x as f64, y as f64, z as f64))
                })
            })
            .collect(),
        (0..2)
            .flat_map(|z| {
                (0..2).flat_map(move |y| {
                    (0..2).map(move |x| {
                        CellDefinition::Hexahedron([
                            point(x, y, z),
                            point(x + 1, y, z),
                            point(x + 1, y + 1, z),
                            point(x, y + 1, z),
                            point(x, y, z + 1),
                            point(x + 1, y, z + 1),
                            point(x + 1, y + 1, z + 1),
                            point(x, y + 1, z + 1),
                        ])
                    })
                })
            })
            .collect(),
    )
    .unwrap();
    let mut patches = ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"].map(|name| BoundaryPatch {
        name: name.into(),
        face_indices: Vec::new(),
        boundary_type: BoundaryType::Wall,
    });
    for (index, face) in mesh.faces().iter().enumerate() {
        if face.neighbour.is_none() {
            let patch = if face.center.x == 0.0 {
                0
            } else if face.center.x == 2.0 {
                1
            } else if face.center.y == 0.0 {
                2
            } else if face.center.y == 2.0 {
                3
            } else if face.center.z == 0.0 {
                4
            } else {
                5
            };
            patches[patch].face_indices.push(index);
        }
    }
    mesh.with_boundary_patches(patches.into()).unwrap()
}

#[test]
fn physical_boundary_resolution_builds_mesh_bound_velocity_pressure_and_flux_conditions() {
    let mesh = two_cell_mesh_with_named_patches();
    let case = IncompressibleCase::steady(
        mesh.clone(),
        vec![
            (
                "left".into(),
                IncompressibleBoundaryCondition::VelocityInlet {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
            (
                "right".into(),
                IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
            ),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "top".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(0.25, 0.0, 0.0),
                },
            ),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );

    let resolved = case.resolve_boundaries().unwrap();
    for &face in &mesh.boundary_patches()[0].face_indices {
        assert_eq!(resolved.boundary_flux()[face], -1.0);
        assert_eq!(
            resolved
                .velocity_boundary()
                .component(flursys::MomentumComponent::X)
                .condition(face),
            Some(flursys::ScalarBoundaryCondition::FixedValue(1.0))
        );
        assert_eq!(
            resolved.pressure_correction_boundary().condition(face),
            Some(flursys::ScalarBoundaryCondition::ZeroGradient)
        );
    }
    for &face in &mesh.boundary_patches()[1].face_indices {
        assert_eq!(
            resolved.pressure_boundary().condition(face),
            Some(flursys::ScalarBoundaryCondition::FixedValue(0.0))
        );
        assert_eq!(
            resolved.pressure_correction_boundary().condition(face),
            Some(flursys::ScalarBoundaryCondition::FixedValue(0.0))
        );
        assert_eq!(
            resolved
                .velocity_boundary()
                .component(flursys::MomentumComponent::X)
                .condition(face),
            Some(flursys::ScalarBoundaryCondition::ZeroGradient)
        );
    }
}

#[test]
fn high_level_closed_case_returns_finite_solution_and_a_converged_cfd_report() {
    let mesh = four_cell_mesh_with_named_patches();
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );

    let solution = solve_incompressible(&case).unwrap();
    assert!(solution.report.converged());
    assert_eq!(
        solution.report.status(),
        flursys::IncompressibleSolveStatus::Converged
    );
    assert!(solution
        .velocity
        .values()
        .iter()
        .all(|value| { value.x.is_finite() && value.y.is_finite() && value.z.is_finite() }));
    assert!(solution
        .pressure
        .values()
        .iter()
        .all(|value| value.is_finite()));
    assert!(solution
        .face_flux
        .values()
        .iter()
        .all(|value| value.is_finite()));
    assert!(solution.report.final_continuity_rms <= case.solver.continuity_absolute_tolerance);
}

#[test]
fn patch_flux_sums_the_owner_oriented_boundary_faces_without_topology_reconstruction() {
    let mesh = two_cell_mesh_with_named_patches();
    let flux = flursys::FaceField::from_faces(&mesh, |index, _| index as f64 + 0.5);
    let left = flursys::patch_flux(&mesh, &flux, "left").unwrap();
    let expected: f64 = mesh.boundary_patches()[0]
        .face_indices
        .iter()
        .map(|&face| flux[face])
        .sum();
    assert_eq!(left, expected);
    assert!(matches!(
        flursys::patch_flux(&mesh, &flux, "missing"),
        Err(flursys::IncompressibleCaseError::UnknownBoundaryPatch { .. })
    ));
}

#[test]
fn high_level_velocity_inlet_pressure_outlet_channel_converges_with_balanced_boundary_flow() {
    let mesh = four_cell_mesh_with_named_patches();
    let solver = IncompressibleSolverOptions {
        max_outer_iterations: 500,
        continuity_absolute_tolerance: 1.0e-9,
        ..IncompressibleSolverOptions::default()
    };
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            (
                "left".into(),
                IncompressibleBoundaryCondition::VelocityInlet {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
            (
                "right".into(),
                IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
            ),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        solver,
    );

    let solution = solve_incompressible(&case).unwrap();
    assert!(solution.report.converged());
    let inlet = flursys::patch_flux(&case.mesh, &solution.face_flux, "left").unwrap();
    let outlet = flursys::patch_flux(&case.mesh, &solution.face_flux, "right").unwrap();
    assert!((inlet + outlet).abs() <= 1.0e-8);
    assert!(outlet > 0.0);
    assert!((solution.report.net_boundary_flux).abs() <= 1.0e-8);
    assert!(solution.report.total_inflow < 0.0);
    assert!(solution.report.total_outflow > 0.0);
    let left_pressure = (solution.pressure[0] + solution.pressure[2]) * 0.5;
    let right_pressure = (solution.pressure[1] + solution.pressure[3]) * 0.5;
    assert!(left_pressure > right_pressure);
}

#[test]
fn supplied_initial_fields_are_mesh_safe_and_initialize_the_high_level_solution() {
    let mesh = four_cell_mesh_with_named_patches();
    let foreign = four_cell_mesh_with_named_patches();
    let case = IncompressibleCase::steady(
        mesh.clone(),
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    )
    .with_initial_conditions(flursys::IncompressibleInitialConditions::fields(
        flursys::CellField::filled(&foreign, Vec3::ZERO),
        flursys::CellField::filled(&mesh, 0.0),
    ));

    assert!(matches!(
        solve_incompressible(&case),
        Err(flursys::IncompressibleSolveError::Case(
            flursys::IncompressibleCaseError::Field(flursys::FieldError::MeshMismatch { .. })
        ))
    ));
}

#[test]
fn supplied_initial_pressure_field_rejects_a_foreign_mesh() {
    let mesh = four_cell_mesh_with_named_patches();
    let foreign = four_cell_mesh_with_named_patches();
    let case = IncompressibleCase::steady(
        mesh.clone(),
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    )
    .with_initial_conditions(flursys::IncompressibleInitialConditions::fields(
        flursys::CellField::filled(&mesh, Vec3::ZERO),
        flursys::CellField::filled(&foreign, 0.0),
    ));
    assert!(matches!(
        solve_incompressible(&case),
        Err(flursys::IncompressibleSolveError::Case(
            flursys::IncompressibleCaseError::Field(flursys::FieldError::MeshMismatch { .. })
        ))
    ));
}

#[test]
fn high_level_lid_driven_cavity_preserves_impermeable_wall_fluxes() {
    let mesh = four_cell_mesh_with_named_patches();
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "top".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );

    let solution = solve_incompressible(&case).unwrap();
    assert!(solution.report.converged());
    assert!(solution
        .velocity
        .values()
        .iter()
        .any(|velocity| velocity.x > 1.0e-8));
    for patch in ["left", "right", "bottom", "top"] {
        assert!(
            flursys::patch_flux(&case.mesh, &solution.face_flux, patch)
                .unwrap()
                .abs()
                <= 1.0e-12
        );
    }
}

#[test]
fn solution_speed_is_mesh_bound_and_matches_velocity_magnitude() {
    let mesh = four_cell_mesh_with_named_patches();
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    let solution = solve_incompressible(&case).unwrap();
    let speed = solution.speed();
    assert_eq!(speed.mesh_id(), solution.velocity.mesh_id());
    for (speed, velocity) in speed.values().iter().zip(solution.velocity.values()) {
        assert_eq!(*speed, velocity.norm());
    }
}

#[test]
fn unstructured_vtk_export_writes_named_cell_fields_for_a_high_level_solution() {
    let mesh = four_cell_mesh_with_named_patches();
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    let solution = solve_incompressible(&case).unwrap();
    let path = std::env::temp_dir().join(format!(
        "flursys-unstructured-{}-{}.vtk",
        std::process::id(),
        solution.report.outer_iterations
    ));
    flursys::output::write_unstructured_legacy_vtk(&path, "test", &case.mesh, &solution).unwrap();
    let contents = std::fs::read_to_string(&path).unwrap();
    std::fs::remove_file(&path).unwrap();
    assert!(contents.contains("DATASET UNSTRUCTURED_GRID"));
    assert!(contents.contains("CELL_DATA 4"));
    assert!(contents.contains("SCALARS pressure double 1"));
    assert!(contents.contains("SCALARS velocity_magnitude double 1"));
    assert!(contents.contains("VECTORS velocity double"));
}

#[test]
fn gmsh_imported_case_solves_and_exports_through_the_public_workflow() {
    let mesh = parse_gmsh(GMSH_CAVITY).unwrap();
    let case = IncompressibleCase::steady(
        mesh,
        vec![(
            "walls".into(),
            IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
        )],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    let solution = solve_incompressible(&case).unwrap();
    let path = std::env::temp_dir().join(format!(
        "flursys-gmsh-incompressible-{}-{}.vtk",
        std::process::id(),
        solution.report.outer_iterations
    ));
    flursys::output::write_unstructured_legacy_vtk(&path, "gmsh", &case.mesh, &solution).unwrap();
    let contents = std::fs::read_to_string(&path).unwrap();
    std::fs::remove_file(&path).unwrap();
    assert!(solution.report.converged());
    assert!(contents.contains("CELL_DATA 2"));
    assert!(contents.contains("VECTORS velocity double"));
}

#[test]
fn case_validation_rejects_non_finite_physical_boundary_values() {
    let base = |condition| {
        IncompressibleCase::steady(
            four_cell_mesh_with_named_patches(),
            vec![
                ("left".into(), condition),
                ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
                ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
                ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ],
            IncompressibleMaterial {
                density: 1.0,
                kinematic_viscosity: 0.1,
            },
            IncompressibleSolverOptions::default(),
        )
    };
    assert!(matches!(
        base(IncompressibleBoundaryCondition::VelocityInlet {
            velocity: Vec3::new(f64::NAN, 0.0, 0.0)
        })
        .resolve_boundaries(),
        Err(flursys::IncompressibleCaseError::InvalidVelocityBoundary { .. })
    ));
    assert!(matches!(
        base(IncompressibleBoundaryCondition::MovingWall {
            velocity: Vec3::new(0.0, f64::INFINITY, 0.0)
        })
        .resolve_boundaries(),
        Err(flursys::IncompressibleCaseError::InvalidVelocityBoundary { .. })
    ));
    assert!(matches!(
        base(IncompressibleBoundaryCondition::PressureOutlet { pressure: f64::NAN })
            .resolve_boundaries(),
        Err(flursys::IncompressibleCaseError::InvalidOutletPressure { .. })
    ));
}

#[test]
fn case_validation_rejects_non_finite_and_out_of_range_material_properties() {
    for material in [
        IncompressibleMaterial {
            density: 0.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleMaterial {
            density: f64::NAN,
            kinematic_viscosity: 0.1,
        },
        IncompressibleMaterial {
            density: f64::INFINITY,
            kinematic_viscosity: 0.1,
        },
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: -0.1,
        },
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: f64::NAN,
        },
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: f64::INFINITY,
        },
    ] {
        let case = IncompressibleCase::steady(
            four_cell_mesh_with_named_patches(),
            vec![
                ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
                ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
                ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
                ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ],
            material,
            IncompressibleSolverOptions::default(),
        );
        assert!(matches!(
            case.resolve_boundaries(),
            Err(flursys::IncompressibleCaseError::InvalidDensity { .. })
                | Err(flursys::IncompressibleCaseError::InvalidKinematicViscosity { .. })
        ));
    }
}

#[test]
fn case_validation_rejects_unknown_physical_boundary_patch() {
    let case = IncompressibleCase::steady(
        four_cell_mesh_with_named_patches(),
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "does_not_exist".into(),
                IncompressibleBoundaryCondition::NoSlipWall,
            ),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    assert!(matches!(
        case.resolve_boundaries(),
        Err(flursys::IncompressibleCaseError::Numerics(
            flursys::NumericsError::UnknownBoundaryPatch { ref patch }
        )) if patch == "does_not_exist"
    ));
}

#[test]
fn high_level_solver_propagates_a_configured_momentum_linear_failure() {
    let solver = IncompressibleSolverOptions {
        momentum_solver: flursys::LinearSolverOptions {
            max_iterations: 0,
            ..flursys::LinearSolverOptions::default()
        },
        ..IncompressibleSolverOptions::default()
    };
    let case = IncompressibleCase::steady(
        four_cell_mesh_with_named_patches(),
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "top".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        solver,
    );
    assert!(matches!(
        solve_incompressible(&case),
        Err(flursys::IncompressibleSolveError::Simple(
            flursys::SimpleError::MomentumLinearDidNotConverge
        ))
    ));
}

#[test]
fn high_level_solver_options_expose_independent_linear_solver_controls() {
    let options = IncompressibleSolverOptions::default();
    assert_eq!(
        options.momentum_solver,
        flursys::LinearSolverOptions::default()
    );
    assert_eq!(
        options.pressure_solver,
        flursys::LinearSolverOptions::default()
    );
}

#[test]
fn case_validation_rejects_duplicate_patch_conditions_before_iteration() {
    let case = IncompressibleCase::steady(
        four_cell_mesh_with_named_patches(),
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "left".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    assert!(matches!(
        case.resolve_boundaries(),
        Err(flursys::IncompressibleCaseError::Numerics(
            flursys::NumericsError::DuplicateBoundaryPatchCondition { ref patch }
        )) if patch == "left"
    ));
}

#[test]
fn case_validation_rejects_missing_boundary_patch_coverage_before_iteration() {
    let case = IncompressibleCase::steady(
        four_cell_mesh_with_named_patches(),
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    assert!(matches!(
        case.resolve_boundaries(),
        Err(flursys::IncompressibleCaseError::Numerics(
            flursys::NumericsError::MissingBoundaryCondition { .. }
        ))
    ));
}

#[test]
fn high_level_solver_returns_current_fields_with_max_iteration_status() {
    let mesh = four_cell_mesh_with_named_patches();
    let solver = IncompressibleSolverOptions {
        max_outer_iterations: 1,
        continuity_absolute_tolerance: 1.0e-30,
        ..IncompressibleSolverOptions::default()
    };
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "top".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        solver,
    );
    let solution = solve_incompressible(&case).unwrap();
    assert_eq!(
        solution.report.status(),
        flursys::IncompressibleSolveStatus::MaxIterations
    );
    assert_eq!(solution.report.outer_iterations, 1);
    assert!(solution
        .velocity
        .values()
        .iter()
        .all(|velocity| velocity.norm().is_finite()));
}

#[test]
fn high_level_pressure_outlet_backflow_is_rejected_without_inventing_velocity() {
    let case = IncompressibleCase::steady(
        four_cell_mesh_with_named_patches(),
        vec![
            (
                "left".into(),
                IncompressibleBoundaryCondition::VelocityInlet {
                    velocity: Vec3::new(-1.0, 0.0, 0.0),
                },
            ),
            (
                "right".into(),
                IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
            ),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("top".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    let result = solve_incompressible(&case);
    assert!(
        matches!(
            result,
            Err(flursys::IncompressibleSolveError::BackflowAtPressureOutlet { .. })
        ),
        "{result:?}"
    );
}

#[test]
fn high_level_skewed_lid_driven_cavity_converges_without_wall_leakage() {
    let mesh = skewed_nine_cell_cavity();
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            ("left".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("right".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("bottom".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "top".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    let solution = solve_incompressible(&case).unwrap();
    assert!(solution.report.converged());
    assert!(solution.report.net_boundary_flux.abs() <= 1.0e-12);
    assert!(solution
        .velocity
        .values()
        .iter()
        .all(|velocity| velocity.norm().is_finite()));
}

#[test]
fn high_level_three_dimensional_lid_driven_cavity_converges_with_impermeable_boundaries() {
    let mesh = eight_cell_hexahedral_cavity();
    let case = IncompressibleCase::steady(
        mesh,
        vec![
            ("xmin".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("xmax".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("ymin".into(), IncompressibleBoundaryCondition::NoSlipWall),
            (
                "ymax".into(),
                IncompressibleBoundaryCondition::MovingWall {
                    velocity: Vec3::new(1.0, 0.0, 0.0),
                },
            ),
            ("zmin".into(), IncompressibleBoundaryCondition::NoSlipWall),
            ("zmax".into(), IncompressibleBoundaryCondition::NoSlipWall),
        ],
        IncompressibleMaterial {
            density: 1.0,
            kinematic_viscosity: 0.1,
        },
        IncompressibleSolverOptions::default(),
    );
    let solution = solve_incompressible(&case).unwrap();
    assert!(solution.report.converged());
    assert!(solution
        .velocity
        .values()
        .iter()
        .all(|value| value.norm().is_finite()));
    for patch in ["xmin", "xmax", "ymin", "ymax", "zmin", "zmax"] {
        assert!(
            flursys::patch_flux(&case.mesh, &solution.face_flux, patch)
                .unwrap()
                .abs()
                <= 1.0e-12
        );
    }
}
