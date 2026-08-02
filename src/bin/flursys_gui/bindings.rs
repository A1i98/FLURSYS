use super::{case_name, parse_number, MainWindow};
use flursys::cases::{BackwardStepCase, CavityCase, ChannelCase, CylinderCase};
use flursys::{
    BoundaryConditionKind, BoundaryFace, BuoyancyModel, EnergyModel, GeometryFeatureKind,
    GeometryPart, GeometryPartKind, GeometrySketch, Project, ProjectCase, ProjectCoupling,
    ProjectPressureSolver, SketchPlane, SketchProfileKind, ThermalBoundaryCondition,
};
use slint::SharedString;

pub(super) fn sync_project_from_ui(ui: &MainWindow, project: &mut Project) {
    project.name = ui.get_project_name().to_string();
    project.solver.nx = ui.get_nx().max(4) as usize;
    project.solver.ny = ui.get_ny().max(4) as usize;
    project.solver.dt = parse_number(ui.get_dt_text().as_str(), project.solver.dt);
    project.solver.max_iterations = ui.get_iterations().max(1) as usize;
    project.solver.threads = ui.get_threads().max(0) as usize;
    project.solver.gui_update_every = ui.get_gui_update_every().max(1) as usize;
    project.solver.history_every = ui.get_history_every().max(1) as usize;
    project.solver.frame_every = ui.get_frame_every().max(1) as usize;
    project.solver.coupling = if ui.get_coupling_index() == 0 {
        ProjectCoupling::Simple
    } else {
        ProjectCoupling::Projection
    };
    project.solver.pressure_solver = if ui.get_pressure_solver_index() == 0 {
        ProjectPressureSolver::Pcg
    } else {
        ProjectPressureSolver::Sor
    };
    project.solver.velocity_relaxation = parse_number(
        ui.get_velocity_relaxation().as_str(),
        project.solver.velocity_relaxation,
    );
    project.solver.pressure_relaxation = parse_number(
        ui.get_pressure_relaxation().as_str(),
        project.solver.pressure_relaxation,
    );
    project.preprocessing.geometry.extrusion_depth = parse_number(
        ui.get_extrusion_depth().as_str(),
        project.preprocessing.geometry.extrusion_depth,
    );
    project.preprocessing.mesh.cells_z = ui.get_mesh_nz().max(1) as usize;
    project.physics.thermal.model = if ui.get_energy_model_index() == 1 {
        EnergyModel::ConstantProperties
    } else {
        EnergyModel::Off
    };
    project.physics.thermal.initial_temperature = parse_number(
        ui.get_initial_temperature().as_str(),
        project.physics.thermal.initial_temperature,
    );
    project.physics.thermal.thermal_diffusivity = parse_number(
        ui.get_thermal_diffusivity().as_str(),
        project.physics.thermal.thermal_diffusivity,
    );
    project.physics.thermal.source_temperature_rate = parse_number(
        ui.get_thermal_source().as_str(),
        project.physics.thermal.source_temperature_rate,
    );
    project.physics.buoyancy = if ui.get_buoyancy_model_index() == 1 {
        BuoyancyModel::Boussinesq {
            reference_temperature: parse_number(
                ui.get_reference_temperature().as_str(),
                project.physics.thermal.initial_temperature,
            ),
            thermal_expansion: parse_number(ui.get_thermal_expansion().as_str(), 0.0),
            gravity_x: parse_number(ui.get_gravity_x().as_str(), 0.0),
            gravity_y: parse_number(ui.get_gravity_y().as_str(), -9.81),
        }
    } else {
        BuoyancyModel::Off
    };
}

pub(super) fn write_project_to_ui(ui: &MainWindow, project: &Project) {
    ui.set_project_loaded(true);
    ui.set_project_name(SharedString::from(project.name.as_str()));
    ui.set_case_name(SharedString::from(case_name(&project.case)));
    ui.set_case_index(project_case_index(&project.case));
    ui.set_nx(project.solver.nx as i32);
    ui.set_ny(project.solver.ny as i32);
    ui.set_dt_text(SharedString::from(format!("{:.6}", project.solver.dt)));
    ui.set_iterations(project.solver.max_iterations as i32);
    ui.set_threads(project.solver.threads as i32);
    ui.set_gui_update_every(project.solver.gui_update_every as i32);
    ui.set_history_every(project.solver.history_every as i32);
    ui.set_frame_every(project.solver.frame_every as i32);
    ui.set_coupling_index(match project.solver.coupling {
        ProjectCoupling::Simple => 0,
        ProjectCoupling::Projection => 1,
    });
    ui.set_pressure_solver_index(match project.solver.pressure_solver {
        ProjectPressureSolver::Pcg => 0,
        ProjectPressureSolver::Sor => 1,
    });
    ui.set_velocity_relaxation(SharedString::from(format!(
        "{:.3}",
        project.solver.velocity_relaxation
    )));
    ui.set_pressure_relaxation(SharedString::from(format!(
        "{:.3}",
        project.solver.pressure_relaxation
    )));
    ui.set_extrusion_depth(SharedString::from(format!(
        "{:.3}",
        project.preprocessing.geometry.extrusion_depth
    )));
    ui.set_mesh_nz(project.preprocessing.mesh.cells_z as i32);
    ui.set_energy_model_index(match project.physics.thermal.model {
        EnergyModel::Off => 0,
        EnergyModel::ConstantProperties => 1,
    });
    ui.set_initial_temperature(SharedString::from(format!(
        "{:.4}",
        project.physics.thermal.initial_temperature
    )));
    ui.set_thermal_diffusivity(SharedString::from(format!(
        "{:.6e}",
        project.physics.thermal.thermal_diffusivity
    )));
    ui.set_thermal_source(SharedString::from(format!(
        "{:.6e}",
        project.physics.thermal.source_temperature_rate
    )));
    write_thermal_boundary_to_ui(ui, project, 0);
    match project.physics.buoyancy {
        BuoyancyModel::Off => ui.set_buoyancy_model_index(0),
        BuoyancyModel::Boussinesq {
            reference_temperature,
            thermal_expansion,
            gravity_x,
            gravity_y,
        } => {
            ui.set_buoyancy_model_index(1);
            ui.set_reference_temperature(SharedString::from(format!("{reference_temperature:.4}")));
            ui.set_thermal_expansion(SharedString::from(format!("{thermal_expansion:.6e}")));
            ui.set_gravity_x(SharedString::from(format!("{gravity_x:.4}")));
            ui.set_gravity_y(SharedString::from(format!("{gravity_y:.4}")));
        }
    }
    ui.set_geometry_parts_summary(SharedString::from(geometry_parts_summary(project)));
    ui.set_part_name(SharedString::from(format!(
        "Part {}",
        project.preprocessing.geometry.parts.len() + 1
    )));
    ui.set_sketch_name(SharedString::from(format!(
        "Sketch {}",
        project.preprocessing.geometry.sketches.len() + 1
    )));
    ui.set_feature_name(SharedString::from(format!(
        "Extrude {}",
        project.preprocessing.geometry.features.len() + 1
    )));
    write_boundary_to_ui(ui, project, BoundaryFace::Left);
}

pub(super) fn sketch_from_ui(ui: &MainWindow) -> Result<GeometrySketch, String> {
    let sketch_name = ui.get_sketch_name().trim().to_string();
    if sketch_name.is_empty() {
        return Err("sketch name cannot be empty".to_string());
    }
    let size_x = parse_positive(ui.get_sketch_size_x().as_str(), "profile width/radius")?;
    let size_y = parse_positive(ui.get_sketch_size_y().as_str(), "profile height")?;
    let profile = if ui.get_sketch_kind_index() == 1 {
        SketchProfileKind::Circle { radius: size_x }
    } else {
        SketchProfileKind::Rectangle {
            width: size_x,
            height: size_y,
        }
    };
    let sketch = GeometrySketch::from_profile(
        sketch_name,
        SketchPlane::Xy,
        profile,
        parse_finite(ui.get_sketch_pos_x().as_str(), "sketch origin X")?,
        parse_finite(ui.get_sketch_pos_y().as_str(), "sketch origin Y")?,
        parse_finite(ui.get_sketch_pos_z().as_str(), "sketch origin Z")?,
    );
    Ok(sketch)
}

pub(super) fn feature_from_ui(ui: &MainWindow) -> Result<(String, GeometryFeatureKind), String> {
    let feature_name = ui.get_feature_name().trim().to_string();
    if feature_name.is_empty() {
        return Err("feature name cannot be empty".to_string());
    }
    let feature = if ui.get_feature_kind_index() == 1 {
        GeometryFeatureKind::Revolve {
            axis_offset: parse_positive(
                ui.get_revolve_axis_offset().as_str(),
                "revolve axis offset",
            )?,
            angle_degrees: 360.0,
        }
    } else {
        GeometryFeatureKind::Extrude {
            depth: parse_positive(ui.get_feature_depth().as_str(), "extrude depth")?,
        }
    };
    Ok((feature_name, feature))
}

pub(super) fn geometry_part_from_ui(ui: &MainWindow) -> Result<GeometryPart, String> {
    let name = ui.get_part_name().trim().to_string();
    if name.is_empty() {
        return Err("3D part name cannot be empty".to_string());
    }
    let x_size = parse_positive(ui.get_part_size_x().as_str(), "X/radius")?;
    let y_size = parse_positive(ui.get_part_size_y().as_str(), "Y")?;
    let z_size = parse_positive(ui.get_part_size_z().as_str(), "Z/height")?;
    let x = parse_finite(ui.get_part_pos_x().as_str(), "position X")?;
    let y = parse_finite(ui.get_part_pos_y().as_str(), "position Y")?;
    let z = parse_finite(ui.get_part_pos_z().as_str(), "position Z")?;
    let kind = match ui.get_part_kind_index() {
        1 => GeometryPartKind::Cylinder {
            radius: x_size,
            height: z_size,
            segments: 32,
        },
        2 => GeometryPartKind::Cone {
            radius: x_size,
            height: z_size,
            segments: 32,
        },
        3 => GeometryPartKind::Sphere {
            radius: x_size,
            segments: 24,
        },
        4 => GeometryPartKind::Torus {
            major_radius: x_size,
            minor_radius: y_size,
            segments: 32,
        },
        _ => GeometryPartKind::Box {
            length: x_size,
            width: y_size,
            height: z_size,
        },
    };
    Ok(GeometryPart {
        name,
        kind,
        x,
        y,
        z,
    })
}

pub(super) fn parse_positive(value: &str, label: &str) -> Result<f64, String> {
    let value = parse_finite(value, label)?;
    if value > 0.0 {
        Ok(value)
    } else {
        Err(format!("{label} must be positive"))
    }
}

pub(super) fn parse_finite(value: &str, label: &str) -> Result<f64, String> {
    value
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| format!("{label} must be a finite number"))
}

pub(super) fn geometry_parts_summary(project: &Project) -> String {
    let parts = &project.preprocessing.geometry.parts;
    let design = format!(
        "{} sketches · {} features",
        project.preprocessing.geometry.sketches.len(),
        project.preprocessing.geometry.features.len()
    );
    match parts.len() {
        0 => format!("{design} · no custom solids."),
        1 => format!("{design} · 1 solid: {}", parts[0].summary()),
        count => format!(
            "{design} · {count} solids · latest: {}",
            parts[count - 1].summary()
        ),
    }
}

pub(super) fn geometry_model_tree(project: &Project) -> String {
    let geometry = &project.preprocessing.geometry;
    let mut lines = vec!["▾ Geometry".to_string()];

    lines.push(format!("   ▾ Sketches ({})", geometry.sketches.len()));
    if geometry.sketches.is_empty() {
        lines.push("      — None".to_string());
    } else {
        for sketch in &geometry.sketches {
            lines.push(format!(
                "      ◇ {} [{}]",
                sketch.name,
                sketch.plane.label()
            ));
        }
    }

    lines.push(format!("   ▾ Features ({})", geometry.features.len()));
    if geometry.features.is_empty() {
        lines.push("      — None".to_string());
    } else {
        for feature in &geometry.features {
            lines.push(format!(
                "      ◈ {} [{}]",
                feature.name,
                feature.kind.label()
            ));
        }
    }

    let regions = geometry
        .features
        .iter()
        .flat_map(|feature| feature.regions.iter());
    let regions: Vec<_> = regions.collect();
    lines.push(format!("   ▾ Regions ({})", regions.len()));
    if regions.is_empty() {
        lines.push("      — None".to_string());
    } else {
        for region in regions {
            lines.push(format!("      ◌ {} [{}]", region.name, region.kind.label()));
        }
    }

    lines.push(format!("   ▾ Solids ({})", geometry.parts.len()));
    if geometry.parts.is_empty() {
        lines.push("      — None".to_string());
    } else {
        for part in &geometry.parts {
            lines.push(format!("      ◆ {} [{}]", part.name, part.kind.label()));
        }
    }
    lines.join("\n")
}

pub(super) fn project_case_from_index(index: i32) -> ProjectCase {
    match index {
        1 => ProjectCase::from(CylinderCase::default()),
        2 => ProjectCase::from(BackwardStepCase::default()),
        3 => ProjectCase::from(ChannelCase::default()),
        _ => ProjectCase::from(CavityCase::default()),
    }
}

pub(super) fn project_case_index(case: &ProjectCase) -> i32 {
    match case {
        ProjectCase::LidDrivenCavity { .. } => 0,
        ProjectCase::Cylinder { .. } => 1,
        ProjectCase::BackwardFacingStep { .. } => 2,
        ProjectCase::Channel { .. } => 3,
    }
}

pub(super) fn write_boundary_to_ui(ui: &MainWindow, project: &Project, face: BoundaryFace) {
    ui.set_boundary_face_index(boundary_face_index(face));
    let Some(boundary) = project.preprocessing.boundary(face) else {
        return;
    };
    let (kind_index, value) = match &boundary.kind {
        BoundaryConditionKind::CaseDefault => (0, 0.0),
        BoundaryConditionKind::Velocity { u, .. } => (1, *u),
        BoundaryConditionKind::PressureOutlet { pressure } => (2, *pressure),
        BoundaryConditionKind::Wall { u, .. } => (3, *u),
        BoundaryConditionKind::Symmetry => (4, 0.0),
    };
    ui.set_boundary_kind_index(kind_index);
    ui.set_boundary_value(SharedString::from(format!("{value:.6}")));
}

pub(super) fn write_thermal_boundary_to_ui(ui: &MainWindow, project: &Project, face_index: i32) {
    ui.set_thermal_face_index(face_index);
    let boundary = match face_index {
        1 => project.physics.thermal.right,
        2 => project.physics.thermal.bottom,
        3 => project.physics.thermal.top,
        _ => project.physics.thermal.left,
    };
    match boundary {
        ThermalBoundaryCondition::Adiabatic => {
            ui.set_thermal_boundary_kind_index(0);
            ui.set_thermal_boundary_value(SharedString::from(format!(
                "{:.4}",
                project.physics.thermal.initial_temperature
            )));
        }
        ThermalBoundaryCondition::FixedTemperature { temperature } => {
            ui.set_thermal_boundary_kind_index(1);
            ui.set_thermal_boundary_value(SharedString::from(format!("{temperature:.4}")));
        }
    }
}

pub(super) fn boundary_face_from_index(index: i32) -> BoundaryFace {
    match index {
        1 => BoundaryFace::Right,
        2 => BoundaryFace::Bottom,
        3 => BoundaryFace::Top,
        4 => BoundaryFace::Front,
        5 => BoundaryFace::Back,
        _ => BoundaryFace::Left,
    }
}

pub(super) fn boundary_face_index(face: BoundaryFace) -> i32 {
    match face {
        BoundaryFace::Left => 0,
        BoundaryFace::Right => 1,
        BoundaryFace::Bottom => 2,
        BoundaryFace::Top => 3,
        BoundaryFace::Front => 4,
        BoundaryFace::Back => 5,
    }
}
