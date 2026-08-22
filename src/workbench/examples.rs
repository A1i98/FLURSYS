//! Production-backed example projects for the Workbench gallery.
//!
//! Examples construct the same stable geometry, Named Selections, meshing
//! inputs, physical boundaries, and solver controls available to every user.

use super::{GeometrySelectionTarget, WorkbenchError, WorkbenchSession};
use crate::{IncompressibleBoundaryCondition, IncompressibleSolution, MeshDimension, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ExampleProjectId {
    Blank,
    LidDrivenCavity2D,
    LaminarChannel2D,
    CylinderFlow2D,
    SkewedMeshVerification2D,
    Channel3D,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExampleProjectDescriptor {
    pub id: ExampleProjectId,
    pub title: &'static str,
    pub short_description: &'static str,
    pub category: &'static str,
    pub dimension: MeshDimension,
    pub difficulty: &'static str,
    pub capabilities: &'static [&'static str],
    pub expected_behavior: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExampleExpectations {
    pub closed_domain: bool,
    pub expected_positive_streamwise_flow: bool,
    pub expects_curved_boundary: bool,
    pub expects_non_ideal_quality: bool,
    pub notes: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExampleCheck {
    pub name: &'static str,
    pub passed: bool,
    pub measured_value: f64,
    pub expected_condition: &'static str,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExampleVerificationReport {
    pub converged: bool,
    pub finite_fields: bool,
    pub mass_balance: f64,
    pub checks: Vec<ExampleCheck>,
}

#[derive(Debug)]
pub enum ExampleProjectError {
    Workbench(WorkbenchError),
}

impl std::fmt::Display for ExampleProjectError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Workbench(error) => write!(formatter, "example construction failed: {error}"),
        }
    }
}

impl std::error::Error for ExampleProjectError {}

impl From<WorkbenchError> for ExampleProjectError {
    fn from(value: WorkbenchError) -> Self {
        Self::Workbench(value)
    }
}

const DESCRIPTORS: [ExampleProjectDescriptor; 6] = [
    ExampleProjectDescriptor {
        id: ExampleProjectId::Blank,
        title: "Blank Project",
        short_description: "Start with an empty editable Workbench project.",
        category: "New Project",
        dimension: MeshDimension::TwoD,
        difficulty: "Foundation",
        capabilities: &["Editable", "2D / 3D"],
        expected_behavior: "Create geometry, selections, mesh settings, and boundaries from scratch.",
    },
    ExampleProjectDescriptor {
        id: ExampleProjectId::LidDrivenCavity2D,
        title: "Lid-Driven Cavity",
        short_description: "Low-Re closed cavity with one moving wall.",
        category: "Internal Flow",
        dimension: MeshDimension::TwoD,
        difficulty: "Foundation",
        capabilities: &["2D", "Moving Wall", "Gmsh", "Unstructured"],
        expected_behavior: "Closed-domain flux remains near zero while the top wall drives recirculation.",
    },
    ExampleProjectDescriptor {
        id: ExampleProjectId::LaminarChannel2D,
        title: "Laminar Channel",
        short_description: "Open low-Re channel with velocity inlet and pressure outlet.",
        category: "Internal Flow",
        dimension: MeshDimension::TwoD,
        difficulty: "Foundation",
        capabilities: &["2D", "Pressure Outlet", "Gmsh", "Mesh Quality"],
        expected_behavior: "Streamwise flow is positive with a small inlet/outlet mass imbalance.",
    },
    ExampleProjectDescriptor {
        id: ExampleProjectId::CylinderFlow2D,
        title: "Flow Around Cylinder",
        short_description: "Low-Re external flow around a circular hole.",
        category: "External Flow",
        dimension: MeshDimension::TwoD,
        difficulty: "Intermediate",
        capabilities: &["2D", "Curved Geometry", "Gmsh", "Unstructured", "Mesh Quality"],
        expected_behavior: "A curved no-slip boundary produces a steady downstream velocity deficit.",
    },
    ExampleProjectDescriptor {
        id: ExampleProjectId::SkewedMeshVerification2D,
        title: "Skewed-Mesh Verification",
        short_description: "Curved low-Re channel for non-ideal mesh-quality inspection.",
        category: "Verification",
        dimension: MeshDimension::TwoD,
        difficulty: "Intermediate",
        capabilities: &["2D", "Curved Geometry", "Non-orthogonality", "SIMPLE"],
        expected_behavior: "Quality metrics are finite and nontrivial while the production flow path remains usable.",
    },
    ExampleProjectDescriptor {
        id: ExampleProjectId::Channel3D,
        title: "3D Channel",
        short_description: "Small rectangular volume channel through the 3D Gmsh path.",
        category: "3D",
        dimension: MeshDimension::ThreeD,
        difficulty: "Intermediate",
        capabilities: &["3D", "Volume Mesh", "Pressure Outlet", "U / V / W"],
        expected_behavior: "Finite three-component velocity and pressure fields on a real volume mesh.",
    },
];

pub fn example_descriptors() -> &'static [ExampleProjectDescriptor] {
    &DESCRIPTORS
}

pub fn descriptor(id: ExampleProjectId) -> &'static ExampleProjectDescriptor {
    DESCRIPTORS
        .iter()
        .find(|descriptor| descriptor.id == id)
        .expect("all typed example IDs have descriptors")
}

pub fn expectations(id: ExampleProjectId) -> ExampleExpectations {
    match id {
        ExampleProjectId::Blank => ExampleExpectations {
            closed_domain: false,
            expected_positive_streamwise_flow: false,
            expects_curved_boundary: false,
            expects_non_ideal_quality: false,
            notes: "No physical case is configured.",
        },
        ExampleProjectId::LidDrivenCavity2D => ExampleExpectations {
            closed_domain: true,
            expected_positive_streamwise_flow: false,
            expects_curved_boundary: false,
            expects_non_ideal_quality: false,
            notes: "Use the closed-domain pressure reference and check wall leakage.",
        },
        ExampleProjectId::LaminarChannel2D => open_flow_expectations(false, false),
        ExampleProjectId::CylinderFlow2D => open_flow_expectations(true, false),
        ExampleProjectId::SkewedMeshVerification2D => open_flow_expectations(true, true),
        ExampleProjectId::Channel3D => open_flow_expectations(false, false),
    }
}

const fn open_flow_expectations(curved: bool, non_ideal: bool) -> ExampleExpectations {
    ExampleExpectations {
        closed_domain: false,
        expected_positive_streamwise_flow: true,
        expects_curved_boundary: curved,
        expects_non_ideal_quality: non_ideal,
        notes: "Check finite fields and boundary mass balance using the production solver report.",
    }
}

pub fn build_example(id: ExampleProjectId) -> Result<WorkbenchSession, ExampleProjectError> {
    match id {
        ExampleProjectId::Blank => Ok(WorkbenchSession::new()),
        ExampleProjectId::LidDrivenCavity2D => build_cavity(),
        ExampleProjectId::LaminarChannel2D => build_channel(),
        ExampleProjectId::CylinderFlow2D => build_cylinder(false),
        ExampleProjectId::SkewedMeshVerification2D => build_cylinder(true),
        ExampleProjectId::Channel3D => build_channel_3d(),
    }
}

fn configure_2d(session: &mut WorkbenchSession, size: f64) -> Result<(), ExampleProjectError> {
    session.set_mesh_configuration(MeshDimension::TwoD, size, size * 0.5, size, 1)?;
    session.set_material(1.0, 0.1)?;
    session.set_solver_controls(400, 0.7, 0.3, 1.0e-8)?;
    Ok(())
}

fn edge_targets(edges: &[crate::EdgeId]) -> Vec<GeometrySelectionTarget> {
    edges
        .iter()
        .copied()
        .map(GeometrySelectionTarget::Edge)
        .collect()
}

fn build_cavity() -> Result<WorkbenchSession, ExampleProjectError> {
    let mut session = WorkbenchSession::new();
    let rectangle = session.add_rectangle(1.0, 1.0)?;
    session.create_named_selection("top", edge_targets(&[rectangle.top]))?;
    session.create_named_selection("bottom", edge_targets(&[rectangle.bottom]))?;
    session.create_named_selection("left", edge_targets(&[rectangle.left]))?;
    session.create_named_selection("right", edge_targets(&[rectangle.right]))?;
    session.configure_named_boundary(
        "top",
        IncompressibleBoundaryCondition::MovingWall {
            velocity: Vec3::new(1.0, 0.0, 0.0),
        },
    )?;
    for name in ["bottom", "left", "right"] {
        session.configure_named_boundary(name, IncompressibleBoundaryCondition::NoSlipWall)?;
    }
    configure_2d(&mut session, 0.12)?;
    Ok(session)
}

fn build_channel() -> Result<WorkbenchSession, ExampleProjectError> {
    let mut session = WorkbenchSession::new();
    let rectangle = session.add_rectangle(4.0, 1.0)?;
    configure_open_2d_boundaries(&mut session, &rectangle, 0.1)?;
    configure_2d(&mut session, 0.2)?;
    Ok(session)
}

fn configure_open_2d_boundaries(
    session: &mut WorkbenchSession,
    rectangle: &crate::RectangleEntities,
    inlet_velocity: f64,
) -> Result<(), ExampleProjectError> {
    session.create_named_selection("inlet", edge_targets(&[rectangle.left]))?;
    session.create_named_selection("outlet", edge_targets(&[rectangle.right]))?;
    session.create_named_selection("top_wall", edge_targets(&[rectangle.top]))?;
    session.create_named_selection("bottom_wall", edge_targets(&[rectangle.bottom]))?;
    session.configure_named_boundary(
        "inlet",
        IncompressibleBoundaryCondition::VelocityInlet {
            velocity: Vec3::new(inlet_velocity, 0.0, 0.0),
        },
    )?;
    session.configure_named_boundary(
        "outlet",
        IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
    )?;
    for name in ["top_wall", "bottom_wall"] {
        session.configure_named_boundary(name, IncompressibleBoundaryCondition::NoSlipWall)?;
    }
    Ok(())
}

fn build_cylinder(skewed: bool) -> Result<WorkbenchSession, ExampleProjectError> {
    let mut session = WorkbenchSession::new();
    let (width, height, center, radius, size) = if skewed {
        (4.0, 1.5, Vec3::new(1.55, 0.52, 0.0), 0.28, 0.16)
    } else {
        (6.0, 2.0, Vec3::new(2.0, 1.0, 0.0), 0.35, 0.2)
    };
    let (rectangle, hole) = session.add_rectangle_with_circle(width, height, center, radius)?;
    configure_open_2d_boundaries(&mut session, &rectangle, 0.1)?;
    session.create_named_selection("cylinder", edge_targets(&hole.boundary))?;
    session.configure_named_boundary("cylinder", IncompressibleBoundaryCondition::NoSlipWall)?;
    configure_2d(&mut session, size)?;
    Ok(session)
}

fn build_channel_3d() -> Result<WorkbenchSession, ExampleProjectError> {
    let mut session = WorkbenchSession::new();
    let box_entities = session.add_box(2.0, 0.6, 0.5)?;
    session.create_named_selection(
        "inlet",
        vec![GeometrySelectionTarget::Face(box_entities.x_min)],
    )?;
    session.create_named_selection(
        "outlet",
        vec![GeometrySelectionTarget::Face(box_entities.x_max)],
    )?;
    session.create_named_selection(
        "walls",
        [
            box_entities.y_min,
            box_entities.y_max,
            box_entities.z_min,
            box_entities.z_max,
        ]
        .into_iter()
        .map(GeometrySelectionTarget::Face)
        .collect(),
    )?;
    session.configure_named_boundary(
        "inlet",
        IncompressibleBoundaryCondition::VelocityInlet {
            velocity: Vec3::new(0.05, 0.0, 0.0),
        },
    )?;
    session.configure_named_boundary(
        "outlet",
        IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
    )?;
    session.configure_named_boundary("walls", IncompressibleBoundaryCondition::NoSlipWall)?;
    session.set_mesh_configuration(MeshDimension::ThreeD, 0.2, 0.1, 0.2, 1)?;
    session.set_material(1.0, 0.1)?;
    session.set_solver_controls(300, 0.6, 0.25, 1.0e-8)?;
    Ok(session)
}

/// Produces conservative, backend-derived checks without embedding numerical
/// reference data or changing solver behavior.
pub fn verify_solution(
    id: ExampleProjectId,
    solution: &IncompressibleSolution,
) -> ExampleVerificationReport {
    let finite_fields = solution
        .velocity
        .values()
        .iter()
        .all(|value| value.x.is_finite() && value.y.is_finite() && value.z.is_finite())
        && solution
            .pressure
            .values()
            .iter()
            .all(|value| value.is_finite());
    let mass_balance = solution.report.net_boundary_flux.abs();
    let converged = solution.report.converged();
    let mut checks = vec![
        ExampleCheck {
            name: "finite fields",
            passed: finite_fields,
            measured_value: if finite_fields { 1.0 } else { 0.0 },
            expected_condition: "all velocity and pressure values are finite",
        },
        ExampleCheck {
            name: "mass balance",
            passed: mass_balance <= 1.0e-6,
            measured_value: mass_balance,
            expected_condition: "absolute net boundary flux <= 1e-6",
        },
    ];
    if expectations(id).expected_positive_streamwise_flow {
        let mean_x = solution
            .velocity
            .values()
            .iter()
            .map(|value| value.x)
            .sum::<f64>()
            / solution.velocity.values().len().max(1) as f64;
        checks.push(ExampleCheck {
            name: "streamwise flow",
            passed: mean_x > 0.0,
            measured_value: mean_x,
            expected_condition: "mean x velocity > 0",
        });
    }
    ExampleVerificationReport {
        converged,
        finite_fields,
        mass_balance,
        checks,
    }
}
