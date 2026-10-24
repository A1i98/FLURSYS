//! DFG 2D-2 geometry/inlet recipe; pressure-outlet approximation is explicit.
use super::*;
use crate::{IncompressibleBoundaryCondition as B, MeshDimension, Vec3};
pub fn session() -> Result<WorkbenchSession, String> {
    let mut session = WorkbenchSession::new();
    let (rect, hole) = session
        .add_rectangle_with_circle(2.2, 0.41, Vec3::new(0.2, 0.2, 0.0), 0.05)
        .map_err(|e| e.to_string())?;
    let mut group = |name: &str, edges: Vec<crate::EdgeId>, bc| -> Result<(), String> {
        session
            .create_named_selection(
                name,
                edges
                    .into_iter()
                    .map(GeometrySelectionTarget::Edge)
                    .collect(),
            )
            .map_err(|e| e.to_string())?;
        session
            .configure_named_boundary(name, bc)
            .map_err(|e| e.to_string())
    };
    group(
        "inlet",
        vec![rect.left],
        B::VelocityInlet {
            velocity: Vec3::new(1.0, 0., 0.),
        },
    )?;
    group(
        "outlet",
        vec![rect.right],
        B::PressureOutlet { pressure: 0.0 },
    )?;
    group("walls", vec![rect.top, rect.bottom], B::NoSlipWall)?;
    group("cylinder", hole.boundary.to_vec(), B::NoSlipWall)?;
    session
        .set_mesh_configuration(MeshDimension::TwoD, 0.02, 0.002, 0.02, 1)
        .map_err(|e| e.to_string())?;
    session.set_mesh_recipe(MeshRecipe {
        refinements: vec![ThresholdRefinement {
            id: 1,
            name: "cylinder refinement".into(),
            targets: vec![MeshControlTarget::NamedSelection("cylinder".into())],
            sampling: 128,
            size_min: 0.002,
            size_max: 0.02,
            distance_min: 0.02,
            distance_max: 0.2,
            enabled: true,
        }],
        ..Default::default()
    });
    session
        .set_material(1.0, 0.001)
        .map_err(|e| e.to_string())?;
    session
        .set_solver_controls(200, 0.7, 0.3, 1e-9)
        .map_err(|e| e.to_string())?;
    session
        .set_flow_models(
            crate::unstructured_incompressible::TimeIntegration::Backward {
                dt: 0.0025,
                steps: 12000,
            },
            Default::default(),
        )
        .map_err(|e| e.to_string())?;
    session
        .set_physical_end_time(Some(30.0))
        .map_err(|e| e.to_string())?;
    session
        .set_pressure_coupling(crate::simple::PressureCoupling {
            algorithm: crate::simple::CouplingAlgorithm::Pimple,
            pressure_correctors: 2,
            ..Default::default()
        })
        .map_err(|e| e.to_string())?;
    session.set_convection(crate::momentum::MomentumConvection::LimitedLinear);
    let mut scripts = crate::scripting::Settings::default();
    scripts.scripts.insert(
        "dfg-inlet".into(),
        crate::scripting::Script {
            source: include_str!("../../examples/scripting/parabolic-inlet.rhai").into(),
            time_dependent: false,
        },
    );
    scripts.bindings.insert(
        "inlet".into(),
        vec![crate::scripting::Binding {
            script: "dfg-inlet".into(),
            function: "profile".into(),
            quantity: crate::scripting::Quantity::Velocity,
            parameters: [("height".into(), 0.41), ("peak_speed".into(), 1.5)].into(),
        }],
    );
    session.set_scripting(scripts)?;
    session.set_force_analysis(force_analysis::Settings {
        patches: vec!["cylinder".into()],
        measure: 0.1,
        length: 0.1,
        center: Vec3::new(0.2, 0.2, 0.),
        ..Default::default()
    })?;
    Ok(session)
}
