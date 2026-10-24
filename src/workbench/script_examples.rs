//! Embedded scripting recipes shared by the desktop gallery and CLI catalogue.
use super::{manifest, CaseTemplateManifest};
use crate::{
    scripting::{Binding, Quantity, Script, Settings},
    thermal::{Boundary, Options},
    unstructured_incompressible::{IncompressibleBoundaryCondition, TimeIntegration},
    workbench::examples::{build_example, ExampleProjectId},
    Vec3,
};

struct Recipe {
    id: &'static str,
    title: &'static str,
    description: &'static str,
    patch: &'static str,
    quantity: Quantity,
    timed: bool,
    source: &'static str,
    parameters: &'static [(&'static str, f64)],
    notes: &'static str,
}
const RECIPES: &[Recipe] = &[
    Recipe {
        id: "scripted-inlet-parabolic", title: "Rhai · spatial velocity inlet",
        description: "Steady channel with a face-wise parabolic inlet written in Rhai.",
        patch: "inlet", quantity: Quantity::Velocity, timed: false,
        source: include_str!("../../examples/scripting/parabolic-inlet.rhai"),
        parameters: &[("height",1.0),("peak_speed",0.15)],
        notes: "Physical condition: Velocity inlet. Script output: vec3(Ux,Uy,Uz), m/s. y spans 0..1 m. The peak is 0.15 m/s and the continuous profile mean is 0.10 m/s; discrete face sampling may differ. Change height if geometry changes.",
    },
    Recipe {
        id: "scripted-inlet-pulsating", title: "Rhai · pulsating velocity inlet",
        description: "Transient channel with a sinusoidal inlet velocity and editable frequency/amplitude.",
        patch: "inlet", quantity: Quantity::Velocity, timed: true,
        source: include_str!("../../examples/scripting/pulsating-inlet.rhai"),
        parameters: &[("base_speed",0.1),("amplitude",0.02),("frequency",1.0)],
        notes: "Physical condition: Velocity inlet. u=0.10+0.02 sin(2*pi*t) m/s. t is physical time, never the nonlinear iteration. Increase the physical step count to observe more cycles.",
    },
    Recipe {
        id: "scripted-pressure-outlet", title: "Rhai · outlet pressure ramp",
        description: "Transient channel with scripted static kinematic outlet pressure.",
        patch: "outlet", quantity: Quantity::Pressure, timed: true,
        source: include_str!("../../examples/scripting/pressure-ramp.rhai"),
        parameters: &[("initial_pressure",0.0),("final_pressure",0.002),("ramp_time",0.5)],
        notes: "Physical condition: strict Pressure outlet. Ramp p/rho from 0 to 0.002 m²/s² over 0.5 s, then hold. This is not pressure in Pa, total pressure or p_rgh. The normal strict-outlet backflow check remains active.",
    },
    Recipe {
        id: "scripted-backflow-outlet", title: "Rhai · pressure outlet with backflow",
        description: "Scripted static pressure with an explicitly configured incoming backflow velocity.",
        patch: "outlet", quantity: Quantity::Pressure, timed: true,
        source: include_str!("../../examples/scripting/backflow-pressure.rhai"),
        parameters: &[("reference_pressure",0.0),("amplitude",0.0005),("frequency",1.0)],
        notes: "Physical condition: Pressure outlet with explicit backflow velocity (0,0,0) m/s. Only pressure is scripted. Incoming faces use the configured velocity; outgoing faces use the existing outlet treatment. This channel does not intentionally generate flow reversal.",
    },
    Recipe {
        id: "scripted-inlet-temperature", title: "Rhai · spatial inlet temperature",
        description: "Steady energy-transport channel with a nonuniform fixed inlet temperature.",
        patch: "inlet", quantity: Quantity::Temperature, timed: false,
        source: include_str!("../../examples/scripting/inlet-temperature.rhai"),
        parameters: &[("height",1.0),("base_temperature",293.15),("temperature_rise",5.0)],
        notes: "Thermal condition: Fixed temperature. Script returns Kelvin, from 293.15 K at the edges to 298.15 K at the centre. Flow inlet stays constant; walls are adiabatic. Energy is enabled with illustrative constant properties and buoyancy disabled.",
    },
    Recipe {
        id: "scripted-incoming-temperature", title: "Rhai · incoming-only temperature",
        description: "Transient thermal inlet using a scripted Inlet/outlet temperature condition.",
        patch: "inlet", quantity: Quantity::Temperature, timed: true,
        source: include_str!("../../examples/scripting/incoming-temperature.rhai"),
        parameters: &[("base_temperature",293.15),("amplitude",2.0),("frequency",1.0)],
        notes: "Thermal condition: Inlet/outlet temperature. On incoming faces T=293.15+2 sin(2*pi*t) K; outgoing faces use zero gradient. The same thermal hook can be attached to an outlet with explicitly specified physical backflow velocity. No velocity backflow script is supported.",
    },
    Recipe {
        id: "scripted-wall-temperature", title: "Rhai · heated wall ramp",
        description: "Transient channel with a scripted top-wall temperature ramp.",
        patch: "top_wall", quantity: Quantity::Temperature, timed: true,
        source: include_str!("../../examples/scripting/wall-temperature.rhai"),
        parameters: &[("initial_temperature",293.15),("final_temperature",303.15),("ramp_time",0.5)],
        notes: "Flow condition: No-slip wall. Thermal condition: Fixed temperature. Top-wall temperature ramps from 293.15 to 303.15 K over 0.5 s, then holds. The bottom wall is adiabatic. This scripts wall temperature, not wall velocity or heat flux.",
    },
];

pub(super) fn templates() -> Result<Vec<CaseTemplateManifest>, String> {
    let mut entries = Vec::new();
    for recipe in RECIPES {
        let mut session =
            build_example(ExampleProjectId::LaminarChannel2D).map_err(|e| e.to_string())?;
        if recipe.timed {
            session
                .set_flow_models(
                    TimeIntegration::Euler {
                        dt: 0.01,
                        steps: 100,
                    },
                    session.solver_options().turbulence.clone(),
                )
                .map_err(|e| e.to_string())?;
            session
                .set_pressure_coupling(crate::simple::PressureCoupling {
                    algorithm: crate::simple::CouplingAlgorithm::Pimple,
                    ..Default::default()
                })
                .map_err(|e| e.to_string())?;
        }
        if recipe.id == "scripted-backflow-outlet" {
            session
                .configure_named_boundary(
                    "outlet",
                    IncompressibleBoundaryCondition::PressureOutletBackflow {
                        pressure: 0.0,
                        velocity: Vec3::ZERO,
                    },
                )
                .map_err(|e| e.to_string())?;
        }
        if recipe.quantity == Quantity::Temperature {
            let mut thermal = Options {
                enabled: true,
                buoyancy: false,
                ..Default::default()
            };
            thermal.boundaries.insert(
                "inlet".into(),
                Boundary::FixedTemperature {
                    temperature: 293.15,
                },
            );
            thermal.boundaries.insert(
                "outlet".into(),
                Boundary::InletOutlet {
                    incoming_temperature: 293.15,
                },
            );
            thermal
                .boundaries
                .insert("top_wall".into(), Boundary::Adiabatic);
            thermal
                .boundaries
                .insert("bottom_wall".into(), Boundary::Adiabatic);
            if recipe.id == "scripted-incoming-temperature" {
                thermal.boundaries.insert(
                    recipe.patch.into(),
                    Boundary::InletOutlet {
                        incoming_temperature: 293.15,
                    },
                );
            } else {
                thermal.boundaries.insert(
                    recipe.patch.into(),
                    Boundary::FixedTemperature {
                        temperature: 293.15,
                    },
                );
            }
            session.set_thermal(thermal).map_err(|e| e.to_string())?;
        }
        let binding = Binding {
            script: recipe.id.into(),
            function: "profile".into(),
            quantity: recipe.quantity,
            parameters: recipe
                .parameters
                .iter()
                .map(|(k, v)| (k.to_string(), *v))
                .collect(),
        };
        let mut scripting = Settings::default();
        scripting.scripts.insert(
            recipe.id.into(),
            Script {
                source: recipe.source.into(),
                time_dependent: recipe.timed,
            },
        );
        scripting
            .bindings
            .insert(recipe.patch.into(), vec![binding]);
        session.set_scripting(scripting)?;
        let notes=format!("{}\n\nOpen Setup > Boundaries > {} > Scripts / attach profile to inspect the pre-attached code. Use Check / preview, then Attach / update to apply parameter changes. Generate a mesh before solving. {}\n\nInitial Rhai support: Velocity inlet; static Pressure outlet (strict or explicit backflow); Fixed temperature; Inlet/outlet temperature. Heat flux, moving-wall velocity, total pressure, flow-rate, material laws and volume sources are not scripted in this release. Educational setup; not numerically validated.",recipe.notes,recipe.patch,if recipe.timed {"Euler dt=0.01 s, 100 physical steps (1 s), with PIMPLE. Refine time step and mesh as appropriate."}else{"Steady, spatial-only function signature: profile(x,y,z)."});
        let mut entry = manifest(
            recipe.id,
            recipe.title,
            "Rhai boundary scripts",
            recipe.description,
            session,
            &notes,
        );
        entry.capabilities.extend([
            "Rhai".into(),
            format!("{:?} profile", recipe.quantity),
            if recipe.timed {
                "Transient".into()
            } else {
                "Steady".into()
            },
            "Pre-attached script".into(),
        ]);
        entries.push(entry);
    }
    Ok(entries)
}
