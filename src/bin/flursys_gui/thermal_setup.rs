//! Material catalog drafts and explicit per-patch thermal boundary editing.
use super::*;
fn number(s: SharedString, label: &str) -> Result<f64, String> {
    let x = s
        .trim()
        .parse::<f64>()
        .map_err(|_| format!("{label} must be a number"))?;
    if !x.is_finite() {
        return Err(format!("{label} must be finite"));
    }
    Ok(x)
}
pub(super) fn push(ui: &MainWindow, s: &WorkbenchSession) {
    let t = &s.solver_options().thermal;
    let p = s.to_project("material");
    ui.set_material_draft_density(s.material().density.to_string().into());
    ui.set_material_draft_viscosity(s.material().kinematic_viscosity.to_string().into());
    ui.set_material_catalog_index(
        p.material
            .catalog
            .as_ref()
            .and_then(|m| flursys::materials::IDS.iter().position(|id| *id == m.id))
            .map_or(0, |i| i as i32 + 1),
    );
    ui.set_material_eval_temperature(
        p.material
            .catalog
            .as_ref()
            .map_or(t.initial_temperature, |m| m.temperature)
            .to_string()
            .into(),
    );
    ui.set_material_eval_pressure(
        p.material
            .catalog
            .as_ref()
            .map_or(101325.0, |m| m.pressure)
            .to_string()
            .into(),
    );
    ui.set_material_source(
        p.material
            .catalog
            .as_ref()
            .map(|m| format!("{}\n{}", m.source, m.note))
            .unwrap_or_else(|| {
                "User-defined constant properties; no catalog source attached.".into()
            })
            .into(),
    );
    ui.set_thermal_enabled(t.enabled);
    ui.set_thermal_initial(t.initial_temperature.to_string().into());
    ui.set_thermal_cp(t.specific_heat.to_string().into());
    ui.set_thermal_kappa(t.conductivity.to_string().into());
    ui.set_thermal_volume_source(t.source_w_m3.to_string().into());
    ui.set_thermal_buoyancy(t.buoyancy);
    ui.set_thermal_beta(t.expansion.to_string().into());
    ui.set_thermal_reference(t.reference_temperature.to_string().into());
    ui.set_thermal_gx(t.gravity.x.to_string().into());
    ui.set_thermal_gy(t.gravity.y.to_string().into());
    ui.set_thermal_gz(t.gravity.z.to_string().into());
    ui.set_thermal_wall_treatment(
        if t.wall_treatment == flursys::thermal::ThermalWallTreatment::EquilibriumPrt {
            1
        } else {
            0
        },
    );
    ui.set_thermal_relaxation(t.relaxation.to_string().into());
    ui.set_thermal_tolerance(t.relative_tolerance.to_string().into());
    ui.set_thermal_prt(t.turbulent_prandtl.to_string().into());
}
pub(super) fn apply(
    ui: &MainWindow,
    s: &mut WorkbenchSession,
    rho: f64,
    nu: f64,
) -> Result<(), String> {
    let mut t = s.solver_options().thermal.clone();
    t.enabled = ui.get_thermal_enabled();
    t.buoyancy = ui.get_thermal_buoyancy();
    t.initial_temperature = number(ui.get_thermal_initial(), "Initial temperature")?;
    t.specific_heat = number(ui.get_thermal_cp(), "Specific heat")?;
    t.conductivity = number(ui.get_thermal_kappa(), "Conductivity")?;
    t.source_w_m3 = number(ui.get_thermal_volume_source(), "Volumetric heat source")?;
    t.expansion = number(ui.get_thermal_beta(), "Expansion coefficient")?;
    t.reference_temperature = number(ui.get_thermal_reference(), "Reference temperature")?;
    t.gravity = Vec3::new(
        number(ui.get_thermal_gx(), "Gravity X")?,
        number(ui.get_thermal_gy(), "Gravity Y")?,
        number(ui.get_thermal_gz(), "Gravity Z")?,
    );
    t.wall_treatment = if ui.get_thermal_wall_treatment() == 1 {
        flursys::thermal::ThermalWallTreatment::EquilibriumPrt
    } else {
        flursys::thermal::ThermalWallTreatment::Molecular
    };
    t.relaxation = number(ui.get_thermal_relaxation(), "Temperature relaxation")?;
    t.relative_tolerance = number(
        ui.get_thermal_tolerance(),
        "Temperature convergence tolerance",
    )?;
    t.turbulent_prandtl = number(ui.get_thermal_prt(), "Turbulent Prandtl")?;
    t.validate()?;
    if t.enabled
        && t.wall_treatment == flursys::thermal::ThermalWallTreatment::EquilibriumPrt
        && (s.solver_options().turbulence.model != flursys::turbulence::TurbulenceModel::KEpsilon
            || s.solver_options().turbulence.wall_treatment
                != flursys::turbulence::WallTreatment::EquilibriumKEpsilon)
    {
        return Err(
            "Select equilibrium k-epsilon momentum walls before the thermal Prt wall model".into(),
        );
    }
    if !rho.is_finite() || rho <= 0.0 || !nu.is_finite() || nu < 0.0 {
        return Err("Require density > 0 and viscosity ≥ 0".into());
    }
    let catalog = if ui.get_material_catalog_index() > 0 {
        let id = flursys::materials::IDS
            .get(ui.get_material_catalog_index() as usize - 1)
            .ok_or("Unknown material")?;
        let m = flursys::materials::evaluate(
            id,
            number(ui.get_material_eval_temperature(), "Catalog T")?,
            number(ui.get_material_eval_pressure(), "Catalog p")?,
        )?;
        let equal = |a: f64, b: f64| (a - b).abs() <= 1e-10 * a.abs().max(b.abs()).max(1e-30);
        if !equal(rho, m.density)
            || !equal(nu, m.kinematic_viscosity())
            || !equal(t.specific_heat, m.specific_heat)
            || !equal(t.conductivity, m.conductivity)
            || !equal(t.expansion, m.expansion)
        {
            return Err("Reload catalog properties after changing evaluation T/p, or select User-defined to override values".into());
        }
        Some(m)
    } else {
        None
    };
    // All draft data is validated before changing the current case.
    if let Some(m) = catalog {
        s.apply_catalog_material(m).map_err(|e| e.to_string())?;
    } else {
        s.set_user_material(rho, nu).map_err(|e| e.to_string())?;
    }
    s.set_thermal(t).map_err(|e| e.to_string())?;
    Ok(())
}
pub(super) fn push_boundary(ui: &MainWindow, s: &WorkbenchSession, name: &str) {
    use flursys::turbulence::TurbulenceInlet as TI;
    let t = &s.solver_options().turbulence;
    ui.set_turb_patch_nu_tilda(t.nu_tilda.to_string().into());
    ui.set_turb_patch_k(t.k.to_string().into());
    ui.set_turb_patch_omega(t.omega.to_string().into());
    ui.set_turb_patch_epsilon(t.epsilon.to_string().into());
    match t.inlets.get(name) {
        None => ui.set_turb_patch_mode(0),
        Some(TI::ModifiedViscosity{nu_tilda})=>{ui.set_turb_patch_mode(3);ui.set_turb_patch_nu_tilda(nu_tilda.to_string().into());}
        Some(TI::Direct { k, omega, epsilon }) => {
            ui.set_turb_patch_mode(1);
            ui.set_turb_patch_k(k.to_string().into());
            ui.set_turb_patch_omega(omega.to_string().into());
            ui.set_turb_patch_epsilon(epsilon.to_string().into());
        }
        Some(TI::IntensityLength { intensity, length }) => {
            ui.set_turb_patch_mode(2);
            ui.set_turb_patch_intensity(intensity.to_string().into());
            ui.set_turb_patch_length(length.to_string().into());
        }
    }
    ui.set_turb_patch_status(
        if t.inlets.contains_key(name) {
            "Patch-specific incoming RANS data"
        } else {
            "Global incoming RANS values"
        }
        .into(),
    );

    use flursys::thermal::Boundary as B;
    let bc = s.solver_options().thermal.boundaries.get(name);
    let (kind, a, b) = match bc.copied() {
        None => (0, 293.15, 293.15),
        Some(B::Adiabatic) => (0, 0.0, 0.0),
        Some(B::FixedTemperature { temperature }) => (1, temperature, 0.0),
        Some(B::InletOutlet {
            incoming_temperature,
        }) => (2, incoming_temperature, 0.0),
        Some(B::HeatFlux { inward_flux }) => (3, inward_flux, 0.0),
        Some(B::Convection {
            coefficient,
            ambient_temperature,
        }) => (4, coefficient, ambient_temperature),
    };
    ui.set_thermal_bc_kind(kind);
    ui.set_thermal_bc_value(a.to_string().into());
    ui.set_thermal_bc_ambient(b.to_string().into());
    ui.set_thermal_bc_status(
        if bc.is_some() {
            format!("Thermal condition assigned to {name}")
        } else {
            format!("Thermal condition unassigned: {name}")
        }
        .into(),
    );
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_apply_turbulence_patch(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.meshing || state.solving {
            return;
        }
        let result = (|| -> Result<(), String> {
            use flursys::turbulence::TurbulenceInlet as TI;
            let name = state
                .patch_names
                .get(ui.get_patch_index().max(0) as usize)
                .ok_or("Select a patch")?
                .clone();
            let mut t = state.workbench.solver_options().turbulence.clone();
            let b = match ui.get_turb_patch_mode() {
                3=>Some(TI::ModifiedViscosity{nu_tilda:number(ui.get_turb_patch_nu_tilda(),"SA modified viscosity")?}),
                1 => Some(TI::Direct {
                    k: number(ui.get_turb_patch_k(), "k")?,
                    omega: number(ui.get_turb_patch_omega(), "omega")?,
                    epsilon: number(ui.get_turb_patch_epsilon(), "epsilon")?,
                }),
                2 => Some(TI::IntensityLength {
                    intensity: number(ui.get_turb_patch_intensity(), "Intensity")?,
                    length: number(ui.get_turb_patch_length(), "Length")?,
                }),
                _ => None,
            };
            if let Some(b) = b {
                b.validate()?;
                t.inlets.insert(name.clone(), b);
            } else {
                t.inlets.remove(&name);
            }
            let time = state.workbench.solver_options().time;
            state
                .workbench
                .set_flow_models(time, t)
                .map_err(|e| e.to_string())?;
            state.mark_workbench_dirty();
            push_boundary(&ui, &state.workbench, &name);
            state.log(format!("Incoming turbulence updated: {name}"));
            Ok(())
        })();
        if let Err(e) = result {
            state.log(e);
        }
        refresh_ui(&ui, &state);
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_prepare_material_editor(move || {
        if let Some(ui) = weak.upgrade() {
            push(&ui, &shared.borrow().workbench);
        }
    });
    let weak = ui.as_weak();
    ui.on_load_material_catalog(move||{
        let Some(ui)=weak.upgrade()else{return};
        if ui.get_material_catalog_index()==0{ui.set_material_source("User-defined properties. Supply physical data and units explicitly.".into());return}
        let result=(||->Result<flursys::materials::MaterialSnapshot,String>{let id=flursys::materials::IDS.get(ui.get_material_catalog_index() as usize-1).ok_or("Unknown catalog ID")?;flursys::materials::evaluate(id,number(ui.get_material_eval_temperature(),"Catalog T")?,number(ui.get_material_eval_pressure(),"Catalog p")?)})();
        match result {Ok(m)=>{ui.set_material_draft_density(m.density.to_string().into());ui.set_material_draft_viscosity(m.kinematic_viscosity().to_string().into());ui.set_thermal_cp(m.specific_heat.to_string().into());ui.set_thermal_kappa(m.conductivity.to_string().into());ui.set_thermal_beta(m.expansion.to_string().into());ui.set_thermal_reference(m.temperature.to_string().into());ui.set_thermal_initial(m.temperature.to_string().into());ui.set_material_source(format!("{}\n{}",m.source,m.note).into());ui.set_material_dialog_feedback("Properties loaded into draft; Apply to save. Properties are frozen at this evaluation state during the solve.".into());},Err(e)=>ui.set_material_dialog_feedback(e.into())}
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_apply_workbench_thermal_boundary(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.meshing || state.solving {
            return;
        }
        let result = (|| -> Result<(), String> {
            use flursys::thermal::Boundary as B;
            let name = state
                .patch_names
                .get(ui.get_patch_index().max(0) as usize)
                .ok_or("Select a boundary patch")?
                .clone();
            let a = if ui.get_thermal_bc_kind() == 0 {
                0.0
            } else {
                number(ui.get_thermal_bc_value(), "Thermal boundary value")?
            };
            let bc = match ui.get_thermal_bc_kind() {
                0 => B::Adiabatic,
                1 => B::FixedTemperature { temperature: a },
                2 => B::InletOutlet {
                    incoming_temperature: a,
                },
                3 => B::HeatFlux { inward_flux: a },
                _ => B::Convection {
                    coefficient: a,
                    ambient_temperature: number(
                        ui.get_thermal_bc_ambient(),
                        "Ambient temperature",
                    )?,
                },
            };
            let mut t = state.workbench.solver_options().thermal.clone();
            t.boundaries.insert(name.clone(), bc);
            // Validate boundary values even if energy is currently disabled.
            let mut checked = t.clone();
            checked.enabled = true;
            checked.validate()?;
            state.workbench.set_thermal(t).map_err(|e| e.to_string())?;
            state.mark_workbench_dirty();
            push_boundary(&ui, &state.workbench, &name);
            state.log(format!("Thermal condition assigned to {name}"));
            Ok(())
        })();
        if let Err(e) = result {
            state.log(e);
        }
        refresh_ui(&ui, &state);
    });
}
