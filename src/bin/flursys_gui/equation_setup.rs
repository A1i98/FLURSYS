//! Atomic drafts for independent equation controls.
use super::*;
use flursys::equations::{EquationControl, EquationPreconditioner as P, EquationSolvers};
use slint::Model;
fn row(name: &str, c: EquationControl, pressure: bool) -> EquationDraft {
    EquationDraft {
        name: name.into(),
        pressure,
        absolute: c.absolute_tolerance.to_string().into(),
        relative: c.relative_tolerance.to_string().into(),
        iterations: c.max_iterations as i32,
        preconditioner: match c.preconditioner {
            P::None => 0,
            P::Jacobi => 1,
            P::Ic0 => 2,
            P::Gamg => 3,
            P::Auto => 4,
        },
    }
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_open_equation_controls(move || {
        let Some(ui) = weak.upgrade() else { return };
        let state = shared.borrow();
        if state.solving || state.meshing {
            return;
        }
        let s = state.workbench.solver_options();
        let e = s.equations.clone().unwrap_or_else(|| EquationSolvers {
            velocity: EquationControl::from_options(s.momentum_solver),
            pressure: EquationControl {
                preconditioner: P::Ic0,
                ..EquationControl::from_options(s.pressure_solver)
            },
            temperature: EquationControl::from_options(s.momentum_solver),
            k: EquationControl::from_options(s.momentum_solver),
            omega: EquationControl::from_options(s.momentum_solver),
            epsilon: EquationControl::from_options(s.momentum_solver),
            nu_tilda: EquationControl::from_options(s.momentum_solver),
        });
        ui.set_equation_drafts(ModelRc::new(VecModel::from(vec![
            row("Velocity Ux / Uy / Uz", e.velocity, false),
            row("Pressure correction", e.pressure, true),
            row("Temperature", e.temperature, false),
            row("Turbulent kinetic energy k", e.k, false),
            row("Specific dissipation omega", e.omega, false),
            row("Dissipation epsilon", e.epsilon, false),
            row("SA modified viscosity nu_tilda", e.nu_tilda, false),
        ])));
        ui.set_equation_feedback(
            "Settings for inactive equations are retained for later use.".into(),
        );
        ui.set_equation_controls_open(true);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_apply_equation_controls(move || {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if state.solving || state.meshing {
            return;
        }
        let result = (|| -> Result<(), String> {
            let rows = ui.get_equation_drafts();
            let mut values = Vec::new();
            for i in 0..7 {
                let r = rows.row_data(i).ok_or("Missing equation draft")?;
                let number = |v: SharedString| {
                    v.trim()
                        .parse::<f64>()
                        .map_err(|_| format!("Invalid tolerance for {}", r.name))
                };
                let c = EquationControl {
                    absolute_tolerance: number(r.absolute)?,
                    relative_tolerance: number(r.relative)?,
                    max_iterations: r.iterations.max(0) as usize,
                    preconditioner: match r.preconditioner {
                        0 => P::None,
                        1 => P::Jacobi,
                        2 => P::Ic0,
                        3 => P::Gamg,
                        4 => P::Auto,
                        _ => return Err("Invalid preconditioner".into()),
                    },
                };
                c.validate(i == 1)?;
                values.push(c);
            }
            let e = EquationSolvers {
                velocity: values[0],
                pressure: values[1],
                temperature: values[2],
                k: values[3],
                omega: values[4],
                epsilon: values[5],
                nu_tilda: values[6],
            };
            state
                .workbench
                .set_equation_solvers(e)
                .map_err(|e| e.to_string())?;
            state.mark_workbench_dirty();
            state.log("Independent equation solver controls applied.");
            ui.set_equation_controls_open(false);
            Ok(())
        })();
        if let Err(e) = result {
            ui.set_equation_feedback(e.into());
        }
        refresh_ui(&ui, &state);
    });
}
