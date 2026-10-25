use super::*;
use widgets::{heading, number};
fn bounded(ui: &mut egui::Ui, label: &str, value: &mut f64, min: f64, max: f64) {
    ui.label(label);
    let speed = value.abs().max(1e-6) * 0.01;
    ui.add(egui::DragValue::new(value).range(min..=max).speed(speed));
}
fn equation(
    ui: &mut egui::Ui,
    name: &str,
    c: &mut flursys::equations::EquationControl,
    pressure: bool,
) {
    use flursys::equations::EquationPreconditioner as P;
    egui::CollapsingHeader::new(name)
        .id_salt(("equation", name))
        .show(ui, |ui| {
            bounded(
                ui,
                "Absolute residual tolerance",
                &mut c.absolute_tolerance,
                0.,
                1.,
            );
            bounded(
                ui,
                "Relative residual tolerance",
                &mut c.relative_tolerance,
                0.,
                0.999999,
            );
            ui.label("Maximum linear iterations");
            ui.add(egui::DragValue::new(&mut c.max_iterations).range(1..=1_000_000));
            egui::ComboBox::from_id_salt(("preconditioner", name))
                .selected_text(format!("{:?}", c.preconditioner))
                .show_ui(ui, |ui| {
                    for (p, n) in [
                        (P::None, "None"),
                        (P::Jacobi, "Jacobi"),
                        (P::Ic0, "IC(0)"),
                        (P::Gamg, "Aggregation multigrid"),
                        (P::Auto, "Auto: IC(0) / Jacobi"),
                    ] {
                        ui.add_enabled_ui(pressure || matches!(p, P::None | P::Jacobi), |ui| {
                            ui.selectable_value(&mut c.preconditioner, p, n)
                        });
                    }
                });
            if let Err(e) = c.validate(pressure) {
                widgets::error(ui, &e);
            }
        });
}
impl App {
    pub(super) fn equation_properties(&mut self, ui: &mut egui::Ui) {
        let coupling = &mut self.editor.solver.pressure_coupling;
        heading(ui, "PRESSURE CORRECTIONS");
        ui.add_enabled_ui(
            coupling.algorithm == flursys::simple::CouplingAlgorithm::Pimple,
            |ui| {
                ui.label("Pressure correctors (PIMPLE)");
                ui.add(egui::DragValue::new(&mut coupling.pressure_correctors).range(1..=8));
            },
        );
        if coupling.algorithm != flursys::simple::CouplingAlgorithm::Pimple {
            coupling.pressure_correctors = 1;
        }
        ui.label("Extra non-orthogonal corrections");
        ui.add(egui::DragValue::new(&mut coupling.non_orthogonal_correctors).range(0..=8));
        bounded(
            ui,
            "Explicit non-orthogonal limiter (0–1)",
            &mut coupling.non_orthogonal_limiter,
            0.,
            1.,
        );
        if let Err(e) = coupling.validate(self.editor.solver.time.is_transient()) {
            widgets::error(ui, &e);
        }
        heading(ui, "LINEAR EQUATION SOLVERS");
        let mut enabled = self.editor.solver.equations.is_some();
        if ui
            .checkbox(&mut enabled, "Separate controls for each equation")
            .changed()
        {
            self.editor.solver.equations =
                enabled.then(flursys::equations::EquationSolvers::default);
        }
        if let Some(e) = &mut self.editor.solver.equations {
            equation(ui, "Pressure", &mut e.pressure, true);
            equation(ui, "Velocity", &mut e.velocity, false);
            if self.editor.solver.thermal.enabled {
                equation(ui, "Temperature", &mut e.temperature, false);
            }
            use flursys::turbulence::TurbulenceModel as T;
            match self.editor.solver.turbulence.model {
                T::KOmega | T::KOmegaSst => {
                    equation(ui, "Turbulent kinetic energy k", &mut e.k, false);
                    equation(ui, "Specific dissipation ω", &mut e.omega, false);
                }
                T::KEpsilon | T::RngKEpsilon | T::RealizableKEpsilon => {
                    equation(ui, "Turbulent kinetic energy k", &mut e.k, false);
                    equation(ui, "Dissipation ε", &mut e.epsilon, false);
                }
                T::SpalartAllmaras => equation(ui, "Modified viscosity", &mut e.nu_tilda, false),
                _ => {}
            }
        } else {
            ui.weak("Core default linear controls are active. Enable separate equation controls to customize tolerances and preconditioners.");
        }
    }
    pub(super) fn turbulence_properties(&mut self, ui: &mut egui::Ui) {
        use flursys::turbulence::{TurbulenceInlet as I, TurbulenceModel as T, WallTreatment as W};
        let t = &mut self.editor.solver.turbulence;
        if t.model == T::Laminar {
            t.wall_treatment = W::Resolved;
            return;
        }
        bounded(ui, "Equation relaxation (0–1)", &mut t.relaxation, 1e-6, 1.);
        bounded(
            ui,
            "Relative turbulence tolerance",
            &mut t.relative_tolerance,
            1e-14,
            0.999999,
        );
        let epsilon = t.model.uses_epsilon();
        let omega = matches!(t.model, T::KOmega | T::KOmegaSst);
        let sa = t.model == T::SpalartAllmaras;
        if t.is_rans() {
            heading(ui, "INITIAL / DEFAULT INCOMING TURBULENCE");
            if sa {
                number(ui, "Modified viscosity ν̃ (m²/s)", &mut t.nu_tilda);
            } else {
                number(ui, "Turbulent kinetic energy k (m²/s²)", &mut t.k);
                if epsilon {
                    number(ui, "Dissipation ε (m²/s³)", &mut t.epsilon);
                }
                if omega {
                    number(ui, "Specific dissipation ω (1/s)", &mut t.omega);
                }
            }
            let walls = [
                (W::Resolved, "Wall resolved"),
                (W::EquilibriumKEpsilon, "Equilibrium k–ε wall function"),
                (W::EquilibriumKOmega, "Equilibrium k–ω wall function"),
            ];
            egui::ComboBox::from_id_salt("wall-treatment")
                .selected_text(
                    walls
                        .iter()
                        .find(|(w, _)| *w == t.wall_treatment)
                        .map(|(_, n)| *n)
                        .unwrap_or("Wall resolved"),
                )
                .show_ui(ui, |ui| {
                    for (w, name) in walls {
                        ui.add_enabled_ui(
                            w == W::Resolved
                                || (epsilon && w == W::EquilibriumKEpsilon)
                                || (omega && w == W::EquilibriumKOmega),
                            |ui| ui.selectable_value(&mut t.wall_treatment, w, name),
                        );
                    }
                });
            if sa || (!epsilon && !omega) {
                t.wall_treatment = W::Resolved;
            }
            heading(ui, "PER-PATCH INCOMING / BACKFLOW");
            for patch in &self.project.named_selections {
                egui::CollapsingHeader::new(&patch.name)
                    .id_salt(("incoming", &patch.name))
                    .show(ui, |ui| {
                        let current = t.inlets.get(&patch.name).copied();
                        let mut mode = match current {
                            None => 0,
                            Some(I::Direct { .. }) => 1,
                            Some(I::IntensityLength { .. }) => 2,
                            Some(I::IntensityDiameter { .. }) => 3,
                            Some(I::IntensityViscosityRatio { .. }) => 4,
                            Some(I::ModifiedViscosity { .. }) => 5,
                        };
                        let old = mode;
                        let names = [
                            "Global defaults",
                            "Direct k / ω / ε",
                            "Intensity and length",
                            "Intensity and hydraulic diameter",
                            "Intensity and viscosity ratio",
                            "Modified viscosity (SA)",
                        ];
                        egui::ComboBox::from_id_salt(("incoming-mode", &patch.name))
                            .selected_text(names[mode])
                            .show_ui(ui, |ui| {
                                for (i, n) in names.iter().enumerate() {
                                    ui.add_enabled_ui(i == 0 || ((i == 5) == sa), |ui| {
                                        ui.selectable_value(&mut mode, i, *n)
                                    });
                                }
                            });
                        if old != mode {
                            let v = match mode {
                                1 => Some(I::Direct {
                                    k: t.k,
                                    omega: t.omega,
                                    epsilon: t.epsilon,
                                }),
                                2 => Some(I::IntensityLength {
                                    intensity: 0.05,
                                    length: 0.01,
                                }),
                                3 => Some(I::IntensityDiameter {
                                    intensity: 0.05,
                                    hydraulic_diameter: 0.1,
                                    length_ratio: 0.07,
                                }),
                                4 => Some(I::IntensityViscosityRatio {
                                    intensity: 0.05,
                                    viscosity_ratio: 10.,
                                }),
                                5 => Some(I::ModifiedViscosity {
                                    nu_tilda: t.nu_tilda,
                                }),
                                _ => None,
                            };
                            if let Some(v) = v {
                                t.inlets.insert(patch.name.clone(), v);
                            } else {
                                t.inlets.remove(&patch.name);
                            }
                        }
                        if let Some(v) = t.inlets.get_mut(&patch.name) {
                            match v {
                                I::Direct {
                                    k,
                                    omega: w,
                                    epsilon: e,
                                } => {
                                    number(ui, "k (m²/s²)", k);
                                    if omega {
                                        number(ui, "ω (1/s)", w);
                                    }
                                    if epsilon {
                                        number(ui, "ε (m²/s³)", e);
                                    }
                                }
                                I::IntensityLength { intensity, length } => {
                                    bounded(ui, "Intensity (fraction, 0–1)", intensity, 1e-6, 1.);
                                    number(ui, "Length scale (m)", length);
                                }
                                I::IntensityDiameter {
                                    intensity,
                                    hydraulic_diameter,
                                    length_ratio,
                                } => {
                                    bounded(ui, "Intensity (fraction, 0–1)", intensity, 1e-6, 1.);
                                    number(ui, "Hydraulic diameter (m)", hydraulic_diameter);
                                    number(ui, "Length / diameter ratio", length_ratio);
                                }
                                I::IntensityViscosityRatio {
                                    intensity,
                                    viscosity_ratio,
                                } => {
                                    bounded(ui, "Intensity (fraction, 0–1)", intensity, 1e-6, 1.);
                                    number(ui, "νt / ν", viscosity_ratio);
                                }
                                I::ModifiedViscosity { nu_tilda } => {
                                    number(ui, "ν̃ (m²/s)", nu_tilda)
                                }
                            }
                            if let Err(e) = v.validate() {
                                widgets::error(ui, &e);
                            }
                        }
                    });
            }
        } else {
            heading(ui, "LES COEFFICIENTS");
            if t.model == T::Smagorinsky {
                number(ui, "Ck", &mut t.ck);
                number(ui, "Ce", &mut t.ce);
            }
            if t.model == T::Wale {
                number(ui, "WALE Cw", &mut t.cw);
            }
        }
        if let Err(e) = t.validate() {
            widgets::error(ui, &e);
        }
    }
    pub(super) fn thermal_properties(&mut self, ui: &mut egui::Ui) {
        let t = &mut self.editor.solver.thermal;
        number(ui, "Initial temperature (K)", &mut t.initial_temperature);
        number(ui, "Specific heat Cp (J/kg/K)", &mut t.specific_heat);
        number(ui, "Thermal conductivity (W/m/K)", &mut t.conductivity);
        number(ui, "Volumetric heat source (W/m³)", &mut t.source_w_m3);
        bounded(
            ui,
            "Temperature relaxation (0–1)",
            &mut t.relaxation,
            1e-6,
            1.,
        );
        bounded(
            ui,
            "Relative temperature tolerance",
            &mut t.relative_tolerance,
            1e-14,
            0.999999,
        );
        number(ui, "Turbulent Prandtl number", &mut t.turbulent_prandtl);
        use flursys::thermal::ThermalWallTreatment as W;
        egui::ComboBox::from_id_salt("thermal-wall")
            .selected_text(match t.wall_treatment {
                W::Molecular => "Molecular",
                W::EquilibriumPrt => "Equilibrium turbulent Prandtl",
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut t.wall_treatment, W::Molecular, "Molecular");
                ui.selectable_value(
                    &mut t.wall_treatment,
                    W::EquilibriumPrt,
                    "Equilibrium turbulent Prandtl",
                );
            });
        ui.checkbox(&mut t.buoyancy, "Boussinesq buoyancy");
        if t.buoyancy {
            number(
                ui,
                "Reference temperature (K)",
                &mut t.reference_temperature,
            );
            number(ui, "Thermal expansion β (1/K)", &mut t.expansion);
            widgets::vector(ui, "Gravity acceleration (m/s²)", &mut t.gravity);
            bounded(
                ui,
                "Maximum |β (T−Tref)|",
                &mut t.max_boussinesq_fraction,
                1e-6,
                1.,
            );
            ui.small("Hot fluid accelerates opposite gravity for positive thermal expansion. This is a Boussinesq approximation.");
        }
        if let Err(e) = t.validate() {
            widgets::error(ui, &e);
        }
    }
}
impl App {
    pub(super) fn script_binding_properties(&mut self, ui: &mut egui::Ui) {
        use flursys::scripting::Quantity as Q;
        let names: Vec<_> = self
            .editor
            .solver
            .scripting
            .scripts
            .keys()
            .cloned()
            .collect();
        for (patch, bindings) in &mut self.editor.solver.scripting.bindings {
            egui::CollapsingHeader::new(format!("Bindings · {patch}"))
                .id_salt(("bindings", patch))
                .show(ui, |ui| {
                    let mut remove = None;
                    for (i, b) in bindings.iter_mut().enumerate() {
                        ui.push_id(i, |ui| {
                            egui::ComboBox::from_id_salt("script")
                                .selected_text(&b.script)
                                .show_ui(ui, |ui| {
                                    for n in &names {
                                        ui.selectable_value(&mut b.script, n.clone(), n);
                                    }
                                });
                            egui::ComboBox::from_id_salt("quantity")
                                .selected_text(match b.quantity {
                                    Q::Velocity => "Velocity (m/s)",
                                    Q::Pressure => "Kinematic pressure (m²/s²)",
                                    Q::Temperature => "Temperature (K)",
                                })
                                .show_ui(ui, |ui| {
                                    for (q, n) in [
                                        (Q::Velocity, "Velocity (m/s)"),
                                        (Q::Pressure, "Kinematic pressure (m²/s²)"),
                                        (Q::Temperature, "Temperature (K)"),
                                    ] {
                                        ui.selectable_value(&mut b.quantity, q, n);
                                    }
                                });
                            ui.label("Function name");
                            ui.text_edit_singleline(&mut b.function);
                            let mut remove_parameter = None;
                            for (name, value) in &mut b.parameters {
                                ui.horizontal(|ui| {
                                    number(ui, name, value);
                                    if ui.small_button("Remove").clicked() {
                                        remove_parameter = Some(name.clone());
                                    }
                                });
                            }
                            if let Some(name) = remove_parameter {
                                b.parameters.remove(&name);
                            }
                            if ui.small_button("Add numeric parameter").clicked() {
                                let mut n = 1;
                                while b.parameters.contains_key(&format!("parameter_{n}")) {
                                    n += 1;
                                }
                                b.parameters.insert(format!("parameter_{n}"), 0.);
                            }
                            if ui.button("Remove binding").clicked() {
                                remove = Some(i);
                            }
                            ui.separator();
                        });
                    }
                    if let Some(i) = remove {
                        bindings.remove(i);
                    }
                });
        }
    }
}
impl App {
    pub(super) fn stage_boundary_draft(&mut self) {
        use workbench::PhysicalBoundaryCondition as B;
        let v = self.boundary_velocity;
        let p = self.boundary_pressure;
        let condition = match self.boundary_kind {
            0 => B::NoSlipWall,
            1 => B::MovingWall { velocity: v },
            2 => B::VelocityInlet { velocity: v },
            3 => B::PressureOutlet { pressure: p },
            4 => B::PressureOutletBackflow {
                pressure: p,
                velocity: v,
            },
            5 => B::Slip,
            6 => B::Symmetry,
            7 => B::FarField {
                velocity: v,
                pressure: p,
            },
            8 => B::TotalPressure { pressure: p },
            9 => B::ParabolicInlet {
                velocity: v,
                axis: self.inlet_axis,
                center: self.inlet_center,
                half_width: self.inlet_half_width,
            },
            _ => B::FlowRateInlet {
                rate: p,
                mass: self.boundary_kind == 11,
            },
        };
        if let Some(row) = self
            .editor
            .boundaries
            .iter_mut()
            .find(|b| b.selection == self.boundary_patch)
        {
            row.condition = condition;
        } else {
            self.editor.boundaries.push(workbench::BoundaryAssignment {
                selection: self.boundary_patch.clone(),
                condition,
            });
        }
    }
}
