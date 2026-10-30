use super::*;
impl App {
    pub(super) fn analytical_initialization(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.button("Uniform profile").clicked() {
                self.editor.solver.initial_profile =
                    Some(workbench::initialization::InitialProfile::Uniform {
                        velocity: self.initial_velocity,
                        pressure: self.initial_pressure,
                    });
            }
            if ui.button("Boundary layer profile").clicked() {
                self.editor.solver.initial_profile =
                    Some(workbench::initialization::InitialProfile::BoundaryLayer2D {
                        speed: self.initial_velocity.x.abs().max(1.0),
                        wall_y: 0.0,
                        thickness: self.editor.mesh.global_size,
                        exponent: 1.0 / 7.0,
                        pressure: self.initial_pressure,
                    });
            }
        });
        let Some(profile) = self.editor.solver.initial_profile.as_mut() else {
            return;
        };
        ui.strong("Configured analytical initialization");
        match profile {
            workbench::initialization::InitialProfile::Uniform { velocity, pressure } => {
                ui.label("Uniform initial velocity and kinematic gauge pressure.");
                widgets::vector(ui, "Velocity (m/s)", velocity);
                widgets::number(ui, "Pressure p/ρ", pressure);
            }
            workbench::initialization::InitialProfile::BoundaryLayer2D {
                speed,
                wall_y,
                thickness,
                exponent,
                pressure,
            } => {
                ui.label("Approximate X-direction power-law initial guess; it does not impose an inlet profile.");
                widgets::number(ui, "Freestream speed (m/s)", speed);
                widgets::number(ui, "Wall Y (m)", wall_y);
                widgets::number(ui, "Boundary-layer thickness (m)", thickness);
                widgets::number(ui, "Power exponent", exponent);
                widgets::number(ui, "Pressure p/ρ", pressure);
            }
            workbench::initialization::InitialProfile::TaylorGreen2D { length, speed } => {
                ui.label("Periodic 2D Taylor–Green vortex. Evaluated at cell centres at physical time zero.");
                widgets::number(ui, "Period length (m)", length);
                widgets::number(ui, "Velocity amplitude (m/s)", speed);
            }
        }
        if ui
            .add_enabled(
                !self.busy(),
                egui::Button::new("Initialize analytical fields"),
            )
            .clicked()
        {
            let initialized = (|| -> Result<(), String> {
                self.apply_editor()?;
                let profile = self
                    .project
                    .solver
                    .initial_profile
                    .as_ref()
                    .ok_or("No analytical initial profile")?;
                let mesh = &self.session.mesh().ok_or("Generate a mesh first")?.mesh;
                let fields = profile.fields(mesh)?;
                let id = mesh.id();
                self.reset_solve_session();
                self.jobs.set_initial_fields(id, fields);
                self.session.initialize_solution();
                self.result = None;
                self.result_cache = None;
                self.result_run.clear();
                self.probes.clear();
                self.streamlines.clear();
                self.histories = Default::default();
                self.plane_snapshot = None;
                self.plane_frames.clear();
                self.plane_playing = false;
                self.iteration = 0;
                self.initialized = true;
                self.persist_continuation()?;
                self.show_initialization = false;
                Ok(())
            })();
            match initialized {
                Ok(()) => self.message(
                    "Analytical fields initialized; accepted iteration and physical time reset.",
                ),
                Err(e) => self.message(e),
            }
        }
        ui.separator();
    }
}
