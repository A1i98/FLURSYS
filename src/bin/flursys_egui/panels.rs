use super::*;
use flursys::unstructured_incompressible::{AdaptiveTimeStep, TimeIntegration};
use widgets::{heading, icon_button, number, vector};
impl App {
    pub fn shell(&mut self, ui: &mut egui::Ui) {
        let ctx = ui.ctx().clone();
        if self.first_layout {
            self.browser_open = self.preferences.browser && ui.available_width() >= 1000.;
            self.properties_open = self.preferences.properties;
            self.show_log = self.preferences.messages;
            self.first_layout = false;
        }
        egui::Panel::top("application-bar").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.add(
                    egui::Image::new(egui::include_image!("../../../ui/assets/flursys-logo.svg"))
                        .fit_to_exact_size(egui::vec2(30., 30.)),
                );
                ui.label(
                    RichText::new("FLURSYS")
                        .strong()
                        .size(20.)
                        .color(widgets::ACCENT),
                );
                if ui.available_width() > 850. {
                    ui.weak("CFD WORKBENCH");
                }
                ui.label(RichText::new(concat!("v", env!("CARGO_PKG_VERSION"))).small());
                ui.separator();
                ui.menu_button("File", |ui| {
                    if ui.button("New workspace").clicked() {
                        self.request(Pending::New);
                        ui.close();
                    }
                    if ui.button("Open workspace…").clicked() {
                        if let Some(p) = rfd::FileDialog::new().pick_folder() {
                            self.request(Pending::Open(p));
                        }
                        ui.close();
                    }
                    if ui.button("Save workspace").clicked() {
                        if let Err(e) = self.save() {
                            self.message(e);
                        }
                        ui.close();
                    }
                    if ui.button("Examples…").clicked() {
                        self.show_examples = true;
                        ui.close();
                    }
                    if ui.button("Quit").clicked() {
                        self.request(Pending::Quit);
                        ui.close();
                    }
                });
                ui.menu_button("View", |ui| {
                    if ui
                        .checkbox(&mut self.browser_open, "Project browser")
                        .changed()
                    {
                        self.browser_requested = self.browser_open;
                    }
                    ui.checkbox(&mut self.properties_open, "Properties");
                    ui.checkbox(&mut self.show_log, "Messages");
                });
                if ui.button("Settings").clicked() {
                    self.show_preferences = true;
                }
                ui.menu_button("Tools", |ui| {
                    if ui.button("Project configuration…").clicked() {
                        self.sync_project();
                        self.json_editor =
                            serde_json::to_string_pretty(&self.project).unwrap_or_default();
                        self.show_advanced = true;
                        ui.close();
                    }
                });
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.add(
                        egui::Label::new(
                            RichText::new(format!(
                                "{}{}",
                                self.project.name,
                                if self.dirty || self.cad.session.is_dirty() {
                                    " *"
                                } else {
                                    ""
                                }
                            ))
                            .weak(),
                        )
                        .truncate(),
                    );
                });
            });
            ui.add_space(3.);
            ui.horizontal_wrapped(|ui| {
                for stage in Stage::ALL {
                    let selected = self.stage == stage;
                    if ui
                        .add_sized(
                            [100., 30.],
                            egui::Button::new(RichText::new(stage.label()).strong())
                                .selected(selected),
                        )
                        .clicked()
                    {
                        self.stage(stage);
                    }
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if icon_button(ui, "save", "", !self.busy())
                        .on_hover_text("Save workspace")
                        .clicked()
                    {
                        if let Err(e) = self.save() {
                            self.message(e);
                        }
                    }
                    if self.stage != Stage::Solution
                        && self.jobs.is_running()
                        && icon_button(ui, "stop", "Stop job", true).clicked()
                    {
                        self.jobs.cancel();
                        self.message("Cancellation requested.");
                    }
                    if self.busy() {
                        ui.spinner();
                    }
                });
            });
        });
        egui::Panel::bottom("status-bar").show(ui, |ui| {
            ui.horizontal(|ui| {
                let status = if self.jobs.is_running() {
                    if self.jobs.is_paused() {
                        "Paused"
                    } else {
                        "Running"
                    }
                } else {
                    match self.session.status() {
                        SolveStatus::Converged => "Converged",
                        _ => "Ready",
                    }
                };
                ui.label(RichText::new(status).color(widgets::ACCENT));
                ui.separator();
                ui.label(
                    self.session
                        .mesh()
                        .map(|m| format!("{} cells", m.mesh.cell_count()))
                        .unwrap_or_else(|| "No mesh".into()),
                );
                ui.separator();
                ui.label(format!("Iteration / step {}", self.iteration));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.small(
                        self.workspace
                            .as_ref()
                            .map(|p| p.display().to_string())
                            .unwrap_or_else(|| "Unsaved workspace".into()),
                    );
                });
            });
        });
        if self.show_log {
            egui::Panel::bottom("messages")
                .default_size(180.)
                .resizable(true)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.strong("Messages");
                        if ui.small_button("Clear").clicked() {
                            self.log.clear();
                        }
                    });
                    egui::ScrollArea::vertical()
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for line in &self.log {
                                ui.monospace(line);
                            }
                        });
                });
        }
        if self.stage == Stage::Geometry {
            ui.add_enabled_ui(self.publishing.is_none() && self.loading.is_none(), |ui| {
                self.cad.ui(ui)
            });
        } else {
            let compact = ui.available_width() < 1000.;
            if compact && !self.browser_requested {
                self.browser_open = false;
            }
            if self.browser_open && !compact {
                egui::Panel::left("project-browser")
                    .default_size(210.)
                    .size_range(160.0..=350.0)
                    .resizable(true)
                    .show(ui, |ui| {
                        self.browser(ui);
                    });
            }
            if self.browser_open && compact {
                let mut open = true;
                egui::Window::new("Project browser")
                    .open(&mut open)
                    .default_width(240.)
                    .show(&ctx, |ui| self.browser(ui));
                self.browser_open = open;
            }
            if self.properties_open {
                egui::Panel::left("properties")
                    .default_size(if compact { 280. } else { 330. })
                    .size_range(270.0..=560.0)
                    .resizable(true)
                    .show(ui, |ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(8., 9.);
                        ui.spacing_mut().interact_size.y = 28.;
                        ui.spacing_mut().combo_width = (ui.available_width() - 20.).max(120.);
                        ui.spacing_mut().text_edit_width = (ui.available_width() - 20.).max(120.);
                        let pages = self.pages();
                        ui.horizontal_wrapped(|ui| {
                            for (i, name) in pages.iter().enumerate() {
                                let i = if self.stage == Stage::Results && i >= 1
                                    || self.stage == Stage::Solution && i >= 8
                                {
                                    i + 1
                                } else {
                                    i
                                };
                                if ui.selectable_label(self.page == i, *name).clicked() {
                                    self.page = i;
                                    if self.stage == Stage::Solution
                                        && i >= 7
                                        && self.result.is_none()
                                    {
                                        self.ensure_result();
                                    }
                                    if self.stage == Stage::Mesh {
                                        self.show_geometry = i == 0;
                                    }
                                }
                            }
                        });
                        ui.separator();
                        let busy = self.busy();
                        egui::ScrollArea::vertical()
                            .id_salt(("properties", self.stage, self.page))
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                ui.add_enabled_ui(
                                    !busy
                                        || self.stage == Stage::Results
                                        || (self.stage == Stage::Solution && self.page >= 7),
                                    |ui| self.properties(ui),
                                );
                            });
                    });
            }
            egui::CentralPanel::default().show(ui, |ui| {
                if self.transport.receiver.is_some() {
                    ui.horizontal(|ui| {
                        if ui.button("Pause transport").clicked() {
                            self.transport
                                .paused
                                .store(true, std::sync::atomic::Ordering::Release);
                        }
                        if ui.button("Resume transport").clicked() {
                            self.transport
                                .paused
                                .store(false, std::sync::atomic::Ordering::Release);
                        }
                        if ui.button("Stop transport").clicked() {
                            self.transport
                                .stopped
                                .store(true, std::sync::atomic::Ordering::Release);
                        }
                    });
                }
                if self.stage == Stage::Mesh {
                    egui::Panel::bottom("mesh-actions").show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            if icon_button(ui, "mesh", "Generate mesh", !self.busy()).clicked() { self.generate_mesh(); }
                            if ui.add_enabled(!self.busy(), egui::Button::new("Update settings")).clicked() {
                                match self.apply_editor() { Ok(()) => { self.refresh_mesh(); self.message("Mesh settings updated; generate to apply sizing to cells."); }, Err(e) => self.message(e) }
                            }
                        });
                    });
                }
                if self.stage == Stage::Solution {
                    egui::Panel::bottom("solution-controls")
                        .resizable(false)
                        .show(ui, |ui| {
                            ui.add_space(6.);
                            ui.horizontal_wrapped(|ui| {
                                ui.label(
                                    RichText::new(format!("Iteration {}", self.iteration)).strong(),
                                );
                                ui.separator();
                                match &mut self.editor.solver.time {
                                    TimeIntegration::Steady => {
                                        ui.label("Additional iterations");
                                        ui.add_enabled(!self.jobs.is_running(),egui::DragValue::new(&mut self.editor.solver.max_outer_iterations).range(1..=1_000_000));
                                    },
                                    TimeIntegration::Euler{steps,..}|TimeIntegration::Backward{steps,..}|TimeIntegration::AdaptiveEuler{steps,..}|TimeIntegration::AdaptiveBackward{steps,..} => {
                                        ui.label("Additional time steps");
                                        ui.add_enabled(!self.jobs.is_running(),egui::DragValue::new(steps).range(1..=1_000_000));
                                    }
                                }
                            });
                            let total=self.limit.saturating_sub(self.segment_start);
                            let accepted=self.iteration.saturating_sub(self.segment_start).min(total);
                            if total>0 {
                                ui.add(egui::ProgressBar::new(accepted as f32/total as f32).show_percentage().text(format!("{} / {} · target {} · {:.1}%",accepted,total,self.limit,100.*accepted as f64/total as f64)).desired_width(ui.available_width()));
                            }
                            if total==0 { ui.weak(format!("Next target: {}",self.iteration.saturating_add(self.project.solver.time.parameters().map(|(_,steps)|steps).unwrap_or(self.editor.solver.max_outer_iterations)))); }
                            self.run_controls(ui);
                            ui.add_space(4.);
                        });
                }
                egui::Panel::bottom("workspace-feedback")
                    .resizable(false)
                    .show(ui, |ui| {
                        ui.add(
                            egui::Label::new(
                                RichText::new(&self.feedback).small().color(widgets::ACCENT),
                            )
                            .wrap(),
                        );
                    });
                if self.stage == Stage::Results || (self.stage == Stage::Solution && self.page >= 7) {
                    egui::Panel::top("post-source").show(ui, |ui| self.post_toolbar(ui));
                }
                if (self.stage == Stage::Solution && self.page == 0)
                    || (self.stage == Stage::Results && self.page == 3)
                {
                    if self.stage == Stage::Results && self.migration.chart { self.profile_chart(ui); } else { self.convergence(ui); }
                } else if self.stage == Stage::Results && self.page == 4 {
                    if !self.force_series.is_empty() {
                        let series=self.force_series.clone();
                        let settings=&self.editor.solver.force_analysis;
                        let title=if settings.reattachment {"Signed wall shear (Pa)"} else {"Load coefficients"};
                        let axis=if settings.reattachment {"Distance (m)"} else if self.force_history.first().is_some_and(|s|s.time.is_some()) {"Physical time (s)"} else {"Iteration"};
                        self.native_series_plot(ui,&series,title,axis);
                    } else {
                        ui.centered_and_justified(|ui| {
                            ui.weak(
                                "Choose wall patches and Apply to build a load or shear chart.",
                            );
                        });
                    }
                } else {
                    self.viewport(ui);
                }
            });
        }
        self.windows(&ctx);
        if ctx.input(|i| i.modifiers.command && i.key_pressed(egui::Key::S)) {
            if let Err(e) = self.save() {
                self.message(e);
            }
        }
    }
    fn pages(&self) -> Vec<&'static str> {
        match self.stage {
            Stage::Geometry => vec![],
            Stage::Mesh => vec!["Boundaries", "Sizing", "Quality", "Method"],
            Stage::Setup => vec![
                "Boundaries",
                "Material",
                "Thermal",
                "Scripts",
                "Models",
                "Transport",
            ],
            Stage::Solution => vec![
                "Convergence",
                "Controls",
                "Time",
                "Models",
                "Methods",
                "Compute",
                "Reports",
                "Display",
                "Probes",
                "History",
            ],
            Stage::Results => vec![
                "Display",
                "Information",
                "Plots",
                "Wall loads",
                "Probes",
                "History",
            ],
        }
    }
    fn browser(&mut self, ui: &mut egui::Ui) {
        heading(ui, "PROJECT");
        egui::ScrollArea::vertical().show(ui, |ui| {
            egui::CollapsingHeader::new("Geometry")
                .default_open(true)
                .show(ui, |ui| {
                    for body in self.session.geometry().bodies() {
                        let target = GeometrySelectionTarget::Body(body.id);
                        if ui
                            .selectable_label(
                                self.selected.contains(&target),
                                format!("Body {}", body.id.get()),
                            )
                            .clicked()
                        {
                            self.selected = vec![target];
                        }
                    }
                    for face in self.session.geometry().faces() {
                        let target = GeometrySelectionTarget::Face(face.id);
                        if ui
                            .selectable_label(
                                self.selected.contains(&target),
                                format!("Face {}", face.id.get()),
                            )
                            .clicked()
                        {
                            self.selected = vec![target];
                        }
                    }
                });
            egui::CollapsingHeader::new("Boundary groups")
                .default_open(true)
                .show(ui, |ui| {
                    for group in &self.project.named_selections {
                        if ui
                            .selectable_label(
                                self.boundary_patch == group.name,
                                format!("{} · {} entities", group.name, group.targets.len()),
                            )
                            .clicked()
                        {
                            self.boundary_patch = group.name.clone();
                            self.selected = group.targets.clone();
                        }
                    }
                });
            for (stage, title) in [
                (Stage::Mesh, "Mesh"),
                (Stage::Setup, "Setup"),
                (Stage::Solution, "Solution"),
                (Stage::Results, "Results"),
            ] {
                if ui.selectable_label(self.stage == stage, title).clicked() {
                    self.stage = stage;
                    self.page = 0;
                }
            }
            egui::CollapsingHeader::new(format!("Runs · {}", self.project.runs.len()))
                .default_open(true)
                .show(ui, |ui| {
                    let mut run = None;
                    for r in &self.project.runs {
                        if ui
                            .selectable_label(
                                self.result_run == r.id,
                                format!("{} · {:?}", r.id, r.status),
                            )
                            .clicked()
                        {
                            run = Some(r.id.clone());
                        }
                    }
                    if let Some(run) = run {
                        if let Err(e) = self.open_run(&run) {
                            self.message(e);
                        }
                    }
                });
        });
    }
    fn properties(&mut self, ui: &mut egui::Ui) {
        match self.stage {
            Stage::Mesh => self.mesh_properties(ui),
            Stage::Setup => self.setup_properties(ui),
            Stage::Solution => self.solution_properties(ui),
            Stage::Results => self.results_properties(ui),
            _ => {}
        }
    }
    fn apply_button(&mut self, ui: &mut egui::Ui) {
        ui.add_space(10.);
        if icon_button(ui, "check", "Apply settings", !self.busy()).clicked() {
            if let Err(e) = self.apply_editor() {
                self.message(e);
            }
        }
    }
    fn mesh_properties(&mut self, ui: &mut egui::Ui) {
        match self.page {
            0 => {
                heading(ui, "FLUID DOMAIN");
                ui.label("Select a planar face for 2D, or a fluid body for 3D.");
                let mut dimension = self.session.mesh_dimension();
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut dimension, MeshDimension::TwoD, "2D · planar");
                    ui.selectable_value(&mut dimension, MeshDimension::ThreeD, "3D · volume");
                });
                if dimension != self.session.mesh_dimension() {
                    let mesh = &self.editor.mesh;
                    match self.session.set_mesh_configuration(
                        dimension,
                        mesh.global_size,
                        mesh.min_size,
                        mesh.max_size,
                        mesh.element_order,
                    ) {
                        Ok(()) => {
                            self.sync_project();
                            self.editor = self.project.clone();
                            self.selected.clear();
                            self.refresh_mesh();
                            self.dirty = true;
                        }
                        Err(e) => self.message(e.to_string()),
                    }
                }
                if dimension == MeshDimension::ThreeD {
                    ui.weak("The single visible fluid volume supplies its exterior faces. Select faces below to name boundaries.");
                }

                if dimension == MeshDimension::TwoD {
                    let faces: Vec<_> = self
                        .session
                        .geometry()
                        .faces()
                        .filter(|f| {
                            matches!(
                                f.representation,
                                flursys::GeometryFaceRepresentation::Planar { .. }
                            )
                        })
                        .map(|f| f.id)
                        .collect();
                    egui::ComboBox::from_id_salt("fluid-face")
                        .selected_text(
                            self.session
                                .fluid_face()
                                .map(|f| format!("Face {}", f.get()))
                                .unwrap_or_else(|| "Select fluid face".into()),
                        )
                        .show_ui(ui, |ui| {
                            for f in faces {
                                if ui
                                    .selectable_label(
                                        self.session.fluid_face() == Some(f),
                                        format!("Face {}", f.get()),
                                    )
                                    .clicked()
                                {
                                    match self.session.select_fluid_face(f) {
                                        Ok(()) => {
                                            self.sync_project();
                                            self.editor = self.project.clone();
                                            self.dirty = true;
                                        }
                                        Err(e) => self.message(e.to_string()),
                                    }
                                }
                            }
                        });
                    if ui.button("Use selected face as fluid domain").clicked() {
                        if let Some(GeometrySelectionTarget::Face(f)) = self.selected.first() {
                            let f = *f;
                            if let Err(e) = self.session.select_fluid_face(f) {
                                self.message(e.to_string());
                            } else {
                                self.sync_project();
                                self.editor = self.project.clone();
                                self.dirty = true;
                            }
                        }
                    }
                }
                heading(ui, "NAMED BOUNDARIES");
                ui.label(format!("{} selected", self.selected.len()));
                ui.small("Click to select. Shift/Ctrl adds or removes. Alt-drag selects a region.");
                ui.text_edit_singleline(&mut self.boundary_name);
                if ui
                    .add_enabled(
                        !self.selected.is_empty() && !self.boundary_name.trim().is_empty(),
                        egui::Button::new(
                            if self
                                .project
                                .named_selections
                                .iter()
                                .any(|g| g.name == self.boundary_name.trim())
                            {
                                "Replace boundary group"
                            } else {
                                "Create boundary group"
                            },
                        ),
                    )
                    .clicked()
                {
                    match self.session.set_boundary_group(
                        &self.boundary_name,
                        self.selected.clone(),
                        self.project
                            .named_selections
                            .iter()
                            .any(|g| g.name == self.boundary_name.trim()),
                    ) {
                        Ok(()) => {
                            self.sync_project();
                            self.editor = self.project.clone();
                            self.refresh_mesh();
                            self.dirty = true;
                            self.selected.clear();
                            self.message(
                                "Boundary group updated. Assign its physical condition in Setup.",
                            );
                        }
                        Err(e) => self.message(e.to_string()),
                    }
                }
                if ui.button("Select ungrouped boundaries").clicked() {
                    match self.session.boundary_geometry_targets() {
                        Ok(all) => {
                            self.selected = all
                                .into_iter()
                                .filter(|t| {
                                    !self
                                        .project
                                        .named_selections
                                        .iter()
                                        .any(|g| g.targets.contains(t))
                                })
                                .collect();
                        }
                        Err(e) => self.message(e.to_string()),
                    }
                }
                if !self.project.named_selections.is_empty() {
                    heading(ui, "BOUNDARY GROUPS");
                }
                for group in &self.project.named_selections {
                    if ui
                        .selectable_label(
                            self.boundary_name.trim() == group.name,
                            format!("{} · {} entities", group.name, group.targets.len()),
                        )
                        .clicked()
                    {
                        self.boundary_name = group.name.clone();
                        self.selected = group.targets.clone();
                    }
                }
                if self
                    .project
                    .named_selections
                    .iter()
                    .any(|g| g.name == self.boundary_name.trim())
                    && ui.button("Remove selected boundary group").clicked()
                {
                    if self
                        .session
                        .delete_named_selection(self.boundary_name.trim())
                    {
                        self.sync_project();
                        self.editor = self.project.clone();
                        self.refresh_mesh();
                        self.selected.clear();
                        self.dirty = true;
                        self.message("Boundary group removed; its physical condition was cleared.");
                    }
                }
                ui.add(egui::Label::new(&self.feedback).wrap());
                if let Err(e) = self.session.mesh_generation_inputs() {
                    widgets::error(ui, &e.to_string());
                }
            }
            1 => {
                heading(ui, "GLOBAL SIZING");
                ui.horizontal(|ui| {
                    for (name, divisions) in [("Coarse", 18.), ("Medium", 36.), ("Fine", 72.)] {
                        if ui.button(name).clicked() {
                            let points: Vec<_> = self
                                .geometry_faces
                                .iter()
                                .flat_map(|f| f.vertices.iter())
                                .collect();
                            let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
                            let mut hi =
                                Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
                            for p in points {
                                lo.x = lo.x.min(p.x);
                                lo.y = lo.y.min(p.y);
                                lo.z = lo.z.min(p.z);
                                hi.x = hi.x.max(p.x);
                                hi.y = hi.y.max(p.y);
                                hi.z = hi.z.max(p.z);
                            }
                            let extent = (hi.x - lo.x).max(hi.y - lo.y).max(hi.z - lo.z);
                            if extent.is_finite() && extent > 0. {
                                let h = extent / divisions;
                                self.editor.mesh.global_size = h;
                                self.editor.mesh.min_size = h * 0.25;
                                self.editor.mesh.max_size = h;
                                self.dirty = true;
                            }
                        }
                    }
                });
                ui.weak("Presets use the model bounding dimensions in metres. Existing local controls remain explicit.");

                number(ui, "Target size (m)", &mut self.editor.mesh.global_size);
                number(ui, "Minimum (m)", &mut self.editor.mesh.min_size);
                number(ui, "Maximum (m)", &mut self.editor.mesh.max_size);
                ui.horizontal(|ui| {
                    ui.label("CPU threads (0 = automatic)");
                    ui.add(egui::DragValue::new(&mut self.editor.mesh.threads).range(0..=256));
                });
                heading(ui, "LOCAL CONTROLS");
                let names: Vec<_> = self
                    .project
                    .named_selections
                    .iter()
                    .map(|g| g.name.clone())
                    .collect();
                egui::ComboBox::from_id_salt("mesh-zone")
                    .selected_text(if self.boundary_patch.is_empty() {
                        "Select zone"
                    } else {
                        &self.boundary_patch
                    })
                    .show_ui(ui, |ui| {
                        for name in names {
                            ui.selectable_value(&mut self.boundary_patch, name.clone(), name);
                        }
                    });
                if !self.boundary_patch.is_empty() {
                    let target = vec![MeshControlTarget::NamedSelection(
                        self.boundary_patch.clone(),
                    )];
                    if ui.button("Add local sizing").clicked() {
                        let id = self
                            .editor
                            .mesh_recipe
                            .local_sizes
                            .iter()
                            .map(|c| c.id)
                            .max()
                            .unwrap_or(0)
                            + 1;
                        self.editor.mesh_recipe.local_sizes.push(LocalMeshSize {
                            id,
                            name: format!("Size · {}", self.boundary_patch),
                            targets: target.clone(),
                            size: self.editor.mesh.min_size,
                            enabled: true,
                        });
                    }
                    if ui.button("Add inflation layers").clicked() {
                        let id = self
                            .editor
                            .mesh_recipe
                            .boundary_layers
                            .iter()
                            .map(|c| c.id)
                            .max()
                            .unwrap_or(0)
                            + 1;
                        self.editor
                            .mesh_recipe
                            .boundary_layers
                            .push(BoundaryLayerControl {
                                id,
                                name: format!("Inflation · {}", self.boundary_patch),
                                targets: target.clone(),
                                first_layer_height: self.editor.mesh.min_size * 0.2,
                                growth_ratio: 1.2,
                                layer_count: 5,
                                enabled: true,
                            });
                    }
                    if ui.button("Add distance refinement").clicked() {
                        let id = self
                            .editor
                            .mesh_recipe
                            .refinements
                            .iter()
                            .map(|c| c.id)
                            .max()
                            .unwrap_or(0)
                            + 1;
                        self.editor
                            .mesh_recipe
                            .refinements
                            .push(ThresholdRefinement {
                                id,
                                name: format!("Refine · {}", self.boundary_patch),
                                targets: target,
                                sampling: 100,
                                size_min: self.editor.mesh.min_size,
                                size_max: self.editor.mesh.max_size,
                                distance_min: 0.,
                                distance_max: self.editor.mesh.global_size * 5.,
                                enabled: true,
                            });
                    }
                }
                self.layer_tools(ui);
                let mut value = serde_json::to_value(&self.editor.mesh_recipe).unwrap_or_default();
                widgets::fields(ui, "mesh-recipe", &mut value);
                if let Ok(recipe) = serde_json::from_value(value) {
                    self.editor.mesh_recipe = recipe;
                }
                self.apply_button(ui);
            }
            2 => {
                self.quality_properties(ui);
            }
            _ => {
                heading(ui, "MESHING METHOD");
                use workbench::MeshMethod as M;
                egui::ComboBox::from_id_salt("mesh-method")
                    .selected_text(format!("{:?}", self.editor.mesh_recipe.tuning.method))
                    .show_ui(ui, |ui| {
                        for (m, n) in [
                            (M::Unstructured, "Unstructured"),
                            (M::QuadDominant, "Quad dominant"),
                            (M::MappedQuad, "Mapped quadrilateral"),
                            (M::OGrid, "O-grid"),
                            (M::HexBlock, "Structured hexahedral block"),
                            (M::SweepPrism, "Sweep prisms"),
                            (M::SweepHex, "Sweep hexahedra"),
                        ] {
                            ui.selectable_value(&mut self.editor.mesh_recipe.tuning.method, m, n);
                        }
                    });

                let mut v =
                    serde_json::to_value(&self.editor.mesh_recipe.tuning).unwrap_or_default();
                widgets::fields(ui, "mesh-tuning", &mut v);
                if let Ok(t) = serde_json::from_value(v) {
                    self.editor.mesh_recipe.tuning = t;
                }
                self.apply_button(ui);
            }
        }
    }
    fn setup_properties(&mut self, ui: &mut egui::Ui) {
        match self.page {
            0 => {
                heading(ui, "BOUNDARY CONDITIONS");
                let names: Vec<_> = self
                    .project
                    .named_selections
                    .iter()
                    .map(|g| g.name.clone())
                    .collect();
                for name in names {
                    let assigned = self.session.boundary_assignment(&name).is_some();
                    if ui
                        .selectable_label(
                            self.boundary_patch == name,
                            format!(
                                "{name} · {}",
                                if assigned { "assigned" } else { "unassigned" }
                            ),
                        )
                        .clicked()
                    {
                        self.boundary_patch = name;
                        self.read_boundary();
                    }
                }
                ui.separator();
                ui.strong(&self.boundary_patch);
                const BC: [&str; 12] = [
                    "No-slip wall",
                    "Moving wall",
                    "Velocity inlet",
                    "Pressure outlet",
                    "Pressure outlet / backflow",
                    "Slip",
                    "Symmetry",
                    "Far field",
                    "Total pressure",
                    "Parabolic inlet",
                    "Volume flow inlet",
                    "Mass flow inlet",
                ];
                egui::ComboBox::from_id_salt("boundary-kind")
                    .selected_text(BC[self.boundary_kind.min(11)])
                    .show_ui(ui, |ui| {
                        for (i, n) in BC.iter().enumerate() {
                            ui.selectable_value(&mut self.boundary_kind, i, *n);
                        }
                    });
                if matches!(self.boundary_kind, 1 | 2 | 4 | 7 | 9) {
                    vector(ui, "Velocity (m/s)", &mut self.boundary_velocity);
                }
                if matches!(self.boundary_kind, 3 | 4 | 7 | 8) {
                    number(
                        ui,
                        "Kinematic pressure p/ρ (m²/s²)",
                        &mut self.boundary_pressure,
                    );
                }
                if self.boundary_kind >= 10 {
                    number(ui, "Positive inlet flow rate", &mut self.boundary_pressure);
                }
                if ui
                    .add_enabled(
                        !self.boundary_patch.is_empty(),
                        egui::Button::new("Apply physical condition"),
                    )
                    .clicked()
                {
                    let v = self.boundary_velocity;
                    let p = self.boundary_pressure;
                    use IncompressibleBoundaryCondition as B;
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
                            axis: 1,
                            center: 0.5,
                            half_width: 0.5,
                        },
                        _ => B::FlowRateInlet {
                            rate: p,
                            mass: self.boundary_kind == 11,
                        },
                    };
                    match self
                        .session
                        .configure_named_boundary(&self.boundary_patch, condition)
                    {
                        Ok(()) => {
                            self.sync_project();
                            self.editor = self.project.clone();
                            self.dirty = true;
                            self.message("Boundary condition assigned.");
                        }
                        Err(e) => self.message(e.to_string()),
                    }
                }
                self.boundary_table_editor(ui);
                heading(ui, "PATCH DETAILS");
                let mut p = serde_json::to_value(&self.editor).unwrap_or_default();
                for key in ["boundaries"] {
                    if let Some(v) = p.get_mut(key) {
                        widgets::fields(ui, key, v);
                    }
                }
                if let Ok(v) = serde_json::from_value::<WorkbenchProject>(p) {
                    self.editor.boundaries = v.boundaries;
                }
                self.apply_button(ui);
            }
            1 => {
                heading(ui, "FLUID MATERIAL");
                if ui.button("Open material catalogue").clicked() {
                    match flursys::database::Database::open_default()
                        .and_then(|db| db.entries(false))
                    {
                        Ok(entries) => self.materials = entries,
                        Err(e) => self.message(e.to_string()),
                    }
                }
                if !self.materials.is_empty() {
                    egui::ComboBox::from_id_salt("catalogue-material")
                        .selected_text(if self.material_id.is_empty() {
                            "Select fluid"
                        } else {
                            &self.material_id
                        })
                        .show_ui(ui, |ui| {
                            for m in &self.materials {
                                ui.selectable_value(&mut self.material_id, m.id.clone(), &m.name);
                            }
                        });
                    number(
                        ui,
                        "Evaluation temperature (K)",
                        &mut self.material_temperature,
                    );
                    number(ui, "Evaluation pressure (Pa)", &mut self.material_pressure);
                    if ui.button("Evaluate and apply material").clicked() {
                        match flursys::database::Database::open_default()
                            .and_then(|db| {
                                db.evaluate(
                                    &self.material_id,
                                    self.material_temperature,
                                    self.material_pressure,
                                )
                            })
                            .map_err(|e| e.to_string())
                            .and_then(|m| {
                                self.session
                                    .apply_catalog_material(m)
                                    .map_err(|e| e.to_string())
                            }) {
                            Ok(()) => {
                                self.sync_project();
                                self.editor = self.project.clone();
                                self.dirty = true;
                                self.message(
                                    "Material properties evaluated and applied with provenance.",
                                );
                            }
                            Err(e) => self.message(e),
                        }
                    }
                }

                number(ui, "Density (kg/m³)", &mut self.editor.material.density);
                number(
                    ui,
                    "Kinematic viscosity (m²/s)",
                    &mut self.editor.material.kinematic_viscosity,
                );
                self.apply_button(ui);
            }
            2 => {
                heading(ui, "THERMAL AND BUOYANCY");
                ui.checkbox(&mut self.editor.solver.thermal.enabled, "Solve temperature");
                if self.editor.solver.thermal.enabled {
                    for patch in &self.project.named_selections {
                        self.editor
                            .solver
                            .thermal
                            .boundaries
                            .entry(patch.name.clone())
                            .or_insert(flursys::thermal::Boundary::Adiabatic);
                    }
                    ui.checkbox(
                        &mut self.editor.solver.thermal.buoyancy,
                        "Boussinesq buoyancy",
                    );
                    vector(
                        ui,
                        "Gravity (m/s²)",
                        &mut self.editor.solver.thermal.gravity,
                    );
                    for (patch, boundary) in &mut self.editor.solver.thermal.boundaries {
                        egui::CollapsingHeader::new(format!("Thermal boundary · {patch}"))
                            .id_salt(("thermal-bc", patch))
                            .show(ui, |ui| widgets::thermal_boundary(ui, patch, boundary));
                    }
                    let mut v =
                        serde_json::to_value(&self.editor.solver.thermal).unwrap_or_default();
                    let boundaries = v.as_object_mut().and_then(|v| v.remove("boundaries"));
                    widgets::fields(ui, "thermal", &mut v);
                    if let Some(b) = boundaries {
                        v["boundaries"] = b;
                    }
                    if let Ok(t) = serde_json::from_value(v) {
                        self.editor.solver.thermal = t;
                    }
                }
                self.apply_button(ui);
            }
            3 => self.scripts(ui),
            5 => self.transport_properties(ui),
            _ => {
                self.models(ui);
                self.apply_button(ui);
            }
        }
    }
    fn read_boundary(&mut self) {
        use IncompressibleBoundaryCondition as B;
        if let Some(b) = self.session.boundary_assignment(&self.boundary_patch) {
            self.boundary_kind = match *b {
                B::NoSlipWall => 0,
                B::MovingWall { velocity } => {
                    self.boundary_velocity = velocity;
                    1
                }
                B::VelocityInlet { velocity } => {
                    self.boundary_velocity = velocity;
                    2
                }
                B::PressureOutlet { pressure } => {
                    self.boundary_pressure = pressure;
                    3
                }
                B::PressureOutletBackflow { pressure, velocity } => {
                    self.boundary_velocity = velocity;
                    self.boundary_pressure = pressure;
                    4
                }
                B::Slip => 5,
                B::Symmetry => 6,
                B::FarField { velocity, pressure } => {
                    self.boundary_velocity = velocity;
                    self.boundary_pressure = pressure;
                    7
                }
                B::TotalPressure { pressure } => {
                    self.boundary_pressure = pressure;
                    8
                }
                B::ParabolicInlet { velocity, .. } => {
                    self.boundary_velocity = velocity;
                    9
                }
                B::FlowRateInlet { rate, mass } => {
                    self.boundary_pressure = rate;
                    if mass {
                        11
                    } else {
                        10
                    }
                }
            };
        }
    }
    fn scripts(&mut self, ui: &mut egui::Ui) {
        heading(ui, "BOUNDARY SCRIPTS · RHAI");
        if ui.button("Add script").clicked() {
            let name = format!("profile-{}", self.editor.solver.scripting.scripts.len() + 1);
            self.editor.solver.scripting.scripts.insert(
                name,
                flursys::scripting::Script {
                    source: "fn profile(x, y, z) { vec3(1.0, 0.0, 0.0) }".into(),
                    time_dependent: false,
                },
            );
        }
        for (name, s) in &mut self.editor.solver.scripting.scripts {
            ui.strong(name);
            ui.checkbox(&mut s.time_dependent, "Physical-time argument t");
            ui.add(
                egui::TextEdit::multiline(&mut s.source)
                    .code_editor()
                    .desired_rows(12)
                    .desired_width(f32::INFINITY),
            );
        }
        heading(ui, "CONNECT TO PATCH");
        let names: Vec<_> = self
            .editor
            .solver
            .scripting
            .scripts
            .keys()
            .cloned()
            .collect();
        if !names.is_empty() {
            egui::ComboBox::from_id_salt("script-patch")
                .selected_text(&self.boundary_patch)
                .show_ui(ui, |ui| {
                    for p in &self.project.named_selections {
                        ui.selectable_value(&mut self.boundary_patch, p.name.clone(), &p.name);
                    }
                });
            for name in names {
                if ui.button(format!("Bind {name} to velocity")).clicked() {
                    self.editor.solver.scripting.bindings.insert(
                        self.boundary_patch.clone(),
                        vec![flursys::scripting::Binding {
                            script: name,
                            function: "profile".into(),
                            quantity: flursys::scripting::Quantity::Velocity,
                            parameters: Default::default(),
                        }],
                    );
                }
            }
        }
        let mut value = serde_json::to_value(&self.editor.solver.scripting).unwrap_or_default();
        if let Some(bindings) = value.get_mut("bindings") {
            widgets::fields(ui, "script-bindings", bindings);
        }
        if let Ok(s) = serde_json::from_value(value) {
            self.editor.solver.scripting = s;
        }
        self.apply_button(ui);
    }
    fn models(&mut self, ui: &mut egui::Ui) {
        heading(ui, "FLOW MODEL");
        use flursys::turbulence::TurbulenceModel as T;
        let names = [
            (T::Laminar, "Laminar"),
            (T::KOmega, "k–ω"),
            (T::KOmegaSst, "k–ω SST"),
            (T::KEpsilon, "k–ε"),
            (T::RngKEpsilon, "RNG k–ε"),
            (T::RealizableKEpsilon, "Realizable k–ε"),
            (T::SpalartAllmaras, "Spalart–Allmaras"),
            (T::Smagorinsky, "LES Smagorinsky"),
            (T::Wale, "LES WALE"),
        ];
        egui::ComboBox::from_id_salt("turbulence-model")
            .selected_text(format!("{:?}", self.editor.solver.turbulence.model))
            .show_ui(ui, |ui| {
                for (value, label) in names {
                    ui.selectable_value(&mut self.editor.solver.turbulence.model, value, label);
                }
            });
        let mut v = serde_json::to_value(&self.editor.solver.turbulence).unwrap_or_default();
        widgets::fields(ui, "turbulence", &mut v);
        if let Ok(t) = serde_json::from_value(v) {
            self.editor.solver.turbulence = t;
        }
    }
    fn solution_properties(&mut self, ui: &mut egui::Ui) {
        if self.page >= 7 {
            let saved = self.page;
            self.page = match saved {
                7 => 0,
                8 => 1,
                9 => 5,
                _ => 6,
            };
            self.results_properties(ui);
            self.page = saved;
            return;
        }
        match self.page {
            0 => {
                heading(ui, "CONVERGENCE");
                ui.label(format!(
                    "Active solve: {}",
                    self.continuation_run
                        .as_deref()
                        .unwrap_or("new initialized session")
                ));
                ui.weak("Curve visibility, logarithmic scale and normalization are available above the chart. Hover to inspect raw sample values.");
                ui.label(format!(
                    "Iteration / step {} · active budget {}",
                    self.iteration, self.editor.solver.max_outer_iterations
                ));
                ui.label("Monitors have different definitions. Continuity alone does not establish convergence.");
                if let Some(s) = self.session.solution() {
                    ui.label(format!(
                        "Final continuity {:.6e}",
                        s.report.final_continuity_rms
                    ));
                }
                if ui.button("Export convergence CSV…").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .set_file_name("convergence.csv")
                        .save_file()
                    {
                        let series = self.convergence_series();
                        if let Err(e) = std::fs::write(
                            path,
                            workbench::series_plot::csv(&series, "Iteration / step"),
                        ) {
                            self.message(e.to_string());
                        }
                    }
                }
            }
            1 => {
                heading(ui, "SOLVER CONTROLS");
                ui.horizontal(|ui| {
                    ui.label("Additional iterations / inner correctors");
                    ui.add(
                        egui::DragValue::new(&mut self.editor.solver.max_outer_iterations)
                            .range(1..=1_000_000),
                    );
                });
                number(
                    ui,
                    "Velocity relaxation",
                    &mut self.editor.solver.velocity_relaxation,
                );
                number(
                    ui,
                    "Pressure relaxation",
                    &mut self.editor.solver.pressure_relaxation,
                );
                number(
                    ui,
                    "Continuity tolerance",
                    &mut self.editor.solver.continuity_absolute_tolerance,
                );
                number(
                    ui,
                    "Coupling tolerance",
                    &mut self.editor.solver.coupling_relative_tolerance,
                );
                ui.horizontal(|ui| {
                    ui.label("CPU threads (0 = automatic)");
                    ui.add(egui::DragValue::new(&mut self.editor.solver.threads).range(0..=256));
                });
                self.apply_button(ui);
            }
            2 => {
                self.time(ui);
                self.apply_button(ui);
            }
            3 => {
                self.models(ui);
                self.apply_button(ui);
            }
            4 => {
                heading(ui, "NUMERICAL METHODS");
                self.advanced_physics_editor(ui);
                use flursys::momentum::MomentumConvection as C;
                egui::ComboBox::from_id_salt("convection")
                    .selected_text(format!("{:?}", self.editor.solver.convection))
                    .show_ui(ui, |ui| {
                        for (c, n) in [
                            (C::Upwind, "First-order upwind"),
                            (C::LimitedLinear, "Limited linear"),
                            (C::Central, "Central (unbounded)"),
                            (C::VanLeer, "Van Leer"),
                        ] {
                            ui.selectable_value(&mut self.editor.solver.convection, c, n);
                        }
                    });
                use flursys::simple::CouplingAlgorithm as A;
                egui::ComboBox::from_id_salt("coupling")
                    .selected_text(format!(
                        "{:?}",
                        self.editor.solver.pressure_coupling.algorithm
                    ))
                    .show_ui(ui, |ui| {
                        for (a, n) in [
                            (A::Simple, "SIMPLE"),
                            (A::Simplec, "SIMPLEC"),
                            (A::Pimple, "PIMPLE"),
                        ] {
                            ui.selectable_value(
                                &mut self.editor.solver.pressure_coupling.algorithm,
                                a,
                                n,
                            );
                        }
                    });
                let mut value = serde_json::to_value(&self.editor.solver).unwrap_or_default();
                if let Some(map) = value
                    .get_mut("pressure_coupling")
                    .and_then(|v| v.as_object_mut())
                {
                    map.remove("algorithm");
                }
                let mut per_equation = self.editor.solver.equations.is_some();
                if ui
                    .checkbox(&mut per_equation, "Per-equation linear solver controls")
                    .changed()
                {
                    self.editor.solver.equations =
                        per_equation.then(flursys::equations::EquationSolvers::default);
                    value["equations"] =
                        serde_json::to_value(&self.editor.solver.equations).unwrap_or_default();
                }
                for key in ["pressure_coupling", "equations"] {
                    if let Some(v) = value.get_mut(key).filter(|v| !v.is_null()) {
                        heading(ui, &widgets::label(key));
                        widgets::fields(ui, key, v);
                    }
                }
                value["pressure_coupling"]["algorithm"] =
                    serde_json::to_value(self.editor.solver.pressure_coupling.algorithm)
                        .unwrap_or_default();
                if let Ok(mut s) =
                    serde_json::from_value::<workbench::WorkbenchSolverSettings>(value)
                {
                    s.pressure_coupling.algorithm = self.editor.solver.pressure_coupling.algorithm;
                    self.editor.solver = s;
                }
                self.apply_button(ui);
            }
            5 => {
                self.compute_properties(ui);
                self.apply_button(ui);
            }
            _ => {
                heading(ui, "RUN REPORT");
                ui.label(format!("{:?}", self.session.status()));
                if let Some(s) = self.session.solution() {
                    match &s.budgets {
                        Ok(b) => {
                            ui.label(b.summary());
                        }
                        Err(e) => {
                            widgets::error(ui, e);
                        }
                    }
                }
                if let Some(p) = self.session.performance() {
                    ui.monospace(format!("{p:#?}"));
                }
            }
        }
    }
    fn time(&mut self, ui: &mut egui::Ui) {
        heading(ui, "TIME INTEGRATION");
        let mut mode = match self.editor.solver.time {
            TimeIntegration::Steady => 0,
            TimeIntegration::Euler { .. } => 1,
            TimeIntegration::Backward { .. } => 2,
            TimeIntegration::AdaptiveEuler { .. } => 3,
            TimeIntegration::AdaptiveBackward { .. } => 4,
        };
        let old = mode;
        egui::ComboBox::from_id_salt("time-mode")
            .selected_text(["Steady", "Euler", "BDF2", "Adaptive Euler", "Adaptive BDF2"][mode])
            .show_ui(ui, |ui| {
                for (i, n) in ["Steady", "Euler", "BDF2", "Adaptive Euler", "Adaptive BDF2"]
                    .iter()
                    .enumerate()
                {
                    ui.selectable_value(&mut mode, i, *n);
                }
            });
        let (dt, steps) = self.editor.solver.time.parameters().unwrap_or((0.001, 100));
        if old != mode {
            self.editor.solver.time = match mode {
                1 => TimeIntegration::Euler { dt, steps },
                2 => TimeIntegration::Backward { dt, steps },
                3 => TimeIntegration::AdaptiveEuler {
                    dt,
                    steps,
                    controls: AdaptiveTimeStep::default(),
                },
                4 => TimeIntegration::AdaptiveBackward {
                    dt,
                    steps,
                    controls: AdaptiveTimeStep::default(),
                },
                _ => TimeIntegration::Steady,
            };
            if mode == 0 {
                self.editor.solver.physical_end_time = None;
            }
        }
        match &mut self.editor.solver.time {
            TimeIntegration::Steady => {}
            TimeIntegration::Euler { dt, steps }
            | TimeIntegration::Backward { dt, steps }
            | TimeIntegration::AdaptiveEuler { dt, steps, .. }
            | TimeIntegration::AdaptiveBackward { dt, steps, .. } => {
                number(ui, "Time step (s)", dt);
                ui.horizontal(|ui| {
                    ui.label("Additional physical steps");
                    ui.add(egui::DragValue::new(steps).range(1..=1_000_000));
                });
            }
        }
        match &mut self.editor.solver.time {
            TimeIntegration::AdaptiveEuler { controls, .. }
            | TimeIntegration::AdaptiveBackward { controls, .. } => {
                number(ui, "Maximum Courant number", &mut controls.target_courant);
                number(ui, "Minimum Δt (s)", &mut controls.min_dt);
                number(ui, "Maximum Δt (s)", &mut controls.max_dt);
                number(ui, "Maximum step growth", &mut controls.max_growth);
                let mut enabled = controls.max_diffusion.is_some();
                if ui
                    .checkbox(&mut enabled, "Limit diffusion number (accuracy)")
                    .changed()
                {
                    controls.max_diffusion = enabled.then_some(0.5);
                }
                if let Some(d) = &mut controls.max_diffusion {
                    number(ui, "Maximum diffusion number", d);
                }
            }
            _ => {}
        }
        if mode > 0 {
            let mut enabled = self.editor.solver.physical_end_time.is_some();
            if ui
                .checkbox(&mut enabled, "Stop at absolute physical time")
                .changed()
            {
                self.editor.solver.physical_end_time = enabled.then_some(dt * steps as f64);
            }
            if let Some(t) = &mut self.editor.solver.physical_end_time {
                number(ui, "End time (s)", t);
            }
        }
    }
    fn run_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            if icon_button(
                ui,
                "play",
                if self.jobs.is_paused() {
                    "Resume"
                } else {
                    "Run / Continue"
                },
                !self.busy() || self.jobs.is_paused(),
            )
            .clicked()
            {
                self.start_solve();
            }
            if icon_button(
                ui,
                "pause",
                "Pause",
                self.jobs.is_running() && !self.jobs.is_paused(),
            )
            .clicked()
            {
                self.jobs.set_paused(true);
                self.message("Pause requested; all fields are retained.");
            }
            if icon_button(ui, "stop", "Stop", self.jobs.is_running()).clicked() {
                self.jobs.cancel();
                self.message("Stop requested; accepted fields are retained for continuation.");
            }
            if icon_button(ui, "restart", "Initialize", !self.busy()).clicked() {
                self.show_initialization = true;
            }
            if ui
                .add_enabled(!self.busy(), egui::Button::new("Load restart…"))
                .clicked()
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Restart", &["json"])
                    .pick_file()
                {
                    if let Err(e) = self.load_continuation_checkpoint(&path) {
                        self.message(e);
                    }
                }
            }
        });
    }
    fn results_properties(&mut self, ui: &mut egui::Ui) {
        if self.page == 5 {
            self.probe_properties(ui);
            return;
        }
        if self.page == 6 {
            self.history_properties(ui);
            return;
        }
        match self.page {
            0 => {
                heading(ui, "FIELD AND LOCATION");
                egui::ComboBox::from_id_salt("result-style")
                    .selected_text(
                        [
                            "Filled cells",
                            "Contour lines",
                            "Filled contours",
                            "Vectors only",
                        ][self.result_plot.min(3)],
                    )
                    .show_ui(ui, |ui| {
                        for (i, n) in [
                            "Filled cells",
                            "Contour lines",
                            "Filled contours",
                            "Vectors only",
                        ]
                        .iter()
                        .enumerate()
                        {
                            ui.selectable_value(&mut self.result_plot, i, *n);
                        }
                    });
                ui.add(egui::Slider::new(&mut self.result_levels, 3..=32).text("Contour levels"));
                egui::ComboBox::from_id_salt("colour-palette")
                    .selected_text(
                        ["Turbo", "Viridis", "Blue / red", "Grayscale"][self.result_palette.min(3)],
                    )
                    .show_ui(ui, |ui| {
                        for (i, name) in ["Turbo", "Viridis", "Blue / red", "Grayscale"]
                            .iter()
                            .enumerate()
                        {
                            ui.selectable_value(&mut self.result_palette, i, *name);
                        }
                    });

                let mut fixed = self.result_range.is_some();
                if ui.checkbox(&mut fixed, "Fixed color range").changed() {
                    self.result_range = if fixed {
                        self.result
                            .as_ref()
                            .and_then(|d| d.scalar_range(self.result_field).ok())
                            .map(|(a, b)| [a, b])
                    } else {
                        None
                    };
                }
                if let Some([lo, hi]) = &mut self.result_range {
                    number(ui, "Minimum", lo);
                    number(ui, "Maximum", hi);
                }
                if self.plane_index == 0
                    && self
                        .result
                        .as_ref()
                        .is_some_and(|d| d.mesh().dimension() == MeshDimension::TwoD)
                {
                    ui.checkbox(
                        &mut self.seed_streamlines,
                        "Seed 2D streamlines by clicking",
                    );
                }
                if ui.button("Clear streamlines").clicked() {
                    self.streamlines.clear();
                }

                self.section_creation(ui);
                self.plane_selector(ui);
                egui::CollapsingHeader::new("Recording · interval, plane position and history")
                    .default_open(false)
                    .show(ui, |ui| {
                        self.recording_properties(ui);
                        self.plane_recording_properties(ui);
                    });

                let fields = [
                    ResultFieldKind::VelocityMagnitude,
                    ResultFieldKind::Pressure,
                    ResultFieldKind::VelocityX,
                    ResultFieldKind::VelocityY,
                    ResultFieldKind::VelocityZ,
                    ResultFieldKind::Temperature,
                    ResultFieldKind::VorticityMagnitude,
                    ResultFieldKind::VelocityDivergence,
                    ResultFieldKind::QCriterion,
                    ResultFieldKind::TurbulentK,
                    ResultFieldKind::SpecificDissipation,
                    ResultFieldKind::TurbulentViscosity,
                ];
                if self.plane_index == 0 {
                    egui::ComboBox::from_id_salt("result-field")
                        .selected_text(self.result_field.label())
                        .show_ui(ui, |ui| {
                            for f in fields {
                                let available = self.result.as_ref().is_some_and(|d| {
                                    d.scalar_values(f).len() == d.mesh().cell_count()
                                });
                                ui.add_enabled_ui(available, |ui| {
                                    ui.selectable_value(&mut self.result_field, f, f.label());
                                });
                            }
                        });
                }
                if self.plane_index > 0 {
                    if let Some(snapshot) = &mut self.plane_snapshot {
                        snapshot.plane.display.plot = self.result_plot;
                        snapshot.plane.display.levels = self.result_levels;
                        snapshot.plane.display.palette = self.result_palette;
                        snapshot.plane.display.range = self.result_range;

                        let fields = [
                            "Speed (m/s)",
                            "Pressure p/ρ (m²/s²)",
                            "Velocity X (m/s)",
                            "Velocity Y (m/s)",
                            "Velocity Z (m/s)",
                            "Normal velocity (m/s)",
                        ];
                        egui::ComboBox::from_id_salt("section-colour-field")
                            .selected_text(fields[snapshot.plane.display.field.min(5)])
                            .show_ui(ui, |ui| {
                                for (i, name) in fields.iter().enumerate() {
                                    ui.selectable_value(
                                        &mut snapshot.plane.display.field,
                                        i,
                                        *name,
                                    );
                                }
                            });
                        ui.checkbox(&mut snapshot.plane.display.edges, "Cell edges");
                        ui.checkbox(&mut snapshot.plane.display.vectors, "Velocity vectors");
                        ui.checkbox(
                            &mut snapshot.plane.display.normalized_vectors,
                            "Normalize vector lengths",
                        );
                        number(ui, "Vector scale", &mut snapshot.plane.display.vector_scale);
                        ui.add(
                            egui::Slider::new(
                                &mut snapshot.plane.display.vector_density,
                                50..=2000,
                            )
                            .text("Arrow count"),
                        );
                        ui.checkbox(
                            &mut snapshot.plane.display.normal_markers,
                            "Normal flow markers",
                        );
                    }
                } else {
                    ui.checkbox(&mut self.result_edges, "Cell edges");
                    ui.checkbox(&mut self.result_vectors, "Velocity vectors");
                    if self.result_vectors {
                        ui.checkbox(&mut self.vector_normalized, "Normalize vector lengths");
                        number(ui, "Vector scale", &mut self.vector_scale);
                        ui.add(
                            egui::Slider::new(&mut self.vector_count, 50..=2000)
                                .text("Arrow count"),
                        );
                    }
                }
                if let Some(snapshot) = &self.plane_snapshot {
                    if self.plane_index > 0 {
                        if let Some(plane) =
                            self.project.plane_monitors.get_mut(self.plane_index - 1)
                        {
                            if plane.display != snapshot.plane.display {
                                plane.display = snapshot.plane.display.clone();
                                self.dirty = true;
                            }
                        }
                    }
                }
                self.flow_properties(ui);
                if ui.button("Export field CSV…").clicked() {
                    if let (Some(data), Some(path)) = (
                        &self.result,
                        rfd::FileDialog::new()
                            .set_file_name("fields.csv")
                            .save_file(),
                    ) {
                        let result = if self.plane_index > 0 {
                            self.plane_snapshot
                                .as_ref()
                                .ok_or_else(|| "No plane field available".to_string())
                                .and_then(|s| {
                                    std::fs::write(&path, post::plane_csv(s))
                                        .map_err(|e| e.to_string())
                                })
                        } else {
                            data.write_cell_csv(&path).map_err(|e| e.to_string())
                        };
                        if let Err(e) = result {
                            self.message(e);
                        }
                    }
                }
                if ui.button("Export VTK…").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .set_file_name("solution.vtk")
                        .save_file()
                    {
                        let result = if self
                            .result
                            .as_ref()
                            .is_some_and(|d| d.run_id() != "current")
                        {
                            (|| -> Result<(), String> {
                                let folder = self.workspace.as_ref().ok_or("No workspace")?;
                                let run = self
                                    .project
                                    .runs
                                    .iter()
                                    .find(|r| r.id == self.result_run)
                                    .ok_or("Selected run unavailable")?;
                                let source = run
                                    .solution_path
                                    .as_ref()
                                    .ok_or("Selected run has no VTK solution")?;
                                if source.is_absolute()
                                    || source
                                        .components()
                                        .any(|c| matches!(c, std::path::Component::ParentDir))
                                {
                                    return Err("Invalid saved result path".into());
                                }
                                std::fs::copy(folder.join(source), &path)
                                    .map_err(|e| e.to_string())?;
                                Ok(())
                            })()
                        } else {
                            self.session.export_vtk(&path)
                        };
                        if let Err(e) = result {
                            self.message(e);
                        }
                    }
                }
            }
            2 => {
                heading(ui, "CONSERVATION AND DIAGNOSTICS");
                self.export_diagnostics(ui);
                if let Some(report) = &self.archived_report {
                    if let Some(v) = report.get("conservation").and_then(|v| {
                        serde_json::from_value::<workbench::boundary_reports::ConservationReport>(
                            v.clone(),
                        )
                        .ok()
                    }) {
                        ui.label(v.summary());
                    }
                    if let Some(b) = report.get("surface_budgets").and_then(|v| {
                        serde_json::from_value::<Result<workbench::budgets::Report, String>>(
                            v.clone(),
                        )
                        .ok()
                    }) {
                        match b {
                            Ok(b) => {
                                ui.label(b.summary());
                            }
                            Err(e) => widgets::error(ui, &e),
                        }
                    }
                } else if let Some(s) = self.session.solution() {
                    if let Some(m) = self.session.mesh() {
                        ui.label(
                            workbench::boundary_reports::conservation_report(
                                &m.mesh,
                                s,
                                self.session.material().density,
                            )
                            .summary(),
                        );
                    }
                    match &s.budgets {
                        Ok(b) => {
                            ui.label(b.summary());
                        }
                        Err(e) => widgets::error(ui, e),
                    }
                } else {
                    ui.label("Select a run to inspect its fields. Archived diagnostics are in its saved report.");
                }
                if let Some(d) = &self.result {
                    ui.label(format!(
                        "Run {} · {} cells",
                        d.run_id(),
                        d.mesh().cell_count()
                    ));
                    if let Ok((lo, hi)) = d.scalar_range(self.result_field) {
                        ui.label(format!(
                            "{}\nMin {lo:.6e}\nMax {hi:.6e}",
                            self.result_field.label()
                        ));
                    }
                }
            }
            3 => {
                heading(ui, "HISTORY PLOTS");
                self.profile_editor(ui);
                ui.selectable_value(&mut self.show_probe_plot, false, "Convergence history");
                ui.selectable_value(&mut self.show_probe_plot, true, "Recorded probe history");
                if self.show_probe_plot {
                    self.probe_history_controls(ui);
                }
                ui.label("Convergence monitors are shown together. Recorded physical steps remain distinct from steady iterations.");
            }
            _ => {
                heading(ui, "REFERENCE LOADS");
                let mut settings = self.editor.solver.force_analysis.clone();
                for r in &self.wall_reports {
                    let mut selected = settings.patches.contains(&r.patch);
                    if ui.checkbox(&mut selected, &r.patch).changed() {
                        if selected {
                            settings.patches.push(r.patch.clone());
                        } else {
                            settings.patches.retain(|p| p != &r.patch);
                        }
                    }
                }
                ui.small("No selected patches = all walls.");
                number(ui, "Reference speed (m/s)", &mut settings.speed);
                number(ui, "Reference area (2D: length)", &mut settings.measure);
                number(ui, "Reference length (m)", &mut settings.length);
                vector(ui, "Moment center (m)", &mut settings.center);
                vector(ui, "Drag direction", &mut settings.drag_direction);
                vector(ui, "Lift direction", &mut settings.lift_direction);
                number(
                    ui,
                    "Frequency start time (s)",
                    &mut settings.frequency_start_time,
                );
                ui.checkbox(&mut settings.reattachment, "Wall shear / reattachment");
                if settings.reattachment {
                    vector(ui, "Straight wall start (m)", &mut settings.wall_start);
                    vector(ui, "Downstream direction", &mut settings.wall_direction);
                    number(ui, "Wall tolerance (m)", &mut settings.wall_tolerance);
                }
                self.editor.solver.force_analysis = settings;
                if ui.button("Apply and plot").clicked() {
                    if let Err(e) = self.force_chart() {
                        self.message(e);
                    }
                }
                if ui.button("Export coefficients CSV…").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .set_file_name("coefficients.csv")
                        .save_file()
                    {
                        match force_analysis::csv(
                            &self.force_history,
                            &self.editor.solver.force_analysis,
                        )
                        .and_then(|t| std::fs::write(path, t).map_err(|e| e.to_string()))
                        {
                            Ok(()) => self.message("Coefficient history exported."),
                            Err(e) => self.message(e),
                        }
                    }
                }
                ui.label(
                    self.editor
                        .solver
                        .force_analysis
                        .summary(&self.wall_reports, self.session.material().density),
                );
            }
        }
    }
    fn plane_recording_properties(&mut self, ui: &mut egui::Ui) {
        if self.plane_index > 0 {
            let i = self.plane_index - 1;
            if let Some(p) = self.project.plane_monitors.get_mut(i) {
                heading(ui, "PLANE RECORDING");
                ui.checkbox(&mut p.enabled, "Record velocity and pressure");
                ui.horizontal(|ui| {
                    ui.label("Every N iterations / accepted steps");
                    ui.add(egui::DragValue::new(&mut p.every).range(1..=1_000_000));
                });
                vector(ui, "Origin (m)", &mut p.origin);
                vector(ui, "In-plane U axis", &mut p.u);
                vector(ui, "In-plane V axis", &mut p.v);
                ui.weak("Field colours, vectors and flow paths are configured in Display.");
                if ui.button("Apply recording settings").clicked() {
                    match p.validate() {
                        Ok(()) => {
                            self.dirty = true;
                            self.refresh_plane_preview();
                            self.message("Plane settings applied; recording cadence applies to future solves.");
                        }
                        Err(e) => self.message(e),
                    }
                }
            }
            ui.separator();
            if ui.button("Open recording folder…").clicked() {
                if let Some(path) = rfd::FileDialog::new().pick_folder() {
                    self.load_plane_frames(&path);
                }
            }
            if !self.plane_frames.is_empty() {
                ui.label(format!(
                    "{} recorded frames · playback above viewport",
                    self.plane_frames.len()
                ));
            }
        }
    }
    fn convergence_series(&self) -> Vec<workbench::series_plot::Series> {
        [
            "Continuity RMS",
            "Momentum relative",
            "Velocity change",
            "Pressure linear",
        ]
        .iter()
        .enumerate()
        .map(|(i, name)| workbench::series_plot::Series {
            name: name.to_string(),
            points: self.plot_histories()[i].clone(),
        })
        .collect()
    }
    fn convergence(&mut self, ui: &mut egui::Ui) {
        if self.stage == Stage::Results && self.show_probe_plot {
            self.native_probe_plot(ui);
            return;
        }
        self.native_convergence(ui);
    }
    fn force_chart(&mut self) -> Result<(), String> {
        let settings = &self.editor.solver.force_analysis;
        settings.validate()?;
        let series = if settings.reattachment {
            let axis = settings.wall_direction / settings.wall_direction.norm();
            self.wall_reports
                .iter()
                .filter(|r| settings.patches.is_empty() || settings.patches.contains(&r.patch))
                .map(|r| {
                    let mut points: Vec<_> = r
                        .samples
                        .iter()
                        .filter_map(|s| {
                            let d = s.point - settings.wall_start;
                            let x = d.dot(axis);
                            (x >= 0. && (d - axis * x).norm() <= settings.wall_tolerance)
                                .then_some((x, Some(s.shear_stress.dot(axis))))
                        })
                        .collect();
                    points.sort_by(|a, b| a.0.total_cmp(&b.0));
                    workbench::series_plot::Series {
                        name: r.patch.clone(),
                        points,
                    }
                })
                .collect()
        } else {
            force_analysis::series(&self.force_history, settings)?
        };
        self.force_series = series.clone();
        self.session.set_force_analysis(settings.clone())?;
        self.sync_project();
        self.dirty = true;
        Ok(())
    }
    fn plane_selector(&mut self, ui: &mut egui::Ui) {
        let old = self.plane_index;
        egui::ComboBox::from_id_salt("result-location")
            .selected_text(if self.plane_index == 0 {
                "Full domain".into()
            } else {
                self.project
                    .plane_monitors
                    .get(self.plane_index - 1)
                    .map(|p| p.name.clone())
                    .unwrap_or_else(|| "Full domain".into())
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut self.plane_index, 0, "Full domain");
                for (i, p) in self.project.plane_monitors.iter().enumerate() {
                    ui.selectable_value(&mut self.plane_index, i + 1, &p.name);
                }
            });
        if old != self.plane_index {
            self.streamlines.clear();
            self.probes.clear();
            self.field_playing = false;
            self.plane_playing = false;
            self.plane_snapshot = None;
            self.plane_frames.clear();
            if let Some(p) = self
                .project
                .plane_monitors
                .get(self.plane_index.saturating_sub(1))
                .filter(|_| self.plane_index > 0)
                .cloned()
            {
                if let Some(data) = &self.result {
                    match plane_monitor::intersect(data.mesh(), &p).and_then(|cells| {
                        plane_monitor::sample(
                            &p,
                            &cells,
                            self.result_iteration,
                            data.velocity(),
                            data.pressure(),
                        )
                    }) {
                        Ok(s) => {
                            self.camera.fit_mesh(data.mesh());
                            let normal = p.normal();
                            self.camera.camera = self
                                .camera
                                .camera
                                .looking_from(cad_geom::Vec3::new(normal.x, normal.y, normal.z));
                            self.plane_snapshot = Some(s);
                        }
                        Err(e) => self.message(e),
                    }
                }
                if let Some(folder) = self.recording.clone() {
                    self.load_plane_frames(&folder);
                }
            }
        }
    }
    pub(super) fn load_plane_frames(&mut self, folder: &Path) {
        let Some(plane) = self
            .project
            .plane_monitors
            .get(self.plane_index.saturating_sub(1))
            .filter(|_| self.plane_index > 0)
        else {
            return;
        };
        let id = plane.id.clone();
        let folder = folder.to_path_buf();
        let (tx, rx) = mpsc::channel();
        self.frame_scan = Some((id.clone(), rx));
        std::thread::spawn(move || {
            let mut frames = Vec::new();
            fn scan(folder: &Path, id: &str, out: &mut Vec<PathBuf>, depth: usize) {
                if depth > 6 {
                    return;
                }
                if let Ok(entries) = std::fs::read_dir(folder) {
                    for entry in entries.flatten() {
                        let Ok(kind) = entry.file_type() else {
                            continue;
                        };
                        let path = entry.path();
                        if kind.is_dir() {
                            scan(&path, id, out, depth + 1);
                        } else if kind.is_file()
                            && path.extension().is_some_and(|e| e == "json")
                            && path
                                .file_name()
                                .is_some_and(|n| n.to_string_lossy().starts_with("iteration-"))
                        {
                            if std::fs::read(&path)
                                .ok()
                                .and_then(|b| {
                                    serde_json::from_slice::<plane_monitor::PlaneSnapshot>(&b).ok()
                                })
                                .is_some_and(|s| s.plane.id == id)
                            {
                                out.push(path);
                            }
                        }
                    }
                }
            }
            scan(&folder, &id, &mut frames, 0);
            frames.sort();
            let _ = tx.send(frames);
        });
        self.message("Loading recorded plane history in the background…");
    }
    pub(super) fn open_run(&mut self, id: &str) -> Result<(), String> {
        let path = self
            .workspace
            .as_ref()
            .ok_or("Save or open a workspace first")?;
        let data =
            workbench::load_workspace_result(path, &self.project, id).map_err(|e| e.to_string())?;
        self.camera.fit_mesh(data.mesh());
        self.result_cache = ResultRenderCache::build(&data).ok();
        self.result = Some(data);
        self.result_run = id.into();
        self.probes.clear();
        self.probe_series.clear();
        self.probe_history_job = None;
        self.selected_probe = None;
        self.streamlines.clear();
        self.recording = None;
        self.histories = Default::default();
        self.stage = Stage::Results;
        self.page = 0;
        self.plane_index = 0;
        self.plane_snapshot = None;
        self.plane_frames.clear();
        self.frame_scan = None;
        self.plane_playing = false;
        self.field_playing = false;
        self.force_history.clear();
        self.force_series.clear();
        self.wall_reports.clear();
        if let Some(run) = self.project.runs.iter().find(|r| r.id == id) {
            self.result_iteration = if run.time_steps > 0 {
                run.time_steps
            } else {
                run.iterations.unwrap_or(0)
            };
            let report = flursys::headless::report(path, run)?;
            self.archived_report = Some(report.clone());
            self.histories = continuation::read_histories(&report, self.result_iteration);
            self.wall_reports = report
                .get("wall_reports")
                .and_then(|v| serde_json::from_value(v.clone()).ok())
                .unwrap_or_default();
            self.force_history = flursys::headless::force_history(path, run).unwrap_or_default();
            if let Some(folder) = report
                .get("plane_recording_folder")
                .and_then(|v| v.as_str())
            {
                let p = Path::new(folder);
                if !p.is_absolute()
                    && !p
                        .components()
                        .any(|c| matches!(c, std::path::Component::ParentDir))
                {
                    self.recording = Some(path.join(p));
                }
            }
        }
        self.refresh_field_frames();
        Ok(())
    }
    fn windows(&mut self, ctx: &egui::Context) {
        if self.show_initialization {
            let mut open = true;
            egui::Window::new("Initialize solution").open(&mut open).default_width(460.).frame(widgets::dialog_frame(ctx)).scroll(true).max_height(ctx.content_rect().height() * 0.85).show(ctx,|ui|{self.mapping_editor(ui);ui.separator();ui.label("Uniform initialization resets accepted fields, physical time and iteration numbering. Existing run archives are retained.");vector(ui,"Uniform velocity (m/s)",&mut self.initial_velocity);number(ui,"Kinematic gauge pressure p/ρ",&mut self.initial_pressure);if self.editor.solver.thermal.enabled{number(ui,"Initial temperature (K)",&mut self.editor.solver.thermal.initial_temperature);}if ui.add_enabled(!self.busy(),egui::Button::new("Initialize fields")).clicked(){let valid=[self.initial_velocity.x,self.initial_velocity.y,self.initial_velocity.z,self.initial_pressure].iter().all(|v|v.is_finite());if !valid{self.message("Initialization values must be finite.");}else if let Some(mesh)=self.session.mesh().map(|m|m.mesh.id()){self.reset_solve_session();self.jobs.set_initial_fields(mesh,flursys::IncompressibleInitialConditions::Uniform{velocity:self.initial_velocity,pressure:self.initial_pressure});self.session.initialize_solution();self.histories=Default::default();self.iteration=0;self.force_history.clear();self.streamlines.clear();self.probes.clear();self.result=None;self.result_cache=None;self.result_run.clear();self.plane_snapshot=None;self.plane_frames.clear();self.plane_playing=false;self.archived_report=None;self.initialized=true;if let Err(e)=self.persist_continuation(){self.message(e);}self.show_initialization=false;self.message("Fields initialized; time and iteration numbering reset.");}else{self.message("Generate a mesh before initializing fields.");}}});
            self.show_initialization &= open;
        }

        if self.show_preferences {
            let mut open = true;
            egui::Window::new("Application settings")
                .open(&mut open)
                .resizable(true)
                .default_width(600.)
                .default_pos(ctx.content_rect().center() - egui::vec2(300., 220.))
                .collapsible(false)
                .max_height(ctx.content_rect().height() * 0.85)
                .scroll(true)
                .frame(widgets::dialog_frame(ctx))
                .show(ctx, |ui| {
                    ui.heading("Workspace preferences");
                    ui.weak("Appearance, viewport and external tools. Solver physics remain in Solution.");
                    ui.add_space(8.);
                    ui.horizontal_wrapped(|ui| {
                        for (index, name) in ["Appearance", "Viewport", "Layout", "Tools"].iter().enumerate() {
                            ui.selectable_value(&mut self.preferences_page, index, *name);
                        }
                    });
                    ui.separator();
                    if self.preferences_page == 0 {
                    heading(ui, "APPEARANCE");
                    ui.checkbox(&mut self.preferences.light, "Light appearance");
                    ui.add(
                        egui::Slider::new(&mut self.preferences.scale, 0.8..=1.7).text("UI scale"),
                    );
                    }
                    if self.preferences_page == 1 {
                    heading(ui, "VIEWPORT");
                    ui.checkbox(&mut self.preferences.geometry_edges, "Show CAD feature edges");
                    ui.checkbox(&mut self.preferences.geometry_shading, "Shaded CAD surfaces");
                    ui.checkbox(&mut self.preferences.ambient_occlusion, "Ambient occlusion");
                    ui.add(egui::Slider::new(&mut self.preferences.edge_width, 0.5..=3.0).text("Edge width"));
                    ui.add(egui::Slider::new(&mut self.preferences.navigation_speed, 0.3..=2.0).text("Navigation speed"));
                    }
                    if self.preferences_page == 2 {
                    heading(ui, "LAYOUT");
                    if ui.checkbox(&mut self.browser_open, "Project browser").changed() { self.browser_requested = self.browser_open; }
                    ui.checkbox(&mut self.properties_open, "Properties panel");
                    ui.checkbox(&mut self.show_log, "Messages panel");
                    }
                    if self.preferences_page == 3 {
                    heading(ui, "EXTERNAL TOOLS");
                    ui.label("Gmsh executable · blank uses PATH");
                    ui.text_edit_singleline(&mut self.preferences.gmsh_path);
                    if ui.button("Browse…").clicked() {
                        if let Some(p) = rfd::FileDialog::new().pick_file() {
                            self.preferences.gmsh_path = p.to_string_lossy().to_string();
                        }
                    }
                    ui.add_space(8.);

                    }
                    ui.separator();
                    if ui.button("Restore defaults").clicked() { self.preferences = Preferences::default(); self.browser_open = true; self.properties_open = true; self.show_log = false; }
                    if ui.button("Apply and save").clicked() {
                        self.preferences.browser = self.browser_open;
                        self.preferences.properties = self.properties_open;
                        self.preferences.messages = self.show_log;
                        self.apply_preferences(ctx);
                        if let Some(path) = Self::preferences_path() {
                            let r = (|| -> Result<(), String> {
                                std::fs::create_dir_all(
                                    path.parent().ok_or("No preferences directory")?,
                                )
                                .map_err(|e| e.to_string())?;
                                std::fs::write(
                                    path,
                                    serde_json::to_vec_pretty(&self.preferences)
                                        .map_err(|e| e.to_string())?,
                                )
                                .map_err(|e| e.to_string())
                            })();
                            if let Err(e) = r {
                                self.message(e);
                            } else {
                                self.message("Application settings saved.");
                            }
                        }
                    }
                });
            self.show_preferences &= open;
        }
        if self.show_examples {
            if self.examples.is_empty() {
                match workbench::catalogue::builtin_templates() {
                    Ok(t) => self.examples = t,
                    Err(e) => self.message(e),
                }
            }
            let mut open = true;
            let mut choice = None;
            egui::Window::new("Example workspaces")
                .open(&mut open)
                .default_width(650.)
                .frame(widgets::dialog_frame(ctx))
                .max_height(ctx.content_rect().height() * 0.85)
                .default_height(600.)
                .show(ctx, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        for (i, t) in self.examples.iter().enumerate() {
                            ui.group(|ui| {
                                ui.strong(&t.name);
                                ui.label(&t.description);
                                if ui.button("Create editable workspace").clicked() {
                                    choice = Some(i);
                                }
                            });
                        }
                    });
                });
            self.show_examples = open;
            if let Some(i) = choice {
                self.show_examples = false;
                self.request(Pending::Example(i));
            }
        }
        if self.show_advanced {
            let mut open = true;
            egui::Window::new("Project configuration · expert editor").open(&mut open).default_size([850.,650.]).frame(widgets::dialog_frame(ctx)).show(ctx,|ui|{ui.label("Full canonical configuration. Apply validates all fields; geometry and saved run history remain owned by the workspace.");egui::ScrollArea::vertical().max_height((ui.available_height() - 70.).max(120.)).show(ui,|ui|{ui.add(egui::TextEdit::multiline(&mut self.json_editor).code_editor().desired_width(f32::INFINITY).desired_rows(25));});if ui.add_enabled(!self.busy(),egui::Button::new("Validate and apply")).clicked(){match serde_json::from_str::<WorkbenchProject>(&self.json_editor).map_err(|e|e.to_string()).and_then(|p|{self.editor=p;self.apply_editor()}){Ok(())=>self.message("Configuration applied."),Err(e)=>self.message(e)}}});
            self.show_advanced = open;
        }
        if self.pending_replace.is_some() {
            egui::Modal::new(egui::Id::new("unsaved-project"))
                .frame(widgets::dialog_frame(ctx))
                .show(ctx, |ui| {
                    ui.heading("Unsaved workspace");
                    ui.label("Save your changes before replacing or closing this workspace?");
                    ui.horizontal(|ui| {
                        if ui.button("Save").clicked() {
                            match self.save() {
                                Ok(true) => {
                                    if let Some(p) = self.pending_replace.take() {
                                        self.replace(p);
                                    }
                                }
                                Ok(false) => {}
                                Err(e) => self.message(e),
                            }
                        }
                        if ui.button("Discard").clicked() {
                            if let Some(p) = self.pending_replace.take() {
                                self.replace(p);
                            }
                        }
                        if ui.button("Cancel").clicked() {
                            self.pending_replace = None;
                        }
                    });
                });
        }
    }
}
