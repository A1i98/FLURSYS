use super::*;

impl App {
    pub(super) fn remember_workspace(&mut self) {
        self.analysis_graph = None;
        let Some(path) = self.workspace.clone() else {
            return;
        };
        self.analysis_graph = workbench::analysis_graph::load(&path).ok();
        self.preferences.recent_workspaces.retain(|p| p != &path);
        self.preferences.recent_workspaces.insert(0, path);
        self.preferences.recent_workspaces.truncate(12);
        if let Some(path) = Self::preferences_path() {
            let saved = (|| -> Result<(), String> {
                std::fs::create_dir_all(path.parent().ok_or("Preferences directory is missing")?)
                    .map_err(|e| e.to_string())?;
                std::fs::write(
                    path,
                    serde_json::to_vec_pretty(&self.preferences).map_err(|e| e.to_string())?,
                )
                .map_err(|e| e.to_string())
            })();
            if let Err(e) = saved {
                self.append_log(format!("Recent workspaces: {e}"));
            }
        }
    }
    pub(super) fn home(&mut self, ui: &mut egui::Ui) {
        let width = ui
            .available_rect_before_wrap()
            .intersect(ui.clip_rect())
            .width();
        ui.set_max_width(width);
        let background = if self.preferences.light {
            Color32::from_rgb(240, 244, 249)
        } else {
            Color32::from_rgb(28, 34, 44)
        };
        egui::Frame::new()
            .fill(background)
            .inner_margin(24.)
            .show(ui, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("home-scroll")
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        ui.heading(RichText::new("Your simulation workbench").size(28.));
                        ui.label(
                            RichText::new(
                                "Create geometry, prepare an analysis and explore its results.",
                            )
                            .size(15.)
                            .weak(),
                        );
                        ui.add_space(20.);
                        let columns =
                            ((ui.available_width() + 8.) / 270.).floor().clamp(1., 3.) as usize;
                        let card_width =
                            (ui.available_width() - 8. * (columns - 1) as f32) / columns as f32;
                        egui::Grid::new("home-actions")
                            .num_columns(columns)
                            .spacing(egui::vec2(8., 8.))
                            .show(ui, |ui| {
                                for (i, (icon, title, description, action)) in [
                                    ("new", "New project", "Start with a blank CAD model", 0),
                                    ("open", "Open project", "Continue a saved workspace", 1),
                                    (
                                        "mesh",
                                        "Example library",
                                        "Start with a configured CFD case",
                                        2,
                                    ),
                                ]
                                .into_iter()
                                .enumerate()
                                {
                                    egui::Frame::group(ui.style())
                                        .inner_margin(18.)
                                        .corner_radius(10.)
                                        .show(ui, |ui| {
                                            ui.vertical(|ui| {
                                                ui.set_width((card_width - 38.).max(100.));
                                                ui.set_min_height(110.);
                                                ui.label(RichText::new(title).strong().size(17.));
                                                ui.label(description);
                                                ui.add_space(14.);
                                                if widgets::icon_button(
                                                    ui,
                                                    icon,
                                                    title,
                                                    !self.busy(),
                                                )
                                                .clicked()
                                                {
                                                    match action {
                                                        0 => self.request(Pending::New),
                                                        1 => self.choose_workspace_folder(
                                                            WorkspaceDialogAction::Open,
                                                        ),
                                                        _ => self.show_examples = true,
                                                    }
                                                }
                                            });
                                        });
                                    if (i + 1).is_multiple_of(columns) {
                                        ui.end_row();
                                    }
                                }
                            });
                        ui.add_space(22.);
                        ui.heading("Active project");
                        egui::Frame::group(ui.style())
                            .inner_margin(18.)
                            .corner_radius(10.)
                            .show(ui, |ui| {
                                ui.horizontal_wrapped(|ui| {
                                    ui.label(RichText::new(&self.project.name).strong().size(19.));
                                    ui.weak(if self.dirty || self.cad.session.is_dirty() {
                                        "Modified"
                                    } else if self.workspace.is_some() {
                                        "Saved"
                                    } else {
                                        "New workspace"
                                    });
                                });
                                if let Some(path) = &self.workspace {
                                    ui.add(
                                        egui::Label::new(
                                            RichText::new(path.display().to_string()).small(),
                                        )
                                        .truncate(),
                                    );
                                }
                                ui.add_space(14.);
                                let columns =
                                    (ui.available_width() / 150.).floor().clamp(1., 5.) as usize;
                                let step_width = (ui.available_width() - 8. * (columns - 1) as f32)
                                    / columns as f32;
                                egui::Grid::new("home-workflow")
                                    .num_columns(columns)
                                    .spacing(egui::vec2(8., 8.))
                                    .show(ui, |ui| {
                                        for (i, stage) in Stage::ALL.into_iter().enumerate() {
                                            let detail = match stage {
                                                Stage::Geometry => "Shared CAD model".into(),
                                                Stage::Mesh => self
                                                    .session
                                                    .mesh()
                                                    .map(|m| {
                                                        format!("{} cells", m.mesh.cell_count())
                                                    })
                                                    .unwrap_or("Not generated".into()),
                                                Stage::Setup => format!(
                                                    "{} boundary conditions",
                                                    self.project.boundaries.len()
                                                ),
                                                Stage::Solution => {
                                                    format!("Iteration / step {}", self.iteration)
                                                }
                                                Stage::Results => format!(
                                                    "{} saved runs",
                                                    self.project.runs.len()
                                                ),
                                            };
                                            egui::Frame::group(ui.style()).inner_margin(12.).show(
                                                ui,
                                                |ui| {
                                                    ui.vertical(|ui| {
                                                        ui.set_width((step_width - 26.).max(80.));
                                                        ui.small(format!("{:02}", i + 1));
                                                        ui.strong(stage.label());
                                                        ui.small(detail);
                                                        if ui.button("Open").clicked() {
                                                            self.stage(stage);
                                                        }
                                                    });
                                                },
                                            );
                                            if (i + 1).is_multiple_of(columns) {
                                                ui.end_row();
                                            }
                                        }
                                    });
                                ui.add_space(12.);
                                if widgets::icon_button(ui, "save", "Save project", !self.busy())
                                    .clicked()
                                {
                                    if let Err(e) = self.save() {
                                        self.message(e);
                                    }
                                }
                            });
                        ui.add_space(22.);
                        self.analysis_workbench(ui);
                        ui.heading("Recent projects");
                        let paths = self.preferences.recent_workspaces.clone();
                        if paths.is_empty() {
                            ui.weak("Projects you open or save will appear here.");
                        }
                        for path in paths {
                            egui::Frame::group(ui.style())
                                .inner_margin(12.)
                                .corner_radius(7.)
                                .show(ui, |ui| {
                                    ui.horizontal(|ui| {
                                        ui.vertical(|ui| {
                                            ui.strong(
                                                path.file_name()
                                                    .map(|n| n.to_string_lossy())
                                                    .unwrap_or_default(),
                                            );
                                            ui.add(
                                                egui::Label::new(
                                                    RichText::new(path.display().to_string())
                                                        .small(),
                                                )
                                                .truncate(),
                                            );
                                        });
                                        ui.with_layout(
                                            egui::Layout::right_to_left(egui::Align::Center),
                                            |ui| {
                                                if ui
                                                    .add_enabled(
                                                        !self.busy(),
                                                        egui::Button::new("Open"),
                                                    )
                                                    .clicked()
                                                {
                                                    self.request(Pending::Open(path.clone()));
                                                }
                                            },
                                        );
                                    });
                                });
                        }
                        ui.add_space(20.);
                    });
            });
    }
}

impl App {
    fn analysis_workbench(&mut self, ui: &mut egui::Ui) {
        ui.heading("Analyses connected to this CAD");
        let graph = self.analysis_graph.clone();
        if let Some((root, graph)) = graph {
            let source_key = graph
                .analyses
                .iter()
                .find(|a| a.project_id == graph.source_project)
                .map(|a| a.geometry_key.clone());
            for analysis in graph.analyses {
                egui::Frame::group(ui.style())
                    .inner_margin(12.)
                    .corner_radius(7.)
                    .show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            ui.strong(&analysis.name);
                            ui.weak(if source_key.as_ref() == Some(&analysis.geometry_key) {
                                "CAD revision linked"
                            } else {
                                "Earlier CAD revision"
                            });
                            if self.workspace.as_ref().is_some_and(|p| {
                                p.canonicalize().ok()
                                    == root.join(&analysis.workspace).canonicalize().ok()
                            }) {
                                ui.label(RichText::new("Active").color(widgets::ACCENT));
                            } else if ui
                                .add_enabled(!self.busy(), egui::Button::new("Open analysis"))
                                .clicked()
                            {
                                self.request(Pending::Open(root.join(&analysis.workspace)));
                            }
                        });
                    });
            }
            ui.horizontal_wrapped(|ui|{
                ui.label("New analysis");ui.text_edit_singleline(&mut self.analysis_name);
                let ready=!self.busy() && !self.dirty && !self.cad.session.is_dirty();
                if ui.add_enabled(ready,egui::Button::new("Create from current CAD")).on_hover_text("Copies the current CAD, mesh and setup into an independent analysis without previous run fields").clicked(){
                    if let Some(workspace)=self.workspace.clone(){let project=self.project.clone();let name=self.analysis_name.clone();let (tx,rx)=mpsc::channel();std::thread::spawn(move||{let _=tx.send(workbench::analysis_graph::branch(&workspace,&project,&name));});self.analysis_job=Some(rx);}
                }
            });
            if self.dirty || self.cad.session.is_dirty() {
                ui.weak("Save the project before creating a linked analysis.");
            }
        } else {
            ui.weak("Save a project to create independent analyses from its CAD and mesh.");
        }
        ui.add_space(24.);
    }
}
