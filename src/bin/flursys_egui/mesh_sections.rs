use super::*;

pub(super) fn outlines(
    bounds: (Vec3, Vec3),
    axis: usize,
    fraction: f64,
    slab: Option<f64>,
) -> Vec<[Vec3; 4]> {
    let (lo, hi) = bounds;
    let at = |p: Vec3| match axis {
        1 => p.y,
        2 => p.z,
        _ => p.x,
    };
    let center = at(lo) + (at(hi) - at(lo)) * fraction;
    let positions = if let Some(width) = slab {
        let half = (at(hi) - at(lo)) * width * 0.5;
        vec![center - half, center + half]
    } else {
        vec![center]
    };
    positions
        .into_iter()
        .map(|depth| match axis {
            1 => [
                Vec3::new(lo.x, depth, lo.z),
                Vec3::new(hi.x, depth, lo.z),
                Vec3::new(hi.x, depth, hi.z),
                Vec3::new(lo.x, depth, hi.z),
            ],
            2 => [
                Vec3::new(lo.x, lo.y, depth),
                Vec3::new(hi.x, lo.y, depth),
                Vec3::new(hi.x, hi.y, depth),
                Vec3::new(lo.x, hi.y, depth),
            ],
            _ => [
                Vec3::new(depth, lo.y, lo.z),
                Vec3::new(depth, hi.y, lo.z),
                Vec3::new(depth, hi.y, hi.z),
                Vec3::new(depth, lo.y, hi.z),
            ],
        })
        .collect()
}

impl App {
    pub(super) fn mesh_section_toolbar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            let image = egui::Image::new(egui::include_image!(
                "../../../vendor/cad-workbench/assets/icons/ui/design/svg/model/section.svg"
            ))
            .fit_to_exact_size(egui::vec2(18., 18.));
            if ui
                .add(egui::Button::image_and_text(image, "Cutaway").selected(self.mesh_cut))
                .on_hover_text("Reveal interior cell faces without changing the simulation mesh")
                .clicked()
            {
                self.mesh_cut = !self.mesh_cut;
                if self.mesh_cut {
                    self.show_geometry = false;
                }
            }
            if self.mesh_cut {
                ui.separator();
                ui.label("Axis");
                for (i, name) in ["X", "Y", "Z"].iter().enumerate() {
                    ui.selectable_value(&mut self.mesh_cut_axis, i, *name);
                }
                ui.toggle_value(&mut self.mesh_cut_reverse, "Flip side")
                    .on_hover_text("Keep the opposite half, or the outside of the slab");
                ui.toggle_value(&mut self.mesh_slab, "Slab")
                    .on_hover_text("Keep a band of cells around the section position");
                ui.toggle_value(&mut self.mesh_section_only, "Section only")
                    .on_hover_text(
                        "Hide the exterior skin and show only the exposed interior cell faces",
                    );
                ui.toggle_value(&mut self.mesh_ghost_shell, "Ghost shell")
                    .on_hover_text("Show removed exterior faces faintly for orientation");
                ui.toggle_value(&mut self.mesh_cut_plane, "Plane outline");
                if ui.button("Show all").clicked() {
                    self.mesh_cut = false;
                    self.mesh_cut_reverse = false;
                    self.mesh_slab = false;
                    self.mesh_section_only = false;
                    self.mesh_ghost_shell = false;
                }
            }
        });
        if !self.mesh_cut {
            return;
        }
        ui.horizontal_wrapped(|ui| {
            ui.label("Position");
            if ui
                .small_button("−")
                .on_hover_text("Move section back by 5%")
                .clicked()
            {
                self.mesh_cut_fraction = (self.mesh_cut_fraction - 0.05).max(0.);
                self.mesh_cut_preview = self.mesh_cut_fraction;
            }
            let position = ui
                .add(
                    egui::Slider::new(&mut self.mesh_cut_preview, 0.0..=1.0)
                        .custom_formatter(|v, _| format!("{:.0}%", v * 100.)),
                )
                .on_hover_text(
                    "Whole-cell section; mesh geometry updates when you release the slider",
                );
            if position.drag_stopped() || position.changed() && !position.dragged() {
                self.mesh_cut_fraction = self.mesh_cut_preview;
            }
            if ui
                .small_button("+")
                .on_hover_text("Move section forward by 5%")
                .clicked()
            {
                self.mesh_cut_fraction = (self.mesh_cut_fraction + 0.05).min(1.);
                self.mesh_cut_preview = self.mesh_cut_fraction;
            }
            if ui.button("Centre").clicked() {
                self.mesh_cut_fraction = 0.5;
                self.mesh_cut_preview = 0.5;
            }
            if self.mesh_slab {
                ui.label("Thickness");
                let thickness = ui.add(
                    egui::Slider::new(&mut self.mesh_slab_preview, 0.01..=1.0)
                        .custom_formatter(|v, _| format!("{:.0}%", v * 100.)),
                );
                if thickness.drag_stopped() || thickness.changed() && !thickness.dragged() {
                    self.mesh_slab_width = self.mesh_slab_preview;
                }
            }
            if let Some(cache) = &self.mesh_cache {
                let at = |p: Vec3| match self.mesh_cut_axis {
                    1 => p.y,
                    2 => p.z,
                    _ => p.x,
                };
                let (lo, hi) = cache.bounds();
                let position = at(lo) + (at(hi) - at(lo)) * self.mesh_cut_preview;
                let prefix = if self.mesh_cut_fraction != self.mesh_cut_preview {
                    "Preview "
                } else {
                    ""
                };
                ui.weak(format!(
                    "{prefix}{} = {position:.5e} m",
                    ["X", "Y", "Z"][self.mesh_cut_axis.min(2)]
                ));
            }
        });
    }
    pub(super) fn mesh_section_preview(&self, painter: &egui::Painter, rect: egui::Rect) {
        if !self.mesh_cut
            || self.show_geometry
            || !matches!(self.stage, Stage::Mesh | Stage::Setup)
            || self.mesh_cut_fraction == self.mesh_cut_preview
                && (!self.mesh_slab || self.mesh_slab_width == self.mesh_slab_preview)
        {
            return;
        }
        let Some(cache) = &self.mesh_cache else {
            return;
        };
        for corners in outlines(
            cache.bounds(),
            self.mesh_cut_axis,
            self.mesh_cut_preview,
            self.mesh_slab.then_some(self.mesh_slab_preview),
        ) {
            for i in 0..4 {
                if let (Some((a, _)), Some((b, _))) = (
                    self.camera.screen(corners[i], rect),
                    self.camera.screen(corners[(i + 1) % 4], rect),
                ) {
                    painter.line_segment(
                        [a, b],
                        egui::Stroke::new(1.7, Color32::from_rgb(255, 195, 95)),
                    );
                }
            }
        }
    }
}
