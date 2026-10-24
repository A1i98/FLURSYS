use egui::{Color32, RichText, Ui};
use serde_json::Value;
pub const ACCENT: Color32 = Color32::from_rgb(65, 165, 245);
pub fn heading(ui: &mut Ui, title: &str) {
    ui.add_space(8.);
    ui.label(RichText::new(title).strong().color(ACCENT));
    ui.separator();
}
pub fn icon_button(ui: &mut Ui, icon: &str, label: &str, enabled: bool) -> egui::Response {
    ui.add_enabled_ui(enabled, |ui| {
        let response = ui.add_sized(
            [
                if label.is_empty() {
                    30.
                } else {
                    ui.fonts_mut(|f| {
                        f.layout_no_wrap(
                            label.into(),
                            egui::FontId::proportional(13.),
                            Color32::WHITE,
                        )
                        .size()
                        .x
                    }) + 42.
                },
                30.,
            ],
            egui::Button::new(if label.is_empty() {
                String::new()
            } else {
                format!("    {label}")
            }),
        );
        let r = egui::Rect::from_center_size(
            egui::pos2(response.rect.left() + 15., response.rect.center().y),
            egui::vec2(18., 18.),
        );
        let image = match icon {
            "play" => Some(egui::include_image!("../../../ui/assets/icons/play.svg")),
            "pause" => Some(egui::include_image!("../../../ui/assets/icons/pause.svg")),
            "stop" => Some(egui::include_image!("../../../ui/assets/icons/stop.svg")),
            "restart" => Some(egui::include_image!("../../../ui/assets/icons/undo.svg")),
            _ => None,
        };
        if let Some(image) = image {
            egui::Image::new(image)
                .tint(Color32::from_white_alpha(if enabled { 255 } else { 80 }))
                .paint_at(ui, r);
        } else if icon == "clear" {
            let ink = ui.visuals().text_color();
            ui.painter().line_segment(
                [
                    r.left_top() + egui::vec2(4., 4.),
                    r.right_bottom() - egui::vec2(4., 4.),
                ],
                egui::Stroke::new(1.5, ink),
            );
            ui.painter().line_segment(
                [
                    r.right_top() + egui::vec2(-4., 4.),
                    r.left_bottom() + egui::vec2(4., -4.),
                ],
                egui::Stroke::new(1.5, ink),
            );
        } else {
            cad_workbench::icons::paint_icon(
                ui.painter(),
                r,
                icon,
                if enabled { 1.0 } else { 0.35 },
            );
        }
        response
    })
    .inner
}
pub fn number(ui: &mut Ui, label: &str, value: &mut f64) {
    ui.vertical(|ui| {
        ui.label(egui::RichText::new(label).color(ui.visuals().weak_text_color()));
        let speed = if *value == 0.0 {
            1e-5
        } else {
            value.abs() * 0.01
        };
        ui.add_sized(
            [ui.available_width(), 28.],
            egui::DragValue::new(value).speed(speed),
        );
    });
}
pub fn vector(ui: &mut Ui, label: &str, value: &mut flursys::Vec3) {
    ui.label(label);
    ui.horizontal(|ui| {
        for (axis, v) in [
            ("X", &mut value.x),
            ("Y", &mut value.y),
            ("Z", &mut value.z),
        ] {
            ui.label(axis);
            let speed = v.abs().max(0.001) * 0.01;
            ui.add(egui::DragValue::new(v).speed(speed));
        }
    });
}
pub fn label(key: &str) -> String {
    let s = key.replace('_', " ");
    let mut c = s.chars();
    match c.next() {
        Some(a) => a.to_uppercase().collect::<String>() + c.as_str(),
        None => s,
    }
}
/// Native typed property rows. The draft is applied only after full model validation.
pub fn fields(ui: &mut Ui, id: &str, value: &mut Value) {
    match value {
        Value::Object(map) => {
            for (key, v) in map {
                let name = label(key);
                let key_id = format!("{id}.{key}");
                match v {
                    Value::Bool(b) => {
                        ui.checkbox(b, name);
                    }
                    Value::Number(n) => {
                        let integer = n.is_u64();
                        let mut x = n.as_f64().unwrap_or(0.0);
                        ui.horizontal(|ui| {
                            ui.label(name);
                            let speed = if integer {
                                1.0
                            } else {
                                x.abs().max(0.001) * 0.01
                            };
                            let mut drag = egui::DragValue::new(&mut x).speed(speed);
                            if integer {
                                drag = drag.max_decimals(0).range(0.0..=1e12);
                            }
                            ui.add(drag);
                        });
                        if x.is_finite() {
                            *v = if integer {
                                Value::from(x.round().max(0.) as u64)
                            } else {
                                Value::from(x)
                            };
                        }
                    }
                    Value::String(text) => {
                        ui.label(name);
                        let choices: &[&str] = match key.as_str() {
                            "preconditioner" => &["none", "jacobi", "ic0", "gamg", "auto"],
                            "backend" => &["serial", "parallel", "gpu"],
                            "gpu_preconditioner" => &["auto", "jacobi"],
                            "quantity" => &["velocity", "pressure", "temperature"],
                            "wall_treatment" if id.contains("thermal") => {
                                &["molecular", "equilibrium-prt"]
                            }
                            _ => &[],
                        };
                        if choices.is_empty() {
                            ui.add(egui::TextEdit::singleline(text).desired_width(f32::INFINITY));
                        } else {
                            egui::ComboBox::from_id_salt(&key_id)
                                .selected_text(text.as_str())
                                .show_ui(ui, |ui| {
                                    for name in choices {
                                        ui.selectable_value(text, name.to_string(), *name);
                                    }
                                });
                        }
                    }
                    Value::Null => {
                        ui.horizontal(|ui| {
                            ui.label(name);
                            ui.weak("Not configured");
                        });
                    }
                    _ => {
                        egui::CollapsingHeader::new(name)
                            .id_salt(key_id.clone())
                            .default_open(false)
                            .show(ui, |ui| fields(ui, &key_id, v));
                    }
                }
            }
        }
        Value::Array(values) => {
            let mut remove = None;
            for (i, v) in values.iter_mut().enumerate() {
                ui.push_id((id, i), |ui| {
                    ui.horizontal(|ui| {
                        ui.label(format!("Item {}", i + 1));
                        if ui.small_button("Remove").clicked() {
                            remove = Some(i);
                        }
                    });
                    fields(ui, &format!("{id}[{i}]"), v);
                });
            }
            if let Some(i) = remove {
                values.remove(i);
            }
            if let Some(first) = values.first().cloned() {
                if ui.button("Duplicate first item").clicked() {
                    values.push(first);
                }
            }
            let draft_id = egui::Id::new(("new-property-item", id));
            let mut draft = ui
                .ctx()
                .data_mut(|d| d.get_temp::<String>(draft_id))
                .unwrap_or_else(|| "{}".into());
            egui::CollapsingHeader::new("Add entry (JSON)")
                .id_salt((id, "add-entry"))
                .show(ui, |ui| {
                    ui.add(
                        egui::TextEdit::multiline(&mut draft)
                            .code_editor()
                            .desired_rows(3),
                    );
                    if ui.button("Add entry").clicked() {
                        match serde_json::from_str(&draft) {
                            Ok(v) => values.push(v),
                            Err(e) => error(ui, &e.to_string()),
                        }
                    }
                });
            ui.ctx().data_mut(|d| d.insert_temp(draft_id, draft));
        }
        Value::String(text) => {
            ui.text_edit_singleline(text);
        }
        _ => {
            ui.label(value.to_string());
        }
    }
}
pub fn error(ui: &mut Ui, message: &str) {
    ui.label(RichText::new(message).color(Color32::from_rgb(245, 125, 115)));
}

pub fn thermal_boundary<T: serde::Serialize + serde::de::DeserializeOwned>(
    ui: &mut Ui,
    id: &str,
    boundary: &mut T,
) {
    use flursys::thermal::Boundary as B;
    let Ok(mut b) = serde_json::to_value(&*boundary).and_then(serde_json::from_value::<B>) else {
        return;
    };
    let mut kind = match b {
        B::Adiabatic => 0,
        B::FixedTemperature { .. } => 1,
        B::InletOutlet { .. } => 2,
        B::HeatFlux { .. } => 3,
        B::Convection { .. } => 4,
        B::Radiation { .. } => 5,
    };
    let previous = kind;
    egui::ComboBox::from_id_salt((id, "thermal-bc"))
        .selected_text(
            [
                "Adiabatic",
                "Fixed temperature",
                "Inlet / outlet temperature",
                "Inward heat flux",
                "External convection",
                "Enclosure radiation",
            ][kind],
        )
        .show_ui(ui, |ui| {
            for (i, n) in [
                "Adiabatic",
                "Fixed temperature",
                "Inlet / outlet temperature",
                "Inward heat flux",
                "External convection",
                "Enclosure radiation",
            ]
            .iter()
            .enumerate()
            {
                ui.selectable_value(&mut kind, i, *n);
            }
        });
    if kind != previous {
        b = match kind {
            1 => B::FixedTemperature {
                temperature: 293.15,
            },
            2 => B::InletOutlet {
                incoming_temperature: 293.15,
            },
            3 => B::HeatFlux { inward_flux: 0. },
            4 => B::Convection {
                coefficient: 10.,
                ambient_temperature: 293.15,
            },
            5 => B::Radiation {
                emissivity: 0.8,
                ambient_temperature: 293.15,
            },
            _ => B::Adiabatic,
        };
    }
    match &mut b {
        B::FixedTemperature { temperature } => number(ui, "Temperature (K)", temperature),
        B::InletOutlet {
            incoming_temperature,
        } => number(ui, "Incoming temperature (K)", incoming_temperature),
        B::HeatFlux { inward_flux } => number(ui, "Inward heat flux (W/m²)", inward_flux),
        B::Convection {
            coefficient,
            ambient_temperature,
        } => {
            number(ui, "Heat transfer coefficient (W/m² K)", coefficient);
            number(ui, "Ambient temperature (K)", ambient_temperature);
        }
        B::Radiation {
            emissivity,
            ambient_temperature,
        } => {
            number(ui, "Emissivity", emissivity);
            number(ui, "Ambient temperature (K)", ambient_temperature);
        }
        _ => {}
    }
    if let Ok(updated) = serde_json::to_value(b).and_then(serde_json::from_value::<T>) {
        *boundary = updated;
    }
}

pub fn dialog_frame(ctx: &egui::Context) -> egui::Frame {
    egui::Frame::window(&ctx.global_style())
        .inner_margin(18.)
        .corner_radius(10.)
        .stroke(egui::Stroke::new(
            1.,
            ctx.global_style()
                .visuals
                .widgets
                .noninteractive
                .bg_stroke
                .color,
        ))
}
