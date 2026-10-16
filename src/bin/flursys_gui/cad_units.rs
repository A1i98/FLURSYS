//! CAD property quantities: internal CAD coordinates retain the existing mm
//! convention. Changing the display unit never rescales the document/imports.
use super::*;
fn factor(unit: i32) -> f64 {
    match unit {
        1 => 10.0,
        2 => 1000.0,
        3 => 25.4,
        _ => 1.0,
    }
}
fn quantity(text: &str, default: f64, angle: bool) -> Result<f64, String> {
    let text = text.trim().to_lowercase();
    let suffixes: &[(&str, f64)] = if angle {
        &[
            ("degrees", 1.0),
            ("deg", 1.0),
            ("°", 1.0),
            ("rad", 180.0 / std::f64::consts::PI),
        ]
    } else {
        &[("mm", 1.0), ("cm", 10.0), ("in", 25.4), ("m", 1000.0)]
    };
    let (value, scale) = suffixes
        .iter()
        .find_map(|(suffix, factor)| text.strip_suffix(suffix).map(|v| (v.trim(), *factor)))
        .unwrap_or((&text, default));
    let value = value
        .parse::<f64>()
        .map_err(|_| "Enter a number, optionally with a unit suffix".to_string())?
        * scale;
    if value.is_finite() {
        Ok(value)
    } else {
        Err("Quantity must be finite".into())
    }
}
pub(super) fn length(ui: &MainWindow, value: SharedString, label: &str) -> Result<f64, String> {
    quantity(&value, factor(ui.get_cad_length_unit()), false).map_err(|e| format!("{label}: {e}"))
}
pub(super) fn angle(ui: &MainWindow, value: SharedString) -> Result<f64, String> {
    quantity(
        &value,
        if ui.get_cad_angle_unit() == 1 {
            180.0 / std::f64::consts::PI
        } else {
            1.0
        },
        true,
    )
    .map_err(|e| format!("Angle: {e}"))
}
fn formatted(value: f64) -> SharedString {
    // Keep enough significant digits for repeated unit changes and small CAD
    // tolerances. Avoid rounding a valid small value to zero.
    value.to_string().into()
}
pub(super) fn length_symbol(ui: &MainWindow) -> &'static str {
    match ui.get_cad_length_unit() {1=>"cm",2=>"m",3=>"in",_=>"mm"}
}
pub(super) fn display_length(ui: &MainWindow, value: f64) -> SharedString {
    formatted(value / factor(ui.get_cad_length_unit()))
}
pub(super) fn display_angle(ui: &MainWindow, value: f64) -> SharedString {
    formatted(if ui.get_cad_angle_unit() == 1 {
        value.to_radians()
    } else {
        value
    })
}
pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = shared.clone();
    ui.on_cad_units_changed(move |old_length, old_angle| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let fields = [
            ui.get_cad_size_x(),
            ui.get_cad_size_y(),
            ui.get_cad_size_z(),
            ui.get_cad_pos_x(),
            ui.get_cad_pos_y(),
            ui.get_cad_pos_z(),
            ui.get_cad_revolve_x(),
            ui.get_cad_revolve_y(),
            ui.get_cad_revolve_z(),
            ui.get_cad_sketch_offset(),
            ui.get_cad_tolerance(),
        ];
        for (index, text) in fields.into_iter().enumerate() {
            if ui.get_cad_operation() == 42 && index < 3 {
                continue;
            }
            if let Ok(value) = quantity(&text, factor(old_length), false) {
                let value = display_length(&ui, value);
                match index {
                    0 => ui.set_cad_size_x(value),
                    1 => ui.set_cad_size_y(value),
                    2 => ui.set_cad_size_z(value),
                    3 => ui.set_cad_pos_x(value),
                    4 => ui.set_cad_pos_y(value),
                    5 => ui.set_cad_pos_z(value),
                    6 => ui.set_cad_revolve_x(value),
                    7 => ui.set_cad_revolve_y(value),
                    8 => ui.set_cad_revolve_z(value),
                    9 => ui.set_cad_sketch_offset(value),
                    _ => ui.set_cad_tolerance(value),
                }
            }
        }
        if let Ok(value) = quantity(
            &ui.get_cad_angle(),
            if old_angle == 1 {
                180.0 / std::f64::consts::PI
            } else {
                1.0
            },
            true,
        ) {
            ui.set_cad_angle(display_angle(&ui, value));
        }
        let state = shared.borrow();
        refresh_ui(&ui, &state);
    });
}
