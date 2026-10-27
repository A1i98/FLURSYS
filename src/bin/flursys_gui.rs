#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]

#[path = "flursys_egui/mod.rs"]
mod app;
fn main() -> eframe::Result {
    app::run()
}
