//! FLURSYS GUI binary entry point.
//!
//! Application state, controllers, and UI adapters live below
//! `flursys_gui/`; this binary performs only startup wiring.

#[path = "flursys_gui/app.rs"]
mod app;

fn main() -> Result<(), slint::PlatformError> {
    app::run()
}
