use std::path::Path;

#[test]
fn workbench_ui_is_composed_from_section_modules() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let main = std::fs::read_to_string(root.join("src/bin/flursys_egui/mod.rs")).unwrap();
    for module in [
        "cad",
        "panels",
        "post",
        "viewport",
        "widgets",
        "transport",
        "continuation",
        "migration",
    ] {
        assert!(
            root.join(format!("src/bin/flursys_egui/{module}.rs"))
                .is_file(),
            "missing UI module: {module}"
        );
        assert!(
            main.contains(&format!("mod {module};")),
            "native workbench must import {module}"
        );
    }
    let entry = std::fs::read_to_string(root.join("src/bin/flursys_gui.rs")).unwrap();
    assert!(entry.contains("flursys_egui/mod.rs"));
    assert!(!entry.contains("slint"));
}
