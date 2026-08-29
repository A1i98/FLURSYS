use std::path::Path;

#[test]
fn workbench_ui_is_composed_from_section_modules() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let main = std::fs::read_to_string(root.join("ui/main.slint")).unwrap();

    for module in [
        "ui/layout/topbar.slint",
        "ui/layout/project_tree.slint",
        "ui/layout/example_gallery.slint",
        "ui/layout/status_bar.slint",
        "ui/layout/project_dialogs.slint",
    ] {
        assert!(root.join(module).is_file(), "missing UI module: {module}");
        assert!(
            main.contains(module.strip_prefix("ui/").unwrap()),
            "main.slint must import {module}",
        );
    }
}
