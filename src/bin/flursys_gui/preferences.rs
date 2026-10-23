//! Per-user display preferences, separate from CFD project configuration.
use super::*;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
#[serde(default)]
struct Preferences {
    gmsh_path: String,
    browser: bool,
    topology: bool,
    panel_size: i32,
}
impl Default for Preferences {
    fn default() -> Self {
        Self {
            gmsh_path: String::new(),
            browser: true,
            topology: false,
            panel_size: 1,
        }
    }
}

pub(super) fn bind(ui: &MainWindow) {
    let path = user_settings_root().map(|root| root.join("interface.json"));
    if let Some(path) = &path {
        match std::fs::read(path) {
            Ok(bytes) => match serde_json::from_slice::<Preferences>(&bytes) {
                Ok(prefs) => {
                    ui.set_preferences_gmsh_path(prefs.gmsh_path.clone().into());
                    GmshMesher::set_executable_override(
                        (!prefs.gmsh_path.is_empty()).then(|| PathBuf::from(&prefs.gmsh_path)),
                    );
                    ui.set_show_browser(prefs.browser);
                    ui.set_show_topology(prefs.topology);
                    ui.set_preferences_panel_size(prefs.panel_size.clamp(0, 2));
                }
                Err(error) => eprintln!("Cannot read interface preferences: {error}"),
            },
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
            Err(error) => eprintln!("Cannot load interface preferences: {error}"),
        }
    }
    let browse = ui.as_weak();
    ui.on_browse_gmsh(move || {
        let Some(ui) = browse.upgrade() else {
            return;
        };
        if let Some(path) = rfd::FileDialog::new()
            .set_title("Select Gmsh executable")
            .pick_file()
        {
            ui.set_gmsh_path_draft(path.to_string_lossy().to_string().into());
        }
    });
    let weak = ui.as_weak();
    ui.on_save_preferences(move || {
        let (Some(ui), Some(path)) = (weak.upgrade(), path.as_ref()) else {
            return;
        };
        let prefs = Preferences {
            gmsh_path: ui.get_preferences_gmsh_path().trim().to_string(),
            browser: ui.get_show_browser(),
            topology: ui.get_show_topology(),
            panel_size: ui.get_preferences_panel_size(),
        };
        GmshMesher::set_executable_override(
            (!prefs.gmsh_path.is_empty()).then(|| PathBuf::from(&prefs.gmsh_path)),
        );
        ui.invoke_refresh_gmsh_status();
        let save = || -> Result<(), Box<dyn std::error::Error>> {
            let parent = path.parent().ok_or("Missing preferences directory")?;
            std::fs::create_dir_all(parent)?;
            let mut file = tempfile::NamedTempFile::new_in(parent)?;
            serde_json::to_writer_pretty(file.as_file_mut(), &prefs)?;
            file.as_file().sync_all()?;
            file.persist(path)?;
            Ok(())
        };
        if let Err(error) = save() {
            eprintln!("Cannot save interface preferences: {error}");
        }
    });
}
