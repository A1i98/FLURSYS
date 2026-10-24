//! cargo run --no-default-features --example headless -- PATH_TO_WORKSPACE
use flursys::{app::WorkbenchJobEvent, headless};
fn main() -> Result<(), String> {
    let path = std::env::args_os()
        .nth(1)
        .map(std::path::PathBuf::from)
        .ok_or(
            "Usage: headless PATH_TO_WORKSPACE (create it with flursys examples create first)",
        )?;
    let (mut project, mut session) = headless::load(&path)?;
    // A parametric driver can edit session controls/material/scripts here.
    if session.mesh().is_none() {
        headless::mesh(
            &path,
            &mut project,
            &mut session,
            &flursys::GmshMesher::auto(),
        )?;
    }
    let result = headless::run(
        &path,
        &mut project,
        &mut session,
        &Default::default(),
        |event| {
            if let WorkbenchJobEvent::SolveProgress(progress) = event {
                eprintln!("{progress:?}");
            }
        },
    )?;
    if let Some(error) = result.failure {
        return Err(error);
    }
    println!("Saved run: {:?}", result.run.map(|r| r.id));
    Ok(())
}
