//! cargo run --no-default-features --example headless_sweep -- BASE_WORKSPACE OUTPUT_ROOT NU...
//! Each viscosity runs in its own new workspace; existing directories are rejected.
use flursys::{headless, workbench::save_workspace};
fn main() -> Result<(), String> {
    let mut args = std::env::args_os().skip(1);
    let base = std::path::PathBuf::from(args.next().ok_or("Missing base workspace")?);
    let root = std::path::PathBuf::from(args.next().ok_or("Missing output root")?);
    let viscosities: Vec<f64> = args
        .map(|s| {
            s.to_string_lossy()
                .parse::<f64>()
                .map_err(|_| "Invalid viscosity".to_string())
        })
        .collect::<Result<_, _>>()?;
    if viscosities.is_empty() || viscosities.iter().any(|v| !v.is_finite() || *v <= 0.0) {
        return Err("Supply finite positive viscosities".into());
    }
    // Check all destinations before starting any solve.
    for i in 0..viscosities.len() {
        if root.join(format!("variant-{i:04}")).exists() {
            return Err(format!("Variant {i} already exists"));
        }
    }
    for (i, nu) in viscosities.into_iter().enumerate() {
        let (source, mut session) = headless::load(&base)?;
        session
            .set_material(session.material().density, nu)
            .map_err(|e| e.to_string())?;
        let mut project = session.to_project(format!("{} · nu={nu:e}", source.name));
        let directory = root.join(format!("variant-{i:04}"));
        save_workspace(&directory, &project).map_err(|e| e.to_string())?;
        headless::mesh(
            &directory,
            &mut project,
            &mut session,
            &flursys::GmshMesher::auto(),
        )?;
        let outcome = headless::run(
            &directory,
            &mut project,
            &mut session,
            &Default::default(),
            |_| {},
        )?;
        println!(
            "{}",
            serde_json::to_string(&outcome).map_err(|e| e.to_string())?
        );
        if let Some(error) = outcome.failure {
            return Err(format!("Variant {i}: {error}"));
        }
    }
    Ok(())
}
