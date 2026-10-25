//! Compressible solver command for the FLURSYS headless CLI.
use flursys::compressible::{self, Settings};
use serde::Deserialize;
use std::path::{Path, PathBuf};
#[derive(Deserialize)]
struct Input {
    mesh: PathBuf,
    #[serde(default)]
    settings: Settings,
}
pub fn run(args: &[String]) -> Result<(), String> {
    if args.is_empty() || args[0] == "--help" || args[0] == "-h" {
        println!("Usage: flursys compressible CASE.json|WORKSPACE_DIRECTORY OUTPUT_DIRECTORY\nExperimental ideal-gas Euler / laminar Navier–Stokes; Gmsh mesh input, SI units.\nSupports conformal translational cyclic_pairs in settings. Produces solution.vtu and report.json.");
        return Ok(());
    }
    if args.len() != 2 {
        return Err("Expected CASE.json and OUTPUT_DIRECTORY".into());
    }
    let source = Path::new(&args[0]);
    let (mesh, settings, mesh_path) = if source.is_dir() {
        let (project, session) = flursys::headless::load(source)?;
        let settings = project
            .solver
            .compressible
            .ok_or("Workspace has no compressible solver configuration")?;
        let mesh = session
            .mesh()
            .ok_or("Workspace has no saved mesh; generate and save it first")?
            .mesh
            .clone();
        (mesh, settings, source.to_path_buf())
    } else {
        let data = std::fs::read(source).map_err(|e| e.to_string())?;
        let input: Input = serde_json::from_slice(&data).map_err(|e| e.to_string())?;
        let mesh_path = if input.mesh.is_absolute() {
            input.mesh.clone()
        } else {
            source.parent().unwrap_or(Path::new(".")).join(&input.mesh)
        };
        let mesh = flursys::load_gmsh(&mesh_path).map_err(|e| format!("{e:?}"))?;
        (mesh, input.settings, mesh_path)
    };
    eprintln!(
        "Compressible {:?}, {} cells, {:?}; experimental, not benchmark validated",
        settings.flux,
        mesh.cell_count(),
        settings.viscosity
    );
    let start = std::time::Instant::now();
    let solution = compressible::solve(&mesh, &settings)?;
    let elapsed = start.elapsed().as_secs_f64();
    if solution.report.cyclic_face_pairs > 0 {
        eprintln!("Cyclic interfaces: {} matched face pairs; single conservative flux per pair; excluded from external boundary outflow.",solution.report.cyclic_face_pairs);
    }

    let directory = Path::new(&args[1]);
    std::fs::create_dir_all(directory).map_err(|e| e.to_string())?;
    compressible::write_vtu(
        &directory.join("solution.vtu"),
        &mesh,
        &solution,
        settings.gas,
    )?;
    let report = serde_json::json!({"format":"flursys-compressible-v1","validation":"experimental; no accuracy or performance claim","mesh":mesh_path,"cells":mesh.cell_count(),"settings":settings,"solve_wall_seconds":elapsed,"report":solution.report,"units_note":"SI; absolute pressure Pa; temperature K; 2D integrals per unit depth; conservation ordering: mass, x/y/z momentum, total energy"});
    let text = serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?;
    let mut temp = tempfile::NamedTempFile::new_in(directory).map_err(|e| e.to_string())?;
    use std::io::Write;
    temp.write_all(&text).map_err(|e| e.to_string())?;
    temp.as_file().sync_all().map_err(|e| e.to_string())?;
    temp.persist(directory.join("report.json"))
        .map_err(|e| e.to_string())?;
    println!(
        "t={:.8e}; {} accepted steps; {} rejected; reached end={}; elapsed={elapsed:.3}s",
        solution.report.time,
        solution.report.accepted_steps,
        solution.report.rejected_steps,
        solution.report.reached_end_time
    );
    if !solution.report.reached_end_time {
        return Err("Step budget exhausted before end time; partial field saved with reached_end_time=false".into());
    }
    Ok(())
}
