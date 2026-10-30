//! CLI for the same project, mesh and solver objects as the desktop Workbench.
#[path = "catalog.rs"]
mod catalog;
#[path = "compressible.rs"]
mod compressible;
#[path = "transport.rs"]
mod transport;
use flursys::workbench::{catalogue, mesh_storage, solve_review};
use flursys::*;
use serde_json::{json, Value};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};
fn emit(value: Value, json_output: bool) {
    if json_output {
        println!("{}", serde_json::to_string_pretty(&value).unwrap());
    } else if let Some(s) = value.get("summary").and_then(Value::as_str) {
        println!("{s}");
    } else {
        println!("{}", serde_json::to_string_pretty(&value).unwrap());
    }
}
fn parse(args: &[String], allowed: &[&str]) -> Result<BTreeMap<String, String>, String> {
    let mut result = BTreeMap::new();
    let mut i = 0;
    while i < args.len() {
        let name = args[i]
            .strip_prefix("--")
            .ok_or_else(|| format!("Unexpected argument '{}'", args[i]))?;
        if !allowed.contains(&name) {
            return Err(format!("Unknown option --{name}"));
        }
        let value = if name == "json"
            || name == "gpu-required"
            || name == "include-gpu"
            || name == "resume-latest"
        {
            "true".into()
        } else {
            i += 1;
            args.get(i)
                .filter(|v| !v.starts_with("--"))
                .ok_or_else(|| format!("Missing value for --{name}"))?
                .clone()
        };
        if result.insert(name.into(), value).is_some() {
            return Err(format!("Repeated option --{name}"));
        }
        i += 1;
    }
    Ok(result)
}
fn number(opts: &BTreeMap<String, String>, key: &str) -> Result<Option<usize>, String> {
    opts.get(key)
        .map(|s| {
            s.parse()
                .map_err(|_| format!("--{key} requires a nonnegative integer"))
        })
        .transpose()
}
pub fn help() {
    println!(
        "Workbench commands:
  flursys mcp [--root DIRECTORY]
  flursys catalog status|materials [--db FILE] [--json]
  flursys catalog import PACK.json [--db FILE] [--json]
  flursys catalog inspect TABLE [--db FILE] [--json]
  flursys catalog evaluate ID --temperature K --pressure PA [--db FILE] [--json]
  flursys catalog apply-material PROJECT --id ID --temperature K --pressure PA [--db FILE]
  flursys catalog apply-profile PROJECT --id ID [--patch NAME] [--db FILE]
  flursys catalog apply-radiation PROJECT --id ID --patch NAME --temperature K --ambient K [--db FILE]
  flursys catalog reference ID [--db FILE] [--json]
  flursys catalog backup --output NEW.sqlite3 [--db FILE]
  flursys devices [--json]
  flursys examples list [--json]
  flursys examples create <id> <directory> [--name NAME] [--preset ID] [--json]
  flursys examples export-templates <directory> [--json]
  flursys compressible CASE.json|WORKSPACE_DIRECTORY OUTPUT_DIRECTORY
  flursys project inspect <directory> [--json]
  flursys project check <directory> [--json]
  flursys project checkpoint <directory> [--json]
  flursys project analyses <directory> [--json]
  flursys project branch <directory> --name NAME [--json]
  flursys project mesh <directory> [--gmsh PATH] [--threads N] [--json]
  flursys project import-mesh <directory> --input mesh.msh [--json]
  flursys project solve <directory> [--threads N] [--iterations N] [--restart FILE] [--backend serial|parallel|gpu] [--adapter NAME] [--gpu-required] [--gpu-min-rows N] [--json]
  flursys project transport-template <directory> --output transport.json [--json]
  flursys project transport <directory> --config transport.json --output NEW_DIRECTORY [--json]
  flursys project benchmark <directory> [--threads 1,2,4] [--iterations N] [--include-gpu] [--gpu-min-rows N] [--json]
  flursys project rename <directory> --name NAME [--json]

Project commands use the GUI Workbench backend and saved mesh assets.
--iterations is the outer-corrector budget (per physical step for transient cases).
Restart requires the exact saved mesh; solve does not silently generate a mesh.
Progress goes to stderr; --json keeps stdout machine-readable.
Transient controls: --dt SECONDS --steps N --end-time SECONDS --max-diffusion NUMBER
Playback cadence: --checkpoint-every N (0 disables playback writes)
Recovery: --recovery-every N --recovery-seconds N --resume-latest
Playback limits: --history-max-frames N --history-max-mib N (0 unlimited)
GPU controls: --gpu-preconditioner auto|jacobi --gpu-restart 2..64
  --gpu-inner-iterations 1..64 --gpu-smoothing 1..8 --gpu-timeout-ms 100..60000
  --gpu-basis-mb 16..16384. Small-row crossover applies only when fallback is allowed."
    );
}
pub fn run(args: &[String]) -> Result<(), String> {
    if args.first().is_some_and(|s| s == "compressible") {
        return compressible::run(&args[1..]);
    }
    if args.iter().any(|s| s == "--help" || s == "-h") {
        help();
        return Ok(());
    }
    if args.first().is_some_and(|s| s == "catalog") {
        return catalog::run(args);
    }
    if args.first().is_some_and(|s| s == "devices") {
        let opts = parse(&args[1..], &["json"])?;
        let adapters = flursys::compute::devices()?;
        emit(
            json!({"summary":format!("{} CPU logical processors\n{}",std::thread::available_parallelism().map_or(1,|n|n.get()),adapters.iter().map(|a|format!("{} · {} · {}",a.name,a.backend,a.device_type)).collect::<Vec<_>>().join("\n")),"cpu_logical_processors":std::thread::available_parallelism().map_or(1,|n|n.get()),"gpu_adapters":adapters}),
            opts.contains_key("json"),
        );
        return Ok(());
    }
    if args.len() < 2 || args.iter().any(|s| s == "--help" || s == "-h") {
        help();
        return Ok(());
    }
    if args[0] == "examples" {
        let templates = catalogue::builtin_templates()?;
        match args[1].as_str() {
            "list" => {
                let opts = parse(&args[2..], &["json"])?;
                emit(
                    json!({"summary":templates.iter().map(|t|format!("{} · {}",t.id,t.name)).collect::<Vec<_>>().join("\n"),
                    "examples":templates.iter().map(|t|json!({"id":t.id,"name":t.name,"dimension":format!("{:?}",t.project_template.mesh.dimension),"description":t.description,"notes":t.expectations.notes,"metadata":t.metadata.summary(),"case_format_version":t.format_version})).collect::<Vec<_>>()}),
                    opts.contains_key("json"),
                );
            }
            "create" => {
                let id = args.get(2).ok_or("Missing example ID")?;
                let folder = PathBuf::from(args.get(3).ok_or("Missing destination directory")?);
                let opts = parse(&args[4..], &["name", "preset", "json"])?;
                if folder.exists()
                    && folder
                        .read_dir()
                        .map_err(|e| e.to_string())?
                        .next()
                        .is_some()
                {
                    return Err(
                        "Destination must be empty; existing projects are never overwritten."
                            .into(),
                    );
                }
                let template = templates
                    .into_iter()
                    .find(|t| &t.id == id)
                    .ok_or("Unknown example ID; run flursys examples list")?;
                let mut project = flursys::workbench::case_format::instantiate(
                    &template,
                    opts.get("preset").map(String::as_str),
                )?;
                catalogue::rename_project(
                    &mut project,
                    opts.get("name")
                        .map_or(template.name.as_str(), String::as_str),
                )?;
                save_workspace(&folder, &project).map_err(|e| e.to_string())?;
                emit(
                    json!({"summary":format!("Created {} at {}",project.name,folder.display()),"name":project.name,"workspace":folder}),
                    opts.contains_key("json"),
                );
            }
            "export-templates" => {
                let folder = PathBuf::from(args.get(2).ok_or("Missing template destination")?);
                let opts = parse(&args[3..], &["json"])?;
                for template in &templates {
                    flursys::workbench::save_case_template(&folder, template)
                        .map_err(|e| e.to_string())?;
                }
                emit(
                    json!({"summary":format!("Exported {} current Workbench templates to {}",templates.len(),folder.display()),"count":templates.len()}),
                    opts.contains_key("json"),
                );
            }
            other => return Err(format!("Unknown examples command '{other}'")),
        }
        return Ok(());
    }
    if args[0] != "project" {
        return Err(format!(
            "Unknown command '{}'. Use flursys --help.",
            args[0]
        ));
    }
    let action = args[1].as_str();
    let folder = Path::new(args.get(2).ok_or("Missing project directory")?);
    let allowed: &[&str] = match action {
        "inspect" | "check" | "checkpoint" | "analyses" => &["json"],
        "transport-template" => &["json", "output"],
        "transport" => &["json", "config", "output", "restart"],
        "benchmark" => &[
            "json",
            "threads",
            "iterations",
            "include-gpu",
            "adapter",
            "gpu-min-rows",
            "gpu-preconditioner",
            "gpu-restart",
            "gpu-inner-iterations",
            "gpu-smoothing",
            "gpu-timeout-ms",
            "gpu-basis-mb",
        ],
        "mesh" => &["json", "gmsh", "threads"],
        "import-mesh" => &["json", "input"],
        "solve" => &[
            "dt",
            "steps",
            "end-time",
            "max-diffusion",
            "checkpoint-every",
            "recovery-every",
            "recovery-seconds",
            "history-max-frames",
            "history-max-mib",
            "resume-latest",
            "json",
            "threads",
            "iterations",
            "restart",
            "gpu-min-rows",
            "gpu-preconditioner",
            "gpu-restart",
            "gpu-inner-iterations",
            "gpu-smoothing",
            "gpu-timeout-ms",
            "gpu-basis-mb",
            "backend",
            "adapter",
            "gpu-required",
        ],
        "rename" | "branch" => &["json", "name"],
        _ => return Err(format!("Unknown project command '{action}'")),
    };
    let opts = parse(&args[3..], allowed)?;
    let json_output = opts.contains_key("json");
    if let Some(path) = opts.get("gmsh") {
        GmshMesher::set_executable_override(Some(PathBuf::from(path)));
    }
    let mut project = load_workspace(folder).map_err(|e| e.to_string())?;
    if action == "analyses" {
        let (root, graph) = flursys::workbench::analysis_graph::load(folder)?;
        emit(
            json!({"summary":format!("{} linked analyses",graph.analyses.len()),"root":root,"workbench":graph}),
            json_output,
        );
        return Ok(());
    }
    if action == "branch" {
        let path = flursys::workbench::analysis_graph::branch(
            folder,
            &project,
            opts.get("name").ok_or("branch requires --name")?,
        )?;
        emit(
            json!({"summary":format!("Analysis created: {}",path.display()),"workspace":path}),
            json_output,
        );
        return Ok(());
    }
    if action == "rename" {
        catalogue::rename_project(
            &mut project,
            opts.get("name").ok_or("rename requires --name")?,
        )?;
        save_workspace(folder, &project).map_err(|e| e.to_string())?;
        emit(
            json!({"summary":format!("Project renamed to {}",project.name),"name":project.name}),
            json_output,
        );
        return Ok(());
    }
    if let Some(backend) = opts.get("backend") {
        project.solver.compute.backend = match backend.as_str() {
            "serial" => flursys::compute::Backend::Serial,
            "parallel" => flursys::compute::Backend::Parallel,
            "gpu" => flursys::compute::Backend::Gpu,
            _ => return Err("--backend must be serial, parallel or gpu".into()),
        };
    }
    if let Some(value) = opts.get("gpu-preconditioner") {
        project.solver.compute.gpu_preconditioner = match value.as_str() {
            "auto" => flursys::compute::GpuPreconditioner::Auto,
            "jacobi" => flursys::compute::GpuPreconditioner::Jacobi,
            _ => return Err("--gpu-preconditioner must be auto or jacobi".into()),
        };
    }
    for (key, field) in [
        ("gpu-restart", &mut project.solver.compute.gpu_restart),
        (
            "gpu-inner-iterations",
            &mut project.solver.compute.gpu_inner_iterations,
        ),
        (
            "gpu-smoothing",
            &mut project.solver.compute.gpu_smoothing_sweeps,
        ),
        (
            "gpu-basis-mb",
            &mut project.solver.compute.gpu_host_basis_mb,
        ),
    ] {
        if let Some(value) = number(&opts, key)? {
            *field = value;
        }
    }
    if let Some(value) = number(&opts, "gpu-timeout-ms")? {
        project.solver.compute.gpu_timeout_ms = value as u64;
    }
    if let Some(rows) = number(&opts, "gpu-min-rows")? {
        project.solver.compute.gpu_min_rows = rows;
    }
    if let Some(adapter) = opts.get("adapter") {
        project.solver.compute.adapter = adapter.clone();
    }
    if opts.contains_key("gpu-required") {
        project.solver.compute.backend = flursys::compute::Backend::Gpu;
        project.solver.compute.cpu_fallback = false;
    }
    let mut session = WorkbenchSession::from_project(&project).map_err(|e| e.to_string())?;
    let restored =
        mesh_storage::restore(folder, &project, &mut session)? || session.mesh().is_some();
    if action == "transport-template" || action == "transport" {
        let mesh = &session
            .mesh()
            .ok_or("A current saved mesh is required for scalar transport")?
            .mesh;
        let output = Path::new(opts.get("output").ok_or("Missing --output")?);
        let report = if action == "transport-template" {
            transport::template(mesh, output)?
        } else {
            transport::execute(
                mesh,
                Path::new(opts.get("config").ok_or("Missing --config")?),
                output,
                opts.get("restart").map(|p| Path::new(p)),
            )?
        };
        emit(report, json_output);
        return Ok(());
    }
    if action == "checkpoint" {
        let mesh = &session
            .mesh()
            .ok_or("Restore the project mesh before inspecting recovery")?
            .mesh;
        if let Some(settings) = &project.solver.compressible {
            let path = flursys::workbench::flow_frame::discover(folder, mesh, settings.gas)?;
            let info = flursys::workbench::checkpoint::inspect(&path)?;
            emit(
                json!({"summary":format!("Recovery checkpoint at iteration/step {}",info.iteration),"checkpoint":info}),
                json_output,
            );
            return Ok(());
        }
        let recovered = flursys::workbench::checkpoint::discover(folder, mesh)?;
        emit(
            json!({"summary":format!("Recovery checkpoint at iteration/step {}",recovered.info.iteration),"checkpoint":recovered.info,"warnings":recovered.warnings}),
            json_output,
        );
        return Ok(());
    }
    if action == "inspect" {
        emit(
            json!({"summary":format!("{}\n{}\nDimension {:?} · mesh restored: {} · {} saved runs",project.name,folder.display(),session.mesh_dimension(),restored,project.runs.len()),
            "name":project.name,"dimension":format!("{:?}",session.mesh_dimension()),"mesh_restored":restored,"runs":project.runs.len(),"ready":session.readiness().is_ok(),"compute":session.solver_options().compute}),
            json_output,
        );
        return Ok(());
    }
    if action == "import-mesh" {
        let path = opts.get("input").ok_or("Missing --input mesh.msh")?;
        flursys::headless::import_mesh(folder, &mut project, &mut session, Path::new(path))?;
        emit(
            json!({"summary":format!("Imported {} cells", session.mesh().unwrap().mesh.cell_count())}),
            json_output,
        );
        return Ok(());
    }
    if action == "mesh" {
        if let Some(n) = number(&opts, "threads")? {
            session.set_mesh_cpu_threads(n);
        }
        flursys::headless::mesh(folder, &mut project, &mut session, &GmshMesher::auto())?;
        emit(
            json!({"summary":format!("Mesh saved: {} cells",session.mesh().unwrap().mesh.cell_count()),"cells":session.mesh().unwrap().mesh.cell_count()}),
            json_output,
        );
        return Ok(());
    }
    if project.solver.compressible.is_some() {
        return compressible_workspace(
            action,
            folder,
            &mut project,
            &mut session,
            &opts,
            json_output,
        );
    }
    if action == "solve" {
        use flursys::unstructured_incompressible::TimeIntegration as Time;
        if ["dt", "steps", "end-time", "max-diffusion"]
            .iter()
            .any(|k| opts.contains_key(*k))
        {
            let mut time = session.solver_options().time;
            let real = |name: &str| -> Result<Option<f64>, String> {
                opts.get(name)
                    .map(|v| v.parse::<f64>().map_err(|_| format!("Invalid --{name}")))
                    .transpose()
            };
            let step = real("dt")?;
            let count = number(&opts, "steps")?;
            match &mut time {
                Time::Steady=>return Err("Time controls require a transient project; choose Euler or BDF2 in project settings".into()),
                Time::Euler{dt,steps}|Time::Backward{dt,steps}|Time::AdaptiveEuler{dt,steps,..}|Time::AdaptiveBackward{dt,steps,..}=>{if let Some(v)=step {*dt=v;}if let Some(v)=count {*steps=v;}}
            }
            if let Some(max) = real("max-diffusion")? {
                match &mut time {
                    Time::AdaptiveEuler { controls, .. }
                    | Time::AdaptiveBackward { controls, .. } => controls.max_diffusion = Some(max),
                    _ => return Err("--max-diffusion requires adaptive time integration".into()),
                }
            }
            session
                .set_flow_models(time, session.solver_options().turbulence.clone())
                .map_err(|e| e.to_string())?;
            if let Some(end) = real("end-time")? {
                session
                    .set_physical_end_time(Some(end))
                    .map_err(|e| e.to_string())?;
            }
        }
        if let Some(n) = number(&opts, "threads")? {
            let available = std::thread::available_parallelism().map_or(1, |n| n.get());
            if n > available {
                return Err(format!(
                    "--threads must be 0 (automatic) or at most {available}"
                ));
            }
            session.set_cpu_threads(n);
        }
        if let Some(n) = number(&opts, "iterations")? {
            if n == 0 || n > 1_000_000 {
                return Err("--iterations must be in 1..1000000".into());
            }
            let s = session.solver_options().clone();
            session
                .set_solver_controls(
                    n,
                    s.velocity_relaxation,
                    s.pressure_relaxation,
                    s.continuity_absolute_tolerance,
                )
                .map_err(|e| e.to_string())?;
        }
    }
    session
        .readiness()
        .map_err(|e| format!("Project is not ready: {e}"))?;
    let case = session.prepare_case().map_err(|e| e.to_string())?;
    let review = solve_review::review(&case, &project.plane_monitors);
    if action == "check" {
        emit(
            json!({"summary":review.summary(),"blocked":review.blocked(),"errors":review.errors,"warnings":review.warnings}),
            json_output,
        );
        return if review.blocked() {
            Err("Pre-run checks failed".into())
        } else {
            Ok(())
        };
    }
    if review.blocked() {
        return Err(review.summary());
    }
    if action == "benchmark" {
        return benchmark(&case, &opts, json_output);
    }
    if opts.contains_key("resume-latest") && opts.contains_key("restart") {
        return Err("Use either --resume-latest or --restart".into());
    }
    let restart = if opts.contains_key("resume-latest") {
        let recovered = flursys::workbench::checkpoint::discover(folder, &case.mesh)?;
        for warning in &recovered.warnings {
            eprintln!("Recovery: {warning}");
        }
        eprintln!(
            "Resuming checkpoint at iteration {}",
            recovered.info.iteration
        );
        Some(recovered.info.path)
    } else {
        opts.get("restart").map(PathBuf::from)
    };
    let run_options = execution_options(&opts, restart)?;
    let outcome =
        flursys::headless::run(folder, &mut project, &mut session, &run_options, |event| {
            use flursys::app::{JobProgress as P, WorkbenchJobEvent as E};
            match event {
                E::SolveProgress(P::Iteration {
                    current,
                    max,
                    residual,
                }) if *current == 1 || current % 10 == 0 || current == max => eprintln!(
                    "step {current}/{max} · continuity {:.6e}",
                    residual.unwrap_or(f64::NAN)
                ),
                E::Checkpoint(Ok(info)) => eprintln!(
                    "Checkpoint saved: iteration {} · {}",
                    info.iteration,
                    info.path.display()
                ),
                E::Checkpoint(Err(e)) => eprintln!("Checkpoint failed: {e}"),
                E::SolveProgress(P::Message(message)) => eprintln!("{message}"),
                _ => {}
            }
        })?;
    emit(
        json!({"summary":outcome.failure.clone().unwrap_or_else(||format!("Run saved: {:?}",session.status())),"run":outcome.run,"restart_checkpoint":outcome.restart_checkpoint,"failed":outcome.failure.is_some()}),
        json_output,
    );
    if let Some(e) = outcome.failure {
        return Err(e);
    }
    Ok(())
}

fn execution_options(
    opts: &BTreeMap<String, String>,
    restart: Option<PathBuf>,
) -> Result<flursys::headless::RunOptions, String> {
    let mut recovery = flursys::workbench::checkpoint::RecoveryPolicy::default();
    recovery.every_iterations = number(opts, "recovery-every")?.unwrap_or(100);
    recovery.every_seconds = number(opts, "recovery-seconds")?.unwrap_or(60) as u64;
    recovery.max_history_frames = number(opts, "history-max-frames")?.unwrap_or(500);
    let mib = number(opts, "history-max-mib")?.unwrap_or(1024) as u64;
    recovery.max_history_bytes = mib
        .checked_mul(1024 * 1024)
        .ok_or("History size is too large")?;
    Ok(flursys::headless::RunOptions {
        restart,
        recovery,
        checkpoint_every: number(opts, "checkpoint-every")?.unwrap_or(100),
    })
}
fn compressible_workspace(
    action: &str,
    folder: &Path,
    project: &mut WorkbenchProject,
    session: &mut WorkbenchSession,
    opts: &BTreeMap<String, String>,
    json_output: bool,
) -> Result<(), String> {
    if action == "benchmark" {
        return Err(
            "Use project solve with explicit backend and thread settings to measure this model"
                .into(),
        );
    }
    let settings = project
        .solver
        .compressible
        .as_mut()
        .ok_or("Missing model configuration")?;
    if let Some(n) = number(opts, "steps")?.or(number(opts, "iterations")?) {
        if n == 0 || n > 10_000_000 {
            return Err("Steps must be in 1..10000000".into());
        }
        settings.max_steps = n;
    }
    for (name, target) in [
        ("dt", &mut settings.max_time_step),
        ("end-time", &mut settings.end_time),
    ] {
        if let Some(text) = opts.get(name) {
            *target = text
                .parse::<f64>()
                .map_err(|_| format!("Invalid --{name}"))?;
        }
    }
    if opts.contains_key("max-diffusion") {
        return Err("The explicit acoustic/diffusive stability bound is computed from the flow state; use --dt as its upper bound".into());
    }
    session
        .set_compressible_settings(Some(settings.clone()))
        .map_err(|e| e.to_string())?;
    if let Some(threads) = number(opts, "threads")? {
        session.set_cpu_threads(threads);
        project.solver.threads = threads;
    }
    session.readiness()?;
    if action == "check" {
        emit(
            json!({"summary":"Flow case is ready","ready":true,"model":"compressible"}),
            json_output,
        );
        return Ok(());
    }
    if opts.contains_key("resume-latest") && opts.contains_key("restart") {
        return Err("Use either --resume-latest or --restart".into());
    }
    let restart = if opts.contains_key("resume-latest") {
        Some(flursys::workbench::flow_frame::discover(
            folder,
            &session.mesh().ok_or("No mesh")?.mesh,
            settings.gas,
        )?)
    } else {
        opts.get("restart").map(PathBuf::from)
    };
    let outcome = flursys::headless::run(
        folder,
        project,
        session,
        &execution_options(opts, restart)?,
        |event| {
            use flursys::app::{JobProgress as P, WorkbenchJobEvent as E};
            match event {
                E::SolveProgress(P::Iteration {
                    current,
                    max,
                    residual,
                }) if current % 10 == 0 || current == max => eprintln!(
                    "step {current}/{max} · mass RHS {:.6e}",
                    residual.unwrap_or(f64::NAN)
                ),
                E::Checkpoint(Err(e)) => eprintln!("Checkpoint failed: {e}"),
                _ => {}
            }
        },
    )?;
    emit(
        json!({"summary":format!("Run saved: {:?}",session.status()),"run":outcome.run,"restart_checkpoint":outcome.restart_checkpoint,"failed":outcome.failure.is_some()}),
        json_output,
    );
    outcome.failure.map_or(Ok(()), Err)
}

/// Explicit user-invoked scaling measurement. No RunRecords, mesh assets,
/// checkpoints or project configuration are modified.
fn benchmark(
    case: &IncompressibleCase,
    opts: &BTreeMap<String, String>,
    json_output: bool,
) -> Result<(), String> {
    let available = std::thread::available_parallelism().map_or(1, |n| n.get());
    let budget = number(opts, "iterations")?.unwrap_or(20);
    if !(1..=1000000).contains(&budget) {
        return Err("--iterations must be in 1..1000000".into());
    }
    let workers = if let Some(list) = opts.get("threads") {
        list.split(',')
            .map(|s| {
                s.parse::<usize>()
                    .map_err(|_| "--threads requires comma-separated positive integers".to_string())
            })
            .collect::<Result<Vec<_>, _>>()?
    } else {
        [1, 2, 4, available]
            .into_iter()
            .filter(|&n| n <= available)
            .collect()
    };
    if workers.is_empty() || workers.iter().any(|&n| n == 0 || n > available) {
        return Err(format!("Benchmark thread counts must be in 1..{available}"));
    }
    let mut unique = Vec::new();
    for n in workers {
        if !unique.contains(&n) {
            unique.push(n);
        }
    }
    let mut runs = unique
        .into_iter()
        .map(|n| (flursys::compute::Backend::Parallel, n))
        .collect::<Vec<_>>();
    if opts.contains_key("include-gpu") {
        runs.push((flursys::compute::Backend::Gpu, available));
    }
    let mut rows = Vec::new();
    let mut reference: Option<(IncompressibleSolution, usize)> = None;
    for (backend, threads) in runs {
        let mut draft = case.clone();
        draft.solver.threads = threads;
        draft.solver.max_outer_iterations = budget;
        draft.solver.compute.backend = backend;
        // A GPU scaling result must not be a disguised CPU fallback.
        draft.solver.compute.cpu_fallback = backend != flursys::compute::Backend::Gpu;
        if let Some(adapter) = opts.get("adapter") {
            draft.solver.compute.adapter = adapter.clone();
        }
        if draft.solver.time.is_transient() {
            return Err("Scaling benchmark currently requires a steady case; use identical saved transient runs for time-step comparisons".into());
        }
        eprintln!("Benchmark: {backend:?}, {threads} workers, up to {budget} iterations");
        let started = std::time::Instant::now();
        match solve_incompressible(&draft) {
            Ok(solution)=>{
                let elapsed=started.elapsed().as_secs_f64();
                let comparison=reference.as_ref().map(|(baseline,workers)|field_comparison(case,baseline,&solution,*workers));
                rows.push(json!({"backend":backend,"threads":threads,"elapsed_seconds":elapsed,"completed_iterations":solution.report.outer_iterations,"status":format!("{:?}",solution.report.status),"performance":solution.report.performance,"field_comparison":comparison}));
                if reference.is_none()&&backend==flursys::compute::Backend::Parallel{reference=Some((solution,threads));}
            },
            Err(e)=>rows.push(json!({"backend":backend,"threads":threads,"elapsed_seconds":started.elapsed().as_secs_f64(),"error":e.to_string()})),
        }
    }
    let failed = rows.iter().any(|row| row.get("error").is_some());
    emit(
        json!({"summary":serde_json::to_string_pretty(&rows).map_err(|e|e.to_string())?,"runs":rows,"notes":"Fresh identical initial fields for each run; no saved project changes. Compare equal completed work only. No numerical accuracy certification."}),
        json_output,
    );
    if failed {
        return Err("One or more scaling configurations failed; inspect the benchmark rows".into());
    }
    Ok(())
}

/// Diagnostics against the first successful CPU run; a comparison is not a
/// reference-accuracy test and different completed work is explicitly labelled.
fn field_comparison(
    case: &IncompressibleCase,
    reference: &IncompressibleSolution,
    current: &IncompressibleSolution,
    workers: usize,
) -> Value {
    let volume = case.mesh.cells().iter().map(|c| c.volume).sum::<f64>();
    let shift = reference
        .pressure
        .values()
        .iter()
        .zip(current.pressure.values())
        .zip(case.mesh.cells())
        .map(|((a, b), cell)| (b - a) * cell.volume)
        .sum::<f64>()
        / volume;
    let mut velocity_error = 0.;
    let mut velocity_scale = 0.;
    let mut pressure_error = 0.;
    let mut pressure_aligned = 0.;
    let mut velocity_max = 0_f64;
    for (i, cell) in case.mesh.cells().iter().enumerate() {
        let du = current.velocity[i] - reference.velocity[i];
        velocity_error += cell.volume * du.norm_squared();
        velocity_scale += cell.volume * reference.velocity[i].norm_squared();
        velocity_max = velocity_max.max(du.norm());
        let dp = current.pressure[i] - reference.pressure[i];
        pressure_error += cell.volume * dp * dp;
        pressure_aligned += cell.volume * (dp - shift).powi(2);
    }
    let flux_error = reference
        .face_flux
        .values()
        .iter()
        .zip(current.face_flux.values())
        .map(|(a, b)| (b - a).powi(2))
        .sum::<f64>();
    let flux_scale = reference
        .face_flux
        .values()
        .iter()
        .map(|v| v * v)
        .sum::<f64>();
    json!({"reference_cpu_threads":workers,"equal_completed_outer_work":reference.report.outer_iterations==current.report.outer_iterations,"velocity_volume_weighted_rms_error":(velocity_error/volume).sqrt(),"velocity_relative_l2_error":(velocity_scale>0.).then(||(velocity_error/velocity_scale).sqrt()),"velocity_max_abs_error":velocity_max,"pressure_volume_weighted_rms_error":(pressure_error/volume).sqrt(),"pressure_gauge_aligned_rms_error":(pressure_aligned/volume).sqrt(),"removed_pressure_mean_shift":shift,"face_flux_relative_l2_error":(flux_scale>0.).then(||(flux_error/flux_scale).sqrt()),"face_flux_rms_error":(flux_error/case.mesh.face_count().max(1) as f64).sqrt(),"note":"Gauge-aligned pressure is diagnostic only; raw error retains differences against prescribed pressure boundaries. Compare equal completed work."})
}
