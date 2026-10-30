use serde_json::{json, Value};
use std::{
    collections::BTreeMap,
    fs,
    io::{self, BufRead, Read, Write},
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
};

const VERSION: &str = "2025-06-18";
const MAX_LINE: usize = 1024 * 1024;
const MAX_OUTPUT: u64 = 128 * 1024;

struct Job {
    child: Child,
    project: PathBuf,
    stdout: PathBuf,
    stderr: PathBuf,
    status: Option<std::process::ExitStatus>,
}
struct SolveJob {
    project: PathBuf,
    control: flursys::headless::ExecutionControl,
    state: std::sync::Arc<std::sync::Mutex<Value>>,
    thread: Option<std::thread::JoinHandle<()>>,
}
struct Server {
    root: PathBuf,
    jobs: BTreeMap<u64, Job>,
    solves: BTreeMap<u64, SolveJob>,
    next: u64,
    temp: tempfile::TempDir,
    initialized: bool,
}

fn err(id: Value, code: i64, message: &str) -> Value {
    json!({"jsonrpc":"2.0","id":id,"error":{"code":code,"message":message}})
}
fn text_result(value: Value, failed: bool) -> Value {
    json!({"content":[{"type":"text","text":serde_json::to_string_pretty(&value).unwrap_or_default()}],"isError":failed})
}
fn arg<'a>(v: &'a Value, key: &str) -> Result<&'a str, String> {
    v.get(key)
        .and_then(Value::as_str)
        .filter(|s| !s.is_empty())
        .ok_or_else(|| format!("Missing or invalid {key}"))
}
fn tool(
    name: &str,
    description: &str,
    mut properties: Value,
    required: &[&str],
    readonly: bool,
) -> Value {
    if let Some(props) = properties.as_object_mut() {
        for (key, value) in props {
            if value.get("type").and_then(Value::as_str) == Some("array")
                && value.get("items").is_none()
            {
                value["items"] = match key.as_str() {
                    "fields" | "run_ids" => json!({"type":"string"}),
                    "point" | "start" | "end" | "seed" | "observed" | "expected" | "weights" => {
                        json!({"type":"number"})
                    }
                    _ => json!({}),
                };
            }
        }
    }
    json!({"name":name,"description":description,"inputSchema":{"type":"object","properties":properties,"required":required,"additionalProperties":false},"annotations":{"readOnlyHint":readonly,"destructiveHint":!readonly,"openWorldHint":false}})
}
fn definitions() -> Vec<Value> {
    let project = json!({"type":"string","description":"Project directory within server root"});
    vec![
        tool(
            "examples_list",
            "List built-in case templates",
            json!({}),
            &[],
            true,
        ),
        tool(
            "example_create",
            "Create a project from a built-in example in an empty directory",
            json!({"id":{"type":"string"},"directory":project,"preset":{"type":"string"},"name":{"type":"string"}}),
            &["id", "directory"],
            false,
        ),
        tool(
            "project_inspect",
            "Inspect a project and its readiness",
            json!({"directory":project}),
            &["directory"],
            true,
        ),
        tool(
            "project_check",
            "Review mesh and solver readiness",
            json!({"directory":project}),
            &["directory"],
            true,
        ),
        tool(
            "project_settings",
            "Read solver settings and boundary assignments",
            json!({"directory":project}),
            &["directory"],
            true,
        ),
        tool(
            "project_update_solver",
            "Merge solver settings into the project; omitted fields remain unchanged",
            json!({"directory":project,"settings":{"type":"object","description":"Partial solver settings using the same structure as project_settings"}}),
            &["directory", "settings"],
            false,
        ),
        tool(
            "project_set_boundary",
            "Set an existing named boundary condition using the structure returned by project_settings",
            json!({"directory":project,"selection":{"type":"string"},"condition":{}}),
            &["directory", "selection", "condition"],
            false,
        ),
        tool("project_geometry", "Inspect saved geometry, named selections and fluid domain", json!({"directory":project}), &["directory"], true),
        tool("project_set_plane", "Add or replace a 3D flow-plane monitor", json!({"directory":project,"plane":{"type":"object"}}), &["directory","plane"], false),
        tool("project_update_mesh", "Merge mesh settings and recipe, invalidate current mesh when changed", json!({"directory":project,"settings":{"type":"object"},"recipe":{"type":"object"}}), &["directory"], false),
        tool("compare_samples", "Compute weighted L1, L2 and maximum errors for paired reference data", json!({"observed":{"type":"array"},"expected":{"type":"array"},"weights":{"type":"array"}}), &["observed","expected"], true),
        tool("sweep_create", "Create independent case variants by replacing one existing solver setting", json!({"directory":project,"output":{"type":"string"},"solver_pointer":{"type":"string","description":"JSON pointer within solver, such as /initial_profile/velocity/x"},"values":{"type":"array","items":{}}}), &["directory","output","solver_pointer","values"], false),
        tool("runs_compare", "Compare saved run status, convergence, conservation and performance reports", json!({"directory":project,"run_ids":{"type":"array","items":{"type":"string"}}}), &["directory","run_ids"], true),
        tool(
            "project_mesh",
            "Start Gmsh mesh generation in the background",
            json!({"directory":project,"threads":{"type":"integer","minimum":1}}),
            &["directory"],
            false,
        ),
        tool(
            "project_import_mesh",
            "Import a Gmsh mesh with matching physical patches",
            json!({"directory":project,"mesh":{"type":"string"}}),
            &["directory", "mesh"],
            false,
        ),
        tool(
            "project_solve",
            "Start a solver job in the background; poll job_status",
            json!({"directory":project,"threads":{"type":"integer","minimum":1},"iterations":{"type":"integer","minimum":1},"backend":{"type":"string","enum":["serial","parallel","gpu"]},"resume_latest":{"type":"boolean"},"checkpoint_every":{"type":"integer","minimum":0}}),
            &["directory"],
            false,
        ),
        tool("job_pause", "Pause a running solve cooperatively", json!({"job_id":{"type":"integer","minimum":1}}), &["job_id"], false),
        tool("job_resume", "Resume a paused solve", json!({"job_id":{"type":"integer","minimum":1}}), &["job_id"], false),
        tool("job_checkpoint", "Request a recovery checkpoint for a running solve", json!({"job_id":{"type":"integer","minimum":1}}), &["job_id"], false),
        tool("job_stop", "Stop a running solve cooperatively and retain accepted fields", json!({"job_id":{"type":"integer","minimum":1}}), &["job_id"], false),
        tool(
            "job_status",
            "Check a background job and read its recent output",
            json!({"job_id":{"type":"integer","minimum":1}}),
            &["job_id"],
            true,
        ),
        tool(
            "job_cancel",
            "Terminate a background process; an unfinished iteration may not be saved",
            json!({"job_id":{"type":"integer","minimum":1}}),
            &["job_id"],
            false,
        ),
        tool("result_fields", "List available saved fields, ranges and volume means", json!({"directory":project,"run_id":{"type":"string"}}), &["directory","run_id"], true),
        tool("result_probe", "Sample fields in the containing 2D cell at a point in metres", json!({"directory":project,"run_id":{"type":"string"},"point":{"type":"array"},"fields":{"type":"array"}}), &["directory","run_id","point","fields"], true),
        tool("result_profile", "Sample fields along a line through a 2D mesh", json!({"directory":project,"run_id":{"type":"string"},"start":{"type":"array"},"end":{"type":"array"},"count":{"type":"integer","minimum":2},"fields":{"type":"array"}}), &["directory","run_id","start","end","count","fields"], true),
        tool("result_streamline", "Trace a 2D streamline in a saved field", json!({"directory":project,"run_id":{"type":"string"},"seed":{"type":"array"},"step_size":{"type":"number"},"max_length":{"type":"number"},"max_steps":{"type":"integer","minimum":1},"direction":{"type":"string","enum":["forward","backward"]}}), &["directory","run_id","seed","step_size","max_length"], true),
        tool("result_section_svg", "Export contours, vectors and streamlines on a 2D domain or 3D plane; output must not exist", json!({"directory":project,"run_id":{"type":"string"},"plane":{"type":"object"},"display":{"type":"object"},"output":{"type":"string"}}), &["directory","run_id","plane","output"], false),
        tool("convergence_svg", "Export saved convergence history to a new SVG", json!({"directory":project,"run_id":{"type":"string"},"output":{"type":"string"}}), &["directory","run_id","output"], false),
        tool("mesh_review", "Inspect mesh quality and preflight diagnostics", json!({"directory":project}), &["directory"], true),
        tool(
            "runs_list",
            "List saved runs in a project",
            json!({"directory":project}),
            &["directory"],
            true,
        ),
        tool(
            "run_report",
            "Read the saved report for a run",
            json!({"directory":project,"run_id":{"type":"string"}}),
            &["directory", "run_id"],
            true,
        ),
    ]
}
fn validate_arguments(name: &str, arguments: &Value) -> Result<(), String> {
    let definition = definitions()
        .into_iter()
        .find(|tool| tool["name"] == name)
        .ok_or_else(|| format!("Unknown tool: {name}"))?;
    let schema = &definition["inputSchema"];
    let args = arguments.as_object().ok_or("Arguments must be an object")?;
    for key in schema["required"].as_array().into_iter().flatten() {
        let key = key.as_str().unwrap_or_default();
        if !args.contains_key(key) {
            return Err(format!("Missing argument: {key}"));
        }
    }
    let properties = schema["properties"]
        .as_object()
        .ok_or("Invalid tool schema")?;
    for (key, value) in args {
        let property = properties
            .get(key)
            .ok_or_else(|| format!("Unknown argument: {key}"))?;
        let valid = match property.get("type").and_then(Value::as_str) {
            Some("string") => value.is_string(),
            Some("integer") => value
                .as_u64()
                .is_some_and(|n| n >= property.get("minimum").and_then(Value::as_u64).unwrap_or(0)),
            Some("boolean") => value.is_boolean(),
            Some("object") => value.is_object(),
            Some("array") => value.is_array(),
            Some("number") => value.as_f64().is_some_and(f64::is_finite),
            _ => true,
        };
        if !valid
            || property
                .get("enum")
                .and_then(Value::as_array)
                .is_some_and(|allowed| !allowed.contains(value))
        {
            return Err(format!("Invalid argument: {key}"));
        }
    }
    Ok(())
}
fn merge(target: &mut Value, patch: &Value) -> Result<(), String> {
    let obj = patch.as_object().ok_or("Settings must be an object")?;
    let dst = target
        .as_object_mut()
        .ok_or("Cannot merge an object into a non-object setting")?;
    for (key, value) in obj {
        let old = dst
            .get_mut(key)
            .ok_or_else(|| format!("Unknown setting: {key}"))?;
        if value.is_object() && old.is_object() {
            merge(old, value)?;
        } else {
            *old = value.clone();
        }
    }
    Ok(())
}
impl Server {
    fn path(&self, value: &str, create: bool) -> Result<PathBuf, String> {
        let raw = Path::new(value);
        if create
            && raw
                .components()
                .any(|c| matches!(c, std::path::Component::ParentDir))
        {
            return Err("Parent path components are not allowed for new paths".into());
        }
        let p = if raw.is_absolute() {
            raw.to_path_buf()
        } else {
            self.root.join(raw)
        };
        let resolved = if p.exists() {
            p.canonicalize()
        } else if create {
            let mut ancestor = p.as_path();
            let mut parts = Vec::new();
            while !ancestor.exists() {
                let name = ancestor.file_name().ok_or("Invalid path")?;
                parts.push(name.to_os_string());
                ancestor = ancestor.parent().ok_or("Invalid path")?;
            }
            let mut x = ancestor.canonicalize().map_err(|e| e.to_string())?;
            for part in parts.iter().rev() {
                x.push(part);
            }
            Ok(x)
        } else {
            p.canonicalize()
        }
        .map_err(|e| e.to_string())?;
        if !resolved.starts_with(&self.root) {
            return Err("Path is outside the MCP root".into());
        }
        Ok(resolved)
    }
    fn project(&self, a: &Value) -> Result<PathBuf, String> {
        self.path(arg(a, "directory")?, false)
    }
    fn idle(&mut self, p: &Path) -> Result<(), String> {
        for job in self.solves.values() {
            let state = job.state.lock().map_err(|_| "Job state poisoned")?;
            if state["status"] == "running"
                || state["status"] == "pause_requested"
                || state["status"] == "stopping"
            {
                if job.project == p {
                    return Err("A job is already running for this project".into());
                }
            }
        }
        for job in self.jobs.values_mut() {
            if job.status.is_none() {
                job.status = job.child.try_wait().map_err(|e| e.to_string())?;
            }
            if job.status.is_none() && job.project == p {
                return Err("A job is already running for this project".into());
            }
        }
        Ok(())
    }
    fn cli(&self, args: Vec<String>) -> Result<Value, String> {
        let output = Command::new(std::env::current_exe().map_err(|e| e.to_string())?)
            .args(args)
            .arg("--json")
            .output()
            .map_err(|e| e.to_string())?;
        if !output.status.success() {
            return Err(String::from_utf8_lossy(&output.stderr).trim().to_owned());
        }
        serde_json::from_slice(&output.stdout).map_err(|e| format!("CLI output: {e}"))
    }
    fn start(&mut self, p: PathBuf, args: Vec<String>) -> Result<Value, String> {
        self.idle(&p)?;
        self.next += 1;
        let id = self.next;
        let stdout = self.temp.path().join(format!("{id}.out"));
        let stderr = self.temp.path().join(format!("{id}.err"));
        let child = Command::new(std::env::current_exe().map_err(|e| e.to_string())?)
            .args(args)
            .arg("--json")
            .stdin(Stdio::null())
            .stdout(Stdio::from(
                fs::File::create(&stdout).map_err(|e| e.to_string())?,
            ))
            .stderr(Stdio::from(
                fs::File::create(&stderr).map_err(|e| e.to_string())?,
            ))
            .spawn()
            .map_err(|e| e.to_string())?;
        self.jobs.insert(
            id,
            Job {
                child,
                project: p,
                stdout,
                stderr,
                status: None,
            },
        );
        Ok(json!({"job_id":id,"status":"running"}))
    }
    fn start_solve(&mut self, project: PathBuf, args: Value) -> Result<Value, String> {
        self.idle(&project)?;
        let control = flursys::headless::ExecutionControl::default();
        let state = std::sync::Arc::new(std::sync::Mutex::new(json!({"status":"running"})));
        self.next += 1;
        let id = self.next;
        let ctrl = control.clone();
        let status = state.clone();
        let folder = project.clone();
        let thread = std::thread::spawn(move || {
            let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(
                || -> Result<Value, String> {
                    let (mut project, mut session) = flursys::headless::load(&folder)?;
                    if let Some(n) = args.get("threads").and_then(Value::as_u64) {
                        if n == 0
                            || n > std::thread::available_parallelism().map_or(1, |v| v.get())
                                as u64
                        {
                            return Err("Invalid worker count".into());
                        }
                        session.set_cpu_threads(n as usize);
                    }
                    if let Some(n) = args.get("iterations").and_then(Value::as_u64) {
                        if n == 0 || n > 1_000_000 {
                            return Err("iterations must be in 1..1000000".into());
                        }
                        if let Some(mut settings) = project.solver.compressible.clone() {
                            settings.max_steps = n as usize;
                            session
                                .set_compressible_settings(Some(settings))
                                .map_err(|e| e.to_string())?;
                        } else {
                            let s = session.solver_options().clone();
                            session
                                .set_solver_controls(
                                    n as usize,
                                    s.velocity_relaxation,
                                    s.pressure_relaxation,
                                    s.continuity_absolute_tolerance,
                                )
                                .map_err(|e| e.to_string())?;
                        }
                    }
                    if let Some(name) = args.get("backend").and_then(Value::as_str) {
                        let mut compute = session.solver_options().compute.clone();
                        compute.backend = match name {
                            "serial" => flursys::compute::Backend::Serial,
                            "parallel" => flursys::compute::Backend::Parallel,
                            "gpu" => flursys::compute::Backend::Gpu,
                            _ => return Err("Invalid backend".into()),
                        };
                        session.set_compute_options(compute)?;
                    }
                    let restart =
                        if args.get("resume_latest").and_then(Value::as_bool) == Some(true) {
                            let mesh = &session.mesh().ok_or("Project has no mesh")?.mesh;
                            if let Some(s) = project.solver.compressible.as_ref() {
                                Some(flursys::workbench::flow_frame::discover(
                                    &folder, mesh, s.gas,
                                )?)
                            } else {
                                Some(
                                    flursys::workbench::checkpoint::discover(&folder, mesh)?
                                        .info
                                        .path,
                                )
                            }
                        } else {
                            None
                        };
                    let options = flursys::headless::RunOptions {
                        restart,
                        checkpoint_every: args
                            .get("checkpoint_every")
                            .and_then(Value::as_u64)
                            .unwrap_or(100) as usize,
                        ..Default::default()
                    };
                    let outcome = flursys::headless::run_with_control(
                        &folder,
                        &mut project,
                        &mut session,
                        &options,
                        &ctrl,
                        |event| {
                            use flursys::app::{JobProgress as P, WorkbenchJobEvent as E};
                            if let Ok(mut state) = status.lock() {
                                match event {
                                    E::SolveProgress(P::Iteration {
                                        current,
                                        max,
                                        residual,
                                    }) => {
                                        state["progress"] = json!({"iteration":current,"target":max,"continuity":residual});
                                    }
                                    E::SolveProgress(P::Metrics { iteration, values }) => {
                                        state["metrics"] =
                                            json!({"iteration":iteration,"values":values});
                                    }
                                    E::SolveProgress(P::Message(message)) => {
                                        state["message"] = json!(message)
                                    }
                                    E::Checkpoint(Ok(info)) => {
                                        state["checkpoint"] =
                                            json!({"iteration":info.iteration,"path":info.path});
                                        state["checkpoint_requested"] = json!(false);
                                    }
                                    E::Checkpoint(Err(error)) => {
                                        state["checkpoint_error"] = json!(error)
                                    }
                                    E::Performance(report) => state["performance"] = json!(report),
                                    E::TimeStep {
                                        index,
                                        time,
                                        dt,
                                        courant,
                                        rejected,
                                    } => {
                                        state["physical_time"] = json!({"step":index,"time":time,"dt":dt,"courant":courant,"rejected":rejected})
                                    }
                                    _ => {}
                                }
                            }
                        },
                    )?;
                    let failed = outcome.failure.clone();
                    let value = serde_json::to_value(outcome).map_err(|e| e.to_string())?;
                    if let Some(error) = failed.filter(|_| !ctrl.is_stopped()) {
                        Err(error)
                    } else {
                        Ok(value)
                    }
                },
            ));
            if let Ok(mut state) = status.lock() {
                match run {
                    Ok(Ok(value)) => {
                        state["status"] = json!(if ctrl.is_stopped() {
                            "stopped"
                        } else {
                            "completed"
                        });
                        state["outcome"] = value;
                    }
                    Ok(Err(error)) => {
                        state["status"] = json!("failed");
                        state["error"] = json!(error);
                    }
                    Err(_) => {
                        state["status"] = json!("failed");
                        state["error"] = json!("Solver worker panicked");
                    }
                }
            }
        });
        self.solves.insert(
            id,
            SolveJob {
                project,
                control,
                state,
                thread: Some(thread),
            },
        );
        Ok(json!({"job_id":id,"status":"running"}))
    }
    fn call(&mut self, name: &str, a: &Value) -> Result<Value, String> {
        let pstr = |p: &Path| p.to_string_lossy().into_owned();
        match name {
            "examples_list" => self.cli(vec!["examples".into(), "list".into()]),
            "example_create" => {
                let p = self.path(arg(a, "directory")?, true)?;
                self.idle(&p)?;
                let mut args = vec![
                    "examples".into(),
                    "create".into(),
                    arg(a, "id")?.into(),
                    pstr(&p),
                ];
                for k in ["name", "preset"] {
                    if let Some(v) = a.get(k).and_then(Value::as_str) {
                        args.extend([format!("--{k}"), v.into()]);
                    }
                }
                self.cli(args)
            }
            "project_inspect" | "project_check" => {
                let p = self.project(a)?;
                let cmd = if name == "project_inspect" {
                    "inspect"
                } else {
                    "check"
                };
                self.cli(vec!["project".into(), cmd.into(), pstr(&p)])
            }
            "project_settings" => {
                let p = self.project(a)?;
                let q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                Ok(
                    json!({"solver":q.solver,"boundaries":q.boundaries,"material":q.material,"mesh":q.mesh,"mesh_recipe":q.mesh_recipe,"planes":q.plane_monitors}),
                )
            }
            "project_update_solver" => {
                let p = self.project(a)?;
                self.idle(&p)?;
                let mut q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                let mut v = serde_json::to_value(&q.solver).map_err(|e| e.to_string())?;
                merge(&mut v, a.get("settings").ok_or("Missing settings")?)?;
                q.solver = serde_json::from_value(v).map_err(|e| e.to_string())?;
                q.validate().map_err(|e| e.to_string())?;
                flursys::save_workspace(&p, &q).map_err(|e| e.to_string())?;
                Ok(json!({"updated":true,"solver":q.solver}))
            }
            "sweep_create" => {
                let source = self.project(a)?;
                self.idle(&source)?;
                let dest = self.path(arg(a, "output")?, true)?;
                if dest.exists() {
                    return Err("Sweep output must not exist".into());
                }
                let pointer = arg(a, "solver_pointer")?;
                if !pointer.starts_with('/') {
                    return Err(
                        "solver_pointer must be an absolute JSON pointer within solver".into(),
                    );
                }
                let vals = a["values"]
                    .as_array()
                    .filter(|v| !v.is_empty() && v.len() <= 32)
                    .ok_or("values must contain 1..32 entries")?;
                let (base, session) = flursys::headless::load(&source)?;
                let mut cases = Vec::new();
                for (i, value) in vals.iter().enumerate() {
                    let mut project = base.clone();
                    let mut solver =
                        serde_json::to_value(&project.solver).map_err(|e| e.to_string())?;
                    let setting = solver
                        .pointer_mut(pointer)
                        .ok_or("Unknown solver pointer")?;
                    if !(setting.is_number() && value.is_number()
                        || setting.is_string() && value.is_string()
                        || setting.is_boolean() && value.is_boolean())
                    {
                        return Err(
                            "Sweep values must match the existing scalar setting type".into()
                        );
                    }
                    *setting = value.clone();
                    project.solver = serde_json::from_value(solver).map_err(|e| e.to_string())?;
                    project.name = format!("{} · variant {}", base.name, i + 1);
                    project.project_id = flursys::WorkbenchProject::blank("variant").project_id;
                    project.runs.clear();
                    project.mesh_asset = None;
                    project.validate().map_err(|e| e.to_string())?;
                    cases.push(project);
                }
                std::fs::create_dir(&dest).map_err(|e| e.to_string())?;
                let mut output = Vec::new();
                for (i, mut project) in cases.into_iter().enumerate() {
                    let folder = dest.join(format!("case-{:03}", i + 1));
                    std::fs::create_dir(&folder).map_err(|e| e.to_string())?;
                    if let Some(mesh) = session.mesh() {
                        project.mesh_asset = Some(flursys::workbench::mesh_storage::save(
                            &folder, &project, mesh,
                        )?);
                    }
                    flursys::save_workspace(&folder, &project).map_err(|e| e.to_string())?;
                    output.push(json!({"directory":folder,"value":vals[i]}));
                }
                Ok(
                    json!({"cases":output,"note":"Cases are independent; start each with project_solve. No solves were started."}),
                )
            }
            "runs_compare" => {
                let p = self.project(a)?;
                let q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                let ids = a["run_ids"]
                    .as_array()
                    .filter(|v| !v.is_empty() && v.len() <= 32)
                    .ok_or("run_ids must contain 1..32 IDs")?;
                let mut reports = Vec::new();
                for id in ids {
                    let id = id.as_str().ok_or("Run IDs must be strings")?;
                    let run = q
                        .runs
                        .iter()
                        .find(|r| r.id == id)
                        .ok_or_else(|| format!("Unknown run: {id}"))?;
                    let report = flursys::headless::report(&p, run)?;
                    reports.push(json!({"run":run,"coupling_convergence":report.get("coupling_convergence"),"conservation":report.get("conservation"),"performance":report.get("performance"),"wall_reports":report.get("wall_reports").and_then(Value::as_array).map(|rows|rows.iter().map(|r|json!({"patch":r.get("patch"),"y_plus_min":r.get("y_plus_min"),"y_plus_max":r.get("y_plus_max")})).collect::<Vec<_>>())}));
                }
                Ok(
                    json!({"runs":reports,"note":"Check model, mesh, reference conditions and convergence before interpreting differences"}),
                )
            }
            "project_geometry" => {
                let p = self.project(a)?;
                let q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                Ok(
                    json!({"geometry":q.geometry,"named_selections":q.named_selections,"fluid_face":q.fluid_face,"dimension":q.mesh.dimension}),
                )
            }
            "project_set_plane" => {
                let p = self.project(a)?;
                self.idle(&p)?;
                let mut q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                if q.mesh.dimension != flursys::MeshDimension::ThreeD {
                    return Err("Flow-plane monitors require a 3D volume mesh".into());
                }
                let plane: flursys::workbench::plane_monitor::FlowPlaneMonitor =
                    serde_json::from_value(a["plane"].clone()).map_err(|e| e.to_string())?;
                plane.validate()?;
                if let Some(old) = q.plane_monitors.iter_mut().find(|p| p.id == plane.id) {
                    *old = plane.clone()
                } else {
                    q.plane_monitors.push(plane.clone())
                }
                q.validate().map_err(|e| e.to_string())?;
                flursys::save_workspace(&p, &q).map_err(|e| e.to_string())?;
                Ok(json!({"plane":plane}))
            }
            "project_update_mesh" => {
                let p = self.project(a)?;
                self.idle(&p)?;
                let mut q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                let original =
                    serde_json::to_value((&q.mesh, &q.mesh_recipe)).map_err(|e| e.to_string())?;
                if let Some(patch) = a.get("settings") {
                    let mut v = serde_json::to_value(&q.mesh).map_err(|e| e.to_string())?;
                    merge(&mut v, patch)?;
                    q.mesh = serde_json::from_value(v).map_err(|e| e.to_string())?;
                }
                if let Some(patch) = a.get("recipe") {
                    let mut v = serde_json::to_value(&q.mesh_recipe).map_err(|e| e.to_string())?;
                    merge(&mut v, patch)?;
                    q.mesh_recipe = serde_json::from_value(v).map_err(|e| e.to_string())?;
                }
                let changed = original
                    != serde_json::to_value((&q.mesh, &q.mesh_recipe))
                        .map_err(|e| e.to_string())?;
                if changed {
                    q.mesh_asset = None;
                    if let Some(context) = q.case_context.as_mut() {
                        context.reference_mesh_intent = None;
                    }
                }
                q.validate().map_err(|e| e.to_string())?;
                flursys::save_workspace(&p, &q).map_err(|e| e.to_string())?;
                Ok(
                    json!({"updated":changed,"mesh_invalidated":changed,"settings":q.mesh,"recipe":q.mesh_recipe}),
                )
            }
            "compare_samples" => {
                fn numbers(v: &Value) -> Result<Vec<f64>, String> {
                    v.as_array()
                        .ok_or("Expected an array")?
                        .iter()
                        .map(|v| {
                            v.as_f64()
                                .filter(|v| v.is_finite())
                                .ok_or_else(|| "Array must contain finite numbers".into())
                        })
                        .collect()
                }
                let observed = numbers(&a["observed"])?;
                let expected = numbers(&a["expected"])?;
                let weights = if let Some(v) = a.get("weights") {
                    numbers(v)?
                } else {
                    Vec::new()
                };
                let m = flursys::workbench::case_format::compare(&observed, &expected, &weights)?;
                Ok(
                    json!({"samples":m.samples,"mean_absolute_error":m.l1,"root_mean_square_error":m.l2,"maximum_absolute_error":m.linf,"relative_l2":m.relative_l2,"note":"Paired values must use consistent locations, units and physical definitions"}),
                )
            }
            "project_set_boundary" => {
                let p = self.project(a)?;
                self.idle(&p)?;
                let mut q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                let selection = arg(a, "selection")?;
                let boundary = q
                    .boundaries
                    .iter_mut()
                    .find(|b| b.selection == selection)
                    .ok_or("Unknown assigned boundary")?;
                boundary.condition =
                    serde_json::from_value(a.get("condition").cloned().ok_or("Missing condition")?)
                        .map_err(|e| e.to_string())?;
                q.validate().map_err(|e| e.to_string())?;
                flursys::save_workspace(&p, &q).map_err(|e| e.to_string())?;
                Ok(json!({"updated": true, "selection": selection}))
            }
            "project_import_mesh" => {
                let p = self.project(a)?;
                self.idle(&p)?;
                let mesh = self.path(arg(a, "mesh")?, false)?;
                self.cli(vec![
                    "project".into(),
                    "import-mesh".into(),
                    pstr(&p),
                    "--input".into(),
                    pstr(&mesh),
                ])
            }
            "project_solve" => {
                let p = self.project(a)?;
                self.start_solve(p, a.clone())
            }
            "project_mesh" => {
                let p = self.project(a)?;
                let cmd = if name == "project_mesh" {
                    "mesh"
                } else {
                    "solve"
                };
                let mut args = vec!["project".into(), cmd.into(), pstr(&p)];
                for (key, opt) in [
                    ("threads", "threads"),
                    ("iterations", "iterations"),
                    ("checkpoint_every", "checkpoint-every"),
                ] {
                    if let Some(v) = a.get(key) {
                        let n = v
                            .as_u64()
                            .ok_or_else(|| format!("{key} must be a nonnegative integer"))?;
                        if n == 0 && key != "checkpoint_every" {
                            return Err(format!("{key} must be positive"));
                        }
                        args.extend([format!("--{opt}"), n.to_string()]);
                    }
                }
                if let Some(v) = a.get("backend").and_then(Value::as_str) {
                    if !["serial", "parallel", "gpu"].contains(&v) {
                        return Err("Invalid backend".into());
                    }
                    args.extend(["--backend".into(), v.into()]);
                }
                if a.get("resume_latest").and_then(Value::as_bool) == Some(true) {
                    args.push("--resume-latest".into());
                }
                self.start(p, args)
            }
            "job_status" | "job_cancel" | "job_pause" | "job_resume" | "job_checkpoint"
            | "job_stop" => {
                let id = a
                    .get("job_id")
                    .and_then(Value::as_u64)
                    .ok_or("Invalid job_id")?;
                if let Some(j) = self.solves.get_mut(&id) {
                    let mut state = j.state.lock().map_err(|_| "Job state poisoned")?;
                    let active =
                        state["status"] == "running" || state["status"] == "pause_requested";
                    if name != "job_status" && !active {
                        return Err("Solve is not active".into());
                    }
                    match name {
                        "job_pause" => {
                            j.control.pause();
                            state["status"] = json!("pause_requested");
                        }
                        "job_resume" => {
                            j.control.resume();
                            state["status"] = json!("running");
                        }
                        "job_checkpoint" => {
                            j.control.checkpoint();
                            state["checkpoint_requested"] = json!(true);
                        }
                        "job_stop" | "job_cancel" => {
                            j.control.stop();
                            state["status"] = json!("stopping");
                        }
                        _ => {}
                    }
                    return Ok(state.clone());
                }
                if !matches!(name, "job_status" | "job_cancel") {
                    return Err("Control is only available for solve jobs".into());
                }
                let j = self.jobs.get_mut(&id).ok_or("Unknown job")?;
                if j.status.is_none() {
                    j.status = j.child.try_wait().map_err(|e| e.to_string())?;
                }
                if name == "job_cancel" && j.status.is_none() {
                    j.child.kill().map_err(|e| e.to_string())?;
                    j.status = Some(j.child.wait().map_err(|e| e.to_string())?);
                }
                fn tail(p: &Path) -> String {
                    let Ok(mut f) = fs::File::open(p) else {
                        return String::new();
                    };
                    let len = f.metadata().map(|m| m.len()).unwrap_or(0);
                    use std::io::Seek;
                    let _ = f.seek(std::io::SeekFrom::Start(len.saturating_sub(MAX_OUTPUT)));
                    let mut v = Vec::new();
                    let _ = f.take(MAX_OUTPUT).read_to_end(&mut v);
                    String::from_utf8_lossy(&v).into_owned()
                }
                Ok(
                    json!({"job_id":id,"status":if j.status.is_none(){"running"}else if j.status.as_ref().is_some_and(|s|s.success()){"completed"}else{"failed"},"exit_code":j.status.as_ref().and_then(|s|s.code()),"stdout":tail(&j.stdout),"stderr":tail(&j.stderr)}),
                )
            }
            "result_fields" | "result_probe" | "result_profile" | "result_streamline"
            | "result_section_svg" => {
                let p = self.project(a)?;
                let output = if name == "result_section_svg" {
                    Some(self.path(arg(a, "output")?, true)?)
                } else {
                    None
                };
                super::mcp_results::run(name, a, &p, output.as_deref())
            }
            "convergence_svg" => {
                let p = self.project(a)?;
                let output = self.path(arg(a, "output")?, true)?;
                super::mcp_results::history_plot(&p, arg(a, "run_id")?, &output)
            }
            "mesh_review" => {
                let p = self.project(a)?;
                let (_, session) = flursys::headless::load(&p)?;
                let mesh = &session.mesh().ok_or("Project has no mesh")?.mesh;
                let review = flursys::workbench::diagnostics::mesh_preflight(mesh);
                Ok(
                    json!({"cells":mesh.cell_count(),"points":mesh.points().len(),"dimension":format!("{:?}",mesh.dimension()),"preflight":review}),
                )
            }
            "runs_list" | "run_report" => {
                let p = self.project(a)?;
                let q = flursys::load_workspace(&p).map_err(|e| e.to_string())?;
                if name == "runs_list" {
                    Ok(json!({"runs":q.runs}))
                } else {
                    let id = arg(a, "run_id")?;
                    let run = q.runs.iter().find(|r| r.id == id).ok_or("Unknown run")?;
                    flursys::headless::report(&p, run)
                }
            }
            _ => Err(format!("Unknown tool: {name}")),
        }
    }
    fn handle(&mut self, msg: Value) -> Option<Value> {
        let Some(o) = msg.as_object() else {
            return Some(err(Value::Null, -32600, "Invalid request"));
        };
        let id = o.get("id").cloned();
        let method = o.get("method").and_then(Value::as_str);
        if id.is_none() {
            if method == Some("notifications/initialized") {
                self.initialized = true;
            }
            return None;
        }
        let id = id.unwrap();
        if o.get("jsonrpc").and_then(Value::as_str) != Some("2.0") || method.is_none() {
            return Some(err(id, -32600, "Invalid request"));
        }
        let params = o.get("params").cloned().unwrap_or(json!({}));
        let result: Result<Value, (i64, String)> = match method.unwrap() {
            "initialize" => {
                let requested = params
                    .get("protocolVersion")
                    .and_then(Value::as_str)
                    .unwrap_or(VERSION);
                let version = if [VERSION, "2025-03-26", "2024-11-05"].contains(&requested) {
                    requested
                } else {
                    VERSION
                };
                Ok(
                    json!({"protocolVersion":version,"capabilities":{"tools":{"listChanged":false},"resources":{"listChanged":false}},"serverInfo":{"name":"flursys","version":env!("CARGO_PKG_VERSION")},"instructions":"Use project_settings to discover settings, project_check before solve, and job_status for asynchronous operations. Paths are confined to the configured root."}),
                )
            }
            "ping" => Ok(json!({})),
            "resources/list" if self.initialized => Ok(
                json!({"resources":[{"uri":"flursys://guide","name":"FLURSYS MCP workflow","mimeType":"text/plain","description":"Tool usage and numerical interpretation"},{"uri":"flursys://fields","name":"Result field reference","mimeType":"application/json","description":"Field identifiers and their physical interpretation"}]}),
            ),
            "resources/read" if self.initialized => {
                let uri = params
                    .get("uri")
                    .and_then(Value::as_str)
                    .ok_or((-32602, "Missing resource uri".to_string()));
                uri.and_then(|uri| {let text=match uri {
                    "flursys://guide"=>"Use examples_list/example_create or project_inspect; project_settings exposes canonical structures. Check mesh_review and project_check before project_solve; poll job_status, use job_pause/job_resume/job_checkpoint/job_stop, then inspect runs_list and run_report. Use result_fields before sampling or rendering. Run status max-iterations is not evidence of convergence. Sweep cases are independent and need separate solves. A 2D streamline is not a time-resolved particle path.",
                    "flursys://fields"=>"{\"pressure\":\"kinematic gauge pressure for incompressible flow\",\"absolute_pressure\":\"Pa, compressible results\",\"velocity_x\":\"m/s\",\"velocity_y\":\"m/s\",\"velocity_z\":\"m/s\",\"speed\":\"m/s\",\"temperature\":\"K\",\"density\":\"kg/m3\",\"vorticity\":\"reconstructed magnitude, 1/s\",\"divergence\":\"reconstructed, 1/s\"}",
                    _=>return Err((-32002,"Resource not found".into()))};Ok(json!({"contents":[{"uri":uri,"mimeType":if uri.ends_with("fields"){"application/json"}else{"text/plain"},"text":text}]}))})
            }

            "tools/list" if self.initialized => Ok(json!({"tools":definitions()})),
            "tools/call" if self.initialized => {
                let name = params.get("name").and_then(Value::as_str);
                match name {
                    Some(n) => {
                        let args = params.get("arguments").cloned().unwrap_or(json!({}));
                        if !args.is_object() {
                            Err((-32602, "arguments must be an object".into()))
                        } else {
                            Ok(
                                match validate_arguments(n, &args).and_then(|_| self.call(n, &args))
                                {
                                    Ok(v) => text_result(v, false),
                                    Err(e) => text_result(json!({"error":e}), true),
                                },
                            )
                        }
                    }
                    None => Err((-32602, "Missing tool name".into())),
                }
            }
            "tools/list" | "tools/call" => Err((-32600, "Server is not initialized".into())),
            _ => Err((-32601, "Method not found".into())),
        };
        Some(match result {
            Ok(v) => json!({"jsonrpc":"2.0","id":id,"result":v}),
            Err((code, message)) => err(id, code, &message),
        })
    }
}
impl Drop for Server {
    fn drop(&mut self) {
        for j in self.solves.values_mut() {
            j.control.stop();
            if let Some(thread) = j.thread.take() {
                let _ = thread.join();
            }
        }
        for j in self.jobs.values_mut() {
            if j.status.is_none() {
                let _ = j.child.kill();
                let _ = j.child.wait();
            }
        }
    }
}
pub fn run(args: &[String]) -> Result<(), String> {
    let root = match args {
        [] => std::env::current_dir().map_err(|e| e.to_string())?,
        [a, b] if a == "--root" => PathBuf::from(b),
        _ => return Err("Usage: flursys mcp [--root DIRECTORY]".into()),
    };
    let root = root.canonicalize().map_err(|e| e.to_string())?;
    if !root.is_dir() {
        return Err("MCP root must be a directory".into());
    }
    let mut server = Server {
        root,
        jobs: BTreeMap::new(),
        solves: BTreeMap::new(),
        next: 0,
        temp: tempfile::tempdir().map_err(|e| e.to_string())?,
        initialized: false,
    };
    let stdin = io::stdin();
    let mut input = stdin.lock();
    let stdout = io::stdout();
    let mut output = stdout.lock();
    loop {
        let mut line = Vec::new();
        let mut overflow = false;
        let mut received = false;
        loop {
            let buf = input.fill_buf().map_err(|e| e.to_string())?;
            if buf.is_empty() {
                break;
            }
            let n = buf
                .iter()
                .position(|b| *b == b'\n')
                .map_or(buf.len(), |i| i + 1);
            let newline = buf[n - 1] == b'\n';
            let len = n - usize::from(newline);
            if !overflow {
                if line.len() + len > MAX_LINE {
                    overflow = true;
                    line.clear()
                } else {
                    line.extend_from_slice(&buf[..len]);
                }
            }
            input.consume(n);
            received = true;
            if newline {
                break;
            }
        }
        if !received {
            break;
        }
        let response = if overflow {
            Some(err(Value::Null, -32600, "Request too large"))
        } else {
            match serde_json::from_slice::<Value>(&line) {
                Ok(v) => server.handle(v),
                Err(_) => Some(err(Value::Null, -32700, "Parse error")),
            }
        };
        if let Some(v) = response {
            serde_json::to_writer(&mut output, &v).map_err(|e| e.to_string())?;
            writeln!(output)
                .and_then(|_| output.flush())
                .map_err(|e| e.to_string())?;
        }
    }
    Ok(())
}
