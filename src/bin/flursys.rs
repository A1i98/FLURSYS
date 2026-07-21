use flursys::cases::{BackwardStepCase, CavityCase, CylinderCase};
use flursys::{
    Case, ConvectionScheme, IncompressibleSolver, PressureSolverKind, PressureVelocityCoupling,
    SimulationConfig,
};
use std::collections::HashMap;
use std::env;
use std::path::PathBuf;

fn main() {
    if let Err(error) = run_cli() {
        eprintln!("Error: {error}");
        std::process::exit(1);
    }
}

fn run_cli() -> Result<(), String> {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() || args[0] == "--help" || args[0] == "-h" {
        print_help();
        return Ok(());
    }
    if args[0] == "list" {
        println!("Available cases:");
        println!("  cavity         lid-driven cavity");
        println!("  cylinder       cylinder flow at Re=100");
        println!("  backward-step  backward-facing step flow");
        return Ok(());
    }

    let case_slug = args[0].as_str();
    let options = parse_options(&args[1..])?;
    if options.contains_key("help") {
        print_case_help(case_slug);
        return Ok(());
    }

    let config = match case_slug {
        "cavity" => cavity_config(&options)?,
        "cylinder" => cylinder_config(&options)?,
        "backward-step" | "bfs" => backward_step_config(&options)?,
        other => return Err(format!("Unknown case '{other}'. Run `flursys list`.")),
    };

    let mut solver = IncompressibleSolver::new(config)?;
    let summary = solver.run()?;
    println!();
    println!("Completed: {}", summary.case_name);
    println!("steps: {}", summary.steps);
    println!("final time: {:.6}", summary.final_time);
    println!("max divergence: {:.6e}", summary.max_divergence);
    println!("pressure residual: {:.6e}", summary.pressure_residual);
    println!("elapsed: {:.3} s", summary.elapsed.as_secs_f64());
    println!("steady convergence detected: {}", summary.converged);
    Ok(())
}

fn cavity_config(options: &HashMap<String, String>) -> Result<SimulationConfig, String> {
    let mut case = CavityCase::default();
    case.length = value(options, "length", case.length)?;
    case.height = value(options, "height", case.height)?;
    case.rho = value(options, "rho", case.rho)?;
    case.lid_velocity = value(options, "u-lid", case.lid_velocity)?;
    case.reynolds = value(options, "re", case.reynolds)?;
    case.nu = case.lid_velocity * case.length / case.reynolds;

    common_config(
        Case::LidDrivenCavity(case),
        options,
        Defaults {
            nx: 64,
            ny: 64,
            dt: 0.001,
            t_end: 100.0,
            max_steps: 100_000,
            output_dir: "results/cavity",
        },
    )
}

fn cylinder_config(options: &HashMap<String, String>) -> Result<SimulationConfig, String> {
    let mut case = CylinderCase::default();
    case.length = value(options, "length", case.length)?;
    case.height = value(options, "height", case.height)?;
    case.diameter = value(options, "diameter", case.diameter)?;
    case.xc = value(options, "xc", case.xc)?;
    case.yc = value(options, "yc", case.yc)?;
    case.rho = value(options, "rho", case.rho)?;
    case.u_inf = value(options, "u-inf", case.u_inf)?;
    case.reynolds = value(options, "re", case.reynolds)?;
    case.perturbation = value(options, "perturb", case.perturbation)?;
    case.nu = case.u_inf * case.diameter / case.reynolds;
    case.mu = case.rho * case.nu;

    common_config(
        Case::CylinderRe100(case),
        options,
        Defaults {
            nx: 250,
            ny: 100,
            dt: 0.002,
            t_end: 100.0,
            max_steps: 50_000,
            output_dir: "results/cylinder-re100",
        },
    )
}

fn backward_step_config(options: &HashMap<String, String>) -> Result<SimulationConfig, String> {
    let mut case = BackwardStepCase::default();
    case.length = value(options, "length", case.length)?;
    case.height = value(options, "height", case.height)?;
    case.step_height = value(options, "step-height", case.step_height)?;
    case.step_x = value(options, "step-x", case.step_x)?;
    case.rho = value(options, "rho", case.rho)?;
    case.u_mean = value(options, "u-mean", case.u_mean)?;
    case.reynolds = value(options, "re", case.reynolds)?;
    case.nu = case.u_mean * case.step_height / case.reynolds;

    common_config(
        Case::BackwardFacingStep(case),
        options,
        Defaults {
            nx: 700,
            ny: 40,
            dt: 0.0025,
            t_end: 100.0,
            max_steps: 40_000,
            output_dir: "results/backward-facing-step",
        },
    )
}

#[derive(Clone, Copy)]
struct Defaults {
    nx: usize,
    ny: usize,
    dt: f64,
    t_end: f64,
    max_steps: usize,
    output_dir: &'static str,
}

fn common_config(
    case: Case,
    options: &HashMap<String, String>,
    defaults: Defaults,
) -> Result<SimulationConfig, String> {
    let convection = match options
        .get("convection")
        .map(String::as_str)
        .unwrap_or("upwind")
    {
        "upwind" | "first-order-upwind" => ConvectionScheme::FirstOrderUpwind,
        "central" => ConvectionScheme::Central,
        other => return Err(format!("Unknown convection scheme '{other}'")),
    };
    let pressure_solver = match options
        .get("pressure-solver")
        .map(String::as_str)
        .unwrap_or("pcg")
    {
        "pcg" => PressureSolverKind::Pcg,
        "sor" => PressureSolverKind::Sor,
        other => return Err(format!("Unknown pressure solver '{other}'")),
    };
    let coupling = match options
        .get("coupling")
        .map(String::as_str)
        .unwrap_or("projection")
    {
        "projection" | "transient" => PressureVelocityCoupling::Projection,
        "simple" | "steady-simple" => PressureVelocityCoupling::Simple,
        other => return Err(format!("Unknown pressure-velocity coupling '{other}'")),
    };

    Ok(SimulationConfig {
        case,
        nx: value(options, "nx", defaults.nx)?,
        ny: value(options, "ny", defaults.ny)?,
        dt: value(options, "dt", defaults.dt)?,
        t_end: value(options, "t-end", defaults.t_end)?,
        max_steps: value(options, "max-steps", defaults.max_steps)?,
        convection,
        coupling,
        pressure_solver,
        pressure_max_iters: value(options, "pressure-iters", 1200_usize)?,
        pressure_tolerance: value(options, "pressure-tol", 1.0e-5_f64)?,
        pressure_omega: value(options, "pressure-omega", 1.7_f64)?,
        velocity_relaxation: value(options, "velocity-relax", 0.7_f64)?,
        pressure_relaxation: value(options, "pressure-relax", 0.3_f64)?,
        print_every: value(options, "print-every", 100_usize)?,
        output_every: value(options, "output-every", 100_usize)?,
        frame_every: value(options, "frame-every", 500_usize)?,
        steady_tolerance: value(options, "steady-tol", 1.0e-7_f64)?,
        minimum_steps: value(options, "minimum-steps", 2_000_usize)?,
        threads: value(options, "threads", 0_usize)?,
        output_dir: PathBuf::from(
            options
                .get("out")
                .cloned()
                .unwrap_or_else(|| defaults.output_dir.to_string()),
        ),
    })
}

fn parse_options(args: &[String]) -> Result<HashMap<String, String>, String> {
    let mut options = HashMap::new();
    let mut i = 0;
    while i < args.len() {
        let token = &args[i];
        if !token.starts_with("--") {
            return Err(format!("Unexpected positional argument '{token}'"));
        }
        let key = token.trim_start_matches("--");
        if key == "help" {
            options.insert("help".to_string(), "true".to_string());
            i += 1;
            continue;
        }
        if i + 1 >= args.len() || args[i + 1].starts_with("--") {
            return Err(format!("Option '--{key}' requires a value"));
        }
        options.insert(key.to_string(), args[i + 1].clone());
        i += 2;
    }
    Ok(options)
}

fn value<T>(options: &HashMap<String, String>, key: &str, default: T) -> Result<T, String>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    match options.get(key) {
        Some(raw) => raw
            .parse::<T>()
            .map_err(|e| format!("Invalid value for --{key}: {e}")),
        None => Ok(default),
    }
}

fn print_help() {
    println!("FLURSYS — Rust-native scientific simulation system");
    println!();
    println!("USAGE:");
    println!("  flursys list");
    println!("  flursys cavity [options]");
    println!("  flursys cylinder [options]");
    println!("  flursys backward-step [options]");
    println!();
    println!("Run `flursys <case> --help` for available options.");
}

fn print_case_help(case: &str) {
    println!("Case: {case}");
    println!();
    println!("Common options:");
    println!("  --nx N --ny N");
    println!("  --dt DT --t-end T --max-steps N");
    println!("  --convection upwind|central");
    println!("  --coupling projection|simple");
    println!("  --pressure-solver pcg|sor");
    println!("  --pressure-iters N --pressure-tol EPS --pressure-omega W");
    println!("  --velocity-relax A --pressure-relax A (SIMPLE only)");
    println!("  --print-every N --output-every N --frame-every N");
    println!("  --steady-tol EPS --minimum-steps N");
    println!("  --threads N (0 = automatic CPU worker count)");
    println!("  --out PATH");
    println!();
    match case {
        "cavity" => {
            println!("Physical options: --length L --height H --rho RHO --u-lid U --re RE");
        }
        "cylinder" => {
            println!("Physical options: --length L --height H --diameter D --xc X --yc Y");
            println!("                  --rho RHO --u-inf U --re RE --perturb EPS");
        }
        "backward-step" | "bfs" => {
            println!("Physical options: --length L --height H --step-height Hs --step-x Xs");
            println!("                  --rho RHO --u-mean U --re RE");
        }
        _ => println!("Unknown case."),
    }
}
