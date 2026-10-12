use flursys::cases::{BackwardStepCase, CavityCase, ChannelCase, CylinderCase, Side};
use flursys::{
    Case, ConvectionScheme, IncompressibleSolver, LidDrivenCavity3DConfig, LidDrivenCavity3DSolver,
    PimpleControls, PressureSolverKind, PressureVelocityCoupling, SimulationConfig,
    SolverBoundaryOverrides, TimeStepSettings,
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
        println!("  turek          Turek-Schäfer / DFG 2D-2 cylinder benchmark at Re=100");
        println!("  backward-step  backward-facing step flow");
        println!("  channel        plane Poiseuille channel flow");
        println!("  cavity-3d      three-dimensional lid-driven cavity");
        return Ok(());
    }

    let case_slug = args[0].as_str();
    let options = parse_options(&args[1..])?;
    if options.contains_key("help") {
        print_case_help(case_slug);
        return Ok(());
    }
    if case_slug == "cavity-3d" {
        return run_3d_solver(cavity_3d_config(&options)?);
    }

    let config = match case_slug {
        "cavity" => cavity_config(&options)?,
        "cylinder" => cylinder_config(&options)?,
        "turek" | "turek-schaefer" => turek_config(&options)?,
        "backward-step" | "bfs" => backward_step_config(&options)?,
        "channel" => channel_config(&options)?,
        other => return Err(format!("Unknown case '{other}'. Run `flursys list`.")),
    };

    run_solver(config)
}

fn run_solver(config: SimulationConfig) -> Result<(), String> {
    let mut solver = IncompressibleSolver::new(config)?;
    let summary = solver.run()?;
    println!();
    println!("Completed: {}", summary.case_name);
    println!("steps: {}", summary.steps);
    println!("final time: {:.6}", summary.final_time);
    println!("max divergence: {:.6e}", summary.max_divergence);
    println!("pressure residual: {:.6e}", summary.pressure_residual);
    if let Some(residual) = summary.momentum_equation_residual {
        println!("normalized momentum equation residual: {:.6e}", residual);
    }
    println!("elapsed: {:.3} s", summary.elapsed.as_secs_f64());
    println!("steady convergence detected: {}", summary.converged);
    Ok(())
}

fn run_3d_solver(config: LidDrivenCavity3DConfig) -> Result<(), String> {
    let summary = LidDrivenCavity3DSolver::new(config)?.run()?;
    println!();
    println!("Completed: 3D lid-driven cavity");
    println!("steps: {}", summary.steps);
    println!("final time: {:.6}", summary.final_time);
    println!("max divergence: {:.6e}", summary.max_divergence);
    println!("pressure residual: {:.6e}", summary.pressure_residual);
    println!("max speed: {:.6e}", summary.max_speed);
    println!("elapsed: {:.3} s", summary.elapsed.as_secs_f64());
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

fn turek_config(options: &HashMap<String, String>) -> Result<SimulationConfig, String> {
    // Canonical DFG 2D-2 geometry and fluid properties.
    let mut case = CylinderCase {
        length: 2.2,
        height: 0.41,
        diameter: 0.1,
        xc: 0.2,
        yc: 0.2,
        rho: 1.0,
        u_inf: 1.0,
        reynolds: 100.0,
        mu: 0.001,
        nu: 0.001,
        perturbation: 1.0e-3,
    };
    case.length = value(options, "length", case.length)?;
    case.height = value(options, "height", case.height)?;
    case.diameter = value(options, "diameter", case.diameter)?;
    case.xc = value(options, "xc", case.xc)?;
    case.yc = value(options, "yc", case.yc)?;
    case.rho = value(options, "rho", case.rho)?;
    case.u_inf = value(options, "u-mean", case.u_inf)?;
    case.reynolds = value(options, "re", case.reynolds)?;
    case.nu = case.u_inf * case.diameter / case.reynolds;
    case.mu = case.rho * case.nu;
    case.perturbation = value(options, "perturb", case.perturbation)?;
    let mut solver_options = options.clone();
    solver_options
        .entry("coupling".to_string())
        .or_insert_with(|| "pimple".to_string());
    solver_options
        .entry("pressure-solver".to_string())
        .or_insert_with(|| "pcg-mg".to_string());
    solver_options
        .entry("pressure-tol".to_string())
        .or_insert_with(|| "1e-6".to_string());
    solver_options
        .entry("velocity-relax".to_string())
        .or_insert_with(|| "1.0".to_string());
    solver_options
        .entry("pressure-relax".to_string())
        .or_insert_with(|| "1.0".to_string());
    let mut config = common_config(
        Case::CylinderRe100(case.clone()),
        &solver_options,
        Defaults {
            nx: 440,
            ny: 82,
            dt: 0.0025,
            t_end: 30.0,
            max_steps: 20_000,
            output_dir: "results/turek-schaefer-re100",
        },
    )?;
    config
        .boundary_overrides
        .set_parabolic_inlet(Side::Left, 1.5 * case.u_inf, 0.0, case.height);
    config
        .boundary_overrides
        .set_velocity(Side::Bottom, 0.0, 0.0);
    config.boundary_overrides.set_velocity(Side::Top, 0.0, 0.0);
    Ok(config)
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

fn channel_config(options: &HashMap<String, String>) -> Result<SimulationConfig, String> {
    let mut case = ChannelCase::default();
    case.length = value(options, "length", case.length)?;
    case.height = value(options, "height", case.height)?;
    case.rho = value(options, "rho", case.rho)?;
    case.u_mean = value(options, "u-mean", case.u_mean)?;
    case.reynolds = value(options, "re", case.reynolds)?;
    case.nu = case.u_mean * case.height / case.reynolds;

    common_config(
        Case::Channel(case),
        options,
        Defaults {
            nx: 200,
            ny: 40,
            dt: 0.002,
            t_end: 100.0,
            max_steps: 40_000,
            output_dir: "results/channel",
        },
    )
}

fn cavity_3d_config(options: &HashMap<String, String>) -> Result<LidDrivenCavity3DConfig, String> {
    let mut cfg = LidDrivenCavity3DConfig::default();
    cfg.nx = value(options, "nx", cfg.nx)?;
    cfg.ny = value(options, "ny", cfg.ny)?;
    cfg.nz = value(options, "nz", cfg.nz)?;
    cfg.length = value(options, "length", cfg.length)?;
    cfg.height = value(options, "height", cfg.height)?;
    cfg.depth = value(options, "depth", cfg.depth)?;
    cfg.density = value(options, "rho", cfg.density)?;
    cfg.lid_velocity = value(options, "u-lid", cfg.lid_velocity)?;
    cfg.reynolds = value(options, "re", cfg.reynolds)?;
    cfg.dt = value(options, "dt", cfg.dt)?;
    cfg.max_steps = value(options, "max-steps", cfg.max_steps)?;
    cfg.pressure_max_iters = value(options, "pressure-iters", cfg.pressure_max_iters)?;
    cfg.pressure_tolerance = value(options, "pressure-tol", cfg.pressure_tolerance)?;
    cfg.pressure_omega = value(options, "pressure-omega", cfg.pressure_omega)?;
    cfg.output_dir = PathBuf::from(
        options
            .get("out")
            .cloned()
            .unwrap_or_else(|| "results/cavity-3d".to_string()),
    );
    Ok(cfg)
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
        "vanleer" | "tvd" => ConvectionScheme::VanLeerTvd,
        other => return Err(format!("Unknown convection scheme '{other}'")),
    };
    let pressure_solver = match options
        .get("pressure-solver")
        .map(String::as_str)
        .unwrap_or("pcg")
    {
        "pcg" => PressureSolverKind::Pcg,
        "pcg-dic" => PressureSolverKind::PcgDic,
        "pcg-mg" | "multigrid" => PressureSolverKind::PcgMg,
        "sor" => PressureSolverKind::Sor,
        other => return Err(format!("Unknown pressure solver '{other}'")),
    };
    let coupling = match options
        .get("coupling")
        .map(String::as_str)
        .unwrap_or("projection")
    {
        "projection" | "transient" => PressureVelocityCoupling::Projection,
        "pimple" => PressureVelocityCoupling::Pimple,
        "simple" | "steady-simple" => PressureVelocityCoupling::Simple,
        "simple-implicit" => PressureVelocityCoupling::SimpleImplicit,
        "simplec" | "simple-consistent" => PressureVelocityCoupling::SimpleConsistent,
        other => return Err(format!("Unknown pressure-velocity coupling '{other}'")),
    };

    Ok(SimulationConfig {
        case,
        nx: value(options, "nx", defaults.nx)?,
        ny: value(options, "ny", defaults.ny)?,
        dt: value(options, "dt", defaults.dt)?,
        time_step: TimeStepSettings::default(),
        t_end: value(options, "t-end", defaults.t_end)?,
        max_steps: value(options, "max-steps", defaults.max_steps)?,
        convection,
        coupling,
        pimple: PimpleControls {
            pressure_correctors: value(options, "pressure-correctors", 2_usize)?,
            momentum_tolerance: value(options, "momentum-tol", 1.0e-5_f64)?,
            first_pressure_relative_tolerance: value(options, "pressure-rel-tol", 0.01_f64)?,
        },
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
        boundary_overrides: SolverBoundaryOverrides::default(),
        physics: Default::default(),
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
    println!("  flursys channel [options]");
    println!("  flursys cavity-3d [options]");

    println!();
    println!("Run `flursys <case> --help` for available options.");
}

fn print_case_help(case: &str) {
    println!("Case: {case}");
    println!();
    println!("Common options:");
    println!("  --nx N --ny N");
    println!("  --dt DT --t-end T --max-steps N");
    println!("  --convection upwind|central|vanleer (Van Leer requires implicit coupling)");
    println!("  --coupling projection|pimple|simple|simple-implicit|simplec");
    println!(
        "  --pressure-solver pcg|pcg-dic|pcg-mg|sor (pcg-mg requires implicit momentum coupling)"
    );
    println!("  --pressure-iters N --pressure-tol EPS --pressure-omega W");
    println!("  --pressure-correctors N --momentum-tol EPS --pressure-rel-tol R (pimple)");
    println!("  --velocity-relax A --pressure-relax A (steady coupling)");
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
        "turek" | "turek-schaefer" => {
            println!(
                "DFG 2D-2 defaults: L=2.2 H=0.41 D=0.1 center=(0.2,0.2), rho=1, Umean=1, Re=100"
            );
            println!("Physical overrides: --length L --height H --diameter D --xc X --yc Y --rho RHO --u-mean U --re RE");
        }
        "backward-step" | "bfs" => {
            println!("Physical options: --length L --height H --step-height Hs --step-x Xs");
            println!("                  --rho RHO --u-mean U --re RE");
        }
        "channel" => {
            println!("Physical options: --length L --height H --rho RHO --u-mean U --re RE");
        }
        "cavity-3d" => {
            println!(
                "Physical options: --length L --height H --depth D --rho RHO --u-lid U --re RE"
            );
            println!("Grid options: --nx N --ny N --nz N");
        }
        _ => println!("Unknown case."),
    }
}
