use crate::cases::{BoundaryKind, Case, CaseKind, Side};
use crate::field::{Field2D, Mask2D};
use crate::grid::UniformGrid2D;
use crate::output;
use crate::physics::{BuoyancyModel, EnergyModel, PhysicsSettings, ThermalBoundaryCondition};
use crate::preprocess::SolverBoundaryOverrides;
use crate::{
    bicgstab_with_preconditioner, pcg, pcg_with_preconditioner, CsrBuilder, CsrMatrix,
    IncompleteCholeskyPreconditioner, JacobiPreconditioner, LinearSolverOptions,
    StructuredMultigridPreconditioner, SymmetricGaussSeidelPreconditioner,
};
use rayon::prelude::*;
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant};

mod boundary;
use self::boundary::*;
mod diagnostics;
use self::diagnostics::*;
mod implicit;
use self::implicit::*;
mod implicit_pressure;
mod momentum;
use self::momentum::*;
mod numerics;
use self::numerics::*;
mod pressure;
mod reporting;
use self::pressure::*;
mod step;
mod thermal;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConvectionScheme {
    FirstOrderUpwind,
    Central,
    VanLeerTvd,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PressureSolverKind {
    Pcg,
    PcgDic,
    PcgMg,
    Sor,
}

/// Pressure-velocity coupling strategy for incompressible flow.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PressureVelocityCoupling {
    /// Transient fractional-step projection method.
    Projection,
    /// Transient pressure-implicit split with implicit momentum and configurable
    /// pressure corrections on the native staggered grid.
    Pimple,
    /// Steady, segregated SIMPLE-style iterations with pseudo-time momentum updates.
    Simple,
    /// Coefficient-based steady SIMPLE with implicit momentum and pressure correction.
    SimpleImplicit,
    /// Consistent SIMPLE (SIMPLEC) with the same staggered finite-volume equations.
    SimpleConsistent,
}

impl PressureVelocityCoupling {
    fn is_coefficient_based(self) -> bool {
        matches!(
            self,
            Self::Pimple | Self::SimpleImplicit | Self::SimpleConsistent
        )
    }

    fn is_consistent(self) -> bool {
        self == Self::SimpleConsistent
    }

    fn is_transient(self) -> bool {
        matches!(self, Self::Projection | Self::Pimple)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PimpleControls {
    pub pressure_correctors: usize,
    pub momentum_tolerance: f64,
    pub first_pressure_relative_tolerance: f64,
}

impl Default for PimpleControls {
    fn default() -> Self {
        Self {
            pressure_correctors: 2,
            momentum_tolerance: 1.0e-5,
            first_pressure_relative_tolerance: 0.01,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TimeStepSettings {
    pub adaptive: bool,
    pub min_dt: f64,
    pub max_dt: f64,
    pub target_cfl: f64,
    pub target_diffusion_number: f64,
    pub max_growth_factor: f64,
}

impl Default for TimeStepSettings {
    fn default() -> Self {
        Self {
            adaptive: false,
            min_dt: 1.0e-6,
            max_dt: 1.0e-2,
            target_cfl: 0.5,
            target_diffusion_number: 0.25,
            max_growth_factor: 1.1,
        }
    }
}

#[derive(Clone, Debug)]
pub struct SimulationConfig {
    pub case: Case,
    pub nx: usize,
    pub ny: usize,
    pub dt: f64,
    pub time_step: TimeStepSettings,
    pub max_steps: usize,
    pub t_end: f64,
    pub convection: ConvectionScheme,
    pub coupling: PressureVelocityCoupling,
    pub pimple: PimpleControls,
    pub pressure_solver: PressureSolverKind,
    pub pressure_max_iters: usize,
    pub pressure_tolerance: f64,
    pub pressure_omega: f64,
    /// SIMPLE momentum under-relaxation. Ignored by the projection method.
    pub velocity_relaxation: f64,
    /// SIMPLE pressure under-relaxation. Ignored by transient projection and PIMPLE.
    pub pressure_relaxation: f64,
    pub print_every: usize,
    pub output_every: usize,
    pub frame_every: usize,
    pub steady_tolerance: f64,
    pub minimum_steps: usize,
    /// Number of worker threads. Zero selects Rayon's automatic CPU count.
    pub threads: usize,
    /// Explicit project-boundary overrides. Empty overrides preserve the
    /// selected case's analytical boundary profiles.
    pub boundary_overrides: SolverBoundaryOverrides,
    /// Physics models assembled by this solver.  Defaults to flow-only.
    pub physics: PhysicsSettings,
    pub output_dir: PathBuf,
}

impl SimulationConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.nx < 4 || self.ny < 4 {
            return Err("nx and ny must be at least 4".to_string());
        }
        if !self.dt.is_finite() || self.dt <= 0.0 {
            return Err("dt must be finite and positive".to_string());
        }
        if !self.t_end.is_finite() || self.t_end < 0.0 {
            return Err("t_end must be finite and non-negative".to_string());
        }
        if self.coupling.is_transient() && self.t_end == 0.0 {
            return Err("t_end must be positive for transient coupling".to_string());
        }
        if self.pimple.pressure_correctors == 0
            || !self.pimple.momentum_tolerance.is_finite()
            || self.pimple.momentum_tolerance <= 0.0
            || !self.pimple.first_pressure_relative_tolerance.is_finite()
            || self.pimple.first_pressure_relative_tolerance < 0.0
        {
            return Err("invalid transient pressure-velocity solver controls".to_string());
        }
        if self.convection == ConvectionScheme::VanLeerTvd && !self.coupling.is_coefficient_based()
        {
            return Err(
                "Van Leer TVD convection currently requires implicit coefficient-based coupling"
                    .to_string(),
            );
        }
        if self.pressure_solver == PressureSolverKind::PcgMg
            && !self.coupling.is_coefficient_based()
        {
            return Err(
                "PCG + structured multigrid currently requires implicit coefficient-based coupling"
                    .to_string(),
            );
        }
        if self.max_steps == 0 {
            return Err("max_steps must be positive".to_string());
        }
        if self.pressure_max_iters == 0 {
            return Err("pressure_max_iters must be positive".to_string());
        }
        if !self.pressure_omega.is_finite()
            || self.pressure_omega <= 0.0
            || self.pressure_omega >= 2.0
        {
            return Err("pressure_omega must lie between 0 and 2".to_string());
        }
        if !self.velocity_relaxation.is_finite()
            || self.velocity_relaxation <= 0.0
            || self.velocity_relaxation > 1.0
        {
            return Err("velocity_relaxation must lie between 0 (exclusive) and 1".to_string());
        }
        if !self.pressure_relaxation.is_finite()
            || self.pressure_relaxation <= 0.0
            || self.pressure_relaxation > 1.0
        {
            return Err("pressure_relaxation must lie between 0 (exclusive) and 1".to_string());
        }
        if !self.steady_tolerance.is_finite() || self.steady_tolerance <= 0.0 {
            return Err("steady_tolerance must be finite and positive".to_string());
        }
        if !self.fluid_properties_are_valid() {
            return Err(
                "fluid density and kinematic viscosity must be finite and positive".to_string(),
            );
        }
        if self.print_every == 0 || self.output_every == 0 || self.frame_every == 0 {
            return Err("Output intervals must be positive".to_string());
        }
        if self.time_step.adaptive {
            if self.coupling != PressureVelocityCoupling::Projection {
                return Err(
                    "adaptive time stepping requires transient projection coupling".to_string(),
                );
            }
            let settings = self.time_step;
            if !settings.min_dt.is_finite() || settings.min_dt <= 0.0 {
                return Err("adaptive min_dt must be finite and positive".to_string());
            }
            if !settings.max_dt.is_finite() || settings.max_dt <= 0.0 {
                return Err("adaptive max_dt must be finite and positive".to_string());
            }
            if settings.min_dt > settings.max_dt {
                return Err("adaptive min_dt must not exceed max_dt".to_string());
            }
            if self.dt < settings.min_dt || self.dt > settings.max_dt {
                return Err("adaptive initial dt must lie between min_dt and max_dt".to_string());
            }
            if !settings.target_cfl.is_finite()
                || settings.target_cfl <= 0.0
                || settings.target_cfl > 1.0
            {
                return Err("adaptive target_cfl must lie between 0 (exclusive) and 1".to_string());
            }
            if !settings.target_diffusion_number.is_finite()
                || settings.target_diffusion_number <= 0.0
                || settings.target_diffusion_number > 0.5
            {
                return Err(
                    "adaptive target_diffusion_number must lie between 0 (exclusive) and 0.5"
                        .to_string(),
                );
            }
            if !settings.max_growth_factor.is_finite() || settings.max_growth_factor < 1.0 {
                return Err("adaptive max_growth_factor must be finite and at least 1".to_string());
            }
        }
        let (length, height) = self.case.domain();
        let thermal_validation_dt = if self.time_step.adaptive {
            self.time_step.min_dt
        } else {
            self.dt
        };
        self.physics.validate(
            thermal_validation_dt,
            length / self.nx as f64,
            height / self.ny as f64,
        )?;
        Ok(())
    }

    fn fluid_properties_are_valid(&self) -> bool {
        let density = self.case.density();
        let viscosity = self.case.kinematic_viscosity();
        density.is_finite() && density > 0.0 && viscosity.is_finite() && viscosity > 0.0
    }
}

#[derive(Clone, Debug)]
pub struct RunSummary {
    pub case_name: String,
    pub steps: usize,
    pub final_time: f64,
    pub max_divergence: f64,
    pub pressure_residual: f64,
    pub momentum_equation_residual: Option<f64>,
    pub elapsed: Duration,
    pub converged: bool,
}

/// Diagnostics and an optional field snapshot produced after one solver iteration.
#[derive(Clone, Debug)]
pub struct SolverStep {
    pub iteration: usize,
    pub time: f64,
    pub time_step: f64,
    pub continuity_residual: f64,
    pub pressure_residual: f64,
    pub pressure_iterations: usize,
    pub momentum_residual: f64,
    pub momentum_equation_residual: Option<f64>,
    pub max_speed: f64,
    pub momentum_cfl: f64,
    pub viscous_diffusion_number: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: f64,
    pub converged: bool,
    /// Whether a transient projection run has reached its configured physical end time.
    pub reached_end_time: bool,
}

/// Cell-centred fields suitable for a UI preview or remote observer.
#[derive(Clone, Debug)]
pub struct FieldUpdate {
    pub nx: usize,
    pub ny: usize,
    pub pressure: Vec<f64>,
    pub speed: Vec<f64>,
    /// Cell-centred velocity reconstructed from the staggered faces for vector plots.
    pub velocity_x: Vec<f64>,
    pub velocity_y: Vec<f64>,
    pub vorticity: Vec<f64>,
    pub solid: Vec<bool>,
    pub temperature: Option<Vec<f64>>,
}

#[derive(Clone, Copy, Debug, Default)]
struct Diagnostics {
    time_step: f64,
    pressure_residual: f64,
    pressure_iterations: usize,
    max_divergence: f64,
    max_speed: f64,
    momentum_cfl: f64,
    viscous_diffusion_number: f64,
    velocity_change: f64,
    momentum_equation_residual: Option<f64>,
    cd: f64,
    cl: f64,
    cd_pressure: f64,
    cd_viscous: f64,
    cl_pressure: f64,
    cl_viscous: f64,
    reattachment: f64,
}

#[derive(Clone, Copy, Debug)]
struct AdaptiveTimeStepInputs {
    max_abs_u: f64,
    max_abs_v: f64,
    dx: f64,
    dy: f64,
    kinematic_viscosity: f64,
    thermal_diffusivity: Option<f64>,
}

pub struct IncompressibleSolver {
    cfg: SimulationConfig,
    grid: UniformGrid2D,
    solid: Mask2D,
    pressure_stencil: PressureStencil2D,
    p: Field2D,
    rhs: Field2D,
    u: Field2D,
    v: Field2D,
    u_star: Field2D,
    v_star: Field2D,
    u_cell: Field2D,
    v_cell: Field2D,
    speed: Field2D,
    vorticity: Field2D,
    temperature: Option<Field2D>,
    step: usize,
    time: f64,
    current_dt: f64,
    frame_index: usize,
    outputs_prepared: bool,
    pcg_workspace: PcgWorkspace,
    implicit_assembly_workspace: Option<ImplicitAssemblyWorkspace>,
    multigrid_preconditioner: Option<StructuredMultigridPreconditioner>,
    last_diag: Diagnostics,
    pool: ThreadPool,
}

impl IncompressibleSolver {
    pub fn new(cfg: SimulationConfig) -> Result<Self, String> {
        cfg.validate()?;
        let (length, height) = cfg.case.domain();
        let grid = UniformGrid2D::new(cfg.nx, cfg.ny, length, height)?;
        let pool = if cfg.threads == 0 {
            ThreadPoolBuilder::new().build()
        } else {
            ThreadPoolBuilder::new().num_threads(cfg.threads).build()
        }
        .map_err(|error| format!("Cannot create CPU worker pool: {error}"))?;
        let mut solid = Mask2D::new(cfg.nx, cfg.ny, false);
        for j in 0..cfg.ny {
            for i in 0..cfg.nx {
                solid[(i, j)] = cfg.case.is_solid(grid.cell_x(i), grid.cell_y(j));
            }
        }
        let pressure_stencil =
            PressureStencil2D::new(&grid, &solid, &cfg.case, &cfg.boundary_overrides);
        let current_dt = cfg.dt;

        let mut solver = Self {
            p: Field2D::new(cfg.nx, cfg.ny, 0.0),
            rhs: Field2D::new(cfg.nx, cfg.ny, 0.0),
            u: Field2D::new(cfg.nx + 1, cfg.ny, 0.0),
            v: Field2D::new(cfg.nx, cfg.ny + 1, 0.0),
            u_star: Field2D::new(cfg.nx + 1, cfg.ny, 0.0),
            v_star: Field2D::new(cfg.nx, cfg.ny + 1, 0.0),
            u_cell: Field2D::new(cfg.nx, cfg.ny, 0.0),
            v_cell: Field2D::new(cfg.nx, cfg.ny, 0.0),
            speed: Field2D::new(cfg.nx, cfg.ny, 0.0),
            vorticity: Field2D::new(cfg.nx, cfg.ny, 0.0),
            temperature: (cfg.physics.thermal.model == EnergyModel::ConstantProperties)
                .then(|| Field2D::new(cfg.nx, cfg.ny, cfg.physics.thermal.initial_temperature)),
            cfg,
            grid,
            solid,
            pressure_stencil,
            step: 0,
            time: 0.0,
            current_dt,
            frame_index: 0,
            outputs_prepared: false,
            pcg_workspace: PcgWorkspace::default(),
            implicit_assembly_workspace: None,
            multigrid_preconditioner: None,
            last_diag: Diagnostics::default(),
            pool,
        };
        solver.initialize_velocity();
        solver.compute_cell_fields();
        Ok(solver)
    }

    pub fn run(&mut self) -> Result<RunSummary, String> {
        self.prepare_output()?;
        let started = Instant::now();
        let mut converged = false;

        println!("Case: {}", self.cfg.case.name());
        println!(
            "Grid: {} x {}, dx={:.6}, dy={:.6}",
            self.grid.nx, self.grid.ny, self.grid.dx, self.grid.dy
        );
        match self.cfg.coupling {
            PressureVelocityCoupling::Projection | PressureVelocityCoupling::Pimple => println!(
                "rho={:.6}, nu={:.6}, dt={:.6}, max_steps={}, t_end={:.6}",
                self.cfg.case.density(),
                self.cfg.case.kinematic_viscosity(),
                self.cfg.dt,
                self.cfg.max_steps,
                self.cfg.t_end
            ),
            PressureVelocityCoupling::Simple => {
                println!(
                    "rho={:.6}, nu={:.6}, pseudo_dt={:.6}, max_iterations={}",
                    self.cfg.case.density(),
                    self.cfg.case.kinematic_viscosity(),
                    self.cfg.dt,
                    self.cfg.max_steps
                )
            }
            PressureVelocityCoupling::SimpleImplicit
            | PressureVelocityCoupling::SimpleConsistent => println!(
                "rho={:.6}, nu={:.6}, iteration_clock_dt={:.6}, max_iterations={}",
                self.cfg.case.density(),
                self.cfg.case.kinematic_viscosity(),
                self.cfg.dt,
                self.cfg.max_steps
            ),
        }
        println!("CPU worker threads: {}", self.pool.current_num_threads());
        if self.cfg.time_step.adaptive {
            let settings = self.cfg.time_step;
            println!(
                "Time stepping: adaptive, initial_dt={:.3e}, min_dt={:.3e}, max_dt={:.3e}, target_cfl={:.3e}, target_diffusion={:.3e}, max_growth={:.3e}",
                self.cfg.dt,
                settings.min_dt,
                settings.max_dt,
                settings.target_cfl,
                settings.target_diffusion_number,
                settings.max_growth_factor
            );
        } else {
            if self.cfg.coupling.is_coefficient_based() && !self.cfg.coupling.is_transient() {
                println!("Time stepping: steady outer iterations; dt is the output/thermal iteration clock only ({:.3e})", self.cfg.dt);
            } else {
                println!("Time stepping: fixed, dt={:.3e}", self.cfg.dt);
            }
        }
        // These are diagnostic reference numbers, not universal stability guarantees.
        println!(
            "Momentum stability diagnostics: CFL <= 1 and Dnu <= 0.5 are common explicit reference limits."
        );
        match self.cfg.coupling {
            PressureVelocityCoupling::Projection => println!("Coupling: transient projection"),
            PressureVelocityCoupling::Pimple => println!(
                "Coupling: transient PIMPLE-like, implicit Euler momentum + {} pressure corrections",
                self.cfg.pimple.pressure_correctors
            ),
            PressureVelocityCoupling::Simple => println!(
                "Coupling: steady SIMPLE-style explicit pseudo-time, velocity relaxation={:.3}, pressure relaxation={:.3}",
                self.cfg.velocity_relaxation, self.cfg.pressure_relaxation
            ),
            PressureVelocityCoupling::SimpleImplicit => println!(
                "Coupling: steady implicit SIMPLE, {:?} momentum convection, velocity relaxation={:.3}, pressure relaxation={:.3}",
                self.cfg.convection,
                self.cfg.velocity_relaxation, self.cfg.pressure_relaxation
            ),
            PressureVelocityCoupling::SimpleConsistent => println!(
                "Coupling: steady implicit SIMPLEC, {:?} momentum convection, velocity relaxation={:.3}, pressure relaxation={:.3}",
                self.cfg.convection,
                self.cfg.velocity_relaxation, self.cfg.pressure_relaxation
            ),
        }
        if self.cfg.coupling.is_coefficient_based() {
            println!(
                "Implicit momentum/correction solvers: momentum BiCGStab; pressure correction PCG + {}, max_iters={}",
                match self.cfg.pressure_solver {
                    PressureSolverKind::PcgDic => "DIC(0)",
                    PressureSolverKind::PcgMg => "structured multigrid",
                    PressureSolverKind::Pcg | PressureSolverKind::Sor => "Jacobi",
                },
                self.cfg.pressure_max_iters.max(100)
            );
            if self.cfg.coupling == PressureVelocityCoupling::Pimple {
                println!(
                    "Linear stopping: momentum abs={:.3e}; pressure abs={:.3e}, first rel={:.3e}, final rel=0",
                    self.cfg.pimple.momentum_tolerance,
                    self.cfg.pressure_tolerance,
                    self.cfg.pimple.first_pressure_relative_tolerance
                );
            } else {
                println!(
                    "Linear stopping: relative={:.3e}",
                    self.cfg.pressure_tolerance
                );
            }
            if self.cfg.pressure_solver == PressureSolverKind::Sor {
                println!(
                    "SOR is not supported for coefficient-based coupling; using PCG + Jacobi."
                );
            }
        } else {
            match self.cfg.pressure_solver {
                PressureSolverKind::Pcg => println!(
                    "Pressure: PCG + Jacobi preconditioner, tol={:.3e}, max_iters={}",
                    self.cfg.pressure_tolerance, self.cfg.pressure_max_iters
                ),
                PressureSolverKind::PcgDic => println!(
                    "Pressure: PCG + DIC(0) preconditioner ({}), tol={:.3e}, max_iters={}",
                    if self.pressure_stencil.dic_supported {
                        "structured no-solid mesh"
                    } else {
                        "falling back to Jacobi for this mesh/boundary"
                    },
                    self.cfg.pressure_tolerance,
                    self.cfg.pressure_max_iters
                ),
                PressureSolverKind::PcgMg => println!(
                    "Pressure: PCG + structured multigrid (coefficient-based coupling), tol={:.3e}, max_iters={}",
                    self.cfg.pressure_tolerance,
                    self.cfg.pressure_max_iters
                ),
                PressureSolverKind::Sor => println!(
                    "Pressure: SOR omega={:.3}, tol={:.3e}, max_iters={}",
                    self.cfg.pressure_omega,
                    self.cfg.pressure_tolerance,
                    self.cfg.pressure_max_iters
                ),
            }
        }
        if !self.cfg.coupling.is_coefficient_based()
            && self.cfg.pressure_solver == PressureSolverKind::Sor
            && self.pool.current_num_threads() > 1
        {
            println!(
                "SOR uses sequential in-place sweeps; use PCG to parallelize the pressure solve."
            );
        }

        while self.step < self.cfg.max_steps && !self.reached_end_time() {
            let diag = self.advance_one_step()?;
            self.last_diag = diag;

            if self.step.is_multiple_of(self.cfg.output_every) {
                self.append_histories(diag)?;
            }
            if self.step.is_multiple_of(self.cfg.frame_every) {
                self.write_snapshot()?;
            }
            if self.step.is_multiple_of(self.cfg.print_every) {
                let progress = if self.cfg.coupling.is_transient() {
                    format!(
                        "step {:>8}/{:<8} t={:>10.4}",
                        self.step, self.cfg.max_steps, self.time
                    )
                } else {
                    format!("iteration {:>8}/{:<8}", self.step, self.cfg.max_steps)
                };
                println!(
                    "{progress} dt={:.3e} div={:.3e} p_res={:.3e} p_it={:>4} du={:.3e} rU={} umax={:.5} CFL={:.3e} Dnu={:.3e} Cd={:.5} Cl={:.5} xr/h={:.5} elapsed={:.1}s",
                    diag.time_step,
                    diag.max_divergence,
                    diag.pressure_residual,
                    diag.pressure_iterations,
                    diag.velocity_change,
                    diag.momentum_equation_residual
                        .map(|residual| format!("{residual:.3e}"))
                        .unwrap_or_else(|| "n/a".to_string()),
                    diag.max_speed,
                    diag.momentum_cfl,
                    diag.viscous_diffusion_number,
                    diag.cd,
                    diag.cl,
                    diag.reattachment,
                    started.elapsed().as_secs_f64()
                );
            }

            if self.has_steady_converged(diag) {
                converged = true;
                break;
            }
        }

        self.compute_cell_fields();
        self.write_final_outputs()?;
        Ok(RunSummary {
            case_name: self.cfg.case.name().to_string(),
            steps: self.step,
            final_time: self.time,
            max_divergence: self.last_diag.max_divergence,
            pressure_residual: self.last_diag.pressure_residual,
            momentum_equation_residual: self.last_diag.momentum_equation_residual,
            elapsed: started.elapsed(),
            converged,
        })
    }

    /// Prepare persistent output for an interactive run.
    pub fn prepare_output(&mut self) -> Result<(), String> {
        if self.outputs_prepared {
            return Ok(());
        }
        output::ensure_output_tree(&self.cfg.output_dir)?;
        self.write_case_summary()?;
        self.write_snapshot()?;
        self.outputs_prepared = true;
        Ok(())
    }

    /// Persist the final state for an interactive run, including a final frame.
    pub fn finalize_output(&mut self) -> Result<(), String> {
        self.prepare_output()?;
        self.write_snapshot()?;
        self.compute_cell_fields();
        self.write_final_outputs()
    }

    /// Advance the numerical state by one iteration, preserving the configured
    /// history and frame cadence for interactive frontends.
    pub fn advance(&mut self) -> Result<SolverStep, String> {
        self.prepare_output()?;
        let diag = self.advance_one_step()?;
        self.last_diag = diag;
        if self.step.is_multiple_of(self.cfg.output_every) {
            self.append_histories(diag)?;
        }
        if self.step.is_multiple_of(self.cfg.frame_every) {
            self.write_snapshot()?;
        }
        let converged = self.has_steady_converged(diag);
        let reached_end_time = self.reached_end_time();

        Ok(SolverStep {
            iteration: self.step,
            time: self.time,
            time_step: diag.time_step,
            continuity_residual: diag.max_divergence,
            pressure_residual: diag.pressure_residual,
            pressure_iterations: diag.pressure_iterations,
            momentum_residual: diag.velocity_change,
            momentum_equation_residual: diag.momentum_equation_residual,
            max_speed: diag.max_speed,
            momentum_cfl: diag.momentum_cfl,
            viscous_diffusion_number: diag.viscous_diffusion_number,
            drag_coefficient: diag.cd,
            lift_coefficient: diag.cl,
            converged,
            reached_end_time,
        })
    }

    fn reached_end_time(&self) -> bool {
        let time_tolerance = f64::EPSILON * self.cfg.t_end.abs().max(1.0) * 16.0;
        self.cfg.coupling.is_transient() && self.time >= self.cfg.t_end - time_tolerance
    }

    fn has_steady_converged(&self, diag: Diagnostics) -> bool {
        !self.cfg.coupling.is_transient()
            && self.cfg.case.kind() != CaseKind::CylinderRe100
            && self.step >= self.cfg.minimum_steps
            && diag.velocity_change < self.cfg.steady_tolerance
            && diag.max_divergence < 10.0 * self.cfg.pressure_tolerance
            && (!self.cfg.coupling.is_coefficient_based()
                || diag
                    .momentum_equation_residual
                    .is_some_and(|residual| residual < self.cfg.steady_tolerance))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cases::CavityCase;
    use crate::{
        BoundaryConditionKind, BoundaryFace, BuoyancyModel, EnergyModel, Project,
        ThermalBoundaryCondition,
    };

    fn test_config(coupling: PressureVelocityCoupling, output_dir: &str) -> SimulationConfig {
        SimulationConfig {
            case: Case::LidDrivenCavity(CavityCase::default()),
            nx: 8,
            ny: 8,
            dt: 1.0e-3,
            time_step: TimeStepSettings::default(),
            max_steps: 10,
            t_end: 2.5e-3,
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling,
            pimple: Default::default(),
            pressure_solver: PressureSolverKind::Pcg,
            pressure_max_iters: 50,
            pressure_tolerance: 1.0e-4,
            pressure_omega: 1.5,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            print_every: 100,
            output_every: 100,
            frame_every: 100,
            steady_tolerance: 1.0e-12,
            minimum_steps: 100,
            threads: 1,
            boundary_overrides: SolverBoundaryOverrides::default(),
            physics: Default::default(),
            output_dir: PathBuf::from(output_dir),
        }
    }

    #[test]
    fn momentum_stability_numbers_match_known_values() {
        let (momentum_cfl, viscous_diffusion_number) =
            momentum_stability_numbers(0.01, 0.1, 0.5, 0.25, 2.0, 1.0);

        assert!((momentum_cfl - 0.08).abs() < 1.0e-12);
        assert!((viscous_diffusion_number - 0.02).abs() < 1.0e-12);
    }

    #[test]
    fn momentum_stability_numbers_report_zero_cfl_at_zero_velocity() {
        let (momentum_cfl, viscous_diffusion_number) =
            momentum_stability_numbers(0.1, 0.25, 0.5, 0.25, 0.0, 0.0);

        assert_eq!(momentum_cfl, 0.0);
        assert!(viscous_diffusion_number > 0.0);
        assert!((viscous_diffusion_number - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn momentum_stability_numbers_include_both_rectangular_grid_directions() {
        let (cfl, _) = momentum_stability_numbers(0.1, 0.2, 0.5, 0.25, 2.0, 1.0);
        let (swapped_cfl, _) = momentum_stability_numbers(0.1, 0.2, 0.25, 0.5, 1.0, 2.0);

        assert!((cfl - swapped_cfl).abs() < 1.0e-12);
    }

    #[test]
    fn adaptive_time_step_is_limited_by_advection() {
        let settings = TimeStepSettings {
            adaptive: true,
            min_dt: 1.0e-3,
            max_dt: 1.0,
            target_cfl: 0.5,
            target_diffusion_number: 0.5,
            max_growth_factor: 10.0,
        };

        let dt = adaptive_time_step(
            0.1,
            AdaptiveTimeStepInputs {
                max_abs_u: 2.0,
                max_abs_v: 1.0,
                dx: 0.5,
                dy: 0.25,
                kinematic_viscosity: 1.0e-3,
                thermal_diffusivity: None,
            },
            settings,
        )
        .unwrap();

        assert!((dt - 0.0625).abs() < 1.0e-12);
    }

    #[test]
    fn adaptive_time_step_is_limited_by_momentum_diffusion() {
        let settings = TimeStepSettings {
            adaptive: true,
            min_dt: 1.0e-3,
            max_dt: 1.0,
            target_cfl: 1.0,
            target_diffusion_number: 0.25,
            max_growth_factor: 10.0,
        };

        let dt = adaptive_time_step(
            0.5,
            AdaptiveTimeStepInputs {
                max_abs_u: 0.0,
                max_abs_v: 0.0,
                dx: 0.5,
                dy: 0.25,
                kinematic_viscosity: 0.1,
                thermal_diffusivity: None,
            },
            settings,
        )
        .unwrap();

        assert!((dt - 0.125).abs() < 1.0e-12);
    }

    #[test]
    fn adaptive_time_step_uses_thermal_diffusivity_when_larger() {
        let settings = TimeStepSettings {
            adaptive: true,
            min_dt: 1.0e-3,
            max_dt: 1.0,
            target_cfl: 1.0,
            target_diffusion_number: 0.25,
            max_growth_factor: 10.0,
        };

        let dt = adaptive_time_step(
            0.5,
            AdaptiveTimeStepInputs {
                max_abs_u: 0.0,
                max_abs_v: 0.0,
                dx: 0.5,
                dy: 0.25,
                kinematic_viscosity: 0.1,
                thermal_diffusivity: Some(0.4),
            },
            settings,
        )
        .unwrap();

        assert!((dt - 0.03125).abs() < 1.0e-12);
    }

    #[test]
    fn adaptive_time_step_limits_growth() {
        let settings = TimeStepSettings {
            adaptive: true,
            min_dt: 1.0e-3,
            max_dt: 0.8,
            target_cfl: 1.0,
            target_diffusion_number: 0.25,
            max_growth_factor: 1.1,
        };

        let dt = adaptive_time_step(
            0.1,
            AdaptiveTimeStepInputs {
                max_abs_u: 0.0,
                max_abs_v: 0.0,
                dx: 1.0,
                dy: 1.0,
                kinematic_viscosity: 1.0e-3,
                thermal_diffusivity: None,
            },
            settings,
        )
        .unwrap();

        assert!((dt - 0.11).abs() < 1.0e-12);
    }

    #[test]
    fn adaptive_time_step_rejects_an_unstable_minimum() {
        let settings = TimeStepSettings {
            adaptive: true,
            min_dt: 0.2,
            max_dt: 1.0,
            target_cfl: 1.0,
            target_diffusion_number: 0.25,
            max_growth_factor: 10.0,
        };

        let error = adaptive_time_step(
            0.5,
            AdaptiveTimeStepInputs {
                max_abs_u: 0.0,
                max_abs_v: 0.0,
                dx: 0.5,
                dy: 0.25,
                kinematic_viscosity: 0.1,
                thermal_diffusivity: None,
            },
            settings,
        )
        .unwrap_err();

        assert!(error.contains("required stable time step"));
        assert!(error.contains("min_dt"));
    }

    #[test]
    fn cavity_configuration_builds() {
        let cfg = SimulationConfig {
            case: Case::LidDrivenCavity(CavityCase::default()),
            nx: 16,
            ny: 16,
            dt: 1.0e-3,
            time_step: TimeStepSettings::default(),
            max_steps: 2,
            t_end: 0.002,
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling: PressureVelocityCoupling::Projection,
            pimple: Default::default(),
            pressure_solver: PressureSolverKind::Pcg,
            pressure_max_iters: 10,
            pressure_tolerance: 1.0e-4,
            pressure_omega: 1.5,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            print_every: 1,
            output_every: 1,
            frame_every: 1,
            steady_tolerance: 1.0e-8,
            minimum_steps: 1,
            threads: 1,
            boundary_overrides: SolverBoundaryOverrides::default(),
            physics: Default::default(),
            output_dir: PathBuf::from("target/test-output"),
        };
        let solver = IncompressibleSolver::new(cfg);
        assert!(solver.is_ok());
    }

    #[test]
    fn projection_step_reports_when_the_physical_end_time_is_reached() {
        let mut solver = IncompressibleSolver::new(test_config(
            PressureVelocityCoupling::Projection,
            "target/projection-end-time-step-test",
        ))
        .unwrap();

        let mut step = solver.advance().unwrap();
        while !step.reached_end_time {
            step = solver.advance().unwrap();
        }

        assert!((step.time - 2.5e-3).abs() < 1.0e-12);
        assert!(step.reached_end_time);
        assert_eq!(
            solver.advance().unwrap_err(),
            "projection simulation has already reached t_end"
        );
    }

    #[test]
    fn simple_steps_do_not_report_a_physical_end_time() {
        let mut config = test_config(
            PressureVelocityCoupling::Simple,
            "target/simple-end-time-step-test",
        );
        config.t_end = 0.0;
        let mut solver = IncompressibleSolver::new(config).unwrap();

        let step = solver.advance().unwrap();

        assert!(!step.reached_end_time);
        assert_eq!(step.iteration, 1);
    }

    #[test]
    fn default_time_step_settings_preserve_fixed_step_behavior() {
        let cfg = SimulationConfig {
            case: Case::LidDrivenCavity(CavityCase::default()),
            nx: 8,
            ny: 8,
            dt: 1.0e-3,
            time_step: TimeStepSettings::default(),
            max_steps: 2,
            t_end: 0.002,
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling: PressureVelocityCoupling::Projection,
            pimple: Default::default(),
            pressure_solver: PressureSolverKind::Pcg,
            pressure_max_iters: 20,
            pressure_tolerance: 1.0e-4,
            pressure_omega: 1.5,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            print_every: 100,
            output_every: 100,
            frame_every: 100,
            steady_tolerance: 1.0e-8,
            minimum_steps: 1,
            threads: 1,
            boundary_overrides: SolverBoundaryOverrides::default(),
            physics: Default::default(),
            output_dir: PathBuf::from("target/fixed-time-step-default"),
        };
        let mut explicit_cfg = cfg.clone();
        explicit_cfg.output_dir = PathBuf::from("target/fixed-time-step-explicit");
        let mut fixed = IncompressibleSolver::new(cfg).unwrap();
        let mut explicit = IncompressibleSolver::new(explicit_cfg).unwrap();

        let fixed_first = fixed.advance().unwrap();
        let explicit_first = explicit.advance().unwrap();
        let fixed_second = fixed.advance().unwrap();
        let explicit_second = explicit.advance().unwrap();

        assert!(!fixed.cfg.time_step.adaptive);
        assert!((fixed_first.time - 1.0e-3).abs() < 1.0e-12);
        assert!((fixed_second.time - 2.0e-3).abs() < 1.0e-12);
        assert!((fixed_first.time_step - 1.0e-3).abs() < 1.0e-12);
        assert!((fixed_second.time_step - 1.0e-3).abs() < 1.0e-12);
        for (left, right) in fixed.u.as_slice().iter().zip(explicit.u.as_slice()) {
            assert!((left - right).abs() < 1.0e-12);
        }
        assert!((explicit_first.time - fixed_first.time).abs() < 1.0e-12);
        assert!((explicit_second.time - fixed_second.time).abs() < 1.0e-12);
    }

    #[test]
    fn project_boundary_override_sets_the_solver_face_velocity() {
        let mut project = Project::default();
        project
            .preprocessing
            .boundary_mut(BoundaryFace::Left)
            .unwrap()
            .kind = BoundaryConditionKind::Velocity {
            u: 1.75,
            v: 0.0,
            w: 0.0,
        };
        let config = project
            .simulation_config("target/boundary-application-test")
            .unwrap();
        let solver = IncompressibleSolver::new(config).unwrap();
        assert_eq!(solver.u[(0, solver.grid.ny / 2)], 1.75);
    }

    #[test]
    fn fixed_temperature_boundary_diffuses_into_the_domain() {
        let mut cfg = SimulationConfig {
            case: Case::LidDrivenCavity(CavityCase::default()),
            nx: 16,
            ny: 16,
            dt: 1.0e-4,
            time_step: TimeStepSettings::default(),
            max_steps: 2,
            t_end: 2.0e-4,
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling: PressureVelocityCoupling::Projection,
            pimple: Default::default(),
            pressure_solver: PressureSolverKind::Pcg,
            pressure_max_iters: 50,
            pressure_tolerance: 1.0e-4,
            pressure_omega: 1.5,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            print_every: 1,
            output_every: 1,
            frame_every: 1,
            steady_tolerance: 1.0e-8,
            minimum_steps: 1,
            threads: 1,
            boundary_overrides: SolverBoundaryOverrides::default(),
            physics: Default::default(),
            output_dir: PathBuf::from("target/thermal-boundary-test"),
        };
        cfg.physics.thermal.model = EnergyModel::ConstantProperties;
        cfg.physics.thermal.initial_temperature = 300.0;
        cfg.physics.thermal.thermal_diffusivity = 1.0e-3;
        cfg.physics.thermal.bottom =
            ThermalBoundaryCondition::FixedTemperature { temperature: 400.0 };

        let mut solver = IncompressibleSolver::new(cfg).unwrap();
        solver.advance().unwrap();
        let temperature = solver.temperature.as_ref().unwrap();
        assert!(temperature[(8, 0)] > 300.0);
        assert!((temperature[(8, 8)] - 300.0).abs() < 1.0e-9);
    }

    #[test]
    fn buoyancy_cannot_be_enabled_without_energy_transport() {
        let mut cfg = SimulationConfig {
            case: Case::LidDrivenCavity(CavityCase::default()),
            nx: 16,
            ny: 16,
            dt: 1.0e-3,
            time_step: TimeStepSettings::default(),
            max_steps: 2,
            t_end: 0.002,
            convection: ConvectionScheme::FirstOrderUpwind,
            coupling: PressureVelocityCoupling::Projection,
            pimple: Default::default(),
            pressure_solver: PressureSolverKind::Pcg,
            pressure_max_iters: 10,
            pressure_tolerance: 1.0e-4,
            pressure_omega: 1.5,
            velocity_relaxation: 0.7,
            pressure_relaxation: 0.3,
            print_every: 1,
            output_every: 1,
            frame_every: 1,
            steady_tolerance: 1.0e-8,
            minimum_steps: 1,
            threads: 1,
            boundary_overrides: SolverBoundaryOverrides::default(),
            physics: Default::default(),
            output_dir: PathBuf::from("target/buoyancy-validation-test"),
        };
        cfg.physics.buoyancy = BuoyancyModel::Boussinesq {
            reference_temperature: 293.15,
            thermal_expansion: 3.4e-3,
            gravity_x: 0.0,
            gravity_y: -9.81,
        };
        let error = IncompressibleSolver::new(cfg).err().unwrap();
        assert!(error.contains("requires the constant-property energy equation"));
    }
}
