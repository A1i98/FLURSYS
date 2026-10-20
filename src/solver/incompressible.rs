use crate::cases::{BoundaryKind, Case, CaseKind, Side};
use crate::field::{Field2D, Mask2D};
use crate::grid::UniformGrid2D;
use crate::output;
use crate::physics::{BuoyancyModel, EnergyModel, PhysicsSettings, ThermalBoundaryCondition};
use crate::preprocess::SolverBoundaryOverrides;
use crate::{
    bicgstab_with_preconditioner_and_workspace, pcg_with_preconditioner_and_workspace,
    AlgebraicMultigrid, BiCgStabWorkspace, CgWorkspace, CsrBuilder, CsrMatrix,
    IncompleteCholeskyPreconditioner, JacobiPreconditioner, LinearSolverOptions, MultigridOptions,
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
mod force_balance;
use self::diagnostics::*;
mod implicit;
use self::implicit::*;
mod time_integration;
use self::time_integration::TransientMomentum;
pub use self::time_integration::TransientTimeScheme;
mod implicit_pressure;
mod implicit_residual;
mod momentum;
use self::momentum::*;
mod numerics;
use self::numerics::*;
mod pressure;
mod reporting;
use self::pressure::*;
mod profile;
use self::profile::{ProfileScope, ProfileStage};
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
    Gamg,
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
    pub time_scheme: TransientTimeScheme,
    /// Reassemble convection and deferred corrections at the current time level.
    pub outer_correctors: usize,
    pub pressure_correctors: usize,
    pub momentum_tolerance: f64,
    pub first_pressure_relative_tolerance: f64,
}

impl Default for PimpleControls {
    fn default() -> Self {
        Self {
            time_scheme: TransientTimeScheme::Euler,
            outer_correctors: 1,
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
            || self.pimple.outer_correctors == 0
            || !self.pimple.momentum_tolerance.is_finite()
            || self.pimple.momentum_tolerance <= 0.0
            || !self.pimple.first_pressure_relative_tolerance.is_finite()
            || self.pimple.first_pressure_relative_tolerance < 0.0
        {
            return Err("invalid transient pressure-velocity solver controls".to_string());
        }
        if self.pimple.time_scheme == TransientTimeScheme::Backward
            && (self.coupling != PressureVelocityCoupling::Pimple
                || self.physics.thermal.model != EnergyModel::Off
                || self.pimple.outer_correctors < 2)
        {
            return Err("BDF2 requires PIMPLE coupling, flow-only physics, and at least two outer correctors".into());
        }
        if self.convection == ConvectionScheme::VanLeerTvd && !self.coupling.is_coefficient_based()
        {
            return Err(
                "Van Leer TVD convection currently requires implicit coefficient-based coupling"
                    .to_string(),
            );
        }
        if matches!(
            self.pressure_solver,
            PressureSolverKind::PcgMg | PressureSolverKind::Gamg
        ) && !self.coupling.is_coefficient_based()
        {
            return Err(
                "multigrid pressure solvers require implicit coefficient-based coupling"
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
    control_volume_force: Option<(f64, f64)>,
    control_volume_outer_force: Option<(f64, f64)>,
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
    previous_time_velocity: Option<(Field2D, Field2D, f64)>,
    frame_index: usize,
    outputs_prepared: bool,
    pcg_workspace: PcgWorkspace,
    pressure_cg_workspace: CgWorkspace,
    momentum_bicg_workspace: BiCgStabWorkspace,
    implicit_assembly_workspace: Option<ImplicitAssemblyWorkspace>,
    multigrid_preconditioner: Option<StructuredMultigridPreconditioner>,
    multigrid_smoothing_steps: usize,
    algebraic_multigrid: Option<AlgebraicMultigrid>,
    algebraic_multigrid_options: MultigridOptions,
    verification_snapshots: Option<(usize, f64)>,
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
            previous_time_velocity: None,
            frame_index: 0,
            outputs_prepared: false,
            pcg_workspace: PcgWorkspace::default(),
            pressure_cg_workspace: CgWorkspace::default(),
            momentum_bicg_workspace: BiCgStabWorkspace::default(),
            implicit_assembly_workspace: None,
            multigrid_preconditioner: None,
            multigrid_smoothing_steps: 1,
            algebraic_multigrid: None,
            algebraic_multigrid_options: MultigridOptions::default(),
            verification_snapshots: None,
            last_diag: Diagnostics::default(),
            pool,
        };
        solver.initialize_velocity();
        solver.compute_cell_fields();
        Ok(solver)
    }

    /// Configure the standalone multigrid hierarchy and invalidate its cached setup.
    pub fn set_algebraic_multigrid_options(
        &mut self,
        options: MultigridOptions,
    ) -> Result<(), String> {
        options.validate().map_err(|error| error.to_string())?;
        if options != self.algebraic_multigrid_options {
            self.algebraic_multigrid_options = options;
            self.algebraic_multigrid = None;
        }
        Ok(())
    }

    pub fn algebraic_multigrid_statistics(&self) -> Option<&[crate::MultigridLevelStatistics]> {
        self.algebraic_multigrid
            .as_ref()
            .map(AlgebraicMultigrid::statistics)
    }

    /// Set matched forward/backward multigrid sweeps, preserving symmetry for PCG.
    pub fn set_multigrid_smoothing_steps(&mut self, steps: usize) -> Result<(), String> {
        if !(1..=8).contains(&steps) {
            return Err("multigrid smoothing steps must be between 1 and 8".to_string());
        }
        if steps != self.multigrid_smoothing_steps {
            self.multigrid_smoothing_steps = steps;
            self.multigrid_preconditioner = None;
        }
        Ok(())
    }

    /// Save cell fields at a fixed step cadence for phase-aligned verification.
    pub fn set_verification_snapshots(
        &mut self,
        every: usize,
        from_time: f64,
    ) -> Result<(), String> {
        if every == 0 || !from_time.is_finite() || from_time < 0.0 {
            return Err(
                "field snapshot interval must be positive and start time finite/nonnegative"
                    .to_string(),
            );
        }
        self.verification_snapshots = Some((every, from_time));
        Ok(())
    }

    pub fn run(&mut self) -> Result<RunSummary, String> {
        {
            let _profile = ProfileScope::new(ProfileStage::Output);
            self.prepare_output()?;
        }
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
                "Coupling: transient PIMPLE-like, implicit {:?} momentum, {} outer correctors + {} pressure corrections",
                self.cfg.pimple.time_scheme,
                self.cfg.pimple.outer_correctors,
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
                "Implicit momentum/correction solvers: momentum BiCGStab; pressure correction {}, max_iters={}",
                match self.cfg.pressure_solver {
                    PressureSolverKind::PcgDic => "PCG + DIC(0)",
                    PressureSolverKind::PcgMg => "PCG + structured multigrid",
                    PressureSolverKind::Gamg if self.algebraic_multigrid_options.krylov_dimension > 0 => "GAMG + flexible GCR",
                    PressureSolverKind::Gamg => "standalone algebraic multigrid",
                    PressureSolverKind::Pcg | PressureSolverKind::Sor => "PCG + Jacobi",
                },
                if self.cfg.pressure_solver == PressureSolverKind::Gamg {
                    self.cfg.pressure_max_iters
                } else { self.cfg.pressure_max_iters.max(100) }
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
            if self.cfg.pressure_solver == PressureSolverKind::Gamg {
                println!(
                    "Multigrid aggregation: shared-face-area, connected clusters. Controls: {:?}",
                    self.algebraic_multigrid_options
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
                PressureSolverKind::PcgMg | PressureSolverKind::Gamg => println!(
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
                let _profile = ProfileScope::new(ProfileStage::Output);
                self.append_histories(diag)?;
            }
            if self.step.is_multiple_of(self.cfg.frame_every) {
                let _profile = ProfileScope::new(ProfileStage::Output);
                self.write_snapshot()?;
            }
            self.write_verification_snapshot_if_due()?;
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

        {
            let _profile = ProfileScope::new(ProfileStage::Output);
            self.compute_cell_fields();
            self.write_final_outputs()?;
        }
        ProfileScope::report();
        if self.algebraic_multigrid_options.profile {
            if let Some(statistics) = self.algebraic_multigrid_statistics() {
                println!("Multigrid levels (times include recursive children):");
                for (index, level) in statistics.iter().enumerate() {
                    println!("  level {index}: {} cells, {} visits, {} smoother sweeps, {} coarse iterations, {:.6}s",
                        level.cells, level.visits, level.smoothing_sweeps, level.coarse_iterations, level.inclusive_seconds);
                }
            }
        }
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
        self.write_verification_snapshot_if_due()?;
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
    fn control_volume_force_obeys_affine_stress_and_bdf2_momentum_balance() {
        let mut cfg = test_config(PressureVelocityCoupling::Pimple, "unused-cv-force-test");
        cfg.case = Case::CylinderRe100(crate::cases::CylinderCase {
            length: 1.0,
            height: 1.0,
            diameter: 0.2,
            xc: 0.5,
            yc: 0.5,
            ..crate::cases::CylinderCase::default()
        });
        cfg.nx = 40;
        cfg.ny = 40;
        cfg.pimple.time_scheme = TransientTimeScheme::Backward;
        cfg.pimple.outer_correctors = 3;
        let mut solver = IncompressibleSolver::new(cfg).unwrap();
        solver.u.fill(0.0);
        solver.v.fill(0.0);
        let old_u = solver.u.clone();
        let old_v = solver.v.clone();
        solver.compute_cell_fields();
        for j in 0..40 {
            for i in 0..40 {
                solver.p[(i, j)] = 17.0 + 3.0 * solver.grid.cell_x(i) - 2.0 * solver.grid.cell_y(j);
            }
        }
        let (cd, cl) = solver
            .control_volume_cylinder_force(0.1, &old_u, &old_v)
            .unwrap();
        assert!((cd + 7.5).abs() < 1e-10, "{cd}");
        assert!((cl - 5.0).abs() < 1e-10, "{cl}");
        // Empty fluid CV: uniform U=(t^2,-t^2/2), p=-2tx+ty+C.
        // Accumulation and stress exactly cancel; convection cancels pairwise.
        solver.solid = Mask2D::new(40, 40, false);
        solver.u.fill(1.0);
        solver.v.fill(-0.5);
        let (mut old_u, mut old_v) = (solver.u.clone(), solver.v.clone());
        old_u.fill(0.81);
        old_v.fill(-0.405);
        let (mut older_u, mut older_v) = (solver.u.clone(), solver.v.clone());
        older_u.fill(0.64);
        older_v.fill(-0.32);
        solver.previous_time_velocity = Some((older_u, older_v, 0.1));
        solver.compute_cell_fields();
        for offset in [0.0, 1000.0] {
            for j in 0..40 {
                for i in 0..40 {
                    solver.p[(i, j)] = offset - 2.0 * solver.grid.cell_x(i) + solver.grid.cell_y(j);
                }
            }
            let (cd, cl) = solver
                .control_volume_cylinder_force(0.1, &old_u, &old_v)
                .unwrap();
            assert!(cd.abs() < 1e-10 && cl.abs() < 1e-10, "{cd}, {cl}");
            let (cd, cl) = solver
                .control_volume_cylinder_force_with_extent(0.1, &old_u, &old_v, 1.5)
                .unwrap();
            assert!(cd.abs() < 1e-10 && cl.abs() < 1e-10, "outer {cd}, {cl}");
        }
    }

    #[test]
    fn closed_cylinder_pressure_force_preserves_gauge_and_affine_balance() {
        for (nx, ny) in [(44, 20), (88, 40)] {
            let mut cfg = test_config(PressureVelocityCoupling::Pimple, "unused-force-audit");
            cfg.case = Case::CylinderRe100(crate::cases::CylinderCase {
                length: 2.2,
                height: 1.0,
                diameter: 0.4,
                xc: 0.8,
                yc: 0.5,
                rho: 2.0,
                u_inf: 1.5,
                ..crate::cases::CylinderCase::default()
            });
            cfg.nx = nx;
            cfg.ny = ny;
            let mut solver = IncompressibleSolver::new(cfg).unwrap();
            solver.u_cell.as_mut_slice().fill(0.0);
            solver.v_cell.as_mut_slice().fill(0.0);
            let area = solver.solid.as_slice().iter().filter(|&&s| s).count() as f64
                * solver.grid.dx
                * solver.grid.dy;
            let q = 0.5 * 2.0 * 1.5_f64.powi(2) * 0.4;
            for offset in [0.0, 7.0, 1000.0] {
                for j in 0..ny {
                    for i in 0..nx {
                        solver.p[(i, j)] =
                            offset + 3.0 * solver.grid.cell_x(i) - 2.0 * solver.grid.cell_y(j);
                    }
                }
                let force = solver.cylinder_force_coefficients();
                assert!((force.cd_pressure + 3.0 * area / q).abs() < 1e-11);
                assert!((force.cl_pressure - 2.0 * area / q).abs() < 1e-11);
                assert_eq!(force.cd_viscous, 0.0);
                assert_eq!(force.cl_viscous, 0.0);
            }
            solver.p.as_mut_slice().fill(17.0);
            let force = solver.cylinder_force_coefficients();
            assert!(force.cd().abs() < 1e-12);
            assert!(force.cl().abs() < 1e-12);
        }
    }

    #[test]
    fn backward_cavity_time_refinement_matches_fine_time_reference() {
        fn integrate(scheme: TransientTimeScheme, dt: f64) -> Vec<f64> {
            let mut cfg = test_config(PressureVelocityCoupling::Pimple, "unused-time-refinement");
            cfg.case = Case::LidDrivenCavity(CavityCase {
                reynolds: 10.0,
                nu: 0.1,
                ..CavityCase::default()
            });
            cfg.dt = dt;
            cfg.t_end = 0.05;
            cfg.max_steps = 1000;
            cfg.pimple.time_scheme = scheme;
            cfg.pimple.outer_correctors = 4;
            cfg.pimple.pressure_correctors = 4;
            cfg.pimple.momentum_tolerance = 1e-12;
            cfg.pimple.first_pressure_relative_tolerance = 0.0;
            cfg.pressure_tolerance = 1e-12;
            cfg.pressure_max_iters = 1000;
            let mut s = IncompressibleSolver::new(cfg).unwrap();
            while !s.reached_end_time() {
                s.advance_one_step().unwrap();
            }
            assert!((s.time - 0.05).abs() < 1e-14);
            let mut values = s.u.as_slice().to_vec();
            values.extend_from_slice(s.v.as_slice());
            values
        }
        let reference = integrate(TransientTimeScheme::Backward, 0.000125);
        let error = |scheme, dt| {
            integrate(scheme, dt)
                .iter()
                .zip(&reference)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt()
        };
        let (b0, b1) = (
            error(TransientTimeScheme::Backward, 0.002),
            error(TransientTimeScheme::Backward, 0.001),
        );
        let (e0, e1) = (
            error(TransientTimeScheme::Euler, 0.002),
            error(TransientTimeScheme::Euler, 0.001),
        );
        eprintln!("time refinement: BDF2={b0:e},{b1:e}; Euler={e0:e},{e1:e}");
        assert!((2.8..5.0).contains(&(b0 / b1)), "BDF2 ratio {}", b0 / b1);
        assert!((1.7..2.3).contains(&(e0 / e1)), "Euler ratio {}", e0 / e1);
        assert!(b1 < 0.4 * e1, "BDF2={b1:e}, Euler={e1:e}");
    }

    #[test]
    fn backward_requires_nonlinear_reassembly_and_preserves_time_history() {
        let mut cfg = test_config(PressureVelocityCoupling::Pimple, "unused-backward-history");
        cfg.pimple.time_scheme = TransientTimeScheme::Backward;
        assert!(cfg.validate().is_err());
        cfg.pimple.outer_correctors = 3;
        cfg.pressure_max_iters = 1000;
        let mut solver = IncompressibleSolver::new(cfg).unwrap();
        let initial_u = solver.u.clone();
        solver.advance_one_step().unwrap();
        assert_eq!(
            solver.previous_time_velocity.as_ref().unwrap().0.as_slice(),
            initial_u.as_slice()
        );
        let step_one_u = solver.u.clone();
        solver.advance_one_step().unwrap();
        assert_eq!(
            solver.previous_time_velocity.as_ref().unwrap().0.as_slice(),
            step_one_u.as_slice()
        );
        assert!((solver.time - 2e-3).abs() < 1e-15);
    }

    #[test]
    fn transient_pressure_outlet_anchors_absolute_pressure() {
        fn solve(initial_pressure: f64, outlet: f64) -> IncompressibleSolver {
            let mut cfg = test_config(
                PressureVelocityCoupling::Pimple,
                "unused-pressure-gauge-test",
            );
            cfg.case = Case::Channel(crate::cases::ChannelCase {
                length: 2.0,
                ..Default::default()
            });
            cfg.pressure_tolerance = 1e-12;
            cfg.pressure_max_iters = 1000;
            cfg.pimple.momentum_tolerance = 1e-12;
            cfg.pimple.first_pressure_relative_tolerance = 0.0;
            cfg.pimple.outer_correctors = 2;
            cfg.pimple.pressure_correctors = 4;
            cfg.boundary_overrides
                .set_pressure_outlet(Side::Right, outlet);
            let mut s = IncompressibleSolver::new(cfg).unwrap();
            s.p.fill(initial_pressure);
            s.advance_one_step().unwrap();
            s
        }
        let zero = solve(0.0, 0.0);
        let initial_shift = solve(7.0, 0.0);
        let outlet_shift = solve(0.0, 5.0);
        let drift = max_field_difference(&zero.p, &initial_shift.p);
        eprintln!("pressure initial-gauge drift={drift:e}");
        assert!(
            drift < 1e-8,
            "fixed pressure outlet retained arbitrary initial offset: {drift}"
        );
        assert!(max_field_difference(&zero.u, &initial_shift.u) < 1e-8);
        assert!(max_field_difference(&zero.v, &initial_shift.v) < 1e-8);
        assert!(max_field_difference(&zero.u, &outlet_shift.u) < 1e-8);
        assert!(max_field_difference(&zero.v, &outlet_shift.v) < 1e-8);
        for (a, b) in zero.p.as_slice().iter().zip(outlet_shift.p.as_slice()) {
            assert!(
                (b - a - 5.0).abs() < 1e-8,
                "outlet pressure offset not applied: {b} vs {a}"
            );
        }
        assert!(zero.max_divergence() < 1e-8);
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
    #[test]
    fn implicit_normal_wall_uses_full_spacing_diffusion() {
        let directory = tempfile::tempdir().unwrap();
        let cfg = test_config(
            PressureVelocityCoupling::Pimple,
            directory.path().to_str().unwrap(),
        );
        let nu = cfg.case.kinematic_viscosity();
        let dt = cfg.dt;
        let mut solver = IncompressibleSolver::new(cfg).unwrap();
        for j in 0..8 {
            for i in 4..8 {
                solver.solid[(i, j)] = true;
            }
        }
        solver.advance().unwrap();
        let row = 3 + 9 * 3;
        let expected = solver.grid.dx * solver.grid.dy / dt
            + 2.0 * nu * (solver.grid.dy / solver.grid.dx + solver.grid.dx / solver.grid.dy);
        let matrix = &solver
            .implicit_assembly_workspace
            .as_ref()
            .unwrap()
            .u_matrix;
        assert!((matrix.get(row, row).unwrap() - expected).abs() < 1e-12);
    }

    #[test]
    fn cylinder_pressure_force_integrates_affine_pressure_on_mask_boundary() {
        let directory = tempfile::tempdir().unwrap();
        let mut cfg = test_config(
            PressureVelocityCoupling::Pimple,
            directory.path().to_str().unwrap(),
        );
        cfg.case = Case::CylinderRe100(crate::cases::CylinderCase {
            length: 1.0,
            height: 1.0,
            xc: 0.5,
            yc: 0.5,
            diameter: 0.3,
            mu: 0.003,
            nu: 0.003,
            ..Default::default()
        });
        cfg.nx = 32;
        cfg.ny = 32;
        let mut solver = IncompressibleSolver::new(cfg).unwrap();
        solver.u_cell.as_mut_slice().fill(0.0);
        solver.v_cell.as_mut_slice().fill(0.0);
        let mut area = 0.0;
        for j in 0..32 {
            for i in 0..32 {
                solver.p[(i, j)] = 17.0 + 2.0 * solver.grid.cell_x(i) - 3.0 * solver.grid.cell_y(j);
                if solver.solid[(i, j)] {
                    area += solver.grid.dx * solver.grid.dy;
                }
            }
        }
        let forces = solver.cylinder_force_coefficients();
        let q = 0.5 * 0.3;
        assert!((forces.cd_pressure + 2.0 * area / q).abs() < 1e-12);
        assert!((forces.cl_pressure - 3.0 * area / q).abs() < 1e-12);
    }

    #[test]
    fn reused_assembly_matches_fresh_workspace_on_every_step() {
        let directory = tempfile::tempdir().unwrap();
        let mut cfg = test_config(
            PressureVelocityCoupling::Pimple,
            directory.path().to_str().unwrap(),
        );
        cfg.max_steps = 4;
        cfg.t_end = 4.0 * cfg.dt;
        let mut cached = IncompressibleSolver::new(cfg.clone()).unwrap();
        let mut fresh = IncompressibleSolver::new(cfg).unwrap();
        for _ in 0..4 {
            fresh.implicit_assembly_workspace = None;
            cached.advance().unwrap();
            fresh.advance().unwrap();
            assert_eq!(cached.u.as_slice(), fresh.u.as_slice());
            assert_eq!(cached.v.as_slice(), fresh.v.as_slice());
            assert_eq!(cached.p.as_slice(), fresh.p.as_slice());
        }
    }

    #[test]
    fn pressure_correctors_agree_across_preconditioners_and_snapshot_output_is_optional() {
        let directory = tempfile::tempdir().unwrap();
        let mut reference_fields = None;
        for (index, kind) in [
            PressureSolverKind::Pcg,
            PressureSolverKind::PcgDic,
            PressureSolverKind::PcgMg,
        ]
        .into_iter()
        .enumerate()
        {
            let output = directory.path().join(index.to_string());
            let mut cfg = test_config(PressureVelocityCoupling::Pimple, output.to_str().unwrap());
            cfg.case = Case::CylinderRe100(crate::cases::CylinderCase {
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
                perturbation: 0.001,
                initial_condition: crate::cases::CylinderInitialCondition::GaussianWake,
            });
            cfg.nx = 88;
            cfg.ny = 16;
            cfg.dt = 0.001;
            cfg.t_end = 0.003;
            cfg.pressure_solver = kind;
            cfg.pressure_max_iters = 4000;
            cfg.pressure_tolerance = 1.0e-10;
            cfg.pimple.momentum_tolerance = 1.0e-10;
            cfg.pimple.pressure_correctors = 3;
            cfg.pimple.first_pressure_relative_tolerance = 0.0;
            cfg.velocity_relaxation = 1.0;
            cfg.pressure_relaxation = 1.0;
            cfg.boundary_overrides
                .set_parabolic_inlet(Side::Left, 1.5, 0.0, 0.41);
            cfg.boundary_overrides.set_velocity(Side::Top, 0.0, 0.0);
            cfg.boundary_overrides.set_velocity(Side::Bottom, 0.0, 0.0);
            let mut solver = IncompressibleSolver::new(cfg).unwrap();
            if index == 2 {
                solver.set_verification_snapshots(1, 0.002).unwrap();
            }
            let summary = solver.run().unwrap();
            assert_eq!(summary.steps, 3);
            assert!(summary.max_divergence < 1.0e-8);
            let workspace = solver.implicit_assembly_workspace.as_ref().unwrap();
            let true_residual = crate::residual(
                &workspace.pressure_matrix,
                &workspace.pressure_correction,
                &workspace.pressure_rhs,
            )
            .unwrap();
            assert!(crate::norm_l2(&true_residual).unwrap() < 2.0e-10);
            if let Some((p, u, v)) = &reference_fields {
                assert!(max_field_difference(&solver.p, p) < 1.0e-6);
                assert!(max_field_difference(&solver.u, u) < 1.0e-8);
                assert!(max_field_difference(&solver.v, v) < 1.0e-8);
            } else {
                reference_fields = Some((solver.p.clone(), solver.u.clone(), solver.v.clone()));
            }
            assert_eq!(output.join("field_snapshots").exists(), index == 2);
            if index == 2 {
                assert!(!output.join("field_snapshots/field_00000001.csv").exists());
                assert!(output.join("field_snapshots/field_00000002.csv").exists());
                assert!(output.join("field_snapshots/field_00000003.csv").exists());
            }
        }
    }
    #[test]
    fn transient_neighbour_correctors_reduce_the_corrected_momentum_defect() {
        let mut previous_residual = f64::INFINITY;
        for correctors in [1, 2, 4] {
            let mut cfg = test_config(PressureVelocityCoupling::Pimple, "target/piso-defect-test");
            cfg.case = Case::CylinderRe100(crate::cases::CylinderCase {
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
                perturbation: 0.001,
                initial_condition: crate::cases::CylinderInitialCondition::GaussianWake,
            });
            cfg.nx = 88;
            cfg.ny = 16;
            cfg.dt = 0.01;
            cfg.t_end = 0.01;
            cfg.pressure_solver = PressureSolverKind::PcgMg;
            cfg.pressure_max_iters = 2000;
            cfg.pressure_tolerance = 1.0e-10;
            cfg.pimple.momentum_tolerance = 1.0e-10;
            cfg.pimple.pressure_correctors = correctors;
            cfg.pimple.first_pressure_relative_tolerance = 0.0;
            cfg.boundary_overrides
                .set_parabolic_inlet(Side::Left, 1.5, 0.0, 0.41);
            cfg.boundary_overrides.set_velocity(Side::Top, 0.0, 0.0);
            cfg.boundary_overrides.set_velocity(Side::Bottom, 0.0, 0.0);
            let mut solver = IncompressibleSolver::new(cfg).unwrap();
            let diag = solver.advance_one_step().unwrap();
            let residual = diag.momentum_equation_residual.unwrap();
            assert!(
                residual < previous_residual,
                "{correctors} correctors: {residual} >= {previous_residual}"
            );
            assert!(diag.max_divergence < 1.0e-8);
            previous_residual = residual;
        }
    }
    #[test]
    fn implicit_outlet_preserves_neumann_momentum_coefficients() {
        let mut cfg = test_config(
            PressureVelocityCoupling::Pimple,
            "target/outlet-neumann-test",
        );
        cfg.case = Case::Channel(crate::cases::ChannelCase {
            length: 2.2,
            height: 0.41,
            rho: 1.0,
            u_mean: 1.0,
            reynolds: 100.0,
            nu: 0.001,
        });
        cfg.pressure_solver = PressureSolverKind::PcgMg;
        cfg.pressure_max_iters = 1000;
        let mut solver = IncompressibleSolver::new(cfg).unwrap();
        solver.u.fill(1.0);
        solver.v.fill(0.25);
        let old_u = solver.u.clone();
        let old_v = solver.v.clone();
        let dt = solver.cfg.dt;
        solver.advance_implicit_pimple(dt, &old_u, &old_v).unwrap();
        let workspace = solver.implicit_assembly_workspace.as_ref().unwrap();
        let nx = solver.grid.nx;
        let j = solver.grid.ny / 2;
        let u_outlet = nx + (nx + 1) * j;
        assert_eq!(workspace.u_matrix.get(u_outlet, u_outlet), Some(1.0));
        assert_eq!(workspace.u_matrix.get(u_outlet, u_outlet - 1), Some(-1.0));
        assert_eq!(workspace.u_pressure_free_rhs[u_outlet], 0.0);
        let row = nx - 1 + nx * j;
        let begin = workspace.v_matrix.row_offsets()[row];
        let end = workspace.v_matrix.row_offsets()[row + 1];
        let row_sum: f64 = workspace.v_matrix.values()[begin..end].iter().sum();
        // A constant transverse field has zero diffusion/convection divergence
        // at a zero-gradient outlet. Only the time derivative remains.
        let transient = solver.grid.dx * solver.grid.dy / dt;
        assert!(
            (row_sum - transient).abs() < 1.0e-10,
            "{row_sum} != {transient}"
        );
    }
}
