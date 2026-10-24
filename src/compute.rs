//! Per-solve execution policy and counters shared by the private Rayon pool.
//! No process-global solver configuration: concurrent analyses remain isolated.
use serde::{Deserialize, Serialize};
use std::cell::{Cell, RefCell};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Instant;
#[cfg(feature = "gpu-compute")]
mod amg;
#[cfg(feature = "gpu-compute")]
mod gpu;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Backend {
    Serial,
    #[default]
    Parallel,
    Gpu,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum GpuPreconditioner {
    #[default]
    Auto,
    Jacobi,
}
pub type LinearControl = Arc<dyn Fn() -> bool + Send + Sync>;
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ComputeOptions {
    pub backend: Backend,
    /// Case-insensitive adapter-name substring; empty selects a hardware adapter.
    pub adapter: String,
    pub cpu_fallback: bool,
    pub parallel_min_rows: usize,
    pub gpu_min_rows: usize,
    /// Fraction of adapter max_buffer_size admitted for estimated working storage.
    pub gpu_memory_fraction: f64,
    pub gpu_preconditioner: GpuPreconditioner,
    pub gpu_restart: usize,
    pub gpu_inner_iterations: usize,
    pub gpu_smoothing_sweeps: usize,
    pub gpu_timeout_ms: u64,
    pub gpu_host_basis_mb: usize,
}
impl Default for ComputeOptions {
    fn default() -> Self {
        Self {
            backend: Backend::Parallel,
            adapter: String::new(),
            cpu_fallback: true,
            parallel_min_rows: 4096,
            gpu_min_rows: 16384,
            gpu_memory_fraction: 0.8,
            gpu_preconditioner: GpuPreconditioner::Auto,
            gpu_restart: 24,
            gpu_inner_iterations: 8,
            gpu_smoothing_sweeps: 2,
            gpu_timeout_ms: 10000,
            gpu_host_basis_mb: 256,
        }
    }
}
impl ComputeOptions {
    pub fn validate(&self) -> Result<(), String> {
        if !(self.gpu_memory_fraction.is_finite()
            && self.gpu_memory_fraction > 0.0
            && self.gpu_memory_fraction <= 1.0)
        {
            return Err("GPU memory fraction must be in (0,1]".into());
        }
        if self.parallel_min_rows == 0 || self.gpu_min_rows == 0 {
            return Err("Compute row thresholds must be positive".into());
        }
        if !(16..=16384).contains(&self.gpu_host_basis_mb) {
            return Err("FGMRES basis budget must be in 16..16384 MiB".into());
        }
        if !(2..=64).contains(&self.gpu_restart)
            || !(1..=64).contains(&self.gpu_inner_iterations)
            || !(1..=8).contains(&self.gpu_smoothing_sweeps)
            || !(100..=60000).contains(&self.gpu_timeout_ms)
        {
            return Err("GPU controls require restart 2..64, inner iterations 1..64, smoothing 1..8 and timeout 100..60000 ms".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct DeviceInfo {
    pub name: String,
    pub backend: String,
    pub device_type: String,
    pub max_buffer_bytes: u64,
    pub max_storage_binding_bytes: u64,
    pub shader_f64: bool,
}
#[derive(Clone, Debug, Default, Serialize)]
pub struct PerformanceReport {
    pub requested_backend: String,
    pub actual_backend: String,
    pub device: Option<DeviceInfo>,
    pub precision: String,
    pub cpu_threads: usize,
    pub elapsed_seconds: f64,
    pub cell_iterations_per_second: f64,
    pub spmv_calls: u64,
    pub spmv_seconds: f64,
    pub linear_solves: u64,
    pub linear_seconds: f64,
    pub gpu_solves: u64,
    pub gpu_seconds: f64,
    pub gpu_peak_working_bytes: u64,
    pub cpu_fallbacks: u64,
    pub fallback_reasons: Vec<String>,
    pub stages: std::collections::BTreeMap<String, f64>,
    pub gpu_methods: std::collections::BTreeMap<String, u64>,
    pub gpu_preconditioner_iterations: u64,
    pub gpu_amg_v_cycles: u64,
    pub gpu_host_basis_peak_bytes: u64,
    pub gpu_notes: Vec<String>,
}
#[derive(Default)]
struct Counters {
    stages: [AtomicU64; 5],
    spmv: AtomicU64,
    spmv_ns: AtomicU64,
    linear: AtomicU64,
    linear_ns: AtomicU64,
    gpu: AtomicU64,
    gpu_ns: AtomicU64,
    peak: AtomicU64,
    fallbacks: AtomicU64,
    reasons: Mutex<Vec<String>>,
}
pub struct Execution {
    pub options: ComputeOptions,
    counters: Counters,
    started: Instant,
    control: Option<LinearControl>,
    interrupted: std::sync::atomic::AtomicBool,
    #[cfg(feature = "gpu-compute")]
    gpu: Option<Mutex<gpu::GpuSolver>>,
}
thread_local! { static GPU_SUPPRESSED: Cell<bool> = const { Cell::new(false) }; static ACTIVE: RefCell<Option<Arc<Execution>>> = const { RefCell::new(None) }; }
pub fn install(runtime: Option<Arc<Execution>>) {
    ACTIVE.with(|s| *s.borrow_mut() = runtime);
}
pub fn current() -> Option<Arc<Execution>> {
    ACTIVE.with(|s| s.borrow().clone())
}
pub(crate) struct CpuLinearScope(bool);
impl CpuLinearScope {
    pub fn enter() -> Self {
        Self(GPU_SUPPRESSED.with(|s| s.replace(true)))
    }
}
impl Drop for CpuLinearScope {
    fn drop(&mut self) {
        GPU_SUPPRESSED.with(|s| s.set(self.0));
    }
}
pub struct Scope(Option<Arc<Execution>>);
impl Scope {
    pub fn enter(runtime: Arc<Execution>) -> Self {
        let old = current();
        install(Some(runtime));
        Self(old)
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        install(self.0.take());
    }
}
pub fn parallel_rows(rows: usize) -> bool {
    rows >= current().map_or(4096, |r| r.options.parallel_min_rows)
        && rayon::current_num_threads() > 1
        && current().is_none_or(|r| r.options.backend != Backend::Serial)
}
#[derive(Clone, Copy)]
pub enum Stage {
    MomentumAssembly = 0,
    PressureAssembly = 1,
    PreconditionerSetup = 2,
    PreconditionerApply = 3,
    MonitorOutput = 4,
}
pub struct StageTimer {
    runtime: Option<Arc<Execution>>,
    stage: Stage,
    start: Instant,
}
impl StageTimer {
    pub fn new(stage: Stage) -> Self {
        Self {
            runtime: current(),
            stage,
            start: Instant::now(),
        }
    }
}
impl Drop for StageTimer {
    fn drop(&mut self) {
        if let Some(r) = &self.runtime {
            r.counters.stages[self.stage as usize].fetch_add(
                self.start.elapsed().as_nanos().min(u64::MAX as u128) as u64,
                Ordering::Relaxed,
            );
        }
    }
}
/// Independent-output kernels; face scattering is deliberately not parallelized.
pub fn try_for_each<T: Send, E: Send>(
    values: &mut [T],
    f: impl Fn(usize, &mut T) -> Result<(), E> + Sync + Send,
) -> Result<(), E> {
    use rayon::prelude::*;
    if parallel_rows(values.len()) {
        values
            .par_iter_mut()
            .enumerate()
            .try_for_each(|(i, v)| f(i, v))
    } else {
        values.iter_mut().enumerate().try_for_each(|(i, v)| f(i, v))
    }
}
pub struct Timer {
    runtime: Option<Arc<Execution>>,
    start: Instant,
    linear: bool,
}
impl Timer {
    pub fn spmv() -> Self {
        Self {
            runtime: current(),
            start: Instant::now(),
            linear: false,
        }
    }
    pub fn linear() -> Self {
        Self {
            runtime: current(),
            start: Instant::now(),
            linear: true,
        }
    }
}
impl Drop for Timer {
    fn drop(&mut self) {
        if let Some(r) = &self.runtime {
            let ns = self.start.elapsed().as_nanos().min(u64::MAX as u128) as u64;
            let (calls, time) = if self.linear {
                (&r.counters.linear, &r.counters.linear_ns)
            } else {
                (&r.counters.spmv, &r.counters.spmv_ns)
            };
            calls.fetch_add(1, Ordering::Relaxed);
            time.fetch_add(ns, Ordering::Relaxed);
        }
    }
}
impl Execution {
    pub fn new(options: ComputeOptions) -> Result<Arc<Self>, String> {
        Self::controlled(options, None)
    }
    pub fn controlled(
        options: ComputeOptions,
        control: Option<LinearControl>,
    ) -> Result<Arc<Self>, String> {
        options.validate()?;
        #[cfg(feature = "gpu-compute")]
        let (gpu, error) = if options.backend == Backend::Gpu {
            match gpu::GpuSolver::new(&options.adapter) {
                Ok(g) => (Some(Mutex::new(g)), None),
                Err(e) => (None, Some(e)),
            }
        } else {
            (None, None)
        };
        #[cfg(not(feature = "gpu-compute"))]
        let error = (options.backend == Backend::Gpu)
            .then(|| "GPU compute was not compiled; build with --features gpu-compute".to_string());
        if !options.cpu_fallback {
            if let Some(e) = &error {
                return Err(e.clone());
            }
        }
        let runtime = Arc::new(Self {
            options,
            counters: Default::default(),
            started: Instant::now(),
            control,
            interrupted: std::sync::atomic::AtomicBool::new(false),
            #[cfg(feature = "gpu-compute")]
            gpu,
        });
        if let Some(e) = error {
            runtime.fallback(&e);
        }
        Ok(runtime)
    }
    fn fallback(&self, reason: &str) {
        self.counters.fallbacks.fetch_add(1, Ordering::Relaxed);
        let mut reasons = self
            .counters
            .reasons
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        if !reasons.iter().any(|s| s == reason) && reasons.len() < 16 {
            reasons.push(reason.into());
        }
    }
    pub fn gpu_enabled(&self) -> bool {
        #[cfg(feature = "gpu-compute")]
        {
            self.gpu.is_some()
        }
        #[cfg(not(feature = "gpu-compute"))]
        {
            false
        }
    }
    pub fn cancelled(&self) -> bool {
        self.interrupted.load(Ordering::Acquire)
    }
    pub fn report(&self, threads: usize, cells: usize, iterations: usize) -> PerformanceReport {
        let load = |a: &AtomicU64| a.load(Ordering::Relaxed);
        let elapsed = self.started.elapsed().as_secs_f64();
        #[cfg(feature = "gpu-compute")]
        let device = self
            .gpu
            .as_ref()
            .and_then(|g| g.lock().ok().map(|g| g.info.clone()));
        #[cfg(not(feature = "gpu-compute"))]
        let device = None;
        #[cfg(feature = "gpu-compute")]
        let (methods, inner_iterations, cycles, host_basis, notes) = self
            .gpu
            .as_ref()
            .and_then(|g| {
                g.lock().ok().map(|g| {
                    (
                        g.methods.clone(),
                        g.inner_iterations,
                        g.v_cycles,
                        g.host_basis_peak,
                        g.notes.clone(),
                    )
                })
            })
            .unwrap_or_default();
        #[cfg(not(feature = "gpu-compute"))]
        let (methods, inner_iterations, cycles, host_basis, notes): (
            std::collections::BTreeMap<String, u64>,
            u64,
            u64,
            u64,
            Vec<String>,
        ) = (Default::default(), 0, 0, 0, Vec::new());
        let used = load(&self.counters.gpu) > 0;
        let attempted = !methods.is_empty();
        PerformanceReport {
            gpu_methods: methods,
            gpu_preconditioner_iterations: inner_iterations,
            gpu_amg_v_cycles: cycles,
            gpu_host_basis_peak_bytes: host_basis,
            gpu_notes: notes,
            requested_backend: format!("{:?}", self.options.backend),
            actual_backend: if used {
                "GPU preconditioning / CPU f64 FGMRES + assembly".into()
            } else if attempted {
                if self.options.cpu_fallback {
                    "CPU fallback after GPU attempts".into()
                } else {
                    "GPU attempt failed; CPU fallback disabled".into()
                }
            } else if self.options.backend == Backend::Gpu && !self.options.cpu_fallback {
                "CPU checks only; no GPU work completed".into()
            } else if threads == 1 {
                "CPU serial".into()
            } else {
                "CPU parallel".into()
            },
            device,
            precision: if used {
                "f32 GPU AMG/Krylov; f64 FGMRES, true residual and fields".into()
            } else {
                "f64".into()
            },
            cpu_threads: threads,
            elapsed_seconds: elapsed,
            cell_iterations_per_second: cells as f64 * iterations as f64
                / elapsed.max(f64::MIN_POSITIVE),
            spmv_calls: load(&self.counters.spmv),
            spmv_seconds: load(&self.counters.spmv_ns) as f64 * 1e-9,
            linear_solves: load(&self.counters.linear),
            linear_seconds: load(&self.counters.linear_ns) as f64 * 1e-9,
            gpu_solves: load(&self.counters.gpu),
            gpu_seconds: load(&self.counters.gpu_ns) as f64 * 1e-9,
            gpu_peak_working_bytes: load(&self.counters.peak),
            stages: [
                "momentum_assembly",
                "pressure_assembly",
                "preconditioner_setup",
                "preconditioner_apply",
                "monitor_output",
            ]
            .into_iter()
            .enumerate()
            .map(|(i, name)| (name.into(), load(&self.counters.stages[i]) as f64 * 1e-9))
            .collect(),
            cpu_fallbacks: load(&self.counters.fallbacks),
            fallback_reasons: self
                .counters
                .reasons
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone(),
        }
    }
}
pub fn devices() -> Result<Vec<DeviceInfo>, String> {
    #[cfg(feature = "gpu-compute")]
    {
        gpu::devices()
    }
    #[cfg(not(feature = "gpu-compute"))]
    {
        Err("Build with --features gpu-compute to enumerate adapters".into())
    }
}
/// GPU is an explicitly selected mixed-precision linear backend. Never accept a
/// correction using an f32 residual alone. CPU fallback retains caller's x.
pub(crate) fn try_gpu(
    matrix: &crate::CsrMatrix,
    rhs: &[f64],
    x: &mut [f64],
    options: crate::LinearSolverOptions,
) -> Option<Result<crate::LinearSolveReport, crate::LinearAlgebraError>> {
    if GPU_SUPPRESSED.with(Cell::get) {
        return None;
    }
    let runtime = current()?;
    if runtime.options.backend != Backend::Gpu {
        return None;
    }
    let fail = |reason: String| {
        if runtime.options.cpu_fallback {
            runtime.fallback(&reason);
            None
        } else {
            Some(Err(crate::LinearAlgebraError::Backend { message: reason }))
        }
    };
    if let Err(e) = checkpoint() {
        return Some(Err(e));
    }
    if runtime.options.cpu_fallback && matrix.nrows() < runtime.options.gpu_min_rows {
        return fail("Matrix below GPU row threshold".into());
    }
    #[cfg(feature = "gpu-compute")]
    {
        let Some(gpu) = &runtime.gpu else {
            return if runtime.options.cpu_fallback {
                None
            } else {
                fail("GPU device unavailable".into())
            };
        };
        let _linear = Timer::linear();
        let start = Instant::now();
        let mut gpu = match gpu.lock() {
            Ok(g) => g,
            Err(_) => return fail("GPU worker lock poisoned".into()),
        };
        let work_before = (gpu.inner_iterations, gpu.v_cycles);
        let result = gpu.checked_solve(matrix, rhs, x, options, &runtime.options);
        runtime.counters.gpu_ns.fetch_add(
            start.elapsed().as_nanos().min(u64::MAX as u128) as u64,
            Ordering::Relaxed,
        );
        runtime
            .counters
            .peak
            .fetch_max(gpu.peak_bytes, Ordering::Relaxed);
        match result {
            Ok(report) => {
                if work_before != (gpu.inner_iterations, gpu.v_cycles) {
                    runtime.counters.gpu.fetch_add(1, Ordering::Relaxed);
                }
                Some(Ok(report))
            }
            Err(e) => {
                if runtime.cancelled() {
                    Some(Err(crate::LinearAlgebraError::Backend { message: e }))
                } else {
                    fail(e)
                }
            }
        }
    }
    #[cfg(not(feature = "gpu-compute"))]
    {
        let _ = (rhs, x, options);
        None
    }
}

/// Cooperative pause/cancel boundary used by CPU Krylov iterations and GPU
/// synchronization. The worker supplies an owned callback, never a borrowed UI.
pub fn checkpoint() -> Result<(), crate::LinearAlgebraError> {
    if let Some(runtime) = current() {
        if runtime.control.as_ref().is_some_and(|control| control()) {
            runtime.interrupted.store(true, Ordering::Release);
            return Err(crate::LinearAlgebraError::Backend {
                message: "Solve cancelled during linear computation".into(),
            });
        }
    }
    Ok(())
}
