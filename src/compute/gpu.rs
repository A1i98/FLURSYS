//! GPU AMG/Krylov preconditioning with an f64 flexible-GMRES outer solve.
//! CSR and ten Krylov vectors stay on device throughout each correction solve.
use super::DeviceInfo;
use super::{amg::Amg, ComputeOptions, GpuPreconditioner};
use crate::{CsrMatrix, LinearSolveReport, LinearSolverOptions};
use egui_wgpu::wgpu;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use wgpu::util::DeviceExt;
fn instance() -> wgpu::Instance {
    wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env())
}
fn info(a: &wgpu::Adapter) -> DeviceInfo {
    let i = a.get_info();
    let l = a.limits();
    DeviceInfo {
        name: i.name,
        backend: format!("{:?}", i.backend),
        device_type: format!("{:?}", i.device_type),
        max_buffer_bytes: l.max_buffer_size,
        max_storage_binding_bytes: u64::from(l.max_storage_buffer_binding_size),
        shader_f64: a.features().contains(wgpu::Features::SHADER_F64),
    }
}
pub fn devices() -> Result<Vec<DeviceInfo>, String> {
    Ok(
        pollster::block_on(instance().enumerate_adapters(wgpu::Backends::all()))
            .iter()
            .map(info)
            .collect(),
    )
}
pub(super) fn ubytes(v: impl IntoIterator<Item = u32>) -> Vec<u8> {
    v.into_iter().flat_map(u32::to_le_bytes).collect()
}
pub(super) fn fbytes(v: impl IntoIterator<Item = f32>) -> Vec<u8> {
    v.into_iter().flat_map(f32::to_le_bytes).collect()
}
struct Storage {
    bytes: u64,
    cached_matrix: Option<CsrMatrix>,
    row_scales: Arc<Vec<f64>>,
    n: usize,
    nnz: usize,
    rows: wgpu::Buffer,
    cols: wgpu::Buffer,
    values: wgpu::Buffer,
    state: wgpu::Buffer,
    partial: wgpu::Buffer,
    read: wgpu::Buffer,
    bind: wgpu::BindGroup,
}
pub struct GpuSolver {
    pub info: DeviceInfo,
    pub peak_bytes: u64,
    pub methods: BTreeMap<String, u64>,
    pub inner_iterations: u64,
    pub v_cycles: u64,
    pub host_basis_peak: u64,
    pub notes: Vec<String>,
    amg: Option<Amg>,
    timeout: Duration,
    driver_error: Arc<Mutex<Option<String>>>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    params: wgpu::Buffer,
    storage: Option<Storage>,
}
impl GpuSolver {
    pub fn new(selector: &str) -> Result<Self, String> {
        let adapters = pollster::block_on(instance().enumerate_adapters(wgpu::Backends::all()));
        let selector = selector.to_lowercase();
        let adapter = adapters
            .into_iter()
            .filter(|a| {
                let i = a.get_info();
                if selector.is_empty() {
                    matches!(
                        i.device_type,
                        wgpu::DeviceType::DiscreteGpu | wgpu::DeviceType::IntegratedGpu
                    )
                } else {
                    i.name.to_lowercase().contains(&selector)
                }
            })
            .min_by_key(|a| match a.get_info().device_type {
                wgpu::DeviceType::DiscreteGpu => 0,
                wgpu::DeviceType::IntegratedGpu => 1,
                _ => 2,
            })
            .ok_or("No matching hardware GPU adapter")?;
        let info = info(&adapter);
        let limits = adapter.limits();
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("FLURSYS linear compute"),
            required_limits: limits,
            ..Default::default()
        }))
        .map_err(|e| e.to_string())?;
        let driver_error = Arc::new(Mutex::new(None));
        let lost = driver_error.clone();
        device.set_device_lost_callback(move |reason, message| {
            if let Ok(mut error) = lost.lock() {
                *error = Some(format!("GPU device lost ({reason:?}): {message}"));
            }
        });
        let scopes = [
            wgpu::ErrorFilter::OutOfMemory,
            wgpu::ErrorFilter::Internal,
            wgpu::ErrorFilter::Validation,
        ]
        .map(|f| device.push_error_scope(f));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("FLURSYS CSR Krylov"),
            source: wgpu::ShaderSource::Wgsl(include_str!("linear.wgsl").into()),
        });
        let entries = (0..6)
            .map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if binding == 0 {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage {
                            read_only: binding < 4,
                        }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect::<Vec<_>>();
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("CFD compute buffers"),
            entries: &entries,
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("CFD mixed-precision BiCGSTAB"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("main"),
            compilation_options: Default::default(),
            cache: None,
        });
        let mut failed = None;
        for scope in scopes.into_iter().rev() {
            if let Some(e) = pollster::block_on(scope.pop()) {
                failed.get_or_insert(e.to_string());
            }
        }
        if let Some(e) = failed {
            return Err(e);
        }
        let params = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("CFD dispatch parameters"),
            contents: &[0; 32],
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        Ok(Self {
            info,
            peak_bytes: 0,
            methods: Default::default(),
            inner_iterations: 0,
            v_cycles: 0,
            host_basis_peak: 0,
            notes: Vec::new(),
            amg: None,
            timeout: Duration::from_secs(10),
            driver_error,
            device,
            queue,
            pipeline,
            layout,
            params,
            storage: None,
        })
    }
    fn storage(&mut self, n: usize, nnz: usize, fraction: f64) -> Result<(), String> {
        let n = u32::try_from(n).map_err(|_| "GPU matrix exceeds u32 indexing")? as usize;
        u32::try_from(nnz).map_err(|_| "GPU matrix exceeds u32 CSR indexing")?;
        let lengths = [
            (n as u64 + 1) * 4,
            nnz as u64 * 4,
            nnz as u64 * 4,
            n as u64 * 40,
            n.div_ceil(256) as u64 * 4,
            (n as u64 * 4).max(n.div_ceil(256) as u64 * 4),
        ];
        let estimated = lengths.iter().sum::<u64>() + 32;
        if self.amg.as_ref().is_some_and(|amg| {
            amg.bytes + estimated > (self.info.max_buffer_bytes as f64 * fraction) as u64
        }) {
            self.amg = None;
        }
        // Adapter limits are portable resource limits, not reported free VRAM.
        if estimated as f64 > self.info.max_buffer_bytes as f64 * fraction {
            return Err(format!(
                "GPU working set {estimated} bytes exceeds configured resource budget"
            ));
        }
        if lengths[..5]
            .iter()
            .any(|&n| n > self.info.max_storage_binding_bytes)
            || lengths.iter().any(|&n| n > self.info.max_buffer_bytes)
        {
            return Err("GPU buffer binding limit exceeded".into());
        }
        if n.div_ceil(256) > self.device.limits().max_compute_workgroups_per_dimension as usize {
            return Err("GPU dispatch dimension limit exceeded".into());
        }
        self.peak_bytes = self.peak_bytes.max(estimated);
        if self
            .storage
            .as_ref()
            .is_some_and(|s| s.n == n && s.nnz == nnz)
        {
            return Ok(());
        }
        let storage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
        let buffer = |label: &str, size: u64, usage: wgpu::BufferUsages| {
            self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: size.max(4),
                usage,
                mapped_at_creation: false,
            })
        };
        let rows = buffer("CSR row offsets", lengths[0], storage);
        let cols = buffer("CSR column indices", lengths[1], storage);
        let values = buffer("CSR values", lengths[2], storage);
        let state = buffer(
            "Resident Krylov vectors",
            lengths[3],
            storage | wgpu::BufferUsages::COPY_SRC,
        );
        let partial = buffer(
            "Dot partial sums",
            lengths[4],
            storage | wgpu::BufferUsages::COPY_SRC,
        );
        let read = buffer(
            "CFD scalar/readback staging",
            lengths[5],
            wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        );
        let entries = [&self.params, &rows, &cols, &values, &state, &partial]
            .into_iter()
            .enumerate()
            .map(|(i, b)| wgpu::BindGroupEntry {
                binding: i as u32,
                resource: b.as_entire_binding(),
            })
            .collect::<Vec<_>>();
        let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("CFD linear system"),
            layout: &self.layout,
            entries: &entries,
        });
        self.storage = Some(Storage {
            bytes: estimated,
            cached_matrix: None,
            row_scales: Arc::new(Vec::new()),
            n,
            nnz,
            rows,
            cols,
            values,
            state,
            partial,
            read,
            bind,
        });
        Ok(())
    }
    fn dispatch(&self, op: u32, a: u32, b: u32, alpha: f64, beta: f64, omega: f64) {
        let s = self.storage.as_ref().unwrap();
        let mut bytes = ubytes([s.n as u32, op, a, b]);
        bytes.extend(fbytes([alpha as f32, beta as f32, omega as f32, 0.]));
        self.queue.write_buffer(&self.params, 0, &bytes);
        let mut encoder = self.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &s.bind, &[]);
            pass.dispatch_workgroups(s.n.div_ceil(256) as u32, 1, 1);
        }
        self.queue.submit([encoder.finish()]);
    }
    fn read(&self, source: &wgpu::Buffer, bytes: u64) -> Result<Vec<f32>, String> {
        let s = self
            .storage
            .as_ref()
            .ok_or("GPU staging buffer unavailable")?;
        let mut encoder = self.device.create_command_encoder(&Default::default());
        encoder.copy_buffer_to_buffer(source, 0, &s.read, 0, bytes);
        self.queue.submit([encoder.finish()]);
        let slice = s.read.slice(0..bytes);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
        struct Unmap<'a>(&'a wgpu::Buffer);
        impl Drop for Unmap<'_> {
            fn drop(&mut self) {
                self.0.unmap();
            }
        }
        let _unmap = Unmap(&s.read);
        let mut waited = Duration::ZERO;
        loop {
            super::checkpoint().map_err(|e| e.to_string())?;
            if let Some(e) = self.driver_error.lock().map_err(|e| e.to_string())?.clone() {
                return Err(e);
            }
            let tick = Instant::now();
            self.device
                .poll(wgpu::PollType::Poll)
                .map_err(|e| e.to_string())?;
            match rx.try_recv() {
                Ok(result) => {
                    result.map_err(|e| e.to_string())?;
                    break;
                }
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    return Err("GPU mapping callback disconnected".into())
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {}
            }
            std::thread::sleep(Duration::from_millis(1));
            waited += tick.elapsed();
            if waited >= self.timeout {
                let message = format!(
                    "GPU disabled for this run after readback timeout ({} ms)",
                    self.timeout.as_millis()
                );
                if let Ok(mut failed) = self.driver_error.lock() {
                    *failed = Some(message.clone());
                }
                return Err(message);
            }
        }
        let data = slice.get_mapped_range().map_err(|e| e.to_string())?;
        Ok(data
            .chunks_exact(4)
            .map(|bytes| f32::from_le_bytes(bytes.try_into().unwrap()))
            .collect())
    }
    fn dot(&self, a: u32, b: u32) -> Result<f64, String> {
        self.dispatch(7, a, b, 0., 0., 0.);
        let s = self.storage.as_ref().unwrap();
        let value = self
            .read(&s.partial, s.n.div_ceil(256) as u64 * 4)?
            .iter()
            .map(|&v| v as f64)
            .sum::<f64>();
        if value.is_finite() {
            Ok(value)
        } else {
            Err("Non-finite GPU dot product".into())
        }
    }
    fn correction(
        &self,
        rhs: &[f64],
        tolerance: f64,
        max: usize,
    ) -> Result<(Vec<f32>, usize), String> {
        let s = self.storage.as_ref().unwrap();
        self.queue.write_buffer(
            &s.state,
            s.n as u64 * 4,
            &fbytes(rhs.iter().map(|&v| v as f32)),
        );
        self.dispatch(0, 0, 0, 0., 0., 0.);
        let initial = self.dot(1, 1)?.sqrt();
        let target = (initial * tolerance).max(1e-12);
        let mut rho_old = 1.;
        let mut alpha = 1.;
        let mut omega = 1.;
        let mut iterations = 0;
        for k in 0..max {
            super::checkpoint().map_err(|e| e.to_string())?;
            let rho = self.dot(2, 1)?;
            if rho.abs() < 1e-30 {
                return Err("GPU BiCGSTAB rho breakdown".into());
            }
            let beta = if k == 0 {
                0.
            } else {
                (rho / rho_old) * (alpha / omega)
            };
            if !beta.is_finite() {
                return Err("GPU BiCGSTAB beta breakdown".into());
            }
            self.dispatch(1, 0, 0, 0., beta, omega);
            self.dispatch(2, 0, 0, 0., 0., 0.);
            let denominator = self.dot(2, 4)?;
            if denominator.abs() < 1e-30 {
                return Err("GPU BiCGSTAB alpha breakdown".into());
            }
            alpha = rho / denominator;
            self.dispatch(3, 0, 0, alpha, 0., 0.);
            iterations = k + 1;
            if self.dot(5, 5)?.sqrt() <= target {
                self.dispatch(6, 0, 0, alpha, 0., 0.);
                break;
            }
            self.dispatch(4, 0, 0, 0., 0., 0.);
            let tt = self.dot(6, 6)?;
            if tt.abs() < 1e-30 {
                return Err("GPU BiCGSTAB t breakdown".into());
            }
            omega = self.dot(6, 5)? / tt;
            if !omega.is_finite() || omega.abs() < 1e-30 {
                return Err("GPU BiCGSTAB omega breakdown".into());
            }
            self.dispatch(5, 0, 0, alpha, 0., omega);
            if self.dot(1, 1)?.sqrt() <= target {
                break;
            }
            rho_old = rho;
        }
        Ok((self.read(&s.state, s.n as u64 * 4)?, iterations))
    }
    fn note(&mut self, note: String) {
        if self.notes.len() < 16 && !self.notes.contains(&note) {
            self.notes.push(note);
        }
    }
    fn smooth_correction(&self, input: &[f64], steps: usize) -> Result<Vec<f32>, String> {
        let s = self.storage.as_ref().ok_or("GPU storage unavailable")?;
        self.queue.write_buffer(
            &s.state,
            s.n as u64 * 4,
            &fbytes(input.iter().map(|&v| v as f32)),
        );
        self.dispatch(0, 0, 0, 0., 0., 0.);
        for _ in 0..steps {
            super::checkpoint().map_err(|e| e.to_string())?;
            self.dispatch(8, 0, 0, 0., 0., 0.);
            self.dispatch(9, 0, 0, 0., 0., 0.);
        }
        self.read(&s.state, s.n as u64 * 4)
    }
    pub fn checked_solve(
        &mut self,
        matrix: &CsrMatrix,
        rhs: &[f64],
        x: &mut [f64],
        options: LinearSolverOptions,
        config: &ComputeOptions,
    ) -> Result<LinearSolveReport, String> {
        super::checkpoint().map_err(|e| e.to_string())?;
        if let Some(error) = self.driver_error.lock().map_err(|e| e.to_string())?.clone() {
            return Err(error);
        }
        self.timeout = Duration::from_millis(config.gpu_timeout_ms);
        let scopes = [
            wgpu::ErrorFilter::OutOfMemory,
            wgpu::ErrorFilter::Internal,
            wgpu::ErrorFilter::Validation,
        ]
        .map(|f| self.device.push_error_scope(f));
        let mut candidate = x.to_vec();
        let mut result = self.solve(matrix, rhs, &mut candidate, options, config);
        for scope in scopes.into_iter().rev() {
            if let Some(error) = pollster::block_on(scope.pop()) {
                let error = error.to_string();
                if let Ok(mut failed) = self.driver_error.lock() {
                    *failed = Some(format!(
                        "GPU disabled for this run after driver error: {error}"
                    ));
                }
                result = Err(error);
            }
        }
        if result.as_ref().is_ok_and(|report| report.converged()) {
            x.copy_from_slice(&candidate);
        }
        result
    }
    fn solve(
        &mut self,
        matrix: &CsrMatrix,
        rhs: &[f64],
        x: &mut [f64],
        options: LinearSolverOptions,
        config: &ComputeOptions,
    ) -> Result<LinearSolveReport, String> {
        let n = matrix.nrows();
        if n == 0 {
            return Err("Empty GPU system".into());
        }
        let residual = crate::linear::residual(matrix, x, rhs).map_err(|e| e.to_string())?;
        let initial = crate::linear::norm_l2(&residual).map_err(|e| e.to_string())?;
        if initial
            <= options
                .absolute_tolerance
                .max(options.relative_tolerance * initial)
        {
            self.note("Already-converged linear systems skip GPU dispatch".into());
            return Ok(LinearSolveReport {
                status: crate::LinearSolverStatus::Converged,
                iterations: 0,
                initial_residual: initial,
                final_residual: initial,
                relative_residual: if initial > 0. { 1. } else { 0. },
            });
        }
        drop(residual);
        let basis_budget = config.gpu_host_basis_mb as u64 * 1048576;
        let fitting = ((basis_budget / (n as u64 * 8)).saturating_sub(1) / 2) as usize;
        if fitting < 2 {
            return Err("FGMRES basis budget is too small for this matrix; increase CPU basis memory budget".into());
        }
        let restart = config.gpu_restart.min(fitting);
        self.host_basis_peak = self
            .host_basis_peak
            .max((2 * restart + 1) as u64 * n as u64 * 8);
        if restart < config.gpu_restart {
            self.note(format!(
                "FGMRES restart limited to {restart} by CPU basis memory budget"
            ));
        }
        self.storage(n, matrix.nnz(), config.gpu_memory_fraction)?;
        let unchanged = self
            .storage
            .as_ref()
            .and_then(|s| s.cached_matrix.as_ref())
            .is_some_and(|old| old == matrix);
        if !unchanged {
            let mut scales = Vec::with_capacity(n);
            let mut coefficients = Vec::with_capacity(matrix.nnz());
            for row in 0..n {
                let range = matrix.row_offsets()[row]..matrix.row_offsets()[row + 1];
                let scale = matrix.values()[range.clone()]
                    .iter()
                    .fold(0_f64, |m, v| m.max(v.abs()));
                if !scale.is_finite() || scale == 0. {
                    return Err("GPU matrix has empty/non-finite row".into());
                }
                let diagonal = matrix.get(row, row).ok_or("GPU Jacobi requires diagonal")? / scale;
                if (diagonal as f32).abs() < 1e-30 {
                    return Err("GPU Jacobi diagonal below f32 range".into());
                }
                scales.push(scale);
                coefficients.extend(
                    matrix.values()[range]
                        .iter()
                        .map(|&value| (value / scale) as f32),
                );
            }
            let storage = self.storage.as_mut().unwrap();
            self.queue.write_buffer(
                &storage.rows,
                0,
                &ubytes(matrix.row_offsets().iter().map(|&i| i as u32)),
            );
            self.queue.write_buffer(
                &storage.cols,
                0,
                &ubytes(matrix.column_indices().iter().map(|&i| i as u32)),
            );
            self.queue
                .write_buffer(&storage.values, 0, &fbytes(coefficients));
            storage.cached_matrix = Some(matrix.clone());
            storage.row_scales = Arc::new(scales);
        }
        let storage = self.storage.as_ref().unwrap();
        let scales = storage.row_scales.clone();
        let root_bytes = storage.bytes;
        let magnitude = matrix.values().iter().fold(0_f64, |m, v| m.max(v.abs()));
        let symmetric = config.gpu_preconditioner == GpuPreconditioner::Auto
            && matrix.is_symmetric(1e-12 * magnitude)
            && (0..n).all(|row| {
                matrix.get(row, row).is_some_and(|d| d > 0.)
                    && (matrix.row_offsets()[row]..matrix.row_offsets()[row + 1]).all(|k| {
                        matrix.column_indices()[k] == row || matrix.values()[k] <= 1e-12 * magnitude
                    })
            });
        let mut use_amg = false;
        if symmetric {
            let budget = (self.info.max_buffer_bytes as f64 * config.gpu_memory_fraction) as u64;
            let prepared = if self
                .amg
                .as_ref()
                .is_some_and(|amg| amg.same_topology(matrix))
            {
                self.amg.as_mut().unwrap().upload(&self.queue, matrix)
            } else {
                self.amg = None;
                if self.device.limits().max_storage_buffers_per_shader_stage < 7 {
                    Err("Adapter lacks seven storage bindings required by GPU AMG".into())
                } else {
                    Amg::new(
                        &self.device,
                        &self.queue,
                        matrix,
                        budget.saturating_sub(root_bytes),
                    )
                    .map(|amg| {
                        self.amg = Some(amg);
                    })
                }
            };
            match prepared {
                Ok(()) => use_amg = true,
                Err(error) => {
                    self.amg = None;
                    self.note(format!(
                        "GPU AMG setup unavailable; GPU Krylov preconditioning selected: {error}"
                    ));
                }
            }
        }
        if let Some(amg) = &self.amg {
            self.peak_bytes = self.peak_bytes.max(root_bytes + amg.bytes);
        }
        let method = if use_amg {
            format!(
                "f64 FGMRES / GPU aggregation AMG ({} levels)",
                self.amg.as_ref().unwrap().level_count()
            )
        } else {
            "f64 FGMRES / GPU Jacobi-BiCGSTAB".into()
        };
        *self.methods.entry(method.clone()).or_default() += 1;
        let report = crate::linear::fgmres(matrix, rhs, x, options, restart, |rhs| {
            super::checkpoint()?;
            let delta = if use_amg {
                let (buffer, amplitude) = self
                    .amg
                    .as_ref()
                    .unwrap()
                    .apply(&self.device, &self.queue, rhs, config.gpu_smoothing_sweeps)
                    .map_err(|message| crate::LinearAlgebraError::Backend { message })?;
                let values = self
                    .read(&buffer, n as u64 * 4)
                    .map_err(|message| crate::LinearAlgebraError::Backend { message })?;
                self.v_cycles += 1;
                values
                    .into_iter()
                    .map(|v| v as f64 * amplitude)
                    .collect::<Vec<_>>()
            } else {
                let mut input = rhs
                    .iter()
                    .zip(scales.iter())
                    .map(|(r, s)| r / s)
                    .collect::<Vec<_>>();
                let amplitude = input.iter().fold(0_f64, |m, v| m.max(v.abs()));
                if amplitude <= 0. || !amplitude.is_finite() {
                    return Err(crate::LinearAlgebraError::Backend {
                        message: "GPU Krylov preconditioner RHS outside working range".into(),
                    });
                }
                for value in &mut input {
                    *value /= amplitude;
                }
                let values = match self.correction(&input, 1e-2, config.gpu_inner_iterations) {
                    Ok((values, steps)) => {
                        self.inner_iterations += steps as u64;
                        values
                    }
                    Err(error) => {
                        super::checkpoint()?;
                        self.note(format!(
                            "GPU inner recurrence restarted with polynomial Jacobi: {error}"
                        ));
                        let values = self
                            .smooth_correction(&input, config.gpu_inner_iterations)
                            .map_err(|message| crate::LinearAlgebraError::Backend { message })?;
                        self.inner_iterations += config.gpu_inner_iterations as u64;
                        values
                    }
                };
                values
                    .into_iter()
                    .map(|value| value as f64 * amplitude)
                    .collect()
            };
            if delta.iter().any(|v| !v.is_finite()) {
                return Err(crate::LinearAlgebraError::Backend {
                    message: "Non-finite GPU preconditioner result".into(),
                });
            }
            Ok(delta)
        })
        .map_err(|e| e.to_string())?;
        if !report.converged() {
            return Err(format!("GPU-preconditioned FGMRES exhausted {} outer iterations: true L2 {:.6e}, target {:.6e}, initial {:.6e}; method {method}",report.iterations,report.final_residual,options.absolute_tolerance.max(options.relative_tolerance*report.initial_residual),report.initial_residual));
        }
        Ok(report)
    }
}
