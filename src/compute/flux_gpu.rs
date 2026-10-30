use super::{gpu, ComputeOptions, DeviceInfo};
use crate::compressible::{
    flux::{evaluate, Input},
    Conservative, FluxScheme, IdealGas,
};
use egui_wgpu::wgpu;
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
struct Buffers {
    capacity: usize,
    input: wgpu::Buffer,
    output: wgpu::Buffer,
    read: wgpu::Buffer,
    bind: wgpu::BindGroup,
}
pub(super) struct Solver {
    pub info: DeviceInfo,
    pub peak_bytes: u64,
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    layout: wgpu::BindGroupLayout,
    params: wgpu::Buffer,
    buffers: Option<Buffers>,
    driver_error: Arc<Mutex<Option<String>>>,
    timeout: Duration,
    batch: usize,
}
impl Solver {
    pub fn new(options: &ComputeOptions) -> Result<Self, String> {
        let (device, queue, info, driver_error) =
            gpu::request_device(&options.adapter, "FLURSYS conservative flux")?;
        let limit = info.max_storage_binding_bytes.min(info.max_buffer_bytes);
        let budget = (info.max_buffer_bytes as f64 * options.gpu_memory_fraction) as u64;
        let batch = (limit / 64).min(budget / 128).min(262144) as usize;
        if batch == 0 {
            return Err("Insufficient GPU flux-buffer budget".into());
        }
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Conservative face flux"),
            source: wgpu::ShaderSource::Wgsl(include_str!("flux.wgsl").into()),
        });
        let entries = (0..3)
            .map(|binding| wgpu::BindGroupLayoutEntry {
                binding,
                visibility: wgpu::ShaderStages::COMPUTE,
                ty: wgpu::BindingType::Buffer {
                    ty: if binding == 2 {
                        wgpu::BufferBindingType::Uniform
                    } else {
                        wgpu::BufferBindingType::Storage {
                            read_only: binding == 0,
                        }
                    },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            })
            .collect::<Vec<_>>();
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Conservative face flux"),
            entries: &entries,
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Conservative face flux"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("main"),
            compilation_options: Default::default(),
            cache: None,
        });
        let params = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Flux parameters"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        if let Some(e) = pollster::block_on(scope.pop()) {
            return Err(e.to_string());
        }
        Ok(Self {
            info,
            peak_bytes: 16,
            device,
            queue,
            pipeline,
            layout,
            params,
            buffers: None,
            driver_error,
            timeout: Duration::from_millis(options.gpu_timeout_ms),
            batch,
        })
    }
    fn buffers(&mut self, n: usize) -> Result<(), String> {
        if self.buffers.as_ref().is_some_and(|b| b.capacity >= n) {
            return Ok(());
        }
        let capacity = n.next_power_of_two().min(self.batch).max(n);
        let scope = self.device.push_error_scope(wgpu::ErrorFilter::OutOfMemory);
        let make = |label, size, usage| {
            self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage,
                mapped_at_creation: false,
            })
        };
        let input = make(
            "Flux states",
            capacity as u64 * 64,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        );
        let output = make(
            "Face flux",
            capacity as u64 * 32,
            wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
        );
        let read = make(
            "Flux readback",
            capacity as u64 * 32,
            wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        );
        let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &self.layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: input.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: output.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: self.params.as_entire_binding(),
                },
            ],
        });
        if let Some(e) = pollster::block_on(scope.pop()) {
            return Err(e.to_string());
        }
        self.peak_bytes = self.peak_bytes.max(capacity as u64 * 128 + 16);
        self.buffers = Some(Buffers {
            capacity,
            input,
            output,
            read,
            bind,
        });
        Ok(())
    }
    pub fn solve(
        &mut self,
        mut inputs: impl Iterator<Item = Input>,
        gas: IdealGas,
        scheme: FluxScheme,
    ) -> Result<Vec<(Conservative, bool)>, String> {
        let mut result = Vec::with_capacity(inputs.size_hint().0);
        loop {
            let rows = inputs.by_ref().take(self.batch).collect::<Vec<_>>();
            if rows.is_empty() {
                break;
            }
            self.buffers(rows.len())?;
            let mut packed = Vec::with_capacity(rows.len() * 16);
            for row in &rows {
                let l = row.left;
                let r = row.right;
                let n = row.normal;
                packed.extend(
                    [
                        l.density,
                        l.velocity.x,
                        l.velocity.y,
                        l.velocity.z,
                        l.pressure,
                        r.density,
                        r.velocity.x,
                        r.velocity.y,
                        r.velocity.z,
                        r.pressure,
                        n.x,
                        n.y,
                        n.z,
                        0.,
                        0.,
                        0.,
                    ]
                    .map(|v| v as f32),
                );
            }
            if packed.iter().any(|v| !v.is_finite()) {
                return Err("Flow states cannot be represented in GPU f32 storage".into());
            }
            let buffers = self.buffers.as_ref().ok_or("GPU buffers unavailable")?;
            self.queue
                .write_buffer(&buffers.input, 0, &gpu::fbytes(packed));
            let scheme_id = match scheme {
                FluxScheme::Rusanov => 0,
                FluxScheme::Hllc => 1,
                FluxScheme::CentralUpwind => 2,
            };
            let mut params = gpu::ubytes([rows.len() as u32, scheme_id]);
            params.extend((gas.gamma as f32).to_le_bytes());
            params.extend(0u32.to_le_bytes());
            self.queue.write_buffer(&self.params, 0, &params);
            let bytes = rows.len() as u64 * 32;
            let mut encoder = self.device.create_command_encoder(&Default::default());
            {
                let mut pass = encoder.begin_compute_pass(&Default::default());
                pass.set_pipeline(&self.pipeline);
                pass.set_bind_group(0, &buffers.bind, &[]);
                pass.dispatch_workgroups((rows.len() as u32).div_ceil(128), 1, 1);
            }
            encoder.copy_buffer_to_buffer(&buffers.output, 0, &buffers.read, 0, bytes);
            self.queue.submit([encoder.finish()]);
            let slice = buffers.read.slice(0..bytes);
            let (tx, rx) = std::sync::mpsc::channel();
            slice.map_async(wgpu::MapMode::Read, move |r| {
                let _ = tx.send(r);
            });
            struct Unmap<'a>(&'a wgpu::Buffer);
            impl Drop for Unmap<'_> {
                fn drop(&mut self) {
                    self.0.unmap();
                }
            }
            let _unmap = Unmap(&buffers.read);
            let start = Instant::now();
            loop {
                super::checkpoint().map_err(|e| e.to_string())?;
                if let Some(e) = self.driver_error.lock().map_err(|e| e.to_string())?.clone() {
                    return Err(e);
                }
                self.device
                    .poll(wgpu::PollType::Poll)
                    .map_err(|e| e.to_string())?;
                match rx.try_recv() {
                    Ok(r) => {
                        r.map_err(|e| e.to_string())?;
                        break;
                    }
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        return Err("GPU flux mapping disconnected".into());
                    }
                    _ => {}
                }
                if start.elapsed() > self.timeout {
                    return Err("GPU flux timeout".into());
                }
                std::thread::sleep(Duration::from_millis(1));
            }
            let values = {
                let map = slice.get_mapped_range().map_err(|e| e.to_string())?;
                map.chunks_exact(4)
                    .map(|v| f32::from_le_bytes(v.try_into().unwrap()) as f64)
                    .collect::<Vec<_>>()
            };
            if values.iter().any(|v| !v.is_finite()) {
                return Err("Non-finite GPU conservative face flux".into());
            }
            for (i, row) in values.chunks_exact(8).enumerate() {
                let flux = Conservative(row[..5].try_into().unwrap());
                if i % (rows.len() / 32).max(1) == 0 || i + 1 == rows.len() {
                    let input = rows[i];
                    let (reference, _) =
                        evaluate(input.left, input.right, gas, input.normal, scheme);
                    let sound = gas
                        .sound_speed(input.left)
                        .max(gas.sound_speed(input.right));
                    let rho = input.left.density.max(input.right.density);
                    let speed = sound + input.left.velocity.norm().max(input.right.velocity.norm());
                    let scales = [
                        rho * speed,
                        rho * speed * speed,
                        rho * speed * speed,
                        rho * speed * speed,
                        (input.left.conservative(gas).0[4]
                            .abs()
                            .max(input.right.conservative(gas).0[4].abs())
                            + input.left.pressure.max(input.right.pressure))
                            * speed,
                    ];
                    if (0..5)
                        .any(|k| (flux.0[k] - reference.0[k]).abs() > 2e-5 * scales[k].max(1e-20))
                    {
                        return Err("GPU face flux failed f64 reference sampling".into());
                    }
                }
                result.push((flux, row[5] > 0.5));
            }
        }
        Ok(result)
    }
}
