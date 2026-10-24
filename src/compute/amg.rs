//! Resident aggregation AMG V-cycle. One submission for a full cycle; a bounded
//! terminal inverse is constructed in f64 at setup and applied on GPU.
use super::gpu::{fbytes, ubytes};
use crate::linear::DeviceHierarchy;
use crate::CsrMatrix;
use egui_wgpu::wgpu;
use wgpu::util::DeviceExt;
struct Level {
    n: usize,
    coarse: usize,
    rows: wgpu::Buffer,
    cols: wgpu::Buffer,
    values: wgpu::Buffer,
    vectors: wgpu::Buffer,
    inverse: wgpu::Buffer,
    _links: wgpu::Buffer,
    _params: Vec<wgpu::Buffer>,
    groups: Vec<wgpu::BindGroup>,
}
pub(super) struct Amg {
    hierarchy: DeviceHierarchy,
    levels: Vec<Level>,
    pipeline: wgpu::ComputePipeline,
    pub bytes: u64,
    pub scale: f64,
    uploaded: bool,
}
fn terminal_inverse(matrix: &CsrMatrix, scale: f64) -> Result<Vec<f32>, String> {
    let n = matrix.nrows();
    let mut lower = vec![0.; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut value = matrix.get(i, j).unwrap_or(0.) / scale;
            for k in 0..j {
                value -= lower[i * n + k] * lower[j * n + k];
            }
            lower[i * n + j] = if i == j {
                if !value.is_finite() || value <= 0. {
                    return Err("GPU AMG coarse operator is not positive definite".into());
                }
                value.sqrt()
            } else {
                value / lower[j * n + j]
            };
        }
    }
    let mut inverse = vec![0_f32; n * n];
    for column in 0..n {
        let mut x = vec![0.; n];
        for i in 0..n {
            let mut value = if i == column { 1. } else { 0. };
            for j in 0..i {
                value -= lower[i * n + j] * x[j];
            }
            x[i] = value / lower[i * n + i];
        }
        for i in (0..n).rev() {
            let mut value = x[i];
            for j in i + 1..n {
                value -= lower[j * n + i] * x[j];
            }
            x[i] = value / lower[i * n + i];
            inverse[i * n + column] = x[i] as f32;
            if !inverse[i * n + column].is_finite() {
                return Err("GPU AMG coarse inverse exceeds f32 range".into());
            }
        }
    }
    Ok(inverse)
}
impl Amg {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        matrix: &CsrMatrix,
        budget: u64,
    ) -> Result<Self, String> {
        let hierarchy = DeviceHierarchy::new(matrix).map_err(|e| e.to_string())?;
        let mut bytes = 0;
        let limits = device.limits();
        for (i, level) in hierarchy.levels.iter().enumerate() {
            let n = level.matrix.nrows() as u64;
            let coarse = hierarchy
                .levels
                .get(i + 1)
                .map_or(0, |l| l.matrix.nrows() as u64);
            let sizes = [
                (n + 1) * 4,
                level.matrix.nnz() as u64 * 4,
                level.matrix.nnz() as u64 * 4,
                n * 16,
                (2 * n + coarse + 1) * 4,
                if coarse == 0 { n * n * 4 } else { 4 },
            ];
            for size in sizes {
                if size > u64::from(limits.max_storage_buffer_binding_size)
                    || size > limits.max_buffer_size
                {
                    return Err("GPU AMG buffer exceeds adapter binding limit".into());
                }
                bytes += size.max(4);
            }
            if n.div_ceil(128) > u64::from(limits.max_compute_workgroups_per_dimension) {
                return Err("GPU AMG dispatch limit exceeded".into());
            }
            bytes += 7 * 32;
        }
        if bytes > budget {
            return Err(format!(
                "GPU AMG needs {bytes} additional bytes, resource budget is {budget}"
            ));
        }
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("FLURSYS aggregation V-cycle"),
            source: wgpu::ShaderSource::Wgsl(include_str!("amg.wgsl").into()),
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("GPU AMG buffers"),
            entries: &(0..8)
                .map(|binding| wgpu::BindGroupLayoutEntry {
                    binding,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: if binding == 0 {
                            wgpu::BufferBindingType::Uniform
                        } else {
                            wgpu::BufferBindingType::Storage {
                                read_only: binding != 4 && binding != 6,
                            }
                        },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                })
                .collect::<Vec<_>>(),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("GPU AMG V-cycle"),
            layout: Some(&pipeline_layout),
            module: &module,
            entry_point: Some("main"),
            compilation_options: Default::default(),
            cache: None,
        });
        let buffer = |name, size, usage| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(name),
                size,
                usage,
                mapped_at_creation: false,
            })
        };
        let usage = wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST;
        let vector_buffers = hierarchy
            .levels
            .iter()
            .map(|l| {
                buffer(
                    "AMG resident fields",
                    l.matrix.nrows() as u64 * 16,
                    usage | wgpu::BufferUsages::COPY_SRC,
                )
            })
            .collect::<Vec<_>>();
        let dummy = buffer("AMG terminal child", 4, usage);
        let mut levels = Vec::new();
        for (i, level) in hierarchy.levels.iter().enumerate() {
            let matrix = &level.matrix;
            let n = matrix.nrows();
            let coarse = hierarchy.levels.get(i + 1).map_or(0, |l| l.matrix.nrows());
            let rows = buffer("AMG CSR rows", (n as u64 + 1) * 4, usage);
            let cols = buffer("AMG CSR columns", (matrix.nnz() as u64 * 4).max(4), usage);
            let values = buffer("AMG coefficients", (matrix.nnz() as u64 * 4).max(4), usage);
            let inverse = buffer(
                "AMG terminal inverse",
                if coarse == 0 { (n * n * 4) as u64 } else { 4 },
                usage,
            );
            let mut transfer = Vec::new();
            if let Some(parents) = &level.parents {
                transfer.extend(parents.iter().map(|&v| v as u32));
                let mut offsets = vec![0_u32; coarse + 1];
                for &parent in parents {
                    offsets[parent + 1] += 1;
                }
                for j in 0..coarse {
                    offsets[j + 1] += offsets[j];
                }
                let mut next = offsets[..coarse].to_vec();
                let mut members = vec![0_u32; n];
                for (fine, &parent) in parents.iter().enumerate() {
                    members[next[parent] as usize] = fine as u32;
                    next[parent] += 1;
                }
                transfer.extend(offsets);
                transfer.extend(members);
            } else {
                transfer.push(0);
            }
            let links = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("AMG parent/child gather map"),
                contents: &ubytes(transfer),
                usage: wgpu::BufferUsages::STORAGE,
            });
            let mut params = Vec::new();
            let mut groups = Vec::new();
            for op in 0..7 {
                let mut data = ubytes([n as u32, coarse as u32, op, 0]);
                data.extend(fbytes([2. / 3., 0., 0., 0.]));
                let uniform = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("AMG dispatch"),
                    contents: &data,
                    usage: wgpu::BufferUsages::UNIFORM,
                });
                let buffers = [
                    &uniform,
                    &rows,
                    &cols,
                    &values,
                    &vector_buffers[i],
                    &links,
                    vector_buffers.get(i + 1).unwrap_or(&dummy),
                    &inverse,
                ];
                let entries = buffers
                    .into_iter()
                    .enumerate()
                    .map(|(binding, b)| wgpu::BindGroupEntry {
                        binding: binding as u32,
                        resource: b.as_entire_binding(),
                    })
                    .collect::<Vec<_>>();
                groups.push(device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("AMG level operation"),
                    layout: &layout,
                    entries: &entries,
                }));
                params.push(uniform);
            }
            levels.push(Level {
                n,
                coarse,
                rows,
                cols,
                values,
                vectors: vector_buffers[i].clone(),
                inverse,
                _links: links,
                _params: params,
                groups,
            });
        }
        let mut result = Self {
            hierarchy,
            levels,
            pipeline,
            bytes,
            scale: 1.,
            uploaded: false,
        };
        result.upload(queue, matrix)?;
        Ok(result)
    }
    pub fn same_topology(&self, matrix: &CsrMatrix) -> bool {
        self.hierarchy.same_topology(matrix)
    }
    pub fn upload(&mut self, queue: &wgpu::Queue, matrix: &CsrMatrix) -> Result<(), String> {
        if self.uploaded
            && self.same_topology(matrix)
            && self.hierarchy.levels[0].matrix.values() == matrix.values()
        {
            return Ok(());
        }
        self.hierarchy.update(matrix).map_err(|e| e.to_string())?;
        self.scale = matrix.values().iter().fold(0_f64, |m, v| m.max(v.abs()));
        if self.scale <= 0. || !self.scale.is_finite() {
            return Err("GPU AMG invalid matrix scale".into());
        }
        for (data, level) in self.hierarchy.levels.iter().zip(&self.levels) {
            let matrix = &data.matrix;
            for i in 0..matrix.nrows() {
                if (matrix.get(i, i).unwrap_or(0.) / self.scale) as f32 <= 0. {
                    return Err("GPU AMG diagonal outside positive f32 range".into());
                }
            }
            if !self.uploaded {
                queue.write_buffer(
                    &level.rows,
                    0,
                    &ubytes(matrix.row_offsets().iter().map(|&i| i as u32)),
                );
                queue.write_buffer(
                    &level.cols,
                    0,
                    &ubytes(matrix.column_indices().iter().map(|&i| i as u32)),
                );
            }
            queue.write_buffer(
                &level.values,
                0,
                &fbytes(matrix.values().iter().map(|&v| (v / self.scale) as f32)),
            );
            if data.parents.is_none() {
                queue.write_buffer(
                    &level.inverse,
                    0,
                    &fbytes(terminal_inverse(matrix, self.scale)?),
                );
            }
        }
        self.uploaded = true;
        Ok(())
    }
    fn record(&self, encoder: &mut wgpu::CommandEncoder, level: usize, op: usize) {
        let level = &self.levels[level];
        let count = if op == 4 { level.coarse } else { level.n };
        let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, &level.groups[op], &[]);
        pass.dispatch_workgroups(count.div_ceil(128) as u32, 1, 1);
    }
    fn cycle(&self, encoder: &mut wgpu::CommandEncoder, level: usize, sweeps: usize) {
        if self.levels[level].coarse == 0 {
            self.record(encoder, level, 6);
            return;
        }
        for _ in 0..sweeps {
            self.record(encoder, level, 1);
            self.record(encoder, level, 2);
        }
        self.record(encoder, level, 3);
        self.record(encoder, level, 4);
        self.cycle(encoder, level + 1, sweeps);
        self.record(encoder, level, 5);
        for _ in 0..sweeps {
            self.record(encoder, level, 1);
            self.record(encoder, level, 2);
        }
    }
    pub fn apply(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        rhs: &[f64],
        sweeps: usize,
    ) -> Result<(wgpu::Buffer, f64), String> {
        let amplitude = rhs
            .iter()
            .map(|v| v / self.scale)
            .fold(0_f64, |m, v| m.max(v.abs()));
        if amplitude <= 0. || !amplitude.is_finite() {
            return Err("GPU AMG right-hand-side scaling failed".into());
        }
        queue.write_buffer(
            &self.levels[0].vectors,
            self.levels[0].n as u64 * 4,
            &fbytes(rhs.iter().map(|v| (v / self.scale / amplitude) as f32)),
        );
        let mut encoder = device.create_command_encoder(&Default::default());
        self.record(&mut encoder, 0, 0);
        self.cycle(&mut encoder, 0, sweeps);
        queue.submit([encoder.finish()]);
        Ok((self.levels[0].vectors.clone(), amplitude))
    }
    pub fn level_count(&self) -> usize {
        self.levels.len()
    }
}
