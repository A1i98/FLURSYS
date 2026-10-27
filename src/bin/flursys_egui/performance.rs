use super::*;
use std::sync::Arc;

pub(super) fn envelope(points: &[(f64, Option<f64>)], pixels: usize) -> Vec<(f64, Option<f64>)> {
    let chunk_size = points.len().div_ceil(pixels.max(1)).max(1);
    let mut out = Vec::with_capacity(points.len().min(pixels.saturating_mul(4)));
    let mut start = 0;
    while start < points.len() {
        if !points[start].0.is_finite() || !points[start].1.is_some_and(f64::is_finite) {
            if out
                .last()
                .is_some_and(|p: &(f64, Option<f64>)| p.1.is_some())
            {
                out.push((points[start].0, None));
            }
            start += 1;
            continue;
        }
        let end = points[start..]
            .iter()
            .position(|p| !p.0.is_finite() || !p.1.is_some_and(f64::is_finite))
            .map_or(points.len(), |n| start + n);
        for chunk in points[start..end].chunks(chunk_size) {
            let min = chunk
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map_or(0, |(i, _)| i);
            let max = chunk
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                .map_or(0, |(i, _)| i);
            let mut indices = [0, min, max, chunk.len() - 1];
            indices.sort_unstable();
            let mut previous = None;
            for i in indices {
                if previous != Some(i) {
                    out.push(chunk[i]);
                }
                previous = Some(i);
            }
        }
        start = end;
    }
    out
}

struct Request {
    generation: u64,
    path: PathBuf,
    mesh: Arc<UnstructuredMesh>,
    render_mesh: Option<Arc<MeshRenderCache>>,
    run: String,
}
type Loaded = Result<
    (
        usize,
        ResultDataset,
        ResultRenderCache,
        Option<flursys::compressible::Snapshot>,
    ),
    String,
>;
#[derive(Default)]
pub(super) struct FieldLoader {
    generation: u64,
    selected_path: Option<PathBuf>,
    mesh: Option<Arc<UnstructuredMesh>>,
    render_mesh: Option<Arc<MeshRenderCache>>,
    pending: Option<Request>,
    receiver: Option<Receiver<(u64, Loaded)>>,
}
impl FieldLoader {
    pub fn busy(&self) -> bool {
        self.receiver.is_some() || self.pending.is_some()
    }
    pub fn request(
        &mut self,
        path: PathBuf,
        mesh: &UnstructuredMesh,
        run: String,
        ctx: &egui::Context,
    ) {
        if self.mesh.as_ref().is_none_or(|m| m.id() != mesh.id()) {
            self.mesh = Some(Arc::new(mesh.clone()));
            self.render_mesh = None;
        }
        self.selected_path = Some(path.clone());
        self.generation = self.generation.wrapping_add(1);
        self.pending = self.mesh.as_ref().map(|mesh| Request {
            generation: self.generation,
            path,
            mesh: mesh.clone(),
            render_mesh: self.render_mesh.clone(),
            run,
        });
        self.start(ctx);
    }
    fn start(&mut self, ctx: &egui::Context) {
        if self.receiver.is_some() {
            return;
        }
        let Some(request) = self.pending.take() else {
            return;
        };
        let (tx, rx) = mpsc::channel();
        self.receiver = Some(rx);
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let loaded = load_field(
                &request.path,
                &request.mesh,
                &request.run,
                request.render_mesh,
            );
            let _ = tx.send((request.generation, loaded));
            ctx.request_repaint();
        });
    }
    pub fn poll(&mut self, ctx: &egui::Context) -> Option<Loaded> {
        let result = self.receiver.as_ref()?.try_recv();
        match result {
            Ok((generation, loaded)) => {
                self.receiver = None;
                if let Ok((_, _, cache, _)) = &loaded {
                    if self
                        .mesh
                        .as_ref()
                        .is_some_and(|m| m.id() == cache.mesh_cache().mesh_id())
                    {
                        self.render_mesh = Some(cache.shared_mesh_cache());
                        if let Some(pending) = &mut self.pending {
                            pending.render_mesh = self.render_mesh.clone();
                        }
                    }
                }
                self.start(ctx);
                (generation == self.generation).then_some(loaded)
            }
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => {
                self.receiver = None;
                self.start(ctx);
                Some(Err("Saved-field loading worker stopped unexpectedly".into()))
            }
        }
    }
}
fn load_field(
    path: &Path,
    mesh: &UnstructuredMesh,
    run: &str,
    render_mesh: Option<Arc<MeshRenderCache>>,
) -> Loaded {
    if workbench::flow_frame::is_frame(path) {
        let frame = workbench::flow_frame::FlowFrame::read(path)?;
        let data = frame.dataset_on(mesh)?;
        let cache = match render_mesh {
            Some(cache) => ResultRenderCache::build_with_mesh_cache(&data, cache),
            None => ResultRenderCache::build(&data),
        }
        .map_err(|e| e.to_string())?;
        let state = match frame.fields {
            workbench::flow_frame::FlowFields::Compressible { state } => Some(state),
            _ => None,
        };
        return Ok((frame.step, data, cache, state));
    }
    let (index, initial) = workbench::checkpoint::load(path, mesh)?;
    let data = match initial {
        IncompressibleInitialConditions::Restart {
            velocity,
            pressure,
            temperature,
            turbulence,
            ..
        } => ResultDataset::from_values(
            run,
            mesh,
            pressure.values().to_vec(),
            velocity.values().to_vec(),
        )
        .and_then(|d| d.with_temperature(temperature.map(|t| t.values().to_vec())))
        .map(|d| d.with_turbulence(turbulence.as_ref())),
        IncompressibleInitialConditions::Fields { velocity, pressure } => {
            ResultDataset::from_values(
                run,
                mesh,
                pressure.values().to_vec(),
                velocity.values().to_vec(),
            )
        }
        _ => return Err("Saved frame does not contain complete fields".into()),
    }
    .map_err(|e| e.to_string())?;
    let cache = match render_mesh {
        Some(cache) => ResultRenderCache::build_with_mesh_cache(&data, cache),
        None => ResultRenderCache::build(&data),
    }
    .map_err(|e| e.to_string())?;
    Ok((index, data, cache, None))
}
impl App {
    pub(super) fn request_field_frame(&mut self, ctx: &egui::Context) {
        let Some(path) = self.field_frames.get(self.field_frame).cloned() else {
            return;
        };
        let Some(mesh) = self
            .result
            .as_ref()
            .map(|d| d.mesh())
            .or_else(|| self.session.mesh().map(|m| &m.mesh))
        else {
            return;
        };
        self.field_loader
            .request(path, mesh, self.result_run.clone(), ctx);
    }
    pub(super) fn poll_field_frame(&mut self, ctx: &egui::Context) {
        if self.export.is_some() {
            return;
        }
        if self.live_fields
            || self
                .field_loader
                .selected_path
                .as_ref()
                .is_some_and(|path| self.field_frames.get(self.field_frame) != Some(path))
        {
            self.field_loader = Default::default();
            return;
        }
        match self.field_loader.poll(ctx) {
            Some(Ok((index, data, cache, state))) => {
                let mesh_id = self
                    .result
                    .as_ref()
                    .map(|d| d.mesh().id())
                    .or_else(|| self.session.mesh().map(|m| m.mesh.id()));
                if data.run_id() != self.result_run || mesh_id != Some(data.mesh().id()) {
                    return;
                }
                if let Some(state) = state {
                    self.update_gas_result_report(state);
                }
                self.result_iteration = index;
                self.result_cache = Some(cache);
                for probe in &mut self.probes {
                    if let Some(value) = data.probe_cell(probe.cell_index) {
                        *probe = value;
                    }
                }
                self.result = Some(data);
                self.plane_tick = Instant::now();
                if self.plane_index > 0 {
                    self.refresh_plane_preview();
                }
            }
            Some(Err(e)) => {
                self.field_playing = false;
                self.message(e);
            }
            None => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::envelope;

    #[test]
    fn envelope_retains_extrema_endpoints_and_order() {
        let mut points: Vec<_> = (0..100_000).map(|i| (i as f64, Some(1.))).collect();
        points[1357].1 = Some(-90.);
        points[73421].1 = Some(200.);
        let sampled = envelope(&points, 800);
        assert_eq!(sampled.first(), points.first());
        assert_eq!(sampled.last(), points.last());
        assert!(sampled.contains(&points[1357]));
        assert!(sampled.contains(&points[73421]));
        assert!(sampled.windows(2).all(|w| w[0].0 < w[1].0));
        assert!(sampled.len() <= 3200);
    }

    #[test]
    fn envelope_preserves_missing_data_breaks() {
        let mut points: Vec<_> = (0..1000).map(|i| (i as f64, Some(i as f64))).collect();
        points[413].1 = None;
        points[414].1 = Some(f64::NAN);
        let sampled = envelope(&points, 10);
        let gap = sampled
            .iter()
            .position(|p| p.1.is_none())
            .expect("missing interval");
        assert_eq!(sampled[gap - 1], points[412]);
        assert_eq!(sampled[gap + 1], points[415]);
    }
}
