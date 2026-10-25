use super::*;
use std::{
    io::{BufWriter, Write},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc::{SyncSender, TrySendError},
        Arc,
    },
};
#[derive(Clone)]
pub(super) struct Options {
    width: u32,
    height: u32,
    fps: u16,
    first: usize,
    last: usize,
    stride: usize,
    max_frames: usize,
    max_mb: usize,
    fast_palette: bool,
    flow_frames: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            width: 960,
            height: 540,
            fps: 10,
            first: 1,
            last: 0,
            stride: 1,
            max_frames: 300,
            max_mb: 64,
            fast_palette: false,
            flow_frames: 120,
        }
    }
}
#[derive(Clone, Debug)]
struct Ticket {
    id: u64,
    rect: egui::Rect,
    scale: f32,
}
struct Capture {
    ticket: Ticket,
    image: Arc<egui::ColorImage>,
}
enum Command {
    Capture(Capture),
    Field {
        path: PathBuf,
        mesh: Arc<UnstructuredMesh>,
        run: String,
        plane: Option<Box<plane_monitor::FlowPlaneMonitor>>,
        physical: bool,
    },
    Plane(PathBuf),
}
struct FieldFrame {
    index: usize,
    data: ResultDataset,
    cache: ResultRenderCache,
    snapshot: Option<plane_monitor::PlaneSnapshot>,
}
enum Loaded {
    Field(Box<FieldFrame>),
    Plane(Box<plane_monitor::PlaneSnapshot>),
}
enum Event {
    Loaded(Box<Loaded>),
    Encoded { frames: usize, bytes: u64 },
    Finished(Result<Outcome, String>),
}
struct Outcome {
    frames: usize,
    bytes: u64,
    limited: bool,
    cancelled: bool,
}
enum Source {
    Field(Vec<PathBuf>),
    Plane(Vec<PathBuf>),
    Flow,
    Still,
}
struct Restore {
    camera: viewport::View,
    live: bool,
    flow: bool,
    fields: bool,
    planes: bool,
    field: usize,
    plane: usize,
    phase: f64,
}
pub(super) struct Job {
    id: u64,
    path: PathBuf,
    source: Source,
    indices: Vec<usize>,
    position: usize,
    frames: usize,
    bytes: u64,
    pending: bool,
    requested_at: Option<Instant>,
    encoding: bool,
    settle: usize,
    started: Instant,
    cancel: Arc<AtomicBool>,
    send: SyncSender<Command>,
    mesh: Option<Arc<UnstructuredMesh>>,
    loading: bool,
    receive: Receiver<Event>,
    options: Options,
    restore: Restore,
    gif: bool,
    failure: Option<String>,
}
impl Job {
    pub(super) fn frozen_camera(&self) -> viewport::View {
        self.restore.camera.clone()
    }
}
impl Drop for Job {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Release);
    }
}
struct CountWriter {
    inner: BufWriter<std::fs::File>,
    bytes: u64,
}
impl Write for CountWriter {
    fn write(&mut self, data: &[u8]) -> std::io::Result<usize> {
        let n = self.inner.write(data)?;
        self.bytes += n as u64;
        Ok(n)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}
fn pixels(capture: Capture, dimensions: Option<(u32, u32)>) -> Result<image::RgbaImage, String> {
    let image = capture.image;
    let r = capture.ticket.rect;
    let scale = capture.ticket.scale;
    let x = (r.left() * scale).floor().max(0.) as usize;
    let y = (r.top() * scale).floor().max(0.) as usize;
    let right = ((r.right() * scale).ceil().max(0.) as usize).min(image.size[0]);
    let bottom = ((r.bottom() * scale).ceil().max(0.) as usize).min(image.size[1]);
    let w = right.saturating_sub(x);
    let h = bottom.saturating_sub(y);
    if w == 0 || h == 0 {
        return Err("No visible capture area; restore the window and retry export.".into());
    }
    let mut rgba = Vec::with_capacity(w * h * 4);
    for row in y..y + h {
        for p in &image.pixels[row * image.size[0] + x..row * image.size[0] + x + w] {
            rgba.extend_from_slice(&p.to_array());
        }
    }
    let source =
        image::RgbaImage::from_raw(w as u32, h as u32, rgba).ok_or("Invalid image buffer")?;
    if let Some((width, height)) = dimensions {
        // Preserve geometry if the window size or DPI changed during capture.
        // Letterbox instead of stretching scientific plots to the output aspect.
        let scale = (width as f64 / w as f64).min(height as f64 / h as f64);
        let fitted_width = ((w as f64 * scale).round() as u32).clamp(1, width);
        let fitted_height = ((h as f64 * scale).round() as u32).clamp(1, height);
        let fitted = image::imageops::resize(
            &source,
            fitted_width,
            fitted_height,
            image::imageops::FilterType::Lanczos3,
        );
        let mut output = image::RgbaImage::from_pixel(width, height, *source.get_pixel(0, 0));
        image::imageops::overlay(
            &mut output,
            &fitted,
            ((width - fitted_width) / 2) as i64,
            ((height - fitted_height) / 2) as i64,
        );
        Ok(output)
    } else {
        Ok(source)
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn capture(size: [usize; 2], rect: egui::Rect) -> Capture {
        Capture {
            ticket: Ticket {
                id: 1,
                rect,
                scale: 1.0,
            },
            image: Arc::new(egui::ColorImage::filled(size, egui::Color32::WHITE)),
        }
    }

    #[test]
    fn crop_preserves_full_brightness() {
        let image = pixels(
            capture(
                [8, 6],
                egui::Rect::from_min_max(egui::pos2(2., 1.), egui::pos2(6., 5.)),
            ),
            None,
        )
        .unwrap();
        assert_eq!(image.dimensions(), (4, 4));
        assert!(image.pixels().all(|p| p.0 == [255; 4]));
    }

    #[test]
    fn rejects_capture_outside_window() {
        assert!(pixels(
            capture(
                [8, 6],
                egui::Rect::from_min_max(egui::pos2(20., 20.), egui::pos2(30., 30.),)
            ),
            None
        )
        .is_err());
    }

    #[test]
    fn resize_produces_exact_output_dimensions() {
        let image = pixels(
            capture(
                [8, 6],
                egui::Rect::from_min_max(egui::Pos2::ZERO, egui::pos2(8., 6.)),
            ),
            Some((16, 9)),
        )
        .unwrap();
        assert_eq!(image.dimensions(), (16, 9));
        assert!(image.pixels().all(|p| p.0 == [255; 4]));
    }
}

fn worker(
    path: PathBuf,
    gif: bool,
    options: Options,
    total: usize,
    cancel: Arc<AtomicBool>,
    receive: Receiver<Command>,
    send: mpsc::Sender<Event>,
) {
    let result = (|| -> Result<Outcome, String> {
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        let temporary = tempfile::NamedTempFile::new_in(parent).map_err(|e| e.to_string())?;
        let mut encoder = if gif {
            let writer = CountWriter {
                inner: BufWriter::new(temporary.reopen().map_err(|e| e.to_string())?),
                bytes: 0,
            };
            let mut e = gif::Encoder::new(writer, options.width as u16, options.height as u16, &[])
                .map_err(|e| e.to_string())?;
            e.set_repeat(gif::Repeat::Infinite)
                .map_err(|e| e.to_string())?;
            Some(e)
        } else {
            None
        };
        let mut frames = 0;
        let mut bytes = 0;
        let mut limited = false;
        let mut section_cache: Option<(String, Vec<plane_monitor::PlaneCell>)> = None;
        while frames < total {
            if cancel.load(Ordering::Acquire) {
                return Ok(Outcome {
                    frames,
                    bytes,
                    limited: false,
                    cancelled: true,
                });
            }
            let command = match receive.recv_timeout(Duration::from_millis(50)) {
                Ok(c) => c,
                Err(mpsc::RecvTimeoutError::Timeout) => continue,
                Err(_) => {
                    return Ok(Outcome {
                        frames,
                        bytes,
                        limited: false,
                        cancelled: true,
                    })
                }
            };
            let capture = match command {
                Command::Capture(capture) => capture,
                Command::Field {
                    path,
                    mesh,
                    run,
                    plane,
                    physical,
                } => {
                    let (index, initial) = workbench::checkpoint::load(&path, &mesh)?;
                    let (pressure, velocity, temperature, turbulence, time) = match initial {
                        IncompressibleInitialConditions::Restart {
                            pressure,
                            velocity,
                            temperature,
                            turbulence,
                            time,
                            ..
                        } => (
                            pressure,
                            velocity,
                            temperature,
                            turbulence,
                            physical.then_some(time),
                        ),
                        IncompressibleInitialConditions::Fields { pressure, velocity } => {
                            (pressure, velocity, None, None, None)
                        }
                        _ => return Err("Recorded frame does not contain complete fields".into()),
                    };
                    let data = ResultDataset::from_values(
                        &run,
                        &mesh,
                        pressure.values().to_vec(),
                        velocity.values().to_vec(),
                    )
                    .and_then(|d| d.with_temperature(temperature.map(|t| t.values().to_vec())))
                    .map_err(|e| e.to_string())?
                    .with_turbulence(turbulence.as_ref());
                    let cache = ResultRenderCache::build(&data).map_err(|e| e.to_string())?;
                    let snapshot = if let Some(plane) = plane {
                        if section_cache.as_ref().is_none_or(|(id, _)| *id != plane.id) {
                            section_cache = Some((
                                plane.id.clone(),
                                plane_monitor::intersect(data.mesh(), &plane)?,
                            ));
                        }
                        let mut snapshot = plane_monitor::sample(
                            &plane,
                            &section_cache.as_ref().unwrap().1,
                            index,
                            data.velocity(),
                            data.pressure(),
                        )?;
                        snapshot.physical_time = time;
                        Some(snapshot)
                    } else {
                        None
                    };
                    let _ = send.send(Event::Loaded(Box::new(Loaded::Field(Box::new(
                        FieldFrame {
                            index,
                            data,
                            cache,
                            snapshot,
                        },
                    )))));
                    continue;
                }
                Command::Plane(path) => {
                    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
                    let plane: plane_monitor::PlaneSnapshot =
                        serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
                    let _ = send.send(Event::Loaded(Box::new(Loaded::Plane(Box::new(plane)))));
                    continue;
                }
            };
            let image = pixels(capture, gif.then_some((options.width, options.height)))?;
            if cancel.load(Ordering::Acquire) {
                return Ok(Outcome {
                    frames,
                    bytes,
                    limited: false,
                    cancelled: true,
                });
            }
            if let Some(e) = &mut encoder {
                let mut rgba = image.into_raw();
                let mut frame = if options.fast_palette {
                    let palette: Vec<u8> = (0..256)
                        .flat_map(|v| {
                            [
                                ((v >> 5) * 255 / 7) as u8,
                                (((v >> 2) & 7) * 255 / 7) as u8,
                                ((v & 3) * 255 / 3) as u8,
                            ]
                        })
                        .collect();
                    let indices: Vec<u8> = rgba
                        .chunks_exact(4)
                        .map(|p| (p[0] & 0xe0) | ((p[1] & 0xe0) >> 3) | (p[2] >> 6))
                        .collect();
                    gif::Frame::from_palette_pixels(
                        options.width as u16,
                        options.height as u16,
                        indices,
                        palette,
                        None,
                    )
                } else {
                    gif::Frame::from_rgba_speed(
                        options.width as u16,
                        options.height as u16,
                        &mut rgba,
                        10,
                    )
                };
                // GIF stores centiseconds. Distribute rounding error between
                // frames so e.g. 30 FPS does not become 33.3 FPS.
                let fps = u64::from(options.fps.max(1));
                let timestamp = |n: usize| (n as u64 * 100 + fps / 2) / fps;
                frame.delay = (timestamp(frames + 1) - timestamp(frames)).max(1) as u16;
                frame.dispose = gif::DisposalMethod::Keep;
                if cancel.load(Ordering::Acquire) {
                    return Ok(Outcome {
                        frames,
                        bytes,
                        limited: false,
                        cancelled: true,
                    });
                }
                e.write_frame(&frame).map_err(|e| e.to_string())?;
                bytes = e.get_ref().bytes;
            } else {
                let file = temporary.reopen().map_err(|e| e.to_string())?;
                let mut e = png::Encoder::new(BufWriter::new(file), image.width(), image.height());
                e.set_color(png::ColorType::Rgba);
                e.set_depth(png::BitDepth::Eight);
                let mut png = e.write_header().map_err(|e| e.to_string())?;
                png.write_image_data(image.as_raw())
                    .map_err(|e| e.to_string())?;
                png.finish().map_err(|e| e.to_string())?;
                bytes = temporary
                    .as_file()
                    .metadata()
                    .map_err(|e| e.to_string())?
                    .len();
            }
            frames += 1;
            let _ = send.send(Event::Encoded { frames, bytes });
            if gif && bytes >= options.max_mb as u64 * 1024 * 1024 {
                limited = frames < total;
                break;
            }
        }
        if let Some(e) = encoder {
            let mut writer = e.into_inner().map_err(|e| e.to_string())?;
            writer.flush().map_err(|e| e.to_string())?;
            bytes = writer.bytes;
        }
        if cancel.load(Ordering::Acquire) {
            return Ok(Outcome {
                frames,
                bytes,
                limited: false,
                cancelled: true,
            });
        }
        temporary.persist(&path).map_err(|e| e.to_string())?;
        Ok(Outcome {
            frames,
            bytes,
            limited,
            cancelled: false,
        })
    })();
    let _ = send.send(Event::Finished(result));
}
impl App {
    fn gif_source_len(&self) -> usize {
        if !self.field_frames.is_empty() && (self.plane_index == 0 || self.plane_frames.is_empty())
        {
            self.field_frames.len()
        } else if !self.plane_frames.is_empty() {
            self.plane_frames.len()
        } else {
            self.gif_options.flow_frames
        }
    }
    pub(super) fn begin_export(&mut self, gif: bool) {
        if self.export.is_some() {
            return;
        }
        if gif {
            if self.streamlines.is_empty()
                && self.plane_frames.is_empty()
                && self.field_frames.is_empty()
            {
                self.seed_flow();
            }
            if self.streamlines.is_empty()
                && self.plane_frames.is_empty()
                && self.field_frames.is_empty()
            {
                self.message("GIF needs recorded fields, plane frames or seeded flow paths.");
                return;
            }
            self.gif_options.first = 1;
            self.gif_options.last = self.gif_source_len();
            self.show_gif_options = true;
        } else {
            self.start_image_export(false);
        }
    }
    fn export_indices(&self) -> Vec<usize> {
        let count = self.gif_source_len();
        let o = &self.gif_options;
        let first = o.first.saturating_sub(1).min(count.saturating_sub(1));
        let last = o.last.max(1).min(count).saturating_sub(1);
        if first > last {
            return Vec::new();
        };
        let step = o
            .stride
            .max(1)
            .max((last - first + 1).div_ceil(o.max_frames.max(1)));
        let mut indices: Vec<_> = (first..=last).step_by(step).take(o.max_frames).collect();
        if let Some(end) = indices.last_mut() {
            *end = last;
        }
        indices
    }
    pub(super) fn gif_settings_window(&mut self, ctx: &egui::Context) {
        if self.export.is_some() || !self.show_gif_options {
            return;
        }
        let mut open = true;
        let mut start = false;
        egui::Window::new("Animated GIF export").open(&mut open).default_width(430.).collapsible(false).frame(widgets::dialog_frame(ctx)).show(ctx,|ui|{
            let count=self.gif_source_len();ui.label(format!("{count} source frames available"));
            let o=&mut self.gif_options;
            ui.horizontal(|ui|{ui.label("Width × height (pixels)");ui.add(egui::DragValue::new(&mut o.width).range(160..=1920));ui.add(egui::DragValue::new(&mut o.height).range(120..=1080));});
            ui.horizontal(|ui|{ui.label("Playback FPS");ui.add(egui::DragValue::new(&mut o.fps).range(1..=50));});
            ui.horizontal(|ui|{ui.label("First / last source frame");ui.add(egui::DragValue::new(&mut o.first).range(1..=count.max(1)));ui.add(egui::DragValue::new(&mut o.last).range(1..=count.max(1)));});
            ui.horizontal(|ui|{ui.label("Sample every N frames");ui.add(egui::DragValue::new(&mut o.stride).range(1..=count.max(1)));});
            ui.horizontal(|ui|{ui.label("Maximum output frames");ui.add(egui::DragValue::new(&mut o.max_frames).range(2..=5000));});
            ui.horizontal(|ui|{ui.label("Size limit (MiB, checked after each frame)");ui.add(egui::DragValue::new(&mut o.max_mb).range(1..=1024));});
            ui.checkbox(&mut o.fast_palette,"Fast fixed palette (reduced colour fidelity)");
            let indices=self.export_indices();ui.label(format!("{} output frames · {:.1} s · fixed dimensions",indices.len(),indices.len()as f64/self.gif_options.fps as f64));
            ui.weak("Frames are sampled across the chosen range when the limit is reached. GIF playback uses fixed FPS, not physical-time spacing. The size limit may be exceeded by one encoded frame. Cancel discards this export and preserves any existing destination.");
            start=ui.add_enabled(!indices.is_empty(),egui::Button::new("Choose file and export…")).clicked();
        });
        self.show_gif_options = open;
        if start {
            self.show_gif_options = false;
            self.start_image_export(true);
        }
    }
    fn start_image_export(&mut self, gif: bool) {
        let Some(path) = rfd::FileDialog::new()
            .set_file_name(if gif { "flow.gif" } else { "field.png" })
            .save_file()
        else {
            return;
        };
        let source = if !gif {
            Source::Still
        } else if !self.field_frames.is_empty()
            && (self.plane_index == 0 || self.plane_frames.is_empty())
        {
            Source::Field(self.field_frames.clone())
        } else if !self.plane_frames.is_empty() {
            Source::Plane(self.plane_frames.clone())
        } else {
            Source::Flow
        };
        let indices = if gif { self.export_indices() } else { vec![0] };
        if indices.is_empty() {
            self.message("Empty export frame range.");
            return;
        }
        let restore = Restore {
            camera: self.camera.clone(),
            live: self.live_fields,
            flow: self.flow_playing,
            fields: self.field_playing,
            planes: self.plane_playing,
            field: self.field_frame,
            plane: self.plane_frame,
            phase: self.flow_phase,
        };
        self.live_fields = false;
        self.flow_playing = false;
        self.field_playing = false;
        self.plane_playing = false;
        let options = self.gif_options.clone();
        let cancel = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::sync_channel(1);
        let (events, receive) = mpsc::channel();
        let worker_path = path.clone();
        let worker_options = options.clone();
        let worker_cancel = cancel.clone();
        let total = indices.len();
        std::thread::spawn(move || {
            worker(
                worker_path,
                gif,
                worker_options,
                total,
                worker_cancel,
                rx,
                events,
            )
        });
        let id = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos() as u64;
        let mesh = if matches!(&source, Source::Field(_)) {
            self.result
                .as_ref()
                .map(|d| Arc::new(d.mesh().clone()))
                .or_else(|| self.session.mesh().map(|m| Arc::new(m.mesh.clone())))
        } else {
            None
        };
        self.export = Some(Job {
            id,
            path,
            source,
            indices,
            position: 0,
            frames: 0,
            bytes: 0,
            pending: false,
            requested_at: None,
            encoding: false,
            settle: 3,
            started: Instant::now(),
            cancel,
            send: tx,
            mesh,
            loading: false,
            receive,
            options,
            restore,
            gif,
            failure: None,
        });
        self.select_export_frame();
    }
    fn select_export_frame(&mut self) {
        let run = self.result_run.clone();
        let plane = if self.plane_index > 0 {
            self.project
                .plane_monitors
                .get(self.plane_index - 1)
                .cloned()
        } else {
            None
        };
        let physical = self
            .project
            .runs
            .iter()
            .find(|r| r.id == run)
            .map(|r| r.time_steps > 0)
            .unwrap_or(self.project.solver.time.is_transient());
        let Some(job) = &mut self.export else { return };
        let index = job.indices[job.position];
        let command = match &job.source {
            Source::Field(paths) => match &job.mesh {
                Some(mesh) => Some(Command::Field {
                    path: paths[index].clone(),
                    mesh: mesh.clone(),
                    run,
                    plane: plane.map(Box::new),
                    physical,
                }),
                None => {
                    job.failure = Some("No matching mesh for field export".into());
                    job.cancel.store(true, Ordering::Release);
                    None
                }
            },
            Source::Plane(paths) => Some(Command::Plane(paths[index].clone())),
            Source::Flow => {
                self.flow_phase = index as f64 / job.options.flow_frames.max(1) as f64;
                None
            }
            Source::Still => None,
        };
        if let Some(command) = command {
            match job.send.try_send(command) {
                Ok(()) => job.loading = true,
                Err(_) => {
                    job.failure = Some("Frame loader unavailable".into());
                    job.cancel.store(true, Ordering::Release);
                }
            }
        }
    }
    pub(super) fn export_aspect(&self) -> Option<f32> {
        self.export
            .as_ref()
            .filter(|j| j.gif)
            .map(|j| j.options.width as f32 / j.options.height as f32)
    }
    pub(super) fn export_footer(&mut self, ui: &mut egui::Ui) {
        if self.export.is_none() {
            return;
        }
        egui::Panel::bottom("image-export-progress").show(ui, |ui| {
            if let Some(job) = &self.export {
                ui.horizontal_wrapped(|ui| {
                    let cancelled = job.cancel.load(Ordering::Acquire);
                    ui.label(if cancelled {
                        "Cancelling export…"
                    } else {
                        "Exporting image frames"
                    });
                    ui.label(format!(
                        "{} / {} · {:.1} MiB · {:.0} s",
                        job.frames,
                        job.indices.len(),
                        job.bytes as f64 / 1048576.,
                        job.started.elapsed().as_secs_f64()
                    ));
                    if ui
                        .add_enabled(!cancelled, egui::Button::new("Cancel export · Esc"))
                        .clicked()
                    {
                        job.cancel.store(true, Ordering::Release);
                    }
                });
                ui.add(
                    egui::ProgressBar::new(job.frames as f32 / job.indices.len().max(1) as f32)
                        .show_percentage(),
                );
            }
        });
    }
    pub(super) fn request_export_capture(&mut self, ctx: &egui::Context) {
        let Some(job) = &mut self.export else { return };
        if job.cancel.load(Ordering::Acquire)
            || job.pending
            || job.encoding
            || job.loading
            || job.frames >= job.indices.len()
        {
            return;
        }
        if job.settle > 0 {
            job.settle -= 1;
            return;
        }
        let Some(rect) = self.viewport_rect else {
            return;
        };
        job.pending = true;
        job.requested_at = Some(Instant::now());
        ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::new(
            Ticket {
                id: job.id,
                rect,
                scale: ctx.pixels_per_point(),
            },
        )));
    }
    pub(super) fn export_logic(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            if let Some(j) = &self.export {
                j.cancel.store(true, Ordering::Release);
            }
        }
        let screenshots = ctx.input(|i| {
            i.events
                .iter()
                .filter_map(|e| match e {
                    egui::Event::Screenshot {
                        image, user_data, ..
                    } => user_data
                        .data
                        .as_ref()
                        .and_then(|d| d.downcast_ref::<Ticket>())
                        .map(|t| (t.clone(), image.clone())),
                    _ => None,
                })
                .collect::<Vec<_>>()
        });
        for (ticket, image) in screenshots {
            if let Some(job) = &mut self.export {
                if ticket.id != job.id || !job.pending || job.cancel.load(Ordering::Acquire) {
                    continue;
                }
                match job
                    .send
                    .try_send(Command::Capture(Capture { ticket, image }))
                {
                    Ok(()) => {
                        job.pending = false;
                        job.requested_at = None;
                        job.encoding = true;
                    }
                    Err(TrySendError::Full(_)) => {
                        job.cancel.store(true, Ordering::Release);
                    }
                    Err(TrySendError::Disconnected(_)) => {
                        job.cancel.store(true, Ordering::Release);
                    }
                }
            }
        }
        if let Some(job) = &mut self.export {
            if job.pending
                && job
                    .requested_at
                    .is_some_and(|t| t.elapsed() > Duration::from_secs(30))
            {
                job.failure =
                    Some("Screenshot timed out. Restore the window and retry export.".into());
                job.cancel.store(true, Ordering::Release);
            }
        }
        let mut finished = None;
        let mut advance = false;
        let mut loaded = None;
        if let Some(job) = &mut self.export {
            loop {
                match job.receive.try_recv() {
                    Ok(Event::Loaded(frame)) => {
                        job.loading = false;
                        if !job.cancel.load(Ordering::Acquire) {
                            loaded = Some(*frame);
                        }
                    }
                    Ok(Event::Encoded { frames, bytes }) => {
                        job.frames = frames;
                        job.bytes = bytes;
                        job.encoding = false;
                        if frames < job.indices.len() {
                            job.position = frames;
                            job.settle = 0;
                            advance = true;
                        }
                    }
                    Ok(Event::Finished(result)) => {
                        finished = Some(result);
                        break;
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        finished = Some(Err("Image export worker disconnected.".into()));
                        break;
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                }
            }
        }
        if finished.is_none() {
            if let Some(frame) = loaded {
                match frame {
                    Loaded::Field(frame) => {
                        let FieldFrame {
                            index,
                            data,
                            cache,
                            snapshot,
                        } = *frame;
                        self.result_iteration = index;
                        for probe in &mut self.probes {
                            if let Some(sample) = data.probe_cell(probe.cell_index) {
                                *probe = sample;
                            }
                        }
                        self.result = Some(data);
                        self.result_cache = Some(cache);
                        if let Some(snapshot) = snapshot {
                            self.plane_snapshot = Some(snapshot);
                        }
                    }
                    Loaded::Plane(plane) => {
                        let mut plane = *plane;
                        if let Some(current) = &self.plane_snapshot {
                            if current.plane.id == plane.plane.id {
                                plane.plane.display = current.plane.display.clone();
                            }
                        }
                        for probe in &mut self.probes {
                            if let Some(sample) = plane
                                .samples
                                .iter()
                                .find(|s| s.geometry.cell == probe.cell_index)
                            {
                                probe.pressure = sample.pressure;
                                probe.velocity = sample.velocity;
                                probe.velocity_magnitude = sample.velocity.norm();
                            }
                        }
                        self.plane_snapshot = Some(plane);
                    }
                }
            }
        }
        if let Some(result) = finished {
            if let Some(job) = self.export.take() {
                self.camera = job.restore.camera.clone();
                self.live_fields = job.restore.live;
                self.flow_playing = job.restore.flow;
                self.field_playing = job.restore.fields;
                self.plane_playing = job.restore.planes;
                self.flow_phase = job.restore.phase;
                self.field_frame = job.restore.field;
                self.plane_frame = job.restore.plane;
                match &job.source {
                    Source::Field(_) => {
                        self.read_field_frame();
                    }
                    Source::Plane(_) => {
                        self.read_plane_frame();
                    }
                    _ => {}
                }
                if let Some(error) = &job.failure {
                    self.message(format!("Export failed: {error}"));
                } else {
                    match result {
                        Ok(o) if o.cancelled => self.message("Export cancelled; temporary frames discarded and existing destination preserved."),
                        Ok(o) => self.message(format!("Exported {} · {} frames · {:.1} MiB{}",job.path.display(),o.frames,o.bytes as f64/1048576.,if o.limited{" · stopped at size limit"}else{""})),
                        Err(e) => self.message(format!("Export failed: {e}")),
                    }
                }
            }
        } else if advance {
            self.select_export_frame();
        }
    }
}
