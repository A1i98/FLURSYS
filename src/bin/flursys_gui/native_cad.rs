//! Hosts the complete CAD workbench in the Geometry page. The worker owns its document,
//! browser, command dialogs and GPU renderer; Slint only transports pixels and input.
use super::*;
use egui_wgpu::wgpu;
use std::sync::mpsc::{self, Receiver, SyncSender};
use std::time::Instant;

enum Request {
    Input(egui::Event),
    Resize(u32, u32),
    Visible(bool),
    Project(String, Option<String>),
    Rename(String, String),
    Publish(i32, Option<String>),
    Browser(bool),
    Fit,
}
enum Reply {
    Frame(u32, u32, Vec<u8>),
    Document(String, String),
    Published(
        String,
        i32,
        Box<Result<(flursys::GeometryTopology, String), String>>,
    ),
    Unchanged(String, i32),
    Planes(
        String,
        Vec<flursys::workbench::plane_monitor::FlowPlaneMonitor>,
        Option<String>,
    ),
    Error(String),
}
struct FrameTarget {
    size: (u32, u32),
    texture: wgpu::Texture,
    view: wgpu::TextureView,
    read: wgpu::Buffer,
    stride: u32,
}
struct PendingFrame {
    read: wgpu::Buffer,
    ready: Receiver<Result<(), wgpu::BufferAsyncError>>,
    width: u32,
    height: u32,
    stride: u32,
    started: Instant,
}
impl Drop for PendingFrame {
    fn drop(&mut self) {
        self.read.unmap();
    }
}
struct Gpu {
    pending: Option<PendingFrame>,
    target: Option<FrameTarget>,
    adapter_name: String,
    device: wgpu::Device,
    queue: wgpu::Queue,
    renderer: egui_wgpu::Renderer,
    ctx: egui::Context,
    app: cad_workbench::SolveApp,
    start: Instant,
    modifiers: egui::Modifiers,
}
impl Gpu {
    fn new() -> Result<Self, String> {
        let adapter = super::render_device::adapter(&super::render_device::options().adapter)?;
        let adapter_name = adapter.get_info().name;
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            required_limits: wgpu::Limits::downlevel_defaults().using_resolution(adapter.limits()),
            ..Default::default()
        }))
        .map_err(|e| e.to_string())?;
        let mut renderer =
            egui_wgpu::Renderer::new(&device, wgpu::TextureFormat::Rgba8Unorm, Default::default());
        let target = cad_workbench::gpu::GpuTarget {
            format: wgpu::TextureFormat::Rgba8Unorm,
            depth: None,
            samples: 1,
            tess: false,
        };
        renderer
            .callback_resources
            .insert(cad_workbench::gpu::Renderer::new(&device, target, None));
        let ctx = egui::Context::default();
        // Font installation takes effect at the next frame boundary. Register
        // named CAD families before run_ui; installing inside ui() is too late
        // for ui-bold/ui-semibold on the first frame of an embedded context.
        cad_workbench::fonts::setup_bundled_fonts(&ctx);
        cad_workbench::fonts::load_system_fonts_in_background(&ctx);
        let mut app = cad_workbench::SolveApp::new(cad_engine::Session::default(), services());
        app.home.open = false;
        app.custom_titlebar = false;
        app.ui.show_timeline = false;
        app.viewport.gpu = Some(target);
        app.preferences.timeline_left = true;
        Ok(Self {
            target: None,
            pending: None,
            adapter_name,
            device,
            queue,
            renderer,
            ctx,
            app,
            start: Instant::now(),
            modifiers: Default::default(),
        })
    }
    fn project(&mut self, document: Option<String>) -> Result<(), String> {
        self.pending = None;
        let doc = match document {
            Some(s) => cad_engine::doc::Document::from_json(&s).map_err(|e| e.to_string())?,
            None => Default::default(),
        };
        self.app = cad_workbench::SolveApp::new(cad_engine::Session::new(doc), services());
        self.app.home.open = false;
        self.app.custom_titlebar = false;
        self.app.ui.show_timeline = false;
        self.app.home.open = false;
        self.app.viewport.gpu = Some(cad_workbench::gpu::GpuTarget {
            format: wgpu::TextureFormat::Rgba8Unorm,
            depth: None,
            samples: 1,
            tess: false,
        });
        self.app.fit_view();
        Ok(())
    }
    fn frame(&mut self, width: u32, height: u32, events: Vec<egui::Event>) -> Result<(), String> {
        if self.pending.is_some() {
            return Err("CAD frame already in flight".into());
        }
        for event in &events {
            match event {
                egui::Event::ModifiersChanged(m)
                | egui::Event::PointerButton { modifiers: m, .. }
                | egui::Event::Key { modifiers: m, .. }
                | egui::Event::MouseWheel { modifiers: m, .. } => self.modifiers = *m,
                _ => {}
            }
        }
        let mut input = egui::RawInput {
            screen_rect: Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(width as f32, height as f32),
            )),
            time: Some(self.start.elapsed().as_secs_f64()),
            events,
            focused: true,
            ..Default::default()
        };
        self.app.raw_input_hook(&mut input);
        self.app.ui.show_timeline = false;
        self.app.home.open = false;
        let app = &mut self.app;
        cad_workbench::frame::set_left_docking(&self.ctx, true);
        let mut out = self.ctx.run_ui(input, |ui| {
            app.logic(ui.ctx());
            app.ui(ui);
        });
        for command in &out.platform_output.commands {
            if let egui::OutputCommand::CopyText(text) = command {
                if let Ok(mut clipboard) = arboard::Clipboard::new() {
                    let _ = clipboard.set_text(text);
                }
            }
        }
        let jobs = self.ctx.tessellate(out.shapes, out.pixels_per_point);
        for (id, deltas) in &out.textures_delta.set {
            for delta in deltas {
                self.renderer
                    .update_texture(&self.device, &self.queue, *id, delta);
            }
        }
        let started = Instant::now();
        let size = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };
        if self
            .target
            .as_ref()
            .is_none_or(|t| t.size != (width, height))
        {
            let texture = self.device.create_texture(&wgpu::TextureDescriptor {
                label: Some("Reusable CAD frame"),
                size,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8Unorm,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
                view_formats: &[],
            });
            let view = texture.create_view(&Default::default());
            let stride = (width * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
                * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
            let read = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("Reusable CAD readback"),
                size: u64::from(stride) * u64::from(height),
                usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.target = Some(FrameTarget {
                size: (width, height),
                texture,
                view,
                read,
                stride,
            });
        }
        let target = self
            .target
            .as_ref()
            .ok_or("CAD render target unavailable")?;
        let view = &target.view;
        let texture = &target.texture;
        let read = &target.read;
        let stride = target.stride;
        let mut encoder = self.device.create_command_encoder(&Default::default());
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [width, height],
            pixels_per_point: out.pixels_per_point,
        };
        let commands =
            self.renderer
                .update_buffers(&self.device, &self.queue, &mut encoder, &jobs, &screen);
        {
            let pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("CAD workbench"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                ..Default::default()
            });
            self.renderer
                .render(&mut pass.forget_lifetime(), &jobs, &screen);
        }
        encoder.copy_texture_to_buffer(
            texture.as_image_copy(),
            wgpu::TexelCopyBufferInfo {
                buffer: &read,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(stride),
                    rows_per_image: Some(height),
                },
            },
            size,
        );
        self.queue.submit(
            commands
                .into_iter()
                .chain(std::iter::once(encoder.finish())),
        );
        let (tx, rx) = mpsc::channel();
        let slice = read.slice(..);
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        self.pending = Some(PendingFrame {
            read: read.clone(),
            ready: rx,
            width,
            height,
            stride,
            started,
        });
        for id in &out.textures_delta.free {
            self.renderer.free_texture(id);
        }
        out.textures_delta.clear();
        Ok(())
    }
    fn poll_frame(&mut self) -> Result<Option<(u32, u32, Vec<u8>)>, String> {
        let Some(pending) = self.pending.as_ref() else {
            return Ok(None);
        };
        self.device
            .poll(wgpu::PollType::Poll)
            .map_err(|e| e.to_string())?;
        match pending.ready.try_recv() {
            Ok(result) => result.map_err(|e| e.to_string())?,
            Err(mpsc::TryRecvError::Empty) => {
                if pending.started.elapsed() > Duration::from_secs(5) {
                    self.pending = None;
                    return Err("CAD GPU readback exceeded five seconds".into());
                }
                return Ok(None);
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.pending = None;
                return Err("CAD frame callback disconnected".into());
            }
        }
        let pending = self.pending.take().ok_or("CAD frame unavailable")?;
        let slice = pending.read.slice(..);
        let bytes = slice.get_mapped_range().map_err(|e| e.to_string())?;
        let mut rgba = Vec::with_capacity((pending.width * pending.height * 4) as usize);
        for row in bytes
            .chunks(pending.stride as usize)
            .take(pending.height as usize)
        {
            rgba.extend_from_slice(&row[..(pending.width * 4) as usize]);
        }
        drop(bytes);
        super::render_device::record(pending.started, rgba.len(), &self.adapter_name);
        Ok(Some((pending.width, pending.height, rgba)))
    }
    fn plane_catalog(
        &self,
    ) -> (
        Vec<flursys::workbench::plane_monitor::FlowPlaneMonitor>,
        Option<String>,
    ) {
        use flursys::workbench::plane_monitor::FlowPlaneMonitor;
        let model =
            cad_engine::doc::world_state(&self.app.session.doc, &self.app.session.model.state());
        let mut planes = Vec::new();
        let mut add = |id: String, name: String, p: cad_engine::geom::Plane| {
            let v = |a: cad_engine::geom::Vec3| Vec3::new(a.x, a.y, a.z);
            planes.push(FlowPlaneMonitor {
                id,
                name,
                origin: v(p.origin) * 0.001,
                u: v(p.x),
                v: v(p.y),
                enabled: false,
                every: 10,
                display: Default::default(),
            });
        };
        for (name, plane) in [
            ("XY", cad_engine::geom::Plane::XY),
            ("XZ", cad_engine::geom::Plane::XZ),
            ("YZ", cad_engine::geom::Plane::YZ),
        ] {
            add(format!("origin-{name}"), name.into(), plane);
        }
        for c in &model.construct {
            if let cad_engine::doc::construct::ConstructGeom::Plane(p) = c.geom {
                add(format!("datum-{}", c.feature), c.name.clone(), p);
            }
        }
        for s in &model.sketches {
            add(
                format!("sketch-{}", s.feature),
                format!("{} · sketch plane", s.name),
                s.plane,
            );
        }
        let selected = if self.app.dialog.is_some() || self.app.session.active_sketch.is_some() {
            None
        } else {
            self.app
                .session
                .selection
                .iter()
                .rev()
                .find_map(|s| match s {
                    cad_engine::Sel::Plane { name } => planes
                        .iter()
                        .find(|p| p.name == *name)
                        .map(|p| p.id.clone()),
                    cad_engine::Sel::Feature { id } => planes
                        .iter()
                        .find(|p| p.id == format!("datum-{id}") || p.id == format!("sketch-{id}"))
                        .map(|p| p.id.clone()),
                    _ => None,
                })
        };
        (planes, selected)
    }
    fn geometry(&self) -> Result<flursys::GeometryTopology, String> {
        if self.app.session.active_sketch.is_some()
            || self.app.dialog.is_some()
            || self.app.applying.is_some()
        {
            return Err(
                "Finish or cancel the current sketch / feature before entering the CFD workflow."
                    .into(),
            );
        }
        let model =
            cad_engine::doc::world_state(&self.app.session.doc, &self.app.session.model.state());
        let mut geometry = flursys::GeometryTopology::new();
        if !model.bodies.is_empty() {
            // CAD lengths are millimetres; solver geometry is always metres.
            for body in &model.bodies {
                let scaled = cad_kernel::transform_matrix(
                    &body.body,
                    [
                        [0.001, 0., 0., 0.],
                        [0., 0.001, 0., 0.],
                        [0., 0., 0.001, 0.],
                        [0., 0., 0., 1.],
                    ],
                )
                .map_err(|e| e.to_string())?;
                let text = scaled.host_brep_json().map_err(|e| e.to_string())?;
                geometry
                    .cad_import_brep(&text, &body.name)
                    .map_err(|e| e.to_string())?;
            }
        } else {
            // Evaluated sketches include solved constraints, exact circles/arcs and inner loops.
            for sketch in &model.sketches {
                if sketch.profiles.is_empty() {
                    continue;
                }
                let p = sketch.plane;
                let v = |p: cad_engine::geom::Vec3| Vec3::new(p.x, p.y, p.z);
                let frame = flursys::CadPlaneFrame {
                    origin: v(p.origin) * 0.001,
                    x_axis: v(p.x),
                    y_axis: v(p.y),
                    normal: v(p.x.cross(p.y)),
                };
                geometry
                    .cad_import_drawing_frame(frame, sketch.sketch.clone())
                    .map_err(|e| e.to_string())?;
            }
        }
        Ok(geometry)
    }
}
fn services() -> cad_workbench::Services {
    cad_workbench::Services {
        pick_open: Some(Box::new(|extensions| {
            let extensions = extensions.iter().map(|s| s.to_string()).collect::<Vec<_>>();
            Box::pin(async move {
                rfd::AsyncFileDialog::new()
                    .add_filter("CAD files", &extensions)
                    .pick_file()
                    .await
                    .map(|f| f.path().to_string_lossy().to_string())
            })
        })),
        pick_save: Some(Box::new(|name, extensions| {
            let name = name.to_string();
            let extensions = extensions.iter().map(|s| s.to_string()).collect::<Vec<_>>();
            Box::pin(async move {
                rfd::AsyncFileDialog::new()
                    .set_file_name(name)
                    .add_filter("CAD files", &extensions)
                    .save_file()
                    .await
                    .map(|f| f.path().to_string_lossy().to_string())
            })
        })),
        ..Default::default()
    }
}
fn worker(rx: Receiver<Request>, tx: SyncSender<Reply>) {
    let mut gpu = match Gpu::new() {
        Ok(g) => g,
        Err(e) => {
            let _ = tx.send(Reply::Error(format!("CAD renderer could not start: {e}")));
            return;
        }
    };
    let (mut width, mut height) = (1100, 760);
    let mut visible = true;
    let mut last_frame = Instant::now();
    let mut project = String::new();
    let mut revision = u64::MAX;
    let mut events = Vec::new();
    let mut publish = Vec::new();
    let mut catalog = None;
    let mut catalog_input = None;
    let mut catalog_model = None;
    #[cfg(debug_assertions)]
    let mut capture = std::env::var_os("FLURSYS_CAD_DEBUG_CAPTURE");
    loop {
        let first = match rx.recv_timeout(if visible {
            Duration::from_secs_f64(1.0 / f64::from(super::render_device::options().fps))
        } else {
            Duration::from_millis(200)
        }) {
            Ok(r) => Some(r),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(_) => return,
        };
        let requests = first.into_iter().chain(rx.try_iter()).collect::<Vec<_>>();
        let had_requests = !requests.is_empty();
        for request in requests {
            match request {
                Request::Input(e) => events.push(e),
                Request::Resize(w, h) => {
                    width = w.clamp(320, 4096);
                    height = h.clamp(240, 2160);
                }
                Request::Visible(v) => visible = v,
                Request::Fit => gpu.app.fit_view(),
                Request::Browser(show) => gpu.app.ui.show_browser = show,
                Request::Project(id, doc) => {
                    project = id;
                    revision = u64::MAX;
                    catalog = None;
                    catalog_input = None;
                    if let Err(e) = gpu.project(doc) {
                        let _ = tx.send(Reply::Error(e));
                    }
                }
                Request::Rename(id, name) => {
                    if id == project && gpu.app.session.doc.name != name {
                        std::sync::Arc::make_mut(&mut gpu.app.session.doc).name = name;
                        gpu.app.session.revision = gpu.app.session.revision.wrapping_add(1);
                    }
                }
                Request::Publish(step, hash) => publish.push((step, hash)),
            }
        }
        match gpu.poll_frame() {
            Ok(Some((frame_width, frame_height, rgba))) => {
                #[cfg(debug_assertions)]
                if let Some(path) = capture.take() {
                    if let Ok(file) = std::fs::File::create(path) {
                        let mut png = png::Encoder::new(file, frame_width, frame_height);
                        png.set_color(png::ColorType::Rgba);
                        png.set_depth(png::BitDepth::Eight);
                        if let Ok(mut writer) = png.write_header() {
                            let _ = writer.write_image_data(&rgba);
                        }
                    }
                }
                if visible {
                    let _ = tx.try_send(Reply::Frame(frame_width, frame_height, rgba));
                }
            }
            Ok(None) => {}
            Err(error) => {
                gpu.pending = None;
                gpu.target = None;
                let _ = tx.send(Reply::Error(format!("CAD GPU readback failed: {error}")));
                visible = false;
            }
        }
        if !visible || gpu.pending.is_some() {
            continue;
        }
        // Background evaluation gets a heartbeat without redrawing an idle
        // viewport at full FPS. Input and egui repaint requests draw promptly.
        if !had_requests
            && events.is_empty()
            && publish.is_empty()
            && !gpu.ctx.has_requested_repaint()
            && last_frame.elapsed() < Duration::from_millis(250)
        {
            continue;
        }
        let frame_period =
            Duration::from_secs_f64(1.0 / f64::from(super::render_device::options().fps));
        if let Some(delay) = frame_period.checked_sub(last_frame.elapsed()) {
            std::thread::sleep(delay);
        }
        last_frame = Instant::now();
        if let Err(error) = gpu.frame(width, height, std::mem::take(&mut events)) {
            gpu.pending = None;
            gpu.target = None;
            let _ = tx.send(Reply::Error(format!("CAD rendering failed: {error}")));
            visible = false;
        }
        if gpu.app.session.revision != revision {
            revision = gpu.app.session.revision;
            let _ = tx.send(Reply::Document(
                project.clone(),
                gpu.app.session.doc.to_json(),
            ));
        }
        let key = (
            gpu.app.session.revision,
            gpu.app.session.selection.clone(),
            gpu.app.dialog.is_some() || gpu.app.session.active_sketch.is_some(),
        );
        let evaluated_model = gpu.app.session.model.state();
        if catalog_input.as_ref() != Some(&key)
            || catalog_model
                .as_ref()
                .is_none_or(|old| !std::sync::Arc::ptr_eq(old, &evaluated_model))
        {
            catalog_model = Some(evaluated_model);
            catalog_input = Some(key);
            let planes = gpu.plane_catalog();
            if catalog.as_ref() != Some(&planes) {
                catalog = Some(planes.clone());
                let _ = tx.send(Reply::Planes(project.clone(), planes.0, planes.1));
            }
        }
        for (step, previous) in publish.drain(..) {
            let model =
                cad_engine::doc::world_state(&gpu.app.session.doc, &gpu.app.session.model.state());
            let hash = match model.host_geometry_hash() {
                Ok(h) => h,
                Err(e) => {
                    let _ = tx.send(Reply::Published(
                        project.clone(),
                        step,
                        Box::new(Err(e.to_string())),
                    ));
                    continue;
                }
            };
            if previous.as_ref() == Some(&hash)
                && gpu.app.session.active_sketch.is_none()
                && gpu.app.dialog.is_none()
                && gpu.app.applying.is_none()
            {
                let _ = tx.send(Reply::Unchanged(project.clone(), step));
            } else {
                let result = gpu.geometry().map(|g| (g, hash));
                let _ = tx.send(Reply::Published(project.clone(), step, Box::new(result)));
            }
        }
    }
}
fn modifiers(ctrl: bool, shift: bool, alt: bool) -> egui::Modifiers {
    egui::Modifiers {
        ctrl,
        shift,
        alt,
        command: ctrl,
        mac_cmd: false,
    }
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) -> Timer {
    let (tx, rx) = mpsc::channel();
    let (out_tx, out_rx) = mpsc::sync_channel(3);
    std::thread::spawn(move || {
        let errors = out_tx.clone();
        if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| worker(rx, out_tx))).is_err() {
            let _ = errors.send(Reply::Error(
                "The CAD worker stopped unexpectedly. Your last saved document is preserved."
                    .into(),
            ));
        }
    });
    let sender = tx.clone();
    ui.on_cad_native_resize(move |w, h| {
        let _ = sender.send(Request::Resize(w.max(1.) as u32, h.max(1.) as u32));
    });
    let sender = tx.clone();
    ui.on_cad_native_pointer(move |kind, x, y, button, ctrl, shift, alt| {
        let pos = egui::pos2(x, y);
        let event = if kind == 0 {
            egui::Event::PointerMoved(pos)
        } else if kind == 3 {
            egui::Event::PointerGone
        } else {
            egui::Event::PointerButton {
                pos,
                button: match button {
                    1 => egui::PointerButton::Secondary,
                    2 => egui::PointerButton::Middle,
                    _ => egui::PointerButton::Primary,
                },
                pressed: kind == 1,
                modifiers: modifiers(ctrl, shift, alt),
            }
        };
        let _ = sender.send(Request::Input(event));
    });
    let sender = tx.clone();
    ui.on_cad_native_wheel(move |x, y, ctrl, shift, alt| {
        let _ = sender.send(Request::Input(egui::Event::MouseWheel {
            phase: egui::TouchPhase::Move,
            unit: egui::MouseWheelUnit::Point,
            delta: egui::vec2(x, y),
            modifiers: modifiers(ctrl, shift, alt),
        }));
    });
    let sender = tx.clone();
    let key_ui = ui.as_weak();
    ui.on_cad_native_key(move |text, pressed, ctrl, shift, alt| {
        let m = modifiers(ctrl, shift, alt);
        let _ = sender.send(Request::Input(egui::Event::ModifiersChanged(m)));
        let text = text.as_str();
        if pressed && ctrl && !alt {
            if let Some(ui) = key_ui.upgrade() {
                match text.to_ascii_lowercase().as_str() {
                    "s" => {
                        if shift {
                            ui.invoke_save_project_as();
                        } else {
                            ui.invoke_save_project_native();
                        }
                        return;
                    }
                    "o" => {
                        ui.invoke_open_project_native();
                        return;
                    }
                    "n" => {
                        ui.invoke_new_project();
                        return;
                    }
                    _ => {}
                }
            }
            if text.eq_ignore_ascii_case("c") {
                let _ = sender.send(Request::Input(egui::Event::Copy));
                return;
            }
            if text.eq_ignore_ascii_case("x") {
                let _ = sender.send(Request::Input(egui::Event::Cut));
                return;
            }
        }
        let special = [
            (slint::platform::Key::Return, egui::Key::Enter),
            (slint::platform::Key::Escape, egui::Key::Escape),
            (slint::platform::Key::Backspace, egui::Key::Backspace),
            (slint::platform::Key::Delete, egui::Key::Delete),
            (slint::platform::Key::Tab, egui::Key::Tab),
            (slint::platform::Key::LeftArrow, egui::Key::ArrowLeft),
            (slint::platform::Key::RightArrow, egui::Key::ArrowRight),
            (slint::platform::Key::UpArrow, egui::Key::ArrowUp),
            (slint::platform::Key::DownArrow, egui::Key::ArrowDown),
        ];
        let key = special
            .into_iter()
            .find_map(|(from, to)| (text == SharedString::from(from).as_str()).then_some(to))
            .or_else(|| egui::Key::from_name(&text.to_uppercase()));
        if pressed && ctrl && text.eq_ignore_ascii_case("v") {
            if let Ok(mut clipboard) = arboard::Clipboard::new() {
                if let Ok(value) = clipboard.get_text() {
                    let _ = sender.send(Request::Input(egui::Event::Paste(value)));
                    return;
                }
            }
        }
        if let Some(key) = key {
            let _ = sender.send(Request::Input(egui::Event::Key {
                key,
                physical_key: None,
                pressed,
                repeat: false,
                modifiers: m,
            }));
        }
        if pressed && !ctrl && !alt && text.chars().all(|c| !c.is_control() && (c as u32) < 0xe000)
        {
            let _ = sender.send(Request::Input(egui::Event::Text(text.to_string())));
        }
    });
    let sender = tx.clone();
    let shared = state.clone();
    let weak = ui.as_weak();
    ui.on_select_step(move |step| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if state.current_step == 0 && step != 0 {
            ui.set_cad_native_feedback("Preparing geometry for CFD…".into());
            let _ = sender.send(Request::Publish(
                step,
                state.workbench_project.cad_geometry_hash.clone(),
            ));
        } else {
            apply_workflow_step(&ui, &mut state, step);
        }
    });
    let sender = tx.clone();
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_reset_geometry_view(move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        if shared.borrow().current_step == 0 {
            let _ = sender.send(Request::Fit);
        } else {
            let mut state = shared.borrow_mut();
            state.view_3d = Default::default();
            refresh_ui(&ui, &state);
        }
    });
    let timer = Timer::default();
    let weak = ui.as_weak();
    let shared = state.clone();
    let mut project = String::new();
    let mut sent_title = None;
    let mut was_visible = false;
    let mut browser = true;
    timer.start(TimerMode::Repeated, Duration::from_millis(16), move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        let document_generation=format!("{}:{}",state.workbench_project.project_id,state.cad_reload_generation);
        if project != document_generation {
            project = document_generation;
            ui.set_examples_open(false);
            let _ = tx.send(Request::Project(
                project.clone(),
                state.workbench_project.cad_document.clone(),
            ));
            ui.set_cad_native_feedback("".into());
        }
        let title=(project.clone(),state.workbench_project.name.clone());
        if sent_title.as_ref()!=Some(&title) {
            let _=tx.send(Request::Rename(title.0.clone(),title.1.clone()));
            sent_title=Some(title);
        }
        let visible = state.current_step == 0 && !ui.get_examples_open();
        if visible != was_visible {
            let _ = tx.send(Request::Visible(visible));
            was_visible = visible;
        }
        if browser != ui.get_show_browser() {
            browser = ui.get_show_browser();
            let _ = tx.send(Request::Browser(browser));
        }
        let mut latest = None;
        while let Ok(reply) = out_rx.try_recv() {
            match reply {
                Reply::Frame(w, h, pixels) => latest = Some((w, h, pixels)),
                Reply::Document(id, doc) if id == project => {
                    if state.workbench_project.cad_document.as_ref() != Some(&doc) {
                        state.workbench_project.cad_document = Some(doc);
                        state.mark_workbench_dirty();
                    }
                }
                Reply::Published(id, step, result) if id == project => match *result {
                    Ok((geometry, hash)) => {
                        // The native document is authoritative. Old topology IDs cannot be
                        // reassigned to a different shape's CFD groups by their integer alone.
                        let mut candidate=state.workbench.to_project(state.workbench_project.name.clone());
                        candidate.mesh.dimension=if geometry.bodies().next().is_some() {MeshDimension::ThreeD}else{MeshDimension::TwoD};
                        candidate.geometry=geometry;
                        candidate.fluid_face=None;
                        candidate.named_selections.clear();
                        candidate.boundaries.clear();
                        candidate.mesh_recipe=Default::default();
                        match WorkbenchSession::from_project(&candidate) {
                            Ok(session)=>state.workbench=session,
                            Err(e)=>{ui.set_cad_native_feedback(e.to_string().into());continue;}
                        }
                        ui.set_mesh_dimension_index(if state.workbench.mesh_dimension()==MeshDimension::ThreeD {1}else{0});
                        state.wb_selected_targets.clear();
                        state.post.reset_result();state.active_result=None;state.result_render_cache=None;
                        let topology=state.workbench.geometry().clone();state.geometry_editor.fit_view(&topology);
                        state.log("CAD geometry updated. Select the CFD domain and define boundary groups in Mesh.");
                        rebuild_tree_rows(&mut state);
                        state.workbench_project.cad_geometry_hash = Some(hash);
                        state.mark_workbench_dirty();
                        ui.set_cad_native_feedback("".into());
                        apply_workflow_step(&ui, &mut state, step);
                    }
                    Err(e) => ui.set_cad_native_feedback(e.into()),
                },
                Reply::Unchanged(id, step) if id == project => {
                    ui.set_cad_native_feedback("".into());
                    apply_workflow_step(&ui, &mut state, step);
                }
                Reply::Planes(id,planes,selected) if id==project=>plane_monitors::refresh_catalog(&ui,&mut state,planes,selected),
                Reply::Error(e) => {
                    state.log(&e);
                    ui.set_cad_native_feedback(e.into());
                }
                _ => {}
            }
        }
        if let Some((w, h, pixels)) = latest {
            let image =
                slint::SharedPixelBuffer::<slint::Rgba8Pixel>::clone_from_slice(&pixels, w, h);
            ui.set_cad_native_image(slint::Image::from_rgba8(image));
        }
    });
    timer
}
