//! Independent display-device policy; never modifies the solver backend.
use egui_wgpu::wgpu;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock, RwLock};
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Options {
    pub cpu: bool,
    pub adapter: String,
    pub fps: u32,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            cpu: false,
            adapter: String::new(),
            fps: 30,
        }
    }
}
static OPTIONS: OnceLock<RwLock<Options>> = OnceLock::new();
static FRAME_NS: AtomicU64 = AtomicU64::new(0);
static FRAMES: AtomicU64 = AtomicU64::new(0);
static READBACK: AtomicU64 = AtomicU64::new(0);
static LAST: OnceLock<Mutex<String>> = OnceLock::new();
type Cache = Option<(Options, Result<Arc<cad_gpu::capture::Capturer>, String>)>;
static CAPTURE: OnceLock<Mutex<Cache>> = OnceLock::new();
pub fn options() -> Options {
    OPTIONS
        .get_or_init(Default::default)
        .read()
        .unwrap_or_else(|p| p.into_inner())
        .clone()
}
pub fn configure(mut options: Options) {
    options.fps = options.fps.clamp(5, 120);
    // Applying settings also retries an earlier adapter initialization failure.
    if let Ok(mut cache) = CAPTURE.get_or_init(Default::default).lock() {
        if cache.as_ref().is_some_and(|(_, result)| result.is_err()) {
            *cache = None;
        }
    }
    *OPTIONS
        .get_or_init(Default::default)
        .write()
        .unwrap_or_else(|p| p.into_inner()) = options;
}
pub fn record(start: std::time::Instant, bytes: usize, renderer: &str) {
    FRAME_NS.fetch_add(
        start.elapsed().as_nanos().min(u64::MAX as u128) as u64,
        Ordering::Relaxed,
    );
    FRAMES.fetch_add(1, Ordering::Relaxed);
    READBACK.fetch_add(bytes as u64, Ordering::Relaxed);
    *LAST
        .get_or_init(Default::default)
        .lock()
        .unwrap_or_else(|p| p.into_inner()) = renderer.into();
}
pub fn summary() -> String {
    let frames = FRAMES.load(Ordering::Relaxed);
    format!(
        "Renderer: {}\n{} frames · mean draw/readback {:.2} ms · {:.1} MiB GPU readback",
        LAST.get_or_init(Default::default)
            .lock()
            .unwrap_or_else(|p| p.into_inner()),
        frames,
        FRAME_NS.load(Ordering::Relaxed) as f64 * 1e-6 / frames.max(1) as f64,
        READBACK.load(Ordering::Relaxed) as f64 / 1048576.
    )
}
pub fn adapter(selector: &str) -> Result<wgpu::Adapter, String> {
    let instance =
        wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let name = selector.to_lowercase();
    pollster::block_on(instance.enumerate_adapters(wgpu::Backends::all()))
        .into_iter()
        .filter(|a| name.is_empty() || a.get_info().name.to_lowercase().contains(&name))
        .min_by_key(|a| match a.get_info().device_type {
            wgpu::DeviceType::DiscreteGpu => 0,
            wgpu::DeviceType::IntegratedGpu => 1,
            wgpu::DeviceType::VirtualGpu => 2,
            _ => 3,
        })
        .ok_or("No matching display adapter".into())
}
fn capture() -> Result<Arc<cad_gpu::capture::Capturer>, String> {
    let options = options();
    let mut cache = CAPTURE
        .get_or_init(Default::default)
        .lock()
        .map_err(|e| e.to_string())?;
    if let Some((old, result)) = cache.as_ref() {
        if old.adapter == options.adapter {
            return result.clone();
        }
    }
    let result = (|| {
        let a = adapter(&options.adapter)?;
        let (device, queue) = pollster::block_on(a.request_device(&wgpu::DeviceDescriptor {
            label: Some("FLURSYS display"),
            required_features: cad_gpu::depth_features(&a),
            required_limits: wgpu::Limits::downlevel_defaults().using_resolution(a.limits()),
            ..Default::default()
        }))
        .map_err(|e| e.to_string())?;
        cad_gpu::capture::Capturer::new(&a, device, queue)
            .map(Arc::new)
            .map_err(|e| e.to_string())
    })();
    *cache = Some((options, result.clone()));
    result
}
pub fn render(
    scene: &cad_render::Scene,
    camera: &cad_render::Camera,
    width: u32,
    height: u32,
    unlit: bool,
) -> Result<Vec<u8>, String> {
    let started = std::time::Instant::now();
    let cpu=||if unlit{cad_render::render_unlit(scene,camera,width as usize,height as usize)}else{cad_render::render(scene,camera,width as usize,height as usize)}.rgba;
    if options().cpu {
        let pixels = cpu();
        record(started, 0, "CPU");
        return Ok(pixels);
    }
    match capture().and_then(|c| {
        let pixels = if unlit {
            c.rgba_unlit(scene, camera, width, height)
        } else {
            c.rgba(scene, camera, width, height)
        }
        .map_err(|e| e.to_string())?;
        Ok((pixels, c.adapter_name().to_string()))
    }) {
        Ok((pixels, name)) => {
            record(started, pixels.len(), &name);
            Ok(pixels)
        }
        Err(e) => {
            let pixels = cpu();
            record(started, 0, &format!("CPU fallback: {e}"));
            Ok(pixels)
        }
    }
}
