//! Project compute controls and independent per-user display preferences.
use super::*;
use flursys::compute::{Backend, ComputeOptions, GpuPreconditioner};

pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_open_compute(move || {
        if let Some(ui) = weak.upgrade() {
            open(&ui, &shared.borrow());
        }
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_apply_compute(move || {
        let Some(ui) = weak.upgrade() else {
            return false;
        };
        let result = {
            let mut state = shared.borrow_mut();
            apply(&ui, &mut state)
        };
        match result {
            Ok(()) => {
                ui.invoke_save_preferences();
                true
            }
            Err(error) => {
                ui.set_compute_feedback(error.into());
                false
            }
        }
    });
}

fn open(ui: &MainWindow, state: &AppState) {
    let config = &state.workbench.solver_options().compute;
    let mut names = vec!["Automatic".to_string()];
    let mut info = match flursys::compute::devices() {
        Ok(devices) => {
            for device in &devices {
                if !names.contains(&device.name) {
                    names.push(device.name.clone());
                }
            }
            devices
                .iter()
                .map(|d| {
                    format!(
                        "{} · {} · {} · max buffer {} MiB",
                        d.name,
                        d.backend,
                        d.device_type,
                        d.max_buffer_bytes / 1048576
                    )
                })
                .collect::<Vec<_>>()
                .join("\n")
        }
        Err(error) => error,
    };
    let display = render_device::options();
    for name in [&config.adapter, &display.adapter] {
        if !name.is_empty() && !names.contains(name) {
            names.push(name.clone());
        }
    }
    ui.set_compute_gpu_index(names.iter().position(|n| n == &config.adapter).unwrap_or(0) as i32);
    ui.set_compute_display_index(
        names
            .iter()
            .position(|n| n == &display.adapter)
            .unwrap_or(0) as i32,
    );
    ui.set_compute_adapters(ModelRc::new(VecModel::from(
        names
            .into_iter()
            .map(SharedString::from)
            .collect::<Vec<_>>(),
    )));
    ui.set_compute_backend(match config.backend {
        Backend::Serial => 0,
        Backend::Parallel => 1,
        Backend::Gpu => 2,
    });
    ui.set_compute_threads(
        state
            .workbench
            .solver_options()
            .threads
            .min(i32::MAX as usize) as i32,
    );
    ui.set_compute_fallback(config.cpu_fallback);
    ui.set_compute_preconditioner(if config.gpu_preconditioner == GpuPreconditioner::Auto {
        0
    } else {
        1
    });
    ui.set_compute_restart(config.gpu_restart as i32);
    ui.set_compute_inner(config.gpu_inner_iterations as i32);
    ui.set_compute_sweeps(config.gpu_smoothing_sweeps as i32);
    ui.set_compute_timeout(config.gpu_timeout_ms as i32);
    ui.set_compute_basis_mb(config.gpu_host_basis_mb as i32);
    ui.set_compute_parallel_rows(config.parallel_min_rows.min(10000000) as i32);
    ui.set_compute_gpu_rows(config.gpu_min_rows.min(10000000) as i32);
    ui.set_compute_memory_percent((config.gpu_memory_fraction * 100.).round() as i32);
    ui.set_compute_cpu_display(display.cpu);
    ui.set_compute_fps(display.fps as i32);
    info.push_str(&format!(
        "\n{} CPU logical processors\n{}",
        std::thread::available_parallelism().map_or(1, |n| n.get()),
        render_device::summary()
    ));
    if let Some(p) = state.workbench.performance() {
        info.push_str(&format!("\nLast solve: {} · {:.3}s · {:.3e} cell-iterations/s\nSpMV {:.3}s · linear {:.3}s · GPU {:.3}s\n{}", p.actual_backend, p.elapsed_seconds, p.cell_iterations_per_second, p.spmv_seconds, p.linear_seconds, p.gpu_seconds, [p.fallback_reasons.join("\n"),p.gpu_methods.keys().cloned().collect::<Vec<_>>().join("\n"),p.gpu_notes.join("\n")].join("\n")));
    }
    ui.set_compute_information(info.into());
    ui.set_compute_feedback("".into());
    ui.set_compute_open(true);
}

fn apply(ui: &MainWindow, state: &mut AppState) -> Result<(), String> {
    if state.solving || state.meshing || state.post.mapping.busy() {
        return Err("Stop the active job before changing compute settings.".into());
    }
    let names = ui.get_compute_adapters();
    let name = |index: i32| {
        if index <= 0 {
            String::new()
        } else {
            names
                .row_data(index as usize)
                .map(|n| n.to_string())
                .unwrap_or_default()
        }
    };
    let options = ComputeOptions {
        backend: match ui.get_compute_backend() {
            0 => Backend::Serial,
            2 => Backend::Gpu,
            _ => Backend::Parallel,
        },
        adapter: name(ui.get_compute_gpu_index()),
        cpu_fallback: ui.get_compute_fallback(),
        parallel_min_rows: ui.get_compute_parallel_rows().max(1) as usize,
        gpu_min_rows: ui.get_compute_gpu_rows().max(1) as usize,
        gpu_memory_fraction: f64::from(ui.get_compute_memory_percent()) / 100.,
        gpu_preconditioner: if ui.get_compute_preconditioner() == 0 {
            GpuPreconditioner::Auto
        } else {
            GpuPreconditioner::Jacobi
        },
        gpu_restart: ui.get_compute_restart().max(2) as usize,
        gpu_inner_iterations: ui.get_compute_inner().max(1) as usize,
        gpu_smoothing_sweeps: ui.get_compute_sweeps().max(1) as usize,
        gpu_timeout_ms: ui.get_compute_timeout().max(100) as u64,
        gpu_host_basis_mb: ui.get_compute_basis_mb().max(16) as usize,
    };
    let threads = ui.get_compute_threads().max(0) as usize;
    let available = std::thread::available_parallelism().map_or(1, |n| n.get());
    if threads > available {
        return Err(format!(
            "Choose 0 (automatic) or at most {available} workers."
        ));
    }
    let changed = state.workbench.solver_options().compute != options
        || state.workbench.solver_options().threads != threads;
    state.workbench.set_compute_options(options)?;
    state.workbench.set_cpu_threads(threads);
    // Run reads this draft; keep it aligned with the project compute controls.
    ui.set_wb_cpu_threads(threads as i32);
    ui.set_preferences_render_adapter(name(ui.get_compute_display_index()).into());
    ui.set_preferences_cpu_display(ui.get_compute_cpu_display());
    ui.set_preferences_display_fps(ui.get_compute_fps());
    render_device::configure(render_device::Options {
        cpu: ui.get_compute_cpu_display(),
        adapter: name(ui.get_compute_display_index()),
        fps: ui.get_compute_fps().clamp(5, 120) as u32,
    });
    if changed {
        state.mark_workbench_dirty();
    }
    state.log(
        "Applied compute and rendering settings. CAD adapter changes take effect after restart.",
    );
    refresh_ui(ui, state);
    Ok(())
}
