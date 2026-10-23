//! Opt-in real-window smoke test. Uses manufactured data, never modifies a user workspace.
use super::*;
use flursys::workbench::plane_monitor::*;

fn snapshot() -> PlaneSnapshot {
    let plane = FlowPlaneMonitor {
        id: "smoke".into(),
        name: "Mid-plane · synthetic verification".into(),
        origin: Vec3::new(0., 0., 0.005),
        u: Vec3::new(1., 0., 0.),
        v: Vec3::new(0., 1., 0.),
        enabled: false,
        every: 10,
        display: PlaneDisplay {
            edges: false,
            vectors: true,
            ..Default::default()
        },
    };
    let mut samples = Vec::new();
    for y in 0..12 {
        for x in 0..12 {
            let d = 0.1 / 12.;
            let a = x as f64 * d;
            let b = y as f64 * d;
            samples.push(PlaneSample {
                geometry: PlaneCell {
                    cell: y * 12 + x,
                    point: Vec3::new(a + d / 2., b + d / 2., 0.005),
                    polygon: vec![
                        Vec3::new(a, b, 0.005),
                        Vec3::new(a + d, b, 0.005),
                        Vec3::new(a + d, b + d, 0.005),
                        Vec3::new(a, b + d, 0.005),
                    ],
                },
                velocity: Vec3::new(0.05 - b - d / 2., a + d / 2. - 0.05, 0.),
                pressure: a - b,
            });
        }
    }
    PlaneSnapshot {
        plane,
        iteration: 100,
        physical_time: Some(1.0),
        samples,
        path: None,
    }
}

#[test]
#[ignore = "requires a real display; writes screenshots only under /tmp"]
fn unified_section_real_window_smoke() {
    let ui = MainWindow::new().unwrap();
    let state = Rc::new(RefCell::new(AppState::new()));
    let sample = std::env::var_os("FLURSYS_PREVIEW_FRAME")
        .map(|p| serde_json::from_slice::<PlaneSnapshot>(&std::fs::read(p).unwrap()).unwrap())
        .unwrap_or_else(snapshot);
    let sample = std::sync::Arc::new(sample);
    {
        let mut s = state.borrow_mut();
        s.workbench_project.plane_monitors = vec![sample.plane.clone()];
        s.post
            .plane_samples
            .insert(sample.plane.id.clone(), (*sample).clone());
        s.current_step = 4;
    }
    bind_callbacks(&ui, &state);
    ui.set_current_step(4);
    ui.set_post_is_3d(true);
    ui.set_post_has_result(true);
    ui.set_flow_plane_names(ModelRc::new(VecModel::from(vec![SharedString::from(
        sample.plane.name.clone(),
    )])));
    plane_monitors::refresh_locations(&ui, &state.borrow());
    ui.invoke_result_section_selected(4);
    let recording = tempfile::tempdir().unwrap();
    let mut paths = Vec::new();
    for index in 0..3 {
        let mut frame = (*sample).clone();
        frame.iteration += index;
        for value in &mut frame.samples {
            value.pressure += index as f64 * 0.1;
        }
        let path = recording.path().join(format!("iteration-{index:08}.json"));
        std::fs::write(&path, serde_json::to_vec(&frame).unwrap()).unwrap();
        paths.push(path);
    }
    state.borrow_mut().post.plane_frames = paths;
    ui.invoke_flow_plane_navigate(4, 0., 0., 800., 500.);
    let before = ui.get_flow_plane_image().to_rgba8().unwrap();
    ui.set_flow_plane_field(1);
    ui.invoke_flow_plane_style();
    let after = ui.get_flow_plane_image().to_rgba8().unwrap();
    assert_ne!(
        before.as_bytes(),
        after.as_bytes(),
        "Changing scalar must redraw archived field data"
    );
    ui.invoke_flow_plane_navigate(1, 0., 1., 800., 500.);
    ui.invoke_flow_plane_navigate(0, 20., -10., 800., 500.);
    let point = sample.samples[sample.samples.len() / 2].geometry.point;
    let camera = *state.borrow().post.section_camera.borrow();
    let pixel = camera.screen_at(&sample, point);
    ui.invoke_flow_plane_navigate(3, pixel[0] as f32, pixel[1] as f32, 800., 500.);
    assert!(ui.get_flow_plane_readout().contains("p/ρ="));
    ui.invoke_flow_plane_playback(3);
    assert_eq!(state.borrow().post.plane_frame, 1);
    assert!(ui.get_flow_plane_readout().contains("p/ρ="));
    ui.set_flow_plane_loop(true);
    ui.set_flow_plane_first(1);
    ui.set_flow_plane_last(2);
    {
        let mut s = state.borrow_mut();
        s.post.plane_playing = true;
        s.post.plane_tick = None;
        post_processing::tick(&ui, &mut s);
        assert_eq!(s.post.plane_frame, 0);
        s.post.plane_playing = false;
    }
    ui.invoke_flow_plane_navigate(2, 0., 0., 800., 500.);
    ui.window().set_size(slint::LogicalSize::new(1400., 950.));
    let folder = PathBuf::from("/tmp/flursys-section-ui");
    std::fs::create_dir_all(&folder).unwrap();
    let timer = Rc::new(Timer::default());
    let weak_timer = Rc::downgrade(&timer);
    let weak = ui.as_weak();
    let phase = Rc::new(RefCell::new(0));
    let stage = phase.clone();
    timer.start(TimerMode::Repeated, Duration::from_millis(900), move || {
        let ui = weak.upgrade().unwrap();
        let mut n = stage.borrow_mut();
        let pixels = ui.window().take_snapshot().unwrap();
        let name = match *n {
            0 => "results",
            1 => "solution",
            2 => "results-900",
            _ => "solution-900",
        };
        let file = std::fs::File::create(folder.join(format!("{name}.png"))).unwrap();
        let mut png = png::Encoder::new(file, pixels.width(), pixels.height());
        png.set_color(png::ColorType::Rgba);
        png.set_depth(png::BitDepth::Eight);
        png.write_header()
            .unwrap()
            .write_image_data(pixels.as_bytes())
            .unwrap();
        match *n {
            0 => {
                ui.set_current_step(3);
                ui.set_solution_show_plane(true);
                ui.set_solution_properties_section(5);
            }
            1 => {
                ui.window().set_size(slint::LogicalSize::new(900., 950.));
                ui.set_current_step(4);
            }
            2 => {
                ui.set_current_step(3);
            }
            _ => {
                weak_timer.upgrade().unwrap().stop();
                slint::quit_event_loop().unwrap();
            }
        }
        *n += 1;
    });
    ui.run().unwrap();
    assert_eq!(*phase.borrow(), 4);
}
