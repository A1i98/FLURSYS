//! Shared orthographic camera for CAD, mesh inspection and result surfaces.
//!
//! Camera math is shared between viewport rendering and
//! input handling. This implementation uses FLURSYS coordinates and raster API.
use super::{PREVIEW_HEIGHT, PREVIEW_WIDTH};
use flursys::{Ray3, Vec3};
use slint::ComponentHandle;

// Shared camera implementation; attribution is in THIRD_PARTY_NOTICES.txt.
use cad_render as cad_camera;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct View3d {
    pub yaw: f32,
    pub pitch: f32,
    pub zoom: f32,
    /// Translation in preview pixels, independent of model units.
    pub pan: (f64, f64),
}

impl Default for View3d {
    fn default() -> Self {
        Self {
            yaw: 0.65,
            pitch: 0.48,
            zoom: 1.0,
            pan: (0.0, 0.0),
        }
    }
}

impl View3d {
    pub fn orbit_from(&mut self, yaw: f32, pitch: f32, dx: f32, dy: f32) {
        if [yaw, pitch, dx, dy].iter().all(|v| v.is_finite()) {
            let mut camera = cad_camera::Camera {
                yaw: f64::from(yaw),
                pitch: f64::from(pitch),
                ..cad_camera::Camera::default()
            };
            camera.orbit(f64::from(dx), f64::from(dy));
            self.yaw = camera.yaw as f32;
            self.pitch = camera.pitch as f32;
        }
    }

    pub fn zoom_by(&mut self, delta: f32) {
        if delta.is_finite() {
            let mut camera = cad_camera::Camera {
                distance: 1.0 / f64::from(self.zoom.clamp(0.05, 50.0)),
                ..cad_camera::Camera::default()
            };
            camera.zoom_at(
                (f64::from(delta.clamp(-2000.0, 2000.0)) * 0.0015).exp(),
                None,
            );
            self.zoom = ((1.0 / camera.distance) as f32).clamp(0.05, 50.0);
        }
    }

    pub fn fit(&mut self) {
        self.zoom = 1.0;
        self.pan = (0.0, 0.0);
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Camera3d {
    camera: cad_camera::Camera,
    projection: cad_camera::Mat4,
    pub centre: Vec3,
    pub scale: f64,
    pan: (f64, f64),
    radius: f64,
    raster: (u32, u32),
}

fn camera_point(p: Vec3) -> cad_geom::Vec3 {
    cad_geom::Vec3::new(p.x, p.y, p.z)
}

fn model_point(p: cad_geom::Vec3) -> Vec3 {
    Vec3::new(p.x, p.y, p.z)
}

impl Camera3d {
    pub fn fit(points: impl IntoIterator<Item = Vec3>, view: View3d) -> Self {
        let mut bounds = cad_geom::Aabb3::EMPTY;
        for p in points {
            bounds.add(camera_point(p));
        }
        if bounds.is_empty() {
            bounds.add(cad_geom::Vec3::new(-0.5, -0.5, -0.5));
            bounds.add(cad_geom::Vec3::new(0.5, 0.5, 0.5));
        }
        let radius = (bounds.diagonal() * 0.5).max(1e-12);
        let zoom = if view.zoom.is_finite() {
            view.zoom.clamp(0.05, 50.0)
        } else {
            1.0
        };
        // Normalize framing to the actual scene size. The upstream camera's
        // interactive fit assumes millimetres; CFD geometry can be sub-metre.
        let scale =
            f64::from(PREVIEW_WIDTH.min(PREVIEW_HEIGHT)) * 0.84 / (2.0 * radius) * f64::from(zoom);
        let camera = cad_camera::Camera {
            target: bounds.center(),
            yaw: if view.yaw.is_finite() {
                f64::from(view.yaw)
            } else {
                0.65
            },
            pitch: if view.pitch.is_finite() {
                f64::from(view.pitch)
            } else {
                0.48
            },
            distance: f64::from(PREVIEW_HEIGHT) / (0.8 * scale),
            ..cad_camera::Camera::default()
        };
        Self {
            projection: camera
                .view_proj(f64::from(PREVIEW_WIDTH) / f64::from(PREVIEW_HEIGHT), radius),
            centre: model_point(camera.target),
            camera,
            scale,
            pan: if view.pan.0.is_finite() && view.pan.1.is_finite() {
                view.pan
            } else {
                (0.0, 0.0)
            },
            radius,
            raster: (PREVIEW_WIDTH, PREVIEW_HEIGHT),
        }
    }

    /// Same camera used by native CAD rendering and CPU-side entity picking.
    pub fn scene_camera(&self) -> (cad_camera::Camera, f64) {
        let mut camera = self.camera;
        camera.pan(self.pan.0, self.pan.1, f64::from(self.raster.1));
        (camera, self.radius)
    }
    pub fn screen(&self, p: Vec3) -> (f64, f64) {
        let c = self.projection.apply(camera_point(p));
        (
            (c[0] / c[3] + 1.0) * 0.5 * f64::from(self.raster.0) + self.pan.0,
            (1.0 - c[1] / c[3]) * 0.5 * f64::from(self.raster.1) + self.pan.1,
        )
    }

    pub fn project(&self, p: Vec3) -> (i32, i32) {
        let (x, y) = self.screen(p);
        (x.round() as i32, y.round() as i32)
    }

    pub fn ray(&self, point: (f64, f64)) -> Option<Ray3> {
        if !point.0.is_finite() || !point.1.is_finite() {
            return None;
        }
        let (origin, direction) = self.camera.ray(
            point.0 - self.pan.0,
            point.1 - self.pan.1,
            f64::from(self.raster.0),
            f64::from(self.raster.1),
        );
        Ray3::new(model_point(origin), model_point(direction))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn projected_points_lie_on_pick_rays_after_pan_zoom_and_orbit() {
        for size in [1e-5, 1.0, 1e5] {
            for (yaw, pitch) in [
                (0.65, 0.48),
                (1.57, -1.2),
                (0.0, std::f32::consts::FRAC_PI_2),
            ] {
                let origin = Vec3::new(12.0, -3.0, 7.0) * size;
                let camera = Camera3d::fit(
                    [origin, origin + Vec3::new(2.0, 3.0, 4.0) * size],
                    View3d {
                        yaw,
                        pitch,
                        zoom: 2.4,
                        pan: (37.5, -25.0),
                    },
                );
                let p = origin + Vec3::new(0.3, 1.2, 2.7) * size;
                let ray = camera.ray(camera.screen(p)).unwrap();
                assert!((p - ray.origin).cross(ray.direction).norm() < size * 1e-9);
            }
        }
    }
    #[test]
    fn orbit_does_not_change_scale_and_fit_preserves_orientation() {
        let points = [Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 1.0, 2.0)];
        let a = Camera3d::fit(points, View3d::default());
        let mut view = View3d::default();
        view.orbit_from(0.0, 0.0, 100.0, 1000.0);
        assert_eq!(a.scale, Camera3d::fit(points, view).scale);
        let angles = (view.yaw, view.pitch);
        view.zoom_by(-200.0);
        view.pan = (10.0, 20.0);
        view.fit();
        assert_eq!(angles, (view.yaw, view.pitch));
        assert_eq!(view.zoom, 1.0);
        assert_eq!(view.pan, (0.0, 0.0));
    }
}

/// Convert logical UI pixels to the contained preview image's pixel scale.
pub(super) fn preview_scale(width: f32, height: f32) -> f32 {
    (width.max(1.0) / PREVIEW_WIDTH as f32)
        .min(height.max(1.0) / PREVIEW_HEIGHT as f32)
        .max(1e-6)
}

/// Event adapters shared by the CAD, meshing, setup and result workspaces.
pub(super) fn bind(
    ui: &super::MainWindow,
    state: &std::rc::Rc<std::cell::RefCell<super::AppState>>,
) {
    use super::{geometry_view_angles, geometry_view_label, refresh_ui};
    use flursys::{GeometrySelectionTarget, MeshDimension};
    let weak = ui.as_weak();
    let pick_state = state.clone();
    ui.on_cad_pick(move |x, y, width, height, additive| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = pick_state.borrow_mut();
        if super::cad::preview(&state).is_some() || state.cad_job.is_some() {
            return;
        }
        let Some(point) = super::render::preview_image_point(x, y, width, height) else {
            return;
        };
        if super::measure::pick(&ui, &mut state, point, true) {
            refresh_ui(&ui, &state);
            return;
        }
        if let Some(target) = super::render::pick_canonical_geometry_target(
            state.workbench.geometry(),
            state.view_3d,
            point,
            state.geometry_pick_mode,
        ) {
            super::select_tree_geometry_target(&mut state, target, additive);
            super::cad_profiles::consume(&ui, &mut state);
            ui.set_inspector_mode(0);
            if !ui.get_cad_task_open() {
                ui.set_geometry_sidebar_tab(2);
            }
        } else if !additive {
            state.wb_selected_targets.clear();
            state.geometry_editor.selection.clear();
            state.selected_tree = None;
        }
        super::rebuild_tree_rows(&mut state);
        refresh_ui(&ui, &state);
    });
    let weak_ui = ui.as_weak();
    let orbit_state = state.clone();
    ui.on_geometry_drag(
        move |mouse_x, mouse_y, pressed_x, pressed_y, width, height| {
            let Some(ui) = weak_ui.upgrade() else {
                return;
            };
            let mut state = orbit_state.borrow_mut();
            let new_gesture = state.geometry_drag_anchor.is_none_or(|(x, y, _, _)| {
                (x - pressed_x).abs() > f32::EPSILON || (y - pressed_y).abs() > f32::EPSILON
            });
            if new_gesture {
                state.geometry_drag_anchor =
                    Some((pressed_x, pressed_y, state.view_3d.yaw, state.view_3d.pitch));
            }
            if let Some((start_x, start_y, start_yaw, start_pitch)) = state.geometry_drag_anchor {
                let scale = preview_scale(width, height);
                state.view_3d.orbit_from(
                    start_yaw,
                    start_pitch,
                    (mouse_x - start_x) / scale,
                    (mouse_y - start_y) / scale,
                );
                state.geometry_view_axis = 7;
            }
            state.show_geometry_3d = true;
            state.show_mesh = false;
            refresh_ui(&ui, &state);
        },
    );
    let weak = ui.as_weak();
    let pan_state = state.clone();
    ui.on_viewport_pan(move |dx, dy, width, height| {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = pan_state.borrow_mut();
        let scale = f64::from(preview_scale(width, height));
        if state.show_geometry_3d
            || (state.current_step == 4
                && state
                    .active_result()
                    .is_some_and(|r| r.mesh().dimension() == MeshDimension::ThreeD))
            || (state.show_mesh && state.workbench.mesh_dimension() == MeshDimension::ThreeD)
        {
            if ui.get_result_slice_axis() > 0 && state.current_step == 4 {
                return;
            }
            state.view_3d.pan.0 += f64::from(dx) / scale;
            state.view_3d.pan.1 += f64::from(dy) / scale;
        } else {
            state.mesh_view.center_x -= f64::from(dx) / scale / state.mesh_view.pixels_per_unit;
            state.mesh_view.center_y += f64::from(dy) / scale / state.mesh_view.pixels_per_unit;
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let display_state = state.clone();
    ui.on_cad_display_changed(move || {
        if let Some(ui) = weak.upgrade() {
            refresh_ui(&ui, &display_state.borrow());
        }
    });

    let orbit_end_state = state.clone();
    ui.on_geometry_drag_end(move || {
        orbit_end_state.borrow_mut().geometry_drag_anchor = None;
    });

    let weak_ui = ui.as_weak();
    let zoom_state = state.clone();
    ui.on_geometry_zoom(move |delta_y| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = zoom_state.borrow_mut();
        state.view_3d.zoom_by(delta_y);
        state.show_geometry_3d = true;
        state.show_mesh = false;
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let view_axis_state = state.clone();
    ui.on_select_geometry_view_axis(move |axis| {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = view_axis_state.borrow_mut();
        let axis = axis.clamp(0, 6);
        let (yaw, pitch) = geometry_view_angles(axis);
        state.view_3d.yaw = yaw;
        state.view_3d.pitch = pitch;
        state.geometry_view_axis = axis;
        state.geometry_drag_anchor = None;
        state.log(format!(
            "Selected {} geometry view.",
            geometry_view_label(axis)
        ));
        refresh_ui(&ui, &state);
    });

    let weak_ui = ui.as_weak();
    let reset_geometry_state = state.clone();
    ui.on_reset_geometry_view(move || {
        let Some(ui) = weak_ui.upgrade() else {
            return;
        };
        let mut state = reset_geometry_state.borrow_mut();
        state.view_3d.fit();
        state.geometry_drag_anchor = None;
        state.show_geometry_3d = true;
        state.show_mesh = false;
        state.log("Geometry camera reset.");
        refresh_ui(&ui, &state);
    });

    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_frame_selection(move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        if let Some(view) = super::render::frame_selection(
            state.workbench.geometry(),
            &state.wb_selected_targets,
            state.view_3d,
        ) {
            state.view_3d = view;
            state.geometry_drag_anchor = None;
        } else {
            ui.set_cad_feedback("Select a visible face, edge, vertex or body to frame.".into());
        }
        refresh_ui(&ui, &state);
    });
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_cad_normal_view(move || {
        let Some(ui) = weak.upgrade() else {
            return;
        };
        let mut state = shared.borrow_mut();
        let frame = state
            .wb_selected_targets
            .iter()
            .find_map(|target| match target {
                GeometrySelectionTarget::Face(face) => {
                    state.workbench.geometry().sketch_frame_for_face(*face).ok()
                }
                _ => None,
            });
        if let Some(frame) = frame {
            let n = frame.normal;
            state.view_3d.yaw = (-n.x).atan2(-n.y) as f32;
            state.view_3d.pitch = n.z.clamp(-1.0, 1.0).asin() as f32;
            state.geometry_view_axis = 7;
            if let Some(view) = super::render::frame_selection(
                state.workbench.geometry(),
                &state.wb_selected_targets,
                state.view_3d,
            ) {
                state.view_3d = view;
            }
            state.geometry_drag_anchor = None;
        } else {
            ui.set_cad_feedback("Normal view needs a planar face or sketch region.".into());
        }
        refresh_ui(&ui, &state);
    });
}

#[cfg(test)]
mod integration_tests {
    use super::super::*;
    use super::*;
    use slint::Model;
    use slint::platform::{
        Platform, WindowAdapter,
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
    };
    struct SoftwarePlatform(Rc<MinimalSoftwareWindow>);
    impl Platform for SoftwarePlatform {
        fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, slint::PlatformError> {
            Ok(self.0.clone())
        }
        fn duration_since_start(&self) -> Duration {
            Duration::from_secs(1)
        }
    }

    #[test]
    fn cad_navigation_frames_real_solids_and_preserves_document_and_layout() {
        let window = MinimalSoftwareWindow::new(RepaintBufferType::NewBuffer);
        slint::platform::set_platform(Box::new(SoftwarePlatform(window.clone()))).unwrap();
        let ui = MainWindow::new().unwrap();
        let state = Rc::new(RefCell::new(AppState::new()));
        let body = state
            .borrow_mut()
            .workbench
            .geometry_mut()
            .cad_add_primitive(
                flursys::CadPrimitive::Cylinder {
                    radius: 1.0,
                    height: 2.0,
                },
                Vec3::new(3.0, -2.0, 1.0),
                "Cylinder",
            )
            .unwrap();
        state
            .borrow_mut()
            .workbench
            .geometry_mut()
            .cad_add_primitive(
                flursys::CadPrimitive::Box {
                    length: 2.0,
                    width: 3.0,
                    height: 1.0,
                },
                Vec3::new(-2.0, -1.0, 0.0),
                "Base",
            )
            .unwrap();
        let original = state.borrow().workbench.geometry().clone();
        bind_callbacks(&ui, &state);
        cad::bind(&ui, &state);
        ui.set_geometry_cad_view_3d(true);
        ui.set_cad_workbench(1);
        ui.invoke_geometry_drag(40.0, 55.0, 10.0, 10.0, 1040.0, 640.0);
        ui.invoke_geometry_drag_end();
        assert_eq!(state.borrow().geometry_view_axis, 7);
        let before = state.borrow().view_3d;
        ui.invoke_viewport_pan(80.0, -30.0, 1040.0, 640.0);
        assert_eq!(state.borrow().view_3d.pan, (40.0, -15.0));
        ui.invoke_geometry_zoom(-300.0);
        assert!(state.borrow().view_3d.zoom > before.zoom);
        let points = render::project_measurement(
            &original,
            state.borrow().view_3d,
            &[Vec3::new(3.0, -2.0, 3.0), Vec3::new(-1.0, 0.5, 1.0)],
        );
        let pick = |index: usize, additive| {
            ui.invoke_cad_pick(
                points[index].0 as f32,
                points[index].1 as f32,
                PREVIEW_WIDTH as f32,
                PREVIEW_HEIGHT as f32,
                additive,
            );
        };
        pick(0, false);
        assert_eq!(state.borrow().wb_selected_targets.len(), 1);
        pick(1, true);
        assert_eq!(state.borrow().wb_selected_targets.len(), 2);
        pick(1, true);
        assert_eq!(state.borrow().wb_selected_targets.len(), 1);
        for axis in 0..=6 {
            ui.invoke_select_geometry_view_axis(axis);
            assert_eq!(state.borrow().geometry_view_axis, axis);
            assert_eq!(state.borrow().view_3d.pan, (40.0, -15.0));
        }
        state.borrow_mut().wb_selected_targets = vec![GeometrySelectionTarget::Body(body)];
        ui.invoke_cad_frame_selection();
        assert!(state.borrow().view_3d.zoom > 1.0);
        let cap = original
            .body(body)
            .unwrap()
            .faces
            .iter()
            .copied()
            .find(|face| original.sketch_frame_for_face(*face).is_ok())
            .unwrap();
        state.borrow_mut().wb_selected_targets = vec![GeometrySelectionTarget::Face(cap)];
        ui.invoke_cad_normal_view();
        let frame = original.sketch_frame_for_face(cap).unwrap();
        let view = state.borrow().view_3d;
        let camera = Camera3d::fit(
            original
                .renderable_faces()
                .iter()
                .flat_map(|face| face.vertices.iter().copied()),
            view,
        );
        let ray = camera.ray(camera.screen(frame.origin)).unwrap();
        assert!(ray.direction.dot(frame.normal) < -0.99999);
        ui.invoke_reset_geometry_view();
        assert_eq!(state.borrow().view_3d.pan, (0.0, 0.0));
        assert_eq!(state.borrow().view_3d.zoom, 1.0);
        ui.invoke_select_geometry_view_axis(3);
        state.borrow_mut().wb_selected_targets.clear();
        rebuild_tree_rows(&mut state.borrow_mut());
        refresh_ui(&ui, &state.borrow());
        ui.show().unwrap();
        let mut rendered_modes = Vec::new();
        for (width, height) in [(1440u32, 1000u32), (1100, 760)] {
            window.set_size(slint::PhysicalSize::new(width, height));
            for mode in 0..=2 {
                ui.set_cad_display_mode(mode);
                ui.invoke_cad_display_changed();
                let image = ui.get_visualization_image().to_rgba8().unwrap();
                if width == 1440 {
                    rendered_modes.push(image.as_bytes().to_vec());
                }
                window.request_redraw();
                let mut pixels = vec![slint::Rgb8Pixel::default(); (width * height) as usize];
                window.draw_if_needed(|renderer| {
                    renderer.render(&mut pixels, width as usize);
                });
                assert!(pixels.iter().any(|pixel| pixel.r > 100));
                let path = format!("/tmp/flursys-cad-unified-{width}-{mode}.png");
                let file = std::fs::File::create(path).unwrap();
                let mut encoder = png::Encoder::new(file, width, height);
                encoder.set_color(png::ColorType::Rgb);
                let mut writer = encoder.write_header().unwrap();
                let bytes: Vec<u8> = pixels
                    .iter()
                    .flat_map(|pixel| [pixel.r, pixel.g, pixel.b])
                    .collect();
                writer.write_image_data(&bytes).unwrap();
            }
        }
        // Route actual middle-button pointer events through the Slint TouchArea.
        // Callback-only tests cannot detect a broken pointer capture contract.
        let before_pan = state.borrow().view_3d.pan;
        use slint::platform::{PointerEventButton, WindowEvent};
        let from = slint::LogicalPosition::new(500.0, 450.0);
        let to = slint::LogicalPosition::new(540.0, 475.0);
        ui.window().dispatch_event(WindowEvent::PointerPressed {
            position: from,
            button: PointerEventButton::Middle,
        });
        ui.window()
            .dispatch_event(WindowEvent::PointerMoved { position: to });
        ui.window().dispatch_event(WindowEvent::PointerReleased {
            position: to,
            button: PointerEventButton::Middle,
        });
        assert_ne!(
            state.borrow().view_3d.pan,
            before_pan,
            "Middle drag must reach the shared pan controller"
        );
        assert!(state.borrow().geometry_drag_anchor.is_none());
        assert_ne!(rendered_modes[0], rendered_modes[1]);
        assert_ne!(rendered_modes[1], rendered_modes[2]);
        assert_eq!(state.borrow().workbench.geometry(), &original);
        // Topology is opt-in; hiding it never changes persistent geometry IDs.
        assert!(
            !ui.get_project_tree_model()
                .iter()
                .any(|row| row.label == "Topology")
        );
        ui.set_show_topology(true);
        ui.invoke_refresh_browser();
        assert!(
            ui.get_project_tree_model()
                .iter()
                .any(|row| row.label == "Topology")
        );
        ui.set_show_topology(false);
        ui.invoke_refresh_browser();
        let old_density = state.borrow().workbench.material().density;
        assert!(!ui.invoke_apply_fluid_material("-1".into(), "0.005".into()));
        assert_eq!(state.borrow().workbench.material().density, old_density);
        assert!(!ui.get_material_dialog_feedback().is_empty());
        assert!(ui.invoke_apply_fluid_material("1.25".into(), "0.005".into()));
        assert_eq!(state.borrow().workbench.material().density, 1.25);
        assert_eq!(
            state.borrow().workbench.material().kinematic_viscosity,
            0.005
        );
        let out = std::path::Path::new("/tmp/flursys-workbench-preview");
        std::fs::create_dir_all(out).unwrap();
        let capture = |name: &str, width: u32, height: u32| {
            window.set_size(slint::PhysicalSize::new(width, height));
            window.request_redraw();
            let mut pixels = vec![slint::Rgb8Pixel::default(); (width * height) as usize];
            window.draw_if_needed(|renderer| {
                renderer.render(&mut pixels, width as usize);
            });
            let file = std::fs::File::create(out.join(format!("{name}-{width}.png"))).unwrap();
            let mut encoder = png::Encoder::new(file, width, height);
            encoder.set_color(png::ColorType::Rgb);
            let bytes = pixels
                .iter()
                .flat_map(|p| [p.r, p.g, p.b])
                .collect::<Vec<_>>();
            encoder
                .write_header()
                .unwrap()
                .write_image_data(&bytes)
                .unwrap();
        };
        for step in 0..=4 {
            ui.invoke_select_step(step);
            for (w, h) in [(1440, 900), (1100, 760)] {
                capture(&format!("workspace-{step}"), w, h);
            }
        }
        ui.invoke_select_step(0);
        ui.invoke_new_project();
        assert!(ui.get_unsaved_replacement_pending());
        capture("unsaved-dialog", 1440, 900);
        ui.invoke_cancel_project_replacement();
        assert!(!ui.get_unsaved_replacement_pending());
        assert_eq!(state.borrow().workbench.geometry(), &original);
        ui.set_material_draft_density("1.25".into());
        ui.set_material_draft_viscosity("0.005".into());
        ui.set_material_dialog_feedback("".into());
        ui.set_material_dialog_open(true);
        capture("fluid-dialog", 1440, 900);
        ui.set_material_dialog_open(false);
        ui.hide().unwrap();
    }
}
