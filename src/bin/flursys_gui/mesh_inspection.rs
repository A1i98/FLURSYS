//! Visual cutaways and quantitative quality summaries. No mesh topology is edited.
use super::*;
use flursys::{MeshRenderCache, UnstructuredMesh};

#[derive(Clone, Copy, Debug)]
pub(super) struct MeshClip {
    pub axis: usize,
    pub fraction: f64,
    pub inverted: bool,
    pub quality: Option<(MeshQualityMetric, f64)>,
    pub cell: Option<(flursys::MeshId, usize)>,
}
impl MeshClip {
    pub fn mask(self, mesh: &UnstructuredMesh, cache: &MeshRenderCache) -> Vec<bool> {
        let coordinate = |p: Vec3| match self.axis {
            0 => p.x,
            1 => p.y,
            _ => p.z,
        };
        let bounds = cache.bounds();
        let low = coordinate(bounds.0);
        let high = coordinate(bounds.1);
        let cut = low + (high - low) * self.fraction.clamp(0., 1.);
        mesh.cells()
            .iter()
            .enumerate()
            .map(|(index, c)| {
                if let Some((id, cell)) = self.cell {
                    return id != mesh.id() || index == cell;
                }
                if let Some((metric, threshold)) = self.quality {
                    let value = cache.quality().values(metric)[index];
                    return !value.is_finite()
                        || if metric == MeshQualityMetric::CellMeasure {
                            value < threshold
                        } else {
                            value > threshold
                        };
                }
                if self.inverted {
                    coordinate(c.center) >= cut
                } else {
                    coordinate(c.center) <= cut
                }
            })
            .collect()
    }
}
pub(super) fn face_cell(mesh: &UnstructuredMesh, face: usize, mask: &[bool]) -> Option<usize> {
    let f = &mesh.faces()[face];
    match (mask[f.owner], f.neighbour.map(|n| mask[n]).unwrap_or(false)) {
        (true, false) => Some(f.owner),
        (false, true) => f.neighbour,
        _ => None,
    }
}
pub(super) fn pick(
    mesh: &UnstructuredMesh,
    cache: &MeshRenderCache,
    clip: MeshClip,
    origin: Vec3,
    direction: Vec3,
) -> Option<MeshSelection> {
    let mask = clip.mask(mesh, cache);
    let mut nearest = f64::INFINITY;
    let mut hit = None;
    for (i, f) in mesh.faces().iter().enumerate() {
        if face_cell(mesh, i, &mask).is_none() {
            continue;
        }
        for k in 1..f.vertices.len().saturating_sub(1) {
            let a = mesh.points()[f.vertices[0]].position;
            let b = mesh.points()[f.vertices[k]].position;
            let c = mesh.points()[f.vertices[k + 1]].position;
            let e1 = b - a;
            let e2 = c - a;
            let p = direction.cross(e2);
            let det = e1.dot(p);
            if det.abs() <= f64::EPSILON * e1.norm() * e2.norm() {
                continue;
            }
            let inv = 1. / det;
            let t = origin - a;
            let u = t.dot(p) * inv;
            if !(0.0..=1.0).contains(&u) {
                continue;
            }
            let q = t.cross(e1);
            let v = direction.dot(q) * inv;
            if v < 0. || u + v > 1. {
                continue;
            }
            let distance = e2.dot(q) * inv;
            if distance >= 0. && distance < nearest {
                nearest = distance;
                hit = Some(MeshSelection::face(mesh.id(), i));
            }
        }
    }
    hit
}
pub(super) fn refresh(ui: &MainWindow, state: &AppState) {
    let Some(cache) = state.workbench.mesh_render_cache() else {
        ui.set_mesh_quality_stats("Generate a mesh to inspect quality.".into());
        ui.set_mesh_quality_low("—".into());
        ui.set_mesh_quality_high("—".into());
        ui.set_mesh_quality_bins(ModelRc::new(VecModel::from(vec![0_f32; 10])));
        return;
    };
    let values = cache.quality().values(state.mesh_quality_metric);
    let finite = values
        .iter()
        .copied()
        .filter(|v| v.is_finite())
        .collect::<Vec<_>>();
    if finite.is_empty() {
        ui.set_mesh_quality_stats("No finite quality values.".into());
        ui.set_mesh_quality_bins(ModelRc::new(VecModel::from(vec![0_f32; 10])));
        ui.set_mesh_quality_low("—".into());
        ui.set_mesh_quality_high("—".into());
        return;
    }
    let lo = finite.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = finite.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let mut bins = [0_usize; 10];
    let mut above = 0;
    let mut sum = 0.;
    for v in &finite {
        let index = if hi > lo {
            (((v - lo) / (hi - lo) * 10.).floor() as usize).min(9)
        } else {
            0
        };
        bins[index] += 1;
        sum += v;
        if if state.mesh_quality_metric == MeshQualityMetric::CellMeasure {
            *v < state.mesh_quality_threshold
        } else {
            *v > state.mesh_quality_threshold
        } {
            above += 1;
        }
    }
    if state.mesh_quality_metric == MeshQualityMetric::CellMeasure {
        bins.reverse();
    }
    let peak = bins.iter().copied().max().unwrap_or(1).max(1) as f32;
    ui.set_mesh_quality_bins(ModelRc::new(VecModel::from(
        bins.iter().map(|v| *v as f32 / peak).collect::<Vec<_>>(),
    )));
    let (left, right) = if state.mesh_quality_metric == MeshQualityMetric::CellMeasure {
        (hi, lo)
    } else {
        (lo, hi)
    };
    ui.set_mesh_quality_low(format!("{left:.3e}").into());
    ui.set_mesh_quality_high(format!("{right:.3e}").into());
    ui.set_mesh_quality_stats(format!("{} · all {} cells\nMin {lo:.4e} · mean {:.4e} · max {hi:.4e}\n{} / {} ({:.2}%) beyond threshold {:.4e}\n{} non-finite · histogram: 10 equal-width bins",state.mesh_quality_metric.label(),values.len(),sum/finite.len() as f64,above,finite.len(),100.*above as f64/finite.len() as f64,state.mesh_quality_threshold,values.len()-finite.len()).into());
}
/// Centre a selected cell and expose it even when it lies inside a 3D volume.
pub(super) fn focus_cell(ui: &MainWindow, state: &mut AppState, index: usize) {
    let Some(g) = state.workbench.mesh() else {
        return;
    };
    let Some(c) = g.mesh.cells().get(index) else {
        return;
    };
    let Some(cache) = state.workbench.mesh_render_cache() else {
        return;
    };
    let mut radius = 0.0_f64;
    for face in &c.faces {
        for v in &g.mesh.faces()[*face].vertices {
            radius = radius.max((g.mesh.points()[*v].position - c.center).norm());
        }
    }
    if g.mesh.dimension() == MeshDimension::ThreeD {
        let bounds = cache.bounds();
        let span = (bounds.1 - bounds.0).norm();
        let mut view = state.view_3d;
        view.zoom = (span / (8.0 * radius.max(1e-12))).clamp(1.0, 50.0) as f32;
        view.pan = (0.0, 0.0);
        let camera = viewport::Camera3d::fit(cache.positions().iter().copied(), view);
        let point = camera.screen(c.center);
        view.pan = (
            f64::from(PREVIEW_WIDTH) * 0.5 - point.0,
            f64::from(PREVIEW_HEIGHT) * 0.5 - point.1,
        );
        state.view_3d = view;
        state.mesh_clip = Some(MeshClip {
            axis: 0,
            fraction: 1.0,
            inverted: false,
            quality: None,
            cell: Some((g.mesh.id(), index)),
        });
        ui.set_mesh_clip_enabled(true);
    } else {
        state.mesh_view.center_x = c.center.x;
        state.mesh_view.center_y = c.center.y;
        state.mesh_view.pixels_per_unit = (f64::from(PREVIEW_HEIGHT.min(PREVIEW_WIDTH))
            / (6.0 * radius.max(1e-12)))
        .clamp(ViewTransform::MIN_ZOOM, ViewTransform::MAX_ZOOM);
    }
    state.show_mesh = true;
    state.show_geometry_3d = false;
}

pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_inspection_apply(move |action| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        if action == 6 {
            let index = state.workbench.mesh_render_cache().and_then(|cache| {
                let values = cache.quality().values(state.mesh_quality_metric);
                cache
                    .quality()
                    .bad_cells(state.mesh_quality_metric, state.mesh_quality_threshold)
                    .into_iter()
                    .max_by(|a, b| {
                        if state.mesh_quality_metric == MeshQualityMetric::CellMeasure {
                            values[*b].total_cmp(&values[*a])
                        } else {
                            values[*a].total_cmp(&values[*b])
                        }
                    })
            });
            if let Some(index) = index {
                let id = state.workbench.mesh().unwrap().mesh.id();
                state
                    .workbench
                    .set_mesh_selection(Some(MeshSelection::cell(id, index)));
                focus_cell(&ui, &mut state, index);
            } else {
                state.log("No cells violate the active quality threshold");
            }
            refresh_ui(&ui, &state);
            return;
        }
        if action == 4 || action == 5 {
            let result = (|| -> Result<(), String> {
                let generated = state.workbench.mesh().ok_or("Generate a mesh first")?;
                let cache = state
                    .workbench
                    .mesh_render_cache()
                    .ok_or("Mesh quality cache is unavailable")?;
                let ext = if action == 4 { "csv" } else { "json" };
                if let Some(path) = rfd::FileDialog::new()
                    .set_file_name(format!("mesh-quality.{ext}"))
                    .add_filter("Mesh quality", &[ext])
                    .save_file()
                {
                    let text = if action == 4 {
                        flursys::workbench::mesh_report::csv(&generated.mesh, cache)?
                    } else {
                        flursys::workbench::mesh_report::json(
                            &generated.mesh,
                            cache,
                            state.workbench.mesh_recipe(),
                        )?
                    };
                    result_analysis::atomic_text(&path, &text)?;
                }
                Ok(())
            })();
            if let Err(e) = result {
                state.log(format!("Mesh report export failed: {e}"));
            }
            refresh_ui(&ui, &state);
            return;
        }
        if action == 2 {
            state.mesh_clip = Some(MeshClip {
                axis: 0,
                fraction: 1.0,
                inverted: false,
                quality: Some((state.mesh_quality_metric, state.mesh_quality_threshold)),
                cell: None,
            });
            ui.set_mesh_clip_enabled(true);
            state.show_mesh = state.workbench.has_mesh();
            state.show_geometry_3d = false;
            state.mesh_color_mode = MeshColorMode::Quality;
            refresh_ui(&ui, &state);
            return;
        }
        state.mesh_clip = if action == 1 || !ui.get_mesh_clip_enabled() {
            None
        } else {
            Some(MeshClip {
                axis: ui.get_mesh_clip_axis().clamp(0, 2) as usize,
                fraction: f64::from(ui.get_mesh_clip_position().clamp(0, 100)) / 100.,
                inverted: ui.get_mesh_clip_invert(),
                quality: None,
                cell: None,
            })
        };
        if action == 1 {
            ui.set_mesh_clip_enabled(false);
        }
        state.workbench.set_mesh_selection(None);
        state.workbench.set_mesh_hover(None);
        state.show_mesh = state.workbench.has_mesh();
        state.show_geometry_3d = false;
        refresh_ui(&ui, &state);
    });
}
