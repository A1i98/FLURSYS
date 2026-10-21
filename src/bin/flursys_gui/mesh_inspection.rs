//! Visual cutaways and quantitative quality summaries. No mesh topology is edited.
use super::*;
use flursys::{MeshRenderCache, UnstructuredMesh};

#[derive(Clone, Copy, Debug)]
pub(super) struct MeshClip {
    pub axis: usize,
    pub fraction: f64,
    pub inverted: bool,
}
impl MeshClip {
    pub fn mask(self, mesh: &UnstructuredMesh, bounds: (Vec3, Vec3)) -> Vec<bool> {
        let coordinate = |p: Vec3| match self.axis {
            0 => p.x,
            1 => p.y,
            _ => p.z,
        };
        let low = coordinate(bounds.0);
        let high = coordinate(bounds.1);
        let cut = low + (high - low) * self.fraction.clamp(0., 1.);
        mesh.cells()
            .iter()
            .map(|c| {
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
    let mask = clip.mask(mesh, cache.bounds());
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
        if *v > state.mesh_quality_threshold {
            above += 1;
        }
    }
    let peak = bins.iter().copied().max().unwrap_or(1).max(1) as f32;
    ui.set_mesh_quality_bins(ModelRc::new(VecModel::from(
        bins.iter().map(|v| *v as f32 / peak).collect::<Vec<_>>(),
    )));
    ui.set_mesh_quality_low(format!("{lo:.3e}").into());
    ui.set_mesh_quality_high(format!("{hi:.3e}").into());
    ui.set_mesh_quality_stats(format!("{} · all {} cells\nMin {lo:.4e} · mean {:.4e} · max {hi:.4e}\n{} / {} ({:.2}%) above {:.4e}\n{} non-finite · histogram: 10 equal-width bins",state.mesh_quality_metric.label(),values.len(),sum/finite.len() as f64,above,finite.len(),100.*above as f64/finite.len() as f64,state.mesh_quality_threshold,values.len()-finite.len()).into());
}
pub(super) fn bind(ui: &MainWindow, state: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let shared = state.clone();
    ui.on_mesh_inspection_apply(move |action| {
        let Some(ui) = weak.upgrade() else { return };
        let mut state = shared.borrow_mut();
        state.mesh_clip = if action == 1 || !ui.get_mesh_clip_enabled() {
            None
        } else {
            Some(MeshClip {
                axis: ui.get_mesh_clip_axis().clamp(0, 2) as usize,
                fraction: f64::from(ui.get_mesh_clip_position().clamp(0, 100)) / 100.,
                inverted: ui.get_mesh_clip_invert(),
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
