use super::*;
use cad_render::{Camera, StandardView};
use egui::{Pos2, Rect, Shape, Stroke};
fn cp(p: Vec3) -> cad_geom::Vec3 {
    cad_geom::Vec3::new(p.x, p.y, p.z)
}
#[derive(Clone)]
pub struct View {
    pub camera: Camera,
    pub radius: f64,
    home: Camera,
    matrix: cad_render::Mat4,
    aspect: f64,
}
impl Default for View {
    fn default() -> Self {
        let mut camera = Camera::default();
        camera.distance = 3.;
        camera.set_view(StandardView::Top);
        Self {
            home: camera,
            camera,
            radius: 1.,
            matrix: cad_render::Mat4::IDENTITY,
            aspect: 1.,
        }
    }
}
impl View {
    fn fit(&mut self, points: impl Iterator<Item = Vec3>, spatial: bool) {
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for p in points {
            lo.x = lo.x.min(p.x);
            lo.y = lo.y.min(p.y);
            lo.z = lo.z.min(p.z);
            hi.x = hi.x.max(p.x);
            hi.y = hi.y.max(p.y);
            hi.z = hi.z.max(p.z);
        }
        if !lo.x.is_finite() {
            return;
        }
        self.radius = ((hi - lo).norm() * 0.5).max(1e-9);
        self.camera.target = cp((lo + hi) * 0.5);
        self.camera.distance = self.radius * 3.2 / self.aspect.min(1.).max(0.05);
        self.camera.set_view(if spatial {
            StandardView::Iso
        } else {
            StandardView::Top
        });
        self.home = self.camera;
    }
    pub fn fit_geometry(&mut self, g: &GeometryTopology) {
        self.fit(
            g.vertices()
                .map(|v| v.position)
                .chain(g.renderable_faces().into_iter().flat_map(|f| f.vertices)),
            g.bodies().next().is_some(),
        );
    }
    pub fn fit_mesh(&mut self, m: &UnstructuredMesh) {
        self.fit(
            m.points().iter().map(|p| p.position),
            m.dimension() == MeshDimension::ThreeD,
        );
    }
    pub(super) fn screen(&self, p: Vec3, r: Rect) -> Option<(Pos2, f64)> {
        let c = self.matrix.apply(cp(p));
        if c[3] <= 1e-12 {
            return None;
        }
        let x = c[0] / c[3];
        let y = c[1] / c[3];
        let z = c[2] / c[3];
        Some((
            r.min
                + egui::vec2(
                    ((x + 1.) * 0.5 * r.width() as f64) as f32,
                    ((1. - y) * 0.5 * r.height() as f64) as f32,
                ),
            z,
        ))
    }
}
pub fn edge_path(g: &GeometryTopology, e: &EdgeGeometry) -> Option<Vec<Vec3>> {
    match *e {
        EdgeGeometry::Kernel { id, .. } => Some(g.cad_edge(id)?.path.clone()),
        EdgeGeometry::Line { start, end } => {
            Some(vec![g.vertex(start)?.position, g.vertex(end)?.position])
        }
        EdgeGeometry::CircularArc { start, center, end } => {
            let (a, c, b) = (
                g.vertex(start)?.position,
                g.vertex(center)?.position,
                g.vertex(end)?.position,
            );
            let r = (a - c).norm();
            let first = (a.y - c.y).atan2(a.x - c.x);
            let sweep = ((b.y - c.y).atan2(b.x - c.x) - first).rem_euclid(std::f64::consts::TAU);
            Some(
                (0..=64)
                    .map(|i| {
                        let angle = first + sweep * i as f64 / 64.;
                        Vec3::new(c.x + r * angle.cos(), c.y + r * angle.sin(), a.z)
                    })
                    .collect(),
            )
        }
    }
}
fn scalar_color(x: f64, lo: f64, hi: f64, palette: usize) -> Color32 {
    let t = ((x - lo) / (hi - lo).max(1e-30)).clamp(0., 1.);
    let stops = match palette {
        0 => [
            [48., 18., 59.],
            [50., 100., 235.],
            [25., 215., 130.],
            [245., 215., 45.],
            [180., 4., 38.],
        ],
        2 => [
            [36., 67., 169.],
            [100., 150., 220.],
            [225., 225., 225.],
            [224., 125., 95.],
            [164., 20., 38.],
        ],
        3 => [
            [20., 20., 20.],
            [75., 75., 75.],
            [130., 130., 130.],
            [185., 185., 185.],
            [245., 245., 245.],
        ],
        _ => [
            [68., 1., 84.],
            [59., 82., 139.],
            [33., 145., 140.],
            [94., 201., 98.],
            [253., 231., 37.],
        ],
    };
    let k = t * 4.;
    let a = (k.floor() as usize).min(3);
    let f = (k - a as f64).clamp(0., 1.);
    let c =
        std::array::from_fn::<_, 3, _>(|i| (stops[a][i] * (1. - f) + stops[a + 1][i] * f) as u8);
    Color32::from_rgb(c[0], c[1], c[2])
}
fn distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let d = b - a;
    let t = ((p - a).dot(d) / d.length_sq().max(1e-10)).clamp(0., 1.);
    p.distance(a + d * t)
}
impl App {
    pub fn viewport(&mut self, ui: &mut egui::Ui) {
        if self.stage == Stage::Mesh && self.page == 2 {
            self.show_geometry = false;
        }
        ui.horizontal_wrapped(|ui| {
            if matches!(self.stage, Stage::Mesh | Stage::Setup)
                && !(self.stage == Stage::Mesh && self.page == 2)
            {
                ui.selectable_value(&mut self.show_geometry, true, "CAD geometry");
                ui.selectable_value(&mut self.show_geometry, false, "Mesh");
                ui.separator();
            }
            if self.post_view() {
                ui.toggle_value(&mut self.probe_enabled, "Probe points");
                if self.plane_index == 0
                    && self
                        .result
                        .as_ref()
                        .is_some_and(|d| d.mesh().dimension() == MeshDimension::TwoD)
                {
                    ui.toggle_value(&mut self.seed_streamlines, "Seed flow paths");
                }
                if widgets::icon_button(ui, "clear", "Clear probes", true).clicked() {
                    self.probes.clear();
                }
            } else {
                for (i, label) in ["Edge", "Face", "Vertex", "Body"].iter().enumerate() {
                    if ui.selectable_label(self.pick == i, *label).clicked() {
                        self.pick = i;
                    }
                }
                if widgets::icon_button(ui, "clear", "Clear", true).clicked() {
                    self.selected.clear();
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if self.stage == Stage::Mesh && !self.show_geometry && self.page != 2 {
                    ui.checkbox(&mut self.mesh_wireframe, "Wireframe");
                }
            });
        });
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        self.viewport_rect = Some(rect);
        let painter = ui.painter_at(rect);
        painter.rect_filled(
            rect,
            0.,
            if self.preferences.light {
                Color32::from_rgb(232, 237, 244)
            } else {
                Color32::from_rgb(31, 37, 47)
            },
        );
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0. {
                self.camera.camera.zoom_at(
                    (scroll as f64 * 0.002 * self.preferences.navigation_speed as f64).exp(),
                    None,
                );
            }
        }
        let moving = response.dragged_by(egui::PointerButton::Middle)
            || response.dragged_by(egui::PointerButton::Primary);
        let box_select = ui.input(|i| i.modifiers.alt) && self.stage == Stage::Mesh;
        if moving && !box_select {
            let d = ui.input(|i| i.pointer.delta()) * self.preferences.navigation_speed;
            if ui.input(|i| i.modifiers.shift)
                || response.dragged_by(egui::PointerButton::Middle)
                || self.session.mesh_dimension() == MeshDimension::TwoD
            {
                self.camera
                    .camera
                    .pan(d.x as f64, d.y as f64, rect.height() as f64);
            } else {
                self.camera.camera.orbit(d.x as f64, d.y as f64);
            }
        }
        if box_select && response.drag_started() {
            self.drag_select = response.interact_pointer_pos();
        }
        let aspect = (rect.width() / rect.height().max(1.)) as f64;
        let adjustment = self.camera.aspect.min(1.).max(0.05) / aspect.min(1.).max(0.05);
        self.camera.camera.distance *= adjustment;
        self.camera.home.distance *= adjustment;
        self.camera.aspect = aspect;
        self.camera.matrix = self.camera.camera.view_proj(
            (rect.width() / rect.height().max(1.)) as f64,
            self.camera.radius,
        );
        let mut pick_polygons: Vec<(GeometrySelectionTarget, Vec<Pos2>, f64)> = Vec::new();
        if self.post_view() && self.plane_index > 0 && self.plane_snapshot.is_some() {
            let snapshot = self.plane_snapshot.as_ref().unwrap();
            let style = &snapshot.plane.display;
            let values: Vec<_> = snapshot
                .samples
                .iter()
                .map(|s| plane_monitor::sample_scalar(snapshot, s, style.field))
                .collect();
            let min = values.iter().copied().fold(f64::INFINITY, f64::min);
            let max = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            let [lo, hi] = style
                .range
                .filter(|[a, b]| a.is_finite() && b.is_finite() && b > a)
                .unwrap_or([min, max]);
            let mut mesh = egui::Mesh::default();
            let mut edges = Vec::new();
            let node_values = plane_nodes(snapshot, &values);
            let mut contour_triangles = Vec::new();
            for (s, v) in snapshot.samples.iter().zip(values) {
                let points: Vec<_> = s
                    .geometry
                    .polygon
                    .iter()
                    .filter_map(|p| self.camera.screen(*p, rect).map(|s| s.0))
                    .collect();
                let c = scalar_color(v, lo, hi, style.palette);
                if !matches!(style.plot, 1 | 3) {
                    polygon(&mut mesh, &points, c);
                }
                if matches!(style.plot, 1 | 2) {
                    for (a, b) in s
                        .geometry
                        .polygon
                        .iter()
                        .zip(s.geometry.polygon.iter().cycle().skip(1))
                        .take(s.geometry.polygon.len())
                    {
                        let get = |p: &Vec3| {
                            let (sum, n) = node_values[&point_key(*p)];
                            sum / n as f64
                        };
                        contour_triangles.push([(s.geometry.point, v), (*a, get(a)), (*b, get(b))]);
                    }
                }
                if style.edges {
                    for pair in points
                        .iter()
                        .zip(points.iter().cycle().skip(1))
                        .take(points.len())
                    {
                        edges.push([*pair.0, *pair.1]);
                    }
                }
            }
            painter.add(Shape::mesh(mesh));
            draw_contours(
                &painter,
                &self.camera,
                rect,
                &contour_triangles,
                lo,
                hi,
                style.levels,
            );
            for edge in edges {
                painter.line_segment(edge, Stroke::new(0.6, Color32::from_black_alpha(140)));
            }
            if style.vectors {
                let stride = (snapshot.samples.len() / style.vector_density.max(1)).max(1);
                let vmax = snapshot
                    .samples
                    .iter()
                    .map(|s| s.velocity.norm())
                    .fold(0., f64::max)
                    .max(1e-30);
                for s in snapshot.samples.iter().step_by(stride) {
                    let n = snapshot.plane.normal();
                    let v = s.velocity - n * s.velocity.dot(n);
                    let len = if style.normalized_vectors {
                        v.norm().max(1e-30)
                    } else {
                        vmax
                    };
                    let end = s.geometry.point
                        + v * (self.camera.radius * 0.07 * style.vector_scale / len);
                    if let (Some((a, _)), Some((b, _))) = (
                        self.camera.screen(s.geometry.point, rect),
                        self.camera.screen(end, rect),
                    ) {
                        painter.arrow(a, b - a, Stroke::new(1., Color32::WHITE));
                    }
                }
            }
            if style.normal_markers {
                let n = snapshot.plane.normal();
                let stride = snapshot
                    .samples
                    .len()
                    .div_ceil(style.vector_density.max(1))
                    .max(1);
                for sample in snapshot.samples.iter().step_by(stride) {
                    if let Some((point, _)) = self.camera.screen(sample.geometry.point, rect) {
                        let normal = sample.velocity.dot(n);
                        if normal > 1e-10 {
                            painter.circle_filled(point, 2., Color32::from_rgb(85, 210, 255));
                        } else if normal < -1e-10 {
                            painter.line_segment(
                                [point - egui::vec2(2., 2.), point + egui::vec2(2., 2.)],
                                Stroke::new(1., Color32::from_rgb(255, 170, 75)),
                            );
                            painter.line_segment(
                                [point + egui::vec2(-2., 2.), point + egui::vec2(2., -2.)],
                                Stroke::new(1., Color32::from_rgb(255, 170, 75)),
                            );
                        }
                    }
                }
            }
            legend(&painter, rect, lo, hi, style.palette);
            painter.text(
                rect.left_top() + egui::vec2(14., 12.),
                egui::Align2::LEFT_TOP,
                format!(
                    "{} · {} · iteration {}{}",
                    snapshot.plane.name,
                    [
                        "Speed (m/s)",
                        "Pressure p/ρ (m²/s²)",
                        "Velocity X (m/s)",
                        "Velocity Y (m/s)",
                        "Velocity Z (m/s)",
                        "Normal velocity (m/s)"
                    ][style.field.min(5)],
                    snapshot.iteration,
                    snapshot
                        .physical_time
                        .map(|t| format!(" · t={t:.6e} s"))
                        .unwrap_or_default()
                ),
                egui::FontId::proportional(12.),
                Color32::WHITE,
            );
        } else if let Some(cache) = if self.post_view() {
            self.result_cache.as_ref().map(|c| c.mesh_cache())
        } else if self.show_geometry && matches!(self.stage, Stage::Mesh | Stage::Setup) {
            None
        } else {
            self.mesh_cache.as_ref()
        } {
            let palette = if self.post_view() {
                self.result_palette
            } else {
                1
            };
            let data = self.result.as_ref().filter(|_| self.post_view());
            let transport = self
                .transport
                .latest
                .as_ref()
                .filter(|_| self.stage == Stage::Setup && self.page == 5);
            let quality = if self.stage == Stage::Mesh && self.page == 2 {
                Some(cache.quality().values(match self.mesh_quality {
                    1 => MeshQualityMetric::AspectRatio,
                    2 => MeshQualityMetric::Skewness,
                    3 => MeshQualityMetric::CellMeasure,
                    4 => MeshQualityMetric::AdjacentVolumeRatio,
                    _ => MeshQualityMetric::NonOrthogonality,
                }))
            } else {
                None
            };
            let values = data
                .map(|d| d.scalar_values(self.result_field))
                .or_else(|| {
                    transport.map(|s| match self.transport.field {
                        1 => s.phase_fraction.as_slice(),
                        2 => s.density.as_slice(),
                        _ => s.temperature.as_slice(),
                    })
                })
                .or(quality.as_deref());
            let source_mesh = data
                .map(|d| d.mesh())
                .or_else(|| self.session.mesh().map(|m| &m.mesh));
            let range = values.as_ref().map(|v| {
                (
                    v.iter()
                        .copied()
                        .filter(|v| v.is_finite())
                        .fold(f64::INFINITY, f64::min),
                    v.iter()
                        .copied()
                        .filter(|v| v.is_finite())
                        .fold(f64::NEG_INFINITY, f64::max),
                )
            });
            let visible_cells: Vec<bool> = source_mesh
                .map(|source| {
                    let component = |p: Vec3| match self.mesh_cut_axis {
                        1 => p.y,
                        2 => p.z,
                        _ => p.x,
                    };
                    let low = source
                        .points()
                        .iter()
                        .map(|p| component(p.position))
                        .fold(f64::INFINITY, f64::min);
                    let high = source
                        .points()
                        .iter()
                        .map(|p| component(p.position))
                        .fold(f64::NEG_INFINITY, f64::max);
                    source
                        .cells()
                        .iter()
                        .enumerate()
                        .map(|(i, cell)| {
                            (self.stage != Stage::Mesh
                                || !self.mesh_cut
                                || component(cell.center)
                                    <= low + (high - low) * self.mesh_cut_fraction)
                                && (self.stage != Stage::Mesh
                                    || self.page != 2
                                    || self.quality_above.is_none_or(|threshold| {
                                        quality.as_ref().is_some_and(|q| q[i] >= threshold)
                                    }))
                        })
                        .collect()
                })
                .unwrap_or_default();
            let projected_positions: Vec<_> = cache
                .positions()
                .iter()
                .map(|p| self.camera.screen(*p, rect))
                .collect();
            let range = if self.post_view() {
                self.result_range
                    .filter(|[a, b]| a.is_finite() && b.is_finite() && b > a)
                    .map(|[a, b]| (a, b))
                    .or(range)
            } else if self.stage == Stage::Mesh && self.page == 2 {
                self.quality_range.map(|[a, b]| (a, b)).or(range)
            } else {
                range
            };
            if let (Some(source), Some(v), Some((lo, hi))) = (source_mesh, values.as_ref(), range) {
                if source.dimension() == MeshDimension::TwoD {
                    let mut fill = egui::Mesh::default();
                    let mut contour_triangles = Vec::new();
                    let mut nodes = vec![(0.0, 0usize); source.points().len()];
                    for (index, cell) in source.cells().iter().enumerate() {
                        let mut vertices = std::collections::BTreeSet::new();
                        for &face in &cell.faces {
                            vertices.extend(source.faces()[face].vertices.iter().copied());
                        }
                        for vertex in vertices {
                            nodes[vertex].0 += v[index];
                            nodes[vertex].1 += 1;
                        }
                    }
                    for (index, cell) in source.cells().iter().enumerate() {
                        if !visible_cells[index] {
                            continue;
                        }
                        for &face in &cell.faces {
                            let ids = &source.faces()[face].vertices;
                            if ids.len() != 2 {
                                continue;
                            }
                            let a = source.points()[ids[0]].position;
                            let b = source.points()[ids[1]].position;
                            if let (Some((c, _)), Some((a_screen, _)), Some((b_screen, _))) = (
                                self.camera.screen(cell.center, rect),
                                self.camera.screen(a, rect),
                                self.camera.screen(b, rect),
                            ) {
                                polygon(
                                    &mut fill,
                                    &[c, a_screen, b_screen],
                                    scalar_color(v[index], lo, hi, palette),
                                );
                            }
                            contour_triangles.push([
                                (cell.center, v[index]),
                                (a, nodes[ids[0]].0 / nodes[ids[0]].1.max(1) as f64),
                                (b, nodes[ids[1]].0 / nodes[ids[1]].1.max(1) as f64),
                            ]);
                        }
                    }
                    if !self.post_view() || !matches!(self.result_plot, 1 | 3) {
                        painter.add(Shape::mesh(fill));
                    }
                    if self.post_view() && matches!(self.result_plot, 1 | 2) {
                        draw_contours(
                            &painter,
                            &self.camera,
                            rect,
                            &contour_triangles,
                            lo,
                            hi,
                            self.result_levels,
                        );
                    }
                }
            }
            let mut tris = Vec::with_capacity(cache.surface_triangles().len());
            for (tri, face) in cache.surface_triangles().iter().zip(cache.triangle_faces()) {
                if source_mesh.is_some_and(|m| !visible_cells[m.faces()[*face].owner]) {
                    continue;
                }
                let projected: Option<Vec<_>> =
                    tri.iter().map(|i| projected_positions[*i]).collect();
                let Some(p) = projected else {
                    continue;
                };
                let color =
                    if let (Some(v), Some((lo, hi)), Some(mesh)) = (&values, range, source_mesh) {
                        scalar_color(
                            v.get(mesh.faces()[*face].owner).copied().unwrap_or(lo),
                            lo,
                            hi,
                            palette,
                        )
                    } else {
                        let q = cache.face_patch_indices()[*face].unwrap_or(0);
                        [
                            Color32::from_rgb(64, 93, 121),
                            Color32::from_rgb(70, 115, 130),
                            Color32::from_rgb(95, 98, 143),
                        ][q % 3]
                    };
                tris.push((
                    [p[0].0, p[1].0, p[2].0],
                    p.iter().map(|p| p.1).sum::<f64>() / 3.,
                    color,
                ));
            }
            if self.stage == Stage::Mesh && (self.mesh_cut || self.quality_above.is_some()) {
                if let (Some(source), Some(v), Some((lo, hi))) =
                    (source_mesh, values.as_ref(), range)
                {
                    if source.dimension() == MeshDimension::ThreeD {
                        for face in source.faces() {
                            let Some(neighbour) = face.neighbour else {
                                continue;
                            };
                            if visible_cells[face.owner] == visible_cells[neighbour] {
                                continue;
                            }
                            let owner = if visible_cells[face.owner] {
                                face.owner
                            } else {
                                neighbour
                            };
                            for i in 1..face.vertices.len().saturating_sub(1) {
                                let ids =
                                    [face.vertices[0], face.vertices[i], face.vertices[i + 1]];
                                let points: Option<Vec<_>> =
                                    ids.iter().map(|i| projected_positions[*i]).collect();
                                if let Some(p) = points {
                                    tris.push((
                                        [p[0].0, p[1].0, p[2].0],
                                        p.iter().map(|p| p.1).sum::<f64>() / 3.,
                                        scalar_color(v[owner], lo, hi, palette),
                                    ));
                                }
                            }
                        }
                    }
                }
            }
            tris.sort_by(|a, b| b.1.total_cmp(&a.1));
            let mut mesh = egui::Mesh::default();
            for (p, _, color) in tris {
                polygon(&mut mesh, &p, color);
            }
            if (!self.mesh_wireframe || self.stage != Stage::Mesh || self.page == 2)
                && !(self.post_view() && matches!(self.result_plot, 1 | 3))
            {
                painter.add(Shape::mesh(mesh));
            }
            if self.post_view() && matches!(self.result_plot, 1 | 2) {
                if let (Some(v), Some((lo, hi)), Some(source)) = (&values, range, source_mesh) {
                    let mut nodes = vec![(0.0, 0usize); cache.positions().len()];
                    for (t, f) in cache.surface_triangles().iter().zip(cache.triangle_faces()) {
                        let value = v[source.faces()[*f].owner];
                        for &i in t {
                            nodes[i].0 += value;
                            nodes[i].1 += 1;
                        }
                    }
                    let triangles: Vec<_> = cache
                        .surface_triangles()
                        .iter()
                        .map(|t| {
                            t.map(|i| (cache.positions()[i], nodes[i].0 / nodes[i].1.max(1) as f64))
                        })
                        .collect();
                    draw_contours(
                        &painter,
                        &self.camera,
                        rect,
                        &triangles,
                        lo,
                        hi,
                        self.result_levels,
                    );
                }
            }

            if data.is_none() {
                if let Some((lo, hi)) = range {
                    legend(&painter, rect, lo, hi, palette);
                    if self.stage == Stage::Mesh && self.page == 2 {
                        let names = [
                            "Non-orthogonality (deg)",
                            "Aspect ratio",
                            "Skewness",
                            "Cell measure",
                            "Adjacent volume ratio",
                        ];
                        painter.text(
                            rect.left_top() + egui::vec2(12., 12.),
                            egui::Align2::LEFT_TOP,
                            names[self.mesh_quality.min(4)],
                            egui::FontId::proportional(12.),
                            ui.visuals().text_color(),
                        );
                    }
                }
            }
            if (data.is_none() || self.result_edges)
                && (self.stage != Stage::Mesh || self.page != 2 || self.quality_edges)
                && !(self.stage == Stage::Mesh && (self.mesh_cut || self.quality_above.is_some()))
            {
                for edge in cache.edges() {
                    if let (Some((a, _)), Some((b, _))) =
                        (projected_positions[edge[0]], projected_positions[edge[1]])
                    {
                        painter.line_segment(
                            [a, b],
                            Stroke::new(
                                0.7,
                                if data.is_some() {
                                    Color32::from_black_alpha(110)
                                } else {
                                    Color32::from_rgb(168, 203, 225)
                                },
                            ),
                        );
                    }
                }
            }
            if let (Some(d), Some((lo, hi))) = (data, range) {
                legend(&painter, rect, lo, hi, palette);
                painter.text(
                    rect.left_top() + egui::vec2(12., 12.),
                    egui::Align2::LEFT_TOP,
                    self.result_field.label(),
                    egui::FontId::proportional(12.),
                    Color32::WHITE,
                );
                if self.result_vectors {
                    let stride = d
                        .mesh()
                        .cell_count()
                        .div_ceil(self.vector_count.max(1))
                        .max(1);
                    let max = d
                        .velocity_magnitude()
                        .iter()
                        .copied()
                        .fold(0., f64::max)
                        .max(1e-30);
                    for (i, c) in d.mesh().cells().iter().enumerate().step_by(stride) {
                        let denominator = if self.vector_normalized {
                            d.velocity()[i].norm().max(1e-30)
                        } else {
                            max
                        };
                        let end = c.center
                            + d.velocity()[i]
                                * (self.camera.radius * 0.06 * self.vector_scale.max(1e-6)
                                    / denominator);
                        if let (Some((a, _)), Some((b, _))) = (
                            self.camera.screen(c.center, rect),
                            self.camera.screen(end, rect),
                        ) {
                            painter.arrow(a, b - a, Stroke::new(1., Color32::WHITE));
                        }
                    }
                }
            }
        } else {
            let native_gpu = self.render.is_some();
            if native_gpu {
                self.geometry_gpu.paint(
                    &painter,
                    rect,
                    &self.camera,
                    &self.geometry_faces,
                    &self.geometry_edges,
                    &self.selected,
                    self.pick,
                    &self.preferences,
                );
            }
            let mut triangles = Vec::new();
            for f in &self.geometry_faces {
                let pts: Vec<_> = f
                    .vertices
                    .iter()
                    .filter_map(|p| self.camera.screen(*p, rect))
                    .collect();
                if pts.len() < 3 {
                    continue;
                }
                let target = if self.pick == 3 {
                    f.body
                        .map(GeometrySelectionTarget::Body)
                        .unwrap_or(GeometrySelectionTarget::Face(f.face))
                } else {
                    GeometrySelectionTarget::Face(f.face)
                };
                let depth = pts.iter().map(|p| p.1).sum::<f64>() / pts.len() as f64;
                let points = pts.into_iter().map(|p| p.0).collect::<Vec<_>>();
                pick_polygons.push((target, points, depth));
                if native_gpu {
                    continue;
                }
                for tri in triangulate(f) {
                    if let Some(p) = tri
                        .into_iter()
                        .map(|p| self.camera.screen(p, rect))
                        .collect::<Option<Vec<_>>>()
                    {
                        triangles.push((
                            [p[0].0, p[1].0, p[2].0],
                            depth,
                            if self.selected.contains(&target) {
                                Color32::from_rgb(55, 117, 159)
                            } else {
                                Color32::from_rgb(63, 76, 91)
                            },
                        ));
                    }
                }
            }
            triangles.sort_by(|a, b| b.1.total_cmp(&a.1));
            let mut mesh = egui::Mesh::default();
            for (p, _, c) in triangles {
                polygon(&mut mesh, &p, c);
            }
            painter.add(Shape::mesh(mesh));
            for (id, path) in &self.geometry_edges {
                if native_gpu {
                    break;
                }
                let c = if self.selected.contains(&GeometrySelectionTarget::Edge(*id)) {
                    Color32::from_rgb(255, 206, 85)
                } else {
                    Color32::from_rgb(110, 201, 220)
                };
                for w in path.windows(2) {
                    if let (Some((a, _)), Some((b, _))) = (
                        self.camera.screen(w[0], rect),
                        self.camera.screen(w[1], rect),
                    ) {
                        painter.line_segment([a, b], Stroke::new(1.4, c));
                    }
                }
            }
        }
        if self.post_view() {
            for (i, probe) in self.probes.iter().enumerate() {
                if let Some((point, _)) = self.camera.screen(probe.center, rect) {
                    painter.circle_filled(point, 4., Color32::from_rgb(255, 190, 60));
                    painter.text(
                        point + egui::vec2(7., -7.),
                        egui::Align2::LEFT_BOTTOM,
                        format!("P{}", i + 1),
                        egui::FontId::proportional(12.),
                        ui.visuals().text_color(),
                    );
                }
            }
            if self.flow_playing {
                for path in &self.streamlines {
                    for offset in [0., 0.25, 0.5, 0.75] {
                        let phase = (self.flow_phase + offset) % 1.;
                        if let Some(point) = path
                            .points
                            .get((phase * path.points.len().saturating_sub(1) as f64) as usize)
                        {
                            if let Some((point, _)) = self.camera.screen(*point, rect) {
                                painter.circle_filled(point, 3., Color32::from_rgb(255, 220, 95));
                            }
                        }
                    }
                }
            }
        }
        let compact_navigation = rect.width() < 300. || rect.height() < 260.;
        let navigation = if compact_navigation {
            Rect::from_min_size(
                rect.right_top() + egui::vec2(-40., 8.),
                egui::vec2(30., 30.),
            )
        } else {
            Rect::from_min_size(
                rect.right_top() + egui::vec2(-145., 8.),
                egui::vec2(135., 125.),
            )
        };
        self.navigator(ui, &painter, navigation);
        if response.double_clicked()
            && !navigation.contains(response.interact_pointer_pos().unwrap_or_default())
        {
            self.camera.camera = self.camera.home;
        }
        if let Some(start) = self.drag_select {
            if let Some(end) = response.interact_pointer_pos() {
                let box_rect = Rect::from_two_pos(start, end);
                painter.rect_stroke(
                    box_rect,
                    0.,
                    Stroke::new(1., widgets::ACCENT),
                    egui::StrokeKind::Inside,
                );
                if response.drag_stopped() {
                    let mut targets = Vec::new();
                    if self.pick == 0 {
                        for (id, path) in &self.geometry_edges {
                            if path
                                .iter()
                                .filter_map(|p| self.camera.screen(*p, rect))
                                .any(|p| box_rect.contains(p.0))
                            {
                                targets.push(GeometrySelectionTarget::Edge(*id));
                            }
                        }
                    } else {
                        for (target, points, _) in &pick_polygons {
                            if points.iter().any(|p| box_rect.contains(*p)) {
                                targets.push(*target);
                            }
                        }
                    }
                    for t in targets {
                        if !self.selected.contains(&t) {
                            self.selected.push(t);
                        }
                    }
                    self.drag_select = None;
                }
            }
        }
        if response.clicked() {
            if let Some(p) = response
                .interact_pointer_pos()
                .filter(|p| !navigation.contains(*p))
            {
                if self.post_view() {
                    if self.plane_index > 0 && self.plane_snapshot.is_some() {
                        self.pick_plane_probe(p, rect);
                    } else if let Some(d) = &self.result {
                        let (o, v) = self.camera.camera.ray(
                            (p.x - rect.left()) as f64,
                            (p.y - rect.top()) as f64,
                            rect.width() as f64,
                            rect.height() as f64,
                        );
                        if let Some(cache) = &self.result_cache {
                            let selection = if d.mesh().dimension() == MeshDimension::TwoD {
                                let t = -o.z / v.z;
                                cache
                                    .mesh_cache()
                                    .pick_cell_2d((o.x + t * v.x, o.y + t * v.y))
                            } else {
                                cache.mesh_cache().pick_face_3d(
                                    Vec3::new(o.x, o.y, o.z),
                                    Vec3::new(v.x, v.y, v.z),
                                )
                            };
                            if let Some(s) = selection {
                                let cell = match s.target() {
                                    MeshSelectionTarget::Cell(c) => c,
                                    MeshSelectionTarget::Face(f) => d.mesh().faces()[f].owner,
                                };
                                self.selected_probe = d.probe_cell(cell);
                                if self.probe_enabled {
                                    if let Some(probe) = self.selected_probe.clone() {
                                        if !self.probes.iter().any(|p| {
                                            p.cell_index == cell && p.run_id == probe.run_id
                                        }) {
                                            self.probes.push(probe);
                                        }
                                    }
                                }
                                if self.seed_streamlines
                                    && d.mesh().dimension() == MeshDimension::TwoD
                                {
                                    let t = -o.z / v.z;
                                    let seed = Vec3::new(o.x + t * v.x, o.y + t * v.y, 0.0);
                                    if let Ok(field) = StreamlineField::from_dataset(d) {
                                        if let Ok(path) = field.integrate(
                                            seed,
                                            StreamlineDirection::Forward,
                                            StreamlineOptions {
                                                step_size: self.camera.radius * 0.005,
                                                max_steps: 3000,
                                                max_length: self.camera.radius * 15.,
                                                stagnation_speed: 1e-10,
                                            },
                                        ) {
                                            self.streamlines.push(path);
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else {
                    let shift = ui.input(|i| i.modifiers.shift || i.modifiers.ctrl);
                    let target = match self.pick {
                        0 => self
                            .geometry_edges
                            .iter()
                            .filter_map(|(id, path)| {
                                let dist = path
                                    .windows(2)
                                    .filter_map(|w| {
                                        Some(distance(
                                            p,
                                            self.camera.screen(w[0], rect)?.0,
                                            self.camera.screen(w[1], rect)?.0,
                                        ))
                                    })
                                    .fold(f32::INFINITY, f32::min);
                                (dist < 8.).then_some((GeometrySelectionTarget::Edge(*id), dist))
                            })
                            .min_by(|a, b| a.1.total_cmp(&b.1))
                            .map(|p| p.0),
                        2 => self
                            .session
                            .geometry()
                            .vertices()
                            .filter_map(|v| {
                                self.camera.screen(v.position, rect).map(|s| {
                                    (GeometrySelectionTarget::Vertex(v.id), s.0.distance(p))
                                })
                            })
                            .filter(|(_, d)| *d < 9.)
                            .min_by(|a, b| a.1.total_cmp(&b.1))
                            .map(|a| a.0),
                        _ => {
                            let (origin, direction) = self.camera.camera.ray(
                                (p.x - rect.left()) as f64,
                                (p.y - rect.top()) as f64,
                                rect.width() as f64,
                                rect.height() as f64,
                            );
                            let ray = flursys::geometry::Ray3::new(
                                Vec3::new(origin.x, origin.y, origin.z),
                                Vec3::new(direction.x, direction.y, direction.z),
                            );
                            ray.and_then(|r| self.session.geometry().pick_face(r))
                                .map(|hit| {
                                    if self.pick == 3 {
                                        hit.body
                                            .map(GeometrySelectionTarget::Body)
                                            .unwrap_or(GeometrySelectionTarget::Face(hit.face))
                                    } else {
                                        GeometrySelectionTarget::Face(hit.face)
                                    }
                                })
                        }
                    };
                    if !shift {
                        self.selected.clear();
                    }
                    if let Some(t) = target {
                        if let Some(i) = self.selected.iter().position(|x| *x == t) {
                            self.selected.remove(i);
                        } else {
                            self.selected.push(t);
                        }
                    }
                }
            }
        }
        if self.post_view() {
            for path in &self.streamlines {
                for segment in path.points.windows(2) {
                    if let (Some((a, _)), Some((b, _))) = (
                        self.camera.screen(segment[0], rect),
                        self.camera.screen(segment[1], rect),
                    ) {
                        painter.line_segment([a, b], Stroke::new(1.2, Color32::WHITE));
                    }
                }
            }
        }
        painter.text(
            rect.right_bottom() + egui::vec2(-12., -12.),
            egui::Align2::RIGHT_BOTTOM,
            concat!("FLURSYS v", env!("CARGO_PKG_VERSION")),
            egui::FontId::proportional(11.),
            ui.visuals().weak_text_color(),
        );
        painter.text(rect.left_bottom()+egui::vec2(12.,-12.),egui::Align2::LEFT_BOTTOM,"Drag: orbit / 2D pan    Middle or Shift-drag: pan    Wheel: zoom    Alt-drag: box select",egui::FontId::proportional(11.),Color32::from_rgb(156,174,195));
    }
    fn navigator(&mut self, ui: &mut egui::Ui, painter: &egui::Painter, r: Rect) {
        let center = r.center() + egui::vec2(4., 7.);
        if r.width() > 50. {
            painter.circle_stroke(center, 44., Stroke::new(1., Color32::from_gray(80)));
        }
        let home = Rect::from_min_size(r.min, egui::vec2(28., 28.));
        cad_workbench::icons::paint_icon(painter, home, "home", 1.);
        if ui
            .interact(home, egui::Id::new("nav-home"), egui::Sense::click())
            .on_hover_text("Home view")
            .clicked()
        {
            self.camera.camera = self.camera.home;
        }
        if r.width() < 50. {
            return;
        }
        let (right, up, _) = self.camera.camera.basis();
        for (i, (axis, name, c)) in [
            (cad_geom::Vec3::X, "X", Color32::from_rgb(244, 94, 109)),
            (cad_geom::Vec3::Y, "Y", Color32::from_rgb(90, 211, 123)),
            (cad_geom::Vec3::Z, "Z", Color32::from_rgb(83, 145, 246)),
        ]
        .into_iter()
        .enumerate()
        {
            for sign in [-1., 1.] {
                let d = axis * sign;
                let p = center + egui::vec2(d.dot(right) as f32, -d.dot(up) as f32) * 40.;
                let button = Rect::from_center_size(p, egui::vec2(22., 22.));
                painter.circle_filled(p, 10., c.gamma_multiply(if sign > 0. { 1. } else { 0.45 }));
                painter.text(
                    p,
                    egui::Align2::CENTER_CENTER,
                    if sign > 0. {
                        name.to_string()
                    } else {
                        format!("−{name}")
                    },
                    egui::FontId::proportional(10.),
                    Color32::WHITE,
                );
                if ui
                    .interact(
                        button,
                        egui::Id::new(("axis", i, sign as i32)),
                        egui::Sense::click(),
                    )
                    .clicked()
                {
                    self.camera.camera = self.camera.camera.looking_from(d);
                }
            }
        }
    }
}
fn polygon(mesh: &mut egui::Mesh, points: &[Pos2], color: Color32) {
    if points.len() < 3 {
        return;
    }
    let start = mesh.vertices.len() as u32;
    for &pos in points {
        mesh.vertices.push(egui::epaint::Vertex {
            pos,
            uv: egui::epaint::WHITE_UV,
            color,
        });
    }
    for i in 1..points.len() - 1 {
        mesh.indices
            .extend([start, start + i as u32, start + i as u32 + 1]);
    }
}
fn triangulate(f: &flursys::RenderableFace) -> Vec<[Vec3; 3]> {
    let Some(&origin) = f.vertices.first() else {
        return vec![];
    };
    let Some(x) = f
        .vertices
        .iter()
        .map(|p| *p - origin)
        .find(|d| d.norm() > 1e-14)
    else {
        return vec![];
    };
    let x = x / x.norm();
    let Some(n) = f
        .vertices
        .iter()
        .map(|p| x.cross(*p - origin))
        .find(|n| n.norm() > 1e-14)
    else {
        return vec![];
    };
    let y = (n / n.norm()).cross(x);
    let unit = f
        .vertices
        .iter()
        .map(|p| (*p - origin).norm())
        .fold(0., f64::max)
        .max(1e-12);
    let local = |p: &Vec3| {
        let d = *p - origin;
        cad_geom::Vec2::new(d.dot(x) / unit, d.dot(y) / unit)
    };
    let region = cad_geom::Region2 {
        outer: cad_geom::Loop2::polygon(&f.vertices.iter().map(local).collect::<Vec<_>>()),
        holes: f
            .holes
            .iter()
            .map(|h| cad_geom::Loop2::polygon(&h.iter().map(local).collect::<Vec<_>>()))
            .collect(),
    };
    region
        .triangulate(1e-8)
        .into_iter()
        .map(|t| t.map(|p| origin + x * (p.x * unit) + y * (p.y * unit)))
        .collect()
}
fn legend(p: &egui::Painter, r: Rect, lo: f64, hi: f64, palette: usize) {
    if r.width() < 300. || r.height() < 300. {
        let bar = Rect::from_min_max(
            r.left_bottom() + egui::vec2(12., -34.),
            r.right_bottom() + egui::vec2(-12., -24.),
        );
        for i in 0..100 {
            let t = i as f64 / 99.;
            let left = bar.left() + bar.width() * i as f32 / 100.;
            let right = bar.left() + bar.width() * (i + 1) as f32 / 100.;
            p.rect_filled(
                Rect::from_min_max(egui::pos2(left, bar.top()), egui::pos2(right, bar.bottom())),
                0.,
                scalar_color(lo + t * (hi - lo), lo, hi, palette),
            );
        }
        let ink = p.ctx().global_style().visuals.text_color();
        p.text(
            bar.left_bottom() + egui::vec2(0., 3.),
            egui::Align2::LEFT_TOP,
            format!("{lo:.2e}"),
            egui::FontId::proportional(10.),
            ink,
        );
        p.text(
            bar.right_bottom() + egui::vec2(0., 3.),
            egui::Align2::RIGHT_TOP,
            format!("{hi:.2e}"),
            egui::FontId::proportional(10.),
            ink,
        );
        return;
    }
    let bar = Rect::from_min_size(
        r.right_top() + egui::vec2(-85., 150.),
        egui::vec2(14., (r.height() - 220.).max(90.)),
    );
    for i in 0..100 {
        let f = i as f64 / 99.;
        p.rect_filled(
            Rect::from_min_max(
                egui::pos2(
                    bar.left(),
                    bar.bottom() - bar.height() * (i + 1) as f32 / 100.,
                ),
                egui::pos2(bar.right(), bar.bottom() - bar.height() * i as f32 / 100.),
            ),
            0.,
            scalar_color(lo + f * (hi - lo), lo, hi, palette),
        );
    }
    for f in [0., 0.25, 0.5, 0.75, 1.] {
        p.text(
            egui::pos2(bar.right() + 5., bar.bottom() - bar.height() * f as f32),
            egui::Align2::LEFT_CENTER,
            format!("{:.3e}", lo + f * (hi - lo)),
            egui::FontId::proportional(10.),
            Color32::WHITE,
        );
    }
}
fn point_key(p: Vec3) -> [u64; 3] {
    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
}
fn plane_nodes(
    snapshot: &plane_monitor::PlaneSnapshot,
    values: &[f64],
) -> std::collections::HashMap<[u64; 3], (f64, usize)> {
    let mut nodes = std::collections::HashMap::new();
    for (s, v) in snapshot.samples.iter().zip(values) {
        for p in &s.geometry.polygon {
            let e = nodes.entry(point_key(*p)).or_insert((0.0, 0usize));
            e.0 += *v;
            e.1 += 1;
        }
    }
    nodes
}
/// Display-only vertex averaging and linear isolines; stored cell fields stay unchanged.
fn draw_contours(
    painter: &egui::Painter,
    view: &View,
    rect: Rect,
    triangles: &[[(Vec3, f64); 3]],
    lo: f64,
    hi: f64,
    levels: usize,
) {
    if !lo.is_finite() || !hi.is_finite() || hi <= lo {
        return;
    }
    for tri in triangles {
        for level in 1..levels.clamp(3, 32) {
            let iso = lo + (hi - lo) * level as f64 / levels as f64;
            let mut points = Vec::with_capacity(2);
            for i in 0..3 {
                let (a, av) = tri[i];
                let (b, bv) = tri[(i + 1) % 3];
                if (av < iso && bv >= iso) || (bv < iso && av >= iso) {
                    let t = (iso - av) / (bv - av);
                    if let Some((p, _)) = view.screen(a + (b - a) * t, rect) {
                        points.push(p);
                    }
                }
            }
            if points.len() == 2 {
                painter.line_segment(
                    [points[0], points[1]],
                    Stroke::new(0.8, Color32::from_white_alpha(190)),
                );
            }
        }
    }
}

/// Uses SolidCraft's own shaded, depth-tested viewport pipelines.
#[derive(Default)]
pub struct GeometryGpu {
    input_key: u64,
    key: u64,
    slot: cad_workbench::gpu::SceneSlot,
    empty: cad_workbench::gpu::SceneSlot,
}
impl GeometryGpu {
    fn paint(
        &mut self,
        painter: &egui::Painter,
        rect: Rect,
        view: &View,
        faces: &[flursys::RenderableFace],
        edges: &[(flursys::EdgeId, Vec<Vec3>)],
        selected: &[GeometrySelectionTarget],
        pick: usize,
        preferences: &Preferences,
    ) {
        use cad_workbench::gpu::{Frame, GpuScene};
        use std::hash::{Hash, Hasher};
        let mut fingerprint = std::collections::hash_map::DefaultHasher::new();
        for face in faces {
            face.face.hash(&mut fingerprint);
            face.body.hash(&mut fingerprint);
            for p in face.vertices.iter().chain(face.holes.iter().flatten()) {
                p.x.to_bits().hash(&mut fingerprint);
                p.y.to_bits().hash(&mut fingerprint);
                p.z.to_bits().hash(&mut fingerprint);
            }
        }
        for (id, points) in edges {
            id.hash(&mut fingerprint);
            for p in points {
                p.x.to_bits().hash(&mut fingerprint);
                p.y.to_bits().hash(&mut fingerprint);
                p.z.to_bits().hash(&mut fingerprint);
            }
        }
        selected.hash(&mut fingerprint);
        pick.hash(&mut fingerprint);
        preferences.geometry_edges.hash(&mut fingerprint);
        preferences.geometry_shading.hash(&mut fingerprint);
        preferences.edge_width.to_bits().hash(&mut fingerprint);
        let input_key = fingerprint.finish();
        if input_key != self.input_key || self.key == 0 {
            let mut scene = GpuScene::default();
            for face in faces {
                let target = if pick == 3 {
                    face.body
                        .map(GeometrySelectionTarget::Body)
                        .unwrap_or(GeometrySelectionTarget::Face(face.face))
                } else {
                    GeometrySelectionTarget::Face(face.face)
                };
                let color = if selected.contains(&target) {
                    [74, 155, 205, 255]
                } else {
                    [71, 85, 105, 255]
                };
                for triangle in triangulate(face) {
                    let normal = if preferences.geometry_shading {
                        cp((triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]))
                            .normalized()
                            .unwrap_or(cad_geom::Vec3::Z)
                            .to_f32()
                    } else {
                        [0.; 3]
                    };
                    for point in triangle {
                        scene.tri(cp(point).to_f32(), normal, color);
                    }
                }
            }
            if preferences.geometry_edges {
                for (id, points) in edges {
                    let chosen = selected.contains(&GeometrySelectionTarget::Edge(*id));
                    let color = if chosen {
                        [255, 204, 87, 255]
                    } else {
                        [8, 16, 27, 255]
                    };
                    for pair in points.windows(2) {
                        scene.line(
                            cp(pair[0]).to_f32(),
                            cp(pair[1]).to_f32(),
                            color,
                            if chosen {
                                preferences.edge_width + 1.0
                            } else {
                                preferences.edge_width
                            },
                            chosen,
                        );
                    }
                }
            }
            let mut hash = std::collections::hash_map::DefaultHasher::new();
            scene.tris.hash(&mut hash);
            scene.lines.hash(&mut hash);
            scene.overlay.hash(&mut hash);
            let key = hash.finish() | (1 << 63);
            if key != self.key {
                *self.slot.lock().unwrap_or_else(|e| e.into_inner()) = Some(scene);
                self.key = key;
            }
            self.input_key = input_key;
        }
        let key = self.key;
        let eye = view.camera.eye().to_f32();
        let ppp = painter.ctx().pixels_per_point();
        painter.add(egui_wgpu::Callback::new_paint_callback(
            rect,
            GeometryCallback(Frame {
                product: false,
                key,
                slot: self.slot.clone(),
                sketch_key: 0,
                sketch_slot: self.empty.clone(),
                hl_key: 0,
                hl_slot: self.empty.clone(),
                pv_key: 0,
                pv_slot: self.empty.clone(),
                pv_hide: true,
                view_proj: view.matrix.to_f32(),
                back: view.camera.back().to_f32(),
                size_px: [rect.width() * ppp, rect.height() * ppp],
                clip: [0.; 4],
                cap: [0.4, 0.45, 0.5, 1.],
                images: Vec::new(),
                analysis: [0.; 8],
                analysis_colors: [0.; 36],
                access: None,
                outline: preferences.geometry_edges,
                eye: [
                    eye[0],
                    eye[1],
                    eye[2],
                    if view.camera.fov > 0. { 1. } else { 0. },
                ],
                ao_radius: if preferences.ambient_occlusion {
                    (view.radius * 0.08) as f32
                } else {
                    0.
                },
                ssaa: 1.,
            }),
        ));
    }
}

// Separate resource ownership keeps workbench uploads from replacing the CAD
// editor's model/highlight buffers when switching workflow stages.
struct GeometryRenderer(cad_workbench::gpu::Renderer);
struct GeometryCallback(cad_workbench::gpu::Frame);
pub fn install_geometry_renderer(rs: &egui_wgpu::RenderState) {
    let target = cad_workbench::gpu::GpuTarget {
        format: rs.target_format,
        depth: egui_wgpu::depth_format_from_bits(24, 8),
        samples: 1,
        tess: false,
    };
    rs.renderer
        .write()
        .callback_resources
        .insert(GeometryRenderer(cad_workbench::gpu::Renderer::new(
            &rs.device, target, None,
        )));
}
impl egui_wgpu::CallbackTrait for GeometryCallback {
    fn prepare(
        &self,
        device: &egui_wgpu::wgpu::Device,
        queue: &egui_wgpu::wgpu::Queue,
        _screen: &egui_wgpu::ScreenDescriptor,
        encoder: &mut egui_wgpu::wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<egui_wgpu::wgpu::CommandBuffer> {
        if let Some(renderer) = resources.get_mut::<GeometryRenderer>() {
            renderer.0.prepare(device, queue, encoder, &self.0);
        }
        Vec::new()
    }
    fn paint(
        &self,
        info: egui::PaintCallbackInfo,
        pass: &mut egui_wgpu::wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        if let Some(renderer) = resources.get::<GeometryRenderer>() {
            let vp = info.viewport_in_pixels();
            renderer.0.paint(
                &self.0,
                pass,
                [
                    vp.left_px as f32,
                    vp.top_px as f32,
                    vp.width_px as f32,
                    vp.height_px as f32,
                ],
            );
        }
    }
}
