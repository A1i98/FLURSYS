use super::*;
use cad_workbench::gpu::GpuScene;
use egui::{Rect, Stroke};
use std::hash::{Hash, Hasher};
use viewport::scalar_color;

#[derive(Default)]
pub(super) struct MeshScene {
    key: Option<u64>,
    gpu: viewport::GeometryGpu,
    range: Option<(f64, f64)>,
    triangles: usize,
    visible_cells: usize,
    edges: usize,
    reduced_edges: bool,
    software: Vec<([Vec3; 3], Color32)>,
}

pub(super) fn patch_color(
    condition: Option<&workbench::PhysicalBoundaryCondition>,
    index: usize,
) -> Color32 {
    use workbench::PhysicalBoundaryCondition as B;
    match condition {
        Some(
            B::VelocityInlet { .. }
            | B::ParabolicInlet { .. }
            | B::FlowRateInlet { .. }
            | B::TotalPressure { .. },
        ) => Color32::from_rgb(71, 183, 219),
        Some(B::PressureOutlet { .. } | B::PressureOutletBackflow { .. }) => {
            Color32::from_rgb(225, 160, 76)
        }
        Some(B::Symmetry | B::Slip) => Color32::from_rgb(158, 132, 207),
        Some(B::FarField { .. }) => Color32::from_rgb(91, 172, 134),
        Some(B::NoSlipWall | B::MovingWall { .. }) => Color32::from_rgb(112, 142, 168),
        None => [
            Color32::from_rgb(104, 145, 177),
            Color32::from_rgb(91, 160, 161),
            Color32::from_rgb(143, 133, 176),
        ][index % 3],
    }
}

fn triangle(scene: &mut GpuScene, points: [Vec3; 3], normal: Vec3, color: Color32) {
    for p in points {
        scene.tri(
            [p.x as f32, p.y as f32, p.z as f32],
            [normal.x as f32, normal.y as f32, normal.z as f32],
            color.to_array(),
        );
    }
}
fn line(scene: &mut GpuScene, a: Vec3, b: Vec3, color: Color32, width: f32, overlay: bool) {
    scene.line(
        [a.x as f32, a.y as f32, a.z as f32],
        [b.x as f32, b.y as f32, b.z as f32],
        color.to_array(),
        width,
        overlay,
    );
}
fn arrow(scene: &mut GpuScene, a: Vec3, b: Vec3, color: Color32) {
    let d = b - a;
    if d.norm() <= 1e-30 {
        return;
    }
    let mut side = d.cross(Vec3::new(0., 0., 1.));
    if side.norm() < d.norm() * 1e-6 {
        side = d.cross(Vec3::new(0., 1., 0.));
    }
    side = side / side.norm().max(1e-30) * d.norm() * 0.16;
    line(scene, a, b, color, 1.3, true);
    for p in [b - d * 0.25 + side, b - d * 0.25 - side] {
        line(scene, b, p, color, 1.3, true);
    }
}
fn contours(
    scene: &mut GpuScene,
    tri: [(Vec3, f64); 3],
    lo: f64,
    hi: f64,
    levels: usize,
    count: &mut usize,
) {
    if hi <= lo || *count >= 200_000 {
        return;
    }
    for level in 1..levels.clamp(3, 32) {
        let iso = lo + (hi - lo) * level as f64 / levels.clamp(3, 32) as f64;
        let mut pair = Vec::with_capacity(2);
        for i in 0..3 {
            let (a, av) = tri[i];
            let (b, bv) = tri[(i + 1) % 3];
            if (av < iso && bv >= iso) || (bv < iso && av >= iso) {
                pair.push(a + (b - a) * ((iso - av) / (bv - av)));
            }
        }
        if pair.len() == 2 {
            line(
                scene,
                pair[0],
                pair[1],
                Color32::from_rgb(224, 232, 239),
                0.8,
                false,
            );
            *count += 1;
        }
    }
}

impl App {
    pub(super) fn paint_mesh_scene(&mut self, painter: &egui::Painter, rect: Rect) {
        let post = self.post_view();
        let data = self.result.as_ref().filter(|_| post);
        let Some(source) = data
            .map(|d| d.mesh())
            .or_else(|| self.session.mesh().map(|m| &m.mesh))
        else {
            return;
        };
        let Some(cache) = (if post {
            self.result_cache.as_ref().map(|c| c.mesh_cache())
        } else {
            self.mesh_cache.as_deref()
        }) else {
            return;
        };
        let quality_mode = self.stage == Stage::Mesh && self.page == 2;
        let quality = quality_mode.then(|| {
            cache.quality().values(match self.mesh_quality {
                1 => MeshQualityMetric::AspectRatio,
                2 => MeshQualityMetric::Skewness,
                3 => MeshQualityMetric::CellMeasure,
                4 => MeshQualityMetric::AdjacentVolumeRatio,
                _ => MeshQualityMetric::NonOrthogonality,
            })
        });
        let transport = self
            .transport
            .latest
            .as_ref()
            .filter(|_| self.stage == Stage::Setup && self.page == 5);
        let values = data
            .map(|d| d.scalar_values(self.result_field))
            .or_else(|| {
                transport.map(|s| match self.transport.field {
                    1 => s.phase_fraction.as_slice(),
                    2 => s.density.as_slice(),
                    _ => s.temperature.as_slice(),
                })
            })
            .or(quality);
        let requested_range = if post {
            self.result_range
        } else if quality_mode {
            self.quality_range
        } else {
            None
        };
        let cut_active = matches!(self.stage, Stage::Mesh | Stage::Setup) && self.mesh_cut;
        let caps_only = cut_active && self.mesh_section_only;
        let volume = self.mesh_detail == 3
            && !post
            && source.dimension() == MeshDimension::ThreeD
            && !caps_only;
        let filtered = cut_active || quality_mode && self.quality_above.is_some();
        let palette = if post { self.result_palette } else { 1 };
        let mut hash = std::collections::hash_map::DefaultHasher::new();
        cache.mesh_id().hash(&mut hash);
        (
            values.map(|v| v.as_ptr() as usize),
            self.result_iteration,
            self.mesh_quality,
            quality_mode,
            post,
            self.result_plot,
            self.result_levels,
            palette,
            self.mesh_detail,
            self.mesh_volume_edges,
        )
            .hash(&mut hash);
        (
            self.boundary_symbols,
            self.mesh_wireframe,
            self.result_edges,
            self.quality_edges,
            self.mesh_cut,
            self.mesh_cut_axis,
            self.mesh_cut_fraction.to_bits(),
            self.quality_above.map(f64::to_bits),
            self.boundary_patch.as_str(),
            self.preferences.light,
        )
            .hash(&mut hash);
        (
            self.mesh_cut_reverse,
            self.mesh_slab,
            self.mesh_slab_width.to_bits(),
            self.mesh_section_only,
            self.mesh_cut_plane,
            self.mesh_ghost_shell,
            cut_active,
        )
            .hash(&mut hash);
        requested_range.map(|r| r.map(f64::to_bits)).hash(&mut hash);
        (
            self.result_vectors,
            self.vector_count,
            self.vector_scale.to_bits(),
            self.vector_normalized,
            format!("{:?}", self.project.boundaries),
            transport.map(|s| s.step),
        )
            .hash(&mut hash);
        let key = hash.finish();
        if self.mesh_gpu.key != Some(key) {
            let prepared_at = Instant::now();
            let mut scene = GpuScene::default();
            self.mesh_gpu.software.clear();
            let range = requested_range
                .filter(|r| r[0].is_finite() && r[1].is_finite() && r[1] > r[0])
                .map(|r| (r[0], r[1]))
                .or_else(|| {
                    values.map(|v| {
                        v.iter()
                            .filter(|v| v.is_finite())
                            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                                (lo.min(*v), hi.max(*v))
                            })
                    })
                });
            self.mesh_gpu.range = range;
            let (lo, hi) = cache.bounds();
            let section = workbench::mesh_viewport::MeshSection {
                axis: self.mesh_cut_axis,
                fraction: self.mesh_cut_fraction,
                reverse: self.mesh_cut_reverse,
                slab_width: self.mesh_slab.then_some(self.mesh_slab_width),
            };
            let visible = filtered.then(|| {
                source
                    .cells()
                    .iter()
                    .enumerate()
                    .map(|(i, c)| {
                        (!cut_active || section.contains(c.center, (lo, hi)))
                            && (!quality_mode
                                || self
                                    .quality_above
                                    .is_none_or(|t| quality.is_some_and(|v| v[i] >= t)))
                    })
                    .collect::<Vec<_>>()
            });
            self.mesh_gpu.visible_cells = visible
                .as_ref()
                .map_or(source.cell_count(), |v| v.iter().filter(|v| **v).count());
            let is_visible = |i: usize| visible.as_ref().is_none_or(|v| v[i]);
            let colors: Vec<_> = source
                .boundary_patches()
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    patch_color(
                        self.project
                            .boundaries
                            .iter()
                            .find(|b| b.selection == p.name)
                            .map(|b| &b.condition),
                        i,
                    )
                })
                .collect();
            let color = |cell: usize, face: Option<usize>| {
                if let (Some(v), Some((lo, hi))) = (values, range) {
                    scalar_color(v[cell], lo, hi, palette)
                } else if cut_active && face.is_some_and(|f| source.faces()[f].neighbour.is_some())
                {
                    Color32::from_rgb(173, 133, 83)
                } else if face
                    .and_then(|f| cache.face_patch_indices()[f])
                    .is_some_and(|p| source.boundary_patches()[p].name == self.boundary_patch)
                {
                    Color32::from_rgb(179, 151, 93)
                } else {
                    face.and_then(|f| cache.face_patch_indices()[f])
                        .and_then(|p| colors.get(p).copied())
                        .unwrap_or(Color32::from_rgb(99, 135, 163))
                }
            };
            let filled = !(volume
                || post && matches!(self.result_plot, 1 | 3)
                || self.stage == Stage::Mesh
                    && self.mesh_wireframe
                    && self.mesh_detail != 1
                    && !quality_mode);
            let isolines = post && matches!(self.result_plot, 1 | 2);
            let nodes = if isolines {
                let mut n = vec![(0., 0usize); source.points().len()];
                if let Some(v) = values {
                    for f in source.faces() {
                        for &i in &f.vertices {
                            n[i].0 += v[f.owner];
                            n[i].1 += 1;
                            if let Some(c) = f.neighbour {
                                n[i].0 += v[c];
                                n[i].1 += 1;
                            }
                        }
                    }
                }
                Some(n)
            } else {
                None
            };
            let mut triangle_count = 0;
            let mut contour_count = 0;
            let mut emit = |points: [Vec3; 3],
                            normal: Vec3,
                            c: Color32,
                            scalars: Option<[f64; 3]>,
                            fill: bool| {
                if filled && fill {
                    triangle(&mut scene, points, normal, c);
                    triangle_count += 1;
                    if self.render.is_none() && self.mesh_gpu.software.len() < 20_000 {
                        self.mesh_gpu.software.push((points, c));
                    }
                }
                if let (Some(s), Some((lo, hi))) = (scalars, range) {
                    contours(
                        &mut scene,
                        [(points[0], s[0]), (points[1], s[1]), (points[2], s[2])],
                        lo,
                        hi,
                        self.result_levels,
                        &mut contour_count,
                    );
                }
            };
            if filled || isolines {
                if source.dimension() == MeshDimension::TwoD {
                    for (i, cell) in source
                        .cells()
                        .iter()
                        .enumerate()
                        .filter(|(i, _)| is_visible(*i))
                    {
                        if let Some(points) = cache.cell_polygon(i) {
                            for j in 1..points.len().saturating_sub(1) {
                                let at =
                                    |k: usize| Vec3::new(points[k].0, points[k].1, cell.center.z);
                                emit(
                                    [at(0), at(j), at(j + 1)],
                                    Vec3::ZERO,
                                    color(i, None),
                                    None,
                                    true,
                                );
                            }
                        }
                        if let (Some(nodes), Some(v)) = (&nodes, values) {
                            for &f in &cell.faces {
                                let ids = &source.faces()[f].vertices;
                                if ids.len() == 2 {
                                    let p = ids
                                        .iter()
                                        .map(|&n| source.points()[n].position)
                                        .collect::<Vec<_>>();
                                    emit(
                                        [cell.center, p[0], p[1]],
                                        Vec3::ZERO,
                                        color(i, None),
                                        Some([
                                            v[i],
                                            nodes[ids[0]].0 / nodes[ids[0]].1.max(1) as f64,
                                            nodes[ids[1]].0 / nodes[ids[1]].1.max(1) as f64,
                                        ]),
                                        false,
                                    );
                                }
                            }
                        }
                    }
                } else {
                    for (f, face) in source.faces().iter().enumerate() {
                        if caps_only && face.neighbour.is_none() {
                            continue;
                        }
                        let cell = match face.neighbour {
                            None if is_visible(face.owner) => Some(face.owner),
                            Some(n) if filtered && is_visible(face.owner) != is_visible(n) => {
                                Some(if is_visible(face.owner) {
                                    face.owner
                                } else {
                                    n
                                })
                            }
                            _ => None,
                        };
                        let Some(cell) = cell else {
                            continue;
                        };
                        for j in 1..face.vertices.len().saturating_sub(1) {
                            let mut ids =
                                [face.vertices[0], face.vertices[j], face.vertices[j + 1]];
                            if cell != face.owner {
                                ids.swap(1, 2);
                            }
                            let points = ids.map(|i| source.points()[i].position);
                            let s = nodes
                                .as_ref()
                                .map(|n| ids.map(|i| n[i].0 / n[i].1.max(1) as f64));
                            emit(
                                points,
                                if values.is_some() {
                                    Vec3::ZERO
                                } else if cell == face.owner {
                                    face.normal
                                } else {
                                    -face.normal
                                },
                                color(cell, Some(f)),
                                s,
                                true,
                            );
                        }
                    }
                }
            }
            let draw_edges = volume
                || if post {
                    self.result_edges
                } else if quality_mode {
                    self.quality_edges
                } else {
                    self.mesh_detail != 1
                };
            let mut edge_count = 0;
            let mut reduced = false;
            if draw_edges {
                use workbench::mesh_viewport::{wireframe_edges, WireframeScope};
                let scope = if caps_only {
                    WireframeScope::Section
                } else if volume {
                    WireframeScope::Volume
                } else {
                    WireframeScope::Surface
                };
                let limit = if volume {
                    self.mesh_volume_edges
                } else if self.mesh_detail == 2 {
                    0
                } else {
                    150_000
                };
                let wire = match wireframe_edges(source, visible.as_deref(), scope, limit) {
                    Ok(w) => w,
                    Err(e) => {
                        self.feedback = e;
                        return;
                    }
                };
                edge_count = wire.edges.len();
                reduced = wire.reduced;
                for edge in wire.edges {
                    let face = &source.faces()[edge.face];
                    let ink = if volume {
                        if let (Some(values), Some((lo, hi))) = (values, range) {
                            let cell = if is_visible(face.owner) {
                                face.owner
                            } else {
                                face.neighbour.unwrap_or(face.owner)
                            };
                            scalar_color(values[cell], lo, hi, palette)
                        } else if face.neighbour.is_none() {
                            Color32::from_rgb(158, 188, 210)
                        } else {
                            Color32::from_rgb(91, 128, 157)
                        }
                    } else if post || quality_mode {
                        Color32::from_rgb(40, 52, 67)
                    } else {
                        Color32::from_rgb(178, 198, 214)
                    };
                    line(
                        &mut scene,
                        source.points()[edge.vertices[0]].position,
                        source.points()[edge.vertices[1]].position,
                        ink,
                        if volume { 0.6 } else { 0.7 },
                        false,
                    );
                }
            }
            if !post && self.boundary_symbols && !caps_only {
                for patch in source.boundary_patches() {
                    let selected = self.boundary_patch == patch.name;
                    let condition = self
                        .project
                        .boundaries
                        .iter()
                        .find(|b| b.selection == patch.name)
                        .map(|b| &b.condition);
                    let ink = if selected {
                        Color32::from_rgb(255, 208, 103)
                    } else {
                        patch_color(condition, 0)
                    };
                    for &f in patch.face_indices.iter().step_by(
                        patch
                            .face_indices
                            .len()
                            .div_ceil(if selected { 16 } else { 8 })
                            .max(1),
                    ) {
                        let face = &source.faces()[f];
                        if !is_visible(face.owner) {
                            continue;
                        }
                        let normal = face.normal * self.camera.radius * 0.055;
                        use workbench::PhysicalBoundaryCondition as B;
                        match condition {
                            Some(
                                B::VelocityInlet { .. }
                                | B::ParabolicInlet { .. }
                                | B::FlowRateInlet { .. }
                                | B::TotalPressure { .. },
                            ) => arrow(&mut scene, face.center + normal, face.center, ink),
                            Some(B::PressureOutlet { .. } | B::PressureOutletBackflow { .. }) => {
                                arrow(&mut scene, face.center, face.center + normal, ink)
                            }
                            _ => {
                                let side = normal.cross(Vec3::new(0., 0., 1.));
                                let side = if side.norm() > 1e-20 {
                                    side
                                } else {
                                    normal.cross(Vec3::new(0., 1., 0.))
                                };
                                line(
                                    &mut scene,
                                    face.center - side * 0.4,
                                    face.center + side * 0.4,
                                    ink,
                                    2.,
                                    true,
                                );
                            }
                        }
                    }
                }
            }
            if let Some(d) = data.filter(|_| self.result_vectors) {
                let max = d
                    .velocity_magnitude()
                    .iter()
                    .copied()
                    .fold(0., f64::max)
                    .max(1e-30);
                for (i, cell) in source.cells().iter().enumerate().step_by(
                    source
                        .cell_count()
                        .div_ceil(self.vector_count.clamp(1, 5_000))
                        .max(1),
                ) {
                    let denom = if self.vector_normalized {
                        d.velocity()[i].norm().max(1e-30)
                    } else {
                        max
                    };
                    arrow(
                        &mut scene,
                        cell.center,
                        cell.center
                            + d.velocity()[i]
                                * (self.camera.radius * 0.06 * self.vector_scale.max(1e-6) / denom),
                        Color32::from_rgb(231, 239, 245),
                    );
                }
            }
            if cut_active && self.mesh_ghost_shell {
                for face in source
                    .faces()
                    .iter()
                    .filter(|f| f.neighbour.is_none() && !is_visible(f.owner))
                {
                    for j in 1..face.vertices.len().saturating_sub(1) {
                        for i in [face.vertices[0], face.vertices[j], face.vertices[j + 1]] {
                            let p = source.points()[i].position;
                            scene.trans_tri(
                                [p.x as f32, p.y as f32, p.z as f32],
                                [0.; 3],
                                [142, 166, 188, 32],
                            );
                        }
                    }
                }
            }
            if cut_active && self.mesh_cut_plane {
                for corners in mesh_sections::outlines(
                    (lo, hi),
                    self.mesh_cut_axis,
                    self.mesh_cut_fraction,
                    self.mesh_slab.then_some(self.mesh_slab_width),
                ) {
                    for j in 0..4 {
                        line(
                            &mut scene,
                            corners[j],
                            corners[(j + 1) % 4],
                            Color32::from_rgb(247, 187, 88),
                            1.5,
                            true,
                        );
                    }
                }
            }
            self.mesh_gpu.triangles = triangle_count;
            self.mesh_gpu.edges = edge_count;
            self.mesh_gpu.reduced_edges = reduced;
            if self.capture.is_some() {
                eprintln!("Mesh scene: {} cells, {} triangles, {} lines, {:.1} MiB vertex data; prepared in {:.3}s",source.cell_count(),triangle_count,self.mesh_gpu.edges,(scene.tris.len()+scene.trans.len()+scene.lines.len()+scene.overlay.len()) as f64/1048576.,prepared_at.elapsed().as_secs_f64());
            }
            self.mesh_gpu.gpu.upload(scene);
            self.mesh_gpu.key = Some(key);
        }
        if self.render.is_some() {
            let mut prefs = self.preferences.clone();
            prefs.geometry_edges = false;
            prefs.ambient_occlusion = false;
            self.mesh_gpu
                .gpu
                .paint_frame(painter, rect, &self.camera, &prefs);
        } else {
            let mut triangles = self
                .mesh_gpu
                .software
                .iter()
                .filter_map(|(p, c)| {
                    Some((
                        p.map(|p| self.camera.screen(p, rect))
                            .into_iter()
                            .collect::<Option<Vec<_>>>()?,
                        *c,
                    ))
                })
                .collect::<Vec<_>>();
            triangles.sort_by(|a, b| b.0[0].1.total_cmp(&a.0[0].1));
            for (p, c) in triangles {
                painter.add(egui::Shape::convex_polygon(
                    p.iter().map(|p| p.0).collect(),
                    c,
                    Stroke::NONE,
                ));
            }
        }
        if let Some((lo, hi)) = self.mesh_gpu.range {
            viewport::legend(painter, rect, lo, hi, palette);
        }
        let caption = if post {
            self.result_field.label()
        } else if quality_mode {
            match self.mesh_quality {
                1 => "Aspect ratio",
                2 => "Skewness",
                3 => "Cell measure",
                4 => "Adjacent volume ratio",
                _ => "Non-orthogonality (deg)",
            }
        } else if volume {
            if self.mesh_volume_edges == 0 {
                "Volume wireframe · all interior cell edges"
            } else {
                "Volume wireframe · distributed interior edge detail"
            }
        } else if caps_only {
            "Exposed interior cell faces"
        } else if cut_active {
            "Whole-cell cutaway · interior faces highlighted"
        } else if source.dimension() == MeshDimension::ThreeD {
            "Exterior surface · depth-tested mesh"
        } else {
            "Planar mesh"
        };
        painter.text(
            rect.left_top() + egui::vec2(12., 12.),
            egui::Align2::LEFT_TOP,
            caption,
            egui::FontId::proportional(12.),
            if self.preferences.light {
                Color32::from_rgb(44, 55, 67)
            } else {
                Color32::from_rgb(205, 218, 231)
            },
        );
        painter.text(
            rect.left_bottom() + egui::vec2(12., -34.),
            egui::Align2::LEFT_BOTTOM,
            format!(
                "{} / {} cells shown · {} triangles · {} edges{}",
                self.mesh_gpu.visible_cells,
                source.cell_count(),
                self.mesh_gpu.triangles,
                self.mesh_gpu.edges,
                if self.mesh_gpu.reduced_edges {
                    " · reduced edge detail"
                } else {
                    ""
                }
            ),
            egui::FontId::proportional(11.),
            Color32::from_rgb(145, 164, 181),
        );
    }
}

impl App {
    pub(super) fn paint_boundary_legend(
        &mut self,
        ui: &mut egui::Ui,
        painter: &egui::Painter,
        viewport: Rect,
    ) -> Option<Rect> {
        let patches = self.session.mesh()?.mesh.boundary_patches();
        if patches.is_empty() {
            return None;
        }
        let count = patches.len().min(10);
        let rect = Rect::from_min_size(
            viewport.left_top() + egui::vec2(10., 36.),
            egui::vec2(
                200f32.min((viewport.width() - 20.).max(100.)),
                28. + count as f32 * 25.,
            ),
        );
        let background = if self.preferences.light {
            Color32::from_rgba_unmultiplied(244, 247, 251, 235)
        } else {
            Color32::from_rgba_unmultiplied(28, 35, 46, 235)
        };
        painter.rect_filled(rect, 7., background);
        painter.text(
            rect.left_top() + egui::vec2(10., 8.),
            egui::Align2::LEFT_TOP,
            "BOUNDARY ZONES",
            egui::FontId::proportional(11.),
            widgets::ACCENT,
        );
        let mut selected = None;
        for (i, patch) in patches.iter().take(count).enumerate() {
            let row = Rect::from_min_size(
                rect.left_top() + egui::vec2(6., 25. + i as f32 * 25.),
                egui::vec2(rect.width() - 12., 24.),
            );
            let response = ui.interact(
                row,
                ui.id().with(("boundary-legend", i)),
                egui::Sense::click(),
            );
            if response.hovered() || self.boundary_patch == patch.name {
                painter.rect_filled(row, 4., Color32::from_rgba_unmultiplied(74, 109, 143, 80));
            }
            let c = patch_color(
                self.project
                    .boundaries
                    .iter()
                    .find(|b| b.selection == patch.name)
                    .map(|b| &b.condition),
                i,
            );
            painter.circle_filled(row.left_center() + egui::vec2(10., 0.), 4., c);
            painter.text(
                row.left_center() + egui::vec2(23., 0.),
                egui::Align2::LEFT_CENTER,
                format!("{} · {}", patch.name, patch.face_indices.len()),
                egui::FontId::proportional(12.),
                ui.visuals().text_color(),
            );
            if response.clicked() {
                selected = Some(patch.name.clone());
            }
        }
        if let Some(name) = selected {
            self.boundary_patch = name;
            if self.stage == Stage::Setup {
                self.page = 0;
                self.read_boundary();
            }
        }
        Some(rect)
    }
}
