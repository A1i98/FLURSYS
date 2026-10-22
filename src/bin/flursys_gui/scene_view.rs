//! Workbench scene adapter to the CAD viewport pipelines. Solver meshes and
//! topology IDs remain authoritative; display buffers never replace CFD data.
use super::render::{
    canonical_edge_path, image_from_rgba, normalize, patch_color, quality_color, scalar_color,
    scalar_range,
};
use super::viewport::{Camera3d, View3d};
use super::*;
use cad_render::{Camera, Rgb, Scene, SceneLine, SceneMesh};
use flursys::GeometryTopology;
use std::sync::Arc;

pub(super) struct SceneView {
    scene: Scene,
    mesh: cad_geom::Mesh,
    colors: Vec<(u32, Rgb)>,
    camera: Camera,
    origin: cad_geom::Vec3,
    unit: f64,
}
fn cp(p: Vec3) -> cad_geom::Vec3 {
    cad_geom::Vec3::new(p.x, p.y, p.z)
}
fn color(c: [u8; 4]) -> Rgb {
    Rgb(c[0], c[1], c[2])
}
impl SceneView {
    pub fn new(camera: Camera, radius: f64) -> Self {
        let origin = camera.target;
        let unit = radius.max(1e-12);
        let mut camera = camera;
        camera.target = cad_geom::Vec3::ZERO;
        camera.distance /= unit;
        Self {
            scene: Scene {
                radius: 1.0,
                background: Some((Rgb(46, 52, 64), Rgb(46, 52, 64))),
                ..Default::default()
            },
            mesh: Default::default(),
            colors: Vec::new(),
            camera,
            origin,
            unit,
        }
    }
    pub fn spatial(points: impl IntoIterator<Item = Vec3>, view: View3d) -> Self {
        let camera = Camera3d::fit(points, view);
        let (cam, radius) = camera.scene_camera();
        Self::new(cam, radius)
    }
    pub fn planar(view: &ViewTransform, radius: f64) -> Self {
        let target = view.screen_to_world((
            f64::from(PREVIEW_WIDTH) * 0.5,
            f64::from(PREVIEW_HEIGHT) * 0.5,
        ));
        Self::new(
            Camera {
                target: cad_geom::Vec3::new(target.0, target.1, 0.0),
                yaw: 0.0,
                pitch: std::f64::consts::FRAC_PI_2,
                distance: f64::from(PREVIEW_HEIGHT) / (0.8 * view.pixels_per_unit.max(1e-12)),
                ..Default::default()
            },
            radius,
        )
    }
    fn point(&self, p: Vec3) -> cad_geom::Vec3 {
        (cp(p) - self.origin) / self.unit
    }
    pub fn triangle(&mut self, p: [Vec3; 3], c: [u8; 4]) {
        let p = p.map(|p| self.point(p));
        if p.iter().any(|p| !p.is_finite()) {
            return;
        }
        let Some(index) = u32::try_from(self.mesh.positions.len())
            .ok()
            .filter(|n| *n < u32::MAX - 3)
        else {
            return;
        };
        let n = (p[1] - p[0])
            .cross(p[2] - p[0])
            .normalized()
            .unwrap_or(cad_geom::Vec3::Z);
        let face = self.mesh.triangles.len() as u32;
        self.mesh.positions.extend(p);
        self.mesh.normals.extend([n; 3]);
        self.mesh.triangles.push([index, index + 1, index + 2]);
        self.mesh.tri_face.push(face);
        self.colors.push((face, color(c)));
    }
    pub fn polygon(&mut self, outer: &[Vec3], holes: &[Vec<Vec3>], c: [u8; 4]) {
        if outer.len() == 3 && holes.is_empty() {
            self.triangle([outer[0], outer[1], outer[2]], c);
            return;
        }
        let Some(&origin) = outer.first() else { return };
        let Some(x) = outer.iter().map(|p| *p - origin).find(|p| p.norm() > 1e-14) else {
            return;
        };
        let x = x / x.norm();
        let Some(n) = outer
            .iter()
            .map(|p| x.cross(*p - origin))
            .find(|n| n.norm() > 1e-14)
        else {
            return;
        };
        let n = n / n.norm();
        let y = n.cross(x);
        let local = |p: Vec3| {
            let d = p - origin;
            cad_geom::Vec2::new(d.dot(x) / self.unit, d.dot(y) / self.unit)
        };
        let region = cad_geom::Region2 {
            outer: cad_geom::Loop2::polygon(&outer.iter().map(|p| local(*p)).collect::<Vec<_>>()),
            holes: holes
                .iter()
                .map(|h| cad_geom::Loop2::polygon(&h.iter().map(|p| local(*p)).collect::<Vec<_>>()))
                .collect(),
        };
        for t in region.triangulate(1e-8) {
            self.triangle(
                t.map(|p| origin + x * (p.x * self.unit) + y * (p.y * self.unit)),
                c,
            );
        }
    }
    pub fn line(&mut self, points: &[Vec3], c: [u8; 4], width: f64, on_top: bool) {
        if points.len() < 2 {
            return;
        }
        let points = points.iter().map(|p| self.point(*p)).collect::<Vec<_>>();
        if points.iter().any(|p| !p.is_finite()) {
            return;
        }
        self.scene.lines.push(SceneLine {
            points,
            color: color(c),
            width,
            on_top,
        });
    }
    pub fn marker(&mut self, p: Vec3, c: [u8; 4], pixels: f64) {
        let (r, u, _) = self.camera.basis();
        let k = self.camera.distance * 0.8 / f64::from(PREVIEW_HEIGHT) * self.unit * pixels;
        let r = Vec3::new(r.x, r.y, r.z) * k;
        let u = Vec3::new(u.x, u.y, u.z) * k;
        self.line(&[p - r, p + r], c, 2.0, true);
        self.line(&[p - u, p + u], c, 2.0, true);
    }
    pub fn arrow(&mut self, start: Vec3, end: Vec3, c: [u8; 4], on_top: bool) {
        self.line(&[start, end], c, 1.2, on_top);
        let d = end - start;
        let len = d.norm();
        if len < 1e-20 {
            return;
        }
        let (r, u, _) = self.camera.basis();
        let r = Vec3::new(r.x, r.y, r.z);
        let u = Vec3::new(u.x, u.y, u.z);
        let dx = d.dot(r);
        let dy = d.dot(u);
        let l = dx.hypot(dy);
        if l < 1e-20 {
            return;
        }
        let h = (len * 0.3)
            .min(self.camera.distance * 0.8 * self.unit * 5.0 / f64::from(PREVIEW_HEIGHT));
        let back = (r * dx + u * dy) / l * h;
        let side = (r * (-dy) + u * dx) / l * (h * 0.5);
        self.line(&[end - back + side, end, end - back - side], c, 1.2, on_top);
    }
    pub fn pixels(mut self, width: u32, height: u32, unlit: bool, transparent: bool) -> Vec<u8> {
        if !self.mesh.triangles.is_empty() {
            self.scene.meshes.push(SceneMesh {
                mesh: Arc::new(self.mesh),
                color: Rgb(165, 174, 190),
                face_colors: self.colors,
                opacity: 1.0,
            });
        }
        let output = match cad_gpu::capture::headless() {
            Ok(cad_gpu::capture::Headless::Gpu(c)) => if unlit {
                c.rgba_unlit(&self.scene, &self.camera, width, height)
            } else {
                c.rgba(&self.scene, &self.camera, width, height)
            }
            .map_err(|e| e.to_string()),
            Ok(cad_gpu::capture::Headless::Cpu) => Ok(if unlit {
                cad_render::render_unlit(&self.scene, &self.camera, width as usize, height as usize)
            } else {
                cad_render::render(&self.scene, &self.camera, width as usize, height as usize)
            }
            .rgba),
            Err(e) => Err(e),
        };
        let mut pixels = output.unwrap_or_else(|error| {
            // A failed GPU picture must not blank the geometry. The fallback is
            // the CAD project's own raster pipeline, with the same scene/camera.
            static WARNING: std::sync::Once = std::sync::Once::new();
            WARNING.call_once(|| eprintln!("Workbench viewport uses CPU CAD rendering: {error}"));
            if unlit {
                cad_render::render_unlit(&self.scene, &self.camera, width as usize, height as usize)
            } else {
                cad_render::render(&self.scene, &self.camera, width as usize, height as usize)
            }
            .rgba
        });
        if transparent {
            for p in pixels.chunks_exact_mut(4) {
                if p[..3] == [46, 52, 64] {
                    p[3] = 0;
                }
            }
        }
        pixels
    }
    pub fn image(self, unlit: bool, transparent: bool) -> slint::Image {
        let (width, height) = (PREVIEW_WIDTH * 2, PREVIEW_HEIGHT * 2);
        image_from_rgba(
            width,
            height,
            self.pixels(width, height, unlit, transparent),
        )
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn mesh(
    mesh: &flursys::UnstructuredMesh,
    cache: &flursys::MeshRenderCache,
    view: &ViewTransform,
    display: MeshDisplayMode,
    mode: MeshColorMode,
    metric: MeshQualityMetric,
    selected: Option<MeshSelection>,
    hovered: Option<MeshSelection>,
    patch: Option<usize>,
    spatial: View3d,
    clip: Option<mesh_inspection::MeshClip>,
) -> slint::Image {
    let mut scene = if mesh.dimension() == MeshDimension::ThreeD {
        SceneView::spatial(cache.positions().iter().copied(), spatial)
    } else {
        SceneView::planar(view, (cache.bounds().1 - cache.bounds().0).norm() * 0.5)
    };
    let mask = clip
        .filter(|_| mesh.dimension() == MeshDimension::ThreeD)
        .map(|c| c.mask(mesh, cache))
        .unwrap_or_else(|| vec![true; mesh.cell_count()]);
    let visible = |i| mesh_inspection::face_cell(mesh, i, &mask);
    let values = cache.quality().values(metric);
    let (lo, hi) = cache.quality().color_range(metric).unwrap_or((0.0, 0.0));
    let quality = |v: f64| {
        if metric == MeshQualityMetric::CellMeasure {
            quality_color(-v, -hi, -lo)
        } else {
            quality_color(v, lo, hi)
        }
    };
    let face_color = |i: usize| {
        let f = &mesh.faces()[i];
        let matches = |s: MeshSelection| match s.target() {
            MeshSelectionTarget::Face(j) => i == j,
            MeshSelectionTarget::Cell(j) => f.owner == j || f.neighbour == Some(j),
        };
        if selected.is_some_and(matches)
            || patch.is_some() && patch == cache.face_patch_indices()[i]
        {
            [255, 214, 95, 255]
        } else if hovered.is_some_and(matches) {
            [127, 234, 188, 255]
        } else {
            match mode {
                MeshColorMode::Neutral => [70, 109, 128, 255],
                MeshColorMode::Patches => patch_color(cache.face_patch_indices()[i]),
                MeshColorMode::Quality => quality(values[visible(i).unwrap_or(f.owner)]),
            }
        }
    };
    if display.draws_surface() {
        if mesh.dimension() == MeshDimension::TwoD {
            for i in 0..mesh.cell_count() {
                if let Some(p) = cache.cell_polygon(i) {
                    let p = p
                        .iter()
                        .map(|&(x, y)| Vec3::new(x, y, 0.0))
                        .collect::<Vec<_>>();
                    let c = if selected.is_some_and(|s| s.target() == MeshSelectionTarget::Cell(i))
                    {
                        [255, 214, 95, 255]
                    } else if hovered.is_some_and(|s| s.target() == MeshSelectionTarget::Cell(i)) {
                        [127, 234, 188, 255]
                    } else if mode == MeshColorMode::Quality {
                        quality(values[i])
                    } else {
                        [48, 79, 96, 255]
                    };
                    scene.polygon(&p, &[], c);
                }
            }
        } else {
            for (i, f) in mesh.faces().iter().enumerate() {
                if visible(i).is_some() {
                    let p = f
                        .vertices
                        .iter()
                        .map(|&i| cache.positions()[i])
                        .collect::<Vec<_>>();
                    scene.polygon(&p, &[], face_color(i));
                }
            }
        }
    }
    if display.draws_edges() {
        for (i, f) in mesh.faces().iter().enumerate() {
            if mesh.dimension() == MeshDimension::ThreeD && visible(i).is_none() {
                continue;
            }
            let tint = face_color(i);
            let emphasized = tint == [255, 214, 95, 255] || tint == [127, 234, 188, 255];
            let c = if emphasized {
                tint
            } else if !display.draws_surface() {
                if mode == MeshColorMode::Neutral {
                    [225, 235, 248, 255]
                } else {
                    tint
                }
            } else {
                [12, 18, 26, 255]
            };
            let mut p = f
                .vertices
                .iter()
                .map(|&j| cache.positions()[j])
                .collect::<Vec<_>>();
            if let Some(&first) = p.first() {
                p.push(first);
            }
            scene.line(
                &p,
                c,
                if emphasized { 2.2 } else { 1.0 },
                !display.draws_surface(),
            );
        }
    }
    scene.image(true, !display.draws_surface())
}

pub(super) fn geometry(
    topology: &GeometryTopology,
    view: View3d,
    pick_mode: i32,
    selected: &[GeometrySelectionTarget],
    hovered: Option<GeometrySelectionTarget>,
    display: i32,
) -> slint::Image {
    let surfaces = topology.renderable_faces();
    let paths = topology.cad_open_drawing_paths();
    let points = surfaces
        .iter()
        .flat_map(|s| s.vertices.iter().copied())
        .chain(paths.iter().flatten().copied())
        .collect::<Vec<_>>();
    let mut scene = SceneView::spatial(points, view);
    let sketches = topology
        .sketches()
        .filter_map(|s| s.profile_face)
        .collect::<BTreeSet<_>>();
    for surface in &surfaces {
        let is_sketch = surface.body.is_none() && sketches.contains(&surface.face);
        let chosen = selected.contains(&GeometrySelectionTarget::Face(surface.face))
            || surface
                .body
                .is_some_and(|b| selected.contains(&GeometrySelectionTarget::Body(b)));
        let hover = hovered == Some(GeometrySelectionTarget::Face(surface.face))
            || surface
                .body
                .is_some_and(|b| hovered == Some(GeometrySelectionTarget::Body(b)));
        let c = if chosen {
            [58, 136, 238, 255]
        } else if hover {
            [103, 150, 209, 255]
        } else {
            [165, 174, 190, 255]
        };
        if !is_sketch && display != 2 {
            scene.polygon(&surface.vertices, &surface.holes, c);
        }
        let kernel = surface.body.is_some_and(|b| topology.cad_body(b).is_some());
        if is_sketch || (!kernel && display != 1) {
            for p in std::iter::once(&surface.vertices).chain(&surface.holes) {
                let mut p = p.clone();
                if let Some(&first) = p.first() {
                    p.push(first);
                }
                scene.line(
                    &p,
                    if is_sketch {
                        [104, 222, 237, 255]
                    } else {
                        [67, 76, 92, 255]
                    },
                    1.3,
                    is_sketch || display == 2,
                );
            }
        }
    }
    for (_, body) in topology.cad_bodies().filter(|(_, b)| b.visible) {
        if display != 1 {
            for e in body.edges.iter() {
                scene.line(&e.path, [62, 105, 121, 255], 1.2, display == 2);
            }
        }
        if pick_mode == 2 {
            for v in body.vertices.iter() {
                if let Some(v) = topology.vertex(v.id) {
                    scene.marker(v.position, [104, 222, 237, 255], 2.0);
                }
            }
        }
    }
    for p in paths {
        scene.line(&p, [101, 191, 235, 255], 1.5, true);
    }
    for (target, c) in std::iter::once((hovered, [142, 232, 176, 255])).chain(
        selected
            .iter()
            .copied()
            .map(|t| (Some(t), [255, 226, 143, 255])),
    ) {
        match target {
            Some(GeometrySelectionTarget::Vertex(id)) => {
                if let Some(v) = topology.vertex(id) {
                    scene.marker(v.position, c, 4.0);
                }
            }
            Some(GeometrySelectionTarget::Edge(id)) => {
                if let Some(p) = topology
                    .edge(id)
                    .and_then(|e| canonical_edge_path(topology, &e.geometry))
                {
                    scene.line(&p, c, 2.7, true);
                }
            }
            _ => {}
        }
    }
    scene.image(false, false)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn results(
    cache: &ResultRenderCache,
    field: ResultFieldKind,
    colormap: i32,
    plot: i32,
    density: i32,
    scale: &str,
    range: Option<(f64, f64)>,
    view: &ViewTransform,
    streamlines: &[StreamlinePath],
    spatial: View3d,
    edges: bool,
    width: u32,
    height: u32,
) -> Vec<u8> {
    let mesh = cache.mesh_cache();
    let three = mesh.dimension() == MeshDimension::ThreeD;
    let mut scene = if three {
        SceneView::spatial(mesh.positions().iter().copied(), spatial)
    } else {
        SceneView::planar(view, (mesh.bounds().1 - mesh.bounds().0).norm() * 0.5)
    };
    let values = (0..cache.cell_count())
        .filter_map(|i| cache.scalar_for_cell(field, i))
        .collect::<Vec<_>>();
    let (lo, hi) = range.unwrap_or_else(|| scalar_range(&values, false));
    if three {
        for (&t, &f) in mesh.surface_triangles().iter().zip(mesh.triangle_faces()) {
            let Some(value) = cache.scalar_for_face(field, f) else {
                continue;
            };
            let p = t.map(|i| mesh.positions()[i]);
            scene.triangle(
                p,
                if plot == 3 {
                    [9, 16, 22, 255]
                } else {
                    scalar_color(normalize(value, lo, hi), colormap)
                },
            );
            if edges {
                scene.line(&[p[0], p[1], p[2], p[0]], [25, 35, 45, 255], 0.8, false);
            }
        }
    } else {
        for (i, &value) in values.iter().enumerate() {
            if let Some(poly) = mesh.cell_polygon(i) {
                let p = poly
                    .iter()
                    .map(|&(x, y)| Vec3::new(x, y, 0.0))
                    .collect::<Vec<_>>();
                if plot != 3 {
                    scene.polygon(&p, &[], scalar_color(normalize(value, lo, hi), colormap));
                }
                if edges {
                    let mut p = p;
                    if let Some(&a) = p.first() {
                        p.push(a);
                    }
                    scene.line(&p, [25, 35, 45, 255], 0.8, true);
                }
            }
        }
    }
    if matches!(plot, 2 | 3) {
        let max = (0..cache.cell_count())
            .filter_map(|i| cache.velocity_for_cell(i))
            .map(|v| v.norm())
            .fold(0.0, f64::max);
        if max > 1e-20 {
            let stride = super::render::vector_sample_stride(cache.cell_count(), density);
            let mult = super::render::vector_scale_multiplier(scale);
            if three {
                let step =
                    super::render::vector_sample_stride(mesh.surface_triangles().len(), density);
                for (&t, &f) in mesh
                    .surface_triangles()
                    .iter()
                    .zip(mesh.triangle_faces())
                    .step_by(step)
                {
                    if let Some(u) = cache
                        .owner_for_face(f)
                        .and_then(|o| cache.velocity_for_cell(o))
                    {
                        let center = (mesh.positions()[t[0]]
                            + mesh.positions()[t[1]]
                            + mesh.positions()[t[2]])
                            / 3.0;
                        let len = (mesh.bounds().1 - mesh.bounds().0).norm() * 0.08 / max * mult;
                        scene.arrow(center, center + u * len, [238, 246, 250, 255], false);
                    }
                }
            } else {
                for i in (0..cache.cell_count()).step_by(stride) {
                    if let (Some(p), Some(u)) = (mesh.cell_polygon(i), cache.velocity_for_cell(i)) {
                        let n = p.len().max(1) as f64;
                        let center = p
                            .iter()
                            .fold(Vec3::ZERO, |s, &(x, y)| s + Vec3::new(x, y, 0.0))
                            / n;
                        scene.arrow(
                            center,
                            center + u * (12.0 / view.pixels_per_unit / max * mult),
                            [238, 246, 250, 255],
                            true,
                        );
                    }
                }
            }
        }
    }
    for path in streamlines {
        scene.line(&path.points, [255, 245, 157, 255], 1.5, !three);
    }
    scene.pixels(width, height, true, false)
}
