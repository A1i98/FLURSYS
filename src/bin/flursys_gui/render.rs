#[path = "result_slice.rs"]
mod result_slice;
pub(super) use result_slice::{pick_result_slice_cell, render_result_slice};
#[path = "sketch_visual.rs"]
mod sketch_visual;
use super::viewport::{Camera3d, View3d};
use super::{MeshColorMode, PREVIEW_HEIGHT, PREVIEW_WIDTH, ResidualSample};
use flursys::{
    GeometrySelectionTarget, GeometrySketch, GeometryTopology, MeshDimension, MeshQualityMetric,
    MeshRenderCache, MeshSelection, ResultFieldKind, ResultRenderCache, SketchAxis,
    SketchEntityKind, StreamlinePath, UnstructuredMesh, Vec3, ViewTransform,
};
use slint::{Image, Rgba8Pixel, SharedPixelBuffer};
use std::collections::VecDeque;

#[cfg(test)]
use flursys::MeshSelectionTarget;

pub(super) fn render_empty_image() -> Image {
    let mut pixels = vec![0_u8; (PREVIEW_WIDTH * PREVIEW_HEIGHT * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    image_from_rgba(PREVIEW_WIDTH, PREVIEW_HEIGHT, pixels)
}

#[allow(clippy::too_many_arguments)] // Render inputs are independent UI state, not owned data.
pub(super) fn render_workbench_mesh(
    mesh: &UnstructuredMesh,
    cache: &MeshRenderCache,
    view: &ViewTransform,
    display_mode: super::MeshDisplayMode,
    color_mode: MeshColorMode,
    metric: MeshQualityMetric,
    selected: Option<MeshSelection>,
    hovered: Option<MeshSelection>,
    highlighted_patch: Option<usize>,
    view_3d: View3d,
    clip: Option<super::mesh_inspection::MeshClip>,
) -> Image {
    super::scene_view::mesh(
        mesh,
        cache,
        view,
        display_mode,
        color_mode,
        metric,
        selected,
        hovered,
        highlighted_patch,
        view_3d,
        clip,
    )
}

/// Renders immutable archived or live unstructured result data. Surface values
/// always come from the result dataset's mesh/cache, never from the editable
/// workbench mesh currently in memory.
#[allow(clippy::too_many_arguments)]
pub(super) fn render_persisted_result(
    cache: &ResultRenderCache,
    field: ResultFieldKind,
    colormap: i32,
    plot: i32,
    vector_density: i32,
    vector_scale: &str,
    display_range: Option<(f64, f64)>,
    view: &ViewTransform,
    streamlines: &[StreamlinePath],
    levels: usize,
    edges: bool,
    view_3d: View3d,
) -> Image {
    let (width, height) = (PREVIEW_WIDTH, PREVIEW_HEIGHT);
    let mut pixels = super::scene_view::results(
        cache,
        field,
        colormap,
        plot,
        vector_density,
        vector_scale,
        display_range,
        view,
        streamlines,
        view_3d,
        edges,
        width,
        height,
        levels,
    );
    if cache.mesh_cache().dimension() == MeshDimension::TwoD && matches!(plot, 1 | 2) {
        draw_result_contours(
            &mut pixels,
            width,
            height,
            cache,
            field,
            view,
            display_range,
            levels,
        );
    }
    draw_colormap_legend(&mut pixels, width, height, colormap);
    image_from_rgba(width, height, pixels)
}

#[cfg(test)]
#[allow(clippy::too_many_arguments)]
fn render_persisted_pixels(
    width: u32,
    height: u32,
    cache: &ResultRenderCache,
    field: ResultFieldKind,
    colormap: i32,
    plot: i32,
    vector_density: i32,
    vector_scale: &str,
    display_range: Option<(f64, f64)>,
    view: &ViewTransform,
    streamlines: &[StreamlinePath],
    view_3d: View3d,
) -> Vec<u8> {
    super::scene_view::results(
        cache,
        field,
        colormap,
        plot,
        vector_density,
        vector_scale,
        display_range,
        view,
        streamlines,
        view_3d,
        false,
        width,
        height,
        12,
    )
}

pub(super) fn vector_sample_stride(cell_count: usize, density: i32) -> usize {
    let target = match density.clamp(0, 2) {
        0 => 48,
        1 => 96,
        _ => 192,
    };
    cell_count.div_ceil(target).max(1)
}

pub(super) fn vector_scale_multiplier(text: &str) -> f64 {
    match text.trim() {
        "" => 1.0,
        value => value
            .parse::<f64>()
            .ok()
            .filter(|value| value.is_finite() && *value > 0.0)
            .map(|value| value.clamp(0.05, 5.0))
            .unwrap_or(1.0),
    }
}

/// Fit any model size and origin into the result viewport.
pub(super) fn result_screen_ray(
    cache: &MeshRenderCache,
    point: (f64, f64),
    view_3d: View3d,
) -> (Vec3, Vec3) {
    let camera = Camera3d::fit(cache.positions().iter().copied(), view_3d);
    let ray = camera.ray(point).or_else(|| camera.ray((260.0, 160.0)));
    match ray {
        Some(ray) => (ray.origin, ray.direction),
        None => (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
    }
}

#[cfg(test)]
fn draw_filled_polygon(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    points: &[(f64, f64)],
    color: [u8; 4],
) {
    if points.len() < 3 {
        return;
    }
    let first_y = points
        .iter()
        .map(|point| point.1.floor() as i32)
        .min()
        .unwrap_or(0)
        .max(0);
    let last_y = points
        .iter()
        .map(|point| point.1.ceil() as i32)
        .max()
        .unwrap_or(-1)
        .min(height as i32 - 1);
    for y in first_y..=last_y {
        let scan_y = f64::from(y) + 0.5;
        let mut crossings = Vec::new();
        for (a, b) in points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .take(points.len())
        {
            if (a.1 > scan_y) != (b.1 > scan_y) {
                crossings.push(a.0 + (b.0 - a.0) * (scan_y - a.1) / (b.1 - a.1));
            }
        }
        crossings.sort_by(f64::total_cmp);
        for pair in crossings.chunks_exact(2) {
            let first_x = pair[0].floor().max(0.0) as i32;
            let last_x = pair[1].ceil().min(f64::from(width) - 1.0) as i32;
            for x in first_x..=last_x {
                set_pixel(pixels, width, height, x as u32, y as u32, color);
            }
        }
    }
}

pub(super) fn patch_color(index: Option<usize>) -> [u8; 4] {
    const COLORS: [[u8; 4]; 6] = [
        [72, 166, 190, 255],
        [235, 177, 86, 255],
        [114, 185, 132, 255],
        [193, 118, 177, 255],
        [108, 142, 220, 255],
        [180, 130, 205, 255],
    ];
    index.map_or([70, 109, 128, 255], |value| COLORS[value % COLORS.len()])
}
pub(super) fn quality_color(value: f64, minimum: f64, maximum: f64) -> [u8; 4] {
    let t = if maximum > minimum {
        ((value - minimum) / (maximum - minimum)).clamp(0.0, 1.0)
    } else {
        0.5
    };
    [
        (40.0 + 215.0 * t) as u8,
        (180.0 - 145.0 * t) as u8,
        (125.0 - 90.0 * t) as u8,
        255,
    ]
}

/// Renders stable CAD faces directly from `GeometryTopology`, without using a
/// an intermediate preview mesh. Selection uses the same `FaceId` as CAD picking.
pub(super) fn render_canonical_geometry_3d(
    topology: &GeometryTopology,
    view: View3d,
    pick_mode: i32,
    selected_targets: &[GeometrySelectionTarget],
    hovered_target: Option<GeometrySelectionTarget>,
    display_mode: i32,
) -> Image {
    super::scene_view::geometry(
        topology,
        view,
        pick_mode,
        selected_targets,
        hovered_target,
        display_mode,
    )
}

/// Builds the orthographic canonical CAD ray for a viewport point using the
/// exact camera transform shared by `render_canonical_geometry_3d`.
pub(super) fn canonical_geometry_ray(
    topology: &GeometryTopology,
    view: View3d,
    point: (f64, f64),
) -> Option<flursys::Ray3> {
    canonical_camera(topology, view)?.ray(point)
}

fn canonical_camera(topology: &GeometryTopology, view: View3d) -> Option<CanonicalCamera> {
    let surfaces = topology.renderable_faces();
    let paths = topology.cad_open_drawing_paths();
    let points = surfaces
        .iter()
        .flat_map(|s| s.vertices.iter().copied())
        .chain(paths.iter().flatten().copied())
        .collect::<Vec<_>>();
    (!points.is_empty()).then(|| CanonicalCamera::fit(points, view))
}

/// Frame the selected geometry without changing document visibility or history.
pub(super) fn frame_selection(
    topology: &GeometryTopology,
    targets: &[GeometrySelectionTarget],
    mut view: View3d,
) -> Option<View3d> {
    let surfaces = topology.renderable_faces();
    let mut points = Vec::new();
    for surface in &surfaces {
        if targets.contains(&GeometrySelectionTarget::Face(surface.face))
            || surface
                .body
                .is_some_and(|id| targets.contains(&GeometrySelectionTarget::Body(id)))
        {
            points.extend(surface.vertices.iter().copied());
        }
    }
    for target in targets {
        match *target {
            GeometrySelectionTarget::Vertex(id) => {
                if let Some(vertex) = topology.vertex(id) {
                    points.push(vertex.position);
                }
            }
            GeometrySelectionTarget::Edge(id) => {
                if let Some(path) = topology
                    .edge(id)
                    .and_then(|edge| canonical_edge_path(topology, &edge.geometry))
                {
                    points.extend(path);
                }
            }
            _ => {}
        }
    }
    if points.is_empty() {
        return None;
    }
    let focus = Camera3d::fit(
        points,
        View3d {
            zoom: 1.0,
            pan: (0.0, 0.0),
            ..view
        },
    );
    let scene = Camera3d::fit(
        surfaces
            .iter()
            .flat_map(|face| face.vertices.iter().copied()),
        View3d {
            zoom: 1.0,
            pan: (0.0, 0.0),
            ..view
        },
    );
    view.zoom = (focus.scale / scene.scale).clamp(0.05, 50.0) as f32;
    let scene = Camera3d::fit(
        surfaces
            .iter()
            .flat_map(|face| face.vertices.iter().copied()),
        View3d {
            pan: (0.0, 0.0),
            ..view
        },
    );
    let center = scene.screen(focus.centre);
    view.pan = (
        f64::from(PREVIEW_WIDTH) * 0.5 - center.0,
        f64::from(PREVIEW_HEIGHT) * 0.5 - center.1,
    );
    Some(view)
}

/// Face, sketch-edge, or sketch-vertex selection in the same orthographic camera
/// used for rendering. Candidates behind a nearer face are not selectable.
pub(super) fn measurement_surface_point(
    topology: &GeometryTopology,
    view: View3d,
    point: (f64, f64),
) -> Option<Vec3> {
    topology
        .pick_face(canonical_camera(topology, view)?.ray(point)?)
        .map(|hit| hit.position)
}

pub(super) fn project_measurement(
    topology: &GeometryTopology,
    view: View3d,
    points: &[Vec3],
) -> Vec<(i32, i32)> {
    canonical_camera(topology, view)
        .map(|camera| points.iter().map(|p| camera.project(*p)).collect())
        .unwrap_or_default()
}

pub(super) fn annotate_measurement(image: Image, points: &[(i32, i32)]) -> Image {
    let Some(buffer) = image.to_rgba8() else {
        return image;
    };
    let (width, height) = (buffer.width(), buffer.height());
    let scale = f64::from(width) / f64::from(PREVIEW_WIDTH);
    let points = points
        .iter()
        .map(|&(x, y)| {
            (
                (f64::from(x) * scale).round() as i32,
                (f64::from(y) * scale).round() as i32,
            )
        })
        .collect::<Vec<_>>();
    let mut pixels = buffer.as_bytes().to_vec();
    for pair in points.windows(2) {
        sketch_visual::line(
            &mut pixels,
            width,
            height,
            pair[0],
            pair[1],
            [255, 210, 112, 255],
        );
    }
    for p in &points {
        draw_marker(&mut pixels, width, height, *p, [255, 210, 112, 255]);
        draw_ellipse(&mut pixels, width, height, *p, 6, 6, [255, 210, 112, 255]);
    }
    image_from_rgba(width, height, pixels)
}

/// Pick only exterior edges of the selected 2D fluid domain. Coincident
/// edges belonging to other sketch regions must never win the hit test.
pub(super) fn pick_domain_edge(
    topology: &GeometryTopology,
    view: View3d,
    point: (f64, f64),
    targets: &[GeometrySelectionTarget],
) -> Option<GeometrySelectionTarget> {
    let camera = canonical_camera(topology, view)?;
    targets
        .iter()
        .filter_map(|target| {
            let GeometrySelectionTarget::Edge(id) = target else {
                return None;
            };
            let path = canonical_edge_path(topology, &topology.edge(*id)?.geometry)?;
            let distance = path
                .windows(2)
                .map(|pair| {
                    let a = camera.screen(pair[0]);
                    let b = camera.screen(pair[1]);
                    let d = (b.0 - a.0, b.1 - a.1);
                    let length = d.0 * d.0 + d.1 * d.1;
                    let t = if length > 0.0 {
                        ((point.0 - a.0) * d.0 + (point.1 - a.1) * d.1) / length
                    } else {
                        0.0
                    }
                    .clamp(0.0, 1.0);
                    (point.0 - a.0 - t * d.0).hypot(point.1 - a.1 - t * d.1)
                })
                .min_by(f64::total_cmp)?;
            (distance <= 8.0).then_some((*target, distance))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|hit| hit.0)
}

/// Window selection: the complete sampled edge must lie inside the rectangle.
pub(super) fn domain_edges_in_box(
    topology: &GeometryTopology,
    view: View3d,
    a: (f64, f64),
    b: (f64, f64),
    targets: &[GeometrySelectionTarget],
) -> Vec<GeometrySelectionTarget> {
    let Some(camera) = canonical_camera(topology, view) else {
        return Vec::new();
    };
    let inside = |p: Vec3| {
        let p = camera.screen(p);
        p.0 >= a.0.min(b.0) && p.0 <= a.0.max(b.0) && p.1 >= a.1.min(b.1) && p.1 <= a.1.max(b.1)
    };
    let surfaces = if targets
        .iter()
        .any(|t| matches!(t, GeometrySelectionTarget::Face(_)))
    {
        topology.renderable_faces()
    } else {
        Vec::new()
    };
    targets
        .iter()
        .copied()
        .filter(|target| match target {
            GeometrySelectionTarget::Edge(id) => topology
                .edge(*id)
                .and_then(|edge| canonical_edge_path(topology, &edge.geometry))
                .is_some_and(|path| !path.is_empty() && path.into_iter().all(&inside)),
            GeometrySelectionTarget::Face(id) => {
                let pieces = surfaces
                    .iter()
                    .filter(|s| s.face == *id)
                    .collect::<Vec<_>>();
                !pieces.is_empty()
                    && pieces
                        .iter()
                        .all(|s| !s.vertices.is_empty() && s.vertices.iter().copied().all(&inside))
            }
            _ => false,
        })
        .collect()
}

pub(super) fn pick_canonical_geometry_target(
    topology: &GeometryTopology,
    view: View3d,
    point: (f64, f64),
    mode: i32,
) -> Option<GeometrySelectionTarget> {
    let camera = canonical_camera(topology, view)?;
    let ray = camera.ray(point)?;
    if mode == 3 {
        let face = topology.pick_face(ray)?.face;
        return topology
            .bodies()
            .find(|body| body.faces.contains(&face))
            .map(|body| GeometrySelectionTarget::Body(body.id));
    }
    if mode == 0 {
        return topology
            .pick_face(ray)
            .map(|hit| GeometrySelectionTarget::Face(hit.face));
    }
    let front = topology.pick_face(ray).map(|hit| hit.distance);
    let visible = |position: Vec3| {
        front.is_none_or(|distance| {
            (position - ray.origin).dot(ray.direction) <= distance + 8.0 / camera.scale
        })
    };
    if mode == 2 {
        return topology
            .vertices()
            .filter(|v| topology.is_vertex_visible(v.id))
            .filter_map(|vertex| {
                let position = camera.project(vertex.position);
                let distance =
                    (point.0 - f64::from(position.0)).hypot(point.1 - f64::from(position.1));
                (distance <= 9.0 && visible(vertex.position)).then_some((
                    distance,
                    (vertex.position - ray.origin).dot(ray.direction),
                    GeometrySelectionTarget::Vertex(vertex.id),
                ))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)))
            .map(|(_, _, target)| target);
    }
    if mode != 1 {
        return None;
    }
    topology
        .edges()
        .filter(|e| topology.is_edge_visible(e.id))
        .filter_map(|edge| {
            let path = canonical_edge_path(topology, &edge.geometry)?;
            path.windows(2)
                .filter_map(|segment| {
                    let (a, b) = (camera.project(segment[0]), camera.project(segment[1]));
                    let dx = f64::from(b.0) - f64::from(a.0);
                    let dy = f64::from(b.1) - f64::from(a.1);
                    let t = if dx == 0.0 && dy == 0.0 {
                        0.0
                    } else {
                        ((point.0 - f64::from(a.0)) * dx + (point.1 - f64::from(a.1)) * dy)
                            / (dx * dx + dy * dy)
                    }
                    .clamp(0.0, 1.0);
                    let closest = segment[0] + (segment[1] - segment[0]) * t;
                    let distance = distance_to_segment(
                        point,
                        (f64::from(a.0), f64::from(a.1)),
                        (f64::from(b.0), f64::from(b.1)),
                    );
                    (distance <= 8.0 && visible(closest))
                        .then_some((distance, (closest - ray.origin).dot(ray.direction)))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)))
                .map(|(distance, depth)| (distance, depth, GeometrySelectionTarget::Edge(edge.id)))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)))
        .map(|(_, _, target)| target)
}

pub(super) fn canonical_edge_path(
    topology: &GeometryTopology,
    edge: &flursys::EdgeGeometry,
) -> Option<Vec<Vec3>> {
    match *edge {
        flursys::EdgeGeometry::Kernel { id, .. } => Some(topology.cad_edge(id)?.path.clone()),
        flursys::EdgeGeometry::Line { start, end } => Some(vec![
            topology.vertex(start)?.position,
            topology.vertex(end)?.position,
        ]),
        flursys::EdgeGeometry::CircularArc { start, center, end } => {
            let (start, center, end) = (
                topology.vertex(start)?.position,
                topology.vertex(center)?.position,
                topology.vertex(end)?.position,
            );
            let delta = start - center;
            let radius = delta.norm();
            let first = delta.y.atan2(delta.x);
            let sweep = ((end.y - center.y).atan2(end.x - center.x) - first)
                .rem_euclid(std::f64::consts::TAU);
            Some(
                (0..=32)
                    .map(|i| {
                        let angle = first + sweep * f64::from(i) / 32.0;
                        Vec3::new(
                            center.x + radius * angle.cos(),
                            center.y + radius * angle.sin(),
                            start.z,
                        )
                    })
                    .collect(),
            )
        }
    }
}

type CanonicalCamera = Camera3d;

pub(super) fn render_empty_preview() -> Image {
    let mut pixels = vec![0_u8; (PREVIEW_WIDTH * PREVIEW_HEIGHT * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    image_from_rgba(PREVIEW_WIDTH, PREVIEW_HEIGHT, pixels)
}

pub(super) fn render_sketch_2d(
    sketch: &GeometrySketch,
    pending_segment: Option<((f64, f64), (f64, f64))>,
) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    let center = (width as i32 / 2, height as i32 / 2);
    let scale = 140.0_f64;
    for index in -8..=8 {
        let x = center.0 + (f64::from(index) * scale / 2.0) as i32;
        let y = center.1 + (f64::from(index) * scale / 2.0) as i32;
        draw_line(
            &mut pixels,
            width,
            height,
            (x, 0),
            (x, height as i32),
            [25, 44, 56, 255],
        );
        draw_line(
            &mut pixels,
            width,
            height,
            (0, y),
            (width as i32, y),
            [25, 44, 56, 255],
        );
    }
    let horizontal = if sketch.selected_axis == SketchAxis::Horizontal {
        [240, 195, 109, 255]
    } else {
        [81, 143, 176, 255]
    };
    let vertical = if sketch.selected_axis == SketchAxis::Vertical {
        [240, 195, 109, 255]
    } else {
        [81, 143, 176, 255]
    };
    draw_line(
        &mut pixels,
        width,
        height,
        (0, center.1),
        (width as i32, center.1),
        horizontal,
    );
    draw_line(
        &mut pixels,
        width,
        height,
        (center.0, 0),
        (center.0, height as i32),
        vertical,
    );
    if let Some((start, end)) = pending_segment {
        let start = sketch_point(center, scale, start.0, start.1);
        let end = sketch_point(center, scale, end.0, end.1);
        let preview = [255, 181, 88, 255];
        draw_line(&mut pixels, width, height, start, end, preview);
        draw_marker(&mut pixels, width, height, start, preview);
        draw_marker(&mut pixels, width, height, end, preview);
    }
    for entity in &sketch.entities {
        let color = if sketch.selected_entity == Some(entity.id) {
            [240, 195, 109, 255]
        } else {
            [104, 222, 237, 255]
        };
        match entity.kind {
            SketchEntityKind::Line { x1, y1, x2, y2 } => {
                let start = sketch_point(center, scale, x1, y1);
                let end = sketch_point(center, scale, x2, y2);
                draw_line(&mut pixels, width, height, start, end, color);
                draw_marker(&mut pixels, width, height, start, color);
                draw_marker(&mut pixels, width, height, end, color);
            }
            SketchEntityKind::Circle {
                center_x,
                center_y,
                radius,
            } => draw_ellipse(
                &mut pixels,
                width,
                height,
                sketch_point(center, scale, center_x, center_y),
                (radius * scale) as i32,
                (radius * scale) as i32,
                color,
            ),
        }
    }
    for dimension in &sketch.dimensions {
        let start = sketch_point(center, scale, dimension.x1, dimension.y1);
        let end = sketch_point(center, scale, dimension.x2, dimension.y2);
        draw_line(&mut pixels, width, height, start, end, [240, 195, 109, 255]);
        draw_marker(&mut pixels, width, height, start, [240, 195, 109, 255]);
        draw_marker(&mut pixels, width, height, end, [240, 195, 109, 255]);
    }
    image_from_rgba(width, height, pixels)
}

fn sketch_point(center: (i32, i32), scale: f64, x: f64, y: f64) -> (i32, i32) {
    (center.0 + (x * scale) as i32, center.1 - (y * scale) as i32)
}

fn draw_marker(pixels: &mut [u8], width: u32, height: u32, center: (i32, i32), color: [u8; 4]) {
    if center.0 < -4
        || center.1 < -4
        || center.0 as i64 > width as i64 + 4
        || center.1 as i64 > height as i64 + 4
    {
        return;
    }
    draw_line(
        pixels,
        width,
        height,
        (center.0 - 4, center.1),
        (center.0 + 4, center.1),
        color,
    );
    draw_line(
        pixels,
        width,
        height,
        (center.0, center.1 - 4),
        (center.0, center.1 + 4),
        color,
    );
}

pub(super) fn preview_image_point(x: f32, y: f32, width: f32, height: f32) -> Option<(f64, f64)> {
    if width <= 0.0 || height <= 0.0 {
        return None;
    }
    let scale = (width / PREVIEW_WIDTH as f32).min(height / PREVIEW_HEIGHT as f32);
    let offset_x = 0.5 * (width - PREVIEW_WIDTH as f32 * scale);
    let offset_y = 0.5 * (height - PREVIEW_HEIGHT as f32 * scale);
    let px = (x - offset_x) / scale;
    let py = (y - offset_y) / scale;
    ((0.0..=PREVIEW_WIDTH as f32).contains(&px) && (0.0..=PREVIEW_HEIGHT as f32).contains(&py))
        .then_some((f64::from(px), f64::from(py)))
}

pub(super) fn distance_to_segment(point: (f64, f64), start: (f64, f64), end: (f64, f64)) -> f64 {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let denominator = dx * dx + dy * dy;
    if denominator <= f64::EPSILON {
        return ((point.0 - start.0).powi(2) + (point.1 - start.1).powi(2)).sqrt();
    }
    let fraction =
        (((point.0 - start.0) * dx + (point.1 - start.1) * dy) / denominator).clamp(0.0, 1.0);
    let closest = (start.0 + fraction * dx, start.1 + fraction * dy);
    ((point.0 - closest.0).powi(2) + (point.1 - closest.1).powi(2)).sqrt()
}

// Six depth-ordered signed-axis handles share the viewport camera.
// Coordinates come from the same yaw/pitch basis as the shared CAD camera.
fn navigator_handles(yaw: f32, pitch: f32) -> Vec<(i32, (i32, i32), f64)> {
    let (yaw, pitch) = (f64::from(yaw), f64::from(pitch));
    let mut handles = Vec::new();
    for (axis, (x, y, z)) in [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)]
        .into_iter()
        .enumerate()
    {
        for sign in [-1., 1.] {
            let horizontal = sign * (x * yaw.cos() - y * yaw.sin());
            let vertical = sign * (z * pitch.cos() + (x * yaw.sin() + y * yaw.cos()) * pitch.sin());
            let depth = sign * ((x * yaw.sin() + y * yaw.cos()) * pitch.cos() - z * pitch.sin());
            handles.push((
                if sign > 0. {
                    axis as i32
                } else {
                    axis as i32 + 4
                },
                (
                    (48. + horizontal * 32.).round() as i32,
                    (48. - vertical * 32.).round() as i32,
                ),
                depth,
            ));
        }
    }
    handles.sort_by(|a, b| b.2.total_cmp(&a.2));
    handles
}
pub(super) fn render_orientation_overlay(yaw: f32, pitch: f32) -> Image {
    // Supersample the compass while keeping its hit targets in logical 96px coordinates.
    const S: i32 = 3;
    const W: u32 = 288;
    let mut pixels = vec![0u8; (W * W * 4) as usize];
    for y in 0..288i32 {
        for x in 0..288i32 {
            let r = ((x - 144) as f64).hypot((y - 144) as f64) / 3.;
            if r <= 46.5 {
                let c = if r > 45.8 {
                    [72, 80, 96, 190]
                } else {
                    [38, 44, 55, 125]
                };
                let i = (y as usize * 288 + x as usize) * 4;
                pixels[i..i + 4].copy_from_slice(&c);
            }
        }
    }
    for (axis, p, _) in navigator_handles(yaw, pitch) {
        let c = match axis {
            0 | 4 => [244, 91, 112, 255],
            1 | 5 => [111, 212, 122, 255],
            _ => [70, 117, 244, 255],
        };
        let p = (p.0 * S, p.1 * S);
        draw_line(&mut pixels, W, W, (144, 144), p, c);
        let radius = if axis < 3 { 9 * S } else { 8 * S };
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx * dx + dy * dy <= radius * radius {
                    let (x, y) = (p.0 + dx, p.1 + dy);
                    if (0..288).contains(&x) && (0..288).contains(&y) {
                        let i = (y as usize * 288 + x as usize) * 4;
                        let edge = dx * dx + dy * dy >= (radius - 3) * (radius - 3);
                        pixels[i..i + 4].copy_from_slice(&if axis < 3 || edge {
                            c
                        } else {
                            [39, 45, 57, 255]
                        });
                    }
                }
            }
        }
        let ink = if axis < 3 { [250, 252, 255, 255] } else { c };
        let (x, y) = (p.0 - 3 * S, p.1 - 4 * S);
        let strokes: &[((i32, i32), (i32, i32))] = match axis % 4 {
            0 => &[((0, 0), (6, 8)), ((6, 0), (0, 8))],
            1 => &[((0, 0), (3, 4)), ((6, 0), (3, 4)), ((3, 4), (3, 8))],
            _ => &[((0, 0), (6, 0)), ((6, 0), (0, 8)), ((0, 8), (6, 8))],
        };
        for (a, b) in strokes {
            for offset in -1..=1 {
                draw_line(
                    &mut pixels,
                    W,
                    W,
                    (x + a.0 * S + offset, y + a.1 * S),
                    (x + b.0 * S + offset, y + b.1 * S),
                    ink,
                );
            }
        }
    }
    image_from_rgba(W, W, pixels)
}
pub(super) fn pick_orientation_overlay(yaw: f32, pitch: f32, x: f32, y: f32) -> Option<i32> {
    if !x.is_finite() || !y.is_finite() {
        return None;
    }
    navigator_handles(yaw, pitch)
        .into_iter()
        .rev()
        .find(|(_, p, _)| {
            (f64::from(x) - f64::from(p.0)).hypot(f64::from(y) - f64::from(p.1)) <= 10.
        })
        .map(|h| h.0)
}

pub(super) fn draw_ellipse(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    center: (i32, i32),
    radius_x: i32,
    radius_y: i32,
    color: [u8; 4],
) {
    let mut previous = None;
    for step in 0..=48 {
        let angle = step as f32 * std::f32::consts::TAU / 48.0;
        let point = (
            center.0 + (radius_x as f32 * angle.cos()) as i32,
            center.1 + (radius_y as f32 * angle.sin()) as i32,
        );
        if let Some(last) = previous {
            draw_line(pixels, width, height, last, point, color);
        }
        previous = Some(point);
    }
}

pub(super) fn scalar_range(values: &[f64], symmetric: bool) -> (f64, f64) {
    let (mut min_value, mut max_value) = values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), value| {
            (min.min(value), max.max(value))
        });
    if symmetric {
        let bound = min_value.abs().max(max_value.abs()).max(1.0e-12);
        min_value = -bound;
        max_value = bound;
    } else if !min_value.is_finite() || !max_value.is_finite() {
        min_value = 0.0;
        max_value = 1.0;
    } else if max_value <= min_value {
        let pad = (min_value.abs() * 0.01).max(1e-12);
        min_value -= pad;
        max_value += pad;
    }
    (min_value, max_value)
}

pub(super) fn normalize(value: f64, min_value: f64, max_value: f64) -> f32 {
    ((value - min_value) / (max_value - min_value)).clamp(0.0, 1.0) as f32
}

fn draw_colormap_legend(pixels: &mut [u8], width: u32, height: u32, colormap: i32) {
    let left = width.saturating_sub(18);
    for y in 12..height.saturating_sub(12) {
        let t = 1.0 - (y - 12) as f32 / (height.saturating_sub(24)) as f32;
        let color = scalar_color(t, colormap);
        for x in left..width.saturating_sub(8) {
            set_pixel(pixels, width, height, x, y, color);
        }
    }
    for x in left.saturating_sub(1)..width.saturating_sub(7) {
        set_pixel(pixels, width, height, x, 11, [220, 230, 236, 255]);
        set_pixel(
            pixels,
            width,
            height,
            x,
            height.saturating_sub(12),
            [220, 230, 236, 255],
        );
    }
}

/// Workbench jobs report continuity during the solve; do not substitute
/// histories belonging to the separate structured solver.
pub(super) fn render_workbench_continuity_chart(history: &VecDeque<ResidualSample>) -> Image {
    let (width, height) = (PREVIEW_WIDTH, PREVIEW_HEIGHT);
    let mut pixels = vec![0u8; (width * height * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    let positive = history
        .iter()
        .map(|s| s.continuity)
        .filter(|v| v.is_finite() && *v > 0.0)
        .collect::<Vec<_>>();
    if positive.is_empty() {
        if history.iter().any(|s| s.continuity == 0.0) {
            for n in 0..=4 {
                let y = 12 + (height - 25) * n / 4;
                draw_line(
                    &mut pixels,
                    width,
                    height,
                    (12, y as i32),
                    (width as i32 - 13, y as i32),
                    [29, 48, 61, 255],
                );
            }
            draw_line(
                &mut pixels,
                width,
                height,
                (12, height as i32 - 13),
                (width as i32 - 13, height as i32 - 13),
                [77, 168, 184, 255],
            );
        }
        return image_from_rgba(width, height, pixels);
    }
    let lo = positive
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min)
        .log10()
        .floor()
        - 0.25;
    let hi = positive.iter().copied().fold(0.0, f64::max).log10().ceil() + 0.25;
    let span = (hi - lo).max(1.0);
    for n in 0..=4 {
        let y = 12 + (height - 25) * n / 4;
        draw_line(
            &mut pixels,
            width,
            height,
            (12, y as i32),
            (width as i32 - 13, y as i32),
            [29, 48, 61, 255],
        );
    }
    let mut previous = None;
    for (index, sample) in history.iter().enumerate() {
        if !sample.continuity.is_finite() || sample.continuity < 0.0 {
            previous = None;
            continue;
        }
        let log = if sample.continuity > 0.0 {
            sample.continuity.log10()
        } else {
            lo
        };
        let x = 12 + (width - 25) as usize * index / history.len().saturating_sub(1).max(1);
        let y = 12.0 + (hi - log).clamp(0.0, span) / span * f64::from(height - 25);
        let point = (x as i32, y.round() as i32);
        if let Some(old) = previous {
            draw_line(&mut pixels, width, height, old, point, [77, 168, 184, 255]);
        } else {
            draw_marker(&mut pixels, width, height, point, [77, 168, 184, 255]);
        }
        previous = Some(point);
    }
    image_from_rgba(width, height, pixels)
}

pub(super) fn draw_line(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    start: (i32, i32),
    end: (i32, i32),
    color: [u8; 4],
) {
    if width == 0 || height == 0 {
        return;
    }
    // Clip before integer rasterization: zoomed/off-screen edges must not
    // overflow Bresenham arithmetic or walk billions of invisible pixels.
    let (ax, ay) = (start.0 as f64, start.1 as f64);
    let (dx, dy) = (end.0 as f64 - ax, end.1 as f64 - ay);
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    for (p, q) in [
        (-dx, ax),
        (dx, width as f64 - 1.0 - ax),
        (-dy, ay),
        (dy, height as f64 - 1.0 - ay),
    ] {
        if p == 0.0 {
            if q < 0.0 {
                return;
            }
            continue;
        }
        let r = q / p;
        if p < 0.0 {
            lo = lo.max(r);
        } else {
            hi = hi.min(r);
        }
        if lo > hi {
            return;
        }
    }
    let start = ((ax + lo * dx).round() as i32, (ay + lo * dy).round() as i32);
    let end = ((ax + hi * dx).round() as i32, (ay + hi * dy).round() as i32);
    let (mut x0, mut y0) = start;
    let (x1, y1) = end;
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut error = dx + dy;
    loop {
        if x0 >= 0 && y0 >= 0 {
            set_pixel(pixels, width, height, x0 as u32, y0 as u32, color);
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let twice_error = 2 * error;
        if twice_error >= dy {
            error += dy;
            x0 += sx;
        }
        if twice_error <= dx {
            error += dx;
            y0 += sy;
        }
    }
}

pub(super) fn scalar_color(value: f32, colormap: i32) -> [u8; 4] {
    const TURBO: [[u8; 3]; 5] = [
        [48, 18, 59],
        [46, 91, 195],
        [34, 196, 173],
        [239, 220, 70],
        [199, 44, 46],
    ];
    const VIRIDIS: [[u8; 3]; 5] = [
        [68, 1, 84],
        [59, 82, 139],
        [33, 145, 140],
        [94, 201, 98],
        [253, 231, 37],
    ];
    const PLASMA: [[u8; 3]; 5] = [
        [13, 8, 135],
        [126, 3, 168],
        [204, 71, 120],
        [248, 149, 64],
        [240, 249, 33],
    ];
    const BLUE_RED: [[u8; 3]; 5] = [
        [49, 54, 149],
        [116, 173, 209],
        [247, 247, 247],
        [244, 109, 67],
        [165, 0, 38],
    ];
    let stops = match colormap {
        1 => &VIRIDIS,
        2 => &PLASMA,
        3 => &BLUE_RED,
        _ => &TURBO,
    };
    let scaled = value.clamp(0.0, 1.0) * (stops.len() - 1) as f32;
    let index = scaled.floor() as usize;
    let blend = scaled - index as f32;
    let lower = stops[index];
    let upper = stops[(index + 1).min(stops.len() - 1)];
    [
        (lower[0] as f32 + blend * (upper[0] as f32 - lower[0] as f32)) as u8,
        (lower[1] as f32 + blend * (upper[1] as f32 - lower[1] as f32)) as u8,
        (lower[2] as f32 + blend * (upper[2] as f32 - lower[2] as f32)) as u8,
        255,
    ]
}

pub(super) fn fill(pixels: &mut [u8], color: [u8; 4]) {
    for pixel in pixels.chunks_exact_mut(4) {
        pixel.copy_from_slice(&color);
    }
}

pub(super) fn set_pixel(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    x: u32,
    y: u32,
    color: [u8; 4],
) {
    if x >= width || y >= height {
        return;
    }
    let index = ((y * width + x) * 4) as usize;
    pixels[index..index + 4].copy_from_slice(&color);
}

pub(super) fn image_from_rgba(width: u32, height: u32, pixels: Vec<u8>) -> Image {
    let mut buffer = SharedPixelBuffer::<Rgba8Pixel>::new(width, height);
    buffer.make_mut_bytes().copy_from_slice(&pixels);
    Image::from_rgba8(buffer)
}

/// Display-only contour reconstruction; stored cell data is never modified.
fn draw_result_contours(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    cache: &ResultRenderCache,
    field: ResultFieldKind,
    view: &ViewTransform,
    range: Option<(f64, f64)>,
    levels: usize,
) {
    let values = (0..cache.cell_count())
        .map(|c| cache.scalar_for_cell(field, c).unwrap_or(0.0))
        .collect::<Vec<_>>();
    let (lo, hi) = range.unwrap_or_else(|| scalar_range(&values, false));
    if hi.partial_cmp(&lo) != Some(std::cmp::Ordering::Greater)
        || values.iter().all(|v| *v == values[0])
    {
        return;
    }
    let mut nodes = std::collections::HashMap::<(u64, u64), (f64, usize)>::new();
    for (i, v) in values.iter().enumerate() {
        if let Some(poly) = cache.mesh_cache().cell_polygon(i) {
            for &(x, y) in poly {
                let a = nodes.entry((x.to_bits(), y.to_bits())).or_default();
                a.0 += v;
                a.1 += 1;
            }
        }
    }
    for (i, &v) in values.iter().enumerate() {
        let Some(poly) = cache.mesh_cache().cell_polygon(i) else {
            continue;
        };
        if poly.len() < 3 {
            continue;
        }
        let center = (
            poly.iter().map(|p| p.0).sum::<f64>() / poly.len() as f64,
            poly.iter().map(|p| p.1).sum::<f64>() / poly.len() as f64,
        );
        for j in 0..poly.len() {
            let (a, b) = (poly[j], poly[(j + 1) % poly.len()]);
            let value = |p: (f64, f64)| {
                let n = nodes[&(p.0.to_bits(), p.1.to_bits())];
                n.0 / n.1 as f64
            };
            let triangle = [(center, v), (a, value(a)), (b, value(b))];
            for level in 1..levels.clamp(3, 32) {
                let iso = lo + (hi - lo) * level as f64 / levels as f64;
                let mut intersections = Vec::with_capacity(2);
                for k in 0..3 {
                    let ((a, va), (b, vb)) = (triangle[k], triangle[(k + 1) % 3]);
                    if (va < iso && vb >= iso) || (vb < iso && va >= iso) {
                        let t = (iso - va) / (vb - va);
                        intersections.push(
                            view.world_to_screen((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)),
                        );
                    }
                }
                if intersections.len() == 2 {
                    let (a, b) = (intersections[0], intersections[1]);
                    draw_line(
                        pixels,
                        width,
                        height,
                        (a.0 as i32, a.1 as i32),
                        (b.0 as i32, b.1 as i32),
                        [26, 39, 48, 255],
                    );
                }
            }
        }
    }
}

pub(super) fn overlay_markers(image: Image, points: &[(f64, f64)], view: &ViewTransform) -> Image {
    let Some(buffer) = image.to_rgba8() else {
        return image;
    };
    let (w, h) = (buffer.width(), buffer.height());
    let mut pixels = buffer.as_bytes().to_vec();
    for &point in points {
        let p = view.world_to_screen(point);
        draw_ellipse(
            &mut pixels,
            w,
            h,
            (p.0 as i32, p.1 as i32),
            3,
            3,
            [255, 248, 207, 255],
        );
    }
    image_from_rgba(w, h, pixels)
}

#[cfg(test)]
mod tests {
    use super::*;
    use flursys::{CellDefinition, Point, ResultDataset};

    #[test]
    fn three_dimensional_mouse_picking_distinguishes_faces_edges_and_vertices() {
        let mut topology = GeometryTopology::new();
        let rectangle = topology.add_rectangle(2.0, 1.0).unwrap();
        let camera = canonical_camera(
            &topology,
            View3d {
                yaw: 0.65,
                pitch: 0.48,
                zoom: 1.0,
                ..View3d::default()
            },
        )
        .unwrap();
        let vertex = topology.vertex(rectangle.vertices[0]).unwrap().position;
        let point = camera.project(vertex);
        assert_eq!(
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(point.0), f64::from(point.1)),
                2
            ),
            Some(GeometrySelectionTarget::Vertex(rectangle.vertices[0]))
        );
        let second = topology.vertex(rectangle.vertices[1]).unwrap().position;
        let middle = camera.project((vertex + second) * 0.5);
        assert_eq!(
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(middle.0), f64::from(middle.1)),
                1
            ),
            Some(GeometrySelectionTarget::Edge(rectangle.bottom))
        );
        let center = camera.project(Vec3::new(1.0, 0.5, 0.0));
        assert_eq!(
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(center.0), f64::from(center.1)),
                0
            ),
            Some(GeometrySelectionTarget::Face(rectangle.face))
        );
    }

    #[test]
    fn extruded_model_still_supports_face_and_visible_sketch_edge_picking() {
        let mut topology = GeometryTopology::new();
        let rectangle = topology.add_rectangle(2.0, 1.0).unwrap();
        let extrusion = topology.extrude_planar_face(rectangle.face, 0.5).unwrap();
        let camera = canonical_camera(
            &topology,
            View3d {
                yaw: 0.65,
                pitch: 0.48,
                zoom: 1.0,
                ..View3d::default()
            },
        )
        .unwrap();
        let top = camera.project(Vec3::new(1.0, 0.5, 0.5));
        assert_eq!(
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(top.0), f64::from(top.1)),
                0
            ),
            Some(GeometrySelectionTarget::Face(extrusion.top_face))
        );
        let edges = [
            rectangle.bottom,
            rectangle.right,
            rectangle.top,
            rectangle.left,
        ];
        assert!(edges.into_iter().any(|id| {
            let edge = topology.edge(id).unwrap();
            let flursys::EdgeGeometry::Line { start, end } = edge.geometry else {
                return false;
            };
            let (start, end) = (
                topology.vertex(start).unwrap().position,
                topology.vertex(end).unwrap().position,
            );
            let midpoint = camera.project((start + end) * 0.5);
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(midpoint.0), f64::from(midpoint.1)),
                1,
            ) == Some(GeometrySelectionTarget::Edge(id))
        }));
        assert!(extrusion.top_edges.iter().any(|id| {
            let flursys::EdgeGeometry::Line { start, end } = topology.edge(*id).unwrap().geometry
            else {
                return false;
            };
            let (start, end) = (
                topology.vertex(start).unwrap().position,
                topology.vertex(end).unwrap().position,
            );
            let midpoint = camera.project((start + end) * 0.5);
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(midpoint.0), f64::from(midpoint.1)),
                1,
            ) == Some(GeometrySelectionTarget::Edge(*id))
        }));
        assert!(extrusion.vertical_edges.iter().any(|id| {
            let flursys::EdgeGeometry::Line { start, end } = topology.edge(*id).unwrap().geometry
            else {
                return false;
            };
            let (start, end) = (
                topology.vertex(start).unwrap().position,
                topology.vertex(end).unwrap().position,
            );
            let midpoint = camera.project((start + end) * 0.5);
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(midpoint.0), f64::from(midpoint.1)),
                1,
            ) == Some(GeometrySelectionTarget::Edge(*id))
        }));
        let flursys::EdgeGeometry::Line { start, .. } =
            topology.edge(extrusion.top_edges[0]).unwrap().geometry
        else {
            panic!("expected rim edge")
        };
        let point = camera.project(topology.vertex(start).unwrap().position);
        assert_eq!(
            pick_canonical_geometry_target(
                &topology,
                View3d {
                    yaw: 0.65,
                    pitch: 0.48,
                    zoom: 1.0,
                    ..View3d::default()
                },
                (f64::from(point.0), f64::from(point.1)),
                2
            ),
            Some(GeometrySelectionTarget::Vertex(start))
        );
    }

    #[test]
    fn concave_sketch_faces_fill_only_their_interior() {
        let mut pixels = vec![0_u8; 12 * 12 * 4];
        draw_filled_polygon(
            &mut pixels,
            12,
            12,
            &[
                (1.0, 1.0),
                (9.0, 1.0),
                (9.0, 4.0),
                (4.0, 4.0),
                (4.0, 9.0),
                (1.0, 9.0),
            ],
            [50, 80, 90, 255],
        );
        let pixel = |x: usize, y: usize| &pixels[(y * 12 + x) * 4..(y * 12 + x + 1) * 4];
        assert_eq!(pixel(2, 7), [50, 80, 90, 255]);
        assert_eq!(pixel(7, 2), [50, 80, 90, 255]);
        assert_eq!(pixel(7, 7), [0, 0, 0, 0]);
    }

    #[test]
    fn volume_mesh_and_result_surfaces_share_the_panned_camera_and_pick_ray() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
            ],
            vec![CellDefinition::tetrahedron([0, 1, 2, 3])],
        )
        .unwrap();
        let cache = MeshRenderCache::build(&mesh).unwrap();
        let view = View3d {
            zoom: 1.6,
            pan: (37.0, -12.0),
            ..View3d::default()
        };
        let camera = Camera3d::fit(cache.positions().iter().copied(), view);
        let (face, screen) = mesh
            .faces()
            .iter()
            .enumerate()
            .find_map(|(face, data)| {
                let screen = camera.screen(data.center);
                let (origin, direction) = result_screen_ray(&cache, screen, view);
                (cache.pick_face_3d(origin, direction).map(|s| s.target())
                    == Some(MeshSelectionTarget::Face(face)))
                .then_some((face, screen))
            })
            .unwrap();
        let image = render_workbench_mesh(
            &mesh,
            &cache,
            &ViewTransform::default(),
            super::super::MeshDisplayMode::Surface,
            MeshColorMode::Neutral,
            MeshQualityMetric::AspectRatio,
            Some(MeshSelection::face(mesh.id(), face)),
            None,
            None,
            view,
            None,
        )
        .to_rgba8()
        .unwrap();
        let offset =
            (screen.1.round() as usize * PREVIEW_WIDTH as usize + screen.0.round() as usize) * 4;
        let mesh_scale = image.width() as f64 / PREVIEW_WIDTH as f64;
        let mesh_offset = ((screen.1 * mesh_scale).round() as usize * image.width() as usize
            + (screen.0 * mesh_scale).round() as usize)
            * 4;
        assert_eq!(
            &image.as_bytes()[mesh_offset..mesh_offset + 4],
            &[255, 214, 95, 255]
        );
        let dataset = ResultDataset::from_values(
            "3d-camera",
            &mesh,
            vec![2.0],
            vec![Vec3::new(1.0, 0.5, 0.25)],
        )
        .unwrap();
        let result = ResultRenderCache::build(&dataset).unwrap();
        let pixels = render_persisted_pixels(
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
            &result,
            ResultFieldKind::Pressure,
            0,
            0,
            1,
            "",
            None,
            &ViewTransform::default(),
            &[],
            view,
        );
        assert_ne!(&pixels[offset..offset + 4], &[9, 16, 22, 255]);
        let shifted = render_persisted_pixels(
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
            &result,
            ResultFieldKind::Pressure,
            0,
            0,
            1,
            "",
            None,
            &ViewTransform::default(),
            &[],
            View3d::default(),
        );
        assert_ne!(pixels, shifted);
    }

    fn result_cache() -> ResultRenderCache {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::TwoD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(1.0, 1.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
            ],
            vec![CellDefinition::polygon(vec![0, 1, 2, 3])],
        )
        .unwrap();
        let dataset = ResultDataset::from_values(
            "render-test",
            &mesh,
            vec![2.0],
            vec![Vec3::new(1.0, 0.5, 0.0)],
        )
        .unwrap();
        ResultRenderCache::build(&dataset).unwrap()
    }

    #[test]
    fn persisted_renderer_supports_every_palette_and_plot_mode() {
        let cache = result_cache();
        let view = ViewTransform::default();
        for colormap in 0..=3 {
            for plot in 0..=3 {
                let pixels = render_persisted_pixels(
                    240,
                    160,
                    &cache,
                    ResultFieldKind::Pressure,
                    colormap,
                    plot,
                    1,
                    "",
                    None,
                    &view,
                    &[],
                    View3d::default(),
                );
                assert_eq!(pixels.len(), 240 * 160 * 4);
                assert!(
                    pixels
                        .chunks_exact(4)
                        .any(|pixel| pixel != [9, 16, 22, 255])
                );
            }
        }
    }

    #[test]
    fn three_dimensional_vectors_render_from_exterior_owner_cells() {
        let mesh = UnstructuredMesh::from_cells(
            MeshDimension::ThreeD,
            vec![
                Point::new(0.0, 0.0, 0.0),
                Point::new(1.0, 0.0, 0.0),
                Point::new(0.0, 1.0, 0.0),
                Point::new(0.0, 0.0, 1.0),
            ],
            vec![CellDefinition::tetrahedron([0, 1, 2, 3])],
        )
        .unwrap();
        let moving = ResultDataset::from_values(
            "moving-3d",
            &mesh,
            vec![1.0],
            vec![Vec3::new(1.0, 0.0, 0.0)],
        )
        .unwrap();
        let still = ResultDataset::from_values(
            "still-3d",
            &mesh,
            vec![1.0],
            vec![Vec3::new(0.0, 0.0, 0.0)],
        )
        .unwrap();
        let view = ViewTransform::default();
        let moving_pixels = render_persisted_pixels(
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
            &ResultRenderCache::build(&moving).unwrap(),
            ResultFieldKind::Pressure,
            0,
            3,
            1,
            "",
            None,
            &view,
            &[],
            View3d::default(),
        );
        let still_pixels = render_persisted_pixels(
            PREVIEW_WIDTH,
            PREVIEW_HEIGHT,
            &ResultRenderCache::build(&still).unwrap(),
            ResultFieldKind::Pressure,
            0,
            3,
            1,
            "",
            None,
            &view,
            &[],
            View3d::default(),
        );

        assert_ne!(moving_pixels, still_pixels);
    }

    #[test]
    fn vector_sampling_and_manual_scale_are_bounded_and_deterministic() {
        assert_eq!(vector_sample_stride(1_000, 0), 21);
        assert_eq!(vector_sample_stride(1_000, 1), 11);
        assert_eq!(vector_sample_stride(1_000, 2), 6);
        assert_eq!(vector_scale_multiplier(""), 1.0);
        assert_eq!(vector_scale_multiplier("2.5"), 2.5);
        assert_eq!(vector_scale_multiplier("nan"), 1.0);
        assert_eq!(vector_scale_multiplier("100"), 5.0);
    }
}
