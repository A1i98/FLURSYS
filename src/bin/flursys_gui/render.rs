#[path = "result_slice.rs"]
mod result_slice;
pub(super) use result_slice::{pick_result_slice_cell, render_result_slice};
#[path = "sketch_visual.rs"]
mod sketch_visual;
use super::viewport::{Camera3d, View3d};
use super::{residual_level, MeshColorMode, ResidualSample, PREVIEW_HEIGHT, PREVIEW_WIDTH};
use flursys::{
    BoundaryConditionKind, BoundaryFace, ExtrudedMesh3D, FieldUpdate, GeometryPart,
    GeometryPartKind, GeometrySelectionTarget, GeometrySketch, GeometryTopology, MeshDimension,
    MeshQualityMetric, MeshRenderCache, MeshSelection, Project, ProjectCase, ResultFieldKind,
    ResultRenderCache, SketchAxis, SketchEntityKind, StreamlinePath, StructuredMesh2D,
    UnstructuredMesh, Vec3, ViewTransform,
};
use slint::{Image, Rgba8Pixel, SharedPixelBuffer};
use std::collections::VecDeque;

#[cfg(test)]
use flursys::{EnergyModel, MeshSelectionTarget};

type MeshNode = (usize, usize, usize);
type BoundaryFaceNodes = (BoundaryFace, [MeshNode; 4]);

pub(super) fn render_empty_image() -> Image {
    let mut pixels = vec![0_u8; (PREVIEW_WIDTH * PREVIEW_HEIGHT * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    image_from_rgba(PREVIEW_WIDTH, PREVIEW_HEIGHT, pixels)
}

#[allow(dead_code)]
pub(super) fn render_mesh(project: &Project, selected_boundary: Option<BoundaryFace>) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    let (length, domain_height) = project_case_domain(&project.case);
    let Ok(mesh) =
        StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, domain_height)
    else {
        return image_from_rgba(width, height, pixels);
    };
    let scale = (f64::from(width - 50) / mesh.length).min(f64::from(height - 50) / mesh.height);
    let draw_width = mesh.length * scale;
    let draw_height = mesh.height * scale;
    let origin_x = (f64::from(width) - draw_width) * 0.5;
    let origin_y = (f64::from(height) + draw_height) * 0.5;
    // Rasterise the analytical case obstacle before drawing edges so the
    // displayed structured cells match the active 2D flow domain.
    for py in 0..height {
        let y = (origin_y - f64::from(py)) / scale;
        if !(0.0..=mesh.height).contains(&y) {
            continue;
        }
        for px in 0..width {
            let x = (f64::from(px) - origin_x) / scale;
            if (0.0..=mesh.length).contains(&x) && project_case_is_solid(&project.case, x, y) {
                set_pixel(&mut pixels, width, height, px, py, [28, 31, 35, 255]);
            }
        }
    }
    let columns = sampled_indices(mesh.nx, 96);
    let rows = sampled_indices(mesh.ny, 96);
    for &i in &columns {
        let (x, _) = mesh.node(i, 0);
        let x = (origin_x + x * scale) as i32;
        draw_line(
            &mut pixels,
            width,
            height,
            (x, origin_y as i32),
            (x, (origin_y - draw_height) as i32),
            [52, 87, 106, 255],
        );
    }
    for &j in &rows {
        let (_, y) = mesh.node(0, j);
        let y = (origin_y - y * scale) as i32;
        draw_line(
            &mut pixels,
            width,
            height,
            (origin_x as i32, y),
            ((origin_x + draw_width) as i32, y),
            [52, 87, 106, 255],
        );
    }
    draw_boundary_line_2d(
        &mut pixels,
        width,
        height,
        BoundaryFace::Bottom,
        selected_boundary,
        (origin_x as i32, origin_y as i32),
        ((origin_x + draw_width) as i32, origin_y as i32),
    );
    draw_boundary_line_2d(
        &mut pixels,
        width,
        height,
        BoundaryFace::Right,
        selected_boundary,
        ((origin_x + draw_width) as i32, origin_y as i32),
        (
            (origin_x + draw_width) as i32,
            (origin_y - draw_height) as i32,
        ),
    );
    draw_boundary_line_2d(
        &mut pixels,
        width,
        height,
        BoundaryFace::Top,
        selected_boundary,
        (
            (origin_x + draw_width) as i32,
            (origin_y - draw_height) as i32,
        ),
        (origin_x as i32, (origin_y - draw_height) as i32),
    );
    draw_boundary_line_2d(
        &mut pixels,
        width,
        height,
        BoundaryFace::Left,
        selected_boundary,
        (origin_x as i32, (origin_y - draw_height) as i32),
        (origin_x as i32, origin_y as i32),
    );
    image_from_rgba(width, height, pixels)
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

#[allow(dead_code)]
pub(super) fn draw_boundary_line_2d(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    face: BoundaryFace,
    selected: Option<BoundaryFace>,
    start: (i32, i32),
    end: (i32, i32),
) {
    let color = if selected == Some(face) {
        [255, 255, 255, 255]
    } else {
        boundary_color(face)
    };
    draw_line(pixels, width, height, start, end, color);
}

pub(super) fn boundary_color(face: BoundaryFace) -> [u8; 4] {
    match face {
        BoundaryFace::Left => [77, 168, 184, 255],
        BoundaryFace::Right => [240, 195, 109, 255],
        BoundaryFace::Top => [116, 189, 135, 255],
        BoundaryFace::Bottom => [205, 115, 175, 255],
        BoundaryFace::Front => [111, 148, 236, 255],
        BoundaryFace::Back => [175, 126, 210, 255],
    }
}

pub(super) fn project_case_domain(case: &ProjectCase) -> (f64, f64) {
    match case {
        ProjectCase::LidDrivenCavity { length, height, .. }
        | ProjectCase::Cylinder { length, height, .. }
        | ProjectCase::BackwardFacingStep { length, height, .. }
        | ProjectCase::Channel { length, height, .. } => (*length, *height),
    }
}

#[allow(dead_code)]
pub(super) fn project_case_is_solid(case: &ProjectCase, x: f64, y: f64) -> bool {
    match case {
        ProjectCase::Cylinder {
            diameter,
            center_x,
            center_y,
            ..
        } => (x - center_x).powi(2) + (y - center_y).powi(2) <= (0.5 * diameter).powi(2),
        ProjectCase::BackwardFacingStep {
            step_x,
            step_height,
            ..
        } => x < *step_x && y < *step_height,
        ProjectCase::LidDrivenCavity { .. } | ProjectCase::Channel { .. } => false,
    }
}

pub(super) fn mesh_inspection(project: &Project) -> String {
    let (length, height) = project_case_domain(&project.case);
    let Ok(base) = StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, height)
    else {
        return "Invalid planar mesh settings.".to_string();
    };
    let Ok(mesh) = ExtrudedMesh3D::new(
        base,
        project.preprocessing.mesh.cells_z,
        project.preprocessing.geometry.extrusion_depth,
    ) else {
        return "Invalid extrusion settings.".to_string();
    };
    format!(
        "2D cells      {:>10}\n3D preview    {:>10}\nNodes         {:>10}\ndx / dy / dz  {:.3e} / {:.3e} / {:.3e}\nCell volume   {:.3e}\nAspect ratio  {:.3}",
        base.cell_count(),
        mesh.cell_count(),
        mesh.node_count(),
        base.dx,
        base.dy,
        mesh.dz,
        mesh.cell_volume(),
        mesh.aspect_ratio(),
    )
}

#[cfg(test)]
pub(super) fn preflight_report(project: &Project) -> Result<String, String> {
    project.validate()?;
    let (length, height) = project_case_domain(&project.case);
    let base = StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, height)?;
    let mesh = ExtrudedMesh3D::new(
        base,
        project.preprocessing.mesh.cells_z,
        project.preprocessing.geometry.extrusion_depth,
    )?;
    let mut notices = vec![format!(
        "READY\n{} × {} flow cells\n{} preview cells",
        base.nx,
        base.ny,
        mesh.cell_count()
    )];
    if base.aspect_ratio() > 10.0 {
        notices.push(format!(
            "WARNING: 2D aspect ratio {:.2}",
            base.aspect_ratio()
        ));
    }
    if mesh.aspect_ratio() > 20.0 {
        notices.push(format!(
            "WARNING: 3D preview aspect ratio {:.2}",
            mesh.aspect_ratio()
        ));
    }
    if project.physics.thermal.model == EnergyModel::ConstantProperties {
        notices.push("Energy: explicit CFL/Fourier checks active".to_string());
    }
    notices.push(format!(
        "GUI / CSV / frame: {} / {} / {} iterations",
        project.solver.gui_update_every, project.solver.history_every, project.solver.frame_every,
    ));
    Ok(notices.join("\n"))
}

pub(super) fn boundary_summary(project: &Project) -> String {
    [
        BoundaryFace::Left,
        BoundaryFace::Right,
        BoundaryFace::Bottom,
        BoundaryFace::Top,
        BoundaryFace::Front,
        BoundaryFace::Back,
    ]
    .into_iter()
    .map(|face| {
        let kind = project
            .preprocessing
            .boundary(face)
            .map(|boundary| boundary_kind_label(&boundary.kind))
            .unwrap_or("missing");
        format!("{:<6} {kind}", face.label())
    })
    .collect::<Vec<_>>()
    .join("\n")
}

pub(super) fn boundary_kind_label(kind: &BoundaryConditionKind) -> &'static str {
    match kind {
        BoundaryConditionKind::CaseDefault => "case default",
        BoundaryConditionKind::Velocity { .. } => "velocity",
        BoundaryConditionKind::PressureOutlet { .. } => "pressure outlet",
        BoundaryConditionKind::Wall { .. } => "wall",
        BoundaryConditionKind::Symmetry => "symmetry",
    }
}

pub(super) fn sampled_indices(count: usize, maximum_lines: usize) -> Vec<usize> {
    if count <= maximum_lines {
        return (0..=count).collect();
    }
    let stride = (count as f64 / maximum_lines as f64).ceil() as usize;
    let mut indices: Vec<_> = (0..=count).step_by(stride).collect();
    if indices.last().copied() != Some(count) {
        indices.push(count);
    }
    indices
}

#[allow(dead_code)] // Compatibility renderer remains covered by legacy preview tests only.
pub(super) fn render_geometry_3d(
    project: &Project,
    yaw: f32,
    pitch: f32,
    zoom: f32,
    selected_boundary: Option<BoundaryFace>,
) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    let (length, domain_height) = project_case_domain(&project.case);
    let Ok(base) =
        StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, domain_height)
    else {
        return image_from_rgba(width, height, pixels);
    };
    let Ok(mesh) = ExtrudedMesh3D::new(
        base,
        project.preprocessing.mesh.cells_z,
        project.preprocessing.geometry.extrusion_depth,
    ) else {
        return image_from_rgba(width, height, pixels);
    };
    let camera = MeshCamera::fit(&mesh, yaw, pitch, zoom);
    let xs = sampled_indices(mesh.base.nx, 28);
    let ys = sampled_indices(mesh.base.ny, 28);
    let zs = sampled_indices(mesh.nz, 16);
    for &k in &zs {
        let color = if k == 0 || k == mesh.nz {
            [94, 126, 143, 255]
        } else {
            [37, 65, 82, 255]
        };
        for &i in &xs {
            draw_line(
                &mut pixels,
                width,
                height,
                camera.project(mesh.node(i, 0, k)),
                camera.project(mesh.node(i, mesh.base.ny, k)),
                color,
            );
        }
        for &j in &ys {
            draw_line(
                &mut pixels,
                width,
                height,
                camera.project(mesh.node(0, j, k)),
                camera.project(mesh.node(mesh.base.nx, j, k)),
                color,
            );
        }
    }
    for &i in &xs {
        for &j in &ys {
            draw_line(
                &mut pixels,
                width,
                height,
                camera.project(mesh.node(i, j, 0)),
                camera.project(mesh.node(i, j, mesh.nz)),
                [45, 82, 103, 255],
            );
        }
    }
    draw_boundaries_3d(&mut pixels, width, height, &mesh, camera, selected_boundary);
    draw_case_solid_3d(&mut pixels, width, height, &project.case, &mesh, camera);
    for (index, part) in project.preprocessing.geometry.parts.iter().enumerate() {
        draw_part_with_camera(&mut pixels, width, height, part, camera, part_color(index));
    }

    image_from_rgba(width, height, pixels)
}

/// Renders stable CAD faces directly from `GeometryTopology`, without using a
/// legacy project preview mesh. Selection uses the same `FaceId` as CAD picking.
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

#[allow(dead_code)]
pub(super) fn draw_boundaries_3d(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    mesh: &ExtrudedMesh3D,
    camera: MeshCamera,
    selected: Option<BoundaryFace>,
) {
    for (face, corners) in boundary_face_nodes(mesh) {
        let color = if selected == Some(face) {
            [255, 255, 255, 255]
        } else {
            boundary_color(face)
        };
        let points = corners.map(|(i, j, k)| camera.project(mesh.node(i, j, k)));
        for index in 0..4 {
            draw_line(
                pixels,
                width,
                height,
                points[index],
                points[(index + 1) % 4],
                color,
            );
        }
    }
}

pub(super) fn boundary_face_nodes(mesh: &ExtrudedMesh3D) -> [BoundaryFaceNodes; 6] {
    [
        (
            BoundaryFace::Left,
            [
                (0, 0, 0),
                (0, mesh.base.ny, 0),
                (0, mesh.base.ny, mesh.nz),
                (0, 0, mesh.nz),
            ],
        ),
        (
            BoundaryFace::Right,
            [
                (mesh.base.nx, 0, 0),
                (mesh.base.nx, mesh.base.ny, 0),
                (mesh.base.nx, mesh.base.ny, mesh.nz),
                (mesh.base.nx, 0, mesh.nz),
            ],
        ),
        (
            BoundaryFace::Bottom,
            [
                (0, 0, 0),
                (mesh.base.nx, 0, 0),
                (mesh.base.nx, 0, mesh.nz),
                (0, 0, mesh.nz),
            ],
        ),
        (
            BoundaryFace::Top,
            [
                (0, mesh.base.ny, 0),
                (mesh.base.nx, mesh.base.ny, 0),
                (mesh.base.nx, mesh.base.ny, mesh.nz),
                (0, mesh.base.ny, mesh.nz),
            ],
        ),
        (
            BoundaryFace::Front,
            [
                (0, 0, 0),
                (mesh.base.nx, 0, 0),
                (mesh.base.nx, mesh.base.ny, 0),
                (0, mesh.base.ny, 0),
            ],
        ),
        (
            BoundaryFace::Back,
            [
                (0, 0, mesh.nz),
                (mesh.base.nx, 0, mesh.nz),
                (mesh.base.nx, mesh.base.ny, mesh.nz),
                (0, mesh.base.ny, mesh.nz),
            ],
        ),
    ]
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

pub(super) fn pick_boundary_2d(project: &Project, point: (f64, f64)) -> Option<BoundaryFace> {
    let (length, domain_height) = project_case_domain(&project.case);
    let mesh =
        StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, domain_height).ok()?;
    let scale = (f64::from(PREVIEW_WIDTH - 50) / mesh.length)
        .min(f64::from(PREVIEW_HEIGHT - 50) / mesh.height);
    let draw_width = mesh.length * scale;
    let draw_height = mesh.height * scale;
    let origin_x = (f64::from(PREVIEW_WIDTH) - draw_width) * 0.5;
    let origin_y = (f64::from(PREVIEW_HEIGHT) + draw_height) * 0.5;
    let edges = [
        (
            BoundaryFace::Bottom,
            (origin_x, origin_y),
            (origin_x + draw_width, origin_y),
        ),
        (
            BoundaryFace::Right,
            (origin_x + draw_width, origin_y),
            (origin_x + draw_width, origin_y - draw_height),
        ),
        (
            BoundaryFace::Top,
            (origin_x + draw_width, origin_y - draw_height),
            (origin_x, origin_y - draw_height),
        ),
        (
            BoundaryFace::Left,
            (origin_x, origin_y - draw_height),
            (origin_x, origin_y),
        ),
    ];
    nearest_boundary(
        point,
        edges
            .into_iter()
            .map(|(face, start, end)| (face, distance_to_segment(point, start, end))),
        14.0,
    )
}

pub(super) fn pick_boundary_3d(
    project: &Project,
    view: View3d,
    point: (f64, f64),
) -> Option<BoundaryFace> {
    let (length, height) = project_case_domain(&project.case);
    let base = StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, height).ok()?;
    let mesh = ExtrudedMesh3D::new(
        base,
        project.preprocessing.mesh.cells_z,
        project.preprocessing.geometry.extrusion_depth,
    )
    .ok()?;
    let camera = MeshCamera::fit(&mesh, view.yaw, view.pitch, view.zoom);
    let candidates = boundary_face_nodes(&mesh)
        .into_iter()
        .map(|(face, corners)| {
            let points = corners.map(|node| {
                let point = camera.project(mesh.node(node.0, node.1, node.2));
                (f64::from(point.0), f64::from(point.1))
            });
            let distance = (0..4)
                .map(|index| distance_to_segment(point, points[index], points[(index + 1) % 4]))
                .fold(f64::INFINITY, f64::min);
            (face, distance)
        });
    nearest_boundary(point, candidates, 18.0)
}

pub(super) fn nearest_boundary(
    _point: (f64, f64),
    candidates: impl Iterator<Item = (BoundaryFace, f64)>,
    tolerance: f64,
) -> Option<BoundaryFace> {
    candidates
        .min_by(|(_, left), (_, right)| {
            left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal)
        })
        .and_then(|(face, distance)| (distance <= tolerance).then_some(face))
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

#[allow(dead_code)]
pub(super) fn draw_case_solid_3d(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    case: &ProjectCase,
    mesh: &ExtrudedMesh3D,
    camera: MeshCamera,
) {
    const COLOR: [u8; 4] = [240, 195, 109, 255];
    match case {
        ProjectCase::Cylinder {
            diameter,
            center_x,
            center_y,
            ..
        } => {
            let radius = 0.5 * *diameter;
            let mut previous_bottom = None;
            let mut previous_top = None;
            for step in 0..=48 {
                let angle = step as f64 * std::f64::consts::TAU / 48.0;
                let x = *center_x + radius * angle.cos();
                let y = *center_y + radius * angle.sin();
                let bottom = camera.project((x, y, 0.0));
                let top = camera.project((x, y, mesh.depth));
                if let Some(previous) = previous_bottom {
                    draw_line(pixels, width, height, previous, bottom, COLOR);
                }
                if let Some(previous) = previous_top {
                    draw_line(pixels, width, height, previous, top, COLOR);
                }
                if step % 8 == 0 {
                    draw_line(pixels, width, height, bottom, top, COLOR);
                }
                previous_bottom = Some(bottom);
                previous_top = Some(top);
            }
        }
        ProjectCase::BackwardFacingStep {
            step_x,
            step_height,
            ..
        } => draw_case_box(
            pixels,
            width,
            height,
            camera,
            (0.0, 0.0, 0.0),
            (*step_x, *step_height, mesh.depth),
            COLOR,
        ),
        ProjectCase::LidDrivenCavity { .. } | ProjectCase::Channel { .. } => {}
    }
}

#[allow(dead_code)]
pub(super) fn draw_case_box(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    camera: MeshCamera,
    min: (f64, f64, f64),
    max: (f64, f64, f64),
    color: [u8; 4],
) {
    let corners = [
        (min.0, min.1, min.2),
        (max.0, min.1, min.2),
        (max.0, max.1, min.2),
        (min.0, max.1, min.2),
        (min.0, min.1, max.2),
        (max.0, min.1, max.2),
        (max.0, max.1, max.2),
        (min.0, max.1, max.2),
    ]
    .map(|point| camera.project(point));
    for (from, to) in [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ] {
        draw_line(pixels, width, height, corners[from], corners[to], color);
    }
}

#[allow(dead_code)]
pub(super) fn draw_part_with_camera(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    part: &GeometryPart,
    camera: MeshCamera,
    color: [u8; 4],
) {
    match part.kind {
        GeometryPartKind::Box {
            length,
            width: part_width,
            height: part_height,
        } => draw_case_box(
            pixels,
            width,
            height,
            camera,
            (
                part.x - 0.5 * length,
                part.y - 0.5 * part_width,
                part.z - 0.5 * part_height,
            ),
            (
                part.x + 0.5 * length,
                part.y + 0.5 * part_width,
                part.z + 0.5 * part_height,
            ),
            color,
        ),
        GeometryPartKind::Cylinder {
            radius,
            height: part_height,
            ..
        } => draw_camera_ring_solid(
            pixels,
            width,
            height,
            part,
            radius,
            part_height,
            camera,
            color,
        ),
        GeometryPartKind::Cone {
            radius,
            height: part_height,
            ..
        } => {
            let apex = camera.project((part.x, part.y, part.z + 0.5 * part_height));
            let mut previous = None;
            for step in 0..=32 {
                let angle = step as f64 * std::f64::consts::TAU / 32.0;
                let point = camera.project((
                    part.x + radius * angle.cos(),
                    part.y + radius * angle.sin(),
                    part.z - 0.5 * part_height,
                ));
                if let Some(last) = previous {
                    draw_line(pixels, width, height, last, point, color);
                }
                if step % 8 == 0 {
                    draw_line(pixels, width, height, point, apex, color);
                }
                previous = Some(point);
            }
        }
        GeometryPartKind::Sphere { radius, .. } => {
            for latitude in [-0.75_f64, -0.35, 0.0, 0.35, 0.75] {
                let z = radius * latitude;
                draw_camera_ring(
                    pixels,
                    width,
                    height,
                    part,
                    (radius * radius - z * z).sqrt(),
                    z,
                    camera,
                    color,
                );
            }
        }
        GeometryPartKind::Torus {
            major_radius,
            minor_radius,
            ..
        } => {
            for tube_angle in [
                0.0_f64,
                std::f64::consts::FRAC_PI_2,
                std::f64::consts::PI,
                std::f64::consts::PI * 1.5,
            ] {
                let mut previous = None;
                for step in 0..=40 {
                    let ring_angle = step as f64 * std::f64::consts::TAU / 40.0;
                    let radial = major_radius + minor_radius * tube_angle.cos();
                    let point = camera.project((
                        part.x + radial * ring_angle.cos(),
                        part.y + radial * ring_angle.sin(),
                        part.z + minor_radius * tube_angle.sin(),
                    ));
                    if let Some(last) = previous {
                        draw_line(pixels, width, height, last, point, color);
                    }
                    previous = Some(point);
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
#[allow(dead_code)]
fn draw_camera_ring_solid(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    part: &GeometryPart,
    radius: f64,
    part_height: f64,
    camera: MeshCamera,
    color: [u8; 4],
) {
    let mut previous_bottom = None;
    let mut previous_top = None;
    for step in 0..=32 {
        let angle = step as f64 * std::f64::consts::TAU / 32.0;
        let x = part.x + radius * angle.cos();
        let y = part.y + radius * angle.sin();
        let bottom = camera.project((x, y, part.z - 0.5 * part_height));
        let top = camera.project((x, y, part.z + 0.5 * part_height));
        if let Some(last) = previous_bottom {
            draw_line(pixels, width, height, last, bottom, color);
        }
        if let Some(last) = previous_top {
            draw_line(pixels, width, height, last, top, color);
        }
        if step % 8 == 0 {
            draw_line(pixels, width, height, bottom, top, color);
        }
        previous_bottom = Some(bottom);
        previous_top = Some(top);
    }
}

#[allow(clippy::too_many_arguments)]
#[allow(dead_code)]
fn draw_camera_ring(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    part: &GeometryPart,
    radius: f64,
    z_offset: f64,
    camera: MeshCamera,
    color: [u8; 4],
) {
    let mut previous = None;
    for step in 0..=32 {
        let angle = step as f64 * std::f64::consts::TAU / 32.0;
        let point = camera.project((
            part.x + radius * angle.cos(),
            part.y + radius * angle.sin(),
            part.z + z_offset,
        ));
        if let Some(last) = previous {
            draw_line(pixels, width, height, last, point, color);
        }
        previous = Some(point);
    }
}

#[derive(Clone, Copy)]
pub(super) struct MeshCamera {
    camera: Camera3d,
    #[cfg(test)]
    yaw: f64,
    #[cfg(test)]
    pitch: f64,
    z_exaggeration: f64,
}

impl MeshCamera {
    pub(super) fn fit(mesh: &ExtrudedMesh3D, yaw: f32, pitch: f32, zoom: f32) -> Self {
        let z_exaggeration = preview_z_exaggeration(mesh);
        let camera = Camera3d::fit(
            [
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(
                    mesh.base.length,
                    mesh.base.height,
                    mesh.depth * z_exaggeration,
                ),
            ],
            View3d {
                yaw,
                pitch,
                zoom,
                ..View3d::default()
            },
        );
        Self {
            camera,
            #[cfg(test)]
            yaw: f64::from(yaw),
            #[cfg(test)]
            pitch: f64::from(pitch),
            z_exaggeration,
        }
    }

    fn project(&self, (x, y, z): (f64, f64, f64)) -> (i32, i32) {
        self.camera
            .project(Vec3::new(x, y, z * self.z_exaggeration))
    }
}

#[cfg(test)]
const TRIAD_ORIGIN: (i32, i32) = (PREVIEW_WIDTH as i32 - 48, PREVIEW_HEIGHT as i32 - 48);
#[cfg(test)]
const TRIAD_LENGTH: f64 = 25.0;
#[cfg(test)]
const TRIAD_COLORS: [[u8; 4]; 3] = [[235, 91, 91, 255], [91, 210, 125, 255], [91, 144, 235, 255]];

#[cfg(test)]
pub(super) fn orientation_triad_endpoints(camera: MeshCamera) -> [(i32, i32); 3] {
    triad_endpoints(camera.yaw, camera.pitch)
}

#[cfg(test)]
fn triad_endpoints(yaw: f64, pitch: f64) -> [(i32, i32); 3] {
    [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)].map(|direction| {
        let (x, y, z) = direction;
        let horizontal = x * yaw.cos() - y * yaw.sin();
        let depth = x * yaw.sin() + y * yaw.cos();
        let (x, y) = (horizontal, z * pitch.cos() + depth * pitch.sin());
        let length = x.hypot(y).max(0.15);
        (
            TRIAD_ORIGIN.0 + (x / length * TRIAD_LENGTH) as i32,
            TRIAD_ORIGIN.1 - (y / length * TRIAD_LENGTH) as i32,
        )
    })
}

#[cfg(test)]
pub(super) fn pick_orientation_triad(point: (f64, f64), camera: MeshCamera) -> Option<i32> {
    pick_triad_endpoints(point, orientation_triad_endpoints(camera))
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

#[cfg(test)]
pub(super) fn pick_canonical_orientation_axis(
    yaw: f32,
    pitch: f32,
    point: (f64, f64),
) -> Option<i32> {
    pick_triad_endpoints(point, triad_endpoints(f64::from(yaw), f64::from(pitch)))
}

#[cfg(test)]
fn pick_triad_endpoints(point: (f64, f64), endpoints: [(i32, i32); 3]) -> Option<i32> {
    if let Some((axis, _)) = endpoints
        .into_iter()
        .enumerate()
        .map(|(axis, endpoint)| {
            (
                axis as i32,
                (point.0 - f64::from(endpoint.0)).hypot(point.1 - f64::from(endpoint.1)),
            )
        })
        .filter(|(_, distance)| *distance <= 10.0)
        .min_by(|a, b| a.1.total_cmp(&b.1))
    {
        return Some(axis);
    }
    endpoints
        .into_iter()
        .enumerate()
        .map(|(axis, endpoint)| {
            (
                axis as i32,
                distance_to_segment(
                    point,
                    (f64::from(TRIAD_ORIGIN.0), f64::from(TRIAD_ORIGIN.1)),
                    (f64::from(endpoint.0), f64::from(endpoint.1)),
                ),
            )
        })
        .min_by(|(_, left), (_, right)| left.total_cmp(right))
        .and_then(|(axis, distance)| (distance <= 6.0).then_some(axis))
}

#[cfg(test)]
fn draw_canonical_orientation_triad(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    yaw: f32,
    pitch: f32,
) {
    draw_triad_endpoints(
        pixels,
        width,
        height,
        triad_endpoints(f64::from(yaw), f64::from(pitch)),
    );
}

#[cfg(test)]
fn draw_triad_endpoints(pixels: &mut [u8], width: u32, height: u32, endpoints: [(i32, i32); 3]) {
    draw_marker(pixels, width, height, TRIAD_ORIGIN, [224, 235, 242, 255]);
    for (axis, (endpoint, color)) in endpoints.into_iter().zip(TRIAD_COLORS).enumerate() {
        draw_line(pixels, width, height, TRIAD_ORIGIN, endpoint, color);
        draw_marker(pixels, width, height, endpoint, color);
        let dx = f64::from(endpoint.0 - TRIAD_ORIGIN.0);
        let dy = f64::from(endpoint.1 - TRIAD_ORIGIN.1);
        let length = dx.hypot(dy).max(1.0);
        let (ux, uy) = (dx / length, dy / length);
        for sign in [-1.0, 1.0] {
            let wing = (
                endpoint.0 - (ux * 7.0 - sign * uy * 4.0) as i32,
                endpoint.1 - (uy * 7.0 + sign * ux * 4.0) as i32,
            );
            draw_line(pixels, width, height, endpoint, wing, color);
        }
        let label = (endpoint.0 + 5, endpoint.1 - 5);
        match axis {
            0 => {
                draw_line(
                    pixels,
                    width,
                    height,
                    label,
                    (label.0 + 6, label.1 + 8),
                    color,
                );
                draw_line(
                    pixels,
                    width,
                    height,
                    (label.0 + 6, label.1),
                    (label.0, label.1 + 8),
                    color,
                );
            }
            1 => {
                draw_line(
                    pixels,
                    width,
                    height,
                    label,
                    (label.0 + 3, label.1 + 4),
                    color,
                );
                draw_line(
                    pixels,
                    width,
                    height,
                    (label.0 + 6, label.1),
                    (label.0 + 3, label.1 + 4),
                    color,
                );
                draw_line(
                    pixels,
                    width,
                    height,
                    (label.0 + 3, label.1 + 4),
                    (label.0 + 3, label.1 + 8),
                    color,
                );
            }
            _ => {
                draw_line(pixels, width, height, label, (label.0 + 6, label.1), color);
                draw_line(
                    pixels,
                    width,
                    height,
                    (label.0 + 6, label.1),
                    (label.0, label.1 + 8),
                    color,
                );
                draw_line(
                    pixels,
                    width,
                    height,
                    (label.0, label.1 + 8),
                    (label.0 + 6, label.1 + 8),
                    color,
                );
            }
        }
    }
}

#[cfg(test)]
pub(super) fn pick_orientation_axis_3d(
    project: &Project,
    yaw: f32,
    pitch: f32,
    zoom: f32,
    point: (f64, f64),
) -> Option<i32> {
    let (length, height) = project_case_domain(&project.case);
    let base = StructuredMesh2D::new(project.solver.nx, project.solver.ny, length, height).ok()?;
    let mesh = ExtrudedMesh3D::new(
        base,
        project.preprocessing.mesh.cells_z,
        project.preprocessing.geometry.extrusion_depth,
    )
    .ok()?;
    pick_orientation_triad(point, MeshCamera::fit(&mesh, yaw, pitch, zoom))
}

pub(super) fn preview_z_exaggeration(mesh: &ExtrudedMesh3D) -> f64 {
    (0.16 * mesh.base.length.max(mesh.base.height) / mesh.depth).clamp(1.0, 12.0)
}

#[allow(dead_code)]
pub(super) fn render_geometry_3d_legacy(
    project: &Project,
    yaw: f32,
    pitch: f32,
    zoom: f32,
) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);

    let front_width = (286.0 * zoom) as i32;
    let front_height = (202.0 * zoom) as i32;
    let front_left = 218 - front_width / 2;
    let front_right = front_left + front_width;
    let front_top = 164 - front_height / 2;
    let front_bottom = front_top + front_height;
    let depth = 62.0 * zoom;
    let offset_x = (yaw.cos() * depth) as i32;
    let offset_y = (-0.68 * depth + pitch.sin() * depth * 0.82) as i32;
    let front = [
        (front_left, front_bottom),
        (front_right, front_bottom),
        (front_right, front_top),
        (front_left, front_top),
    ];
    let back = front.map(|(x, y)| (x + offset_x, y + offset_y));
    let mesh = &project.preprocessing.mesh;
    let nx = project.solver.nx.clamp(4, 28) as i32;
    let ny = project.solver.ny.clamp(4, 24) as i32;
    let nz = mesh.cells_z.clamp(1, 16) as i32;

    for column in 0..=nx {
        let x = front_left + (front_right - front_left) * column / nx;
        draw_line(
            &mut pixels,
            width,
            height,
            (x, front_top),
            (x, front_bottom),
            [45, 82, 103, 255],
        );
    }
    for row in 0..=ny {
        let y = front_top + (front_bottom - front_top) * row / ny;
        draw_line(
            &mut pixels,
            width,
            height,
            (front_left, y),
            (front_right, y),
            [45, 82, 103, 255],
        );
    }
    for layer in 0..=nz {
        let ratio = layer as f32 / nz as f32;
        for &(from, to) in &[(front[0], back[0]), (front[1], back[1])] {
            let x = from.0 + ((to.0 - from.0) as f32 * ratio) as i32;
            let y = from.1 + ((to.1 - from.1) as f32 * ratio) as i32;
            draw_line(
                &mut pixels,
                width,
                height,
                (x, y),
                (x + front_right - front_left, y),
                [34, 62, 79, 255],
            );
        }
    }
    for index in 0..4 {
        draw_line(
            &mut pixels,
            width,
            height,
            front[index],
            front[(index + 1) % 4],
            [196, 215, 226, 255],
        );
        draw_line(
            &mut pixels,
            width,
            height,
            back[index],
            back[(index + 1) % 4],
            [94, 126, 143, 255],
        );
        draw_line(
            &mut pixels,
            width,
            height,
            front[index],
            back[index],
            [94, 126, 143, 255],
        );
    }

    draw_line(
        &mut pixels,
        width,
        height,
        front[0],
        front[3],
        [77, 168, 184, 255],
    );
    draw_line(
        &mut pixels,
        width,
        height,
        front[1],
        front[2],
        [240, 195, 109, 255],
    );
    draw_line(
        &mut pixels,
        width,
        height,
        front[2],
        front[3],
        [116, 189, 135, 255],
    );
    draw_line(
        &mut pixels,
        width,
        height,
        front[0],
        front[1],
        [205, 115, 175, 255],
    );

    if project.preprocessing.geometry.parts.is_empty() {
        match &project.case {
            ProjectCase::Cylinder { .. } => draw_ellipse(
                &mut pixels,
                width,
                height,
                (front_left + 105, (front_top + front_bottom) / 2),
                24,
                24,
                [240, 195, 109, 255],
            ),
            ProjectCase::BackwardFacingStep { .. } => {
                let x = front_left + 90;
                let y = front_bottom - 55;
                draw_line(
                    &mut pixels,
                    width,
                    height,
                    (front_left, y),
                    (x, y),
                    [240, 195, 109, 255],
                );
                draw_line(
                    &mut pixels,
                    width,
                    height,
                    (x, y),
                    (x, front_bottom),
                    [240, 195, 109, 255],
                );
            }
            ProjectCase::LidDrivenCavity { .. } | ProjectCase::Channel { .. } => {}
        }
    } else {
        let scale = geometry_scene_scale(&project.preprocessing.geometry.parts);
        for (index, part) in project.preprocessing.geometry.parts.iter().enumerate() {
            let color = part_color(index);
            match &part.kind {
                GeometryPartKind::Box {
                    length,
                    width,
                    height,
                } => draw_part_box(
                    &mut pixels,
                    PREVIEW_WIDTH,
                    PREVIEW_HEIGHT,
                    part,
                    *length,
                    *width,
                    *height,
                    yaw,
                    scale * f64::from(zoom),
                    color,
                ),
                GeometryPartKind::Cylinder { radius, height, .. } => draw_part_cylinder(
                    &mut pixels,
                    PREVIEW_WIDTH,
                    PREVIEW_HEIGHT,
                    part,
                    *radius,
                    *height,
                    yaw,
                    scale * f64::from(zoom),
                    color,
                ),
                GeometryPartKind::Cone { radius, height, .. } => draw_part_cone(
                    &mut pixels,
                    PREVIEW_WIDTH,
                    PREVIEW_HEIGHT,
                    part,
                    *radius,
                    *height,
                    yaw,
                    scale * f64::from(zoom),
                    color,
                ),
                GeometryPartKind::Sphere { radius, .. } => draw_part_sphere(
                    &mut pixels,
                    PREVIEW_WIDTH,
                    PREVIEW_HEIGHT,
                    part,
                    *radius,
                    yaw,
                    scale * f64::from(zoom),
                    color,
                ),
                GeometryPartKind::Torus {
                    major_radius,
                    minor_radius,
                    ..
                } => draw_part_torus(
                    &mut pixels,
                    PREVIEW_WIDTH,
                    PREVIEW_HEIGHT,
                    part,
                    *major_radius,
                    *minor_radius,
                    yaw,
                    scale * f64::from(zoom),
                    color,
                ),
            }
        }
    }

    image_from_rgba(width, height, pixels)
}

pub(super) fn geometry_scene_scale(parts: &[GeometryPart]) -> f64 {
    let extent = parts.iter().fold(1.0_f64, |extent, part| {
        let size = match &part.kind {
            GeometryPartKind::Box {
                length,
                width,
                height,
            } => 0.5 * length.max(*width).max(*height),
            GeometryPartKind::Cylinder { radius, height, .. }
            | GeometryPartKind::Cone { radius, height, .. } => radius.max(0.5 * height),
            GeometryPartKind::Sphere { radius, .. } => *radius,
            GeometryPartKind::Torus {
                major_radius,
                minor_radius,
                ..
            } => major_radius + minor_radius,
        };
        extent
            .max(part.x.abs() + size)
            .max(part.y.abs() + size)
            .max(part.z.abs() + size)
    });
    (110.0 / extent).clamp(16.0, 72.0)
}

pub(super) fn part_color(index: usize) -> [u8; 4] {
    const COLORS: [[u8; 4]; 5] = [
        [77, 168, 184, 255],
        [240, 195, 109, 255],
        [116, 189, 135, 255],
        [205, 115, 175, 255],
        [134, 147, 241, 255],
    ];
    COLORS[index % COLORS.len()]
}

pub(super) fn scene_point(x: f64, y: f64, z: f64, rotation: f32, scale: f64) -> (i32, i32) {
    let angle = f64::from(rotation) + 0.75;
    let horizontal = x * angle.cos() - y * angle.sin();
    let depth = x * angle.sin() + y * angle.cos();
    (
        (260.0 + horizontal * scale) as i32,
        (218.0 - z * scale - depth * scale * 0.36) as i32,
    )
}

#[allow(clippy::too_many_arguments)]
pub(super) fn draw_part_box(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    length: f64,
    width: f64,
    height: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    let hx = 0.5 * length;
    let hy = 0.5 * width;
    let hz = 0.5 * height;
    let corners = [
        (-hx, -hy, -hz),
        (hx, -hy, -hz),
        (hx, hy, -hz),
        (-hx, hy, -hz),
        (-hx, -hy, hz),
        (hx, -hy, hz),
        (hx, hy, hz),
        (-hx, hy, hz),
    ]
    .map(|(x, y, z)| scene_point(part.x + x, part.y + y, part.z + z, rotation, scale));
    for (from, to) in [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ] {
        draw_line(
            pixels,
            image_width,
            image_height,
            corners[from],
            corners[to],
            color,
        );
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn draw_part_cylinder(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    radius: f64,
    height: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    let mut previous_bottom = None;
    let mut previous_top = None;
    for step in 0..=32 {
        let angle = step as f64 * std::f64::consts::TAU / 32.0;
        let x = part.x + radius * angle.cos();
        let y = part.y + radius * angle.sin();
        let bottom = scene_point(x, y, part.z - 0.5 * height, rotation, scale);
        let top = scene_point(x, y, part.z + 0.5 * height, rotation, scale);
        if let Some(previous) = previous_bottom {
            draw_line(pixels, image_width, image_height, previous, bottom, color);
        }
        if let Some(previous) = previous_top {
            draw_line(pixels, image_width, image_height, previous, top, color);
        }
        if step % 8 == 0 {
            draw_line(pixels, image_width, image_height, bottom, top, color);
        }
        previous_bottom = Some(bottom);
        previous_top = Some(top);
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn draw_part_cone(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    radius: f64,
    height: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    let apex = scene_point(part.x, part.y, part.z + 0.5 * height, rotation, scale);
    let mut previous = None;
    for step in 0..=32 {
        let angle = step as f64 * std::f64::consts::TAU / 32.0;
        let point = scene_point(
            part.x + radius * angle.cos(),
            part.y + radius * angle.sin(),
            part.z - 0.5 * height,
            rotation,
            scale,
        );
        if let Some(last) = previous {
            draw_line(pixels, image_width, image_height, last, point, color);
        }
        if step % 8 == 0 {
            draw_line(pixels, image_width, image_height, point, apex, color);
        }
        previous = Some(point);
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn draw_part_sphere(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    radius: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    for latitude in [-0.75_f64, -0.35, 0.0, 0.35, 0.75] {
        let z = radius * latitude;
        let ring = (radius * radius - z * z).sqrt();
        draw_ring(
            pixels,
            image_width,
            image_height,
            part,
            ring,
            z,
            rotation,
            scale,
            color,
        );
    }
    for angle in [
        0.0_f64,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI * 0.25,
        std::f64::consts::PI * 0.75,
    ] {
        let mut previous = None;
        for step in 0..=32 {
            let phi = -std::f64::consts::FRAC_PI_2 + step as f64 * std::f64::consts::PI / 32.0;
            let point = scene_point(
                part.x + radius * phi.cos() * angle.cos(),
                part.y + radius * phi.cos() * angle.sin(),
                part.z + radius * phi.sin(),
                rotation,
                scale,
            );
            if let Some(last) = previous {
                draw_line(pixels, image_width, image_height, last, point, color);
            }
            previous = Some(point);
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn draw_part_torus(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    major_radius: f64,
    minor_radius: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    for tube_angle in [
        0.0_f64,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
        std::f64::consts::PI * 1.5,
    ] {
        let mut previous = None;
        for step in 0..=40 {
            let ring_angle = step as f64 * std::f64::consts::TAU / 40.0;
            let radial = major_radius + minor_radius * tube_angle.cos();
            let point = scene_point(
                part.x + radial * ring_angle.cos(),
                part.y + radial * ring_angle.sin(),
                part.z + minor_radius * tube_angle.sin(),
                rotation,
                scale,
            );
            if let Some(last) = previous {
                draw_line(pixels, image_width, image_height, last, point, color);
            }
            previous = Some(point);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_ring(
    pixels: &mut [u8],
    image_width: u32,
    image_height: u32,
    part: &GeometryPart,
    radius: f64,
    z_offset: f64,
    rotation: f32,
    scale: f64,
    color: [u8; 4],
) {
    let mut previous = None;
    for step in 0..=32 {
        let angle = step as f64 * std::f64::consts::TAU / 32.0;
        let point = scene_point(
            part.x + radius * angle.cos(),
            part.y + radius * angle.sin(),
            part.z + z_offset,
            rotation,
            scale,
        );
        if let Some(last) = previous {
            draw_line(pixels, image_width, image_height, last, point, color);
        }
        previous = Some(point);
    }
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

pub(super) fn render_scalar_field(
    field: &FieldUpdate,
    values: &[f64],
    symmetric: bool,
    colormap: i32,
    plot: i32,
    contour_levels: usize,
) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let (min_value, max_value) = scalar_range(values, symmetric);
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    if plot != 3 {
        for y in 0..height {
            let j = ((height - 1 - y) as usize * field.ny / height as usize).min(field.ny - 1);
            for x in 0..width {
                let i = (x as usize * field.nx / width as usize).min(field.nx - 1);
                let index = i + field.nx * j;
                let color = if field.solid.get(index).copied().unwrap_or(true) {
                    [22, 25, 29, 255]
                } else {
                    scalar_color(
                        normalize(
                            values.get(index).copied().unwrap_or(0.0),
                            min_value,
                            max_value,
                        ),
                        colormap,
                    )
                };
                set_pixel(&mut pixels, width, height, x, y, color);
            }
        }
        draw_colormap_legend(&mut pixels, width, height, colormap);
    }
    if matches!(plot, 1 | 2) {
        draw_contours(
            &mut pixels,
            width,
            height,
            field,
            values,
            min_value,
            max_value,
            contour_levels,
        );
    }
    if matches!(plot, 2 | 3) {
        draw_velocity_vectors(&mut pixels, width, height, field);
    }
    image_from_rgba(width, height, pixels)
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

#[allow(clippy::too_many_arguments)]
fn draw_contours(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    field: &FieldUpdate,
    values: &[f64],
    min_value: f64,
    max_value: f64,
    levels: usize,
) {
    if field.nx < 2 || field.ny < 2 || values.len() != field.nx * field.ny {
        return;
    }
    let color = [235, 242, 245, 235];
    for level_index in 1..levels {
        let level = min_value + (max_value - min_value) * level_index as f64 / levels as f64;
        for j in 0..field.ny - 1 {
            for i in 0..field.nx - 1 {
                let indices = [
                    i + field.nx * j,
                    i + 1 + field.nx * j,
                    i + 1 + field.nx * (j + 1),
                    i + field.nx * (j + 1),
                ];
                if indices
                    .iter()
                    .any(|&index| field.solid.get(index).copied().unwrap_or(true))
                {
                    continue;
                }
                let points = [
                    (
                        (i as f64 + 0.5) * width as f64 / field.nx as f64,
                        height as f64 - (j as f64 + 0.5) * height as f64 / field.ny as f64,
                    ),
                    (
                        (i as f64 + 1.5) * width as f64 / field.nx as f64,
                        height as f64 - (j as f64 + 0.5) * height as f64 / field.ny as f64,
                    ),
                    (
                        (i as f64 + 1.5) * width as f64 / field.nx as f64,
                        height as f64 - (j as f64 + 1.5) * height as f64 / field.ny as f64,
                    ),
                    (
                        (i as f64 + 0.5) * width as f64 / field.nx as f64,
                        height as f64 - (j as f64 + 1.5) * height as f64 / field.ny as f64,
                    ),
                ];
                let mut crossings = Vec::with_capacity(4);
                for (start, end) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
                    let a = values[indices[start]];
                    let b = values[indices[end]];
                    if (a < level) != (b < level) {
                        let fraction = (level - a) / (b - a);
                        crossings.push((
                            points[start].0 + fraction * (points[end].0 - points[start].0),
                            points[start].1 + fraction * (points[end].1 - points[start].1),
                        ));
                    }
                }
                for pair in crossings.chunks_exact(2) {
                    draw_line(
                        pixels,
                        width,
                        height,
                        (pair[0].0 as i32, pair[0].1 as i32),
                        (pair[1].0 as i32, pair[1].1 as i32),
                        color,
                    );
                }
            }
        }
    }
}

fn draw_velocity_vectors(pixels: &mut [u8], width: u32, height: u32, field: &FieldUpdate) {
    if field.velocity_x.len() != field.nx * field.ny
        || field.velocity_y.len() != field.nx * field.ny
    {
        return;
    }
    let max_speed = field
        .velocity_x
        .iter()
        .zip(&field.velocity_y)
        .map(|(u, v)| u.hypot(*v))
        .filter(|speed| speed.is_finite())
        .fold(0.0_f64, f64::max);
    if max_speed <= f64::EPSILON {
        return;
    }
    let stride_x = (field.nx / 22).max(1);
    let stride_y = (field.ny / 14).max(1);
    let arrow_scale = 0.42
        * ((stride_x * width as usize / field.nx).min(stride_y * height as usize / field.ny))
            as f64;
    for j in (0..field.ny).step_by(stride_y) {
        for i in (0..field.nx).step_by(stride_x) {
            let index = i + field.nx * j;
            if field.solid.get(index).copied().unwrap_or(true) {
                continue;
            }
            let u = field.velocity_x[index];
            let v = field.velocity_y[index];
            let magnitude = u.hypot(v);
            if !magnitude.is_finite() || magnitude <= max_speed * 0.015 {
                continue;
            }
            let start = (
                (i as f64 + 0.5) * width as f64 / field.nx as f64,
                height as f64 - (j as f64 + 0.5) * height as f64 / field.ny as f64,
            );
            let length = arrow_scale * (magnitude / max_speed).sqrt();
            let end = (
                start.0 + length * u / magnitude,
                start.1 - length * v / magnitude,
            );
            let color = [245, 248, 250, 255];
            draw_line(
                pixels,
                width,
                height,
                (start.0 as i32, start.1 as i32),
                (end.0 as i32, end.1 as i32),
                color,
            );
            let angle = (start.1 - end.1).atan2(end.0 - start.0);
            for offset in [-2.6_f64, 2.6] {
                let head = (
                    end.0 - 5.0 * (angle + offset).cos(),
                    end.1 + 5.0 * (angle + offset).sin(),
                );
                draw_line(
                    pixels,
                    width,
                    height,
                    (end.0 as i32, end.1 as i32),
                    (head.0 as i32, head.1 as i32),
                    color,
                );
            }
        }
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

pub(super) fn render_residual_chart(history: &VecDeque<ResidualSample>) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [46, 52, 64, 255]);
    for divisor in 1..5 {
        let y = height * divisor / 5;
        for x in 0..width {
            set_pixel(&mut pixels, width, height, x, y, [29, 48, 61, 255]);
        }
    }
    if history.len() > 1 {
        draw_series(
            &mut pixels,
            width,
            height,
            history,
            |sample| sample.continuity,
            [77, 168, 184, 255],
        );
        draw_series(
            &mut pixels,
            width,
            height,
            history,
            |sample| sample.momentum,
            [116, 189, 135, 255],
        );
        draw_series(
            &mut pixels,
            width,
            height,
            history,
            |sample| sample.pressure,
            [240, 195, 109, 255],
        );
    }
    image_from_rgba(width, height, pixels)
}

pub(super) fn draw_series(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    history: &VecDeque<ResidualSample>,
    value: impl Fn(&ResidualSample) -> f64,
    color: [u8; 4],
) {
    let count = history.len() - 1;
    let mut previous = None;
    for (index, sample) in history.iter().enumerate() {
        let x = (index as u32 * (width - 1) / count as u32) as i32;
        let level = residual_level(value(sample));
        let y = ((1.0 - level) * (height - 1) as f32) as i32;
        if let Some((old_x, old_y)) = previous {
            draw_line(pixels, width, height, (old_x, old_y), (x, y), color);
        }
        previous = Some((x, y));
    }
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
    fn canonical_cad_triad_is_drawn_in_the_viewport_corner_even_without_geometry() {
        let mut pixels = vec![0_u8; (PREVIEW_WIDTH * PREVIEW_HEIGHT * 4) as usize];
        draw_canonical_orientation_triad(&mut pixels, PREVIEW_WIDTH, PREVIEW_HEIGHT, 0.65, 0.48);
        let endpoints = triad_endpoints(0.65, 0.48);
        for (axis, endpoint) in endpoints.into_iter().enumerate() {
            let index = ((endpoint.1 as u32 * PREVIEW_WIDTH + endpoint.0 as u32) * 4) as usize;
            assert_eq!(&pixels[index..index + 4], &TRIAD_COLORS[axis]);
            assert_eq!(
                pick_canonical_orientation_axis(
                    0.65,
                    0.48,
                    (f64::from(endpoint.0), f64::from(endpoint.1))
                ),
                Some(axis as i32)
            );
        }
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
                assert!(pixels
                    .chunks_exact(4)
                    .any(|pixel| pixel != [9, 16, 22, 255]));
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
