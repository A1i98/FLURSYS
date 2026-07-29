use super::{residual_level, ResidualSample, PREVIEW_HEIGHT, PREVIEW_WIDTH};
use flursys::{
    BoundaryConditionKind, BoundaryFace, EnergyModel, ExtrudedMesh3D, FieldUpdate, GeometryPart,
    GeometryPartKind, Project, ProjectCase, StructuredMesh2D,
};
use slint::{Image, Rgba8Pixel, SharedPixelBuffer};
use std::collections::VecDeque;

type MeshNode = (usize, usize, usize);
type BoundaryFaceNodes = (BoundaryFace, [MeshNode; 4]);

pub(super) fn render_empty_image() -> Image {
    let mut pixels = vec![0_u8; (PREVIEW_WIDTH * PREVIEW_HEIGHT * 4) as usize];
    fill(&mut pixels, [11, 18, 24, 255]);
    image_from_rgba(PREVIEW_WIDTH, PREVIEW_HEIGHT, pixels)
}

pub(super) fn render_mesh(project: &Project, selected_boundary: Option<BoundaryFace>) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [11, 18, 24, 255]);
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
        | ProjectCase::BackwardFacingStep { length, height, .. } => (*length, *height),
    }
}

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
        ProjectCase::LidDrivenCavity { .. } => false,
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
    fill(&mut pixels, [11, 18, 24, 255]);
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
    // Parametric workbench parts remain an overlay until they can be converted
    // into conformal CFD cells by a CAD/meshing kernel.
    let part_scale = geometry_scene_scale(&project.preprocessing.geometry.parts) * f64::from(zoom);
    for (index, part) in project.preprocessing.geometry.parts.iter().enumerate() {
        let color = part_color(index);
        match &part.kind {
            GeometryPartKind::Box {
                length,
                width: part_width,
                height: part_height,
            } => draw_part_box(
                &mut pixels,
                width,
                height,
                part,
                *length,
                *part_width,
                *part_height,
                yaw,
                part_scale,
                color,
            ),
            GeometryPartKind::Cylinder {
                radius,
                height: part_height,
                ..
            } => draw_part_cylinder(
                &mut pixels,
                width,
                height,
                part,
                *radius,
                *part_height,
                yaw,
                part_scale,
                color,
            ),
        }
    }
    image_from_rgba(width, height, pixels)
}

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
    yaw: f32,
    pitch: f32,
    zoom: f32,
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
    let camera = MeshCamera::fit(&mesh, yaw, pitch, zoom);
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
        ProjectCase::LidDrivenCavity { .. } => {}
    }
}

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

#[derive(Clone, Copy)]
pub(super) struct MeshCamera {
    length: f64,
    height: f64,
    depth: f64,
    yaw: f64,
    pitch: f64,
    scale: f64,
    z_exaggeration: f64,
}

impl MeshCamera {
    fn fit(mesh: &ExtrudedMesh3D, yaw: f32, pitch: f32, zoom: f32) -> Self {
        let mut camera = Self {
            length: mesh.base.length,
            height: mesh.base.height,
            depth: mesh.depth,
            yaw: f64::from(yaw),
            pitch: f64::from(pitch),
            scale: 1.0,
            z_exaggeration: preview_z_exaggeration(mesh),
        };
        let corners = [
            (0.0, 0.0, 0.0),
            (mesh.base.length, 0.0, 0.0),
            (0.0, mesh.base.height, 0.0),
            (mesh.base.length, mesh.base.height, 0.0),
            (0.0, 0.0, mesh.depth),
            (mesh.base.length, 0.0, mesh.depth),
            (0.0, mesh.base.height, mesh.depth),
            (mesh.base.length, mesh.base.height, mesh.depth),
        ];
        let projected: Vec<_> = corners
            .into_iter()
            .map(|point| camera.project_unscaled(point))
            .collect();
        let (min_x, max_x) = projected
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &(x, _)| {
                (min.min(x), max.max(x))
            });
        let (min_y, max_y) = projected
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), &(_, y)| {
                (min.min(y), max.max(y))
            });
        camera.scale = (460.0 / (max_x - min_x).max(1.0e-9))
            .min(280.0 / (max_y - min_y).max(1.0e-9))
            * f64::from(zoom);
        camera
    }

    fn project_unscaled(&self, (x, y, z): (f64, f64, f64)) -> (f64, f64) {
        let x = x - 0.5 * self.length;
        let y = y - 0.5 * self.height;
        let z = (z - 0.5 * self.depth) * self.z_exaggeration;
        let horizontal = x * self.yaw.cos() - y * self.yaw.sin();
        let depth = x * self.yaw.sin() + y * self.yaw.cos();
        (horizontal, z * self.pitch.cos() + depth * self.pitch.sin())
    }

    fn project(&self, point: (f64, f64, f64)) -> (i32, i32) {
        let (x, y) = self.project_unscaled(point);
        (
            (260.0 + x * self.scale) as i32,
            (160.0 - y * self.scale) as i32,
        )
    }
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
    fill(&mut pixels, [11, 18, 24, 255]);

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
            ProjectCase::LidDrivenCavity { .. } => {}
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
            GeometryPartKind::Cylinder { radius, height, .. } => radius.max(0.5 * height),
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

pub(super) fn render_scalar_field(field: &FieldUpdate, values: &[f64], symmetric: bool) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
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
    } else if !min_value.is_finite() || max_value <= min_value {
        min_value = 0.0;
        max_value = 1.0;
    }
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    for y in 0..height {
        let j = ((height - 1 - y) as usize * field.ny / height as usize).min(field.ny - 1);
        for x in 0..width {
            let i = (x as usize * field.nx / width as usize).min(field.nx - 1);
            let index = i + field.nx * j;
            let color = if field.solid[index] {
                [22, 25, 29, 255]
            } else {
                let normalized =
                    ((values[index] - min_value) / (max_value - min_value)).clamp(0.0, 1.0) as f32;
                if symmetric {
                    diverging_color(normalized)
                } else {
                    speed_color(normalized)
                }
            };
            set_pixel(&mut pixels, width, height, x, y, color);
        }
    }
    image_from_rgba(width, height, pixels)
}

pub(super) fn render_speed_field(field: &FieldUpdate) -> Image {
    render_scalar_field(field, &field.speed, false)
}

pub(super) fn render_residual_chart(history: &VecDeque<ResidualSample>) -> Image {
    let width = PREVIEW_WIDTH;
    let height = PREVIEW_HEIGHT;
    let mut pixels = vec![0_u8; (width * height * 4) as usize];
    fill(&mut pixels, [11, 18, 24, 255]);
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

pub(super) fn speed_color(value: f32) -> [u8; 4] {
    let r = (255.0 * value.sqrt()) as u8;
    let g = (255.0 * (1.0 - (2.0 * value - 1.0).abs())) as u8;
    let b = (255.0 * (1.0 - value).sqrt()) as u8;
    [r, g, b, 255]
}

pub(super) fn diverging_color(value: f32) -> [u8; 4] {
    if value < 0.5 {
        let blend = value * 2.0;
        [
            (64.0 + 191.0 * blend) as u8,
            (112.0 + 133.0 * blend) as u8,
            255,
            255,
        ]
    } else {
        let blend = (value - 0.5) * 2.0;
        [
            255,
            (245.0 * (1.0 - blend)) as u8,
            (255.0 * (1.0 - blend)) as u8,
            255,
        ]
    }
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
