//! Scalar sections through the actual volume mesh, with no surface projection.
use super::*;

pub(crate) fn render_result_slice(
    dataset: &flursys::ResultDataset,
    field: ResultFieldKind,
    plane: i32,
    percent: i32,
    colormap: i32,
    display_range: Option<(f64, f64)>,
) -> Image {
    let axis = match plane {
        1 => 2,
        2 => 1,
        _ => 0,
    };
    let axes = match axis {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    let xyz = |p: Vec3| [p.x, p.y, p.z];
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for point in dataset.mesh().points() {
        let p = xyz(point.position);
        for i in 0..3 {
            lo[i] = lo[i].min(p[i]);
            hi[i] = hi[i].max(p[i]);
        }
    }
    let coordinate = lo[axis] + (hi[axis] - lo[axis]) * f64::from(percent.clamp(1, 99)) / 100.0;
    let Ok(polygons) = dataset.plane_slice(axis, coordinate) else {
        return render_empty_image();
    };
    let (width, height) = (PREVIEW_WIDTH, PREVIEW_HEIGHT);
    let span_x = (hi[axes.0] - lo[axes.0]).max(1e-12);
    let span_y = (hi[axes.1] - lo[axes.1]).max(1e-12);
    let scale = (f64::from(width) * 0.85 / span_x).min(f64::from(height) * 0.8 / span_y);
    let (minimum, maximum) = display_range
        .or_else(|| dataset.scalar_range(field).ok())
        .unwrap_or((0.0, 1.0));
    let view = ViewTransform {
        center_x: (lo[axes.0] + hi[axes.0]) * 0.5,
        center_y: (lo[axes.1] + hi[axes.1]) * 0.5,
        pixels_per_unit: scale,
        viewport_width: f64::from(width),
        viewport_height: f64::from(height),
    };
    let mut scene = super::super::scene_view::SceneView::planar(&view, span_x.hypot(span_y) * 0.5);
    let values = dataset.scalar_values(field);
    for (cell, polygon) in polygons {
        let Some(&value) = values.get(cell) else {
            continue;
        };
        let points = polygon
            .iter()
            .map(|p| {
                let v = xyz(*p);
                Vec3::new(v[axes.0], v[axes.1], 0.0)
            })
            .collect::<Vec<_>>();
        scene.polygon(
            &points,
            &[],
            scalar_color(normalize(value, minimum, maximum), colormap),
        );
    }
    let mut pixels = scene.pixels(width, height, true, false);
    draw_colormap_legend(&mut pixels, width, height, colormap);
    image_from_rgba(width, height, pixels)
}

pub(crate) fn pick_result_slice_cell(
    dataset: &flursys::ResultDataset,
    plane: i32,
    percent: i32,
    screen: (f64, f64),
) -> Option<usize> {
    let axis = match plane {
        1 => 2,
        2 => 1,
        _ => 0,
    };
    let axes = match axis {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for point in dataset.mesh().points() {
        let p = point.position;
        let v = [p.x, p.y, p.z];
        for i in 0..3 {
            lo[i] = lo[i].min(v[i]);
            hi[i] = hi[i].max(v[i]);
        }
    }
    let coordinate = lo[axis] + (hi[axis] - lo[axis]) * f64::from(percent.clamp(1, 99)) / 100.0;
    let scale = (f64::from(PREVIEW_WIDTH) * 0.85 / (hi[axes.0] - lo[axes.0]).max(1e-12))
        .min(f64::from(PREVIEW_HEIGHT) * 0.8 / (hi[axes.1] - lo[axes.1]).max(1e-12));
    let world = (
        (screen.0 - f64::from(PREVIEW_WIDTH) * 0.5) / scale + (lo[axes.0] + hi[axes.0]) * 0.5,
        (f64::from(PREVIEW_HEIGHT) * 0.5 - screen.1) / scale + (lo[axes.1] + hi[axes.1]) * 0.5,
    );
    for (cell, poly) in dataset.plane_slice(axis, coordinate).ok()? {
        let points = poly
            .iter()
            .map(|p| {
                let v = [p.x, p.y, p.z];
                (v[axes.0], v[axes.1])
            })
            .collect::<Vec<_>>();
        let mut inside = false;
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            if (a.1 > world.1) != (b.1 > world.1)
                && world.0 < (b.0 - a.0) * (world.1 - a.1) / (b.1 - a.1) + a.0
            {
                inside = !inside;
            }
        }
        if inside {
            return Some(cell);
        }
    }
    None
}
