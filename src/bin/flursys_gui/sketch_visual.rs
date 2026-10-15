//! Pixel coverage for readable sketch edges without changing model geometry.
pub(super) fn line(
    pixels: &mut [u8],
    width: u32,
    height: u32,
    a: (i32, i32),
    b: (i32, i32),
    color: [u8; 4],
) {
    let (ax, ay, bx, by) = (a.0 as f64, a.1 as f64, b.0 as f64, b.1 as f64);
    let (dx, dy) = (bx - ax, by - ay);
    let len = dx * dx + dy * dy;
    let min_x = (a.0.min(b.0) as i64 - 2).clamp(0, width as i64) as u32;
    let max_x = (a.0.max(b.0) as i64 + 2).clamp(0, width as i64) as u32;
    let min_y = (a.1.min(b.1) as i64 - 2).clamp(0, height as i64) as u32;
    let max_y = (a.1.max(b.1) as i64 + 2).clamp(0, height as i64) as u32;
    let horizontal = dx.abs() >= dy.abs();
    let (major_min, major_max) = if horizontal {
        (min_x, max_x)
    } else {
        (min_y, max_y)
    };
    for major in major_min..major_max {
        let t = if horizontal && dx.abs() > 1e-9 {
            (major as f64 + 0.5 - ax) / dx
        } else if !horizontal && dy.abs() > 1e-9 {
            (major as f64 + 0.5 - ay) / dy
        } else {
            0.0
        };
        let minor = if horizontal { ay + t * dy } else { ax + t * dx };
        let minor_limit = if horizontal { height } else { width };
        let first = ((minor.floor() as i64) - 3).clamp(0, minor_limit as i64) as u32;
        let last = ((minor.ceil() as i64) + 3).clamp(0, minor_limit as i64) as u32;
        for m in first..last {
            let (x, y) = if horizontal { (major, m) } else { (m, major) };
            let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
            let t = if len > 0.0 {
                ((px - ax) * dx + (py - ay) * dy) / len
            } else {
                0.0
            }
            .clamp(0.0, 1.0);
            let coverage = (1.65 - (px - ax - t * dx).hypot(py - ay - t * dy)).clamp(0.0, 1.0);
            let index = ((y * width + x) * 4) as usize;
            for c in 0..3 {
                pixels[index + c] = (pixels[index + c] as f64 * (1.0 - coverage)
                    + color[c] as f64 * coverage)
                    .round() as u8;
            }
        }
    }
}
