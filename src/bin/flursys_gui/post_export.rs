//! Atomic image/data exports. GIF uses one palette for the whole loop so
//! stationary field colours cannot flicker as tracer positions change.
use super::*;
use std::io::Write;

pub(super) fn save_png(path: &Path, image: &slint::Image) -> Result<(), String> {
    let pixels = image.to_rgba8().ok_or("No rendered image available")?;
    let temp = tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
        .map_err(|e| e.to_string())?;
    let file = temp.reopen().map_err(|e| e.to_string())?;
    let mut encoder = png::Encoder::new(
        std::io::BufWriter::new(file),
        pixels.width(),
        pixels.height(),
    );
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(|e| e.to_string())?;
    writer
        .write_image_data(pixels.as_bytes())
        .map_err(|e| e.to_string())?;
    writer.finish().map_err(|e| e.to_string())?;
    temp.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
pub(super) fn write_probes(path: &Path, probes: &[ResultProbe]) -> Result<(), String> {
    let temp = tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
        .map_err(|e| e.to_string())?;
    let mut file = std::io::BufWriter::new(temp.reopen().map_err(|e| e.to_string())?);
    writeln!(file, "cell,x,y,z,p_over_rho,ux,uy,uz,speed").map_err(|e| e.to_string())?;
    for p in probes {
        writeln!(
            file,
            "{},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}",
            p.cell_index,
            p.center.x,
            p.center.y,
            p.center.z,
            p.pressure,
            p.velocity.x,
            p.velocity.y,
            p.velocity.z,
            p.velocity_magnitude
        )
        .map_err(|e| e.to_string())?;
    }
    file.flush().map_err(|e| e.to_string())?;
    drop(file);
    temp.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
pub(super) fn save_gif(
    path: &Path,
    w: u32,
    h: u32,
    pixels: &[u8],
    traces: &[TimedPath],
    view: &ViewTransform,
) -> Result<(), String> {
    if w == 0
        || h == 0
        || w > u16::MAX as u32
        || h > u16::MAX as u32
        || pixels.len() != w as usize * h as usize * 4
    {
        return Err("Invalid GIF dimensions or pixel buffer".into());
    }
    let mut base = pixels.to_vec();
    let quantized = gif::Frame::from_rgba_speed(w as u16, h as u16, &mut base, 10);
    let transparent = quantized.transparent;
    let mut palette = quantized
        .palette
        .ok_or("GIF quantizer returned no palette")?;
    let marker = [255u8, 248, 207];
    let marker_index = if let Some(i) = palette
        .chunks_exact(3)
        .enumerate()
        .position(|(i, p)| p == marker && Some(i as u8) != transparent)
    {
        i as u8
    } else if palette.len() / 3 < 256 {
        let i = (palette.len() / 3) as u8;
        palette.extend(marker);
        i
    } else {
        let i = if transparent == Some(255) {
            254usize
        } else {
            255
        };
        palette[i * 3..i * 3 + 3].copy_from_slice(&marker);
        i as u8
    };
    let mut lookup = std::collections::HashMap::<[u8; 4], u8>::new();
    let mut background = Vec::with_capacity(w as usize * h as usize);
    for rgba in pixels.chunks_exact(4) {
        let key = [rgba[0], rgba[1], rgba[2], rgba[3]];
        let index = *lookup.entry(key).or_insert_with(|| {
            if key[3] < 128 {
                if let Some(index) = transparent {
                    return index;
                }
            }
            palette
                .chunks_exact(3)
                .enumerate()
                .filter(|(i, _)| Some(*i as u8) != transparent)
                .min_by_key(|(_, p)| {
                    (0..3)
                        .map(|j| (i32::from(p[j]) - i32::from(key[j])).pow(2))
                        .sum::<i32>()
                })
                .map_or(0, |(i, _)| i as u8)
        });
        background.push(index);
    }
    let temp = tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
        .map_err(|e| e.to_string())?;
    let file = temp.reopen().map_err(|e| e.to_string())?;
    let mut encoder =
        gif::Encoder::new(std::io::BufWriter::new(file), w as u16, h as u16, &palette)
            .map_err(|e| e.to_string())?;
    encoder
        .set_repeat(gif::Repeat::Infinite)
        .map_err(|e| e.to_string())?;
    for frame in 0..60 {
        let mut indices = background.clone();
        for (x, y) in markers(traces, frame as f64 / 60.0) {
            let (x, y) = view.world_to_screen((x, y));
            let (x, y) = (x as i32, y as i32);
            for dy in -2..=2 {
                for dx in -2..=2 {
                    let (px, py) = (x + dx, y + dy);
                    if px >= 0 && py >= 0 && px < w as i32 && py < h as i32 {
                        indices[py as usize * w as usize + px as usize] = marker_index;
                    }
                }
            }
        }
        let frame = gif::Frame {
            width: w as u16,
            height: h as u16,
            buffer: std::borrow::Cow::Owned(indices),
            delay: 5,
            transparent,
            dispose: gif::DisposalMethod::Background,
            ..Default::default()
        };
        encoder.write_frame(&frame).map_err(|e| e.to_string())?;
    }
    let mut writer = encoder.into_inner().map_err(|e| e.to_string())?;
    writer.flush().map_err(|e| e.to_string())?;
    drop(writer);
    temp.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
