use crate::field::{Field2D, Mask2D};
use crate::grid::UniformGrid2D;
use std::fs::{create_dir_all, File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::Path;

pub fn ensure_output_tree(root: &Path) -> Result<(), String> {
    create_dir_all(root).map_err(|e| format!("Cannot create {}: {e}", root.display()))?;
    create_dir_all(root.join("frames"))
        .map_err(|e| format!("Cannot create frames directory: {e}"))?;
    Ok(())
}

pub fn write_field_csv(
    path: &Path,
    grid: &UniformGrid2D,
    solid: &Mask2D,
    p: &Field2D,
    u: &Field2D,
    v: &Field2D,
    vorticity: &Field2D,
) -> Result<(), String> {
    let file = File::create(path).map_err(|e| format!("Cannot create {}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    writeln!(w, "i,j,x,y,solid,p,u,v,speed,vorticity").map_err(io_err)?;
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            let speed = (u[(i, j)].powi(2) + v[(i, j)].powi(2)).sqrt();
            writeln!(
                w,
                "{i},{j},{:.12},{:.12},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e}",
                grid.cell_x(i),
                grid.cell_y(j),
                if solid[(i, j)] { 1 } else { 0 },
                p[(i, j)],
                u[(i, j)],
                v[(i, j)],
                speed,
                vorticity[(i, j)]
            )
            .map_err(io_err)?;
        }
    }
    Ok(())
}

pub fn write_legacy_vtk(
    path: &Path,
    title: &str,
    grid: &UniformGrid2D,
    solid: &Mask2D,
    p: &Field2D,
    u: &Field2D,
    v: &Field2D,
    vorticity: &Field2D,
) -> Result<(), String> {
    let file = File::create(path).map_err(|e| format!("Cannot create {}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    writeln!(w, "# vtk DataFile Version 3.0").map_err(io_err)?;
    writeln!(w, "{title}").map_err(io_err)?;
    writeln!(w, "ASCII").map_err(io_err)?;
    writeln!(w, "DATASET STRUCTURED_POINTS").map_err(io_err)?;
    writeln!(w, "DIMENSIONS {} {} 1", grid.nx, grid.ny).map_err(io_err)?;
    writeln!(w, "ORIGIN {:.12} {:.12} 0", 0.5 * grid.dx, 0.5 * grid.dy).map_err(io_err)?;
    writeln!(w, "SPACING {:.12} {:.12} 1", grid.dx, grid.dy).map_err(io_err)?;
    writeln!(w, "POINT_DATA {}", grid.nx * grid.ny).map_err(io_err)?;

    writeln!(w, "SCALARS pressure double 1").map_err(io_err)?;
    writeln!(w, "LOOKUP_TABLE default").map_err(io_err)?;
    write_scalar_field(&mut w, p)?;

    writeln!(w, "SCALARS speed double 1").map_err(io_err)?;
    writeln!(w, "LOOKUP_TABLE default").map_err(io_err)?;
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            let speed = (u[(i, j)].powi(2) + v[(i, j)].powi(2)).sqrt();
            writeln!(w, "{speed:.12e}").map_err(io_err)?;
        }
    }

    writeln!(w, "SCALARS vorticity double 1").map_err(io_err)?;
    writeln!(w, "LOOKUP_TABLE default").map_err(io_err)?;
    write_scalar_field(&mut w, vorticity)?;

    writeln!(w, "SCALARS solid int 1").map_err(io_err)?;
    writeln!(w, "LOOKUP_TABLE default").map_err(io_err)?;
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            writeln!(w, "{}", if solid[(i, j)] { 1 } else { 0 }).map_err(io_err)?;
        }
    }

    writeln!(w, "VECTORS velocity double").map_err(io_err)?;
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            writeln!(w, "{:.12e} {:.12e} 0", u[(i, j)], v[(i, j)]).map_err(io_err)?;
        }
    }
    Ok(())
}

fn write_scalar_field<W: Write>(w: &mut W, f: &Field2D) -> Result<(), String> {
    for j in 0..f.ny() {
        for i in 0..f.nx() {
            writeln!(w, "{:.12e}", f[(i, j)]).map_err(io_err)?;
        }
    }
    Ok(())
}

pub fn append_history(path: &Path, header: &str, row: &str) -> Result<(), String> {
    let new_file = !path.exists();
    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .map_err(|e| format!("Cannot open {}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    if new_file {
        writeln!(w, "{header}").map_err(io_err)?;
    }
    writeln!(w, "{row}").map_err(io_err)?;
    Ok(())
}

pub fn write_ppm_frame(
    path: &Path,
    grid: &UniformGrid2D,
    solid: &Mask2D,
    field: &Field2D,
    symmetric: bool,
) -> Result<(), String> {
    let file = File::create(path).map_err(|e| format!("Cannot create {}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    writeln!(w, "P6\n{} {}\n255", grid.nx, grid.ny).map_err(io_err)?;

    let mut min_v = f64::INFINITY;
    let mut max_v = f64::NEG_INFINITY;
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            if !solid[(i, j)] {
                let value = field[(i, j)];
                if value.is_finite() {
                    min_v = min_v.min(value);
                    max_v = max_v.max(value);
                }
            }
        }
    }
    if symmetric {
        let bound = min_v.abs().max(max_v.abs()).max(1.0e-14);
        min_v = -bound;
        max_v = bound;
    } else if !(max_v > min_v) {
        max_v = min_v + 1.0;
    }

    // Image rows are written top-to-bottom.
    for j_img in 0..grid.ny {
        let j = grid.ny - 1 - j_img;
        for i in 0..grid.nx {
            let rgb = if solid[(i, j)] {
                [20_u8, 20_u8, 20_u8]
            } else {
                let t = ((field[(i, j)] - min_v) / (max_v - min_v)).clamp(0.0, 1.0);
                if symmetric {
                    diverging_color(t)
                } else {
                    sequential_color(t)
                }
            };
            w.write_all(&rgb).map_err(io_err)?;
        }
    }
    Ok(())
}

fn sequential_color(t: f64) -> [u8; 3] {
    let r = (255.0 * smoothstep(0.45, 1.0, t)) as u8;
    let g = (255.0 * smoothstep(0.05, 0.85, t)) as u8;
    let b = (255.0 * (1.0 - smoothstep(0.0, 0.7, t))) as u8;
    [r, g, b]
}

fn diverging_color(t: f64) -> [u8; 3] {
    if t < 0.5 {
        let a = t / 0.5;
        [
            (240.0 * a) as u8,
            (245.0 * a) as u8,
            (130.0 + 125.0 * a) as u8,
        ]
    } else {
        let a = (t - 0.5) / 0.5;
        [255, (245.0 * (1.0 - a)) as u8, (255.0 * (1.0 - a)) as u8]
    }
}

fn smoothstep(edge0: f64, edge1: f64, x: f64) -> f64 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn io_err(e: std::io::Error) -> String {
    e.to_string()
}
