use crate::field::{Field2D, Field3D, Mask2D};
use crate::grid::{UniformGrid2D, UniformGrid3D};
use crate::{IncompressibleSolution, MeshDimension, UnstructuredMesh};
use std::fs::{create_dir_all, File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::Path;

pub struct VtkFields<'a> {
    pub pressure: &'a Field2D,
    pub u: &'a Field2D,
    pub v: &'a Field2D,
    pub vorticity: &'a Field2D,
    pub temperature: Option<&'a Field2D>,
}

pub struct CsvFields<'a> {
    pub pressure: &'a Field2D,
    pub u: &'a Field2D,
    pub v: &'a Field2D,
    pub vorticity: &'a Field2D,
    pub temperature: Option<&'a Field2D>,
}

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
    fields: CsvFields<'_>,
) -> Result<(), String> {
    let file = File::create(path).map_err(|e| format!("Cannot create {}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    if fields.temperature.is_some() {
        writeln!(w, "i,j,x,y,solid,p,u,v,speed,vorticity,temperature").map_err(io_err)?;
    } else {
        writeln!(w, "i,j,x,y,solid,p,u,v,speed,vorticity").map_err(io_err)?;
    }
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            let speed = (fields.u[(i, j)].powi(2) + fields.v[(i, j)].powi(2)).sqrt();
            write!(
                w,
                "{i},{j},{:.12},{:.12},{},{:.12e},{:.12e},{:.12e},{:.12e},{:.12e}",
                grid.cell_x(i),
                grid.cell_y(j),
                if solid[(i, j)] { 1 } else { 0 },
                fields.pressure[(i, j)],
                fields.u[(i, j)],
                fields.v[(i, j)],
                speed,
                fields.vorticity[(i, j)]
            )
            .map_err(io_err)?;
            if let Some(temperature) = fields.temperature {
                write!(w, ",{:.12e}", temperature[(i, j)]).map_err(io_err)?;
            }
            writeln!(w).map_err(io_err)?;
        }
    }
    Ok(())
}

pub fn write_legacy_vtk(
    path: &Path,
    title: &str,
    grid: &UniformGrid2D,
    solid: &Mask2D,
    fields: VtkFields<'_>,
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
    write_scalar_field(&mut w, fields.pressure)?;

    writeln!(w, "SCALARS speed double 1").map_err(io_err)?;
    writeln!(w, "LOOKUP_TABLE default").map_err(io_err)?;
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            let speed = (fields.u[(i, j)].powi(2) + fields.v[(i, j)].powi(2)).sqrt();
            writeln!(w, "{speed:.12e}").map_err(io_err)?;
        }
    }

    writeln!(w, "SCALARS vorticity double 1").map_err(io_err)?;
    writeln!(w, "LOOKUP_TABLE default").map_err(io_err)?;
    write_scalar_field(&mut w, fields.vorticity)?;

    if let Some(temperature) = fields.temperature {
        writeln!(w, "SCALARS temperature double 1").map_err(io_err)?;
        writeln!(w, "LOOKUP_TABLE default").map_err(io_err)?;
        write_scalar_field(&mut w, temperature)?;
    }

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
            writeln!(w, "{:.12e} {:.12e} 0", fields.u[(i, j)], fields.v[(i, j)]).map_err(io_err)?;
        }
    }
    Ok(())
}

pub fn write_legacy_vtk_3d(
    path: &Path,
    title: &str,
    grid: &UniformGrid3D,
    pressure: &Field3D,
    u: &Field3D,
    v: &Field3D,
    w: &Field3D,
) -> Result<(), String> {
    let file = File::create(path).map_err(|e| format!("Cannot create {}: {e}", path.display()))?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "# vtk DataFile Version 3.0").map_err(io_err)?;
    writeln!(writer, "{title}").map_err(io_err)?;
    writeln!(writer, "ASCII").map_err(io_err)?;
    writeln!(writer, "DATASET STRUCTURED_POINTS").map_err(io_err)?;
    writeln!(writer, "DIMENSIONS {} {} {}", grid.nx, grid.ny, grid.nz).map_err(io_err)?;
    writeln!(
        writer,
        "ORIGIN {:.12} {:.12} {:.12}",
        0.5 * grid.dx,
        0.5 * grid.dy,
        0.5 * grid.dz
    )
    .map_err(io_err)?;
    writeln!(
        writer,
        "SPACING {:.12} {:.12} {:.12}",
        grid.dx, grid.dy, grid.dz
    )
    .map_err(io_err)?;
    writeln!(writer, "POINT_DATA {}", grid.nx * grid.ny * grid.nz).map_err(io_err)?;
    writeln!(writer, "SCALARS pressure double 1").map_err(io_err)?;
    writeln!(writer, "LOOKUP_TABLE default").map_err(io_err)?;
    for value in pressure.as_slice() {
        writeln!(writer, "{value:.12e}").map_err(io_err)?;
    }
    writeln!(writer, "VECTORS velocity double").map_err(io_err)?;
    for k in 0..grid.nz {
        for j in 0..grid.ny {
            for i in 0..grid.nx {
                let uc = 0.5 * (u[(i, j, k)] + u[(i + 1, j, k)]);
                let vc = 0.5 * (v[(i, j, k)] + v[(i, j + 1, k)]);
                let wc = 0.5 * (w[(i, j, k)] + w[(i, j, k + 1)]);
                writeln!(writer, "{uc:.12e} {vc:.12e} {wc:.12e}").map_err(io_err)?;
            }
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
    if !min_v.is_finite() || !max_v.is_finite() {
        min_v = 0.0;
        max_v = 1.0;
    } else if symmetric {
        let bound = min_v.abs().max(max_v.abs()).max(1.0e-14);
        min_v = -bound;
        max_v = bound;
    } else if max_v <= min_v {
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

/// Writes a 2D unstructured incompressible result as legacy ASCII VTK with
/// cell-centred `pressure`, `velocity_magnitude`, and `velocity` fields.
pub fn write_unstructured_legacy_vtk(
    path: &Path,
    title: &str,
    mesh: &UnstructuredMesh,
    solution: &IncompressibleSolution,
) -> Result<(), String> {
    solution
        .velocity
        .ensure_mesh(mesh)
        .map_err(|error| format!("velocity field: {error:?}"))?;
    solution
        .pressure
        .ensure_mesh(mesh)
        .map_err(|error| format!("pressure field: {error:?}"))?;
    if mesh.dimension() != MeshDimension::TwoD {
        return Err("legacy unstructured VTK export currently supports 2D polygon cells".into());
    }
    let cells = (0..mesh.cell_count())
        .map(|cell| polygon_vertices(mesh, cell))
        .collect::<Result<Vec<_>, _>>()?;
    let file =
        File::create(path).map_err(|error| format!("cannot create {}: {error}", path.display()))?;
    let mut writer = BufWriter::new(file);
    writeln!(writer, "# vtk DataFile Version 3.0").map_err(io_err)?;
    writeln!(writer, "{title}\nASCII\nDATASET UNSTRUCTURED_GRID").map_err(io_err)?;
    writeln!(writer, "POINTS {} double", mesh.points().len()).map_err(io_err)?;
    for point in mesh.points() {
        writeln!(
            writer,
            "{:.12e} {:.12e} {:.12e}",
            point.position.x, point.position.y, point.position.z
        )
        .map_err(io_err)?;
    }
    let size: usize = cells.iter().map(|cell| 1 + cell.len()).sum();
    writeln!(writer, "CELLS {} {size}", cells.len()).map_err(io_err)?;
    for cell in &cells {
        write!(writer, "{}", cell.len()).map_err(io_err)?;
        for vertex in cell {
            write!(writer, " {vertex}").map_err(io_err)?;
        }
        writeln!(writer).map_err(io_err)?;
    }
    writeln!(writer, "CELL_TYPES {}", cells.len()).map_err(io_err)?;
    for _ in &cells {
        writeln!(writer, "7").map_err(io_err)?;
    }
    writeln!(writer, "CELL_DATA {}", mesh.cell_count()).map_err(io_err)?;
    write_scalar_cell_field(&mut writer, "pressure", solution.pressure.values())?;
    write_scalar_cell_field(&mut writer, "velocity_magnitude", solution.speed().values())?;
    writeln!(writer, "VECTORS velocity double").map_err(io_err)?;
    for value in solution.velocity.values() {
        writeln!(writer, "{:.12e} {:.12e} {:.12e}", value.x, value.y, value.z).map_err(io_err)?;
    }
    Ok(())
}

fn write_scalar_cell_field(
    writer: &mut BufWriter<File>,
    name: &str,
    values: &[f64],
) -> Result<(), String> {
    writeln!(writer, "SCALARS {name} double 1\nLOOKUP_TABLE default").map_err(io_err)?;
    for value in values {
        writeln!(writer, "{value:.12e}").map_err(io_err)?;
    }
    Ok(())
}

fn polygon_vertices(mesh: &UnstructuredMesh, cell_index: usize) -> Result<Vec<usize>, String> {
    let edges = mesh.cells()[cell_index]
        .faces
        .iter()
        .map(|&face| &mesh.faces()[face].vertices)
        .collect::<Vec<_>>();
    if edges.len() < 3 || edges.iter().any(|edge| edge.len() != 2) {
        return Err(format!("cell {cell_index} is not a polygon"));
    }
    let mut used = vec![false; edges.len()];
    let mut vertices = vec![edges[0][0], edges[0][1]];
    used[0] = true;
    while vertices.len() < edges.len() {
        let current = *vertices.last().expect("polygon starts with an edge");
        let (index, next) = edges
            .iter()
            .enumerate()
            .find_map(|(index, edge)| {
                (!used[index] && edge.contains(&current))
                    .then_some((index, if edge[0] == current { edge[1] } else { edge[0] }))
            })
            .ok_or_else(|| format!("cell {cell_index} does not form one polygon loop"))?;
        used[index] = true;
        vertices.push(next);
    }
    if vertices.last() == Some(&vertices[0]) {
        vertices.pop();
    }
    if vertices.len() != edges.len() {
        return Err(format!("cell {cell_index} has invalid connectivity"));
    }
    Ok(vertices)
}
