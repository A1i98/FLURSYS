//! ASCII VTK XML export for independent compressible solver results.
use super::*;
use crate::MeshDimension;
use std::{io::Write, path::Path};
pub fn write_vtu(
    path: &Path,
    mesh: &UnstructuredMesh,
    s: &Solution,
    gas: IdealGas,
) -> Result<(), String> {
    if s.mesh_id != mesh.id() || s.fields.len() != mesh.cell_count() {
        return Err("Compressible output field length mismatch".into());
    }
    let mut file = tempfile::NamedTempFile::new_in(path.parent().unwrap_or(Path::new(".")))
        .map_err(|e| e.to_string())?;
    {
        let mut out = std::io::BufWriter::new(file.as_file_mut());
        let result = (|| -> std::io::Result<()> {
            writeln!(out,"<?xml version=\"1.0\"?><VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\"><UnstructuredGrid><Piece NumberOfPoints=\"{}\" NumberOfCells=\"{}\">",mesh.points().len(),mesh.cell_count())?;
            writeln!(
                out,
                "<Points><DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">"
            )?;
            for p in mesh.points() {
                writeln!(
                    out,
                    "{:.17e} {:.17e} {:.17e}",
                    p.position.x, p.position.y, p.position.z
                )?;
            }
            writeln!(out, "</DataArray></Points><Cells>")?;
            let mut offsets = vec![];
            let mut offset = 0;
            writeln!(
                out,
                "<DataArray type=\"Int64\" Name=\"connectivity\" format=\"ascii\">"
            )?;
            for c in mesh.cells() {
                let mut ids = c
                    .faces
                    .iter()
                    .flat_map(|f| mesh.faces()[*f].vertices.iter().copied())
                    .collect::<Vec<_>>();
                ids.sort_unstable();
                ids.dedup();
                if mesh.dimension() == MeshDimension::TwoD {
                    ids.sort_by(|a, b| {
                        let a = mesh.points()[*a].position - c.center;
                        let b = mesh.points()[*b].position - c.center;
                        a.y.atan2(a.x).total_cmp(&b.y.atan2(b.x))
                    });
                }
                for id in &ids {
                    write!(out, "{id} ")?;
                }
                writeln!(out)?;
                offset += ids.len();
                offsets.push(offset);
            }
            writeln!(
                out,
                "</DataArray><DataArray type=\"Int64\" Name=\"offsets\" format=\"ascii\">"
            )?;
            for i in offsets {
                write!(out, "{i} ")?;
            }
            writeln!(
                out,
                "</DataArray><DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">"
            )?;
            for _ in mesh.cells() {
                write!(
                    out,
                    "{} ",
                    if mesh.dimension() == MeshDimension::TwoD {
                        7
                    } else {
                        42
                    }
                )?;
            }
            writeln!(out, "</DataArray>")?;
            if mesh.dimension() == MeshDimension::ThreeD {
                let mut offsets = vec![];
                let mut offset = 0;
                writeln!(
                    out,
                    "<DataArray type=\"Int64\" Name=\"faces\" format=\"ascii\">"
                )?;
                for (i, c) in mesh.cells().iter().enumerate() {
                    write!(out, "{} ", c.faces.len())?;
                    offset += 1;
                    for f in &c.faces {
                        let f = &mesh.faces()[*f];
                        write!(out, "{} ", f.vertices.len())?;
                        offset += f.vertices.len() + 1;
                        if f.owner == i {
                            for v in &f.vertices {
                                write!(out, "{v} ")?;
                            }
                        } else {
                            for v in f.vertices.iter().rev() {
                                write!(out, "{v} ")?;
                            }
                        }
                    }
                    writeln!(out)?;
                    offsets.push(offset);
                }
                writeln!(
                    out,
                    "</DataArray><DataArray type=\"Int64\" Name=\"faceoffsets\" format=\"ascii\">"
                )?;
                for i in offsets {
                    write!(out, "{i} ")?;
                }
                writeln!(out, "</DataArray>")?;
            }
            writeln!(
                out,
                "</Cells><CellData Scalars=\"pressure_Pa\" Vectors=\"velocity_m_s\">"
            )?;
            for (name, kind) in [
                ("density_kg_m3", 0),
                ("pressure_Pa", 1),
                ("temperature_K", 2),
                ("Mach", 3),
            ] {
                writeln!(
                    out,
                    "<DataArray type=\"Float64\" Name=\"{name}\" format=\"ascii\">"
                )?;
                for p in &s.fields {
                    let v = match kind {
                        0 => p.density,
                        1 => p.pressure,
                        2 => gas.temperature(*p),
                        _ => p.velocity.norm() / gas.sound_speed(*p),
                    };
                    writeln!(out, "{v:.17e}")?;
                }
                writeln!(out, "</DataArray>")?;
            }
            writeln!(out,"<DataArray type=\"Float64\" Name=\"velocity_m_s\" NumberOfComponents=\"3\" format=\"ascii\">")?;
            for p in &s.fields {
                writeln!(
                    out,
                    "{:.17e} {:.17e} {:.17e}",
                    p.velocity.x, p.velocity.y, p.velocity.z
                )?;
            }
            writeln!(
                out,
                "</DataArray></CellData></Piece></UnstructuredGrid></VTKFile>"
            )?;
            out.flush()
        })();
        result.map_err(|e| e.to_string())?;
    }
    file.as_file().sync_all().map_err(|e| e.to_string())?;
    file.persist(path).map_err(|e| e.to_string())?;
    Ok(())
}
