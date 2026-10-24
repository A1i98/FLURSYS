use crate::{IncompressibleSolution, MeshDimension, UnstructuredMesh};
use std::{
    fs::{File, OpenOptions},
    io::{BufWriter, Write},
    path::Path,
};
fn io_err(e: std::io::Error) -> String {
    e.to_string()
}

/// The current 3D path accepts tetrahedra and rejects unsupported topology.
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
    let cells = (0..mesh.cell_count())
        .map(|cell| vtk_cell(mesh, cell))
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
    let size: usize = cells.iter().map(|(cell, _)| 1 + cell.len()).sum();
    writeln!(writer, "CELLS {} {size}", cells.len()).map_err(io_err)?;
    for (cell, _) in &cells {
        write!(writer, "{}", cell.len()).map_err(io_err)?;
        for vertex in cell {
            write!(writer, " {vertex}").map_err(io_err)?;
        }
        writeln!(writer).map_err(io_err)?;
    }
    writeln!(writer, "CELL_TYPES {}", cells.len()).map_err(io_err)?;
    for (_, cell_type) in &cells {
        writeln!(writer, "{cell_type}").map_err(io_err)?;
    }
    writeln!(writer, "CELL_DATA {}", mesh.cell_count()).map_err(io_err)?;
    write_scalar_cell_field(&mut writer, "pressure", solution.pressure.values())?;
    write_scalar_cell_field(&mut writer, "velocity_magnitude", solution.speed().values())?;
    writeln!(writer, "VECTORS velocity double").map_err(io_err)?;
    for value in solution.velocity.values() {
        writeln!(writer, "{:.12e} {:.12e} {:.12e}", value.x, value.y, value.z).map_err(io_err)?;
    }
    if let Some(t) = &solution.turbulence {
        if t.model != crate::turbulence::TurbulenceModel::SpalartAllmaras {
            write_scalar_cell_field(&mut writer, "turbulent_k", t.k.values())?;
        }
        if let Some(v) = &t.nu_tilda {
            v.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
            write_scalar_cell_field(&mut writer, "modified_turbulent_viscosity", v.values())?;
        }
        if let Some(v) = &t.epsilon {
            v.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
            write_scalar_cell_field(&mut writer, "turbulent_dissipation", v.values())?;
        }
        if matches!(
            t.model,
            crate::turbulence::TurbulenceModel::KOmega
                | crate::turbulence::TurbulenceModel::KOmegaSst
        ) {
            write_scalar_cell_field(&mut writer, "specific_dissipation", t.omega.values())?;
        }
        write_scalar_cell_field(&mut writer, "turbulent_viscosity", t.nut.values())?;
    }
    if let Some(t) = &solution.temperature {
        t.ensure_mesh(mesh).map_err(|e| format!("{e:?}"))?;
        write_scalar_cell_field(&mut writer, "temperature", t.values())?;
    }
    Ok(())
}

fn vtk_cell(mesh: &UnstructuredMesh, cell_index: usize) -> Result<(Vec<usize>, usize), String> {
    match mesh.dimension() {
        MeshDimension::TwoD => Ok((polygon_vertices(mesh, cell_index)?, 7)),
        MeshDimension::ThreeD => {
            let vertices = mesh.cells()[cell_index]
                .faces
                .iter()
                .flat_map(|&face| mesh.faces()[face].vertices.iter().copied())
                .collect::<std::collections::BTreeSet<_>>()
                .into_iter()
                .collect::<Vec<_>>();
            if vertices.len() != 4 {
                return Err(format!(
                    "legacy unstructured VTK export supports only tetrahedral 3D cells; cell {cell_index} has {} vertices",
                    vertices.len()
                ));
            }
            Ok((vertices, 10))
        }
    }
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

/// Cell-centred scalar-transport frames. Axisymmetric meshes remain meridian
/// profiles; the exporter does not invent a revolved 3D velocity field.
pub fn write_transport_vtk(
    path: &Path,
    mesh: &UnstructuredMesh,
    state: &crate::multiphysics::State,
) -> Result<(), String> {
    let n = mesh.cell_count();
    if [
        state.temperature.len(),
        state.phase_fraction.len(),
        state.density.len(),
        state.solid.len(),
    ]
    .iter()
    .any(|&len| len != n)
        || state
            .temperature
            .iter()
            .chain(&state.phase_fraction)
            .chain(&state.density)
            .any(|v| !v.is_finite())
    {
        return Err("Invalid transport field dimensions/values for VTK export".into());
    }
    let cells = (0..n)
        .map(|i| match mesh.dimension() {
            MeshDimension::TwoD => polygon_vertices(mesh, i).map(|v| (v, 7)),
            MeshDimension::ThreeD => {
                let v = mesh.cells()[i]
                    .faces
                    .iter()
                    .flat_map(|&f| mesh.faces()[f].vertices.iter().copied())
                    .collect::<std::collections::BTreeSet<_>>()
                    .into_iter()
                    .collect::<Vec<_>>();
                let kind = if v.len() == 4 { 10 } else { 41 }; // tetrahedron or convex point set
                Ok((v, kind))
            }
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mut writer = BufWriter::new(
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(io_err)?,
    );
    writeln!(writer,"# vtk DataFile Version 3.0\nFLURSYS phase and sensible energy\nASCII\nDATASET UNSTRUCTURED_GRID").map_err(io_err)?;
    writeln!(
        writer,
        "FIELD FieldData 2\nTIME 1 1 double\n{:.17e}\nSTEP 1 1 double\n{}",
        state.time, state.step
    )
    .map_err(io_err)?;
    writeln!(writer, "POINTS {} double", mesh.points().len()).map_err(io_err)?;
    for p in mesh.points() {
        writeln!(
            writer,
            "{:.17e} {:.17e} {:.17e}",
            p.position.x, p.position.y, p.position.z
        )
        .map_err(io_err)?;
    }
    writeln!(
        writer,
        "CELLS {} {}",
        n,
        cells.iter().map(|(v, _)| v.len() + 1).sum::<usize>()
    )
    .map_err(io_err)?;
    for (v, _) in &cells {
        write!(writer, "{}", v.len()).map_err(io_err)?;
        for index in v {
            write!(writer, " {index}").map_err(io_err)?;
        }
        writeln!(writer).map_err(io_err)?;
    }
    writeln!(writer, "CELL_TYPES {n}").map_err(io_err)?;
    for (_, kind) in &cells {
        writeln!(writer, "{kind}").map_err(io_err)?;
    }
    writeln!(writer, "CELL_DATA {n}").map_err(io_err)?;
    write_scalar_cell_field(&mut writer, "phase_fraction", &state.phase_fraction)?;
    write_scalar_cell_field(&mut writer, "temperature_K", &state.temperature)?;
    write_scalar_cell_field(&mut writer, "density_kg_m3", &state.density)?;
    write_scalar_cell_field(
        &mut writer,
        "solid",
        &state
            .solid
            .iter()
            .map(|&v| if v { 1.0 } else { 0.0 })
            .collect::<Vec<_>>(),
    )?;
    writer.flush().map_err(io_err)
}
