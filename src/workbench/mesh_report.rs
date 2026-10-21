//! Portable quality reports from the current mesh, never solver performance data.
use super::{diagnostics::mesh_preflight, MeshQualityMetric, MeshRenderCache};
use crate::{MeshDimension, UnstructuredMesh};
use std::collections::BTreeMap;
use std::fmt::Write;
const METRICS: [MeshQualityMetric; 5] = [
    MeshQualityMetric::AspectRatio,
    MeshQualityMetric::Skewness,
    MeshQualityMetric::NonOrthogonality,
    MeshQualityMetric::CellMeasure,
    MeshQualityMetric::AdjacentVolumeRatio,
];
fn shape(mesh: &UnstructuredMesh, i: usize) -> &'static str {
    let cell = &mesh.cells()[i];
    if mesh.dimension() == MeshDimension::TwoD {
        return match cell.faces.len() {
            3 => "triangle",
            4 => "quadrilateral",
            _ => "polygon",
        };
    }
    let triangles = cell
        .faces
        .iter()
        .filter(|f| mesh.faces()[**f].vertices.len() == 3)
        .count();
    let quads = cell
        .faces
        .iter()
        .filter(|f| mesh.faces()[**f].vertices.len() == 4)
        .count();
    match (cell.faces.len(), triangles, quads) {
        (4, 4, 0) => "tetrahedron",
        (6, 0, 6) => "hexahedron",
        (5, 2, 3) => "prism",
        (5, 4, 1) => "pyramid",
        _ => "polyhedron",
    }
}
fn check(mesh: &UnstructuredMesh, cache: &MeshRenderCache) -> Result<(), String> {
    if cache.mesh_id() != mesh.id() {
        return Err("Quality cache belongs to a different mesh".into());
    }
    Ok(())
}
pub fn csv(mesh: &UnstructuredMesh, cache: &MeshRenderCache) -> Result<String, String> {
    check(mesh, cache)?;
    let mut s=String::from("cell_index_zero_based,shape,center_x_m,center_y_m,center_z_m,measure,measure_unit,aspect_vertex_distance_ratio,center_line_skewness,max_internal_nonorthogonality_deg,max_adjacent_measure_ratio\n");
    let unit = if mesh.dimension() == MeshDimension::TwoD {
        "m2"
    } else {
        "m3"
    };
    let q = cache.quality();
    for (i, c) in mesh.cells().iter().enumerate() {
        writeln!(
            s,
            "{i},{},{:.17e},{:.17e},{:.17e},{:.17e},{unit},{:.17e},{:.17e},{:.17e},{:.17e}",
            shape(mesh, i),
            c.center.x,
            c.center.y,
            c.center.z,
            c.volume,
            q.values(METRICS[0])[i],
            q.values(METRICS[1])[i],
            q.values(METRICS[2])[i],
            q.values(METRICS[4])[i]
        )
        .unwrap();
    }
    Ok(s)
}
pub fn json(
    mesh: &UnstructuredMesh,
    cache: &MeshRenderCache,
    recipe: &super::MeshRecipe,
) -> Result<String, String> {
    check(mesh, cache)?;
    let mut shapes = BTreeMap::<&str, usize>::new();
    for i in 0..mesh.cell_count() {
        *shapes.entry(shape(mesh, i)).or_default() += 1;
    }
    let summaries=METRICS.iter().map(|m|{
        let values=cache.quality().values(*m);
        let mut finite=values.iter().copied().filter(|v|v.is_finite()).collect::<Vec<_>>();finite.sort_by(f64::total_cmp);
        let percentile=|p:f64|->Option<f64>{if finite.is_empty(){return None}let index=p*(finite.len()-1) as f64;let i=index.floor() as usize;let t=index-i as f64;Some(finite[i]*(1.0-t)+finite[index.ceil() as usize]*t)};
        serde_json::json!({"metric":m.label(),"finite_cells":finite.len(),"non_finite_cells":values.len()-finite.len(),"min":finite.first(),"p50":percentile(0.5),"p95":percentile(0.95),"p99":percentile(0.99),"max":finite.last()})
    }).collect::<Vec<_>>();
    let patches=mesh.boundary_patches().iter().map(|p|serde_json::json!({"name":p.name,"faces":p.face_indices.len(),"measure":p.face_indices.iter().map(|i|mesh.faces()[*i].area).sum::<f64>(),"unit":if mesh.dimension()==MeshDimension::TwoD {"m"}else{"m2"}})).collect::<Vec<_>>();
    let bytes = std::mem::size_of_val(mesh.points())
        + std::mem::size_of_val(mesh.faces())
        + std::mem::size_of_val(mesh.cells())
        + mesh
            .cells()
            .iter()
            .map(|c| {
                std::mem::size_of_val(c.faces.as_slice())
                    + std::mem::size_of_val(c.neighbours.as_slice())
            })
            .sum::<usize>()
        + mesh
            .faces()
            .iter()
            .map(|f| std::mem::size_of_val(f.vertices.as_slice()))
            .sum::<usize>();
    serde_json::to_string_pretty(&serde_json::json!({
        "format":"flursys-mesh-quality-v1","nodes":mesh.points().len(),"cells":mesh.cell_count(),"faces":mesh.face_count(),"dimension":mesh.dimension(),"cell_shapes":shapes,"mesh_recipe":recipe,"quality":summaries,"patches":patches,"preflight":mesh_preflight(mesh),
        "mesh_payload_lower_bound_bytes":bytes,"memory_note":"Counted mesh arrays and index payloads only; excludes spare capacity, allocator overhead, patches, render caches, solver fields and matrices. Not process RSS or a pre-generation RAM forecast.",
        "definitions":{"aspect":"maximum/minimum pairwise vertex distance per cell; not Fluent orthogonal quality","skewness":"face-centre displacement from owner-neighbour centre line, divided by centre distance","nonorthogonality":"maximum internal-face normal to owner-neighbour centre-line angle per cell, degrees; boundary faces excluded","adjacent_ratio":"largest max(Vowner/Vneighbour,Vneighbour/Vowner) over internal faces; V is area in 2D","percentiles":"linear interpolation of sorted finite cell values; not volume weighted","measure_unit":if mesh.dimension()==MeshDimension::TwoD {"m2"}else{"m3"}}
    })).map_err(|e|e.to_string())
}

/// Expected connectivity for supported block topologies, not a RAM forecast.
pub fn plan(recipe: &super::MeshRecipe) -> Result<String, String> {
    use super::MeshMethod;
    recipe.tuning.validate()?;
    let [i, j, k] = recipe.tuning.divisions.map(u64::from);
    let (cells,nodes,faces,vertices_per_face,faces_per_cell)=match recipe.tuning.method {
        MeshMethod::MappedQuad=>(i*j,(i+1)*(j+1),2*i*j+i+j,2,4),
        MeshMethod::OGrid=>(4*i*j,4*i*(j+1),4*i*(2*j+1),2,4),
        MeshMethod::HexBlock|MeshMethod::SweepHex=>(i*j*k,(i+1)*(j+1)*(k+1),(i+1)*j*k+i*(j+1)*k+i*j*(k+1),4,6),
        _=>return Ok(format!("{:?}: cell count depends on generated geometry, curvature and size fields. No reliable pre-generation RAM estimate is available.",recipe.tuning.method)),
    };
    let boundary_faces = 2 * faces - faces_per_cell * cells;
    let neighbours = faces_per_cell * cells - boundary_faces;
    let bytes = nodes * std::mem::size_of::<crate::Point>() as u64
        + cells * std::mem::size_of::<crate::Cell>() as u64
        + faces * std::mem::size_of::<crate::Face>() as u64
        + (faces_per_cell * cells + neighbours + vertices_per_face * faces)
            * std::mem::size_of::<usize>() as u64;
    Ok(format!("Last validated plan: {:?}\nExpected block cells: {cells}\nConnected nodes: {nodes} · faces: {faces}\nMesh-array payload only: {:.2} MiB minimum.\nExcludes unused CAD nodes, spare capacity, allocator overhead, render caches and solver fields/matrices. Actual process RAM will be higher; this is not measured RSS. Validate again after changing inputs.",recipe.tuning.method,bytes as f64/(1024.0*1024.0)))
}
