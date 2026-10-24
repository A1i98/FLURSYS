use super::{GeometrySketch, MainWindow, SketchPlane, SketchProfileKind};
pub(super) fn sketch_from_ui(ui: &MainWindow) -> Result<GeometrySketch, String> {
    let sketch_name = ui.get_sketch_name().trim().to_string();
    if sketch_name.is_empty() {
        return Err("sketch name cannot be empty".to_string());
    }
    let size_x = parse_positive(ui.get_sketch_size_x().as_str(), "profile width/radius")?;
    let size_y = parse_positive(ui.get_sketch_size_y().as_str(), "profile height")?;
    let profile = if ui.get_sketch_kind_index() == 1 {
        SketchProfileKind::Circle { radius: size_x }
    } else {
        SketchProfileKind::Rectangle {
            width: size_x,
            height: size_y,
        }
    };
    let sketch = GeometrySketch::from_profile(
        sketch_name,
        SketchPlane::Xy,
        profile,
        parse_finite(ui.get_sketch_pos_x().as_str(), "sketch origin X")?,
        parse_finite(ui.get_sketch_pos_y().as_str(), "sketch origin Y")?,
        parse_finite(ui.get_sketch_pos_z().as_str(), "sketch origin Z")?,
    );
    Ok(sketch)
}

pub(super) fn parse_positive(value: &str, label: &str) -> Result<f64, String> {
    let value = parse_finite(value, label)?;
    if value > 0.0 {
        Ok(value)
    } else {
        Err(format!("{label} must be positive"))
    }
}

pub(super) fn parse_finite(value: &str, label: &str) -> Result<f64, String> {
    value
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| format!("{label} must be a finite number"))
}

// ---- Workbench project tree / inspector helpers ----

pub(super) const TREE_KIND_STAGE: i32 = 0;
pub(super) const TREE_KIND_BODY: i32 = 1;
pub(super) const TREE_KIND_FACE: i32 = 2;
pub(super) const TREE_KIND_EDGE: i32 = 3;
pub(super) const TREE_KIND_VERTEX: i32 = 4;
pub(super) const TREE_KIND_NAMED_SELECTION: i32 = 5;
pub(super) const TREE_KIND_PATCH: i32 = 6;
pub(super) const TREE_KIND_RUN: i32 = 7;
pub(super) const TREE_KIND_SKETCH: i32 = 8;
pub(super) const TREE_KIND_EXTRUDE_FEATURE: i32 = 9;
pub(super) const TREE_KIND_HISTORY_FEATURE: i32 = 10;
pub(super) const TREE_KIND_REGION: i32 = 11;
pub(super) const TREE_KIND_MESH_PANEL: i32 = 12;
pub(super) const TREE_KIND_MESH_CONTROL: i32 = 13;
pub(super) const TREE_GROUP_HISTORY: i32 = 20;
pub(super) const TREE_GROUP_TOPOLOGY: i32 = 21;
pub(super) const TREE_KIND_INERT: i32 = -1;
pub(super) const TREE_GROUP_SKETCHES: i32 = 10;
pub(super) const TREE_GROUP_FEATURES: i32 = 11;
pub(super) const TREE_GROUP_BODIES: i32 = 12;
pub(super) const TREE_GROUP_FACES: i32 = 13;
pub(super) const TREE_GROUP_VERTICES: i32 = 14;
pub(super) const TREE_GROUP_EDGES: i32 = 15;
pub(super) const TREE_GROUP_NAMED: i32 = 16;
pub(super) const TREE_GROUP_BOUNDARIES: i32 = 17;
pub(super) const TREE_GROUP_RUNS: i32 = 18;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct TreeSelection {
    pub kind: i32,
    pub payload: i32,
}

#[derive(Clone, Debug, PartialEq)]
pub(super) struct ProjectTreeRowData {
    pub icon: i32,
    pub label: String,
    pub note: String,
    pub depth: usize,
    pub kind: i32,
    pub payload: i32,
    pub active: bool,
}

/// Inspector page for a tree selection: entity info (0), Named Selection
/// editor (1), mesh panel (2), boundary conditions (3), solver settings (4),
/// results (5).
pub(super) fn inspector_mode_for(kind: i32, step: usize) -> usize {
    match kind {
        TREE_KIND_BODY
        | TREE_KIND_FACE
        | TREE_KIND_EDGE
        | TREE_KIND_VERTEX
        | TREE_KIND_SKETCH
        | TREE_KIND_EXTRUDE_FEATURE
        | TREE_KIND_HISTORY_FEATURE
        | TREE_KIND_REGION => 0,
        TREE_KIND_NAMED_SELECTION => 1,
        TREE_KIND_PATCH => 3,
        TREE_KIND_RUN => 6,
        _ => match step {
            0 => 0,
            1 => 2,
            2 => 3,
            3 => 4,
            _ => 5,
        },
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn build_project_tree_rows(
    body_ids: &[u64],
    face_ids: &[u64],
    vertex_ids: &[u64],
    edge_ids: &[u64],
    sketch_ids: &[u64],
    extrude_features: &[(u64, u64)],
    mesh_cells: Option<usize>,
    named_selections: &[(String, usize)],
    patches: &[(String, bool)],
    runs: &[(String, String)],
    solve_note: &str,
    solved: bool,
    current_step: usize,
    selected: Option<TreeSelection>,
) -> Vec<ProjectTreeRowData> {
    let mut rows: Vec<(usize, String, String, i32, i32)> = Vec::new();
    let entities = body_ids.len()
        + face_ids.len()
        + vertex_ids.len()
        + edge_ids.len()
        + sketch_ids.len()
        + extrude_features.len();
    rows.push((
        0,
        "Geometry".to_string(),
        format!("{entities} entities"),
        TREE_KIND_STAGE,
        0,
    ));
    if !sketch_ids.is_empty() {
        rows.push((
            1,
            "Sketches".to_string(),
            format!("{} sketches", sketch_ids.len()),
            TREE_KIND_INERT,
            TREE_GROUP_SKETCHES,
        ));
        for id in sketch_ids {
            rows.push((
                2,
                format!("Sketch {id}"),
                "canonical plane".to_string(),
                TREE_KIND_SKETCH,
                *id as i32,
            ));
        }
    }
    if !extrude_features.is_empty() {
        rows.push((
            1,
            "Features".to_string(),
            format!("{} features", extrude_features.len()),
            TREE_KIND_INERT,
            TREE_GROUP_FEATURES,
        ));
        for (feature, body) in extrude_features {
            rows.push((
                2,
                format!("Extrude {feature}"),
                format!("Body {body}"),
                TREE_KIND_EXTRUDE_FEATURE,
                *feature as i32,
            ));
        }
    }
    if !body_ids.is_empty() {
        rows.push((
            1,
            "Bodies".to_string(),
            format!("{} bodies", body_ids.len()),
            TREE_KIND_INERT,
            TREE_GROUP_BODIES,
        ));
    }
    for id in body_ids {
        rows.push((
            2,
            format!("Body {id}"),
            String::new(),
            TREE_KIND_BODY,
            *id as i32,
        ));
    }
    if !face_ids.is_empty() {
        rows.push((
            1,
            "Faces".to_string(),
            format!("{} faces", face_ids.len()),
            TREE_KIND_INERT,
            TREE_GROUP_FACES,
        ));
    }
    for id in face_ids {
        rows.push((
            2,
            format!("Face {id}"),
            String::new(),
            TREE_KIND_FACE,
            *id as i32,
        ));
    }
    if !vertex_ids.is_empty() {
        rows.push((
            1,
            "Vertices".to_string(),
            format!("{} vertices", vertex_ids.len()),
            TREE_KIND_INERT,
            TREE_GROUP_VERTICES,
        ));
    }
    for id in vertex_ids {
        rows.push((
            2,
            format!("Vertex {id}"),
            String::new(),
            TREE_KIND_VERTEX,
            *id as i32,
        ));
    }
    if !edge_ids.is_empty() {
        rows.push((
            1,
            "Edges".to_string(),
            format!("{} edges", edge_ids.len()),
            TREE_KIND_INERT,
            TREE_GROUP_EDGES,
        ));
    }
    for id in edge_ids {
        rows.push((
            2,
            format!("Edge {id}"),
            String::new(),
            TREE_KIND_EDGE,
            *id as i32,
        ));
    }
    if !named_selections.is_empty() {
        rows.push((
            1,
            "Boundary groups".to_string(),
            format!("{} groups", named_selections.len()),
            TREE_KIND_INERT,
            TREE_GROUP_NAMED,
        ));
        for (name, members) in named_selections {
            rows.push((
                2,
                name.clone(),
                format!("{members} entities"),
                TREE_KIND_NAMED_SELECTION,
                0,
            ));
        }
    }

    rows.push((
        0,
        "Mesh".to_string(),
        match mesh_cells {
            Some(cells) => format!("{cells} cells"),
            None => "no mesh".to_string(),
        },
        TREE_KIND_STAGE,
        1,
    ));
    rows.push((
        1,
        "Meshing controls".to_string(),
        "Sizing · refinement · inflation".to_string(),
        TREE_KIND_MESH_PANEL,
        1,
    ));
    rows.push((
        1,
        "Mesh quality".to_string(),
        "Metrics · histogram · cutaway".to_string(),
        TREE_KIND_MESH_PANEL,
        2,
    ));

    rows.push((
        0,
        "Setup".to_string(),
        format!("{} patches", patches.len()),
        TREE_KIND_STAGE,
        2,
    ));
    if !patches.is_empty() {
        rows.push((
            1,
            "Boundaries".to_string(),
            String::new(),
            TREE_KIND_INERT,
            TREE_GROUP_BOUNDARIES,
        ));
        for (name, assigned) in patches {
            rows.push((
                2,
                name.clone(),
                if *assigned { "Assigned" } else { "Unassigned" }.to_string(),
                TREE_KIND_PATCH,
                0,
            ));
        }
    }

    rows.push((
        0,
        "Solution".to_string(),
        solve_note.to_string(),
        TREE_KIND_STAGE,
        3,
    ));
    rows.push((
        0,
        "Results".to_string(),
        if solved {
            "solution ready".to_string()
        } else {
            String::from("no solution")
        },
        TREE_KIND_STAGE,
        4,
    ));
    rows.push((
        0,
        "Runs".to_string(),
        format!("{} saved", runs.len()),
        TREE_KIND_INERT,
        TREE_GROUP_RUNS,
    ));
    for (index, (id, status)) in runs.iter().enumerate() {
        rows.push((1, id.clone(), status.clone(), TREE_KIND_RUN, index as i32));
    }

    // Patch payloads are indices into the patch list; NS payloads are indices
    // into the Named Selection list.
    let mut ns_index = 0_usize;
    let mut patch_index = 0_usize;
    rows.into_iter()
        .map(|(depth, label, note, kind, payload)| {
            let resolved_payload = match kind {
                TREE_KIND_NAMED_SELECTION => {
                    ns_index += 1;
                    (ns_index - 1) as i32
                }
                TREE_KIND_PATCH => {
                    patch_index += 1;
                    (patch_index - 1) as i32
                }
                _ => payload,
            };
            let active = match kind {
                TREE_KIND_STAGE => selected.is_none() && payload == current_step as i32,
                TREE_KIND_INERT => false,
                _ => {
                    selected
                        == Some(TreeSelection {
                            kind,
                            payload: resolved_payload,
                        })
                }
            };
            ProjectTreeRowData {
                icon: 0,
                label,
                note,
                depth,
                kind,
                payload: resolved_payload,
                active,
            }
        })
        .collect()
}
