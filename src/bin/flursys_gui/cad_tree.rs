//! Construction history projected from persisted recipes and topology references.
//! Shared inputs remain references to the same IDs, not duplicated geometry.
use super::*;
use flursys::{BodyId, CadRecipe, CadSketchId, FaceId, GeometryTopology};
use std::collections::{BTreeMap, BTreeSet};
fn row(
    depth: usize,
    kind: i32,
    id: u64,
    label: String,
    note: String,
    selected: Option<TreeSelection>,
) -> ProjectTreeRowData {
    ProjectTreeRowData {
        icon: match kind {
            TREE_KIND_BODY => 9,
            TREE_KIND_SKETCH => 1,
            TREE_KIND_REGION => 10,
            _ => 0,
        },
        depth,
        kind,
        payload: id as i32,
        label,
        note,
        active: selected
            == Some(TreeSelection {
                kind,
                payload: id as i32,
            })
            || (kind == TREE_KIND_HISTORY_FEATURE
                && selected
                    == Some(TreeSelection {
                        kind: TREE_KIND_BODY,
                        payload: id as i32,
                    })),
    }
}
pub(super) fn recipe_name(recipe: &CadRecipe) -> &'static str {
    match recipe {
        CadRecipe::Finish{kind,..} => match kind {0=>"Prism fillet",1=>"Prism chamfer",_=>"Prism shell"},
        CadRecipe::Primitive { shape, .. } => match shape {
            flursys::CadPrimitive::Box { .. } => "Box",
            flursys::CadPrimitive::Cylinder { .. } => "Cylinder",
            flursys::CadPrimitive::Sphere { .. } => "Sphere",
            flursys::CadPrimitive::Cone { .. } => "Cone",
            flursys::CadPrimitive::Torus { .. } => "Torus",
        },
        CadRecipe::Extrude { .. } => "Pad",
        CadRecipe::Revolve { .. } => "Revolve",
        CadRecipe::Loft { .. } => "Loft",
        CadRecipe::Sweep { .. } => "Sweep",
        CadRecipe::Taper { .. } => "Taper",
        CadRecipe::Boolean { operation, .. } => match operation {
            flursys::CadBoolean::Union => "Union",
            flursys::CadBoolean::Difference => "Difference",
            flursys::CadBoolean::Intersection => "Intersection",
        },
        CadRecipe::Transform { transform, .. } | CadRecipe::Modified { transform, .. } => {
            match transform {
                flursys::CadTransform::Translate(_) => "Move",
                flursys::CadTransform::Rotate { .. } => "Rotate",
                flursys::CadTransform::Mirror { .. } => "Mirror",
                flursys::CadTransform::Scale { .. } => "Scale",
                flursys::CadTransform::ScaleAxes { .. } => "Scale XYZ",
            }
        }
        CadRecipe::PocketTool { .. } => "Pocket tool",
    }
}
fn name(g: &GeometryTopology, id: BodyId) -> String {
    if g.pocket_features().any(|f| f.body == id) {
        return "Pocket".into();
    }
    if let Some(f) = g.extrude_features().find(|f| f.body == id) {
        if !matches!(
            g.cad_body(id).and_then(|b| b.recipe.as_ref()),
            Some(CadRecipe::Modified { .. })
        ) {
            return if f.additive { "Additive Pad" } else { "Pad" }.into();
        }
    }
    let b = g.cad_body(id).unwrap();
    b.recipe
        .as_ref()
        .map(|r| recipe_name(r).into())
        .unwrap_or_else(|| {
            b.operation
                .split('·')
                .next()
                .unwrap_or("Solid")
                .trim()
                .to_string()
        })
}
fn inputs(g: &GeometryTopology, id: BodyId) -> Vec<BodyId> {
    let Some(b) = g.cad_body(id) else {
        return Vec::new();
    };
    let mut out = b
        .dependencies
        .as_ref()
        .map(|d| d.bodies.clone())
        .unwrap_or_default();
    if let Some(d) = &b.dependencies {
        for face in &d.profiles {
            if let Some(host) = g
                .sketches()
                .find(|s| s.profile_face == Some(*face))
                .and_then(|s| s.host_face)
                .and_then(|f| g.cad_owner(f))
            {
                if host != id && !out.contains(&host) {
                    out.push(host);
                }
            }
        }
    }
    out
}
fn walk(g: &GeometryTopology, id: BodyId, visited: &mut BTreeSet<BodyId>, order: &mut Vec<BodyId>) {
    if !visited.insert(id) {
        return;
    }
    for parent in inputs(g, id) {
        walk(g, parent, visited, order);
    }
    order.push(id);
}
pub(super) fn profiles(g: &GeometryTopology, id: BodyId) -> Vec<FaceId> {
    let mut faces = g
        .cad_body(id)
        .and_then(|b| b.dependencies.as_ref())
        .map(|d| d.profiles.clone())
        .unwrap_or_default();
    if let Some(f) = g.pocket_features().find(|f| f.body == id) {
        if !faces.contains(&f.source_face) {
            faces.push(f.source_face);
        }
    }
    faces
}
fn sketch(
    g: &GeometryTopology,
    owner: CadSketchId,
    depth: usize,
    used: &BTreeSet<FaceId>,
    selected: Option<TreeSelection>,
    rows: &mut Vec<ProjectTreeRowData>,
) {
    let Some(root) = g.sketch(owner) else {
        return;
    };
    rows.push(row(
        depth,
        TREE_KIND_SKETCH,
        owner.get(),
        format!("Sketch {}", owner.get()),
        format!(
            "{} regions · {:?}",
            g.cad_sketch_region_count(owner),
            root.plane
        ),
        selected,
    ));
    if let Some(host) = root.host_face {
        rows.push(row(
            depth + 1,
            TREE_KIND_FACE,
            host.get(),
            format!("Support · Face {}", host.get()),
            g.cad_owner(host)
                .map(|id| format!("Body #{}", id.get()))
                .unwrap_or_default(),
            selected,
        ));
    }
    for region in g
        .sketches()
        .filter(|s| s.id == owner || s.region_owner == Some(owner))
    {
        let shape = match region.profile.as_ref() {
            Some(flursys::CadSketchProfile::Circle { .. }) => "Circle",
            Some(flursys::CadSketchProfile::Rectangle { .. }) => "Rectangle",
            _ => "Profile",
        };
        let in_use = region.profile_face.is_some_and(|f| used.contains(&f));
        rows.push(row(
            depth + 1,
            TREE_KIND_REGION,
            region.id.get(),
            format!("Region {} · {}", region.id.get(), shape),
            format!(
                "{}{}",
                g.cad_sketch_degrees_of_freedom(region.id).map_or("free profile".into(),|n|format!("{n} DOF")),
                if in_use { " · input" } else { "" }
            ),
            selected,
        ));
        if selected.is_some_and(|t| {
            t.kind == TREE_KIND_FACE
                && region
                    .profile_face
                    .is_some_and(|f| f.get() == t.payload as u64)
        }) {
            if let Some(r) = rows.last_mut() {
                r.active = true;
            }
        }
    }
}
pub(super) fn rebuild(state: &mut AppState) {
    let g = state.workbench.geometry();
    // The recipe tree replaces CAD-owned history only. Independent planar
    // sketches and legacy canonical extrusions still use the topology tree.
    if g.cad_bodies().next().is_none() {
        return;
    }
    let selected = state.selected_tree;
    let mut history = Vec::new();
    let used_profiles = g
        .cad_bodies()
        .flat_map(|(id, _)| profiles(g, id))
        .collect::<BTreeSet<_>>();
    let consumed = g
        .sketches()
        .filter(|s| s.profile_face.is_some_and(|f| used_profiles.contains(&f)))
        .map(|s| s.region_owner.unwrap_or(s.id))
        .collect::<BTreeSet<_>>();
    let referenced = g
        .cad_bodies()
        .flat_map(|(id, _)| inputs(g, id))
        .collect::<BTreeSet<_>>();
    let tools = g.pocket_features().map(|f| f.tool).collect::<BTreeSet<_>>();
    let mut tips = g
        .cad_bodies()
        .filter(|(id, b)| (!referenced.contains(id) || b.visible) && !tools.contains(id))
        .map(|(id, _)| id)
        .collect::<Vec<_>>();
    // A corrupt cycle still has visible entries rather than silently disappearing.
    if tips.is_empty() {
        tips = g
            .cad_bodies()
            .filter(|(id, _)| !tools.contains(id))
            .map(|(id, _)| id)
            .collect();
    }
    if !tips.is_empty() {
        history.push(row(
            1,
            TREE_KIND_INERT,
            TREE_GROUP_HISTORY as u64,
            "Model history".into(),
            format!("{} result(s)", tips.len()),
            selected,
        ));
    }
    let mut attached = BTreeSet::new();
    for tip in tips {
        let b = g.cad_body(tip).unwrap();
        history.push(row(
            2,
            TREE_KIND_BODY,
            tip.get(),
            format!("Body · {}", b.name),
            format!(
                "{} · tip #{}",
                if b.visible { "visible" } else { "hidden" },
                tip.get()
            ),
            selected,
        ));
        let mut order = Vec::new();
        walk(g, tip, &mut BTreeSet::new(), &mut order);
        for (index, id) in order
            .into_iter()
            .filter(|id| !tools.contains(id))
            .enumerate()
        {
            let Some(b) = g.cad_body(id) else {
                continue;
            };
            let editable = g.extrude_features().any(|f| f.body == id)
                || g.pocket_features().any(|f| f.body == id)
                || b.recipe.as_ref().is_some_and(|r| {
                    !matches!(
                        r,
                        CadRecipe::Loft { .. }
                            | CadRecipe::PocketTool { .. }
                            | CadRecipe::Extrude { .. }
                    )
                });
            history.push(row(
                3,
                TREE_KIND_HISTORY_FEATURE,
                id.get(),
                format!("{}. {} · #{}", index + 1, name(g, id), id.get()),
                format!(
                    "{} · {}",
                    if b.visible { "visible" } else { "hidden" },
                    if editable {
                        "editable"
                    } else {
                        "source / read only"
                    }
                ),
                selected,
            ));
            if let Some(r) = history.last_mut() {
                r.icon = match name(g, id).as_str() {
                    "Pad" | "Additive Pad" => 2,
                    "Revolve" => 3,
                    "Pocket" => 4,
                    "Loft" => 5,
                    "Mirror" => 6,
                    "Move" | "Rotate" | "Scale" | "Scale XYZ" => 7,
                    "Union" | "Difference" | "Intersection" => 8,
                    _ => 9,
                };
            }
            let mut embedded = b.recipe.as_ref();
            let mut level = 0;
            while let Some(CadRecipe::Modified { source, .. }) = embedded {
                level += 1;
                history.push(row(
                    4,
                    TREE_KIND_INERT,
                    1_000_000u64 + id.get() * 64 + level,
                    format!("Previous · {}", recipe_name(source)),
                    "saved construction".into(),
                    selected,
                ));
                embedded = Some(source);
            }
            let mut owners = BTreeMap::<CadSketchId, BTreeSet<FaceId>>::new();
            for face in profiles(g, id) {
                if let Some(s) = g.sketches().find(|s| s.profile_face == Some(face)) {
                    owners
                        .entry(s.region_owner.unwrap_or(s.id))
                        .or_default()
                        .insert(face);
                } else {
                    history.push(row(
                        4,
                        TREE_KIND_FACE,
                        face.get(),
                        format!("Profile · Face {}", face.get()),
                        "input reference".into(),
                        selected,
                    ));
                }
            }
            for (owner, used) in owners {
                sketch(g, owner, 4, &used, selected, &mut history);
            }
            for s in g.sketches().filter(|s| {
                s.region_owner.is_none()
                    && !consumed.contains(&s.id)
                    && s.host_face.and_then(|f| g.cad_owner(f)) == Some(id)
            }) {
                attached.insert(s.id);
                sketch(g, s.id, 4, &BTreeSet::new(), selected, &mut history);
            }
        }
    }
    let free = g
        .sketches()
        .filter(|s| {
            s.region_owner.is_none() && !consumed.contains(&s.id) && !attached.contains(&s.id)
        })
        .map(|s| s.id)
        .collect::<Vec<_>>();
    if !free.is_empty() {
        history.push(row(
            1,
            TREE_KIND_INERT,
            TREE_GROUP_SKETCHES as u64,
            "Unconsumed sketches".into(),
            format!("{} sketch(es)", free.len()),
            selected,
        ));
        for owner in free {
            sketch(g, owner, 2, &BTreeSet::new(), selected, &mut history);
        }
    }
    // Retain non-CAD topology, mesh and solver sections under their existing IDs.
    let old = std::mem::take(&mut state.tree_rows);
    let mut out = Vec::new();
    let mut topology = Vec::new();
    let mut rest = Vec::new();
    let mut skip = false;
    let mut geometry = true;
    for r in old {
        if r.depth == 0 {
            geometry = r.kind == TREE_KIND_STAGE && r.payload == 0;
            if geometry {
                out.push(r);
                continue;
            }
        }
        if !geometry {
            rest.push(r);
            continue;
        }
        if r.depth == 1 {
            skip = r.kind == TREE_KIND_INERT
                && matches!(
                    r.payload,
                    TREE_GROUP_SKETCHES | TREE_GROUP_FEATURES | TREE_GROUP_BODIES
                );
        }
        if skip {
            continue;
        }
        if matches!(r.kind, TREE_KIND_FACE | TREE_KIND_EDGE | TREE_KIND_VERTEX)
            || (r.kind == TREE_KIND_INERT
                && matches!(
                    r.payload,
                    TREE_GROUP_FACES | TREE_GROUP_EDGES | TREE_GROUP_VERTICES
                ))
        {
            let mut r = r;
            r.depth += 1;
            topology.push(r);
        } else {
            rest.push(r);
        }
    }
    out.extend(history);
    let legacy = g
        .bodies()
        .filter(|b| g.cad_body(b.id).is_none())
        .collect::<Vec<_>>();
    if !legacy.is_empty() {
        out.push(row(
            1,
            TREE_KIND_INERT,
            TREE_GROUP_BODIES as u64,
            "Other geometry".into(),
            String::new(),
            selected,
        ));
        for b in legacy {
            out.push(row(
                2,
                TREE_KIND_BODY,
                b.id.get(),
                format!("Body {}", b.id.get()),
                "legacy topology".into(),
                selected,
            ));
        }
    }
    if !topology.is_empty() {
        out.push(row(
            1,
            TREE_KIND_INERT,
            TREE_GROUP_TOPOLOGY as u64,
            "Topology".into(),
            "faces · edges · vertices".into(),
            selected,
        ));
        out.extend(topology);
    }
    out.extend(rest);
    state.tree_rows = out;
}

pub(super) fn details(g: &GeometryTopology, id: u64) -> Option<String> {
    let (id, b) = g.cad_bodies().find(|(key, _)| key.get() == id)?;
    let parent = inputs(g, id)
        .iter()
        .map(|id| format!("#{}", id.get()))
        .collect::<Vec<_>>()
        .join(", ");
    let faces = profiles(g, id)
        .iter()
        .map(|id| format!("#{}", id.get()))
        .collect::<Vec<_>>()
        .join(", ");
    Some(format!("{} · Body #{}\nName: {}\nVisibility: {}\nInput bodies: {}\nInput profiles: {}\nSaved construction: {}\n\n{}\n\nDouble-click to edit this operation. Source sketch edits rebuild dependent operations.",name(g,id),id.get(),b.name,if b.visible{"visible"}else{"hidden"},if parent.is_empty(){"—"}else{&parent},if faces.is_empty(){"—"}else{&faces},if b.recipe.is_some(){"available"}else{"not available"},b.operation))
}

pub(super) fn bind(ui: &MainWindow, shared: &Rc<RefCell<AppState>>) {
    let weak = ui.as_weak();
    let state = Rc::downgrade(shared);
    ui.on_cad_tree_edit(move |index| {
        let (Some(ui), Some(shared)) = (weak.upgrade(), state.upgrade()) else {
            return;
        };
        if ui.get_cad_busy() || ui.get_cad_task_open() {
            ui.set_cad_feedback(
                "Finish the active task before editing another history step.".into(),
            );
            return;
        }
        let Some(kind) = shared
            .borrow()
            .tree_rows
            .get(index.max(0) as usize)
            .map(|r| r.kind)
        else {
            return;
        };
        ui.invoke_select_tree_row(index, false);
        match kind {
            TREE_KIND_HISTORY_FEATURE | TREE_KIND_BODY => ui.invoke_cad_command(64),
            TREE_KIND_SKETCH | TREE_KIND_REGION => ui.invoke_cad_command(28),
            _ => (),
        }
    });
    let weak = ui.as_weak();
    let state = Rc::downgrade(shared);
    ui.on_cad_tree_context(move |index| {
        let (Some(ui), Some(shared)) = (weak.upgrade(), state.upgrade()) else {
            return false;
        };
        if ui.get_cad_busy() || ui.get_cad_task_open() {
            return false;
        }
        let Some(kind) = shared
            .borrow()
            .tree_rows
            .get(index.max(0) as usize)
            .map(|r| r.kind)
        else {
            return false;
        };
        if !matches!(
            kind,
            TREE_KIND_HISTORY_FEATURE
                | TREE_KIND_BODY
                | TREE_KIND_SKETCH
                | TREE_KIND_REGION
                | TREE_KIND_FACE
                | TREE_KIND_EDGE
                | TREE_KIND_VERTEX
        ) {
            return false;
        }
        ui.invoke_select_tree_row(index, false);
        true
    });
    let weak = ui.as_weak();
    let state = Rc::downgrade(shared);
    ui.on_cad_tree_expand(move |expand| {
        let (Some(ui), Some(shared)) = (weak.upgrade(), state.upgrade()) else {
            return;
        };
        let mut state = shared.borrow_mut();
        let keys = state
            .tree_rows
            .iter()
            .enumerate()
            .filter(|(_i, r)| {
                matches!(
                    r.kind,
                    TREE_KIND_BODY | TREE_KIND_SKETCH | TREE_KIND_HISTORY_FEATURE
                ) || (r.kind == TREE_KIND_INERT
                    && matches!(r.payload, TREE_GROUP_HISTORY | TREE_GROUP_SKETCHES))
            })
            .filter(|(i, r)| {
                state
                    .tree_rows
                    .get(i + 1)
                    .is_some_and(|next| next.depth > r.depth)
            })
            .map(|(_, r)| (r.kind, r.payload))
            .collect::<Vec<_>>();
        for key in keys {
            if expand {
                state.collapsed_tree_groups.remove(&key);
            } else {
                state.collapsed_tree_groups.insert(key);
            }
        }
        refresh_ui(&ui, &state);
    });
}

/// Expand only the ancestor path of a viewport/history selection.
pub(super) fn reveal(state: &mut AppState, target: GeometrySelectionTarget) {
    let region = if let GeometrySelectionTarget::Face(face) = target {
        state
            .workbench
            .geometry()
            .sketches()
            .find(|s| s.profile_face == Some(face))
            .map(|s| s.id)
    } else {
        None
    };
    let index = state.tree_rows.iter().position(|r| match target {
        GeometrySelectionTarget::Body(id) => {
            r.kind == TREE_KIND_HISTORY_FEATURE && r.payload as u64 == id.get()
        }
        GeometrySelectionTarget::Face(id) => {
            if let Some(region) = region {
                r.kind == TREE_KIND_REGION && r.payload as u64 == region.get()
            } else {
                r.kind == TREE_KIND_FACE && r.payload as u64 == id.get()
            }
        }
        GeometrySelectionTarget::Edge(id) => {
            r.kind == TREE_KIND_EDGE && r.payload as u64 == id.get()
        }
        GeometrySelectionTarget::Vertex(id) => {
            r.kind == TREE_KIND_VERTEX && r.payload as u64 == id.get()
        }
    });
    let Some(index) = index else {
        return;
    };
    let mut ancestors: Vec<&ProjectTreeRowData> = Vec::new();
    for row in state.tree_rows.iter().take(index) {
        while ancestors.last().is_some_and(|a| a.depth >= row.depth) {
            ancestors.pop();
        }
        ancestors.push(row);
    }
    let keys = ancestors
        .iter()
        .map(|r| (r.kind, r.payload))
        .collect::<Vec<_>>();
    for key in keys {
        state.collapsed_tree_groups.remove(&key);
    }
}
