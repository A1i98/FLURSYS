//! Load saved construction parameters without creating a second feature.
use super::*;
use flursys::{BodyId, CadRecipe, CadTransform};

fn position(ui: &MainWindow, p: Vec3) {
    ui.set_cad_pos_x(super::cad_units::display_length(ui, p.x));
    ui.set_cad_pos_y(super::cad_units::display_length(ui, p.y));
    ui.set_cad_pos_z(super::cad_units::display_length(ui, p.z));
}
fn axis(ui: &MainWindow, a: Vec3) {
    ui.set_cad_axis(3);
    ui.set_cad_axis_x(a.x.to_string().into());
    ui.set_cad_axis_y(a.y.to_string().into());
    ui.set_cad_axis_z(a.z.to_string().into());
}
pub(super) fn load(ui: &MainWindow, state: &mut AppState, body: BodyId) -> Result<String, String> {
    let b = state
        .workbench
        .geometry()
        .cad_body(body)
        .ok_or("Body not found")?;
    let body_name = b.name.clone();
    let recipe = b
        .recipe
        .as_ref()
        .ok_or("This body has no editable construction recipe")?
        .clone();
    ui.set_cad_reverse(false);
    let operation = match &recipe {
        CadRecipe::Finish {
            kind,
            amount,
            corners,
            opening,
            ..
        } => {
            ui.set_cad_size_x(super::cad_units::display_length(ui, *amount));
            ui.set_cad_finish_corner0(corners & 1 != 0);
            ui.set_cad_finish_corner1(corners & 2 != 0);
            ui.set_cad_finish_corner2(corners & 4 != 0);
            ui.set_cad_finish_corner3(corners & 8 != 0);
            ui.set_cad_finish_opening(i32::from(*opening));
            53 + i32::from(*kind)
        }
        CadRecipe::Primitive { shape, position: p } => {
            let (kind, x, y, z) = match *shape {
                flursys::CadPrimitive::Box {
                    length,
                    width,
                    height,
                } => (0, length, width, height),
                flursys::CadPrimitive::Cylinder { radius, height } => (1, radius, height, 1.0),
                flursys::CadPrimitive::Sphere { radius } => (2, radius, 1.0, 1.0),
                flursys::CadPrimitive::Cone { radius, height } => (3, radius, height, 1.0),
                flursys::CadPrimitive::Torus {
                    major_radius,
                    minor_radius,
                } => (4, major_radius, minor_radius, 1.0),
            };
            ui.set_cad_primitive_kind(kind);
            ui.set_cad_name(b.name.clone().into());
            ui.set_cad_size_x(super::cad_units::display_length(ui, x));
            ui.set_cad_size_y(super::cad_units::display_length(ui, y));
            ui.set_cad_size_z(super::cad_units::display_length(ui, z));
            position(ui, *p);
            200 + kind
        }
        CadRecipe::Revolve {
            pivot,
            axis: a,
            degrees,
            face,
            axis_reference,
            axis_reversed,
        } => {
            ui.set_cad_profile_face(face.get().to_string().into());
            ui.set_cad_axis_mode(match axis_reference {
                Some(flursys::CadAxisReference::ProfileX(_)) => 1,
                Some(flursys::CadAxisReference::ProfileY(_)) => 2,
                Some(flursys::CadAxisReference::Edge(_)) => 3,
                None => 0,
            });
            ui.set_cad_axis_edge(
                match axis_reference {
                    Some(flursys::CadAxisReference::Edge(id)) => id.get().to_string(),
                    _ => String::new(),
                }
                .into(),
            );
            ui.set_cad_reverse(axis_reference.is_some() && *axis_reversed);
            ui.set_cad_revolve_x(super::cad_units::display_length(ui, pivot.x));
            ui.set_cad_revolve_y(super::cad_units::display_length(ui, pivot.y));
            ui.set_cad_revolve_z(super::cad_units::display_length(ui, pivot.z));
            axis(ui, *a);
            ui.set_cad_angle(super::cad_units::display_angle(ui, *degrees));
            2
        }
        CadRecipe::Sweep { displacement, .. } => {
            ui.set_cad_size_x(super::cad_units::display_length(ui, displacement.x));
            ui.set_cad_size_y(super::cad_units::display_length(ui, displacement.y));
            ui.set_cad_size_z(super::cad_units::display_length(ui, displacement.z));
            43
        }
        CadRecipe::Taper {
            distance, factor, ..
        } => {
            ui.set_cad_size_z(super::cad_units::display_length(ui, *distance));
            ui.set_cad_factor(factor.to_string().into());
            46
        }
        CadRecipe::Loft { sections } => {
            state.wb_selected_targets = sections
                .iter()
                .copied()
                .map(GeometrySelectionTarget::Face)
                .collect();
            state.geometry_editor.selection = state.wb_selected_targets.clone();
            state.selected_tree = state
                .wb_selected_targets
                .last()
                .copied()
                .and_then(tree_selection_for_target);
            state.tree_dirty = true;
            20
        }
        CadRecipe::Transform { transform, .. } | CadRecipe::Modified { transform, .. } => {
            match transform {
                CadTransform::Translate(p) => {
                    position(ui, *p);
                    3
                }
                CadTransform::Rotate {
                    pivot,
                    axis: a,
                    degrees,
                } => {
                    position(ui, *pivot);
                    axis(ui, *a);
                    ui.set_cad_angle(super::cad_units::display_angle(ui, *degrees));
                    4
                }
                CadTransform::Mirror { pivot, normal } => {
                    position(ui, *pivot);
                    axis(ui, *normal);
                    41
                }
                CadTransform::Scale { pivot, factor } => {
                    position(ui, *pivot);
                    ui.set_cad_factor(factor.to_string().into());
                    5
                }
                CadTransform::ScaleAxes { pivot, factors } => {
                    position(ui, *pivot);
                    ui.set_cad_size_x(factors.x.to_string().into());
                    ui.set_cad_size_y(factors.y.to_string().into());
                    ui.set_cad_size_z(factors.z.to_string().into());
                    42
                }
            }
        }
        CadRecipe::Boolean {
            operation,
            tolerance,
            ..
        } => {
            ui.set_cad_tolerance(super::cad_units::display_length(ui, *tolerance));
            match operation {
                flursys::CadBoolean::Union => 9,
                flursys::CadBoolean::Difference => 10,
                flursys::CadBoolean::Intersection => 11,
            }
        }
        _ => {
            return Err(
                "Use the Pad/Pocket editor or edit the source sketches for this operation".into(),
            )
        }
    };
    super::cad_properties::open_for_action(ui, operation);
    ui.set_cad_editing_pad(false);
    ui.set_cad_editing_pocket(false);
    ui.set_cad_sketch_editing(false);
    ui.set_cad_edit_body_id(body.get().to_string().into());
    ui.set_cad_task_title(format!("Edit {} · {}", ui.get_cad_task_title(), body_name).into());
    state.show_geometry_3d = true;
    state.show_mesh = false;
    ui.set_geometry_cad_view_3d(true);
    Ok("Editing the saved operation. Preview rebuilds this body and its dependents; Cancel preserves the document.".into())
}

pub(super) fn editable(g: &flursys::GeometryTopology, id: BodyId) -> bool {
    g.extrude_features()
        .any(|f| f.body == id && f.distance.is_some())
        || g.pocket_features().any(|f| f.body == id)
        || g.cad_body(id)
            .and_then(|b| b.recipe.as_ref())
            .is_some_and(|r| {
                matches!(
                    r,
                    CadRecipe::Finish { .. }
                        | CadRecipe::Primitive { .. }
                        | CadRecipe::Revolve { .. }
                        | CadRecipe::Sweep { .. }
                        | CadRecipe::Taper { .. }
                        | CadRecipe::Boolean { .. }
                        | CadRecipe::Transform { .. }
                        | CadRecipe::Modified { .. }
                        | CadRecipe::Loft { .. }
                )
            })
}
pub(super) fn has_source_sketch(g: &flursys::GeometryTopology, id: BodyId) -> bool {
    super::cad_tree::profiles(g, id).iter().any(|face| {
        g.sketches()
            .any(|s| s.profile_face == Some(*face) && s.profile.is_some())
    })
}
