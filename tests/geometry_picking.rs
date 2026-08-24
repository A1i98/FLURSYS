use flursys::{
    CadPickMode, CadSelectionState, GeometrySelectionTarget, GeometryTopology, Ray3, Vec3,
};

fn extruded_rectangle() -> (GeometryTopology, flursys::ExtrudeEntities) {
    let mut geometry = GeometryTopology::new();
    let rectangle = geometry.add_rectangle(2.0, 1.0).unwrap();
    let extrusion = geometry.extrude_planar_face(rectangle.face, 0.5).unwrap();
    (geometry, extrusion)
}

#[test]
fn ray_picking_an_extruded_rectangle_returns_canonical_face_ids() {
    let (geometry, extrusion) = extruded_rectangle();
    let revision = geometry.revision();
    let before = geometry.clone();

    let top = geometry
        .pick_face(Ray3::new(Vec3::new(1.0, 0.5, 2.0), Vec3::new(0.0, 0.0, -1.0)).unwrap())
        .unwrap();
    assert_eq!(top.face, extrusion.top_face);
    assert_eq!(top.body, Some(extrusion.body));

    let side = geometry
        .pick_face(Ray3::new(Vec3::new(1.0, -2.0, 0.25), Vec3::new(0.0, 1.0, 0.0)).unwrap())
        .unwrap();
    assert_eq!(side.face, extrusion.side_faces[0]);
    assert_eq!(side.body, Some(extrusion.body));

    assert_eq!(geometry.revision(), revision);
    assert_eq!(geometry, before);
}

#[test]
fn body_mode_derives_the_body_from_the_visible_canonical_face() {
    let (geometry, extrusion) = extruded_rectangle();
    let ray = Ray3::new(Vec3::new(1.0, 0.5, 2.0), Vec3::new(0.0, 0.0, -1.0)).unwrap();
    let mut selection = CadSelectionState::new(CadPickMode::Body);

    assert_eq!(
        selection.update_hover(&geometry, ray),
        Some(GeometrySelectionTarget::Body(extrusion.body))
    );
    assert_eq!(
        selection.hover(),
        Some(GeometrySelectionTarget::Body(extrusion.body))
    );
    assert_eq!(
        selection.select_hover(),
        Some(GeometrySelectionTarget::Body(extrusion.body))
    );
    assert_eq!(
        selection.selected(),
        Some(GeometrySelectionTarget::Body(extrusion.body))
    );
}

#[test]
fn face_mode_exposes_hover_and_selected_stable_face_without_geometry_mutation() {
    let (geometry, extrusion) = extruded_rectangle();
    let before = geometry.clone();
    let mut selection = CadSelectionState::new(CadPickMode::Face);
    let ray = Ray3::new(Vec3::new(1.0, 0.5, 2.0), Vec3::new(0.0, 0.0, -1.0)).unwrap();

    assert_eq!(
        selection.update_hover(&geometry, ray),
        Some(GeometrySelectionTarget::Face(extrusion.top_face))
    );
    assert_eq!(
        selection.select_hover(),
        Some(GeometrySelectionTarget::Face(extrusion.top_face))
    );
    assert_eq!(geometry, before);
}
