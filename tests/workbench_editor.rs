use flursys::{GeometryEditorState, GeometrySelectionTarget, GeometryTool, WorkbenchSession};

#[test]
fn mouse_drawn_channel_uses_stable_topology_for_named_selection_and_gmsh() {
    let mut session = WorkbenchSession::new();
    let mut editor = GeometryEditorState::new();
    editor.transform.set_viewport(1000.0, 800.0);
    editor.set_tool(GeometryTool::Rectangle);
    assert!(!editor
        .click(session.geometry_mut(), (300.0, 500.0), false)
        .unwrap());
    assert!(editor
        .click(session.geometry_mut(), (700.0, 300.0), false)
        .unwrap());
    session.geometry_changed();

    let mut edges: Vec<_> = session.geometry().edges().map(|edge| edge.id).collect();
    edges.sort();
    session
        .create_named_selection("inlet", vec![GeometrySelectionTarget::Edge(edges[3])])
        .unwrap();
    session
        .create_named_selection("outlet", vec![GeometrySelectionTarget::Edge(edges[1])])
        .unwrap();
    session
        .create_named_selection(
            "walls",
            vec![
                GeometrySelectionTarget::Edge(edges[0]),
                GeometrySelectionTarget::Edge(edges[2]),
            ],
        )
        .unwrap();

    let export = session.build_gmsh_export().unwrap();
    assert!(export
        .document
        .to_geo_string()
        .unwrap()
        .contains("Physical Curve"));
    assert_eq!(session.named_selections().len(), 3);
}

#[test]
fn deleted_face_prunes_named_selection_targets() {
    let mut session = WorkbenchSession::new();
    let rectangle = session.add_rectangle(2.0, 1.0).unwrap();
    session
        .create_named_selection("fluid", vec![GeometrySelectionTarget::Face(rectangle.face)])
        .unwrap();
    session
        .delete_geometry_target(GeometrySelectionTarget::Face(rectangle.face))
        .unwrap();
    assert!(session.named_selections().is_empty());
    assert!(session.fluid_face().is_none());
}
