use flursys::{
    BoundaryPatch, BoundaryType, CellDefinition, MeshDimension, MeshRenderCache, MeshSelection,
    MeshSelectionTarget, Point, UnstructuredMesh,
};

fn two_cell_mesh() -> UnstructuredMesh {
    UnstructuredMesh::from_cells(
        MeshDimension::TwoD,
        vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(2.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(1.0, 1.0, 0.0),
            Point::new(2.0, 1.0, 0.0),
        ],
        vec![
            CellDefinition::polygon(vec![0, 1, 4, 3]),
            CellDefinition::polygon(vec![1, 2, 5, 4]),
        ],
    )
    .unwrap()
    .with_boundary_patches(vec![
        BoundaryPatch {
            name: "inlet".into(),
            face_indices: vec![3],
            boundary_type: BoundaryType::VelocityInlet,
        },
        BoundaryPatch {
            name: "outlet".into(),
            face_indices: vec![5],
            boundary_type: BoundaryType::PressureOutlet,
        },
        BoundaryPatch {
            name: "walls".into(),
            face_indices: vec![0, 2, 4, 6],
            boundary_type: BoundaryType::Wall,
        },
    ])
    .unwrap()
}

#[test]
fn cache_uses_real_mesh_topology_and_picks_2d_faces_and_cells() {
    let mesh = two_cell_mesh();
    let cache = MeshRenderCache::build(&mesh).unwrap();
    assert_eq!(cache.mesh_id(), mesh.id());
    assert_eq!(cache.cell_ranges().len(), 2);
    assert_eq!(cache.face_ranges().len(), mesh.face_count());
    assert_eq!(
        cache.pick_cell_2d((0.25, 0.5)),
        Some(MeshSelection::cell(mesh.id(), 0))
    );
    assert_eq!(
        cache.pick_cell_2d((1.75, 0.5)),
        Some(MeshSelection::cell(mesh.id(), 1))
    );
    assert_eq!(
        cache.pick_face_2d((0.0, 0.5), 0.05),
        Some(MeshSelection::face(mesh.id(), 3))
    );
    assert_eq!(cache.pick_cell_2d((3.0, 3.0)), None);
}

#[test]
fn quality_is_backend_derived_safe_and_thresholded() {
    let mesh = two_cell_mesh();
    let cache = MeshRenderCache::build(&mesh).unwrap();
    let quality = cache.quality();
    assert_eq!(quality.cell_measure(), &[1.0, 1.0]);
    assert!(quality.aspect_ratio().iter().all(|value| value.is_finite()));
    assert_eq!(
        quality.bad_cells(flursys::MeshQualityMetric::CellMeasure, 0.5),
        vec![0, 1]
    );
    assert!(quality
        .bad_cells(flursys::MeshQualityMetric::CellMeasure, 1.0)
        .is_empty());
    assert_eq!(
        quality.color_range(flursys::MeshQualityMetric::CellMeasure),
        Some((1.0, 1.0))
    );
}

#[test]
fn mesh_bound_selection_rejects_a_replacement_mesh() {
    let first = two_cell_mesh();
    let second = two_cell_mesh();
    let selected = MeshSelection::face(first.id(), 3);
    assert!(selected.resolve(&first).is_some());
    assert!(selected.resolve(&second).is_none());
    assert_eq!(selected.target(), MeshSelectionTarget::Face(3));
}

#[test]
fn three_dimensional_cache_contains_exterior_triangles_and_picks_a_mesh_face() {
    let mesh = UnstructuredMesh::from_cells(
        MeshDimension::ThreeD,
        vec![
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 1.0),
        ],
        vec![CellDefinition::tetrahedron([0, 1, 2, 3])],
    )
    .unwrap();
    let cache = MeshRenderCache::build(&mesh).unwrap();
    assert_eq!(cache.surface_triangles().len(), 4);
    assert!(cache
        .quality()
        .cell_measure()
        .iter()
        .all(|value| value.is_finite()));
    assert!(matches!(
        cache
            .pick_face_3d(
                flursys::Vec3::new(2.0, 0.2, 0.2),
                flursys::Vec3::new(-1.0, 0.0, 0.0),
            )
            .map(MeshSelection::target),
        Some(MeshSelectionTarget::Face(_))
    ));
}
