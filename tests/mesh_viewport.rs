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
        quality.bad_cells(flursys::MeshQualityMetric::CellMeasure, 1.5),
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
    assert!(cache.cell_polygon(0).is_none());
    assert!(cache
        .triangle_faces()
        .iter()
        .all(|&f| mesh.faces()[f].neighbour.is_none()));
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

#[test]
fn mesh_sections_support_world_axes_flip_and_thin_slabs() {
    use flursys::{workbench::mesh_viewport::MeshSection, Vec3};
    let bounds = (Vec3::new(10., -4., 2.), Vec3::new(30., 6., 10.));
    let mut section = MeshSection {
        axis: 0,
        fraction: 0.25,
        reverse: false,
        slab_width: None,
    };
    assert!(section.contains(Vec3::new(14., 0., 3.), bounds));
    assert!(!section.contains(Vec3::new(16., 0., 3.), bounds));
    section.reverse = true;
    assert!(!section.contains(Vec3::new(14., 0., 3.), bounds));
    assert!(section.contains(Vec3::new(16., 0., 3.), bounds));
    section = MeshSection {
        axis: 1,
        fraction: 0.5,
        reverse: false,
        slab_width: Some(0.2),
    };
    assert!(section.contains(Vec3::new(20., 1., 6.), bounds));
    assert!(!section.contains(Vec3::new(20., 3., 6.), bounds));
    section.reverse = true;
    assert!(section.contains(Vec3::new(20., 3., 6.), bounds));
    assert!(!section.contains(Vec3::new(20., 1., 6.), bounds));
    section = MeshSection {
        axis: 2,
        fraction: 0.5,
        reverse: false,
        slab_width: None,
    };
    assert!(section.contains(Vec3::new(20., 1., 5.), bounds));
    assert!(!section.contains(Vec3::new(20., 1., 7.), bounds));
    section.fraction = f64::NAN;
    assert!(!section.contains(Vec3::new(20., 1., 5.), bounds));
}

#[test]
fn volume_wireframe_includes_interior_edges_with_bounded_deterministic_detail() {
    use flursys::workbench::mesh_viewport::{wireframe_edges, WireframeScope};
    let mut points = Vec::new();
    for k in 0..3 {
        for j in 0..3 {
            for i in 0..3 {
                points.push(Point::new(i as f64, j as f64, k as f64));
            }
        }
    }
    let at = |i: usize, j: usize, k: usize| i + 3 * (j + 3 * k);
    let mut cells = Vec::new();
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..2 {
                cells.push(CellDefinition::Hexahedron([
                    at(i, j, k),
                    at(i + 1, j, k),
                    at(i + 1, j + 1, k),
                    at(i, j + 1, k),
                    at(i, j, k + 1),
                    at(i + 1, j, k + 1),
                    at(i + 1, j + 1, k + 1),
                    at(i, j + 1, k + 1),
                ]));
            }
        }
    }
    let mesh = UnstructuredMesh::from_cells(MeshDimension::ThreeD, points, cells).unwrap();
    let surface = wireframe_edges(&mesh, None, WireframeScope::Surface, 0).unwrap();
    let volume = wireframe_edges(&mesh, None, WireframeScope::Volume, 0).unwrap();
    assert_eq!(surface.edges.len(), 48);
    assert_eq!(volume.edges.len(), 54);
    assert!(!volume.reduced);
    assert!(surface.edges.iter().all(|e| !e.vertices.contains(&13)));
    assert!(volume.edges.iter().any(|e| e.vertices.contains(&13)));
    let detail = wireframe_edges(&mesh, None, WireframeScope::Volume, 12).unwrap();
    assert!(!detail.edges.is_empty());
    assert!(detail.edges.len() <= 12);
    assert!(detail.reduced);
    assert_eq!(
        detail.edges,
        wireframe_edges(&mesh, None, WireframeScope::Volume, 12)
            .unwrap()
            .edges
    );
    let visible = mesh
        .cells()
        .iter()
        .map(|c| c.center.x < 1.)
        .collect::<Vec<_>>();
    assert_eq!(
        wireframe_edges(&mesh, Some(&visible), WireframeScope::Volume, 0)
            .unwrap()
            .edges
            .len(),
        33
    );
    assert!(wireframe_edges(&mesh, Some(&[true]), WireframeScope::Volume, 12).is_err());
    let planar = two_cell_mesh();
    assert_eq!(
        wireframe_edges(&planar, Some(&[true, false]), WireframeScope::Section, 0)
            .unwrap()
            .edges
            .len(),
        1
    );
}
