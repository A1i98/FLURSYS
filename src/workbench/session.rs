//! Workbench session state: the presentation-layer controller that bridges the
//! Slint UI with the stable geometry topology, the Gmsh meshing backend, and
//! the high-level incompressible solver.
//!
//! The session owns validated domain state. The UI issues commands and renders
//! what this module reports; it never stores duplicate geometry/mesh/BC state.

use super::{
    BoundaryAssignment, GeometrySelectionTarget, MeshControlTarget, MeshRecipe, MeshRenderCache,
    MeshSelection, NamedSelectionError, NamedSelectionStore, PhysicalBoundaryCondition,
    WorkbenchProject,
};
use crate::{
    output::write_unstructured_legacy_vtk, BodyId, BoxEntities, CadExtrudeFeature, CadSketch,
    CadSketchId, CadSketchPlane, CircleHoleEntities, EdgeId, FaceId, GeneratedMesh, GeometryError,
    GeometryGmshExport, GeometryTopology, GmshGeometryExporter, GmshMeshOptions, GmshMesher,
    IncompressibleBoundaryCondition, IncompressibleCase, IncompressibleCaseError,
    IncompressibleMaterial, IncompressibleSolution, IncompressibleSolverOptions, MeshDimension,
    MeshingError, RectangleEntities, Vec3,
};
use std::collections::BTreeMap;
mod flow_domain;
mod structured;

#[derive(Clone, Debug, PartialEq)]
pub enum SolveStatus {
    Idle,
    Solving,
    Converged,
    MaxIterations,
    Failed(String),
}

#[derive(Debug)]
pub enum WorkbenchError {
    Selection(NamedSelectionError),
    Geometry(GeometryError),
    Meshing(MeshingError),
    Case(IncompressibleCaseError),
    NoGeometry { needed: &'static str },
    NoMesh,
    InvalidGrouping { message: String },
}

impl std::fmt::Display for WorkbenchError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Selection(error) => write!(formatter, "{error}"),
            Self::Geometry(error) => write!(formatter, "{error}"),
            Self::Meshing(error) => write!(formatter, "{error}"),
            Self::Case(error) => write!(formatter, "{error:?}"),
            Self::NoGeometry { needed } => {
                write!(
                    formatter,
                    "no {needed} exists; add one in the Geometry stage"
                )
            }
            Self::NoMesh => write!(formatter, "generate a mesh before continuing"),
            Self::InvalidGrouping { message } => {
                write!(formatter, "invalid boundary grouping: {message}")
            }
        }
    }
}

impl std::error::Error for WorkbenchError {}

impl From<NamedSelectionError> for WorkbenchError {
    fn from(value: NamedSelectionError) -> Self {
        Self::Selection(value)
    }
}

impl From<GeometryError> for WorkbenchError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

impl From<MeshingError> for WorkbenchError {
    fn from(value: MeshingError) -> Self {
        Self::Meshing(value)
    }
}

impl From<IncompressibleCaseError> for WorkbenchError {
    fn from(value: IncompressibleCaseError) -> Self {
        Self::Case(value)
    }
}

/// Single source of truth for the unstructured workbench pipeline:
/// geometry → Named Selections → Gmsh mesh → boundary conditions → solution.
pub struct WorkbenchSession {
    geometry: GeometryTopology,
    fluid_face_selection: Option<FaceId>,
    rectangle: Option<RectangleEntities>,
    box_body: Option<BoxEntities>,
    named_selections: NamedSelectionStore,
    mesh_dimension: MeshDimension,
    global_size: f64,
    min_size: f64,
    max_size: f64,
    element_order: u8,
    mesh_recipe: MeshRecipe,
    mesh_threads: usize,
    mesher: GmshMesher,
    mesh: Option<GeneratedMesh>,
    mesh_render_cache: Option<MeshRenderCache>,
    mesh_selection: Option<MeshSelection>,
    mesh_hover: Option<MeshSelection>,
    boundaries: BTreeMap<String, IncompressibleBoundaryCondition>,
    material: IncompressibleMaterial,
    catalog_material: Option<crate::materials::MaterialSnapshot>,
    catalog_history: Vec<serde_json::Value>,
    scalar_transport: Option<crate::multiphysics::Options>,
    solver: IncompressibleSolverOptions,
    solution: Option<IncompressibleSolution>,
    performance: Option<crate::compute::PerformanceReport>,
    status: SolveStatus,
}

impl Default for WorkbenchSession {
    fn default() -> Self {
        Self::new()
    }
}

impl WorkbenchSession {
    pub fn new() -> Self {
        Self {
            geometry: GeometryTopology::new(),
            fluid_face_selection: None,
            rectangle: None,
            box_body: None,
            named_selections: NamedSelectionStore::default(),
            mesh_dimension: MeshDimension::TwoD,
            global_size: 0.1,
            min_size: 0.05,
            max_size: 0.2,
            element_order: 1,
            mesh_recipe: MeshRecipe::default(),
            mesh_threads: 1,
            mesher: GmshMesher::auto(),
            mesh: None,
            mesh_render_cache: None,
            mesh_selection: None,
            mesh_hover: None,
            boundaries: BTreeMap::new(),
            catalog_material: None,
            catalog_history: Vec::new(),
            scalar_transport: None,
            material: IncompressibleMaterial {
                density: 1.0,
                kinematic_viscosity: 0.01,
            },
            solver: IncompressibleSolverOptions {
                threads: 0,
                ..Default::default()
            },
            solution: None,
            performance: None,
            status: SolveStatus::Idle,
        }
    }

    /// Reconstructs transient workbench state from a validated canonical
    /// project document. Meshes, caches, and solver fields remain derived.
    pub fn from_project(project: &WorkbenchProject) -> Result<Self, WorkbenchError> {
        project
            .validate()
            .map_err(|error| WorkbenchError::InvalidGrouping {
                message: error.to_string(),
            })?;
        let mut session = Self::new();
        session.geometry = project.geometry.clone();
        session.fluid_face_selection = project.fluid_face;
        session.solver.equations = project.solver.equations.clone();
        session.solver.pressure_coupling = project.solver.pressure_coupling;
        session.solver.volume_models = project.solver.volume_models;
        session.solver.cyclic_pairs = project.solver.cyclic_pairs.clone();
        session.solver.boundary_profiles = project.solver.boundary_profiles.clone();
        session.solver.thermal = project.solver.thermal.clone();
        session.solver.time = project.solver.time;
        session.solver.turbulence = project.solver.turbulence.clone();
        session.solver.coupling_relative_tolerance = project.solver.coupling_relative_tolerance;
        session.geometry.ensure_extrusion_edges()?;
        session.geometry.ensure_cad_subelements()?;
        session.set_mesh_configuration(
            project.mesh.dimension,
            project.mesh.global_size,
            project.mesh.min_size,
            project.mesh.max_size,
            project.mesh.element_order,
        )?;
        session.mesh_recipe = project.mesh_recipe.clone();
        session.mesh_threads = project.mesh.threads;
        session.set_cpu_threads(project.solver.threads);
        project
            .solver
            .compute
            .validate()
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        session.solver.compute = project.solver.compute.clone();
        session.set_convection(project.solver.convection);
        session.set_coupling_tolerance(project.solver.coupling_relative_tolerance)?;
        session.set_material(
            project.material.density,
            project.material.kinematic_viscosity,
        )?;
        session.catalog_material = project.material.catalog.clone();
        session.catalog_history = project.catalog_history.clone();
        session.scalar_transport = project.scalar_transport.clone();
        session.set_solver_controls(
            project.solver.max_outer_iterations,
            project.solver.velocity_relaxation,
            project.solver.pressure_relaxation,
            project.solver.continuity_absolute_tolerance,
        )?;
        for selection in &project.named_selections {
            session.create_named_selection(&selection.name, selection.targets.clone())?;
        }
        for assignment in &project.boundaries {
            session.configure_named_boundary(
                &assignment.selection,
                boundary_condition_from_document(&assignment.condition),
            )?;
        }
        session.box_body = session
            .geometry
            .bodies()
            .find_map(|body| match &body.representation {
                crate::GeometryBodyRepresentation::Box { .. } if body.faces.len() == 6 => {
                    Some(BoxEntities {
                        body: body.id,
                        x_min: body.faces[0],
                        x_max: body.faces[1],
                        y_min: body.faces[2],
                        y_max: body.faces[3],
                        z_min: body.faces[4],
                        z_max: body.faces[5],
                    })
                }
                _ => None,
            });
        Ok(session)
    }

    /// Captures only persistent workbench intent, never runtime artifacts.
    pub fn to_project(&self, name: impl Into<String>) -> WorkbenchProject {
        let mut project = WorkbenchProject::blank(name);
        project.geometry = self.geometry.clone();
        project.fluid_face = self.fluid_face_selection;
        project.named_selections = self.named_selections.iter().cloned().collect();
        project.mesh.threads = self.mesh_threads;
        project.mesh.dimension = self.mesh_dimension;
        project.mesh.global_size = self.global_size;
        project.mesh.min_size = self.min_size;
        project.mesh.max_size = self.max_size;
        project.mesh.element_order = self.element_order;
        project.mesh_recipe = self.mesh_recipe.clone();
        project.material.catalog = self.catalog_material.clone();
        project.catalog_history = self.catalog_history.clone();
        project.scalar_transport = self.scalar_transport.clone();
        project.solver.equations = self.solver.equations.clone();
        project.solver.pressure_coupling = self.solver.pressure_coupling;
        project.solver.volume_models = self.solver.volume_models;
        project.solver.cyclic_pairs = self.solver.cyclic_pairs.clone();
        project.solver.boundary_profiles = self.solver.boundary_profiles.clone();
        project.solver.thermal = self.solver.thermal.clone();
        project.material.density = self.material.density;
        project.material.kinematic_viscosity = self.material.kinematic_viscosity;
        project.solver.time = self.solver.time;
        project.solver.turbulence = self.solver.turbulence.clone();
        project.solver.max_outer_iterations = self.solver.max_outer_iterations;
        project.solver.continuity_absolute_tolerance = self.solver.continuity_absolute_tolerance;
        project.solver.coupling_relative_tolerance = self.solver.coupling_relative_tolerance;
        project.solver.velocity_relaxation = self.solver.velocity_relaxation;
        project.solver.pressure_relaxation = self.solver.pressure_relaxation;
        project.solver.threads = self.solver.threads;
        project.solver.compute = self.solver.compute.clone();
        project.solver.convection = self.solver.convection;
        project.boundaries = self
            .boundaries
            .iter()
            .map(|(selection, condition)| BoundaryAssignment {
                selection: selection.clone(),
                condition: boundary_condition_to_document(*condition),
            })
            .collect();
        project
    }

    /// Deterministic 2D rectangular channel demo:
    /// left edge → inlet, right edge → outlet, top/bottom edges → walls.
    pub fn demo_channel(width: f64, height: f64) -> Result<Self, WorkbenchError> {
        let mut session = Self::new();
        let rectangle = session.add_rectangle(width, height)?;
        let edges = |ids: &[EdgeId]| {
            ids.iter()
                .copied()
                .map(GeometrySelectionTarget::Edge)
                .collect()
        };
        session.create_named_selection("inlet", edges(&[rectangle.left]))?;
        session.create_named_selection("outlet", edges(&[rectangle.right]))?;
        session.create_named_selection("walls", edges(&[rectangle.bottom, rectangle.top]))?;
        session.boundaries.insert(
            "inlet".into(),
            IncompressibleBoundaryCondition::VelocityInlet {
                velocity: crate::Vec3::new(0.1, 0.0, 0.0),
            },
        );
        session.boundaries.insert(
            "outlet".into(),
            IncompressibleBoundaryCondition::PressureOutlet { pressure: 0.0 },
        );
        session
            .boundaries
            .insert("walls".into(), IncompressibleBoundaryCondition::NoSlipWall);
        Ok(session)
    }

    // ---------------------------------------------------------------
    // Geometry
    // ---------------------------------------------------------------

    pub fn geometry(&self) -> &GeometryTopology {
        &self.geometry
    }

    /// Mutable topology access for the geometry-editor controller only. Call
    /// [`Self::geometry_changed`] once after a successful committed edit.
    pub fn geometry_mut(&mut self) -> &mut GeometryTopology {
        &mut self.geometry
    }

    pub fn create_sketch_on_plane(
        &mut self,
        plane: CadSketchPlane,
    ) -> Result<CadSketch, WorkbenchError> {
        Ok(self.geometry.create_sketch_on_plane(plane)?)
    }

    pub fn create_sketch_on_face(&mut self, face: FaceId) -> Result<CadSketch, WorkbenchError> {
        Ok(self.geometry.create_sketch_on_face(face)?)
    }

    pub fn materialize_sketch_rectangle(
        &mut self,
        sketch: CadSketchId,
        width: f64,
        height: f64,
    ) -> Result<FaceId, WorkbenchError> {
        let face = self
            .geometry
            .materialize_sketch_rectangle(sketch, width, height)?;
        self.geometry_changed();
        Ok(face)
    }

    pub fn materialize_sketch_rectangle_at(
        &mut self,
        sketch: CadSketchId,
        x: f64,
        y: f64,
        width: f64,
        height: f64,
    ) -> Result<FaceId, WorkbenchError> {
        let face = self
            .geometry
            .materialize_sketch_rectangle_at(sketch, x, y, width, height)?;
        self.geometry_changed();
        Ok(face)
    }

    /// Invalidates all products derived from geometry. View and selection
    /// operations deliberately never call this method.
    pub fn geometry_changed(&mut self) {
        self.mesh = None;
        self.mesh_render_cache = None;
        self.mesh_selection = None;
        self.mesh_hover = None;
        self.solution = None;
        self.status = SolveStatus::Idle;
        self.fluid_face_selection = self
            .fluid_face_selection
            .filter(|id| self.geometry.face(*id).is_some());
        let geometry = &self.geometry;
        self.named_selections.retain_targets(|target| match target {
            GeometrySelectionTarget::Vertex(id) => geometry.vertex(id).is_some(),
            GeometrySelectionTarget::Edge(id) => geometry.edge(id).is_some(),
            GeometrySelectionTarget::Face(id) => geometry.face(id).is_some(),
            GeometrySelectionTarget::Body(id) => geometry.body(id).is_some(),
        });
        self.boundaries
            .retain(|name, _| self.named_selections.get(name).is_some());
        self.rectangle = self
            .rectangle
            .take()
            .filter(|rectangle| self.geometry.face(rectangle.face).is_some());
        self.box_body = self
            .box_body
            .take()
            .filter(|box_body| self.geometry.body(box_body.body).is_some());
    }

    /// Adds a planar rectangle and marks its face as the fluid face for 2D meshing.
    pub fn add_rectangle(
        &mut self,
        width: f64,
        height: f64,
    ) -> Result<RectangleEntities, WorkbenchError> {
        let rectangle = self.geometry.add_rectangle(width, height)?;
        self.rectangle = Some(rectangle.clone());
        self.geometry_changed();
        Ok(rectangle)
    }

    /// Adds a planar rectangle with a circular fluid exclusion while retaining
    /// the rectangle as the session's 2D fluid face.
    pub fn add_rectangle_with_circle(
        &mut self,
        width: f64,
        height: f64,
        center: Vec3,
        radius: f64,
    ) -> Result<(RectangleEntities, CircleHoleEntities), WorkbenchError> {
        let (rectangle, hole) = self
            .geometry
            .add_rectangle_with_circle(width, height, center.x, center.y, radius)?;
        self.rectangle = Some(rectangle.clone());
        self.geometry_changed();
        Ok((rectangle, hole))
    }

    /// Adds a box body for 3D meshing through the OpenCASCADE exporter.
    pub fn add_box(
        &mut self,
        length: f64,
        width: f64,
        height: f64,
    ) -> Result<BoxEntities, WorkbenchError> {
        let entities = self.geometry.add_box(length, width, height)?;
        self.box_body = Some(entities.clone());
        self.geometry_changed();
        Ok(entities)
    }

    /// Commits a canonical planar-profile extrusion and invalidates derived
    /// mesh/solution state while retaining stable source and generated IDs.
    pub fn extrude_face(
        &mut self,
        source_face: FaceId,
        distance: f64,
    ) -> Result<crate::ExtrudeEntities, WorkbenchError> {
        let entities = self.geometry.extrude_planar_face(source_face, distance)?;
        self.geometry_changed();
        Ok(entities)
    }

    /// Commits an extrusion tied to a canonical sketch/profile relation.
    pub fn extrude_sketch_face(
        &mut self,
        sketch: CadSketchId,
        source_face: FaceId,
        distance: f64,
    ) -> Result<(CadExtrudeFeature, crate::ExtrudeEntities), WorkbenchError> {
        let result = self
            .geometry
            .extrude_sketch_face(sketch, source_face, distance)?;
        self.geometry_changed();
        Ok(result)
    }

    /// Translates one canonical body and invalidates every geometry-derived
    /// artifact through the same lifecycle path as other committed edits.
    pub fn translate_body(
        &mut self,
        body: BodyId,
        displacement: Vec3,
    ) -> Result<(), WorkbenchError> {
        self.geometry.translate_body(body, displacement)?;
        self.geometry_changed();
        Ok(())
    }

    /// Rotates one canonical body about a finite axis and invalidates every
    /// geometry-derived artifact through the normal geometry-change path.
    pub fn rotate_body(
        &mut self,
        body: BodyId,
        pivot: Vec3,
        axis: Vec3,
        angle_radians: f64,
    ) -> Result<(), WorkbenchError> {
        self.geometry
            .rotate_body(body, pivot, axis, angle_radians)?;
        self.geometry_changed();
        Ok(())
    }

    pub fn fluid_face(&self) -> Option<FaceId> {
        if let Some(face) = self.fluid_face_selection {
            return self.geometry.face(face).map(|_| face);
        }
        self.rectangle
            .as_ref()
            .map(|rectangle| rectangle.face)
            .or_else(|| {
                let mut faces = self.geometry.faces().filter(|face| {
                    matches!(
                        face.representation,
                        crate::GeometryFaceRepresentation::Planar { .. }
                    ) && self.geometry.is_face_visible(face.id)
                        && self.geometry.cad_owner(face.id).is_none()
                });
                let face = faces.next()?.id;
                faces.next().is_none().then_some(face)
            })
    }

    pub fn target_exists(&self, target: GeometrySelectionTarget) -> bool {
        match target {
            GeometrySelectionTarget::Vertex(id) => self.geometry.vertex(id).is_some(),
            GeometrySelectionTarget::Edge(id) => self.geometry.edge(id).is_some(),
            GeometrySelectionTarget::Face(id) => self.geometry.face(id).is_some(),
            GeometrySelectionTarget::Body(id) => self.geometry.body(id).is_some(),
        }
    }

    /// Removes one topology entity through the validated geometry API.
    /// Dependent mesh/solution data and stale Named Selection targets are
    /// invalidated only after the removal succeeds.
    pub fn delete_geometry_target(
        &mut self,
        target: GeometrySelectionTarget,
    ) -> Result<(), WorkbenchError> {
        match target {
            GeometrySelectionTarget::Vertex(id) => {
                self.geometry.remove_vertex(id)?;
            }
            GeometrySelectionTarget::Edge(id) => {
                self.geometry.remove_edge(id)?;
            }
            GeometrySelectionTarget::Face(id) => {
                self.geometry.remove_face(id)?;
            }
            GeometrySelectionTarget::Body(id) => {
                self.geometry.remove_body(id)?;
            }
        }
        self.geometry_changed();
        Ok(())
    }

    // ---------------------------------------------------------------
    // Named Selections
    // ---------------------------------------------------------------

    pub fn named_selections(&self) -> &NamedSelectionStore {
        &self.named_selections
    }

    pub fn create_named_selection(
        &mut self,
        name: &str,
        targets: Vec<GeometrySelectionTarget>,
    ) -> Result<(), WorkbenchError> {
        for target in &targets {
            if !self.target_exists(*target) {
                return Err(NamedSelectionError::UnknownTarget { target: *target }.into());
            }
        }
        self.named_selections.create(name, targets, |_| true)?;
        self.invalidate_mesh();
        Ok(())
    }

    pub fn rename_named_selection(
        &mut self,
        old_name: &str,
        new_name: &str,
    ) -> Result<(), WorkbenchError> {
        self.named_selections.rename(old_name, new_name)?;
        for pair in &mut self.solver.cyclic_pairs {
            if pair.first == old_name {
                pair.first = new_name.trim().into();
            }
            if pair.second == old_name {
                pair.second = new_name.trim().into();
            }
        }
        if let Some(b) = self.solver.boundary_profiles.remove(old_name) {
            self.solver
                .boundary_profiles
                .insert(new_name.trim().into(), b);
        }
        if let Some(b) = self.solver.turbulence.inlets.remove(old_name) {
            self.solver
                .turbulence
                .inlets
                .insert(new_name.trim().into(), b);
        }
        if let Some(bc) = self.solver.thermal.boundaries.remove(old_name) {
            self.solver
                .thermal
                .boundaries
                .insert(new_name.trim().into(), bc);
        }
        if let Some(condition) = self.boundaries.remove(old_name) {
            self.boundaries.insert(new_name.trim().into(), condition);
        }
        let rename_target = |target: &mut MeshControlTarget| {
            if let MeshControlTarget::NamedSelection(name) = target {
                if name == old_name {
                    *name = new_name.trim().into();
                }
            }
        };
        for control in &mut self.mesh_recipe.edge_divisions {
            for target in &mut control.targets {
                rename_target(target);
            }
        }
        for control in &mut self.mesh_recipe.local_sizes {
            control.targets.iter_mut().for_each(rename_target);
        }
        for control in &mut self.mesh_recipe.boundary_layers {
            control.targets.iter_mut().for_each(rename_target);
        }
        for control in &mut self.mesh_recipe.refinements {
            control.targets.iter_mut().for_each(rename_target);
        }
        self.invalidate_mesh();
        Ok(())
    }

    pub fn delete_named_selection(&mut self, name: &str) -> bool {
        let deleted = self.named_selections.delete(name);
        if deleted {
            self.boundaries.remove(name);
            self.solver.thermal.boundaries.remove(name);
            self.solver.turbulence.inlets.remove(name);
            self.solver.boundary_profiles.remove(name);
            self.invalidate_mesh();
        }
        deleted
    }

    /// Deterministic physical-group input derived from Named Selections.
    /// Groups are sorted by name; a selection must contain only edges (2D)
    /// or only faces (3D box).
    fn boundary_groups_2d(&self) -> Result<Vec<(String, Vec<EdgeId>)>, WorkbenchError> {
        let mut groups = Vec::new();
        for selection in self.named_selections.iter() {
            let mut edges = selection.edges();
            if edges.len() != selection.targets.len() {
                return Err(WorkbenchError::InvalidGrouping {
                    message: format!(
                        "Named Selection {:?} must contain only geometry edges for a 2D mesh",
                        selection.name
                    ),
                });
            }
            edges.sort();
            groups.push((selection.name.clone(), edges));
        }
        groups.sort_by(|left, right| left.0.cmp(&right.0));
        Ok(groups)
    }

    fn boundary_groups_3d(&self) -> Result<Vec<(String, Vec<FaceId>)>, WorkbenchError> {
        let mut groups = Vec::new();
        for selection in self.named_selections.iter() {
            let mut faces = Vec::new();
            for target in &selection.targets {
                match target {
                    GeometrySelectionTarget::Face(id) => faces.push(*id),
                    _ => {
                        return Err(WorkbenchError::InvalidGrouping {
                            message: format!(
                                "Named Selection {:?} must contain only geometry faces for a 3D box mesh",
                                selection.name
                            ),
                        })
                    }
                }
            }
            faces.sort();
            groups.push((selection.name.clone(), faces));
        }
        groups.sort_by(|left, right| left.0.cmp(&right.0));
        Ok(groups)
    }

    fn apply_mesh_recipe(
        &self,
        mut export: GeometryGmshExport,
    ) -> Result<GeometryGmshExport, WorkbenchError> {
        export = self.apply_topology_method(export)?;
        let mut source = String::from("\n// FLURSYS persisted advanced mesh recipe.\n");
        let t = &self.mesh_recipe.tuning;
        t.validate()
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        source.push_str(&format!(
            "Mesh.Algorithm = {};\nMesh.Algorithm3D = {};\nMesh.MeshSizeFromCurvature = {};\nMesh.Smoothing = {};\nMesh.Optimize = {};\nMesh.OptimizeNetgen = {};\nMesh.OptimizeThreshold = {};\nMesh.MeshSizeExtendFromBoundary = {};\nMesh.MeshSizeFromPoints = 1;\nMeshSize {{ PointsOf{{ Surface{{:}}; }} }} = {};\n",
            t.algorithm_2d, t.algorithm_3d, t.curvature_points, t.smoothing_steps,
            u8::from(t.optimize), u8::from(t.optimize_netgen), t.optimize_threshold,
            u8::from(t.extend_boundary_sizes), self.global_size));
        // Reject silent clamping of explicit controls by global size bounds.
        for control in self.mesh_recipe.local_sizes.iter().filter(|c| c.enabled) {
            if control.size < self.min_size || control.size > self.max_size {
                return Err(WorkbenchError::InvalidGrouping { message: format!(
                    "Local size '{}' ({}) is outside global bounds [{}, {}]. Adjust the bounds or this control.",
                    control.name, control.size, self.min_size, self.max_size) });
            }
        }
        for control in self.mesh_recipe.refinements.iter().filter(|c| c.enabled) {
            if control.size_min < self.min_size || control.size_max > self.max_size {
                return Err(WorkbenchError::InvalidGrouping {
                    message: format!(
                        "Distance refinement '{}' sizes must lie inside global bounds [{}, {}]",
                        control.name, self.min_size, self.max_size
                    ),
                });
            }
        }
        let entity_kind = match self.mesh_dimension {
            MeshDimension::TwoD => "Curve",
            MeshDimension::ThreeD => "Surface",
        };
        let mut local_sizes = self
            .mesh_recipe
            .local_sizes
            .iter()
            .filter(|control| control.enabled)
            .collect::<Vec<_>>();
        local_sizes.sort_by_key(|control| control.id);
        for control in local_sizes {
            let tags = self.recipe_target_tags(&export, &control.targets)?;
            let entity_kind = match self.mesh_dimension {
                MeshDimension::TwoD => "Curve",
                MeshDimension::ThreeD => "Surface",
            };
            source.push_str(&format!(
                "MeshSize {{ PointsOf{{ {entity_kind}{{{}}}; }} }} = {};\n",
                join_tags(&tags),
                control.size
            ));
        }

        let mut divided = std::collections::BTreeSet::new();
        for control in self.mesh_recipe.edge_divisions.iter().filter(|c| c.enabled) {
            control
                .validate()
                .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
            if self.mesh_dimension != MeshDimension::TwoD {
                return Err(WorkbenchError::InvalidGrouping {
                    message: "Exact edge divisions currently require analytic 2D boundary edges"
                        .into(),
                });
            }
            for tag in self.recipe_target_tags(&export, &control.targets)? {
                if !divided.insert(tag) {
                    return Err(WorkbenchError::InvalidGrouping {
                        message:
                            "Enabled edge division scopes overlap; remove the duplicate control"
                                .into(),
                    });
                }
                let signed = if control.reverse {
                    -(tag as i64)
                } else {
                    tag as i64
                };
                let distribution = if control.symmetric {
                    "Bump"
                } else {
                    "Progression"
                };
                source.push_str(&format!(
                    "Transfinite Curve {{{signed}}} = {} Using {distribution} {:.17};\n",
                    control.cells + 1,
                    control.progression
                ));
            }
        }

        let mut field_id = 1_u32;
        let mut boundary_layers = self
            .mesh_recipe
            .boundary_layers
            .iter()
            .filter(|control| control.enabled)
            .collect::<Vec<_>>();
        boundary_layers.sort_by_key(|control| control.id);
        // Gmsh's BoundaryLayer constraint is a single active field. Merge
        // compatible scopes rather than silently replacing an earlier field.
        if let Some(first) = boundary_layers.first() {
            if self.mesh_dimension != MeshDimension::TwoD {
                return Err(WorkbenchError::InvalidGrouping { message:
                    "Prismatic inflation on arbitrary 3D CAD is not supported by this backend; use surface distance refinement instead".into() });
            }
            let mut tags = std::collections::BTreeSet::new();
            for control in &boundary_layers {
                control
                    .layer_dimensions()
                    .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
                let equal =
                    |a: f64, b: f64| (a - b).abs() <= 64.0 * f64::EPSILON * a.abs().max(b.abs());
                if !equal(control.first_layer_height, first.first_layer_height)
                    || !equal(control.growth_ratio, first.growth_ratio)
                    || control.layer_count != first.layer_count
                {
                    return Err(WorkbenchError::InvalidGrouping {message:
                        "Enabled 2D inflation controls must share first height, growth and target layers; combine compatible walls in one scope or disable conflicting controls".into()});
                }
                for tag in self.recipe_target_tags(&export, &control.targets)? {
                    if !tags.insert(tag) {
                        return Err(WorkbenchError::InvalidGrouping {message:
                            "Inflation scopes overlap. Remove duplicate edges from the enabled controls".into()});
                    }
                }
            }
            let (thickness, _) = first
                .layer_dimensions()
                .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
            source.push_str(&format!(
                "Field[{field_id}] = BoundaryLayer;\nField[{field_id}].CurvesList = {{{}}};\nField[{field_id}].Size = {};\nField[{field_id}].Ratio = {};\nField[{field_id}].Thickness = {thickness};\nField[{field_id}].SizeFar = {};\nField[{field_id}].Quads = 1;\nField[{field_id}].IntersectMetrics = 1;\nBoundaryLayer Field = {field_id};\n",
                join_tags(&tags.into_iter().collect::<Vec<_>>()), first.first_layer_height,
                first.growth_ratio, self.global_size));
            field_id += 1;
        }

        let mut refinements = self
            .mesh_recipe
            .refinements
            .iter()
            .filter(|control| control.enabled)
            .collect::<Vec<_>>();
        refinements.sort_by_key(|control| control.id);
        let mut threshold_fields = Vec::new();
        for control in refinements {
            let tags = self.recipe_target_tags(&export, &control.targets)?;
            if let Some(g) = self.mesh_recipe.tuning.max_size_growth {
                let required = (control.size_max - control.size_min) / g.ln();
                if control.distance_max - control.distance_min < required * (1.0 - 1e-10) {
                    return Err(WorkbenchError::InvalidGrouping {message:format!("Distance control '{}' transition is too short for growth {g}: require width ≥ {required:.6e} m (end ≥ {:.6e} m). Use Calculate transition or increase the end distance.",control.name,control.distance_min+required)});
                }
            }
            let distance_field = field_id;
            let threshold_field = field_id + 1;
            field_id += 2;
            source.push_str(&format!(
                "Field[{distance_field}] = Distance;\nField[{distance_field}].{entity_kind}sList = {{{}}};\nField[{distance_field}].Sampling = {};\nField[{threshold_field}] = Threshold;\nField[{threshold_field}].InField = {distance_field};\nField[{threshold_field}].SizeMin = {};\nField[{threshold_field}].SizeMax = {};\nField[{threshold_field}].DistMin = {};\nField[{threshold_field}].DistMax = {};\n",
                join_tags(&tags),
                control.sampling,
                control.size_min,
                control.size_max,
                control.distance_min,
                control.distance_max,
            ));
            threshold_fields.push(threshold_field);
        }
        for control in self.mesh_recipe.regions.iter().filter(|c| c.enabled) {
            control
                .validate()
                .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
            if control.size < self.min_size || control.size > self.max_size {
                return Err(WorkbenchError::InvalidGrouping {
                    message: format!(
                        "Region '{}' size is outside global bounds [{}, {}]",
                        control.name, self.min_size, self.max_size
                    ),
                });
            }
            let a = control.minimum;
            let b = control.maximum;
            source.push_str(&format!("Field[{field_id}] = Box;\nField[{field_id}].VIn = {};\nField[{field_id}].VOut = {};\nField[{field_id}].XMin = {};\nField[{field_id}].XMax = {};\nField[{field_id}].YMin = {};\nField[{field_id}].YMax = {};\nField[{field_id}].ZMin = {};\nField[{field_id}].ZMax = {};\nField[{field_id}].Thickness = {};\n",control.size,self.global_size,a.x,b.x,a.y,b.y,a.z,b.z,control.transition));
            threshold_fields.push(field_id);
            field_id += 1;
        }
        match threshold_fields.as_slice() {
            [] => {}
            [field] => source.push_str(&format!("Background Field = {field};\n")),
            fields => {
                source.push_str(&format!(
                    "Field[{field_id}] = Min;\nField[{field_id}].FieldsList = {{{}}};\nBackground Field = {field_id};\n",
                    fields.iter().map(u32::to_string).collect::<Vec<_>>().join(", ")
                ));
            }
        }
        export.document = export.document.with_appended_source(source);
        Ok(export)
    }

    fn recipe_target_tags(
        &self,
        export: &GeometryGmshExport,
        targets: &[MeshControlTarget],
    ) -> Result<Vec<usize>, WorkbenchError> {
        let mut stable_targets = Vec::new();
        for target in targets {
            match target {
                MeshControlTarget::Geometry(target) => stable_targets.push(*target),
                MeshControlTarget::NamedSelection(name) => {
                    let selection = self.named_selections.get(name).ok_or_else(|| {
                        WorkbenchError::InvalidGrouping {
                            message: format!(
                                "mesh recipe references missing Named Selection {name:?}"
                            ),
                        }
                    })?;
                    stable_targets.extend(selection.targets.iter().copied());
                }
            }
        }
        let mut tags = Vec::new();
        for target in stable_targets {
            match (self.mesh_dimension, target) {
                (MeshDimension::TwoD, GeometrySelectionTarget::Edge(edge)) => {
                    tags.push(export.map.edge_tag(edge).ok_or_else(|| {
                        WorkbenchError::InvalidGrouping {
                            message: format!("mesh recipe edge {} is not exported", edge.get()),
                        }
                    })?)
                }
                (MeshDimension::ThreeD, GeometrySelectionTarget::Face(face)) => {
                    let face_tags = export.map.face_tags(face);
                    if face_tags.is_empty() {
                        return Err(WorkbenchError::InvalidGrouping {
                            message: format!("mesh recipe face {} is not exported", face.get()),
                        });
                    }
                    tags.extend(face_tags);
                }
                _ => {
                    return Err(WorkbenchError::InvalidGrouping {
                        message: "Mesh recipe controls must target edges in 2D and faces in 3D"
                            .into(),
                    })
                }
            }
        }
        tags.sort_unstable();
        tags.dedup();
        if tags.is_empty() {
            return Err(WorkbenchError::InvalidGrouping {
                message: "mesh recipe control has no exported targets".into(),
            });
        }
        Ok(tags)
    }

    /// Builds the deterministic `.geo` export proving Named Selection →
    /// `GeometryToGmshMap` → Physical Curve/Surface mapping input correctness.
    pub fn build_gmsh_export(&self) -> Result<GeometryGmshExport, WorkbenchError> {
        match self.mesh_dimension {
            MeshDimension::TwoD => {
                let face = self.fluid_face().ok_or(WorkbenchError::NoGeometry {
                    needed: "planar fluid face",
                })?;
                let groups = self.boundary_groups_2d()?;
                if let Some(crate::GeometryFace {
                    representation:
                        crate::GeometryFaceRepresentation::Planar {
                            outer_loop,
                            inner_loops,
                        },
                    ..
                }) = self.geometry.face(face)
                {
                    let assigned = groups
                        .iter()
                        .flat_map(|(_, edges)| edges)
                        .copied()
                        .collect::<std::collections::BTreeSet<_>>();
                    let missing = outer_loop
                        .iter()
                        .chain(inner_loops.iter().flatten())
                        .filter(|e| !assigned.contains(&e.edge))
                        .map(|e| e.edge.get().to_string())
                        .collect::<Vec<_>>();
                    if !missing.is_empty() {
                        return Err(WorkbenchError::InvalidGrouping { message: format!("Assign every fluid boundary edge to a Named Selection before meshing. Missing edges: {}", missing.join(", ")) });
                    }
                }
                Ok(GmshGeometryExporter::planar(
                    &self.geometry,
                    face,
                    groups,
                    "fluid",
                )?)
            }
            MeshDimension::ThreeD => {
                let groups = self.boundary_groups_3d()?;
                let cad_bodies = self
                    .geometry
                    .cad_bodies()
                    .filter(|(_, body)| body.visible)
                    .collect::<Vec<_>>();
                if !cad_bodies.is_empty() {
                    if cad_bodies.len() != 1 {
                        return Err(WorkbenchError::InvalidGrouping { message: "Meshing requires one visible CAD fluid body; hide other bodies or combine them first".into() });
                    }
                    if self.mesh_recipe.tuning.method == super::MeshMethod::HexBlock {
                        return Ok(GmshGeometryExporter::cad_hex_box(
                            &self.geometry,
                            cad_bodies[0].0,
                            groups,
                            "fluid",
                        )?);
                    }
                    return Ok(GmshGeometryExporter::cad_faceted(
                        &self.geometry,
                        cad_bodies[0].0,
                        groups,
                        "fluid",
                    )?);
                }
                if let Some(body) = &self.box_body {
                    Ok(GmshGeometryExporter::rectangular_box(
                        &self.geometry,
                        body.body,
                        groups,
                        "fluid",
                    )?)
                } else if let Some(body) = self.geometry.bodies().find(|body| {
                    matches!(
                        body.representation,
                        crate::GeometryBodyRepresentation::Extrude { .. }
                    )
                }) {
                    Ok(GmshGeometryExporter::extruded_with_layers(
                        &self.geometry,
                        body.id,
                        groups,
                        "fluid",
                        matches!(
                            self.mesh_recipe.tuning.method,
                            super::MeshMethod::SweepPrism | super::MeshMethod::SweepHex
                        )
                        .then_some(self.mesh_recipe.tuning.divisions[2]),
                    )?)
                } else {
                    Err(WorkbenchError::NoGeometry {
                        needed: "box or extrusion body",
                    })
                }
            }
        }
    }

    // ---------------------------------------------------------------
    // Meshing
    // ---------------------------------------------------------------

    pub fn mesher(&self) -> &GmshMesher {
        &self.mesher
    }

    pub fn mesh_dimension(&self) -> MeshDimension {
        self.mesh_dimension
    }

    pub fn mesh_sizes(&self) -> (f64, f64, f64) {
        (self.global_size, self.min_size, self.max_size)
    }

    pub fn element_order(&self) -> u8 {
        self.element_order
    }

    pub fn mesh_recipe(&self) -> &MeshRecipe {
        &self.mesh_recipe
    }

    /// Replaces persisted advanced meshing intent and discards only current
    /// derived state. Workspace run artifacts are project-owned and untouched.
    pub fn set_mesh_recipe(&mut self, recipe: MeshRecipe) {
        if self.mesh_recipe == recipe {
            return;
        }
        self.mesh_recipe = recipe;
        self.mesh = None;
        self.mesh_render_cache = None;
        self.mesh_selection = None;
        self.mesh_hover = None;
        self.solution = None;
        self.status = SolveStatus::Idle;
    }

    pub fn mesh_cpu_threads(&self) -> usize {
        self.mesh_threads
    }

    /// Execution setting only: existing meshes stay valid when this changes.
    pub fn set_mesh_cpu_threads(&mut self, threads: usize) {
        self.mesh_threads = threads;
    }

    /// Validates and stores mesh configuration without invoking Gmsh.
    pub fn set_mesh_configuration(
        &mut self,
        dimension: MeshDimension,
        global_size: f64,
        min_size: f64,
        max_size: f64,
        element_order: u8,
    ) -> Result<(), WorkbenchError> {
        let options = GmshMeshOptions {
            threads: self.mesh_threads,
            dimension,
            characteristic_length: global_size,
            min_size: Some(min_size),
            max_size: Some(max_size),
            element_order,
        };
        options.validate()?;
        if self.mesh_dimension != dimension
            || self.global_size != global_size
            || self.min_size != min_size
            || self.max_size != max_size
            || self.element_order != element_order
        {
            self.invalidate_mesh();
        }
        self.mesh_dimension = dimension;
        self.global_size = global_size;
        self.min_size = min_size;
        self.max_size = max_size;
        self.element_order = element_order;
        Ok(())
    }

    /// Snapshot of the pure inputs handed to the Gmsh backend so generation can
    /// run on a worker thread without borrowing the session.
    pub fn mesh_generation_inputs(
        &self,
    ) -> Result<(GeometryGmshExport, GmshMeshOptions), WorkbenchError> {
        let export = self.apply_mesh_recipe(self.build_gmsh_export()?)?;
        let options = GmshMeshOptions {
            threads: self.mesh_threads,
            dimension: self.mesh_dimension,
            characteristic_length: self.global_size,
            min_size: Some(self.min_size),
            max_size: Some(self.max_size),
            element_order: self.element_order,
        };
        options.validate()?;
        Ok((export, options))
    }

    /// Installs a mesh produced by [`Self::mesh_generation_inputs`] (possibly on
    /// another thread). Boundary assignments whose patch disappeared are
    /// dropped; any previous solution is invalidated.
    pub fn install_mesh(&mut self, generated: GeneratedMesh) {
        let patch_names: std::collections::BTreeSet<String> = generated
            .mesh
            .boundary_patches()
            .iter()
            .map(|patch| patch.name.clone())
            .collect();
        self.boundaries.retain(|name, _| patch_names.contains(name));
        self.solution = None;
        self.status = SolveStatus::Idle;
        self.mesh_render_cache = MeshRenderCache::build(&generated.mesh).ok();
        self.mesh_selection = None;
        self.mesh_hover = None;
        self.mesh = Some(generated);
    }

    pub fn scalar_transport(&self) -> Option<&crate::multiphysics::Options> {
        self.scalar_transport.as_ref()
    }
    pub fn set_scalar_transport(&mut self, options: crate::multiphysics::Options) {
        self.scalar_transport = Some(options);
    }
    pub fn mesh(&self) -> Option<&GeneratedMesh> {
        self.mesh.as_ref()
    }

    pub fn has_mesh(&self) -> bool {
        self.mesh.is_some()
    }

    /// Derived viewport data is valid only for the current mesh identity.
    pub fn mesh_render_cache(&self) -> Option<&MeshRenderCache> {
        self.mesh_render_cache.as_ref()
    }

    pub fn mesh_selection(&self) -> Option<MeshSelection> {
        self.mesh_selection
    }

    pub fn mesh_hover(&self) -> Option<MeshSelection> {
        self.mesh_hover
    }

    pub fn set_mesh_selection(&mut self, selection: Option<MeshSelection>) {
        self.mesh_selection = selection.filter(|selection| {
            self.mesh
                .as_ref()
                .and_then(|generated| selection.resolve(&generated.mesh))
                .is_some()
        });
    }

    /// Hover is intentionally transient and does not alter persistent selection.
    pub fn set_mesh_hover(&mut self, hover: Option<MeshSelection>) {
        self.mesh_hover = hover.filter(|selection| {
            self.mesh
                .as_ref()
                .and_then(|generated| selection.resolve(&generated.mesh))
                .is_some()
        });
    }

    // ---------------------------------------------------------------
    // Boundaries / case
    // ---------------------------------------------------------------

    pub fn patch_names(&self) -> Vec<String> {
        self.mesh
            .as_ref()
            .map(|generated| {
                generated
                    .mesh
                    .boundary_patches()
                    .iter()
                    .map(|patch| patch.name.clone())
                    .collect()
            })
            .unwrap_or_else(|| {
                self.named_selections
                    .iter()
                    .map(|s| s.name.clone())
                    .collect()
            })
    }

    pub fn assign_boundary(
        &mut self,
        patch: &str,
        condition: IncompressibleBoundaryCondition,
    ) -> Result<(), WorkbenchError> {
        if self.cyclic_partner(patch).is_some() {
            return Err(WorkbenchError::InvalidGrouping {
                message:
                    "Remove the cyclic pair before assigning a physical condition to this patch"
                        .into(),
            });
        }
        if self.mesh.is_none() {
            return self.configure_named_boundary(patch, condition);
        }
        if !self.patch_names().iter().any(|name| name == patch) {
            return Err(WorkbenchError::InvalidGrouping {
                message: format!("boundary patch {patch:?} does not exist in the current mesh"),
            });
        }
        if condition_values(condition)
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err(WorkbenchError::Case(
                IncompressibleCaseError::InvalidInitialConditions,
            ));
        }
        self.boundaries.insert(patch.to_string(), condition);
        self.status = SolveStatus::Idle;
        self.solution = None;
        Ok(())
    }

    /// Stores a boundary condition for an existing Named Selection before mesh
    /// generation. `install_mesh` retains it only when Gmsh preserves the patch.
    pub fn configure_named_boundary(
        &mut self,
        selection: &str,
        condition: IncompressibleBoundaryCondition,
    ) -> Result<(), WorkbenchError> {
        if self.cyclic_partner(selection).is_some() {
            return Err(WorkbenchError::InvalidGrouping {
                message: "Remove the cyclic pair before assigning a physical condition".into(),
            });
        }
        if self.named_selections.get(selection).is_none() {
            return Err(WorkbenchError::InvalidGrouping {
                message: format!("Named Selection {selection:?} does not exist"),
            });
        }
        if condition_values(condition)
            .iter()
            .any(|value| !value.is_finite())
        {
            return Err(WorkbenchError::Case(
                IncompressibleCaseError::InvalidInitialConditions,
            ));
        }
        self.boundaries.insert(selection.to_string(), condition);
        self.status = SolveStatus::Idle;
        self.solution = None;
        Ok(())
    }

    pub fn unassign_boundary(&mut self, patch: &str) -> bool {
        let removed = self.boundaries.remove(patch).is_some();
        if removed {
            self.solution = None;
            self.status = SolveStatus::Idle;
        }
        removed
    }

    pub fn boundary_assignment(&self, patch: &str) -> Option<&IncompressibleBoundaryCondition> {
        self.boundaries.get(patch)
    }

    pub fn cyclic_partner(&self, name: &str) -> Option<&str> {
        self.solver.cyclic_pairs.iter().find_map(|p| {
            if p.first == name {
                Some(p.second.as_str())
            } else if p.second == name {
                Some(p.first.as_str())
            } else {
                None
            }
        })
    }
    pub fn set_cyclic_pairs(
        &mut self,
        pairs: Vec<crate::periodic::TranslationalCyclicPair>,
    ) -> Result<(), WorkbenchError> {
        let mesh = self
            .mesh
            .as_ref()
            .ok_or_else(|| WorkbenchError::InvalidGrouping {
                message: "Generate the mesh before pairing cyclic patches".into(),
            })?;
        let mut settings = self.solver.clone();
        settings.cyclic_pairs = pairs.clone();
        settings
            .resolve_cyclic(&mesh.mesh)
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        for p in &pairs {
            self.boundaries.remove(&p.first);
            self.boundaries.remove(&p.second);
        }
        self.solver.cyclic_pairs = pairs;
        self.solution = None;
        self.status = SolveStatus::Idle;
        Ok(())
    }
    pub fn set_volume_models(
        &mut self,
        models: crate::volume_models::VolumeModels,
    ) -> Result<(), WorkbenchError> {
        models
            .validate()
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        if self.solver.volume_models != models {
            self.solution = None;
            self.status = SolveStatus::Idle;
            self.solver.volume_models = models;
        }
        Ok(())
    }
    pub fn set_pressure_coupling(
        &mut self,
        c: crate::simple::PressureCoupling,
    ) -> Result<(), WorkbenchError> {
        c.validate(self.solver.time.is_transient())
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        if self.solver.pressure_coupling != c {
            self.solution = None;
            self.status = SolveStatus::Idle;
            self.solver.pressure_coupling = c;
        }
        Ok(())
    }
    pub fn set_equation_solvers(
        &mut self,
        e: crate::equations::EquationSolvers,
    ) -> Result<(), WorkbenchError> {
        e.validate()
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        if self.solver.equations.as_ref() != Some(&e) {
            self.solution = None;
            self.status = SolveStatus::Idle;
            self.solver.equations = Some(e);
        }
        Ok(())
    }
    pub fn set_boundary_profile(
        &mut self,
        name: &str,
        profile: Option<crate::unstructured_incompressible::BoundaryProfile>,
    ) -> Result<(), WorkbenchError> {
        if let Some(p) = &profile {
            let bc = *self
                .boundaries
                .get(name)
                .ok_or_else(|| WorkbenchError::InvalidGrouping {
                    message: "Assign a physical boundary first".into(),
                })?;
            if p.quantity.is_thermal() {
                let thermal = *self.solver.thermal.boundaries.get(name).ok_or_else(|| {
                    WorkbenchError::InvalidGrouping {
                        message: "Assign the thermal boundary before its time table".into(),
                    }
                })?;
                p.apply_thermal(thermal, 0.0)
                    .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
            } else {
                p.apply(bc, 0.0)
                    .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
            }
        }
        if self.solver.boundary_profiles.get(name) != profile.as_ref() {
            if let Some(p) = profile {
                self.solver.boundary_profiles.insert(name.into(), p);
            } else {
                self.solver.boundary_profiles.remove(name);
            }
            self.solution = None;
            self.status = SolveStatus::Idle;
        }
        Ok(())
    }
    pub fn material(&self) -> &IncompressibleMaterial {
        &self.material
    }

    pub fn set_user_material(&mut self, rho: f64, nu: f64) -> Result<(), WorkbenchError> {
        self.set_material(rho, nu)?;
        self.catalog_material = None;
        Ok(())
    }
    pub fn set_thermal(&mut self, options: crate::thermal::Options) -> Result<(), WorkbenchError> {
        options
            .validate()
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        if options != self.solver.thermal {
            self.solution = None;
            self.status = SolveStatus::Idle;
            self.solver.thermal = options;
        }
        Ok(())
    }
    pub fn apply_catalog_material(
        &mut self,
        material: crate::materials::MaterialSnapshot,
    ) -> Result<(), WorkbenchError> {
        material
            .validate()
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        let mut thermal = self.solver.thermal.clone();
        thermal.specific_heat = material.specific_heat;
        thermal.conductivity = material.conductivity;
        thermal.expansion = material.expansion;
        thermal.reference_temperature = material.temperature;
        // Validate before modifying either the fluid or thermal state.
        thermal
            .validate()
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        let changed = self.catalog_material.as_ref() != Some(&material);
        self.set_material(material.density, material.kinematic_viscosity())?;
        self.set_thermal(thermal)?;
        if changed {
            self.catalog_history
                .push(serde_json::json!({"kind":"fluid-material","evaluated_snapshot":material}));
        }
        self.catalog_material = Some(material);
        Ok(())
    }
    pub fn set_material(
        &mut self,
        density: f64,
        kinematic_viscosity: f64,
    ) -> Result<(), WorkbenchError> {
        if !(density.is_finite() && density > 0.0) {
            return Err(WorkbenchError::Case(
                IncompressibleCaseError::InvalidDensity { value: density },
            ));
        }
        if !(kinematic_viscosity.is_finite() && kinematic_viscosity >= 0.0) {
            return Err(WorkbenchError::Case(
                IncompressibleCaseError::InvalidKinematicViscosity {
                    value: kinematic_viscosity,
                },
            ));
        }
        if self.material.density != density
            || self.material.kinematic_viscosity != kinematic_viscosity
        {
            self.solution = None;
            self.status = SolveStatus::Idle;
            self.catalog_material = None;
        }
        self.material = IncompressibleMaterial {
            density,
            kinematic_viscosity,
        };
        Ok(())
    }

    /// Zero selects all CPUs available to this process. The limit applies per solve.
    pub fn set_compute_options(
        &mut self,
        options: crate::compute::ComputeOptions,
    ) -> Result<(), String> {
        options.validate()?;
        self.solver.compute = options;
        Ok(())
    }

    pub fn set_cpu_threads(&mut self, threads: usize) {
        self.solver.threads = threads;
    }

    pub fn set_convection(&mut self, convection: crate::momentum::MomentumConvection) {
        if self.solver.convection != convection {
            self.solution = None;
            self.status = SolveStatus::Idle;
        }
        self.solver.convection = convection;
    }

    pub fn set_flow_models(
        &mut self,
        time: crate::unstructured_incompressible::TimeIntegration,
        turbulence: crate::turbulence::TurbulenceOptions,
    ) -> Result<(), WorkbenchError> {
        time.validate()
            .and_then(|_| turbulence.validate())
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        for name in turbulence.inlets.keys() {
            if self.named_selections.get(name).is_none() {
                return Err(WorkbenchError::InvalidGrouping {
                    message: format!("Unknown turbulence boundary {name}"),
                });
            }
            if matches!(
                self.boundaries.get(name),
                Some(
                    IncompressibleBoundaryCondition::NoSlipWall
                        | IncompressibleBoundaryCondition::MovingWall { .. }
                        | IncompressibleBoundaryCondition::Slip
                        | IncompressibleBoundaryCondition::Symmetry
                )
            ) {
                return Err(WorkbenchError::InvalidGrouping {
                    message: format!("Incoming turbulence requires an open boundary, not {name}"),
                });
            }
        }
        if matches!(
            turbulence.model,
            crate::turbulence::TurbulenceModel::Smagorinsky
                | crate::turbulence::TurbulenceModel::Wale
        ) && (matches!(
            time,
            crate::unstructured_incompressible::TimeIntegration::Steady
        ) || self.mesh_dimension != MeshDimension::ThreeD)
        {
            return Err(WorkbenchError::InvalidGrouping {
                message: "LES requires transient integration and a 3D mesh".into(),
            });
        }
        if turbulence.model.uses_epsilon()
            && turbulence.wall_treatment != crate::turbulence::WallTreatment::EquilibriumKEpsilon
            && self.boundaries.values().any(|b| {
                matches!(
                    b,
                    IncompressibleBoundaryCondition::NoSlipWall
                        | IncompressibleBoundaryCondition::MovingWall { .. }
                )
            })
        {
            return Err(WorkbenchError::InvalidGrouping {
                message:
                    "Select equilibrium wall treatment for k-epsilon walls, or use resolved SST"
                        .into(),
            });
        }
        if self.solver.time != time || self.solver.turbulence != turbulence {
            self.solution = None;
            self.status = SolveStatus::Idle;
        }
        self.solver.time = time;
        self.solver.turbulence = turbulence;
        Ok(())
    }
    pub fn solver_options(&self) -> &IncompressibleSolverOptions {
        &self.solver
    }

    /// Validates and stores SIMPLE solver controls exposed by the UI panel.
    pub fn set_solver_controls(
        &mut self,
        max_outer_iterations: usize,
        velocity_relaxation: f64,
        pressure_relaxation: f64,
        continuity_absolute_tolerance: f64,
    ) -> Result<(), WorkbenchError> {
        let invalid = |message: &'static str| WorkbenchError::InvalidGrouping {
            message: message.to_string(),
        };
        if max_outer_iterations == 0 {
            return Err(invalid("max iterations must be at least 1"));
        }
        if !(velocity_relaxation.is_finite()
            && velocity_relaxation > 0.0
            && velocity_relaxation <= 1.0)
        {
            return Err(invalid("velocity relaxation must be finite in (0, 1]"));
        }
        if !(pressure_relaxation.is_finite()
            && pressure_relaxation > 0.0
            && pressure_relaxation <= 1.0)
        {
            return Err(invalid("pressure relaxation must be finite in (0, 1]"));
        }
        if !(continuity_absolute_tolerance.is_finite() && continuity_absolute_tolerance > 0.0) {
            return Err(invalid("continuity tolerance must be finite and positive"));
        }
        if self.solver.max_outer_iterations != max_outer_iterations
            || self.solver.velocity_relaxation != velocity_relaxation
            || self.solver.pressure_relaxation != pressure_relaxation
            || self.solver.continuity_absolute_tolerance != continuity_absolute_tolerance
        {
            self.solution = None;
            self.status = SolveStatus::Idle;
        }
        self.solver.max_outer_iterations = max_outer_iterations;
        self.solver.velocity_relaxation = velocity_relaxation;
        self.solver.pressure_relaxation = pressure_relaxation;
        self.solver.continuity_absolute_tolerance = continuity_absolute_tolerance;
        Ok(())
    }

    /// Run/Solve enable rule. The backend remains the final validation authority.
    pub fn readiness(&self) -> Result<(), String> {
        let Some(generated) = &self.mesh else {
            return Err("generate a mesh first (Mesh stage)".to_string());
        };
        if !self.solver.cyclic_pairs.is_empty() {
            self.solver.resolve_cyclic(&generated.mesh)?;
        }
        if !self.solver.boundary_profiles.is_empty() && !self.solver.time.is_transient() {
            return Err("Time-dependent boundaries require transient integration".into());
        }
        for (name, p) in &self.solver.boundary_profiles {
            let bc = *self
                .boundaries
                .get(name)
                .ok_or_else(|| format!("Missing physical boundary for time table: {name}"))?;
            if p.quantity.is_thermal() {
                if !self.solver.thermal.enabled {
                    return Err("Enable temperature transport for thermal time tables".into());
                }
                let t = *self
                    .solver
                    .thermal
                    .boundaries
                    .get(name)
                    .ok_or_else(|| format!("Assign thermal boundary {name}"))?;
                p.apply_thermal(t, 0.0)?;
            } else {
                p.apply(bc, 0.0)?;
            }
        }
        if let Some(e) = &self.solver.equations {
            e.validate()?;
        }
        self.solver.volume_models.validate()?;
        self.solver
            .pressure_coupling
            .validate(self.solver.time.is_transient())?;
        self.solver.turbulence.validate()?;
        for name in self.solver.turbulence.inlets.keys() {
            if !generated
                .mesh
                .boundary_patches()
                .iter()
                .any(|p| &p.name == name)
            {
                return Err(format!("Unknown turbulence boundary {name}"));
            }
            if matches!(
                self.boundaries.get(name),
                Some(
                    IncompressibleBoundaryCondition::NoSlipWall
                        | IncompressibleBoundaryCondition::MovingWall { .. }
                        | IncompressibleBoundaryCondition::Slip
                        | IncompressibleBoundaryCondition::Symmetry
                )
            ) {
                return Err(format!(
                    "Clear incoming turbulence on impermeable boundary {name}"
                ));
            }
        }
        self.solver.thermal.validate()?;
        if self.solver.thermal.enabled
            && self.solver.thermal.wall_treatment
                == crate::thermal::ThermalWallTreatment::EquilibriumPrt
            && (self.solver.turbulence.model != crate::turbulence::TurbulenceModel::KEpsilon
                || self.solver.turbulence.wall_treatment
                    != crate::turbulence::WallTreatment::EquilibriumKEpsilon)
        {
            return Err(
                "Equilibrium Prt thermal walls require equilibrium k-epsilon momentum walls".into(),
            );
        }
        if self.solver.thermal.enabled {
            let missing = generated
                .mesh
                .boundary_patches()
                .iter()
                .filter(|p| !self.solver.thermal.boundaries.contains_key(&p.name))
                .map(|p| p.name.clone())
                .collect::<Vec<_>>();
            if !missing.is_empty() {
                return Err(format!(
                    "Assign thermal boundary conditions: {}",
                    missing.join(", ")
                ));
            }
        }
        let quality = generated.mesh.quality();
        if !quality.min_cell_volume.is_finite()
            || quality.min_cell_volume <= 0.0
            || !quality.max_non_orthogonality_degrees.is_finite()
            || quality.max_non_orthogonality_degrees >= 90.0
        {
            return Err("Invalid mesh geometry: check positive cell measures and owner-to-neighbour face orientation before solving.".into());
        }
        let mut uncovered = 0_usize;
        let mut covered = std::collections::BTreeSet::new();
        for patch in generated.mesh.boundary_patches() {
            for &face in &patch.face_indices {
                covered.insert(face);
            }
        }
        for (index, face) in generated.mesh.faces().iter().enumerate() {
            if face.neighbour.is_none() && !covered.contains(&index) {
                uncovered += 1;
            }
        }
        if uncovered > 0 {
            return Err(format!(
                "{uncovered} exterior faces have no Named Selection / physical group"
            ));
        }
        let missing: Vec<&str> = generated
            .mesh
            .boundary_patches()
            .iter()
            .map(|patch| patch.name.as_str())
            .filter(|name| {
                !self.boundaries.contains_key(*name) && self.cyclic_partner(name).is_none()
            })
            .collect();
        if !missing.is_empty() {
            return Err(format!(
                "assign boundary conditions to every patch; unassigned: {}",
                missing.join(", ")
            ));
        }
        if self.material.density <= 0.0 || !self.material.density.is_finite() {
            return Err("density must be finite and positive".to_string());
        }
        if self.material.kinematic_viscosity < 0.0 || !self.material.kinematic_viscosity.is_finite()
        {
            return Err("kinematic viscosity must be finite and non-negative".to_string());
        }
        Ok(())
    }

    /// Builds an owned case for a worker thread. Fails fast with structured
    /// errors when the run rules are not satisfied.
    pub fn prepare_case(&self) -> Result<IncompressibleCase, WorkbenchError> {
        if let Err(reason) = self.readiness() {
            return Err(WorkbenchError::InvalidGrouping { message: reason });
        }
        let generated = self.mesh.as_ref().expect("readiness verified the mesh");
        let preflight = super::diagnostics::mesh_preflight(&generated.mesh);
        if preflight.blocked() {
            return Err(WorkbenchError::InvalidGrouping {
                message: preflight.summary(),
            });
        }
        let boundaries = generated
            .mesh
            .boundary_patches()
            .iter()
            .filter_map(|patch| {
                self.boundaries
                    .get(&patch.name)
                    .map(|condition| (patch.name.clone(), *condition))
            })
            .collect();
        let case = IncompressibleCase::steady(
            generated.mesh.clone(),
            boundaries,
            self.material,
            self.solver.clone(),
        );
        case.solver
            .volume_models
            .validate_case(&case)
            .map_err(|message| WorkbenchError::InvalidGrouping { message })?;
        Ok(case)
    }

    // ---------------------------------------------------------------
    // Solution lifecycle
    // ---------------------------------------------------------------

    /// Clear the current solution while retaining geometry, mesh and boundary setup.
    pub fn initialize_solution(&mut self) {
        self.performance = None;
        self.solution = None;
        self.status = SolveStatus::Idle;
    }

    pub fn mark_solving(&mut self) {
        self.performance = None;
        self.status = SolveStatus::Solving;
    }

    pub fn complete_solve(
        &mut self,
        outcome: Result<IncompressibleSolution, crate::IncompressibleSolveError>,
    ) {
        match outcome {
            Ok(solution) => {
                self.performance = Some(solution.report.performance.clone());
                self.status = if solution.report.converged() {
                    SolveStatus::Converged
                } else {
                    SolveStatus::MaxIterations
                };
                self.solution = Some(solution);
            }
            Err(error) => {
                self.solution = None;
                self.status = SolveStatus::Failed(error.to_string());
            }
        }
    }

    pub fn status(&self) -> &SolveStatus {
        &self.status
    }

    pub fn performance(&self) -> Option<&crate::compute::PerformanceReport> {
        self.performance.as_ref()
    }
    pub fn set_performance(&mut self, performance: crate::compute::PerformanceReport) {
        self.performance = Some(performance);
    }

    pub fn solution(&self) -> Option<&IncompressibleSolution> {
        self.solution.as_ref()
    }

    /// Exports the current solution through the existing legacy VTK path.
    pub fn export_vtk(&self, path: &std::path::Path) -> Result<(), String> {
        let Some(generated) = &self.mesh else {
            return Err("nothing to export: generate a mesh first".to_string());
        };
        let Some(solution) = &self.solution else {
            return Err("nothing to export: run the solver first".to_string());
        };
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|error| format!("cannot create {}: {error}", parent.display()))?;
        }
        write_unstructured_legacy_vtk(path, "FLURSYS workbench run", &generated.mesh, solution)
    }
}

fn join_tags(tags: &[usize]) -> String {
    tags.iter()
        .map(usize::to_string)
        .collect::<Vec<_>>()
        .join(", ")
}

fn condition_values(condition: IncompressibleBoundaryCondition) -> Vec<f64> {
    let values = match condition {
        IncompressibleBoundaryCondition::ParabolicInlet {
            velocity,
            axis,
            center,
            half_width,
        } => {
            return vec![
                velocity.x,
                velocity.y,
                velocity.z,
                axis as f64,
                center,
                half_width,
            ]
        }
        IncompressibleBoundaryCondition::NoSlipWall
        | IncompressibleBoundaryCondition::Slip
        | IncompressibleBoundaryCondition::Symmetry => [0.0; 4],
        IncompressibleBoundaryCondition::FlowRateInlet { rate, mass } => {
            [rate, if mass { 1.0 } else { 0.0 }, 0.0, 0.0]
        }
        IncompressibleBoundaryCondition::PressureOutletBackflow { velocity, pressure }
        | IncompressibleBoundaryCondition::FarField { velocity, pressure } => {
            [velocity.x, velocity.y, velocity.z, pressure]
        }
        IncompressibleBoundaryCondition::MovingWall { velocity }
        | IncompressibleBoundaryCondition::VelocityInlet { velocity } => {
            [velocity.x, velocity.y, velocity.z, 0.0]
        }
        IncompressibleBoundaryCondition::TotalPressure { pressure }
        | IncompressibleBoundaryCondition::PressureOutlet { pressure } => [pressure, 0.0, 0.0, 0.0],
    };
    values.to_vec()
}

fn boundary_condition_to_document(
    condition: IncompressibleBoundaryCondition,
) -> PhysicalBoundaryCondition {
    match condition {
        IncompressibleBoundaryCondition::TotalPressure { pressure } => {
            PhysicalBoundaryCondition::TotalPressure { pressure }
        }
        IncompressibleBoundaryCondition::ParabolicInlet {
            velocity,
            axis,
            center,
            half_width,
        } => PhysicalBoundaryCondition::ParabolicInlet {
            velocity,
            axis,
            center,
            half_width,
        },
        IncompressibleBoundaryCondition::FlowRateInlet { rate, mass } => {
            PhysicalBoundaryCondition::FlowRateInlet { rate, mass }
        }
        IncompressibleBoundaryCondition::PressureOutletBackflow { pressure, velocity } => {
            PhysicalBoundaryCondition::PressureOutletBackflow { pressure, velocity }
        }
        IncompressibleBoundaryCondition::Slip => PhysicalBoundaryCondition::Slip,
        IncompressibleBoundaryCondition::Symmetry => PhysicalBoundaryCondition::Symmetry,
        IncompressibleBoundaryCondition::FarField { velocity, pressure } => {
            PhysicalBoundaryCondition::FarField { velocity, pressure }
        }
        IncompressibleBoundaryCondition::NoSlipWall => PhysicalBoundaryCondition::NoSlipWall,
        IncompressibleBoundaryCondition::MovingWall { velocity } => {
            PhysicalBoundaryCondition::MovingWall { velocity }
        }
        IncompressibleBoundaryCondition::VelocityInlet { velocity } => {
            PhysicalBoundaryCondition::VelocityInlet { velocity }
        }
        IncompressibleBoundaryCondition::PressureOutlet { pressure } => {
            PhysicalBoundaryCondition::PressureOutlet { pressure }
        }
    }
}

fn boundary_condition_from_document(
    condition: &PhysicalBoundaryCondition,
) -> IncompressibleBoundaryCondition {
    match *condition {
        PhysicalBoundaryCondition::TotalPressure { pressure } => {
            IncompressibleBoundaryCondition::TotalPressure { pressure }
        }
        PhysicalBoundaryCondition::ParabolicInlet {
            velocity,
            axis,
            center,
            half_width,
        } => IncompressibleBoundaryCondition::ParabolicInlet {
            velocity,
            axis,
            center,
            half_width,
        },
        PhysicalBoundaryCondition::FlowRateInlet { rate, mass } => {
            IncompressibleBoundaryCondition::FlowRateInlet { rate, mass }
        }
        PhysicalBoundaryCondition::PressureOutletBackflow { pressure, velocity } => {
            IncompressibleBoundaryCondition::PressureOutletBackflow { pressure, velocity }
        }
        PhysicalBoundaryCondition::Slip => IncompressibleBoundaryCondition::Slip,
        PhysicalBoundaryCondition::Symmetry => IncompressibleBoundaryCondition::Symmetry,
        PhysicalBoundaryCondition::FarField { velocity, pressure } => {
            IncompressibleBoundaryCondition::FarField { velocity, pressure }
        }
        PhysicalBoundaryCondition::NoSlipWall => IncompressibleBoundaryCondition::NoSlipWall,
        PhysicalBoundaryCondition::MovingWall { velocity } => {
            IncompressibleBoundaryCondition::MovingWall { velocity }
        }
        PhysicalBoundaryCondition::VelocityInlet { velocity } => {
            IncompressibleBoundaryCondition::VelocityInlet { velocity }
        }
        PhysicalBoundaryCondition::PressureOutlet { pressure } => {
            IncompressibleBoundaryCondition::PressureOutlet { pressure }
        }
    }
}
