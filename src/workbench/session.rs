//! Workbench session state: the presentation-layer controller that bridges the
//! Slint UI with the stable geometry topology, the Gmsh meshing backend, and
//! the high-level incompressible solver.
//!
//! The session owns validated domain state. The UI issues commands and renders
//! what this module reports; it never stores duplicate geometry/mesh/BC state.

use super::{
    GeometrySelectionTarget, MeshRenderCache, MeshSelection, NamedSelectionError,
    NamedSelectionStore,
};
use crate::{
    output::write_unstructured_legacy_vtk, BoxEntities, CircleHoleEntities, EdgeId, FaceId,
    GeneratedMesh, GeometryError, GeometryGmshExport, GeometryTopology, GmshGeometryExporter,
    GmshMeshOptions, GmshMesher, IncompressibleBoundaryCondition, IncompressibleCase,
    IncompressibleCaseError, IncompressibleMaterial, IncompressibleSolution,
    IncompressibleSolverOptions, MeshDimension, MeshingError, RectangleEntities, Vec3,
};
use std::collections::BTreeMap;

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
    rectangle: Option<RectangleEntities>,
    box_body: Option<BoxEntities>,
    named_selections: NamedSelectionStore,
    mesh_dimension: MeshDimension,
    global_size: f64,
    min_size: f64,
    max_size: f64,
    element_order: u8,
    mesher: GmshMesher,
    mesh: Option<GeneratedMesh>,
    mesh_render_cache: Option<MeshRenderCache>,
    mesh_selection: Option<MeshSelection>,
    mesh_hover: Option<MeshSelection>,
    boundaries: BTreeMap<String, IncompressibleBoundaryCondition>,
    material: IncompressibleMaterial,
    solver: IncompressibleSolverOptions,
    solution: Option<IncompressibleSolution>,
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
            rectangle: None,
            box_body: None,
            named_selections: NamedSelectionStore::default(),
            mesh_dimension: MeshDimension::TwoD,
            global_size: 0.1,
            min_size: 0.05,
            max_size: 0.2,
            element_order: 1,
            mesher: GmshMesher::auto(),
            mesh: None,
            mesh_render_cache: None,
            mesh_selection: None,
            mesh_hover: None,
            boundaries: BTreeMap::new(),
            material: IncompressibleMaterial {
                density: 1.0,
                kinematic_viscosity: 0.01,
            },
            solver: IncompressibleSolverOptions::default(),
            solution: None,
            status: SolveStatus::Idle,
        }
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

    /// Invalidates all products derived from geometry. View and selection
    /// operations deliberately never call this method.
    pub fn geometry_changed(&mut self) {
        self.mesh = None;
        self.mesh_render_cache = None;
        self.mesh_selection = None;
        self.mesh_hover = None;
        self.solution = None;
        self.status = SolveStatus::Idle;
        self.patch_names_clear();
        let geometry = &self.geometry;
        self.named_selections.retain_targets(|target| match target {
            GeometrySelectionTarget::Vertex(id) => geometry.vertex(id).is_some(),
            GeometrySelectionTarget::Edge(id) => geometry.edge(id).is_some(),
            GeometrySelectionTarget::Face(id) => geometry.face(id).is_some(),
            GeometrySelectionTarget::Body(id) => geometry.body(id).is_some(),
        });
        self.rectangle = self
            .rectangle
            .take()
            .filter(|rectangle| self.geometry.face(rectangle.face).is_some());
        self.box_body = self
            .box_body
            .take()
            .filter(|box_body| self.geometry.body(box_body.body).is_some());
    }

    fn patch_names_clear(&mut self) {
        self.boundaries.clear();
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

    pub fn fluid_face(&self) -> Option<FaceId> {
        self.rectangle
            .as_ref()
            .map(|rectangle| rectangle.face)
            .or_else(|| {
                self.geometry.faces().find_map(|face| {
                    matches!(
                        face.representation,
                        crate::GeometryFaceRepresentation::Planar { .. }
                    )
                    .then_some(face.id)
                })
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
        self.status = SolveStatus::Idle;
        self.solution = None;
        Ok(())
    }

    pub fn rename_named_selection(
        &mut self,
        old_name: &str,
        new_name: &str,
    ) -> Result<(), WorkbenchError> {
        self.named_selections.rename(old_name, new_name)?;
        Ok(())
    }

    pub fn delete_named_selection(&mut self, name: &str) -> bool {
        self.named_selections.delete(name)
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

    /// Builds the deterministic `.geo` export proving Named Selection →
    /// `GeometryToGmshMap` → Physical Curve/Surface mapping input correctness.
    pub fn build_gmsh_export(&self) -> Result<GeometryGmshExport, WorkbenchError> {
        match self.mesh_dimension {
            MeshDimension::TwoD => {
                let face = self.fluid_face().ok_or(WorkbenchError::NoGeometry {
                    needed: "planar fluid face",
                })?;
                Ok(GmshGeometryExporter::planar(
                    &self.geometry,
                    face,
                    self.boundary_groups_2d()?,
                    "fluid",
                )?)
            }
            MeshDimension::ThreeD => {
                let body = self
                    .box_body
                    .as_ref()
                    .ok_or(WorkbenchError::NoGeometry { needed: "box body" })?;
                Ok(GmshGeometryExporter::rectangular_box(
                    &self.geometry,
                    body.body,
                    self.boundary_groups_3d()?,
                    "fluid",
                )?)
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
            dimension,
            characteristic_length: global_size,
            min_size: Some(min_size),
            max_size: Some(max_size),
            element_order,
        };
        options.validate()?;
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
        let export = self.build_gmsh_export()?;
        let options = GmshMeshOptions {
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
            .unwrap_or_default()
    }

    pub fn assign_boundary(
        &mut self,
        patch: &str,
        condition: IncompressibleBoundaryCondition,
    ) -> Result<(), WorkbenchError> {
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
        self.boundaries.remove(patch).is_some()
    }

    pub fn boundary_assignment(&self, patch: &str) -> Option<&IncompressibleBoundaryCondition> {
        self.boundaries.get(patch)
    }

    pub fn material(&self) -> &IncompressibleMaterial {
        &self.material
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
        self.material = IncompressibleMaterial {
            density,
            kinematic_viscosity,
        };
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
            .filter(|name| !self.boundaries.contains_key(*name))
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
        Ok(IncompressibleCase::steady(
            generated.mesh.clone(),
            boundaries,
            self.material,
            self.solver.clone(),
        ))
    }

    // ---------------------------------------------------------------
    // Solution lifecycle
    // ---------------------------------------------------------------

    pub fn mark_solving(&mut self) {
        self.status = SolveStatus::Solving;
    }

    pub fn complete_solve(
        &mut self,
        outcome: Result<IncompressibleSolution, crate::IncompressibleSolveError>,
    ) {
        match outcome {
            Ok(solution) => {
                self.status = if solution.report.converged() {
                    SolveStatus::Converged
                } else {
                    SolveStatus::MaxIterations
                };
                self.solution = Some(solution);
            }
            Err(error) => {
                self.status = SolveStatus::Failed(error.to_string());
            }
        }
    }

    pub fn status(&self) -> &SolveStatus {
        &self.status
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

fn condition_values(condition: IncompressibleBoundaryCondition) -> [f64; 4] {
    match condition {
        IncompressibleBoundaryCondition::NoSlipWall => [0.0; 4],
        IncompressibleBoundaryCondition::MovingWall { velocity }
        | IncompressibleBoundaryCondition::VelocityInlet { velocity } => {
            [velocity.x, velocity.y, velocity.z, 0.0]
        }
        IncompressibleBoundaryCondition::PressureOutlet { pressure } => [pressure, 0.0, 0.0, 0.0],
    }
}
