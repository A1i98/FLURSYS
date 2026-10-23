# Workbench project workspace

Implementation audit: 2026-10-21, including current working-tree changes. For equations, units, residuals and numerical
limitations, see [Numerical Methods](NUMERICAL_METHODS.md). This page describes
the CAD/Gmsh/steady-SIMPLE workflow, not the structured transient CLI backend.

The unstructured workbench uses a directory workspace. Its canonical portable document is
`project.json`, represented by `WorkbenchProject`; it stores validated user intent and run
metadata, not generated runtime state.

```text
MyCase.flursys/
├── project.json
├── autosave/
│   └── project.json          # separate recovery snapshot, when written
├── meshes/
│   └── <sha256>.json          # immutable current/recovery mesh assets
├── monitors/
│   └── solve-<id>/<encoded-plane-id>/  # optional plane.json and iteration CSV/SVG
└── runs/
    └── run-0001/
        ├── report.json
        └── solution.vtk      # only when the run produced an exportable solution
```

`save_workspace()` creates the workspace and `runs/`, then atomically writes `project.json`:
it serializes to a temporary sibling, syncs it, and renames it into place. Project data uses
workspace-relative artifact paths only; absolute paths and parent-directory traversal in run
artifact references are rejected. Generated meshes, VTK fields, render caches, worker handles,
and transient UI state are deliberately excluded from `project.json`. The optional
`mesh_asset` identifier references the saved mesh in `meshes/`; the mesh data itself
is stored separately.

## Schema and versioning

The current workspace format is version **2** (`WORKBENCH_PROJECT_FORMAT_VERSION`). A document
contains:

- `format_version`, generated `project_id`, and display `name`;
- complete `GeometryTopology`, including stable IDs, revision, and allocator counters;
- Named Selections targeting stable geometry IDs;
- Gmsh mesh intent (`dimension`, global/min/max size, first-order element order);
- `mesh_recipe`: algorithm/quality tuning, scoped sizes, distance fields, 2D inflation
  and 3D volume-refinement boxes;
- optional native `cad_document` and the last transferred `cad_geometry_hash`;
- Named-Selection boundary assignments, density/viscosity, SIMPLE controls and the
  selected `Upwind` or `LimitedLinear` momentum convection;
- `plane_monitors` (ID, name, origin, orthonormal axes, enabled flag and interval); and
- persistent `runs` metadata.

Loading validates the complete document, including geometry, selection targets, numerical
settings, and artifact paths. Workspace v1 documents migrate their missing mesh recipe to the
safe empty recipe; newer or otherwise mismatched versions return a structured error. A saved
project reconstructs a `WorkbenchSession` and the GUI restores its referenced mesh
without Gmsh. Solution fields remain separate run/restart artifacts. Older projects
without a mesh asset require regeneration once before saving the actual mesh.

## Save, open, and recovery API

The desktop's workspace load/save route takes a **folder** containing `project.json`; saving to
a legacy filename is rejected. `autosave_workspace()` is a separate, atomic API that writes
`autosave/project.json` and never replaces the canonical project document.
`recovery_is_newer()` compares the recovery and canonical modification times (and treats a
recovery without a canonical file as newer) so a caller can decide whether to offer recovery.

The GUI marks persistent workbench edits dirty, writes a conservative recovery snapshot for a
dirty saved workspace, and presents an explicit Recover/Discard choice when a newer recovery is
found. Camera, hover, and view-selection changes are transient and do not dirty the document.

## Templates

A case template is a `case.json` manifest in an immediate child directory of a template root:

```text
cases/templates/
├── blank-2d/case.json
├── blank-3d/case.json
├── lid-driven-cavity/case.json
├── laminar-channel/case.json
├── cylinder-flow/case.json
├── skewed-mesh-verification/case.json
└── channel-3d/case.json
```

`CaseTemplateManifest` contains its own format version, unique ID, name, category,
description, capability labels, expectation metadata, and a `WorkbenchProject` under
`project_template`. Expectations are gallery guidance only; they never alter solver input.
Discovery validates each manifest and its embedded project. Missing roots yield no entries;
malformed manifests and duplicate IDs are returned as individual errors instead of preventing
other templates from being discovered. The gallery combines the built-in root with the optional
user root `$XDG_DATA_HOME/flursys/cases/templates` (or `~/.local/share/flursys/cases/templates`).
**Save Template** writes to a chosen template-library folder, rejects an existing template ID,
and strips run history so the result is reusable intent rather than an artifact snapshot.

## CAD integration, transfer and meshing

Geometry hosts the native CAD workbench from the pinned
`vendor/cad-workbench` submodule. Its document owns sketches, constraints,
features, browser and command dialogs. The host saves its serialized document
separately from the CFD topology; the solver does not operate on viewport triangles.

The transfer evaluates the document, converts CAD millimetres to solver metres,
and imports exact supported solid B-reps or planar sketch profiles into the CFD
geometry model. `cad_geometry_hash` records the evaluated geometry fingerprint.
Adding a standalone construction plane does not invalidate a mesh solely by
changing document history. A body/profile geometry change requires the dependent
CFD products to be regenerated. This is not a guarantee that all named faces
survive every Boolean or topology-changing edit.

Select the fluid region and create Named Selections on geometry edges (2D) or
faces (3D) before meshing. A group name alone does not assign its physical BC.
Groups map to Gmsh physical names and imported boundary patches; dense mesh
triangles are not the persistent selection identity. Inspect the exported domain
and boundary coverage, especially after Boolean operations or geometry changes.

Meshing settings include dimension, global/min/max size, local sizing, refinement
and boundary-layer intent, plus a separate Gmsh thread request. The importer and
exporter support a subset of possible Gmsh/CAD outputs; unsupported entities or
cell types can fail explicitly. Meshing is not a guarantee of solver readiness.

### Boundary and solver setup

Available physical choices are no-slip, moving wall, velocity inlet, pressure
outlet, slip, symmetry, incompressible far-field, volume-flow inlet, mass-flow
inlet, pressure outlet with specified backflow velocity, and planar parabolic
inlet. The numerical-methods document specifies their exact behaviour.

Velocities use m/s. Pressure entries are **kinematic gauge pressure p/rho**, not
pascals. Positive rate means inward total flow; mass-flow conversion uses the
configured density. In 2D, rates are per unit depth. The parabolic inlet input is
peak velocity, with a transverse coordinate axis, centre and positive half-width
in metres; it is not a radial pipe profile or a flow-rate-normalized inlet.

Workbench solves constant-density flow with SIMPLE, BiCGStab momentum and
PCG/IC0 pressure correction. Experimental implicit-Euler transient integration
and wall-resolved k-omega/SST plus Smagorinsky/WALE closures are connected. Solution exposes upwind/limited linear
upwind, relaxation, convergence controls and worker threads. Apply settings stores
the form; Run solver starts the worker. Workbench does not expose full PISO/PIMPLE, BDF2 or heat transport. See
[TRANSIENT_TURBULENCE.md](TRANSIENT_TURBULENCE.md) for model limits and validation status.

### Plane monitors and result plots

Select a CAD plane, configure the recording interval and enable recording before
Run. Section data are sampled every N completed SIMPLE iterations (steady), or
every N accepted physical time steps (transient), and at final completion if not
already due. Transient recordings carry physical time in seconds. Results → CAD section shows the selected plane with speed, kinematic pressure,
velocity components or normal velocity. A colour bar, manual/automatic range,
optional vectors and cell edges apply to the view and manual SVG/PNG exports.
CSV always retains all recorded velocity components and kinematic pressure. Automatic recordings are CSV/SVG
plus `plane.json` in `monitors/solve-<id>/`.

These are cell-centred samples on a 3D convex-cell section, without spatial
smoothing or interpolation between recorded physical time levels. A steady iteration sequence is not a vortex-shedding
animation. Full vectors and kinematic pressure are retained in CSV; plotted arrows
show the in-plane projection. Convergence plot log/normalization options change
only the display, not stopping tolerances.

## Run artifacts and VTK scope

After a workspace-backed solve completes, the desktop creates `runs/run-NNNN/`, writes a
`report.json`, appends a `RunRecord` to `project.json`, and saves the workspace. Records contain
an ordinal/ID, timestamp, status, optional mesh identity and solver summary, a persisted failure
diagnostic when applicable, and relative report and optional solution paths. If an in-memory mesh
and solution are available, the run also writes `solution.vtk`; failed runs do not claim a
solution path.

Workbench result export is legacy ASCII VTK. It writes cell-centred `pressure`,
`velocity_magnitude`, and `velocity`. It supports polygonal 2D cells and **tetrahedral-only**
3D cells; 3D non-tetrahedral cells are rejected rather than emitted incorrectly.

## Legacy project policy

`*.flursys.json` is import-only. Selecting a legacy file inspects it and imports recoverable
intent into a new sibling workspace such as `channel.flursys/project.json`; the original file is
never modified. The importer supports the repository's historical structured project v1 and v2
schemas, recovering supported cavity, channel, cylinder, and backward-facing-step geometry into
canonical topology with stable IDs, Named Selections, compatible boundary conditions, material,
solver controls, and Gmsh mesh intent. Generated legacy meshes and historical runtime results are
intentionally not carried forward: the imported workspace starts with no mesh and no solution,
then uses the normal Gmsh, save, autosave, recovery, and run-artifact paths. All ongoing work is
saved exclusively as a workspace `project.json`.

## Current limitations and reproducibility

The native CAD workbench contains sketching, parametric solid/surface commands
and properties beyond the older primitive/extrusion-only interface. Availability
in its command registry does not guarantee successful evaluation of every model
or compatibility with the CFD transfer and Gmsh exporter. Some host topology,
selection and conversion adapters remain necessary for the mesh/setup workflow.

Saved intent and native CAD state are distinct from regenerated mesh/solver state.
Opening a project is not proof of a previously converged solution; inspect the
selected saved run and its status. Reaching the outer limit is not convergence,
and a low continuity residual alone does not satisfy all SIMPLE stopping criteria.

The recently added profile/rate/backflow boundaries, limited convection and IC0
integration have compile/build checks but no new complete CFD validation campaign.
Record both the FLURSYS commit and CAD submodule commit, plus uncommitted changes,
for reproducibility. The submodule is pinned; updating its tracking branch does
not automatically update the recorded project gitlink.

### Recorded CAD section playback

In Results, choose CAD section and select a plane. Open recording selects a plane-specific folder under `monitors/solve-*/<encoded-plane-id>/`; Play frames, previous/next, the frame slider and FPS replay the recorded field history. Steady sequences show iterative convergence; transient recordings show accepted physical time levels. New recordings retain section geometry, velocity and kinematic pressure in JSON beside CSV/SVG, so field selection, vector overlays and colour ranges remain editable during replay. Older SVG recordings replay their original appearance. Latest field returns to the current solution. PNG/SVG/CSV exports use the selected recorded frame; legacy SVG data can be exported as SVG/PNG, while its original CSV remains beside it.

Solution offers uniform Initialization with editable X/Y/Z velocity (m/s) and kinematic gauge pressure (m²/s²), Run / Continue, Pause/Resume and Stop together. Initialization clears the active solution while preserving archived runs, mesh and boundary settings. Pause cooperatively blocks the current worker at a solver control checkpoint and retains all solver state in memory; Resume continues that same solve. Stop cancels the solve, including a paused worker, while retaining velocity, pressure, conservative face flux and closure state from the last completed nonlinear iteration (steady), or last accepted physical time level (transient), in memory. Run / Continue uses retained fields on the same runtime mesh with the current boundary conditions and numerical settings; convergence history and solver workspaces are rebuilt; complete restarts retain conservative face flux. The iteration budget is additional for each Run and convergence may end it early. Set the budget to 200 to request up to 200 more iterations. Initialize explicitly replaces retained fields. Mesh replacement invalidates continuation. Pause / Resume retains the original worker and its settings; to change inlet or wall speed, Stop, edit the boundary condition, then Run / Continue. In-memory continuation does not survive application restart; use a field checkpoint after reopening.

### Solver preflight and conservation diagnostics

Before Run, fatal non-positive/non-finite cell measures and invalid internal face orientation block execution. The detailed report lists invalid cell/face IDs, faces above 70 degrees non-orthogonality, and the largest adjacent-cell measure ratio. A ratio above 10 and centre-line skewness above 0.5 produce advisory warnings. These thresholds are heuristics, not accuracy guarantees, and skewness definitions differ between tools. Full scans occur during preparation and report generation; the regularly refreshed mesh inspector uses cached quality metrics.

Completed runs save `mesh_preflight` and `conservation` in their report JSON. Solution/Results boundary reports include face-based inlet/outlet flow, net mass flux, relative imbalance normalized by the larger gross inlet/outlet flow, and volume-weighted absolute and signed divergence in inverse seconds. Zero-throughflow domains show relative imbalance as N/A; internal divergence remains informative for cavities. There is no physical time-step factor in these steady diagnostics. They supplement the existing convergence criteria.

Failure messages retain the original technical error and add contextual checks for momentum/pressure linear failures, invalid momentum diagonals, pressure-outlet backflow and singular gradient stencils. These are diagnostic suggestions and do not establish an unmeasured root cause.

### Field checkpoints and report export

The Solution toolbar includes Load restart. Each solve saves `monitors/solve-*/checkpoint.json` after its first completed iteration/time step, at the configured Save restart fields interval (default 100; 0 disables checkpoint writes and signature allocation), and on successful or iteration-limited completion. Atomic replacement retains the last complete file; failed writes are reported and do not intentionally abort the solve. Stop or process termination may lose iterations since the last completed save. New complete checkpoints contain geometry/connectivity, cell order, velocity, kinematic pressure, conservative face flux, turbulence state, physical time when applicable, iteration/step index and the configuration context. Load restart validates the current mesh and queues these fields for the next Run; Initialize clears a queued restart. A legacy velocity/pressure-only restart rebuilds face flux. Complete restarts preserve it. Both rebuild linear solvers and convergence history with current settings; displayed iteration/step numbering starts again, while saved physical time continues. It is distinct from an in-memory Pause/Resume and is not a bitwise continuation of SIMPLE.

Results → Info offers field-statistics CSV and a printable HTML report. The HTML includes archived settings and boundary assignments from the selected run, solver version when recorded, mesh metrics, conservative mass-balance diagnostics, field statistics and convergence charts. Missing historical settings are marked unavailable instead of replaced with current editable settings. Use the browser print dialog to save the HTML as PDF. New report JSON files use report format 2 and retain solver version.

### Plane vector visibility and automatic recording selection

CAD section automatically finds the selected run's recording and opens its last frame; Play starts from the first frame when already at the end. New run reports store the workspace-relative monitor folder. For older reports, discovery requires a unique solve folder inside that run's time window, matching plane geometry and the final iteration. Ambiguous folders require Open recording. Changing the selected run or plane refreshes the recording selection.

Normalize arrow length (default on) shows in-plane direction with a common length, making weak recirculation visible; length no longer represents speed. The colour field retains the actual selected scalar values. Turn normalization off for proportional arrows. Normal flow markers are optional and off by default. For a clear flow view choose Speed, enable Vectors and normalization, and hide Cell edges.

### Explicit meshing controls

In Mesh → Meshing controls, set sizes in metres and choose **Apply mesh settings**
(or Generate Mesh to apply and generate). The required order is minimum ≤ target
≤ maximum. The target now sets the geometry point sizes before local overrides;
it is not a cell count. Enabled local and distance-field sizes outside the global
bounds are rejected rather than silently clamped. Coarse/Medium/Fine are optional,
geometry-scaled drafts; in 3D the target is capped at one third of the smallest
positive model extent. These presets are starting estimates, not quality guarantees.
Use Uniform size to explicitly set both bounds to the target.

Algorithm & quality exposes triangle MeshAdapt/Delaunay/Frontal-Delaunay and
Delaunay/Frontal/HXT tetrahedron algorithms, curvature elements per full circle
(0 disables), smoothing iterations, boundary size extension, native tetrahedron
optimization and optional Netgen optimization. Availability and parallel speedup
depend on the installed Gmsh build. Optimization threshold is Gmsh's dimensionless
quality value, not a CFD non-orthogonality angle. Controls persist in
`mesh_recipe.tuning` and are included in subsequent run reports.

Local sizing and Distance refinement replace an existing control for the same
boundary group. Distance refinement exposes the fine-region distance, transition
end distance and distance sampling. Use a wider transition interval to reduce
size-field gradients; this does not impose a hard adjacent-cell-volume-ratio bound.
Inspect the generated mesh before solving. Boundary layers remain supported only
for 2D edges; no 3D prism-layer capability is implied by these controls.

### Scoped sizing and inflation

Mesh → Meshing controls → Scoped mesh controls accepts either a named boundary
group or the current geometry selection. In the latter mode, select edges (2D)
or faces (3D) in Geometry view, using Shift-click for multiple entities. Mesh
controls do not create or change physical boundary conditions. Give the control
an optional name, choose Sizing, Distance refinement or Inflation, then apply.
Click a saved control to load its values and highlight its scope; Edit, On/off
and Delete operate on that control. New control clears the editing ID. Applying
an edited control retains its ID; replacing a same-scope control avoids duplicates.
Suppression and edits invalidate the current mesh but retain historical Runs.

2D inflation accepts First layer height or Total thickness, growth 1–3 and 1–64
target layers. It shows the requested first/last heights and geometric stack
thickness. For total-thickness input, the first height is derived from the
geometric sum. The backend requests quadrilateral boundary layers; Gmsh can
truncate or alter the layer stack near intersections/corners, so target layer
count is not a verified generated count. Multiple enabled inflation scopes must
use matching height/growth/count and must not overlap; their curves are merged
into a single Gmsh BoundaryLayer field. The normal height is independent of the
isotropic surface-size inputs. No y+ target is inferred from laminar inputs.

**Limitation:** arbitrary 3D CAD prismatic inflation is not implemented. The
current 3D CAD exporter uses faceted surfaces; Gmsh's BoundaryLayer size field is
2D-only. Surface distance refinement is available in 3D and is explicitly not
labelled as inflation. A conformal 3D layer/core mesher with collision handling
requires a separate implementation.

### Mesh display contrast

The mesh viewer starts in Wireframe (transparent): cell interiors and empty
pixels have zero opacity, revealing the viewport background. Neutral edges are
light against the dark viewport; patch and quality edge colours are lifted for
visibility. Surface + edges retains filled cells with contrasting mesh lines.
Selected/hovered faces, cells and named patches retain their highlight colours.
Mesh images render at twice the logical preview resolution; picking and camera
navigation still use the original logical coordinate system. In 3D, wireframe
shows front and rear exterior mesh edges; it is not an interior volume slice.

### Volume refinement and internal quality inspection

A **Volume refinement box** defines an axis-aligned region with minimum/maximum
XYZ coordinates in metres, inside element size and outside transition thickness.
Start from model bounds fills the coordinate draft; adjust it to cover the wake
or other region of interest. The inside size must fit the global size bounds.
Gmsh Box fields are combined with distance fields by the minimum requested size;
the target global size is used outside the box. A zero transition is permitted
but creates an abrupt size-field change. Boxes persist in `mesh_recipe.regions`,
can be edited/suppressed/deleted, and are included in the saved mesh recipe report.
They are sizing regions, not embedded bodies or guaranteed structured blocks.

In Mesh view, **3D cell cutaway** retains whole cells whose centres lie on the
selected side of an X/Y/Z coordinate at a percentage of model bounds. Apply cut
or Reset cut changes only the visualization. The exposed surface follows mesh
faces and is stepped; it is not an interpolated planar section. Picking uses the
same retained cells so hidden exterior faces do not intercept selection.

Mesh quality shows min/mean/max, finite and non-finite counts, the count and
percentage above the current threshold, a numeric colour legend, and ten
equal-width histogram bins. The histogram is normalized to its largest bin;
statistics always cover the entire mesh, including cells hidden by cutaway.
Metrics are FLURSYS definitions, not interchangeable with other meshers' metrics.

### Workbench docking and mesh outline

Properties for Mesh, Setup, Solution and Results now precede their viewport on
the left. The project Browser remains a separate full-height column; Mesh Details
is a separate panel between the Browser and viewport, matching Setup.
The Mesh tree node opens boundary grouping; its Meshing controls and Mesh quality
children open the corresponding Details page. Saved sizing, distance, inflation
and volume-box controls appear under Meshing controls and load their editable
properties when selected. The outline and controls refer to the same persisted IDs.

The old Geometry/Mesh/Fit button strip above the mesh viewport is removed.
Display source and rendering style are properties in Details. Fit sits below
the viewport alongside Edge/Face/Vertex/Body selection. The embedded CAD command
panels and sketch palette use a host-selected left dock; while a command dialog
is active it takes the palette's slot, avoiding overlapping property panels.


### Saving and restoring the actual mesh

Save, Save As, autosave and run finalization now persist the active Workbench mesh
as an immutable `meshes/<sha256>.json` asset. `project.json` stores `mesh_asset`,
not the full mesh payload. Canonical and recovery documents can reference different
assets safely; recovery uses the same workspace asset directory. Templates omit
this workspace-local reference. Save As writes the mesh asset to the destination.

Assets include points, exact cell/face ordering, ownership, patch names/types and
Gmsh report metadata. Opening checks the content checksum, format and geometry/
meshing intent, rebuilds geometry through the validated mesh constructor, restores
original topology indexing and then creates a fresh runtime MeshId/render cache.
Material and physical-BC edits do not require remeshing. Geometry/group/sizing
changes invalidate the active mesh, so the next saved document has no active mesh
reference until regeneration. Unchanged meshes are not rewritten on every autosave.

A missing/corrupt/stale asset is reported and the geometry/setup still opens;
regeneration is then required. Old documents without `mesh_asset` remain readable.
Existing projects that never saved mesh data cannot recover an absent mesh from
settings alone: regenerate once and save with the updated application. This
feature restores the mesh; solver fields still use result/restart artifacts.


## Property navigation and adaptive time controls (2026-10-21)

CFD pages use a shared top section bar and one property scroll surface. Mesh
sections are Boundaries, Sizing, Quality, Refinement and View. Setup separates
Boundaries, Material and Readiness. Solution separates Initialize, Models, Time,
Methods, Reports and Planes. Results includes a Walls section. Geometry keeps its
native contextual CAD ribbon. The Browser remains a separate column.

Property content is measured in an independent extent; its height is not
constrained to the viewport. Offsets reset on section changes and clamp after
resize/content shrink. Property lists share the outer scroll surface so nested
list scrollbars cannot capture the wheel. Solution Apply stays below the
scrolling form. This structural change has not had interactive visual QA.

Solution → Time now offers fixed or adaptive Euler. Adaptive settings, rejection
semantics, physical timestamps and wall diagnostics are documented in
[TRANSIENT_TURBULENCE.md](TRANSIENT_TURBULENCE.md). Numerical validation is pending.

### Topology-aware mesh methods

`Mesh → Method` now persists the topology method, explicit cell divisions I/J/K,
and mapped progression in the project recipe. Methods are implemented using
Gmsh's documented transfinite, recombination and layered extrusion commands:
https://gmsh.info/doc/texinfo/gmsh.html#Structured-grids

- Automatic: the existing triangle/tetrahedron workflow.
- Quad-dominant: 2D surface recombination; triangles may remain. Not a structured mesh.
- Mapped quad: one convex analytic four-line planar face without holes. I/J are
  counts on the first/second oriented face edges; opposite sides share counts.
- O-grid: four conformal 2D blocks around exactly one hole made from four
  equal-radius quarter-circle arcs. The outside must be a convex quadrilateral.
  I is cells per arc (circumference = 4I); J is radial cells. Radial progression
  above 1 grows away from the hole. Degenerate/twisted corner matches and holes
  touching/crossing the outer boundary are rejected.
- Hex block: a canonical box or a recognized six-face axis-aligned CAD box.
  CAD recovery checks shell count, all six plane locations and complete face
  areas; a generic solid is never simply replaced by its bounding box. I/J/K
  correspond to X/Y/Z and currently use uniform spacing.
- Surface sweep: an analytic profile extrusion produces K layers of prisms.
  Its source triangle mesh uses global sizing; I/J are not used.
- Mapped surface sweep: a convex four-line analytic extrusion profile generates
  I×J×K hexahedra with I/J grading and uniform K layers.

Sweep source/target are the original extrusion profile and its translated cap;
arbitrary source/target CAD face matching and general multi-body sweep are not
implemented. Faceted/imported CAD solids generally keep tetrahedral meshing;
the recognized axis-aligned box is the explicit exception. These methods do
not implement general 3D wall inflation.

Explicit structured divisions take precedence over global size bounds. Disable
scoped size/distance/box/inflation controls before using mapped/O-grid/block or
sweep methods; conflicting intent is rejected rather than silently ignored.
Grading is limited to finite 0.5–2 per cell and an end-to-end ratio of at most
1000; explicit structured estimates above 20 million cells are rejected.
The prism sweep cell count depends on the generated source triangulation and
is not covered by that explicit structured estimate. Existing mesh quality
inspection remains necessary. Implementation has been compile-checked;
mesh generation, interactive UI and numerical validation are not established
by compilation alone.

#### Scoped edge divisions and distance-field growth

In `Mesh → Refinement`, choose `Exact edge divisions · 2D`, then scope analytic
boundary edges through an existing named boundary or the current geometry
selection. The specified count is the number of cells on **each** scoped edge,
not the combined count. Uniform/geometric spacing and reversed direction use
the stored curve direction. The optional two-ended mode uses Gmsh's `Bump`
coefficient (0.01–1), which is not a cell-to-cell growth ratio. Controls can be
edited, suppressed, deleted and restored with the project. Duplicate enabled
scopes are rejected. Mapped/block/sweep methods retain their I/J/K definitions;
scoped edge divisions must be disabled for these methods. Exact division
controls are currently limited to analytic 2D boundaries, not faceted CAD edges.

An optional growth limit (1.01–2) for **linear distance-size fields** checks that
`transition_width >= (far_size - near_size) / ln(growth)`. The Calculate minimum
transition button fills an end distance from this bound; Apply refinement saves
the growth setting. A shorter existing control is rejected at mesh preflight
with the required interval. This is a size-field slope criterion, not a bound
on actual neighboring cell-volume ratios, and does not smooth arbitrary box
or intersecting field transitions. Mesh quality inspection remains necessary.

#### Planar CAD face recovery

The faceted CAD exporter now attempts to reconstruct one planar mesher surface
per CAD face from the display triangles. It keeps outer/hole loops and stable
physical names, removes internal triangle diagonals, and checks planarity,
consistent winding, manifold boundary connectivity and area conservation.
If recognition fails, the original faceted face is retained. Curved faces
remain faceted: this is **not** direct STEP/OCC meshing or general 3D inflation.
Build checks do not establish mesh quality or a solver convergence improvement.

#### Quality isolation, focus and reports

`Mesh → Quality` includes the maximum adjacent cell-measure ratio. For every
cell it is the maximum of `max(Vowner/Vneighbour, Vneighbour/Vowner)` on its
internal faces; V denotes area in 2D and volume in 3D. Boundary faces do not
contribute. Cells without internal neighbours have ratio 1.

The cell-measure threshold now selects cells **below** the threshold. The other
quality thresholds select cells **above** it; non-finite values are also flagged.
This only affects inspection, not solver acceptance. The histogram still covers
all cells, including hidden cells. Isolate poor cells exposes threshold-selected
whole cells in 3D without changing the mesh. Focus worst cell and the listed-cell
buttons centre the camera; in 3D focusing isolates that single cell. Show all
restores the full mesh. View filters are disposable and are not physical cuts.

Cell quality CSV exports zero-based IDs, shape, centre coordinates, measure and
all five quality metrics. Mesh report JSON contains shape counts, physical
patches, the recipe, preflight findings, precise metric definitions and finite
cell percentiles (linear interpolation, not volume weighted). Both export the
current editable mesh, not a historical result mesh.

`Mesh → Method → Validate mesh plan` provides expected block connectivity counts
for mapped quads, O-grid, hex blocks and mapped hex sweeps. The displayed memory
number counts base Rust mesh-array/index payload only. It excludes spare
capacity, allocator overhead, patch records, render caches, solver vectors and
matrices, and is **not** measured RSS or a solver RAM requirement. Unknown
unstructured/prism-sweep cell counts are explicitly left unpredicted. Revalidate
after edits; this planning summary does not prove successful generation or quality.

These additions were compile-checked. Interactive behaviour and numerical mesh
quality/convergence improvements have not been established by a generation run.

### Additional transient and pressure-opening controls

In Solution, choose **Transient · BDF2 fixed Δt** for fixed-step backward
integration. Set physical dt and number of steps, and allow enough coupled
correctors per step. One Euler startup step is used for each new or restarted
solve invocation. Two-level BDF history is currently kept in memory only.
This option changes time discretization; pressure/velocity coupling remains
SIMPLE. Adaptive Euler remains a separate choice.

In Setup, **Total-pressure opening** accepts gauge `p0/rho` in m²/s².
The entered value is not pressure in pascals. Velocity is extrapolated rather
than prescribed. Specify incoming thermal/RANS properties separately where
those models are active. This boundary is incompressible and is not a
non-reflecting compressible outlet. These new paths require numerical
benchmark validation before research use.

### Experimental k-epsilon walls

In Solution, choose **RANS · k–ε**, then **Smooth equilibrium wall functions**
under Wall treatment. Set inlet k/epsilon globally or per boundary in Setup.
Only smooth-wall equilibrium treatment is supported; review the completed
wall y+ and force report and provide appropriate near-wall mesh spacing.
The option is not validated for separated cylinder flow or wall heat transfer.
Wall-resolved k-omega/SST remain separate choices; this option cannot be used
with them. Wall settings persist with the project.

### Time-dependent inlet, flow rate and pressure

1. Assign the physical condition to the selected patch in Setup.
2. Under **Physical-time boundary table**, select velocity, flow rate or pressure.
3. Paste CSV rows without a header: `time_s,Ux,Uy,Uz` for velocity, or
   `time_s,value` for scalar flow rate/kinematic pressure.
4. Apply the table, then select a transient mode in Solution. Time is physical
   seconds, not SIMPLE iteration number. Use a sufficiently small dt for ramps.

For a velocity inlet ramp from rest to 1 m/s over one second:

```csv
0,0,0,0
1,1,0,0
2,1,0,0
```

There is one table per patch; outside its time range the endpoint value is
held. Pressure values are p/rho or p0/rho (m²/s²), not Pa. Flow rate follows
the physical inlet's m³/s or kg/s setting (per unit depth for 2D volume flow).
To remove it, choose **Constant / remove time table**, then click Remove.
Changing physical type can make an existing table incompatible; remove or edit
that table before solving. Tables are saved inside the project; direct CSV file
import, periodic repetition and spatial profiles are not implemented.
Temperature and heat-flux tables are now available as described below. The new path is compiled but not numerically benchmark validated.

### Equation controls and thermal completion

Solution → Controls → **Equation solver controls…** opens a scrollable draft
editor for velocity, pressure, temperature, k, omega and epsilon. Change
absolute/relative linear tolerance, iteration count and supported preconditioner,
then Apply. Cancel discards the draft. Inactive equation settings are retained.
The run summary and report JSON include last completed equation linear residuals
and counts. L2 residuals are not interchangeable with reference solvers'
normalized residuals.

Model settings expose a separate turbulence relaxation and relative-update
tolerance. Fluid material → Heat transfer exposes temperature relaxation,
relative-update tolerance and molecular/equilibrium-Prt wall diffusivity.
Equilibrium-Prt requires equilibrium k-epsilon momentum walls; it is not a
validated thermal log-law model.

In Setup, assign the thermal condition first, then select Temperature vs time
or Heat flux vs time in the boundary-table editor. Use rows `time_s,value`;
Kelvin temperatures must be positive, and flux is positive into the fluid in
W/m². Enable heat transfer and transient integration before running. Only one
time table can currently be assigned per patch.

### Pressure linear method selection

In Solution, open the independent equation solver controls and select the
pressure row's **GAMG / GCR** method to use aggregation multigrid. **Auto · IC0
→ Jacobi** instead attempts IC0-PCG and uses Jacobi-PCG only if IC0 factorization
encounters a non-positive pivot. Explicit IC0 remains strict. These methods
retain the entered absolute/relative L2 tolerances and maximum linear iterations.
They do not change the outer SIMPLE convergence criteria or mesh quality.
The default remains IC0; the new options require numerical validation before
production/performance claims.


### Experimental transient PIMPLE and pressure mesh correction

In Solution, select a transient time scheme and then PIMPLE · transient only.
Set the inner pressure correctors (e.g. 2); the outer-iteration limit is still
the maximum coupled budget per time step. Select extra non-orthogonal pressure
passes (e.g. 1) and limiter (e.g. 0.5) to enable deferred pressure flux correction.
Apply settings before running. SIMPLE permits one inner pressure corrector but
can also use the non-orthogonal passes. Zero extra passes preserves the previous
pressure operator. All settings are stored with the project; these new paths
are experimental and have not undergone numerical benchmark validation.


### Volume momentum models (experimental)

Solution → Volume momentum models opens an atomic editor. Resistance can apply
to the whole domain or a box in metres; enter D [1/m²] and F [1/m]. Selected
cells are reported when a solve starts. This is isotropic momentum resistance,
not a complete porous multiphysics model.

Global rotating frame requires a 3D fluid mesh. Enter the angular velocity vector
in rad/s and its origin in metres. With this model active, inlet/wall/initial
velocities, result vectors, plane monitors and exports are RELATIVE to that
frame. A co-rotating wall uses zero relative velocity. This is not a
stationary/rotating multi-zone interface, nor moving-mesh geometry. Apply
invalidates the old active solution; historical Runs retain their own settings.


### Cyclic support scope

Conformal translational cyclic coupling currently belongs to the experimental
compressible CLI. It is not selectable in the Workbench incompressible Setup
panel. CLI settings.cyclic_pairs contains objects with first/second patch names,
translation {x,y,z} in metres, and tolerance. Omit both patch names from
settings.boundaries; retain physical conditions on every other exterior patch.
For x=0 and x=1 matching sides, a first-to-second translation is {x:1,y:0,z:0}.
The mesh must actually have matching face partitions: Gmsh refinement on one
side alone is not enough, and AMI interpolation is not supplied.


### Cyclic pairs in Setup (experimental)

Generate a mesh with conformal translated opposite sides. In Solution select
Laminar, first-order Upwind and disable temperature transport and the rotating
frame, then apply those settings. Open Setup → Boundaries → Cyclic patch pairs.
Choose first and second patches and enter the translation from first to second
in metres. Add to draft validates geometric matching; Apply activates the pair
and replaces ordinary physical conditions on those sides.

Assign physical conditions to all other patches. Select SIMPLE or transient
PIMPLE as appropriate. Cyclic pair rows are marked in the boundary list.
Removing a pair requires assigning physical conditions again. No automatic
Gmsh periodic-mesh constraint, rotational periodicity or AMI is provided.


### Additional turbulence choices (experimental)

Solution → Momentum transport model offers SA-noft2 (wall resolved). Enter
initial/default incoming nu_tilda in m²/s; resolve the wall layer (target y+≈1).
For patch-specific data, Setup → incoming turbulence → SA modified viscosity.
k/omega or intensity-length inputs are not accepted as SA boundary data.
Equation solver controls includes a separate nu_tilda row. Restart fields
preserve nu_tilda, but numerical restart equivalence remains unvalidated.

For k-omega/SST, the Wall treatment selector now offers blended smooth omega
wall functions. Inspect the wall y+ report; large first-cell heights or
separation are not made accurate by selecting this option. Roughness and
general all-y+ behavior remain unsupported.


Experimental RNG k–ε is available under Solution → momentum transport model.
Set initial/incoming k and ε; solid walls require smooth equilibrium wall treatment.
Inspect wall y+; this is not a wall-resolved low-Re model. Saved projects and
checkpoints retain the RNG model and ε field. Numerical validation is pending.


### Realizable k–ε selection

In Solution → momentum transport model choose **Realizable k–ε (experimental)**.
Set positive initial/default incoming k and ε. Configure per-patch turbulence
inlets in Setup where appropriate. For solid walls select smooth equilibrium
wall functions and inspect y+; refining to y+≈1 does not turn this into a
low-Re model. Global rotating frames and equilibrium-Prt thermal walls are not
supported for this model. Existing field restart and VTK ε/νt output are used.
The model is implemented but has not been numerically validated.


## Continued iteration numbering and shared section results

Run / Continue adds the selected budget to the retained state. Progress, monitor
filenames, sampling cadence, checkpoint iteration and run iteration metadata use
the cumulative index; Initialize resets it. A loaded checkpoint resumes from its
stored index. Physical time and the nonlinear corrector count remain distinct
from the cumulative accepted-step index in transient mode. Each saved run retains
its own convergence segment; live curves keep previous segments during Continue.

Settings now contains an optional Gmsh executable path and a file picker. Leave it
empty to use `gmsh` from PATH. The selection is stored in per-user interface
preferences, not in a portable CFD project. It applies to automatic probes and
future meshing jobs, including already-open projects.

Geometry, Solution and Results share the same section controls. In Results,
choose the CAD plane directly from the top selector. The Planes view can render
speed, kinematic pressure, global velocity components or normal velocity from a
recorded JSON frame or the selected run's final field. Choose filled contours,
contour lines, or both; vectors and planar streamlines can overlay the field.
Contours reconstruct vertex values by cell averaging; stored cell data is not
modified. Streamlines use bounded-step integration of piecewise constant projected
cell velocity and stop at the section boundary or stagnation. They are visualization
approximations, not three-dimensional particle tracks or a new physical transient
solution. Vector lengths are speed-scaled by default for new plane settings.

Use the shared Open recording / Play frames controls in Solution or Results to
replay saved iteration or physical-time frames. Section properties have Display,
Recording and Export tabs. CFD buttons and captions now share one component style.
A steady iteration animation is convergence history, not physical flow evolution.
Existing SVG-only recordings retain their original appearance; changing fields
requires JSON records or a final field dataset. Full 3D particle advection and a
complete redesign of every CAD/native property editor are not included here.


### Display location integration

Results → Display → Location now lists the 3D surface, XY/XZ/YZ sections and
project CAD planes together. CAD plane selection stays inside Display and uses
the shared section controls and recorded-frame playback. Built-in axis sections
use actual intersected volume-cell polygons and the same contour/vector engine;
field selection (including temperature and turbulence scalars), colour range,
colour map, contour level count, cell edges and vector visibility reach the
section renderer. Section contour lines use vertex-averaged scalar reconstruction;
these are not interpolated physical fields exported back into the solver.

Surface contour modes also draw isolines reconstructed from vertex averages of
boundary-face scalar values. Inlined CAD plane playback uses the same timer and
frame controls as the Planes view. This change has compiler checks only; visual
interaction and rendering still require runtime validation.


### Single Results display panel

Results now has one Location selector and one Display property editor for surfaces
and CAD sections. The separate Planes tab, upper plane picker, nested plane picker
and duplicate Display/Recording/Export sub-tabs are removed from Results. Recording
settings appear inline for a CAD section; its saved-frame playback replaces the
unrelated 3D slice-sweep/tracer bar. Walls remains a separate reports tab.
Origin XY/XZ/YZ planes at zero offset are omitted from the CAD location entries
because built-in axis sections already cover these orientations. Selection uses an
explicit mapping to the CAD plane catalog, not an arithmetic index assumption.
Palette, contour count, vector density/scale and range are consumed by the same
Display controls. Saving recording cadence does not reset display styling.

The live section canvas omits the publication figure title/axes and uses the
workbench viewport background. Scientific SVG exports retain axes, units and
iteration metadata. Location changes stop any animation from the previous view.

### Section playback and quantitative post-processing

The same planar navigation component is used for recorded sections in Solution
and Results: drag to pan, wheel to zoom, double-click to reset. This is still a
planar SVG viewport, not a replacement of the three-dimensional surface renderer.
Vector normalization and normal-flow markers can be controlled from Display;
normalized arrow lengths are not a speed measurement. Built-in volume sections
also consume vector density/scale and respect the vectors-only mode.

Playback has a first/last frame range, Loop and FPS. Last frame 0 means the end
of the recording. Raw JSON playback reports the archived physical time when
present, otherwise the actual steady iteration. Lock colour range scans all
loaded raw frames in a background worker and stores one finite range in the
plane's persisted display settings. Legacy SVG recordings cannot be recoloured.
The colour lock is for the selected scalar; recompute it after changing fields.

Section statistics CSV uses geometric intersection-polygon areas for area means
and integrals. The signed volume-flow estimate integrates sampled cell U dot n;
it is not the conservative solver face flux and is not a mass-flow report.

Time statistics CSV uses the selected playback range, physical-time trapezoidal
quadrature of U, p/rho and their products. It exports means, RMS fluctuations and
velocity covariance components (m²/s², not density-scaled stresses). Only two
frames are held during integration; missing/nonfinite/nonincreasing timestamps,
legacy images and changing plane/sample geometry are rejected. No SIMPLE-iteration
recording is interpreted as a physical-time average. Sampling frequency limits
resolved fluctuation statistics. This offline export can be repeated after a
restart if the loaded frame sequence is continuous; it does not checkpoint a
live averaging accumulator. Numerical benchmark validation remains outstanding.

The archived-result field selector additionally exposes the reconstructed second
velocity-gradient invariant Q = (tr(G)² - tr(G G))/2, in s⁻². For solenoidal fields
this equals (|Omega|² - |S|²)/2. It shares the existing cell-neighbour LSQ gradient
reconstruction and is included in full cell CSV. Boundary reconstruction accuracy
is limited by available archived cell data; Q is not a conservative diagnostic.
Three-dimensional Q isosurfaces, physical pathlines, comparative run charts and
video export of recorded plane sequences are not implemented by this change.

### Unified interactive section display (October 2026)

`SectionProperties` is now the one section editor used by Geometry, Solution and
Results. Field, palette, contour layers, vector density/scale, streamlines,
normalization, range, recording cadence and exports share the same callbacks and
persisted `PlaneDisplay`. Results does not add a second field picker for CAD
sections. Surface/axis-section controls remain distinct from this section editor.

The section camera now changes the data-to-screen projection and redraws polygons
and vectors at the viewport's current size. It does not magnify the previous
bitmap. The legend and glyph widths remain in screen units. Clicks are transformed
back to world coordinates and tested against the actual intersection polygons;
probe readout uses the owning cell's archived U and p/rho. A selected probe has a
marker and follows the same world point while frames change. Home resets the
camera. The current raw JSON frame is cached separately from its rendered image.
The display still rasterizes SVG geometry for Slint presentation: it is a planar
data view, not an orbitable 3D context view. Scientific SVG export retains axes.

The recording-actions menu can export the selected probe history over the chosen
frame interval. CSV includes actual iteration/time, fixed world coordinates and
sampled fields; missing coverage is a gap, not zero. This is cell sampling without
spatial or temporal interpolation. Probe histories of steady iterations are
labelled by iteration; physical-time statistics still require real timestamps.

Validation: targeted manufactured-field tests cover camera inversion/probing,
irregular-time quadrature, invalid recording rejection and area/range statistics.
An opt-in `unified_section_real_window_smoke` test opens the real Slint interface,
changes scalar fields, navigates/probes and checks frame advance/loop callbacks.
It writes Results/Solution screenshots under `/tmp/flursys-section-ui`; run with
`SLINT_BACKEND=winit-software` and `--ignored --test-threads=1`. Optional
`FLURSYS_PREVIEW_FRAME` selects a raw JSON frame read-only for this smoke test.
This does not validate a CFD solve, native CAD interaction, all historical project
formats or every export dialog. Existing 3D Q-isosurface/pathline and comparative
plot limitations remain as stated above.

The smoke test additionally exercises a three-frame temporary replay, bounded
looping, the persistent probe, and snapshots at wide/narrow requested sizes.
Its temporary replay frames are manufactured variations even when initialized
from an existing raw frame; this is UI/data-path verification, not CFD evidence.

The final local run passed five targeted data tests and the real-window smoke
initialized from a saved cavity JSON frame. A normal GUI build passed and the
standalone software-backend application stayed open for 12 seconds without
logged errors, then was deliberately terminated. The compositor constrained
requested preview sizes to roughly 933–940 actual pixels; full-width responsive
coverage is not claimed. The full numerical suite and all export dialogs were
not exercised by this UI change.

### Additional OpenFOAM-inspired post-processing

For full archived meshes, Results > Display adds **Flow type** and **Strain rate**
to the scalar selector. They use the existing LSQ reconstruction once when the
result dataset is loaded, and are available to the surface/axis-section renderer,
field statistics, HTML report and full cell CSV. These are not solver fields.
Raw CAD-plane recordings do not contain the full neighbour stencil, so the new
3D gradient quantities are not manufactured from those recordings.

The shared section editor now exports horizontal/vertical centreline profiles in
its plane u/v coordinates, using 201 uniformly spaced positions. These work on
arbitrarily oriented CAD sections, including raw archived JSON frames, without
rerunning the solver. CSV contains distance, u/v, world coordinates, cell, U,
p/rho and the currently selected scalar. Samples outside the fluid remain blank;
no boundary values or values across holes are extrapolated. The library also
accepts arbitrary u/v endpoints and 2..2000 sample points.

Section statistics include positive-normal and negative-normal volume fluxes,
absolute flux, reverse absolute-flux fraction, and an absolute-flux-weighted mean
of the selected scalar. Negative flux retains its sign. The mean is blank when
absolute flux is zero. The estimates use geometric intersection areas and sampled
cell velocity; they do not replace conservative mesh face-flux reports.

Reference concepts were inspected in the local OpenFOAM-dev source:
`src/functionObjects/field/flowType/flowType.C`,
`src/functionObjects/field/fieldValues/surfaceFieldValue/`, and
`src/sampling/sampledSet/sampledSets/`. These additions are independent Rust
implementations and do not assert equivalent discretization or validation.

### Custom section profiles and stable time moments

In the shared section editor, enable **Custom profile line**, enter start/end u/v
coordinates in metres and choose 2..2000 sample positions. **Profile CSV** exports
the field samples; **Chart SVG** exports a labelled scientific chart with distance
on the horizontal axis and the selected scalar/units on the vertical axis. Both
consume exactly the same sampling routine. Missing fluid coverage breaks the
curve; connecting line segments are display interpolation, not reconstructed
solver values. These controls apply equally to Solution and Results and to raw
recordings. The draft coordinates are local UI inputs, not saved CAD geometry.

Time means, RMS and covariances now use weighted online central moments rather
than subtracting the square of a large mean from a large second raw moment. The
same physical-time trapezoidal endpoint weights are retained, including irregular
time steps. Non-finite accumulation is rejected. This reduces cancellation for
small fluctuations on a large mean, but cannot recover precision already lost in
the stored fields or compensate for sparse temporal sampling.

Raw-frame playback and quantitative exports now check plane identity and geometry
against the selected plane. An unrelated/moved-plane recording produces an error
instead of silently pairing its data with different section properties.

### Live/replayed section monitors and run comparison

**Solution > Convergence** adds Plane mean speed, Plane mean p/rho and Plane
volume flux. Select the plane in the dropdown below the chart type; enable that
plane's recording in Display before starting. Scalars are updated on each emitted
section snapshot using intersection-polygon areas. History is scoped to the
recording job; continuation iteration numbers retain their global offset. These
plots use linear physical units, with actual physical timestamps for transient
records and iteration numbers for steady records. Residual logarithm/normalization
controls are hidden for the dimensional monitors. History CSV and chart SVG/PNG
exports follow the selected monitor, rather than exporting unrelated residuals.

**Results > Plots > Plane monitor history** shows the same quantities for the CAD
plane selected in Location. Saved raw JSON frames are decoded one at a time in a
background worker. Monitor scalars are reconstructed when selecting a recorded
plane or opening its recording folder. Missing legacy/raw data and mixed or
non-increasing time coordinates produce errors. CSV records area, mean pressure,
mean speed, estimated volume flux and actual iteration/time. Flux integrates
cell U dot n over section polygons; it is not the conservative face-flux balance.

**Results > Plots > Compare runs · 2D** holds up to eight frozen profile curves,
including one optional imported reference. Choose a saved workspace/run, field,
line endpoints and sample count, then Add/update active run; select another Run
and repeat. Field, endpoints and workspace must match. Curves persist while
switching Runs in the current GUI session; comparison setup is not persisted in
project.json. The generic chart engine draws every curve with a legend, shared
axes/units and gaps for missing fluid samples. CSV/SVG exports preserve run labels
and line context. CSV comment metadata includes the field and endpoints.

Reference import accepts a simple two-column, unquoted CSV: distance_m,value,
with an optional header, blank values for gaps, and optional # comments. Distances
must increase strictly and values must use the selected field units; no unit
conversion or reference validation is inferred. The importer limits input to 2 MB
and 20000 positions. Comparison supports 2D full-mesh profiles; arbitrary CAD-plane
profile comparisons are not added here.

Errors are evaluated at finite baseline/reference positions with piecewise-linear
interpolation of each target curve only inside finite segments. No extrapolation
or interpolation across missing segments is performed. RMSE, maximum absolute
error, relative L2 and matched count N are shown. Relative L2 is N/A for a zero
reference norm. If no external reference was imported, the first curve is labelled
a baseline, not a validated reference. These metrics are sample-weighted and are
not volume-integrated CFD error norms. Compilation is checked; this addition has
not received new runtime or numerical benchmark validation.

Opening another recording is blocked while a solve is active, to keep the live
monitor attached to its solve. When reconstructing a replacement archive, old
samples for that plane are cleared first; an invalid archive cannot leave an old
history displayed as the newly selected source.

### Case review, failed-run diagnostics and report refresh

The Solution toolbar has **Review case**. It runs canonical readiness checks,
resolves boundary conditions, checks fatal mesh geometry, and checks enabled
3D plane/volume intersections before the solver starts. The Reports section
shows errors and advisories with navigation to Mesh and Setup. Mesh quality
warnings are advisory. A successful review is not scientific validation.

A failed or stopped solve can expose a **UNCONVERGED** diagnostic dataset in
Results. It comes from a retained completed sample, not necessarily the failed
iteration. Steady solves retain the first completed iteration and then at a
cadence no greater than ten iterations; that cadence divides the configured disk
checkpoint interval. Accepted transient steps retain their complete restart
state. Success replaces the retained state with the final complete solution.
Non-finite field arrays are rejected by the Results dataset validator.

The worker saves failed-last-sample.json for failed jobs under the job's
monitors/solve-* directory, independently of periodic checkpoint settings.
This does not change the run status to converged. The relative artifact path is
recorded in the run report. Select that failed run in the Browser's Runs tree
to reopen it; the restored mesh must match the checkpoint signature exactly.
Old failed runs without this artifact remain unavailable.

Field health uses conservative face fluxes. It reports inflow/outflow,
relative boundary imbalance (N/A for zero throughflow), maximum speed and its
zero-based cell index, pressure range, invalid cells, and two distinct backflow
measures: inward outlet face count and inward outlet flux divided by absolute
outlet flux. Flux is per unit depth in 2D. The worker saves field-health.csv;
the run report includes sampled_field_health. Its sample iteration can differ
from the last residual. Health diagnostics do not certify convergence.

Section controls include disabling the selected recording, disabling all plane
recordings, and resetting the selected monitor's display, interval and enabled
flag. Existing files and CAD planes are retained.

Box selection supports exterior 2D edges and exterior 3D faces. Complete
projected entities must fit inside the rectangle. In 3D this includes occluded
exterior faces. Shift/Ctrl adds entities. Selection is tied to the active domain.

Convergence and monitor-history charts keep one decoded SVG image each on the
UI thread. Exact SVG comparison controls reuse, preventing stale charts.
Hidden convergence charts are not rebuilt. Full histories and vector exports
are retained. The line-profile empty state no longer erases monitor/comparison
charts. No end-to-end speedup factor has been measured.

### Numerical regression automation

The numerical-regression workflow is manual/weekly, separate from build checks.
It runs analytic source-profile refinement, selected 2D/3D cavity regressions,
solve-review checks and existing real-Gmsh production example smoke checks,
uploading logs even on failure. Cylinder smoke checks cover finite fields and
mass balance; they do not establish reference Drag/Lift/Strouhal accuracy.
Reference-error studies, grid/time refinement and interactive UI verification
remain separate acceptance requirements.

### Additional transient and turbulence controls

- Solution → Time includes **Transient · adaptive BDF2 Δt**. Set initial dt,
  additional physical steps, maximum Courant, dt bounds and growth. As with
  other new transient paths, compile success does not imply numerical accuracy.
- Setup → Boundaries → Incoming turbulence includes **Intensity + hydraulic
  diameter** and **Intensity + viscosity ratio**. Intensities are fractions,
  diameters are metres, and viscosity ratios are dimensionless. Apply the
  incoming-turbulence settings after editing. SA still uses modified viscosity.
- Mesh → Refinement → Inflation includes a **First-layer estimate**. Supply
  target y+ and a friction velocity from a prior run or an external estimate.
  The helper uses saved molecular viscosity and h1=2 y+ nu/u_tau, assuming
  a wall-normal layer whose cell centre lies halfway through its thickness.
  It fills the first-height draft and switches to first-height mode; apply
  inflation separately. Inspect actual y+ after solving.
- The estimator does not implement arbitrary 3D prism layers. That backend
  limitation remains explicit; 2D inflation and supported sweeps are distinct.

### Initializing after remeshing and patching a region

Solution → Initialize now offers:

- **Map selected result**: use the currently selected archived/diagnostic dataset
  as the source and the current Workbench mesh as the target.
- **Map VTK file**: read the supported FLURSYS legacy ASCII VTK dialect in a
  background worker. This does not import arbitrary OpenFOAM/ParaView formats.
- **Patch U / p inside box**: use the velocity and kinematic pressure entered
  above the button for cell centres inside the specified world-coordinate box.
  Outside cells retain queued initial fields; if none exist they start at zero.
  Bounds are in metres. For XY 2D cases Z min=Z max=0 is valid.
- **Cancel mapping**: cooperatively cancel preparation. Replacing the project or
  initialization cancels the pending task. An old target MeshId cannot receive
  a completed operation after remeshing.

Mapping requires matching mesh dimension, coordinates and physical domain;
there is no rotation, translation, unit conversion or pressure-reference
conversion. Source cells must be convex with planar faces within tolerance.
Only target-centre coverage is checked; equal domain volume or wall topology
is not proved. All target centres must belong to source cells, otherwise the
whole operation is rejected, preserving previous initialization. Values use
piecewise-constant source-cell injection, without smoothing or extrapolation.

Both operations initialize only U and p/rho. They reset physical time and
iteration count. Face fluxes and thermal/turbulence initialization are rebuilt
by the solver. They are not conservative remapping, a continuation of accepted
physical time, or a converged result. Use exact-mesh **Load restart** for that
different workflow.

For a saved workspace, the worker atomically archives prepared initial fields
under initialization/fields-*.json before applying them. Load restart can reopen
this file on the exact target mesh. Project Save does not automatically queue
that file on a later application launch. The file path and the preparation
description are logged. Preparation runs off the UI thread; source snapshots
copy only mesh/U/p, excluding derived display arrays. BVH construction and file
parsing can finish their current phase before cancellation is observed.

No numerical validation or interactive runtime verification of these new paths
was performed in this implementation step.
