# Workbench project workspace

Implementation audit: 2026-10-20. For equations, units, residuals and numerical
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
and transient UI state are deliberately excluded from `project.json`.

## Schema and versioning

The current workspace format is version **2** (`WORKBENCH_PROJECT_FORMAT_VERSION`). A document
contains:

- `format_version`, generated `project_id`, and display `name`;
- complete `GeometryTopology`, including stable IDs, revision, and allocator counters;
- Named Selections targeting stable geometry IDs;
- Gmsh mesh intent (`dimension`, global/min/max size, first-order element order);
- optional native `cad_document` and the last transferred `cad_geometry_hash`;
- Named-Selection boundary assignments, density/viscosity, SIMPLE controls and the
  selected `Upwind` or `LimitedLinear` momentum convection;
- `plane_monitors` (ID, name, origin, orthonormal axes, enabled flag and interval); and
- persistent `runs` metadata.

Loading validates the complete document, including geometry, selection targets, numerical
settings, and artifact paths. Workspace v1 documents migrate their missing mesh recipe to the
safe empty recipe; newer or otherwise mismatched versions return a structured error. A saved
project can therefore reconstruct a `WorkbenchSession`, but must regenerate its mesh and
solution fields.

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

Workbench solves constant-density laminar steady flow with SIMPLE, BiCGStab
momentum and PCG/IC0 pressure correction. Solution exposes upwind/limited linear
upwind, relaxation, convergence controls and worker threads. Apply settings stores
the form; Run solver starts the worker. No Workbench control currently enables
PIMPLE, BDF2, turbulence or heat transport.

### Plane monitors and result plots

Select a CAD plane, configure the recording interval and enable recording before
Run. Section data are sampled every N completed SIMPLE iterations and at the
final successful iteration if not already due. Results → CAD section shows the selected plane with speed, kinematic pressure,
velocity components or normal velocity. A colour bar, manual/automatic range,
optional vectors and cell edges apply to the view and manual SVG/PNG exports.
CSV always retains all recorded velocity components and kinematic pressure. Automatic recordings are CSV/SVG
plus `plane.json` in `monitors/solve-<id>/`.

These are cell-centred samples on a 3D convex-cell section, not interpolated
physical-time measurements. A steady iteration sequence is not a vortex-shedding
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

In Results, choose CAD section and select a plane. Open recording selects a plane-specific folder under `monitors/solve-*/<encoded-plane-id>/`; Play frames, previous/next, the frame slider and FPS replay the recorded SIMPLE iteration history. This shows iterative convergence, not physical transient time. New recordings retain section geometry, velocity and kinematic pressure in JSON beside CSV/SVG, so field selection, vector overlays and colour ranges remain editable during replay. Older SVG recordings replay their original appearance. Latest field returns to the current solution. PNG/SVG/CSV exports use the selected recorded frame; legacy SVG data can be exported as SVG/PNG, while its original CSV remains beside it.

Solution offers Initialize (uniform zero velocity and kinematic pressure), Run, Pause/Resume and Stop together. Initialization clears the active solution while preserving archived runs, mesh and boundary settings. Pause cooperatively blocks the current worker at a solver control checkpoint and retains all solver state in memory; Resume continues that same solve. Stop cancels the solve, including a paused worker. Resume does not restore a stopped process or survive application restart.

### Solver preflight and conservation diagnostics

Before Run, fatal non-positive/non-finite cell measures and invalid internal face orientation block execution. The detailed report lists invalid cell/face IDs, faces above 70 degrees non-orthogonality, and the largest adjacent-cell measure ratio. A ratio above 10 and centre-line skewness above 0.5 produce advisory warnings. These thresholds are heuristics, not accuracy guarantees, and skewness definitions differ between tools. Full scans occur during preparation and report generation; the regularly refreshed mesh inspector uses cached quality metrics.

Completed runs save `mesh_preflight` and `conservation` in their report JSON. Solution/Results boundary reports include face-based inlet/outlet flow, net mass flux, relative imbalance normalized by the larger gross inlet/outlet flow, and volume-weighted absolute and signed divergence in inverse seconds. Zero-throughflow domains show relative imbalance as N/A; internal divergence remains informative for cavities. There is no physical time-step factor in these steady diagnostics. They supplement the existing convergence criteria.

Failure messages retain the original technical error and add contextual checks for momentum/pressure linear failures, invalid momentum diagonals, pressure-outlet backflow and singular gradient stencils. These are diagnostic suggestions and do not establish an unmeasured root cause.

### Field checkpoints and report export

The Solution toolbar includes Load restart. Each solve saves `monitors/solve-*/checkpoint.json` after its first completed iteration, at the configured Save restart fields interval (default 100; 0 disables checkpoint writes and signature allocation), and on successful or iteration-limited completion. Atomic replacement retains the last complete file; failed writes are reported and do not intentionally abort the solve. Stop or process termination may lose iterations since the last completed save. The file contains geometry/connectivity and cell order, velocity, kinematic pressure, iteration and a material/numerical/boundary context string. Load restart validates the current mesh and queues these fields for the next Run; Initialize clears a queued restart. A field restart rebuilds face flux, linear solvers and convergence history with current settings, and iteration numbering starts again. It is distinct from an in-memory Pause/Resume and is not a bitwise continuation of SIMPLE.

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
