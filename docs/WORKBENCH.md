# Workbench project workspace

The unstructured workbench uses a directory workspace. Its canonical portable document is
`project.json`, represented by `WorkbenchProject`; it stores validated user intent and run
metadata, not generated runtime state.

```text
MyCase.flursys/
├── project.json
├── autosave/
│   └── project.json          # separate recovery snapshot, when written
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

The current workspace format is version **1** (`WORKBENCH_PROJECT_FORMAT_VERSION`). A document
contains:

- `format_version`, generated `project_id`, and display `name`;
- complete `GeometryTopology`, including stable IDs, revision, and allocator counters;
- Named Selections targeting stable geometry IDs;
- Gmsh mesh intent (`dimension`, global/min/max size, first-order element order);
- Named-Selection boundary assignments (`NoSlipWall`, `MovingWall`, `VelocityInlet`, or
  `PressureOutlet`), material, and SIMPLE controls; and
- persistent `runs` metadata.

Loading validates the complete document, including geometry, selection targets, numerical
settings, and artifact paths. Only version 1 is accepted: a newer or otherwise mismatched
version returns a structured unsupported-version error rather than being interpreted as a
compatible file. A saved project can therefore reconstruct a `WorkbenchSession`, but must
regenerate its mesh and solution fields.

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

## Extrusion and Gmsh mapping

`GeometryTopology::extrude_planar_face()` is the canonical topology operation. It requires a
finite non-zero distance and a planar source face. The source remains the bottom cap; the
operation creates a stable top `FaceId`, one stable side `FaceId` per outer or inner-loop edge,
and a stable `BodyId`. Meshes and solutions held by `WorkbenchSession` are invalidated after a
successful extrusion.

For a three-dimensional workbench mesh, Named Selections must contain geometry faces. The Gmsh
exporter writes the planar profile and a Gmsh `Extrude` operation, then maps the source cap,
top cap, and side faces to Gmsh's generated surface expressions for physical-surface groups.
Those Gmsh tags and `out[]` indices are export-local; stable geometry IDs remain the persistent
selection identity. Gmsh then produces the real three-dimensional mesh and the existing importer
maps physical names to boundary patches.

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

Existing versioned `.flursys.json` files remain the legacy structured-project format used by the
CLI. The desktop validates a non-workspace path through `Project::load` in clearly labelled
compatibility mode, but does not replace or mutate the canonical workbench workspace with it.
A workspace directory is loaded through `load_workspace`. There is currently no automatic,
lossless conversion of a legacy project into `WorkbenchProject`: the legacy project model and
its structured-case semantics do not represent the complete canonical unstructured workspace
state. Create or open a workspace directory for canonical workbench editing.

## Current CAD and UI limitations

The implemented CAD scope is stable topology, planar profiles, primitive boxes, and committed
planar-face extrusion for Gmsh export. It is not a general CAD kernel. In particular, FLURSYS
does not currently provide general Boolean union/difference/intersection, fillets, chamfers,
NURBS/B-splines, STEP/IGES import, CAD healing, topology naming across arbitrary Boolean
changes, or a general revolve feature.

The current UI is still a scoped workbench rather than a general CAD editor. It retains an
explicit legacy structured-project adapter for older panels, while workbench workspace save/load
uses the canonical document. Canonical 3D viewport picking returns stable geometry `FaceId`
values (and can resolve their owning `BodyId`); body translation/rotation are geometry commands
that invalidate derived mesh and solution state, unlike camera motion.
