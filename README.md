# FLURSYS

<img src="ui/assets/flursys-logo.svg" alt="FLURSYS logo" width="220">

**FLU**id + **R**ust + **SYS**tem

FLURSYS is a Rust-based scientific simulation project. It currently contains a finite-volume solver
for incompressible fluid flow, with a few standard test cases and multi-core CPU support.

The project is still under development. It includes structured and unstructured workflows,
a structured three-dimensional cavity solver, and an optional graphical workbench with
CAD, Gmsh meshing, steady unstructured flow, and result inspection. Physics and time-integration
support differ between backends; see the implementation-backed documentation below.

## Documentation

- [Numerical methods and current limitations](docs/NUMERICAL_METHODS.md)
- [CAD/mesh workbench, project storage and result artifacts](docs/WORKBENCH.md)

## Build

The CAD workbench is a pinned Git submodule. After cloning, initialize it before building:

```bash
git submodule update --init --recursive
```

```bash
cargo build --release
cargo test
```

## Run

```bash
cargo run --release --bin flursys -- list
cargo run --release --bin flursys -- cavity --threads 4
cargo run --release --bin flursys -- cavity --coupling simple --max-steps 10000
cargo run --release --bin flursys -- cavity-3d --nx 32 --ny 32 --nz 32
cargo run --release --bin flursys -- --project examples/cavity-3d.flursys.json
cargo run --release --features gui --bin flursys-gui
cargo run --release --bin flursys -- --project examples/cavity.flursys.json
```

Available cases include a lid-driven cavity, cylinder flow, backward-facing step, and plane
Poiseuille channel flow. Results are
written to the selected output directory in CSV, VTK, and PPM formats. The current solver supports
transient projection and a steady SIMPLE-style coupling for laminar incompressible flow.
The optional Slint desktop interface keeps the solver on a separate worker thread and shows live
residual, force, and field updates.

Versioned `.flursys.json` files are read-only legacy import input. The workbench imports
recoverable intent into a new portable directory workspace with `project.json`,
separate autosave data, and per-run artifacts. It persists canonical geometry, Named Selections,
mesh intent, physical boundaries, material, SIMPLE controls, and run metadata; generated meshes,
solution fields, and UI caches are derived artifacts.

The workbench can extrude a canonical planar face into stable cap/side faces and a body, pass
Named-Selection physical surfaces through Gmsh, and generate a real 3D unstructured mesh.
The workbench VTK writer supports 2D polygons and tetrahedral 3D cells. This is a scoped CFD
preprocessing workflow, not a general CAD kernel or a completed 3D CAD GUI. See
[`docs/WORKBENCH.md`](docs/WORKBENCH.md), [`docs/GEOMETRY.md`](docs/GEOMETRY.md), and
[`docs/MESHING.md`](docs/MESHING.md) for the supported scope and limitations.

## License

MIT
