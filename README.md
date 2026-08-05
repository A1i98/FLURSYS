# FLURSYS

**FLU**id + **R**ust + **SYS**tem

FLURSYS is a Rust-based scientific simulation project. It currently contains a finite-volume solver
for incompressible fluid flow, with a few standard test cases and multi-core CPU support.

The project is still under development. Future work will cover three-dimensional problems, additional
numerical methods, other equation systems, and a graphical interface.

## Build

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

Simulation projects use versioned `.flursys.json` files, so supported cases can be created,
shared, imported, and run after compilation from either the GUI or CLI.
Their `solver` object supports `"convection": "first-order-upwind"` or `"convection": "central"`.

The workbench persists named boundary conditions, CAD/sketch feature data, mesh intent, and
solver-independent analysis intent in each project. Before a run, FLURSYS creates a capability
checked execution plan rather than silently applying an incompatible solver. The supported
backends are structured 2D incompressible flow and an initial real 3D lid-driven-cavity solver
using a staggered MAC grid and pressure projection. CAD solids are retained as project data but
are not yet meshed or solved; unstructured meshing and general 3D boundary workflows remain
future milestones.

## License

MIT
