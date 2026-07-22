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
cargo run --release --features gui --bin flursys-gui
cargo run --release --bin flursys -- --project examples/cavity.flursys.json
```

Available cases include a lid-driven cavity, cylinder flow, and backward-facing step. Results are
written to the selected output directory in CSV, VTK, and PPM formats. The current solver supports
transient projection and a steady SIMPLE-style coupling for laminar incompressible flow.
The optional Slint desktop interface keeps the solver on a separate worker thread and shows live
residual, force, and field updates.

Simulation projects use versioned `.flursys.json` files, so supported cases can be created,
shared, imported, and run after compilation from either the GUI or CLI.

The workbench also stores named boundary conditions, extrusion depth, and mesh-layer settings in
each project. Its 3D view is an engineering pre-processing preview of the structured 2D domain;
the current numerical kernel remains a 2D structured finite-volume solver. True 3D solving and
unstructured mesh generation are intentionally separate future solver milestones.

## License

MIT
