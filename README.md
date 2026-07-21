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
```

Available cases include a lid-driven cavity, cylinder flow, and backward-facing step. Results are
written to the selected output directory in CSV, VTK, and PPM formats.

## License

MIT
