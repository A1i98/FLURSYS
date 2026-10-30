<p align="center">
  <img src="ui/assets/flursys-icon.png" alt="FLURSYS" width="112">
</p>

<h1 align="center">FLURSYS</h1>

<p align="center">
  <strong>A CFD workbench built in Rust.</strong><br>
  Create geometry, generate a mesh, solve fluid flow and explore the results.<br>
  Use the desktop interface, automate runs with the CLI and Rust API, or connect AI agents through MCP.
</p>

<p align="center">
  <a href="https://github.com/A1i98/FLURSYS/releases/latest"><img src="https://img.shields.io/badge/Windows-Download-0078D4?style=for-the-badge" alt="Download for Windows"></a>
  <a href="https://github.com/A1i98/FLURSYS/releases/latest"><img src="https://img.shields.io/badge/macOS-Download-444444?style=for-the-badge&logo=apple&logoColor=white" alt="Download for macOS"></a>
  <a href="https://github.com/A1i98/FLURSYS/releases/latest"><img src="https://img.shields.io/badge/Linux-Download-E95420?style=for-the-badge&logo=linux&logoColor=white" alt="Download for Linux"></a>
</p>

<p align="center">
  <a href="docs/README.md">Documentation</a> ·
  <a href="examples/README.md">Examples</a> ·
  <a href="https://github.com/A1i98/FLURSYS/issues">Issues</a> ·
  <a href="CONTRIBUTING.md">Contributing</a>
</p>

<p align="center">
  <a href="https://github.com/A1i98/FLURSYS/actions/workflows/ci.yml"><img src="https://github.com/A1i98/FLURSYS/actions/workflows/ci.yml/badge.svg" alt="CI"></a>
  <a href="https://github.com/A1i98/FLURSYS/releases"><img src="https://img.shields.io/github/v/release/A1i98/FLURSYS" alt="Latest release"></a>
  <a href="https://github.com/A1i98/FLURSYS/releases"><img src="https://img.shields.io/github/downloads/A1i98/FLURSYS/total" alt="Downloads"></a>
  <a href="#license"><img src="https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue" alt="MIT or Apache 2.0"></a>
</p>

---

FLURSYS combines SolidCraft CAD, Gmsh meshing and finite-volume solvers in one workspace. It includes steady and transient flow, turbulence models, heat transfer, buoyancy, compressible flow, boundary scripts and result playback.

## Screenshots

| Geometry | Mesh |
| --- | --- |
| ![Geometry workspace](docs/assets/screenshots/geometry.png) | ![Mesh and boundaries](docs/assets/screenshots/mesh.png) |
| **Mesh quality** | **Convergence** |
| ![Mesh quality](docs/assets/screenshots/quality.png) | ![Live convergence monitors](docs/assets/screenshots/convergence.png) |
| **Live flow display** | |
| ![Live velocity contours and vectors](docs/assets/screenshots/flow-display.png) | |

## Results

Explore the [example results](examples/validation/).

## Build and run

Install Rust and Gmsh. On Linux, install the Fontconfig development package as well.

```sh
git clone --recurse-submodules https://github.com/A1i98/FLURSYS.git
cd FLURSYS
cargo run --release --bin flursys-gui
```

Windows packages include Gmsh. For other installations, put `gmsh` on PATH or select its executable in Settings.

## Run an example

Open **File → Examples**, or use the CLI:

```sh
cargo build -p flursys --release --no-default-features --bin flursys
./target/release/flursys examples create laminar-channel ./channel
./target/release/flursys project mesh ./channel
./target/release/flursys project solve ./channel
```

Cases are editable JSON files in [examples/cases](examples/cases). The library includes Couette flow, natural convection, cylinder flow, Taylor–Green vortex, sudden expansion and Sod shock tube.

## Documentation

[Workbench](docs/WORKBENCH.md) · [Meshing](docs/MESHING.md) · [CLI](docs/CLI.md) · [MCP](docs/MCP.md) · [Numerical methods](docs/NUMERICAL_METHODS.md) · [Development](docs/DEVELOPMENT.md)

## License

FLURSYS is available under [MIT](LICENSE) or [Apache 2.0](LICENSE-APACHE). Third-party components retain their own licenses; see [THIRD_PARTY_NOTICES.txt](THIRD_PARTY_NOTICES.txt).
