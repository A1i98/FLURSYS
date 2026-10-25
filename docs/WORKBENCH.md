# Workbench

A workspace is a folder containing `project.json`, mesh assets and run results. Open that folder in the desktop app or pass it to the CLI.

## Geometry

Create or import a model in the SolidCraft editor. For 2D, select the planar fluid face. For 3D, select the fluid volume. Exclude solid obstacles from the fluid domain before meshing.

## Mesh

Name the boundary edges in 2D or surfaces in 3D. Set the global size, then add local sizing or refinement around features that need more resolution. Generate the mesh and inspect its quality.

The size bounds must satisfy `minimum ≤ target ≤ maximum`; local sizes must fit within those bounds. See [Meshing](MESHING.md) for mapped meshes, inflation and quality controls.

## Setup

Assign a physical condition to every boundary and set the fluid properties. Configure temperature, gravity and buoyancy together for natural-convection cases. Boundary scripts are selected on the patch they control.

Compressible cases have their own density, absolute-pressure, gas and boundary-state settings.

## Solution

Choose steady or transient integration, numerical methods and compute settings.

- **Run / Continue** advances the retained incompressible solution by the requested budget.
- **Pause / Resume** suspends or resumes the active solve.
- **Stop** ends the active incompressible run while retaining accepted fields.
- **Initialize** resets the field, iteration count and physical time. Analytical initialisation is available for cases such as Taylor–Green.

For compressible cases, use **Run compressible** and its Pause/Stop controls.

## Results

Select a run and field, then choose the full domain or a section plane. Display controls cover contours, vectors, colour range, cell edges and playback. Use probes and line profiles to inspect values; use plots for convergence and engineering histories.

Recording and display have separate intervals:

| Setting | Purpose |
| --- | --- |
| Full-field recording | Saves snapshots for later inspection and playback |
| Plane recording | Saves samples on the selected section |
| Live refresh | Controls how often the current field is prepared for display |
| Playback FPS | Controls animation speed |

PNG exports the current view. GIF export lets you choose resolution, frame range and size limits; the export can be cancelled.

## Examples

**File → Examples** opens the case library. Search or filter by category, review the source and settings, and choose a mesh or time-step preset before creating the workspace.

Use **Import case file**, **Export selected** or **Reload files** to manage the library. See [Case format](ADDING_A_NEW_CASE.md).
