# Examples

Open **File → Examples** in the desktop app, or create a workspace from the CLI:

```sh
flursys examples list
flursys examples create laminar-channel ./channel --preset coarse
flursys project mesh ./channel
flursys project solve ./channel
```

| Folder | Contents |
| --- | --- |
| [cases](cases) | Editable case files, geometry, scripts and reference data |
| [validation](validation) | Results and reference comparisons |
| [scripting](scripting) | Rhai boundary functions |
| [catalog](catalog) | A sample material-data pack |
| [compressible](compressible) | Standalone ideal-gas solver input |
| [multiphysics](multiphysics) | Phase and heat-transport input |

The case library includes Couette flow, natural convection, cylinder flow, sudden expansion, Taylor–Green and Sod shock tube. Each case carries its settings, reference scales and sources.

Choose a mesh preset (`coarse`, `medium`, `fine`) or a time preset (`time-coarse`, `time-medium`, `time-fine`) when creating a case. Time presets are available for transient cases.

To add your own case, see [Case format](../docs/ADDING_A_NEW_CASE.md). **Reload files** refreshes local cases; Import and Export move cases between libraries. `FLURSYS_EXAMPLES_DIR` selects an additional library directory.

For Rust automation, start with [headless.rs](headless.rs), [headless_control.rs](headless_control.rs) or [headless_sweep.rs](headless_sweep.rs).
