# Boundary scripts

Rhai functions provide velocity, pressure or temperature values at a boundary face. Add a script in the GUI, then bind its function to the relevant patch and quantity.

```rhai
fn profile(x, y, z) {
    vec3(y, 0.0, 0.0)
}
```

Steady functions receive `(x, y, z)`. Time-dependent functions receive `(x, y, z, t)` and use `time_dependent: true`. Parameters supplied by the binding are available through `params`.

Return `vec3(...)` for velocity or a number for pressure and temperature. Coordinates and velocity use metres and m/s; incompressible pressure uses m²/s² and temperature uses kelvin.

| Script | Purpose |
| --- | --- |
| `couette-profile.rhai` | Linear shear profile |
| `parabolic-inlet.rhai` | Parabolic inlet velocity |
| `pulsating-inlet.rhai` | Time-dependent inlet velocity |
| `pressure-ramp.rhai` | Time-dependent pressure |
| `backflow-pressure.rhai` | Pressure for a backflow outlet |
| `inlet-temperature.rhai` | Inlet temperature |
| `incoming-temperature.rhai` | Incoming-flow temperature |
| `wall-temperature.rhai` | Wall temperature |

Scripts run with bounded execution and no file or network access. Complete bindings are included in the scripted [case files](../cases).
