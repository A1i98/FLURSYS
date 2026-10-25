# Numerical methods

FLURSYS uses a cell-centred finite-volume formulation on 2D and 3D meshes. Velocity, pressure and transported quantities are stored per cell; conservative fluxes are evaluated at faces.

## Incompressible flow

The solver advances continuity and momentum with constant reference density:

\[
\nabla\cdot\mathbf{u}=0,
\qquad
\frac{\partial\mathbf{u}}{\partial t}+\nabla\cdot(\mathbf{u}\otimes\mathbf{u})
=-\nabla(p/\rho)+\nabla\cdot(\nu_{\mathrm{eff}}\nabla\mathbf{u})+\mathbf{f}.
\]

Pressure–velocity coupling is selected through SIMPLE, PISO or PIMPLE controls. Pressure corrections enforce the face-flux continuity equation. Velocity and pressure relaxation control the nonlinear updates.

Momentum convection offers first-order upwind and limited linear reconstruction. Diffusion includes non-orthogonal corrections. Linear solver tolerances and preconditioners can be assigned per equation.

## Time integration

Steady runs use nonlinear iterations. Transient runs advance physical time with implicit Euler or BDF2; adaptive variants choose the step from Courant and optional diffusion limits.

For fixed-step BDF2:

\[
\left.\frac{\partial\phi}{\partial t}\right|^{n+1}
\approx\frac{3\phi^{n+1}-4\phi^n+\phi^{n-1}}{2\Delta t}.
\]

Accepted fields and time history are retained in checkpoints. See [Transient flow](TRANSIENT_TURBULENCE.md).

## Turbulence

RANS options include k–ω, SST, standard/RNG/realizable k–ε and Spalart–Allmaras. LES options include Smagorinsky and WALE. Inlet turbulence may be given directly or through intensity and a length, diameter or viscosity-ratio scale.

Choose wall treatment and first-cell spacing together. Inspect wall distance, y+ and wall shear when assessing near-wall resolution.

## Temperature and buoyancy

Temperature transport uses density, heat capacity and conductivity:

\[
\rho C_p\left(\frac{\partial T}{\partial t}+\mathbf{u}\cdot\nabla T\right)
=\nabla\cdot(k_{\mathrm{eff}}\nabla T)+Q.
\]

Boussinesq coupling uses a reference temperature, expansion coefficient and gravity vector. The temperature-dependent acceleration relative to the reference state is

\[
\mathbf{a}_b=-\beta(T-T_{\mathrm{ref}})\mathbf{g}.
\]

## Compressible flow

The ideal-gas solver advances conservative mass, momentum and total energy. Pressure is absolute and the closure is `p = ρRT`.

Flux options are Rusanov, HLLC and central upwind. Time integration uses SSP-RK2 with acoustic and viscous step restrictions. Initial regions specify discontinuous states such as a shock-tube diaphragm.

## Phase and heat transport

The multiphysics runner transports phase and energy fields using a prescribed face flux. Its options include material properties, solid regions and Cartesian or axisymmetric scalar coordinates.

## Boundary conditions

| Quantity | Choices |
| --- | --- |
| Incompressible flow | Velocity, parabolic or flow-rate inlet; pressure or total-pressure opening; backflow state; no-slip/moving/slip wall; symmetry; conformal translational cyclic pairs |
| Temperature | Fixed temperature, adiabatic, heat flux, convection or enclosure radiation |
| Compressible flow | Slip/no-slip wall, prescribed exterior state, supersonic inlet, pressure outlet or transmissive boundary |

Time tables and Rhai functions supply spatial or time-dependent boundary values.

## Reading results

- Incompressible pressure is kinematic gauge pressure, `p/ρ`, in m²/s².
- Compressible pressure is absolute pressure in Pa; scaled examples state their reference units in the case file.
- 2D fluxes and forces are per unit depth.
- Residual curves measure different equations or updates. Inspect conservation and the quantities of interest alongside convergence.
- Contours interpolate values for display; field exports retain the stored cell values.

Published case definitions, reference data and comparison procedures live in [examples/cases](../examples/cases).
