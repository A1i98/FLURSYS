# Numerical Methods

## 1. Scope of the Current Solver

FLURSYS presently provides two structured-grid incompressible-flow backends:

- a **two-dimensional finite-volume solver** for laminar incompressible flow, with either transient projection coupling or a steady SIMPLE-style iteration;
- an **initial three-dimensional staggered-grid projection solver** specialized to the lid-driven cavity.

The 2D backend is the primary solver. It supports constant-density Newtonian flow, optional constant-property thermal transport, optional Boussinesq buoyancy, embedded solid masks for the built-in geometries, configurable first-order-upwind or central momentum convection, and either PCG or SOR for the pressure equation.

The current implementation does **not** contain a turbulence closure, compressible formulation, multiphase model, combustion model, general species transport, or a general unstructured-mesh solver. The distinction matters: an unsteady laminar calculation may develop vortical structures, separation, and periodic shedding, but this is not equivalent to solving Reynolds-averaged or large-eddy turbulence equations.

### 1.1 Primary implementation locations

The principal source files associated with the methods described here are:

| Subject | Source |
| --- | --- |
| 2D incompressible solver | `src/solver/incompressible.rs` |
| 3D lid-driven-cavity solver | `src/solver/incompressible_3d.rs` |
| Structured grids | `src/grid.rs` |
| Thermal and buoyancy models | `src/physics.rs` |
| Cases and physical parameters | `src/cases.rs` |
| Boundary and preprocessing model | `src/preprocess.rs` |
| Project-to-solver configuration | `src/project.rs` |
| Field and VTK/CSV output | `src/output.rs` |

---

## 2. Governing Equations

### 2.1 Incompressible continuity equation

For constant-density incompressible flow,

$$
\nabla\cdot\mathbf{u}=0,
$$

where, in two dimensions,

$$
\mathbf{u}=(u,v).
$$

In Cartesian coordinates,

$$
\frac{\partial u}{\partial x}+\frac{\partial v}{\partial y}=0.
$$

In the 3D cavity backend,

$$
\mathbf{u}=(u,v,w),
$$

and continuity becomes

$$
\frac{\partial u}{\partial x}
+\frac{\partial v}{\partial y}
+\frac{\partial w}{\partial z}=0.
$$

### 2.2 Momentum equation

The 2D solver advances the incompressible Newtonian momentum equation in kinematic-viscosity form,

$$
\frac{\partial \mathbf{u}}{\partial t}
+\nabla\cdot(\mathbf{u}\otimes\mathbf{u})
=-\frac{1}{\rho}\nabla p
+\nu\nabla^2\mathbf{u}
+\mathbf{b},
$$

subject to incompressibility. Here

- $\rho$ is density,
- $p$ is pressure,
- $\nu=\mu/\rho$ is kinematic viscosity,
- $\mathbf{b}$ is acceleration due to an enabled body-force model.

For a divergence-free velocity field, the conservative convective form above is equivalent to the advective form $(\mathbf{u}\cdot\nabla)\mathbf{u}$. The 2D implementation evaluates convection as face fluxes and is therefore most naturally interpreted in conservative form.

In component form,

$$
\frac{\partial u}{\partial t}
+\frac{\partial u^2}{\partial x}
+\frac{\partial uv}{\partial y}
=-\frac{1}{\rho}\frac{\partial p}{\partial x}
+\nu\left(
\frac{\partial^2 u}{\partial x^2}
+\frac{\partial^2 u}{\partial y^2}
\right)+b_x,
$$

$$
\frac{\partial v}{\partial t}
+\frac{\partial uv}{\partial x}
+\frac{\partial v^2}{\partial y}
=-\frac{1}{\rho}\frac{\partial p}{\partial y}
+\nu\left(
\frac{\partial^2 v}{\partial x^2}
+\frac{\partial^2 v}{\partial y^2}
\right)+b_y.
$$

### 2.3 Reynolds number and viscosity

For project-defined canonical cases, the kinematic viscosity is constructed from the specified Reynolds number and the characteristic velocity and length of that case:

$$
\mathrm{Re}=\frac{U_{\mathrm{ref}}L_{\mathrm{ref}}}{\nu}.
$$

The present project conversion uses the following characteristic scales:

| Case | $U_{\mathrm{ref}}$ | $L_{\mathrm{ref}}$ |
| --- | ---: | ---: |
| Lid-driven cavity | lid speed | cavity length |
| Cylinder | freestream speed | cylinder diameter |
| Backward-facing step | mean inlet speed | step height |
| Plane channel | mean speed | channel height |

Consequently,

$$
\nu=\frac{U_{\mathrm{ref}}L_{\mathrm{ref}}}{\mathrm{Re}}.
$$

---

## 3. Thermal Transport and Buoyancy

### 3.1 Constant-property energy equation

When the thermal model is enabled, FLURSYS solves a cell-centred temperature transport equation with constant thermal diffusivity:

$$
\frac{\partial T}{\partial t}
+\nabla\cdot(\mathbf{u}T)
=\alpha\nabla^2T+S_T.
$$

For incompressible flow this is equivalent to

$$
\frac{\partial T}{\partial t}
+\mathbf{u}\cdot\nabla T
=\alpha\nabla^2T+S_T.
$$

The parameter

$$
\alpha=\frac{k}{\rho c_p}
$$

is supplied directly as the thermal diffusivity. The source $S_T$ is stored as a **temperature rate** with units K/s. Accordingly, the current public thermal configuration does not separately expose $k$, $c_p$, and a volumetric source $q'''$; those quantities are represented through $\alpha$ and $S_T$.

The equivalent dimensional energy equation is

$$
\rho c_p
\left(
\frac{\partial T}{\partial t}
+\mathbf{u}\cdot\nabla T
\right)
=k\nabla^2T+q''',
$$

with

$$
S_T=\frac{q'''}{\rho c_p}.
$$

This latter form is useful for interpretation, but the solver advances the diffusivity/source-rate form.

### 3.2 Boussinesq buoyancy

The optional Boussinesq model adds a temperature-dependent acceleration to momentum:

$$
\mathbf{b}
=\mathbf{g}\,\beta\,(T-T_{\mathrm{ref}}),
$$

where $\beta$ is the thermal expansion coefficient and $\mathbf{g}=(g_x,g_y)$ is the configured gravity vector. The sign of the force therefore follows the sign convention supplied through the gravity components.

Temperature is interpolated from adjacent cell centres to the corresponding staggered velocity face before the buoyancy term is applied. Boussinesq buoyancy is only valid in FLURSYS when the constant-property energy equation is enabled.

The model should be interpreted in the usual small-density-variation sense: density is treated as constant in continuity, inertia, pressure coupling, and material properties, while the temperature-induced density variation is represented only through the body-force term.

---

## 4. Structured Staggered Grid

### 4.1 Two-dimensional arrangement

The 2D backend uses a uniform Cartesian staggered arrangement. For a domain of length $L$ and height $H$,

$$
\Delta x=\frac{L}{N_x},
\qquad
\Delta y=\frac{H}{N_y}.
$$

Pressure and scalar quantities are stored at cell centres. Velocity components are stored at the faces normal to their respective directions:

- $p_{i,j}$ and $T_{i,j}$: cell centre,
- $u_{i,j}$: vertical face,
- $v_{i,j}$: horizontal face.

The coordinate locations are

$$
x_{p,i}=\left(i+\frac12\right)\Delta x,
\qquad
y_{p,j}=\left(j+\frac12\right)\Delta y,
$$

$$
x_{u,i}=i\Delta x,
\qquad
y_{u,j}=\left(j+\frac12\right)\Delta y,
$$

$$
x_{v,i}=\left(i+\frac12\right)\Delta x,
\qquad
y_{v,j}=j\Delta y.
$$

This arrangement provides a compact discrete divergence and pressure-gradient pair and avoids the pressure–velocity decoupling associated with an uncorrected collocated discretization.

### 4.2 Three-dimensional arrangement

The cavity backend extends the same MAC-style arrangement to three dimensions:

- pressure at $(N_x,N_y,N_z)$ cell centres,
- $u$ on $x$-normal faces,
- $v$ on $y$-normal faces,
- $w$ on $z$-normal faces.

The 3D grid is uniform in each coordinate, with $\Delta x=L/N_x$, $\Delta y=H/N_y$, and $\Delta z=D/N_z$.

---

## 5. Spatial Discretization in the 2D Momentum Solver

### 5.1 Finite-volume flux form

For an $x$-momentum face, the implementation forms transporting velocities at the neighbouring flux locations and evaluates the conservative convection term as

$$
\mathcal{C}_u
\approx
\frac{u_e\,\phi_e-u_w\,\phi_w}{\Delta x}
+
\frac{v_n\,\phi_n-v_s\,\phi_s}{\Delta y},
$$

where the transported quantity $\phi$ is the $u$ component. An analogous construction is used for $v$ momentum.

The cross-component transporting velocities are obtained from local arithmetic averages of the surrounding staggered faces. This is the standard interpolation required to evaluate fluxes at locations where the two staggered velocity components do not coincide.

### 5.2 Convection schemes

Two face-value rules are currently available for momentum.

#### First-order upwind

For a generic face velocity $U_f$ between left/upstream candidate $\phi_L$ and right/downstream candidate $\phi_R$,

$$
\phi_f=
\begin{cases}
\phi_L, & U_f\ge 0,\\[4pt]
\phi_R, & U_f<0.
\end{cases}
$$

This scheme is robust and bounded for the present linear face interpolation, at the cost of first-order numerical diffusion.

#### Central interpolation

The central option uses

$$
\phi_f=\frac{\phi_L+\phi_R}{2}.
$$

On a uniform grid and away from boundaries or masked solids this is nominally second-order in space for smooth fields. It is less dissipative than upwind but does not introduce a nonlinear boundedness mechanism; consequently, adequate resolution and time-step control remain the responsibility of the simulation setup.

Project files can select either `first-order-upwind` or `central` for the momentum convection scheme.

### 5.3 Viscous diffusion

For a generic face-centred velocity component $\phi$,

$$
\nabla^2\phi
\approx
\frac{\phi_E-2\phi_P+\phi_W}{\Delta x^2}
+
\frac{\phi_N-2\phi_P+\phi_S}{\Delta y^2}.
$$

This is the conventional second-order Cartesian Laplacian in an unobstructed interior region.

### 5.4 Time integration

Momentum is advanced explicitly with a first-order forward-Euler update. In generic form,

$$
\phi^*=
\phi^n
+\Delta t\,
\left[-\mathcal{C}(\phi)
+\nu\nabla_h^2\phi
-\frac{1}{\rho}(\nabla_h p)_\phi
+b_\phi\right].
$$

For the transient projection method, the pressure-gradient term is omitted from the predictor and introduced through the subsequent projection. For SIMPLE-style iterations, the current pressure field is included in the momentum predictor.

The current 2D solver uses a fixed configured $\Delta t$. Automatic adaptive time stepping is not yet part of the implementation described here.

---

## 6. Pressure–Velocity Coupling

FLURSYS provides two distinct 2D coupling modes. They share the same staggered operators and pressure solvers, but their interpretation of the momentum predictor and pressure field differs.

## 6.1 Transient projection method

The projection path is a fractional-step method.

### Step 1 — momentum predictor

An intermediate velocity is computed without the new pressure gradient:

$$
\frac{\mathbf{u}^*-\mathbf{u}^n}{\Delta t}
=
-\nabla_h\cdot(\mathbf{u}\otimes\mathbf{u})^n
+\nu\nabla_h^2\mathbf{u}^n
+\mathbf{b}^n.
$$

### Step 2 — pressure Poisson equation

Requiring the corrected velocity to satisfy discrete continuity gives

$$
\nabla_h^2p^{n+1}
=
\frac{\rho}{\Delta t}
\nabla_h\cdot\mathbf{u}^*.
$$

For each fluid cell, the discrete divergence is

$$
(\nabla_h\cdot\mathbf{u})_{i,j}
=
\frac{u_{i+1,j}-u_{i,j}}{\Delta x}
+
\frac{v_{i,j+1}-v_{i,j}}{\Delta y}.
$$

### Step 3 — velocity correction

The face velocities are projected using the pressure gradient:

$$
u^{n+1}_{i,j}
=
u^*_{i,j}
-\frac{\Delta t}{\rho}
\frac{p_{i,j}-p_{i-1,j}}{\Delta x},
$$

$$
v^{n+1}_{i,j}
=
v^*_{i,j}
-\frac{\Delta t}{\rho}
\frac{p_{i,j}-p_{i,j-1}}{\Delta y}.
$$

Boundary conditions are re-applied after correction.

### Step 4 — scalar update and diagnostics

Cell-centred velocity, speed, vorticity, optional temperature, continuity defect, pressure residual, force coefficients, and stability indicators are then updated.

A compact view of the transient algorithm is

```text
Known state at n
      │
      ├─ apply velocity boundary conditions
      │
      ├─ explicit momentum predictor → u*, v*
      │
      ├─ build divergence-based pressure RHS
      │
      ├─ solve pressure Poisson equation
      │
      ├─ pressure-gradient velocity correction
      │
      ├─ re-apply velocity boundary conditions
      │
      ├─ advance physical time
      │
      ├─ reconstruct cell-centred fields
      │
      ├─ advance temperature, if enabled
      │
      └─ compute residuals and engineering diagnostics
```

## 6.2 SIMPLE-style steady coupling

The second mode is intentionally described as **SIMPLE-style** rather than as a textbook fully implicit SIMPLE implementation. FLURSYS performs segregated pseudo-time momentum iterations with velocity and pressure under-relaxation.

Let $k$ denote the outer SIMPLE iteration and let $\alpha_u$ and $\alpha_p$ be the configured velocity and pressure relaxation factors.

### Momentum predictor with the current pressure

The predictor includes the current pressure gradient and applies the velocity relaxation factor to the explicit pseudo-time increment:

$$
\mathbf{u}^*
=
\mathbf{u}^k
+\alpha_u\Delta t_p
\left[
-\nabla_h\cdot(\mathbf{u}\otimes\mathbf{u})^k
+\nu\nabla_h^2\mathbf{u}^k
-\frac{1}{\rho}\nabla_h p^k
+\mathbf{b}^k
\right],
$$

where $\Delta t_p$ is the configured pseudo-time step.

### Pressure-correction equation

The pressure storage is temporarily used for a correction $p'$. The RHS scaling is consistent with the relaxed velocity correction:

$$
\nabla_h^2p'
=
\frac{\rho}{\alpha_u\Delta t_p}
\nabla_h\cdot\mathbf{u}^*.
$$

### Velocity and pressure correction

Velocity is corrected as

$$
\mathbf{u}^{k+1}
=
\mathbf{u}^*
-\frac{\alpha_u\Delta t_p}{\rho}\nabla_h p',
$$

and pressure is under-relaxed according to

$$
p^{k+1}=p^k+\alpha_p p'.
$$

This formulation is suitable for the current structured solver and its steady iterations, but it should not be confused with a general coefficient-based SIMPLE implementation for arbitrary collocated or unstructured finite-volume systems.

---

## 7. Pressure Equation and Linear Solvers

### 7.1 Discrete Poisson operator

Away from solids and Dirichlet pressure cells, the 2D pressure Laplacian is

$$
(\nabla_h^2p)_{i,j}
=
\frac{p_{i+1,j}-2p_{i,j}+p_{i-1,j}}{\Delta x^2}
+
\frac{p_{i,j+1}-2p_{i,j}+p_{i,j-1}}{\Delta y^2}.
$$

At masked or domain boundaries, unavailable neighbours are removed from the active stencil in a manner corresponding to a zero-normal-gradient treatment unless a pressure outlet imposes a Dirichlet value.

### 7.2 SOR

The Successive Over-Relaxation solver updates an active pressure cell using

$$
p_P^{\mathrm{new}}
=p_P^{\mathrm{old}}
+\omega
\left(
\frac{\displaystyle\sum_N a_Np_N-b_P}{a_P}
-p_P^{\mathrm{old}}
\right),
$$

with

$$
0<\omega<2.
$$

SOR is implemented as an in-place sequential sweep. It is therefore not parallelized by the 2D Rayon worker pool.

### 7.3 Preconditioned conjugate gradient

The alternative pressure solver is PCG applied to the symmetric structured pressure operator, using a Jacobi preconditioner:

$$
M^{-1}\approx\operatorname{diag}(A)^{-1}.
$$

The expensive stencil application, vector updates, dot products, and maximum-residual reductions are parallelized with Rayon. Solver work vectors are retained between outer iterations to avoid repeated allocation.

The stopping quantity is the maximum absolute algebraic residual,

$$
R_p=\|\mathbf{b}-A\mathbf{p}\|_\infty,
$$

and the solve terminates when

$$
R_p<\varepsilon_p
$$

or the configured pressure-iteration limit is reached.

### 7.4 Pressure reference and null space

For a purely Neumann pressure problem, pressure is defined only up to an additive constant. FLURSYS removes this null space explicitly:

- SOR subtracts a reference pressure from the fluid field;
- PCG projects pressure-related vectors to zero mean over non-solid, non-Dirichlet cells.

When a right-side pressure outlet is active, that Dirichlet pressure supplies the physical reference and zero-mean projection is not required.

At present, the executable 2D project validator permits a pressure outlet only on the **right boundary**.

---

## 8. Velocity Boundary Conditions and Solid Masks

### 8.1 Domain boundaries

The active 2D boundary system supports the following flow boundary categories:

- prescribed velocity,
- pressure outlet,
- wall/prescribed wall velocity through the case or project boundary data,
- symmetry,
- case-default behavior.

For the velocity component normal to a domain face:

- a velocity boundary assigns the prescribed normal component;
- symmetry sets the normal velocity to zero;
- a pressure outlet copies the adjacent interior normal velocity, corresponding to a zero-normal-gradient extrapolation.

Tangential conditions are represented through ghost values in the momentum stencil. For a prescribed tangential wall velocity $u_w$, a mirrored ghost value is constructed as

$$
u_g=2u_w-u_P.
$$

A stationary no-slip wall therefore gives $u_g=-u_P$. The corresponding construction is used for the $v$ component.

### 8.2 Embedded solids

Built-in immersed geometry is represented through a cell-centred Boolean solid mask. A staggered face is considered open only when the adjacent fluid topology permits flux through that face. Blocked faces are explicitly zeroed.

For momentum stencils adjacent to a solid, a mirrored velocity neighbour is used, providing a no-slip-like wall treatment at the mask interface. The geometry is consequently represented at Cartesian cell resolution; it is not a body-fitted curvilinear discretization.

This distinction is important for interpreting surface forces and near-wall gradients on curved objects. Cylinder forces, for example, are evaluated on the staircase representation induced by the Cartesian mask.

---

## 9. Thermal Discretization and Thermal Boundary Conditions

### 9.1 Explicit scalar update

Temperature is advanced at cell centres with a first-order explicit finite-volume update:

$$
T_{i,j}^{n+1}
=
T_{i,j}^{n}
+\Delta t
\left[
-\mathcal{C}_T
+\alpha\nabla_h^2T
+S_T
\right].
$$

Thermal convection currently uses first-order upwind face values irrespective of the momentum convection option. For example,

$$
F_e=u_eT_{e,\mathrm{upwind}},
$$

and

$$
\mathcal{C}_T
=
\frac{F_e-F_w}{\Delta x}
+
\frac{F_n-F_s}{\Delta y}.
$$

Thermal diffusion uses the same Cartesian second-difference Laplacian as the momentum diffusion term.

### 9.2 Adiabatic boundary

For an adiabatic domain boundary,

$$
\frac{\partial T}{\partial n}=0.
$$

The implemented ghost treatment sets the neighbour value equal to the adjacent cell-centre value,

$$
T_g=T_P,
$$

which gives zero normal difference at the boundary in the discrete stencil.

### 9.3 Fixed-temperature boundary

For a prescribed wall temperature $T_w$, the ghost value is

$$
T_g=2T_w-T_P.
$$

This places the desired wall temperature halfway between the interior and ghost cell centres, consistent with the Cartesian cell-centred arrangement.

### 9.4 Thermal treatment of masked solids

If a temperature stencil encounters an internal solid cell, the neighbour temperature is replaced by the current fluid-cell value. The present embedded-solid thermal treatment therefore acts as a zero-normal-gradient, effectively adiabatic interface. Prescribed thermal conditions on arbitrary immersed-solid surfaces are not yet implemented.

---

## 10. Stability Measures

Because momentum and temperature are advanced explicitly, time-step selection is a material part of solution quality. FLURSYS reports reference stability measures; these should be interpreted as diagnostics rather than universal sufficiency conditions.

### 10.1 Momentum Courant number

After a completed 2D step, FLURSYS computes

$$
\mathrm{CFL}_m
=\Delta t
\left(
\frac{\max|u|}{\Delta x}
+
\frac{\max|v|}{\Delta y}
\right),
$$

using maxima from the staggered velocity fields.

The solver currently reports $\mathrm{CFL}_m\le 1$ as a common explicit reference limit. It does **not** automatically reduce the time step when this value is exceeded.

### 10.2 Momentum viscous diffusion number

The corresponding viscous diagnostic is

$$
D_\nu
=\nu\Delta t
\left(
\frac{1}{\Delta x^2}
+
\frac{1}{\Delta y^2}
\right).
$$

A value $D_\nu\le 0.5$ is reported as a common explicit reference bound for the two-dimensional diffusion operator. Again, this value is diagnostic; momentum stepping is not currently rejected solely because $D_\nu$ is large.

### 10.3 Thermal advection CFL

When thermal transport is active, the runtime thermal update evaluates

$$
\mathrm{CFL}_T
=\Delta t
\left(
\frac{\max|u_c|}{\Delta x}
+
\frac{\max|v_c|}{\Delta y}
\right),
$$

where $u_c$ and $v_c$ are reconstructed cell-centred velocities. The thermal step is rejected if

$$
\mathrm{CFL}_T>1.
$$

### 10.4 Thermal diffusion condition

Configuration validation evaluates

$$
D_T
=\alpha\Delta t
\left(
\frac{1}{\Delta x^2}
+
\frac{1}{\Delta y^2}
\right),
$$

and requires

$$
D_T\le\frac12.
$$

This is the standard forward-Euler Cartesian diffusion restriction for the implemented 2D scalar Laplacian.

### 10.5 Practical interpretation

The above bounds do not replace mesh and time-step refinement studies. Convective boundedness, immersed-boundary geometry, pressure-solver tolerance, Reynolds number, and the selected convection scheme all influence the behavior of a calculation. In particular, central convection may require substantially more conservative resolution and time-step choices than first-order upwind in convection-dominated regimes.

---

## 11. Residuals and Convergence Measures

FLURSYS records several distinct quantities. They should not be conflated: the pressure residual measures the Poisson solve, the continuity residual measures the corrected velocity field, and the momentum quantity currently reported by the interactive step interface is a change between successive velocity fields.

### 11.1 Pressure residual

For the stencil pressure solver, the reported pressure residual is the maximum local Poisson defect,

$$
R_p
=
\max_{(i,j)\in\Omega_f}
\left|
(\nabla_h^2p)_{i,j}-b_{i,j}
\right|,
$$

excluding solid and Dirichlet outlet cells. PCG equivalently tracks the maximum absolute algebraic residual.

### 11.2 Continuity residual

The corrected-flow continuity measure is

$$
R_c
=
\max_{(i,j)\in\Omega_f}
\left|
\frac{u_{i+1,j}-u_{i,j}}{\Delta x}
+
\frac{v_{i,j+1}-v_{i,j}}{\Delta y}
\right|.
$$

This is an absolute dimensional divergence measure, not a normalized mass-imbalance norm.

### 11.3 Velocity-change measure

The current steady-change diagnostic is

$$
R_u
=
\max
\left(
\|u^{n+1}-u^n\|_\infty,
\|v^{n+1}-v^n\|_\infty
\right).
$$

For cases eligible for steady stopping, convergence is declared only after the configured minimum number of iterations/steps and when

$$
R_u<\varepsilon_{\mathrm{steady}}
$$

and

$$
R_c<10\,\varepsilon_p.
$$

The cylinder case is deliberately excluded from this steady stopping rule because the canonical wake case is intended to admit sustained unsteady vortex shedding.

### 11.4 Current limitation of the convergence metrics

The present residual quantities are absolute rather than nondimensionalized or normalized by characteristic scales. They are useful for monitoring a fixed problem, but tolerances should not be assumed to have identical physical significance across widely different domain scales, velocities, viscosities, or grid spacings.

---

## 12. Reconstructed and Derived Fields

### 12.1 Cell-centred velocity

For visualization and post-processing, staggered velocities are reconstructed at cell centres by arithmetic averaging:

$$
u_{c,i,j}
=\frac12\left(u_{i,j}+u_{i+1,j}\right),
$$

$$
v_{c,i,j}
=\frac12\left(v_{i,j}+v_{i,j+1}\right).
$$

The speed magnitude is

$$
|\mathbf{u}_c|
=\sqrt{u_c^2+v_c^2}.
$$

### 12.2 Vorticity

The 2D out-of-plane vorticity is

$$
\omega_z
=\frac{\partial v}{\partial x}
-\frac{\partial u}{\partial y}.
$$

FLURSYS evaluates derivatives from the reconstructed cell-centred velocity fields. Central differences are used where unobstructed neighbours exist on both sides; one-sided differences are used near a solid or domain edge when only one valid neighbour is available.

### 12.3 Cylinder force coefficients

For the cylinder case, pressure and an approximate viscous contribution are integrated over fluid cells adjacent to the Cartesian solid mask. The resulting forces per unit span are normalized as

$$
C_D=\frac{F_x}{\frac12\rho U_\infty^2D},
\qquad
C_L=\frac{F_y}{\frac12\rho U_\infty^2D}.
$$

The viscous contribution uses a near-wall gradient inferred from the adjacent cell-centred tangential velocity over half a grid spacing. Consequently, $C_D$ and $C_L$ should be expected to exhibit mesh sensitivity associated with the staircase representation of the circular boundary.

### 12.4 Backward-facing-step reattachment length

For the backward-facing-step case, the bottom-wall shear proxy is

$$
\tau_w
\approx
\mu\frac{u_c}{\Delta y/2}.
$$

Downstream of the step, the solver searches for a change from non-positive to positive wall shear and linearly interpolates the zero crossing. The reported nondimensional reattachment length is

$$
\frac{x_r-x_s}{h_s},
$$

where $x_s$ is the step location and $h_s$ is the step height.

---

## 13. Output Quantities

The 2D solver writes field and history information intended for both numerical inspection and external visualization.

### 13.1 History

`history.csv` presently contains

```text
step
 time
 pressure_residual
 pressure_iterations
 max_divergence
 velocity_change
 max_speed
 cd
 cl
 reattachment_x_over_h
 momentum_cfl
 viscous_diffusion_number
```

The drag, lift, and reattachment columns are case-dependent diagnostics and are zero or non-applicable outside their corresponding canonical cases.

### 13.2 Field output

The final 2D field output includes, as applicable,

- pressure,
- reconstructed cell-centred velocity components,
- speed,
- vorticity,
- solid mask information,
- temperature.

CSV and legacy VTK output are provided, with PPM frame generation for selected visual fields during a run.

---

## 14. Three-Dimensional Lid-Driven-Cavity Backend

The current 3D solver is a genuine staggered-grid incompressible projection implementation, but it is intentionally narrower than the 2D backend.

### 14.1 Governing model

The 3D backend solves constant-density laminar incompressible flow,

$$
\nabla\cdot\mathbf{u}=0,
$$

$$
\frac{\partial\mathbf{u}}{\partial t}
+(\mathbf{u}\cdot\nabla)\mathbf{u}
=-\frac{1}{\rho}\nabla p
+\nu\nabla^2\mathbf{u},
$$

for a rectangular lid-driven cavity.

### 14.2 Momentum discretization

Unlike the 2D momentum path, the current 3D predictor is written in an explicit advective form. Interior directional derivatives use centred differences, for example

$$
\frac{\partial u}{\partial x}
\approx
\frac{u_E-u_W}{2\Delta x},
$$

while cross-component transporting velocities are reconstructed by local four-point averages at the required staggered locations. Viscous diffusion uses the standard 7-point Cartesian Laplacian.

The time update is first-order forward Euler with fixed $\Delta t$.

### 14.3 3D pressure projection

The pressure RHS is

$$
b_{i,j,k}
=
\frac{\rho}{\Delta t}
\left[
\frac{u^*_{i+1,j,k}-u^*_{i,j,k}}{\Delta x}
+
\frac{v^*_{i,j+1,k}-v^*_{i,j,k}}{\Delta y}
+
\frac{w^*_{i,j,k+1}-w^*_{i,j,k}}{\Delta z}
\right].
$$

Pressure is solved with SOR on the 3D Cartesian Poisson stencil. The cell $(0,0,0)$ is fixed to zero to remove the pressure null space. Velocity is then corrected with the corresponding face-centred pressure gradients.

### 14.4 3D boundary treatment

The top lid imposes the configured tangential lid velocity. The remaining cavity walls use reflected ghost values consistent with no-slip behavior. The executable project path currently accepts only the supported cavity-default boundary workflow; front and back project faces are retained as symmetry intent in the workbench representation.

### 14.5 Present 3D limitations

The current 3D backend:

- supports only the lid-driven cavity execution path;
- requires a structured mesh;
- does not mesh project CAD solids;
- does not support the 2D thermal or Boussinesq models;
- uses SOR rather than the 2D PCG pressure path;
- uses a fixed time step;
- advances the configured number of steps rather than applying the 2D steady stopping logic;
- does not yet provide the general boundary flexibility of the 2D backend.

For these reasons, 3D capability should presently be regarded as an initial structured projection backend, not as feature parity with the 2D solver.

---

## 15. Parallel Execution

The 2D solver uses Rayon for shared-memory CPU parallelism in operations where data dependencies permit it. Parallelized work includes major momentum loops, matrix-free PCG operator applications, vector updates and reductions, and cell-field reconstruction.

SOR remains sequential because it is implemented as an in-place ordered sweep. Selecting multiple Rayon threads therefore does not make the SOR pressure iteration itself parallel; PCG is the appropriate pressure choice when pressure-solve parallelism is desired.

The current 3D cavity implementation is predominantly sequential.

Parallel execution changes the reduction order of floating-point operations. Small last-bit differences between thread counts are therefore normal and should not be interpreted as a change in the mathematical method.

---

## 16. Accuracy, Robustness, and Interpretation

The formal order of a CFD scheme is only one part of its practical error behavior. For the present solver, the following distinctions are important.

### 16.1 Interior truncation behavior

In a smooth unobstructed region of the 2D Cartesian grid:

- forward Euler is first-order in time;
- first-order upwind momentum convection is first-order along the upwind direction;
- central momentum interpolation is nominally second-order in space;
- viscous diffusion is nominally second-order in space;
- pressure gradients and divergence use centred staggered differences;
- thermal advection is first-order upwind;
- thermal diffusion is nominally second-order in space.

### 16.2 Boundaries and masked geometry

Local accuracy can differ near domain boundaries, solid masks, and locations where one-sided derivative reconstruction is required. Curved built-in solids are represented through a Cartesian mask rather than a body-fitted surface, so geometric error may dominate local truncation error until the grid is sufficiently refined.

### 16.3 Numerical diffusion versus oscillation

First-order upwind is deliberately dissipative. It may smear shear layers and vortical structures but is often robust on coarse grids. Central convection reduces artificial diffusion but can expose under-resolution more directly. FLURSYS currently does not apply TVD, MUSCL, flux limiting, or artificial dissipation to the central option.

### 16.4 Fixed time step

The 2D and 3D momentum solvers currently use a configured fixed time step. The reported momentum CFL and diffusion number aid diagnosis, but the solver does not yet perform automatic step-size adaptation or step rejection for the momentum equation.

### 16.5 Solver tolerance is not discretization error

A small pressure residual indicates that the discrete Poisson system has been solved to the requested algebraic tolerance. It does not establish that the grid is sufficiently fine, that the time step is sufficiently small, or that the underlying physical model is appropriate. Algebraic convergence, iterative convergence, and discretization convergence are separate requirements.

---

## 17. Current Capability Boundary

The following table summarizes the numerical scope described by this document.

| Capability | Current status |
| --- | --- |
| 2D incompressible laminar Navier–Stokes | Implemented |
| Uniform Cartesian staggered grid | Implemented |
| Transient projection coupling | Implemented |
| SIMPLE-style steady coupling | Implemented |
| First-order-upwind momentum convection | Implemented |
| Central momentum convection | Implemented |
| PCG + Jacobi pressure solve in 2D | Implemented |
| SOR pressure solve in 2D | Implemented |
| Constant-property temperature transport | Implemented |
| Adiabatic thermal boundary | Implemented |
| Fixed-temperature thermal boundary | Implemented |
| Boussinesq buoyancy | Implemented |
| Momentum CFL and viscous diagnostics | Implemented |
| Cylinder $C_D$ and $C_L$ diagnostics | Implemented |
| Backward-step reattachment diagnostic | Implemented |
| Initial 3D staggered cavity projection | Implemented |
| Automatic adaptive momentum time step | Not implemented |
| Turbulence closure ($k$–$\varepsilon$, $k$–$\omega$ SST, LES) | Not implemented |
| Compressible flow | Not implemented |
| Multiphase flow | Not implemented |
| General scalar/species transport | Not implemented |
| Heat-flux / Robin thermal boundary | Not implemented |
| General pressure outlet on all 2D faces | Not implemented |
| Unstructured finite-volume solver | Not implemented |
| General CAD-to-3D flow solve | Not implemented |

## References

The following references provide the broader numerical context for the methods used in FLURSYS. They are not substitutes for the implementation details given above.

1. A. J. Chorin, “Numerical Solution of the Navier–Stokes Equations,” *Mathematics of Computation*, 22, 745–762, 1968.
2. S. V. Patankar, *Numerical Heat Transfer and Fluid Flow*, Hemisphere Publishing, 1980.
3. J. H. Ferziger, M. Perić, and R. L. Street, *Computational Methods for Fluid Dynamics*, 4th ed., Springer, 2020.
4. H. K. Versteeg and W. Malalasekera, *An Introduction to Computational Fluid Dynamics: The Finite Volume Method*, 2nd ed., Pearson, 2007.
5. F. Moukalled, L. Mangani, and M. Darwish, *The Finite Volume Method in Computational Fluid Dynamics*, Springer, 2016.
