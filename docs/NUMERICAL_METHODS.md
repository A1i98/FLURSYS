# Numerical Methods

Implementation audit: 2026-10-20, including the current working-tree changes.
This document describes executable code, not a proposed feature set or evidence
of numerical validation. A successful build does not establish accuracy, order
of convergence, stability, or performance.

## 1. Execution paths and capability boundaries

FLURSYS has three distinct flow backends. Their settings, pressure conventions,
residuals and geometry representations must not be interchanged.

| Property | CAD/Gmsh Workbench | Structured 2D | Structured 3D cavity |
|---|---|---|---|
| Layout | Cell-centred, collocated finite volumes | Cartesian staggered/MAC | Cartesian staggered/MAC |
| Geometry | Imported/generated unstructured 2D or 3D mesh | Built-in geometry and Boolean solid masks | Rectangular cavity |
| Coupling | Steady SIMPLE | Projection, explicit SIMPLE-style, implicit SIMPLE, SIMPLEC, PIMPLE-like | Transient projection |
| Physical-time integration | None | Forward Euler for explicit predictor; Euler/BDF2 for implicit PIMPLE | Forward Euler |
| Momentum convection | Upwind or limited linear upwind | Upwind, central, Van Leer deferred correction in supported implicit modes | Centred advective predictor |
| Pressure | PCG + IC0 | Matrix-free PCG/SOR or CSR PCG, DIC/IC0, structured MG, GAMG | SOR |
| Thermal/Boussinesq | Not connected | Optional constant-property temperature/Boussinesq | Not connected |
| Pressure field | Kinematic gauge pressure, p/rho | Dimensional pressure, p | Dimensional pressure, p |

The structured PIMPLE, BDF2, GAMG and thermal implementations are **not** selected
by choosing a 3D mesh or increasing the iteration limit in Workbench. Workbench
iterations are nonlinear steady iterations, not physical time steps.

No backend currently provides a runnable RANS/LES closure, compressible flow,
VOF, reacting flow, species transport, MRF or moving-mesh flow model.

### Implementation map

| Subject | Authoritative source |
|---|---|
| Workbench case, material and physical BCs | [unstructured_incompressible.rs](../src/unstructured_incompressible.rs) |
| Collocated SIMPLE and pressure/flux correction | [simple.rs](../src/simple.rs) |
| General-mesh momentum assembly | [momentum.rs](../src/momentum.rs) |
| Limited linear-upwind correction | [momentum/convection.rs](../src/momentum/convection.rs) |
| Gradients, interpolation and diffusion | [numerics.rs](../src/numerics.rs) |
| CSR Krylov stopping and preconditioners | [linear/solver.rs](../src/linear/solver.rs), [preconditioner.rs](../src/linear/preconditioner.rs) |
| Structured 2D configuration and stopping | [solver/incompressible.rs](../src/solver/incompressible.rs) |
| Structured implicit coupling | [implicit.rs](../src/solver/incompressible/implicit.rs), [implicit_pressure](../src/solver/incompressible/implicit_pressure) |
| Structured time derivative | [time_integration.rs](../src/solver/incompressible/time_integration.rs) |
| GAMG/GCR | [linear/multigrid](../src/linear/multigrid) |
| Structured cylinder diagnostics | [diagnostics.rs](../src/solver/incompressible/diagnostics.rs), [force_balance.rs](../src/solver/incompressible/force_balance.rs) |
| Thermal physics | [physics.rs](../src/physics.rs), [thermal.rs](../src/solver/incompressible/thermal.rs) |
| Structured 3D backend | [incompressible_3d.rs](../src/solver/incompressible_3d.rs) |
| Saved Workbench configuration | [workbench/project.rs](../src/workbench/project.rs), [session.rs](../src/workbench/session.rs) |
| Plane sampling | [plane_monitor.rs](../src/workbench/plane_monitor.rs), [app/jobs.rs](../src/app/jobs.rs) |

## 2. Equations, units and geometry

For constant density and kinematic viscosity, the continuum model is

$$\nabla\cdot\mathbf U=0,$$
$$\frac{\partial\mathbf U}{\partial t}+\nabla\cdot(\mathbf U\otimes\mathbf U)
=-\nabla\pi+\nu\nabla^2\mathbf U+\mathbf b,\qquad \pi=p/\rho.$$

The steady Workbench formulation omits the physical-time derivative and has no
configured thermal/body-force equation. Generic momentum assembly supports a
source and backward-Euler storage, but the active Workbench SIMPLE caller passes
`old=None`, `time_step=None`, and a pressure-gradient source only. This generic
assembly API is not a complete unstructured transient solver.

Solver coordinates are metres; velocity is m/s, density kg/m³, and viscosity
m²/s. Native CAD uses millimetres and its geometry transfer converts to metres.
Workbench pressure is m²/s². Convert a Workbench pressure difference to pascals
with $\Delta p=\rho\Delta\pi$. An absolute pressure requires an independent
reference; a pressure gauge is not an absolute thermodynamic pressure.

For 2D meshes, face measures are edge lengths and cell measures are areas.
Volumetric flow, mass flow and integrated quantities are per unit out-of-plane
depth. A 3D face has an area vector and a cell has a volume.

Built-in structured cases use $Re=U_{ref}L_{ref}/\nu$ with case-specific scales
(e.g. cylinder diameter and the configured reference velocity). Workbench instead
accepts density and viscosity directly. Inlet mean, peak and free-stream speeds
must be distinguished when constructing a Reynolds number.

## 3. General-mesh finite-volume operators

Let $P$ own an internal face, $N$ be its neighbour, $\mathbf S_f$ point from owner
toward neighbour, and $\mathbf d=\mathbf x_N-\mathbf x_P$. Boundary area vectors
point outward from the fluid. The authoritative volume flux is

$$\Phi_f=\mathbf U_f\cdot\mathbf S_f.$$

Positive boundary flux leaves the fluid. Internal contributions are scattered
with opposite signs to owner and neighbour. Thus the cell divergence is

$$D_P=\frac{1}{V_P}\sum_{f\in P}s_{Pf}\Phi_f,$$

where $s_{Pf}=+1$ on owner faces and $-1$ on neighbour faces. The continuity
monitor is an unweighted cell RMS of this divergence, not an RMS of raw flux.

Face interpolation uses mesh geometry weights. Gauss gradients use

$$\nabla q_P=V_P^{-1}\sum_{f\in P}s_{Pf}q_f\mathbf S_f.$$

The least-squares implementation constructs mesh-bound reconstruction stencils,
including boundary information and treatment of reduced-dimensional stencils.
Invalid or unresolved geometry can return a numerical error; least squares is
not assumed well conditioned on every imported mesh. Pressure and its correction
use least-squares gradients. The limited convection correction also builds a
least-squares stencil for each transported component.

### 3.1 Momentum matrix and relaxation

For component $q$, the integrated equation is assembled as

$$\sum_f s_{Pf}\Phi_f q_f-\sum_f s_{Pf}\nu_f\nabla q_f\cdot\mathbf S_f
=-V_P\partial_q\pi.$$

The implicit base contains first-order upwind convection and two-point diffusion.
On an internal face the diffusion coefficient is

$$D_f=\nu_f\frac{\mathbf S_f\cdot\mathbf d}{\mathbf d\cdot\mathbf d}.$$

Negative/invalid geometric coefficients are rejected. Fixed-value boundaries add
normal diffusion and inflow source terms; zero-gradient boundaries extrapolate
owner values. The active solver uses constant viscosity, although generic
operators also accept cell diffusivity and linear/harmonic interpolation.

Equation under-relaxation replaces $a_P$ by $a_P/\alpha_u$ and adds

$$\frac{1-\alpha_u}{\alpha_u}a_Pq_P^k$$

to the RHS using the current nonlinear iterate, not a previous physical-time
field. Momentum components are solved with BiCGStab and symmetric
Gauss–Seidel preconditioning. A nonconverged linear solve is an error.

### 3.2 Deferred non-orthogonal diffusion

The low-level SIMPLE default and 2D Workbench use explicit non-orthogonal momentum diffusion with Gauss gradients. The 3D Workbench uses a conservative limited correction. The operator uses the projection split

$$\mathbf S_{\parallel}=\mathbf d\frac{\mathbf S_f\cdot\mathbf d}{|\mathbf d|^2},
\qquad \mathbf S_{\perp}=\mathbf S_f-\mathbf S_{\parallel},$$
$$J_f= D_f(q_N-q_P)+\nu_f\overline{\nabla q}_f\cdot\mathbf S_{\perp}.$$

The corrected-minus-uncorrected integrated diffusion is added to the RHS using
the current component field. Boundary correction follows the operator's
fixed-value/zero-gradient treatment. This is deferred across SIMPLE iterations;
it is not a separate configurable inner non-orthogonal pressure loop.

In 3D Workbench, the explicit face correction is bounded by the magnitude of the two-point face flux: $J_f=J_{orth}+\mathrm{clamp}(J_{corr},-|J_{orth}|,|J_{orth}|)$. The same signed contribution is added to the owner and subtracted from the neighbour, preserving conservation. This limiter sacrifices unrestricted non-orthogonal correction on poor meshes; mesh independence remains necessary. `SimpleOptions::momentum_diffusion` can change the low-level operator selection.
Selecting explicit least-squares diffusion additionally requires the appropriate
stencil; the active caller uses Gauss for this diffusion correction.

### 3.3 Limited linear upwind

Workbench defaults to `Upwind`. `LimitedLinear` is selectable in Solution and is
saved in the project. It retains the upwind matrix and adds a conservative
explicit reconstruction correction.

For each component, neighbour values and fixed boundary values define cell bounds
$q_P^{min}$ and $q_P^{max}$. A cell limiter $0\le\lambda_P\le1$ reduces the
least-squares slope so reconstructed face values remain within those bounds:

$$q_f^{rec}=q_{up}+\lambda_{up}\nabla q_{up}\cdot(\mathbf x_f-\mathbf x_{up}).$$

For an internal face, $\delta F_f=\Phi_f(q_f^{rec}-q_{up})$ is subtracted from the
owner RHS and added to the neighbour RHS. Physical boundary convection retains
its existing prescribed-value or extrapolation treatment; the correction is
applied only to internal faces.

This is a component-wise limited linear-upwind method, not the structured Van
Leer implementation. Limiting may reduce local order near extrema. It does not
prove global second-order accuracy, positivity of every derived field, or
unconditional nonlinear stability on arbitrary meshes.

## 4. Workbench SIMPLE pressure–velocity coupling

Each outer iteration:

1. Resolve dynamic boundary values from the current velocity and face flux.
2. Assemble and solve the three relaxed momentum equations using the current
   pressure, deferred diffusion and selected convection corrections.
3. Construct the pressure response, Rhie–Chow flux and pressure equation.
4. Solve the pressure correction, relax cell pressure, and correct velocity/flux.
5. Record monitors and optional plane samples; evaluate all stopping criteria.

Momentum equations are volume integrated. The cell response is therefore

$$r_{AU,P}=V_P/a_{P,relaxed},$$

not simply $1/a_P$. The current implementation obtains this scalar response from
the **x-component** momentum diagonal and applies it to the velocity correction;
it is not a full tensor/block pressure response.

Rhie–Chow interpolation combines interpolated velocity and a pressure difference
correction to avoid using uncorrected collocated cell velocities as face fluxes.
For pressure coupling the geometric two-point response is rescaled to the
over-relaxed coefficient

$$d_f=\overline{r_{AU}}_f\frac{|\mathbf S_f|^2}{\mathbf S_f\cdot\mathbf d}.$$

The reconstructed-gradient contribution is used in the flux predictor. In particular, interpolate the product $r_{AU}\nabla\pi$, rather than substituting the product of its separate interpolants; the response-gradient covariance is retained on nonuniform meshes. The 3D pressure source, Rhie–Chow reconstruction and cell velocity correction use the same Gauss gradient, with bounded face-distance interpolation and resolved fixed-value/owner-extrapolated pressure boundaries. The 2D coupling retains its least-squares reconstruction. Three-dimensional LSQ stencil validation uses positive-pivot Cholesky solves instead of a cancellation-prone cofactor inverse, with the existing boundary-normal closure for deficient stencils.
Pressure boundaries add a half-cell/owner-to-face pressure response using their
actual geometry. See `rhie_chow_flux_with_coefficients` and the boundary-aware
pressure coefficient and flux routines in `simple.rs` for the full operator.

With a positive-diagonal pressure matrix, the correction equation represents

$$A_\pi\pi'=-\sum_f s_{Pf}\Phi_f^*.$$

Updates are

$$\pi^{k+1}=\pi^k+\alpha_p\pi',\qquad
\mathbf U^{k+1}=\mathbf U^*-r_{AU}\nabla\pi',$$
$$\Phi_f^{k+1}=\Phi_f^*-d_f(\pi_N'-\pi_P').$$

A fixed-pressure boundary uses $\pi_b'=0$. A zero-gradient correction boundary
preserves prescribed flux. Without a pressure Dirichlet boundary, a reference
cell is constrained to zero correction with symmetric row/column elimination.
The stored face flux remains authoritative after correction: it is not replaced
by interpolating the corrected cell velocity again.

The pressure solver is PCG with zero-fill incomplete Cholesky (**IC0**), rebuilt
for the assembled correction matrix. This is not GAMG or GCR. Failure to build a
valid factorization propagates as an error; there is no silent Jacobi fallback.

### 4.1 Current Workbench defaults

| Setting | Default |
|---|---|
| Outer iterations | 1000 |
| Velocity / pressure relaxation | 0.7 / 0.3 |
| Continuity absolute tolerance | 1e-10 |
| Momentum/velocity coupling relative tolerance | 1e-6 |
| Momentum convection | Upwind |
| Momentum diffusion correction | Explicit non-orthogonal, Gauss gradient |
| Generic linear absolute / relative tolerance | 1e-10 / 1e-8 |
| Generic linear iteration limit | 1000 |
| Worker threads | 1; zero requests available logical CPUs |

These are constructor defaults, not recommended universal CFD settings. The GUI
exposes only a subset of low-level solver options. Project or API overrides must
be recorded when reporting results.

## 5. Workbench physical boundary conditions

All exterior patches must be assigned. Scalar primitives are fixed value and
zero normal gradient; the physical conditions below resolve to those primitives
and/or explicit velocity projections. All pressure values in this section are
kinematic gauge values.

| Condition / saved variant | Implemented behaviour |
|---|---|
| No-slip / `NoSlipWall` | Fixed zero velocity, zero-gradient pressure, impermeable flux |
| Moving wall / `MovingWall` | Fixed supplied vector; pressure zero-gradient. The vector is not automatically projected onto the wall: supply tangential velocity for an impermeable moving wall |
| Velocity inlet / `VelocityInlet` | Fixed uniform vector, flux from that vector and outward face vector; pressure zero-gradient |
| Volume/mass flow / `FlowRateInlet` | Prescribed total inward flow, distributed by face area with locally normal velocity; pressure zero-gradient |
| Parabolic inlet / `ParabolicInlet` | Face-centre evaluation of a planar, truncated parabola; pressure zero-gradient |
| Pressure outlet / `PressureOutlet` | Fixed pressure, zero-gradient velocity; final negative flux is rejected |
| Pressure outlet/backflow / `PressureOutletBackflow` | Fixed pressure on every face; specified incoming velocity for faces with negative current flux, extrapolated velocity otherwise; pressure correction determines the final flux |
| Slip / `Slip` | Zero normal flux and projected tangential owner velocity, with deferred component coupling |
| Symmetry / `Symmetry` | Same implemented constraints as laminar slip; does not include a turbulence symmetry model |
| Far-field / `FarField` | Supplied ambient velocity and zero-gradient pressure on inflow; extrapolated velocity and fixed reference pressure on outflow |

### 5.1 Total-flow inlet

With $A=\sum_{f\in patch}|\mathbf S_f|$ and positive inward volume flow $Q$,

$$\mathbf U_f=-\frac{Q}{A}\mathbf n_f,\qquad
\Phi_f=-Q\frac{|\mathbf S_f|}{A}.$$

For a mass specification, $Q=\dot m/\rho$. Rates must be finite and nonnegative;
patch area and the resulting speed must be valid. Curved patches use each face's
own normal. There is no prescribed tangential/swirl component or simultaneous
flow-rate-normalized profile in this condition.

### 5.2 Planar parabolic inlet

With transverse coordinate $s$ selected from X/Y/Z,

$$\mathbf U_f=\mathbf U_{peak}\max\left(0,1-
\left(\frac{s_f-s_c}{h}\right)^2\right).$$

The centre $s_c$ and half-width $h>0$ are in metres. The supplied vector is the
**peak**, not the mean. On a complete continuous planar channel section the mean
is $2/3$ of the peak; actual discrete flow is the sum of face-centre fluxes and
is not renormalized to enforce that analytic mean. This is not a radial pipe
profile. Select a coordinate transverse to the intended flow.

### 5.3 Open boundaries and limitations

Far-field face classification uses current corrected flux. At near-zero flux it
falls back to the ambient normal flux, then adjacent-cell normal velocity, to
avoid a trivial tangential inlet/outlet two-cycle. It is an incompressible local
switch, not a compressible characteristic/nonreflecting far-field condition.

Pressure outlet/backflow and Far-field are distinct. The former retains fixed
pressure during inflow; the latter switches to zero-gradient pressure on inflow.
The normal boundary flux can be changed by pressure correction in the former,
so the input backflow vector is not a guarantee of an exact final total flux.
Neither condition has a general stability proof for separated flow.

Periodic/cyclic boundaries, total-pressure inlets, time-dependent profile tables,
rotating walls, partial slip, general Robin scalar BCs and thermal wall BCs are
not connected to this Workbench flow solver. Slip on oblique faces uses deferred
scalar-component constraints, not an exact vector block boundary matrix.

## 6. Workbench residuals and stopping

Let $m_k$ be the maximum initial algebraic residual over the three momentum
solves in outer iteration $k$, and $M_k=\max_{j\le k}m_j$. The reported quantities
are

$$R_c=\sqrt{N_c^{-1}\sum_P D_P^2},\qquad
R_m=\frac{m_k}{\max(M_k,10^{-30})},$$
$$R_U=\sqrt{\frac{\sum_P|\mathbf U_P^{k+1}-\mathbf U_P^k|^2}
{\max(\sum_P|\mathbf U_P^{k+1}|^2,10^{-60})}}.$$

Convergence requires simultaneously:

- $R_c\le\varepsilon_c$;
- $R_m\le\varepsilon_{coupling}$ **or** $m_k\le\varepsilon_{momentum,abs}$;
- $R_U\le\varepsilon_{coupling}$.

Both linear solves must also succeed. Reaching the outer limit produces a
`MaxIterations` result, not a converged result. Invalid boundaries, nonfinite
fields and failed linear solves return errors.

Generic CSR PCG/BiCGStab use an absolute L2 residual and the additive threshold

$$\|b-Ax\|_2\le\varepsilon_{abs}
+\varepsilon_{rel}\|b-Ax_0\|_2.$$

The pressure history records a final **linear** residual. The momentum monitor
uses initial momentum residuals normalized across outer iterations; it is not a
final BiCGStab residual or a nondimensional physical equation error. Cell RMS
continuity is not globally normalized by inlet flow or domain scale.

Solution plots may overlay, logarithmically scale or normalize histories for
viewing. Plot normalization does not alter solver tolerances or stored residuals.
Initial continuity is recorded at iteration zero; component-solve histories begin
after the first completed outer iteration.

## 7. Structured 2D backend

### 7.1 Layout and spatial discretization

The grid has uniform $\Delta x=L/N_x$, $\Delta y=H/N_y$. Pressure/temperature are
cell-centred; $u$ and $v$ reside on normal faces. Diffusion uses Cartesian centred
second differences in unobstructed regions. Masked solids close staggered faces
and use reflected no-slip neighbour values; curved geometry remains a staircase,
not a body-fitted circle or a conservative cut-cell reconstruction.

Convection can be first-order upwind, central, or Van Leer TVD. Central is
unbounded. Van Leer is a deferred correction in coefficient-based coupling,
using $\psi(r)=2r/(1+r)$ for $r>0$ and zero otherwise, with upwind fallback when
the required stencil is unavailable. It is rejected for the explicit projection
and explicit SIMPLE-style paths. It is distinct from Workbench's cell slope
limiter.

### 7.2 Explicit projection and SIMPLE-style modes

Projection predicts velocity with forward-Euler convection/diffusion, solves

$$\nabla_h^2p^{n+1}=\rho\,\nabla_h\cdot\mathbf U^*/\Delta t,$$

and corrects $\mathbf U^{n+1}=\mathbf U^*-\Delta t\nabla_h p^{n+1}/\rho$.

The separate explicit SIMPLE-style mode includes current pressure in a relaxed
pseudo-time predictor and uses $\alpha_u\Delta t/\rho$ in its pressure correction.
Pressure is incremented by $\alpha_p p'$. Its pseudo-time is not physical time.

### 7.3 Implicit SIMPLE/SIMPLEC

The coefficient-based path assembles separate staggered momentum matrices,
solves them with right-preconditioned BiCGStab and symmetric Gauss–Seidel, then
solves pressure correction. SIMPLE uses inverse relaxed momentum diagonals.
SIMPLEC adjusts those responses using off-diagonal row sums and falls back to
SIMPLE where the consistent response is nonpositive.

The structured laminar deviatoric-stress correction is evaluated explicitly.
This should not be inferred from the scalar Laplacian implementation in the
separate Workbench momentum path.

### 7.4 Transient implicit PIMPLE-like algorithm

Physical-time history is frozen throughout all nonlinear outer correctors. Each
outer sweep reassembles momentum; each pressure corrector reconstructs a
pressure-free H/A predictor from the current velocity and fixed momentum
coefficients.

Importantly, the transient vector named `pressure_correction` in the code stores
**absolute dimensional pressure** during PIMPLE. It is initialized from the
preceding pressure iterate; after the solve, `p` is replaced with that vector.
In steady implicit SIMPLE it instead stores a pressure increment and pressure
is updated by $p\leftarrow p+\alpha_p p'$. These are not interchangeable algorithms.

Interior velocity corrections use staggered pressure differences and their
component momentum diagonals. The right pressure-outlet velocity is corrected
over the half-cell distance and retained after boundary reapplication.

The time derivative is Euler or variable-step BDF2. For $r=\Delta t_n/\Delta t_{n-1}$,

$$a_0=\frac{1+2r}{1+r},\quad a_1=1+r,\quad a_2=-\frac{r^2}{1+r},$$
$$\frac{\partial U}{\partial t}\approx
\frac{a_0 U^{n+1}-a_1 U^n-a_2 U^{n-1}}{\Delta t_n}.$$

One backward-Euler step initializes BDF2 history. The derivative rejects
$r\le0$ and $r\ge1+\sqrt2$. BDF2 configuration requires PIMPLE, flow-only physics
and at least two outer correctors. Increasing pressure correctors alone does
not increase temporal order or establish nonlinear convergence.

CLI controls include `--coupling pimple`, `--time-scheme euler|backward`,
`--outer-correctors`, `--pressure-correctors`, `--momentum-tol` and
`--pressure-rel-tol`; case construction supplies defaults. Earlier pressure
correctors allow the configured relative tolerance; the last uses zero relative
tolerance. Momentum uses an absolute tolerance in transient implicit coupling.

### 7.5 Pressure solver families

| Selection | Structured implementation |
|---|---|
| `pcg` | Jacobi-preconditioned pressure solve |
| `pcg-dic` | Incomplete-Cholesky/DIC-type preconditioning; supported matrix-free rectangular cases cache factors, unsupported cases fall back to Jacobi; CSR implicit matrices rebuild factors |
| `pcg-mg` | Structured aggregation/Galerkin V-cycle used as PCG preconditioner |
| `gamg` | Standalone agglomerated multigrid with optional restarted flexible GCR |
| `sor` | Sequential ordered SOR in explicit paths; implicit coefficient-based selection uses PCG/Jacobi |

Multigrid selections require coefficient-based coupling. The standalone GAMG
options include smoother, sweeps, aggregation levels, coarse solve, residual
norm, correction scaling and GCR restart length. A V-cycle used inside PCG is not
identical to a standalone GAMG/GCR solve.

`MultigridOptions` defaults currently include DIC–Gauss–Seidel smoothing,
normalized L1, pre/post/finest sweeps 0/2/1, merge-level count 2, GCR dimension 4,
and a PCG/DIC coarse solve. Defaults can be overridden; log the actual settings.
Numeric coarse coefficients are updated where the implicit implementation reuses
an existing hierarchy. Cached data are not proof of an allocation-free solve.

For GAMG normalized L1, the normalization is fixed from the initial iterate:

$$F=\max\left(\sum_i |(Ax_0)_i-s_i\bar x_0|+|b_i-s_i\bar x_0|,10^{-300}\right),
\quad s_i=\sum_j A_{ij},$$
$$R=\sum_i|b_i-(Ax)_i|/F.$$

GAMG stops at $R\le\max(\varepsilon_{abs},\varepsilon_{rel}R_0)$ and also supports
absolute L2. Generic CSR Krylov uses the additive L2 threshold
in Section 6; matrix-free structured Poisson monitoring uses maximum absolute
residual. These numbers cannot be compared as a single common residual.

### 7.6 Boundary conditions, stepping and stopping

Structured case/project overrides support prescribed velocity, symmetry and a
right-side pressure outlet. They do not expose Workbench's arbitrary patch BCs.
The project validator restricts structured pressure outlets to the right side.

Fixed time stepping is the default. Configuration validation permits adaptive
stepping only for the explicit transient `Projection` mode, not PIMPLE or any
SIMPLE mode. Steps are limited by CFL, explicit diffusion constraints, bounds and
growth. The final transient
step is shortened to land on `t_end`. There is no rejected-step rollback.

The recorded momentum stability indicators are

$$Co=\Delta t\left(\frac{\max|u|}{\Delta x}+\frac{\max|v|}{\Delta y}\right),
\qquad Fo_\nu=\nu\Delta t(\Delta x^{-2}+\Delta y^{-2}).$$

Adaptive projection uses $\max(\nu,\alpha)$ for its diffusion constraint when
thermal transport is active. It rejects a required stable step below `min_dt`
instead of silently violating the configured target. Implicit momentum does not
make an arbitrary time step accurate; these indicators do not establish a
universal stability bound for the full coupled algorithm.

The 2D steady stopping predicate requires nontransient coupling, the configured
minimum steps, maximum face-velocity change below `steady_tolerance`, maximum
absolute divergence below `10 * pressure_tolerance`, and, for coefficient-based
coupling, its momentum-equation residual below `steady_tolerance`. The
`CylinderRe100` case is explicitly excluded from steady termination. Transient
runs are governed by physical end time and iteration limits, not this predicate.

## 8. Temperature, buoyancy and structured 3D cavity

The structured 2D scalar model advances

$$\partial_t T+\nabla\cdot(\mathbf U T)=\alpha\nabla^2T+S_T,$$

with explicit upwind advection, centred diffusion, constant thermal diffusivity
and a source expressed in K/s. The configured inputs are $\alpha$ and $S_T$, not
separate conductivity, heat capacity and volumetric heat release. Available
thermal boundaries are adiabatic and fixed temperature, with reflected ghost
values for the latter. Solid-mask handling is not conjugate heat conduction.

Boussinesq adds the implemented acceleration

$$\mathbf b=\mathbf g\,\beta(T-T_{ref}).$$

Its sign follows the supplied gravity components. It requires the thermal model;
density otherwise remains constant. No heat-flux, convection/Robin or radiation
boundary is assembled. Workbench's general-mesh material only contains density
and viscosity and does not activate this scalar solver.

The separate structured 3D cavity uses explicit centred advective momentum,
Cartesian diffusion and SOR pressure projection. It is fixed-step, predominantly
sequential and limited to its cavity workflow. It does not mesh CAD solids or
provide feature parity with the general-mesh 3D steady Workbench solver.

## 9. Diagnostics, results and CAD plane monitors

### 9.1 Cylinder forces on the structured mask

The primary cylinder force integrates pressure and approximate viscous traction
on axis-aligned fluid/solid interfaces. Wall pressure is extrapolated as
$p_w=1.5p_P-0.5p_{away}$ when an away fluid sample exists, otherwise $p_w=p_P$.
Viscous estimates use cell-centred velocity over half a grid spacing; normal
traction includes the implemented $4/3$ factor. This remains a staircase wall
estimate, not exact integration on a smooth circular wall.

For force per unit span,

$$C_D=\frac{F_x}{\tfrac12\rho U_{ref}^2D},\qquad
C_L=\frac{F_y}{\tfrac12\rho U_{ref}^2D}.$$

Pressure and viscous contributions are stored separately. Independent transient
control-volume force diagnostics include momentum storage, convection and stress
on two rectangular contours of half-width 1.25D and 1.5D. They return unavailable
when their geometric/physical requirements are not met; they do not replace the
primary wall-force columns. Their storage derivative follows Euler/BDF2 history.

Lift maximum, mean, half peak-to-peak amplitude and RMS are different quantities.
Strouhal is $St=fD/U_{ref}$ and requires a documented physical-time window and
frequency estimator. An outer-iteration frequency from steady Workbench samples
is not a physical shedding frequency. This document supplies no benchmark value
or automatic validation claim for these quantities.

### 9.2 Output and pressure interpretation

Structured `history.csv` currently has the following header:

```text
step,time,pressure_residual,pressure_iterations,max_divergence,velocity_change,momentum_equation_residual,max_speed,cd,cl,reattachment_x_over_h,momentum_cfl,viscous_diffusion_number,time_step,cd_pressure,cd_viscous,cl_pressure,cl_viscous,control_volume_cd,control_volume_cl,control_volume_outer_cd,control_volume_outer_cl
```

An unavailable optional diagnostic is blank. Structured outputs include final
CSV/VTK, selected PPM frames and optional verification field snapshots.
Workbench persists its own run reports and VTK results. Its adjacent-cell patch
pressure averages and flux reports are not a validated wall-traction calculation.
Do not interpret a generic result panel as proof that cylinder forces were
computed by the structured diagnostic above.

### 9.3 Plane sampling

CAD origin, construction and sketch planes can define a Workbench monitor.
Enabled planes are intersected with the 3D cell mesh once before the solve.
The intersection algorithm assumes convex cells and constructs a sorted section
polygon. A coincident internal face is sampled once using its owner, rather than
plotting two overlapping samples.

Samples contain the intersected cell's **cell-centred** velocity and kinematic
pressure. The plotted location is the mean of polygon vertices; there is no
higher-order point interpolation or exact area-centroid integration. Results → CAD section displays filled cell sections of speed, kinematic pressure,
global velocity components or normal velocity, with a labelled colour bar and
automatic/manual range. Optional arrows show
in-plane velocity; symbols distinguish normal flow. The full vector is retained
in CSV even when the display subsamples arrows.

Every N completed SIMPLE iterations, and at the final successful iteration if
not already scheduled, the job writes CSV and SVG under
`monitors/solve-<id>/<encoded-plane-id>/`. Plane metadata is in `plane.json`.
Display settings are saved per plane; scheduled SVG uses the field/style frozen
at solve start, while CSV always contains every listed field. Manual SVG/PNG
exports use the current display. CSV fields are:

```text
iteration,cell,x_m,y_m,z_m,ux_m_s,uy_m_s,uz_m_s,u_plane_m_s,v_plane_m_s,normal_velocity_m_s,speed_m_s,kinematic_pressure_m2_s2
```

These are steady iteration snapshots, not physical-time animation frames.
Recording adds intersection/sampling, rendering and I/O cost; disable it or use
identical settings during performance comparisons. Scheduled sample errors can
fail a solve through the observer; a final-only export error is logged without
discarding an otherwise valid completed solution.

## 10. Parallelism, verification and known limits

Workbench creates a private Rayon pool; zero threads selects available logical
CPUs. Momentum components may execute concurrently. Its pressure solve is
currently serial. Requested threads do not imply equal utilization in every
stage. Gmsh meshing has a separate thread setting and algorithm-dependent
parallelism; it is not part of the CFD iteration count.

Structured 2D uses shared-memory parallel loops, operator/vector kernels and
reductions where implemented. Ordered SOR remains serial. Floating-point
reduction order can change last-bit results. The structured solver reuses several
assembly/Krylov workspaces and multigrid data; Workbench still performs repeated
assembly, factorization and some gradient-stencil allocation.

The recent Workbench flow-rate/backflow/profile BCs, limited convection,
non-orthogonal momentum correction and IC0 integration have compilation/build
checks. They have not received a new complete mesh/time-refinement or cylinder
validation campaign in this documentation update. Existing sketch compatibility
tests concern CAD geometry and do not establish CFD accuracy.

For reproducible verification record:

- repository commit **and uncommitted changes**, CAD submodule commit, executable
  hash, build profile, compiler and dependency lockfile;
- mesh file/hash, cell/face counts, dimensions, quality, geometry and units;
- material, every patch condition, initial fields and pressure reference;
- backend, convection, time scheme, correctors, relaxation, linear solver,
  residual definitions and all stopping criteria;
- actual completed iterations/time, final residuals, flux imbalance and field
  outputs; failed and maximum-iteration runs separately;
- mesh and time-step refinement, benchmark definitions and measurement window;
- thread count, CPU affinity, repetitions, load, wall/CPU time and peak RSS,
  including whether monitoring/output/setup are inside the timed region.

Equal tolerance strings, similar run duration, fewer pressure iterations, or the
same geometry name do not establish equivalent numerical work. A small algebraic
residual does not establish small discretization or model error. No relative
speed, memory or accuracy advantage is asserted by this document.
