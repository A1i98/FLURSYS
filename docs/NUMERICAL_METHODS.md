# Numerical Methods

Implementation audit: 2026-10-21, including the current working-tree changes.
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
| Coupling | SIMPLE; converged implicit-Euler time levels (experimental) | Projection, explicit SIMPLE-style, implicit SIMPLE, SIMPLEC, PIMPLE-like | Transient projection |
| Physical-time integration | Implicit Euler on fixed meshes (experimental) | Forward Euler for explicit predictor; Euler/BDF2 for implicit PIMPLE | Forward Euler |
| Momentum convection | Upwind or limited linear upwind | Upwind, central, Van Leer deferred correction in supported implicit modes | Centred advective predictor |
| Pressure | PCG + IC0 | Matrix-free PCG/SOR or CSR PCG, DIC/IC0, structured MG, GAMG | SOR |
| Thermal/Boussinesq | Not connected | Optional constant-property temperature/Boussinesq | Not connected |
| Pressure field | Kinematic gauge pressure, p/rho | Dimensional pressure, p | Dimensional pressure, p |

The structured PIMPLE, BDF2, GAMG and thermal implementations are **not** selected
by choosing a 3D mesh or increasing the iteration limit in Workbench. Workbench
iterations are nonlinear steady iterations in Steady mode; Transient mode advances
physical time only after the coupled correctors converge.

The workbench has experimental wall-resolved k-omega/SST and algebraic
Smagorinsky/WALE closures, documented in [TRANSIENT_TURBULENCE.md](TRANSIENT_TURBULENCE.md).
These additions have not been benchmark validated. Standard k-epsilon is now
also available for free-shear cases or an optional experimental smooth equilibrium wall treatment. A separate experimental compressible
CLI is described below; it is not connected to GUI SIMPLE jobs. VOF, reacting
flow, species transport, MRF and moving-mesh flow remain unconnected.

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

The steady Workbench branch omits the physical-time derivative. In transient
mode, the old physical velocity and conservative face flux are frozen during
all nonlinear correctors of each implicit Euler step. The generic momentum
assembler receives the old component field and dt; its relaxation field is the
current nonlinear iterate. Molecular plus turbulent face viscosity and the
explicit remainder of the deviatoric stress are supplied when a closure is active.
See [TRANSIENT_TURBULENCE.md](TRANSIENT_TURBULENCE.md) for the exact scope.

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

### 2.1 Mesh generation and visual inspection

The current Workbench implements global/scoped sizing, Distance/Threshold
refinement, 3D Box volume refinement and 2D inflation. Geometry scopes may use
stable named selections or direct boundary edge/surface selections. Controls are
stored in `mesh_recipe`, including algorithm/optimization choices. Isotropic
explicit sizing outside global bounds is rejected instead of silently clamped.
Distance and Box fields are combined by the minimum requested size.

2D inflation requests a geometric layer stack $h_k=h_1r^{k-1}$ and
$H=\sum_{k=1}^{N}h_k$. For total-thickness input, $h_1$ is derived from this
sum. Equal growth ($r=1$) is handled by summation. Compatible disjoint scopes are
merged into one Gmsh BoundaryLayer field. The requested count is not proof of
N generated layers near corners/collisions. Arbitrary-CAD 3D prism inflation is
not implemented; isotropic refinement is not a substitute for prism layers.

Mesh quality visualization reports global cell statistics and ten equal-width
histogram bins, with non-finite values excluded from finite statistics and
reported separately. Cutaway retains whole cells by centre position and exposes
their mesh faces; it is not a geometric plane intersection. Both cutaway and
transparent wireframe change display only, not the finite-volume operators,
physical boundary groups or mesh used by the solver. See [Meshing](MESHING.md)
for backend controls and [Workbench](WORKBENCH.md) for UI actions.

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
not assumed well conditioned on every imported mesh. Pressure and its correction use least-squares gradients in 2D Workbench;
3D Workbench uses the Gauss coupling gradient described in section 4. The limited convection correction also builds a
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


## Experimental unstructured physical time and turbulence (2026-10-21)

Implicit Euler physical-time integration, wall-resolved k-omega/SST and algebraic
Smagorinsky/WALE closures are now connected to the unstructured momentum path.
See [TRANSIENT_TURBULENCE.md](TRANSIENT_TURBULENCE.md) for equations, constants,
boundary semantics, restart behavior and the explicit validation limitations.
This implementation has not been benchmark validated.

## Conservative compressible equations and k-epsilon extension

The experimental `compressible` module is independent of the GUI SIMPLE jobs.
It is not compressible RANS, PIMPLE, or a replacement for every OpenFOAM solver.
The CLI entry is `flursys-compressible CASE.json OUTPUT_DIRECTORY`. The example
`examples/compressible/ideal_gas_case.json` is an unexecuted input template;
provide a Gmsh mesh and edit all patch names and physical values explicitly.

### Compressible governing equations

For a fixed mesh the conserved variables are `Q = (rho, rho U, rho E)`, with
`E = cv T + |U|²/2`, `p = (gamma-1)(rho E-rho |U|²/2) = rho R T`,
`cv = R/(gamma-1)`, `cp = gamma cv`. Pressure is absolute **Pa**, not the
incompressible solver's kinematic gauge pressure. Temperature is absolute K.
The integrated equations are

- `d(rho V)/dt + sum(rho Un A) = 0`;
- `d(rho U V)/dt + sum((rho U Un + p n - tau.n) A) = 0`;
- `d(rho E V)/dt + sum(((rho E+p)Un - (tau.n).U - lambda grad(T).n) A) = 0`.

`tau = mu(grad U + grad U^T - 2/3 div(U) I)` and `lambda = mu cp / Pr`.
Viscosity is either disabled (Euler), a constant, or Sutherland's law
`mu = mu_ref (T/T_ref)^(3/2) (T_ref+S)/(T+S)`. This is a calorically perfect,
single-species ideal gas with laminar molecular stress and Fourier conduction;
there is no compressible turbulence closure, chemistry or real-gas EOS.

### Spatial and temporal discretisation

Convective face states are piecewise constant: **first-order spatial accuracy**.
Selectable numerical fluxes are local Lax-Friedrichs/Rusanov, a two-sided
local-speed central-upwind flux, and HLLC. HLLC falls back locally to Rusanov if
its computed star state is inadmissible; accepted-stage fallback evaluations
are counted. The central-upwind option uses local wave-speed weights like the
reviewed `shockFluid` flux path, but does not reproduce its reconstruction,
implicit treatment, mesh-motion or complete boundary framework.

Gradients for viscous fluxes use Gauss reconstruction and a face-normal
consistency correction. Interior diffusive fluxes are evaluated once and applied
with opposite signs to owner/neighbour. Arbitrary highly skewed mesh accuracy
and stability are not established. The explicit step restriction includes both
acoustic face speeds and an estimate based on viscous/thermal diffusivity.
Temporal integration is SSP-RK2; both stages obey the configured CFL restriction
(allowed target `0 < CFL <= 0.5`). Candidates with nonpositive density/internal
energy are rejected and retried with halved dt, without clipping conserved
fields. At most 20 retries are allowed; a remaining invalid candidate, dt below
the minimum, or a boundary/evaluation error returns an error. Boundary errors
are not silently treated as successful time-step retries.

Internal face fluxes cancel in the global balance. The report stores initial
and final integrated Q, RK2-integrated outward boundary fluxes, and the defect
`Q_final - Q_initial + integral(boundary_outflux dt)` in the order mass, x/y/z
momentum, total energy. In 2D these quantities are per unit depth. Reaching the
step limit before the requested physical end time is explicitly incomplete;
the CLI saves that partial result and returns failure. These fields are runtime
reports, not evidence of accuracy or benchmark performance.

### Compressible boundary scope and output

- Slip walls reflect normal velocity in the inviscid ghost state and enforce
  zero tangential viscous traction and heat flux.
- Stationary no-slip walls use the same impermeable inviscid treatment and a
  zero-velocity viscous face value. Temperature is optional: missing/null means
  adiabatic; a positive Kelvin value imposes isothermal conduction. Inviscid
  no-slip configurations are rejected.
- Far-field is a prescribed exterior Riemann state, not a nonreflecting or
  fully characteristic boundary implementation.
- Supersonic inlet requires inward normal Mach greater than 1 on every inlet
  face. Transmissive uses the owner state; it is not guaranteed nonreflecting.
- Pressure outlet is a basic subsonic pressure/isentrope ghost prescription;
  pressure is extrapolated for supersonic outflow. Backflow without a supplied
  incoming thermodynamic state is rejected. General characteristic outlets
  remain future work.

Uniform initialisation and optional axis-aligned initial regions are supported;
later regions override earlier ones. There is no compressible restart or GUI
job/result integration yet. `solution.vtu` contains density, velocity, absolute
pressure, temperature and Mach for external viewers such as ParaView;
`report.json` contains settings, timing and conservation diagnostics. Neither
export has been validated interactively in a VTK viewer in this change.

### Standard constant-density k-epsilon

The existing incompressible RANS path now transports epsilon as a distinct
optional field in addition to k; it is not stored under the omega name.
`nu_t = 0.09 k²/epsilon`, `Dk = nu + nu_t`, `Depsilon = nu + nu_t/1.3`,
`Pk = nu_t (2 S:S - 2/3 div(U)²)`. Standard coefficients are
`Cmu=0.09`, `C1=1.44`, `C2=1.92`, `sigma_k=1`, `sigma_epsilon=1.3`, `C3=0`.
Transport includes epsilon production `C1 Pk epsilon/k`, implicit epsilon
loss `C2 epsilon²/k`, k loss `epsilon`, and sign-split dilatation terms. Epsilon
is solved before k, and its updated bounded value is used in k destruction.
The dissipation floor also limits turbulent viscosity to `1e5 nu`; this
numerical bound is not a physical wall model.
The same ordering correction has been applied to k-omega/SST: the k destruction
and SST k-production limit now use newly solved bounded omega.

Incoming/backflow k and epsilon use global or patch-specific values. Outgoing
scalars use zero gradient. Solid walls require the explicit smooth equilibrium
wall treatment described below; without it, k-epsilon wall cases are rejected.
This is not a validated cylinder-wall model. Wall-resolved SST remains available.
Epsilon is included in project settings and full-state checkpoints.

### Sources inspected and validation status

The local source files inspected include OpenFOAM-dev's
`RAS/kEpsilon/kEpsilon.C`, `Base/kOmegaSST/kOmegaSSTBase.C`,
`applications/modules/shockFluid/fluxPredictor.C`,
`shockFluid/thermophysicalPredictor.C`, and `perfectGas/perfectGasI.H`.
The Rust implementation is independent; it is not a literal C++ source import.
The reference tree contains GPLv3-or-later terms and the host currently declares
MIT; no repository license change is made by this work.

Published method references:

- Launder and Spalding (1974), The numerical computation of turbulent flows,
  https://doi.org/10.1016/0045-7825(74)90029-2 .
- Toro, Spruce and Speares (1994), Restoration of the contact surface in the
  HLL-Riemann solver, https://doi.org/10.1007/BF01414629 .
- Kurganov, Noelle and Petrova (2001), Semidiscrete central-upwind schemes for
  hyperbolic conservation laws and Hamilton-Jacobi equations,
  https://doi.org/10.1137/S1064827500373413 .
- Kurganov and Tadmor (2000), New high-resolution central schemes for nonlinear
  conservation laws and convection-diffusion equations,
  https://doi.org/10.1006/jcph.2000.6459 .

Compilation is not validation. No new numerical tests, mesh runs, shock-tube
comparisons, compressible viscous benchmarks or RANS reference comparisons were
run in this change. Required follow-up includes constant-state preservation,
Sod/Lax/contact/shock reflection, viscous Couette/heat conduction, conservation
and grid/time studies; wall-function and compressible-RANS work is separate.

## Workbench temperature and Boussinesq coupling

The unstructured incompressible path optionally solves constant-property temperature:
`dT/dt + div(U T) = div((k/(rho cp) + nut/Prt) grad(T)) + Q/(rho cp)`.
Convection is first-order upwind, diffusion uses harmonic face diffusivity and
limited non-orthogonal correction. Transient transport uses implicit Euler and
the previous physical-step temperature; SIMPLE corrections share that old state.
Temperature has an independent relative-update convergence requirement. Positive
Kelvin values are required; invalid temperatures fail without clipping.

Thermal patch conditions are explicit: adiabatic, prescribed temperature,
inlet/outlet temperature (including backflow), inward heat flux in W/m², and
Robin convection using the wall-to-cell conduction resistance plus `1/h`.
Boussinesq acceleration is `g [1 - beta (T-Tref)]`; pressure remains full
kinematic gauge pressure, not hydrostatically reduced pressure. The default
small-density-variation guard is `abs(beta (T-Tref)) <= 0.1`. This is a
single-phase constant-property model, not compressible energy or conjugate heat
transfer. Temperature is saved in complete restart states and final VTK/CSV
results. Plane-monitor recordings still contain velocity and pressure only.
The current legacy 3D result exporter supports tetrahedral cells only.

## Material data and two-phase preparation

`src/materials.rs` records coefficient provenance at OpenFOAM-dev commit
`421b7a1693061ead99502f0e607122e0e57c0513`: H2O and C2H5OH liquid-property
correlations and the blockedChannel tutorial's idealized air properties.
Catalog evaluation is deliberately restricted to 280–330 K and 95–105 kPa;
these are application limits, not a validated full correlation range. Liquid
pressure dependence is neglected. Properties are evaluated once and frozen
throughout the solve. The selected evaluation state and source are persisted.
Air is a tutorial gas model, not a certified experimental database; water is
not an IAPWS implementation. Liquid/vapour surface tension must not be treated
as the interfacial tension of an arbitrary fluid pair.

`src/multiphase.rs` supplies validated two-fluid definitions and algebraic
mixture properties: volume-averaged density, viscosity and volumetric heat
capacity; arithmetic or harmonic conductivity. It does not implement phase
fraction transport, VOF reconstruction, surface-tension forcing, phase change,
or a coupled multiphase solver. Numerical benchmark validation of the new
thermal and material paths remains outstanding; compilation is not evidence
of solution accuracy.

### Patch-specific incoming RANS quantities

`TurbulenceOptions.inlets` optionally overrides global incoming values for each
named open boundary. Direct data specify positive k, omega and epsilon (the
active RANS model uses its relevant pair). Intensity/length inputs use
`k = 1.5 (I |U_boundary|)^2`, `omega = sqrt(k)/(Cmu^0.25 L)` and
`epsilon = Cmu^0.75 k^1.5/L`, with `Cmu=0.09`, intensity as a fraction and
length in metres. Resolved prescribed velocity is used where available;
otherwise owner tangential velocity plus flux-derived normal velocity is used.
Zero-speed intensity conversion fails explicitly instead of inventing a floor.
Incoming faces use fixed turbulence data, including pressure-outlet backflow;
outgoing faces retain zero-gradient transport. RANS incoming turbulent viscosity
and isotropic k stress use the corresponding incoming data. Wall and symmetry
patches reject these open-boundary overrides. Renaming/deleting a named boundary
also updates/removes its override. Global values remain the fallback. These
settings do not themselves select a wall model or implement synthetic LES inflow.
Numerical validation of this extension remains outstanding.

### Fixed-step backward time integration (BDF2)

Workbench offers `Backward { dt, steps }` / `Transient · BDF2 fixed Δt`.
After one converged Euler startup step, fixed-mesh time derivatives are
`(3 q^(n+1) - 4 q^n + q^(n-1))/(2 dt)`. Assembly uses effective Euler storage
`dt_eff=2 dt/3`, `q_old_eff=(4 q^n-q^(n-1))/3`. The same coefficients are used
for momentum, conservative face-flux temporal correction, temperature and RANS
k/omega/epsilon storage. Only accepted physical steps rotate history. Reported
time and Courant use the physical dt, not dt_eff. Nonlinear convergence remains
mandatory at every step. This remains SIMPLE-based time integration; it does
not implement PISO/PIMPLE. There is no adaptive BDF2 option.

Two-level history is retained during one solve invocation, but is not yet
persisted in restart files. Restart/continue therefore uses one explicitly
announced Euler startup step before returning to BDF2. Repeated single-step
invocations consequently remain Euler. BDF2 is not positivity preserving;
existing temperature failure checks and turbulence bounds remain in force and
can limit effective accuracy. Temporal order and conservation have not yet
been demonstrated with numerical benchmarks. The implementation was compared
against the fixed-step coefficients of the local backwardDdtScheme source.

### Incompressible total-pressure opening

`TotalPressure { pressure }` stores kinematic gauge total pressure (m²/s²).
For negative outward flux, `p/rho = p0/rho - |U_owner|²/2`; for outflow the
specified value is static pressure. Velocity uses natural zero-gradient
extrapolation, with no imposed inflow direction. Pressure is recomputed using
lagged velocity/flux on each SIMPLE iteration and pressure correction is zero
on the opening. This is a simple incompressible Bernoulli opening, not a
compressible characteristic boundary or a non-reflecting wave outlet.
Inflow temperature and turbulence must be supplied independently. Project
serialization and Setup expose this condition. It has not been numerically
validated against a pressure-driven-flow reference.

### Experimental smooth equilibrium k-epsilon wall treatment

Select `EquilibriumKEpsilon` explicitly; the default remains `Resolved` and
rejects k-epsilon solid-wall cases. The option is restricted to k-epsilon.
The implementation follows the scientific relations inspected in local
`nutkWallFunction` and `epsilonWallFunction` sources, with `Cmu=0.09`,
`kappa=0.41`, `E=9.8`. Let `y` be the projected face-to-cell distance,
`ykplus=Cmu^0.25 sqrt(k) y/nu`; the crossover solves
`ykplus=ln(E ykplus)/kappa`. Below crossover wall nut is zero,
`epsilon=2 nu k/y²` and wall production is zero. Above crossover:

- `nut_wall = nu [kappa ykplus / ln(E ykplus) - 1]`;
- `epsilon_wall_cell = Cmu^0.75 k^1.5/(kappa y)`;
- `Pk_wall_cell = (nu+nut_wall) |U_t-Uwall_t|/y × Cmu^0.25 sqrt(k)/(kappa y)`.

Wall-adjacent epsilon rows are constrained in the transport matrix; their
columns are eliminated from other rows. Wall-cell production overrides bulk
production, k has zero-gradient wall data, and momentum diffusion uses
`nu+nut_wall`. Corner-cell constraints and production use face-area averages;
this is a deliberate approximation and does not reproduce every reference
implementation's corner blending. Dissipation constraints are not overwritten
by the bulk numerical viscosity cap. Non-finite data fail explicitly.

Final and completed-step force reports use matching turbulence fields to
include modeled wall viscosity in first-cell shear estimates. Shear-based y+
is distinct from the k-based y+ used by the wall model. Incomplete live U/p
callbacks cannot reconstruct wall-function shear and suppress those reports
rather than substituting laminar shear. A mesh-review message is issued for
completed-step wall reports outside a nominal shear-y+ range of 30–300; this
is diagnostic, not proof of validity or a universal admissibility threshold.

No roughness correction, non-equilibrium/separation treatment, SST/omega wall
function or thermal wall function has been implemented. Existing turbulent
heat diffusivity is unchanged and is not a validated thermal wall treatment.
Flat-plate/channel friction, corner behavior and mesh/y+ sensitivity have not
been numerically validated. Do not infer accuracy from a successful build.

### Physical-time tabulated boundary data

`IncompressibleSolverOptions.boundary_profiles` maps each patch name to one
`BoundaryProfile` with a quantity (velocity, inward flow rate or kinematic
pressure) and 2–10000 `(time, Vec3 value)` samples. Times must be finite,
nonnegative and strictly increasing. Scalars use x with y=z=0; inward flow
rates cannot be negative. Linear interpolation is used between samples and
endpoint values are held outside the table. No periodic extrapolation or
spatial mapping is implemented. The table semantics were compared with the
local Function1 Table interpolation/bounds design; this is an independent
restricted Rust representation, not a parser for reference dictionaries.

Velocity tables support velocity inlets, peak velocities of existing parabolic
inlets, far-field ambient velocity and explicitly prescribed outlet backflow
velocity. Moving walls are intentionally not included. Flow-rate tables retain
the assigned inlet's volumetric/mass-flow units. Pressure tables retain static
or total kinematic-pressure semantics, including far-field/backflow openings.
They do not override physical boundary type or introduce new compressible BCs.

Tables require physical transient integration (Euler, adaptive Euler or fixed
BDF2). They are evaluated at t_(n+1), held fixed during nonlinear correctors,
and reevaluated on a rejected adaptive step at its new candidate time. Initial
boundaries use the restart time, or zero for fresh runs. BDF2 startup captures
initial face flux at that initial time before applying future tables. Tables
are included in project/run solver settings and checkpoint context; restart
uses absolute physical time rather than restarting the table at zero. Naming
changes and boundary deletion update/remove the associated table. Large time
steps can underresolve ramps or table breakpoints; no automatic knot-aligned
stepping or temporal-accuracy claim is provided. Numerical validation remains
outstanding.
