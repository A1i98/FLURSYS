use super::*;

pub(super) fn apply_velocity_boundaries(
    case: &Case,
    boundaries: &SolverBoundaryOverrides,
    grid: &UniformGrid2D,
    solid: &Mask2D,
    time: f64,
    u: &mut Field2D,
    v: &mut Field2D,
) {
    for j in 0..grid.ny {
        let y = grid.u_face_y(j);
        match boundaries.kind(case, Side::Left) {
            BoundaryKind::Velocity => {
                u[(0, j)] = boundaries.velocity(case, Side::Left, 0.0, y, time).0
            }
            BoundaryKind::Symmetry => u[(0, j)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => u[(0, j)] = u[(1, j)],
        }
        match boundaries.kind(case, Side::Right) {
            BoundaryKind::Velocity => {
                u[(grid.nx, j)] = boundaries
                    .velocity(case, Side::Right, grid.length, y, time)
                    .0
            }
            BoundaryKind::Symmetry => u[(grid.nx, j)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => u[(grid.nx, j)] = u[(grid.nx - 1, j)],
        }
    }

    for i in 0..grid.nx {
        let x = grid.v_face_x(i);
        match boundaries.kind(case, Side::Bottom) {
            BoundaryKind::Velocity => {
                v[(i, 0)] = boundaries.velocity(case, Side::Bottom, x, 0.0, time).1
            }
            BoundaryKind::Symmetry => v[(i, 0)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => v[(i, 0)] = v[(i, 1)],
        }
        match boundaries.kind(case, Side::Top) {
            BoundaryKind::Velocity => {
                v[(i, grid.ny)] = boundaries.velocity(case, Side::Top, x, grid.height, time).1
            }
            BoundaryKind::Symmetry => v[(i, grid.ny)] = 0.0,
            BoundaryKind::PressureOutlet { .. } => v[(i, grid.ny)] = v[(i, grid.ny - 1)],
        }
    }

    // Tangential velocity conditions are represented through ghost values in the
    // momentum stencil. Here only blocked faces are explicitly zeroed.
    for j in 0..grid.ny {
        for i in 1..grid.nx {
            if !u_face_open(solid, grid.nx, i, j) {
                u[(i, j)] = 0.0;
            }
        }
    }
    for j in 1..grid.ny {
        for i in 0..grid.nx {
            if !v_face_open(solid, grid.ny, i, j) {
                v[(i, j)] = 0.0;
            }
        }
    }
}

pub(super) fn u_face_open(solid: &Mask2D, nx: usize, i: usize, j: usize) -> bool {
    if i == 0 {
        !solid[(0, j)]
    } else if i == nx {
        !solid[(nx - 1, j)]
    } else {
        !solid[(i - 1, j)] && !solid[(i, j)]
    }
}

pub(super) fn v_face_open(solid: &Mask2D, ny: usize, i: usize, j: usize) -> bool {
    if j == 0 {
        !solid[(i, 0)]
    } else if j == ny {
        !solid[(i, ny - 1)]
    } else {
        !solid[(i, j - 1)] && !solid[(i, j)]
    }
}

pub(super) fn pressure_outlet_cell(
    case: &Case,
    boundaries: &SolverBoundaryOverrides,
    solid: &Mask2D,
    nx: usize,
    i: usize,
    j: usize,
) -> bool {
    i + 1 == nx
        && !solid[(i, j)]
        && matches!(
            boundaries.kind(case, Side::Right),
            BoundaryKind::PressureOutlet { .. }
        )
}
