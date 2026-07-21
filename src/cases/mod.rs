mod backward_step;
mod cavity;
mod cylinder;

pub use backward_step::BackwardStepCase;
pub use cavity::CavityCase;
pub use cylinder::CylinderCase;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CaseKind {
    LidDrivenCavity,
    CylinderRe100,
    BackwardFacingStep,
}

impl CaseKind {
    pub fn slug(self) -> &'static str {
        match self {
            Self::LidDrivenCavity => "cavity",
            Self::CylinderRe100 => "cylinder",
            Self::BackwardFacingStep => "backward-step",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
    Bottom,
    Top,
}

#[derive(Clone, Copy, Debug)]
pub enum BoundaryKind {
    /// Prescribed velocity. On a solid wall this is the wall velocity.
    Velocity,
    /// Zero normal velocity and zero normal gradient of tangential velocity.
    Symmetry,
    /// Fixed pressure with zero normal velocity gradients.
    PressureOutlet { pressure: f64 },
}

#[derive(Clone, Debug)]
pub enum Case {
    LidDrivenCavity(CavityCase),
    CylinderRe100(CylinderCase),
    BackwardFacingStep(BackwardStepCase),
}

impl Case {
    pub fn kind(&self) -> CaseKind {
        match self {
            Self::LidDrivenCavity(_) => CaseKind::LidDrivenCavity,
            Self::CylinderRe100(_) => CaseKind::CylinderRe100,
            Self::BackwardFacingStep(_) => CaseKind::BackwardFacingStep,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::LidDrivenCavity(_) => "Lid-Driven Cavity",
            Self::CylinderRe100(_) => "Cylinder Flow Re=100",
            Self::BackwardFacingStep(_) => "Backward-Facing Step Flow",
        }
    }

    pub fn density(&self) -> f64 {
        match self {
            Self::LidDrivenCavity(c) => c.rho,
            Self::CylinderRe100(c) => c.rho,
            Self::BackwardFacingStep(c) => c.rho,
        }
    }

    pub fn kinematic_viscosity(&self) -> f64 {
        match self {
            Self::LidDrivenCavity(c) => c.nu,
            Self::CylinderRe100(c) => c.nu,
            Self::BackwardFacingStep(c) => c.nu,
        }
    }

    pub fn reference_velocity(&self) -> f64 {
        match self {
            Self::LidDrivenCavity(c) => c.lid_velocity,
            Self::CylinderRe100(c) => c.u_inf,
            Self::BackwardFacingStep(c) => c.u_mean,
        }
    }

    pub fn reference_length(&self) -> f64 {
        match self {
            Self::LidDrivenCavity(c) => c.length,
            Self::CylinderRe100(c) => c.diameter,
            Self::BackwardFacingStep(c) => c.step_height,
        }
    }

    pub fn domain(&self) -> (f64, f64) {
        match self {
            Self::LidDrivenCavity(c) => (c.length, c.height),
            Self::CylinderRe100(c) => (c.length, c.height),
            Self::BackwardFacingStep(c) => (c.length, c.height),
        }
    }

    pub fn boundary_kind(&self, side: Side) -> BoundaryKind {
        match self {
            Self::LidDrivenCavity(_) => BoundaryKind::Velocity,
            Self::CylinderRe100(_) => match side {
                Side::Left => BoundaryKind::Velocity,
                Side::Right => BoundaryKind::PressureOutlet { pressure: 0.0 },
                Side::Bottom | Side::Top => BoundaryKind::Symmetry,
            },
            Self::BackwardFacingStep(_) => match side {
                Side::Left => BoundaryKind::Velocity,
                Side::Right => BoundaryKind::PressureOutlet { pressure: 0.0 },
                Side::Bottom | Side::Top => BoundaryKind::Velocity,
            },
        }
    }

    /// Prescribed velocity at a physical domain boundary.
    pub fn boundary_velocity(&self, side: Side, _x: f64, y: f64, _time: f64) -> (f64, f64) {
        match self {
            Self::LidDrivenCavity(c) => match side {
                Side::Top => (c.lid_velocity, 0.0),
                Side::Left | Side::Right | Side::Bottom => (0.0, 0.0),
            },
            Self::CylinderRe100(c) => match side {
                Side::Left => (c.u_inf, 0.0),
                Side::Right => (c.u_inf, 0.0),
                Side::Bottom | Side::Top => (c.u_inf, 0.0),
            },
            Self::BackwardFacingStep(c) => match side {
                Side::Left => {
                    if y <= c.step_height {
                        (0.0, 0.0)
                    } else {
                        let inlet_height = c.height - c.step_height;
                        let eta = ((y - c.step_height) / inlet_height).clamp(0.0, 1.0);
                        (6.0 * c.u_mean * eta * (1.0 - eta), 0.0)
                    }
                }
                Side::Right => (c.u_mean, 0.0),
                Side::Bottom | Side::Top => (0.0, 0.0),
            },
        }
    }

    pub fn initial_velocity(&self, x: f64, y: f64) -> (f64, f64) {
        match self {
            Self::LidDrivenCavity(_) => (0.0, 0.0),
            Self::CylinderRe100(c) => {
                let r2 = (x - c.xc).powi(2) + (y - c.yc).powi(2);
                if r2 <= (0.5 * c.diameter).powi(2) {
                    (0.0, 0.0)
                } else {
                    // A localized divergence-free perturbation is generated
                    // from a Gaussian streamfunction. It prevents an exactly
                    // symmetric grid and initial field from remaining on the
                    // unstable symmetric wake branch forever.
                    let xp = c.xc + 1.5 * c.diameter;
                    let yp = c.yc + 0.25 * c.diameter;
                    let sigma2 = c.diameter * c.diameter;
                    let dx = x - xp;
                    let dy = y - yp;
                    let psi = c.perturbation * (-(dx * dx + dy * dy) / sigma2).exp();
                    let u_pert = -2.0 * dy / sigma2 * psi;
                    let v_pert = 2.0 * dx / sigma2 * psi;
                    (c.u_inf + u_pert, v_pert)
                }
            }
            Self::BackwardFacingStep(c) => {
                if x < c.step_x && y < c.step_height {
                    (0.0, 0.0)
                } else if y > c.step_height {
                    let inlet_height = c.height - c.step_height;
                    let eta = ((y - c.step_height) / inlet_height).clamp(0.0, 1.0);
                    (6.0 * c.u_mean * eta * (1.0 - eta), 0.0)
                } else {
                    (c.u_mean, 0.0)
                }
            }
        }
    }

    pub fn is_solid(&self, x: f64, y: f64) -> bool {
        match self {
            Self::LidDrivenCavity(_) => false,
            Self::CylinderRe100(c) => {
                (x - c.xc).powi(2) + (y - c.yc).powi(2) <= (0.5 * c.diameter).powi(2)
            }
            Self::BackwardFacingStep(c) => x < c.step_x && y < c.step_height,
        }
    }

    pub fn pressure_reference_required(&self) -> bool {
        matches!(self, Self::LidDrivenCavity(_))
    }

    pub fn cylinder(&self) -> Option<&CylinderCase> {
        match self {
            Self::CylinderRe100(c) => Some(c),
            _ => None,
        }
    }

    pub fn backward_step(&self) -> Option<&BackwardStepCase> {
        match self {
            Self::BackwardFacingStep(c) => Some(c),
            _ => None,
        }
    }
}
