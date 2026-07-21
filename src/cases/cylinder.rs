#[derive(Clone, Debug)]
pub struct CylinderCase {
    pub length: f64,
    pub height: f64,
    pub diameter: f64,
    pub xc: f64,
    pub yc: f64,
    pub rho: f64,
    pub u_inf: f64,
    pub reynolds: f64,
    pub mu: f64,
    pub nu: f64,
    /// Small divergence-free wake perturbation used to trigger the physical
    /// symmetry-breaking mode at Re=100.
    pub perturbation: f64,
}

impl Default for CylinderCase {
    fn default() -> Self {
        Self {
            length: 25.0,
            height: 10.0,
            diameter: 1.0,
            xc: 5.0,
            yc: 5.0,
            rho: 1.0,
            u_inf: 1.0,
            reynolds: 100.0,
            mu: 0.01,
            nu: 0.01,
            perturbation: 1.0e-3,
        }
    }
}
