#[derive(Clone, Debug)]
pub struct BackwardStepCase {
    pub length: f64,
    pub height: f64,
    pub step_height: f64,
    pub step_x: f64,
    pub rho: f64,
    pub u_mean: f64,
    pub reynolds: f64,
    pub nu: f64,
}

impl Default for BackwardStepCase {
    fn default() -> Self {
        let step_height = 1.0;
        let u_mean = 1.0;
        let reynolds = 100.0;
        let nu = u_mean * step_height / reynolds;
        Self {
            length: 35.0,
            height: 2.0,
            step_height,
            step_x: 5.0,
            rho: 1.0,
            u_mean,
            reynolds,
            nu,
        }
    }
}
