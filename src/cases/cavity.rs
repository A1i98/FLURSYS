#[derive(Clone, Debug)]
pub struct CavityCase {
    pub length: f64,
    pub height: f64,
    pub rho: f64,
    pub lid_velocity: f64,
    pub reynolds: f64,
    pub nu: f64,
}

impl Default for CavityCase {
    fn default() -> Self {
        let length = 1.0;
        let lid_velocity = 1.0;
        let reynolds = 100.0;
        let nu = lid_velocity * length / reynolds;
        Self {
            length,
            height: 1.0,
            rho: 1.0,
            lid_velocity,
            reynolds,
            nu,
        }
    }
}
