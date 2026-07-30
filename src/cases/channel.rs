#[derive(Clone, Debug)]
pub struct ChannelCase {
    pub length: f64,
    pub height: f64,
    pub rho: f64,
    pub u_mean: f64,
    pub reynolds: f64,
    pub nu: f64,
}

impl Default for ChannelCase {
    fn default() -> Self {
        let height = 1.0;
        let u_mean = 1.0;
        let reynolds = 100.0;
        Self {
            length: 10.0,
            height,
            rho: 1.0,
            u_mean,
            reynolds,
            nu: u_mean * height / reynolds,
        }
    }
}
