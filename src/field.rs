use std::ops::{Index, IndexMut};

#[derive(Clone, Debug)]
pub struct Field2D {
    nx: usize,
    ny: usize,
    data: Vec<f64>,
}

impl Field2D {
    pub fn new(nx: usize, ny: usize, value: f64) -> Self {
        assert!(nx > 0 && ny > 0, "Field dimensions must be positive");
        Self {
            nx,
            ny,
            data: vec![value; nx * ny],
        }
    }

    #[inline]
    pub fn nx(&self) -> usize {
        self.nx
    }

    #[inline]
    pub fn ny(&self) -> usize {
        self.ny
    }

    #[inline]
    pub fn idx(&self, i: usize, j: usize) -> usize {
        debug_assert!(i < self.nx && j < self.ny);
        i + self.nx * j
    }

    pub fn fill(&mut self, value: f64) {
        self.data.fill(value);
    }

    pub fn as_slice(&self) -> &[f64] {
        &self.data
    }

    pub fn as_mut_slice(&mut self) -> &mut [f64] {
        &mut self.data
    }

    pub fn max_abs(&self) -> f64 {
        self.data.iter().fold(0.0_f64, |m, &x| m.max(x.abs()))
    }

    pub fn min_max(&self) -> (f64, f64) {
        let mut min_v = f64::INFINITY;
        let mut max_v = f64::NEG_INFINITY;
        for &v in &self.data {
            min_v = min_v.min(v);
            max_v = max_v.max(v);
        }
        (min_v, max_v)
    }
}

impl Index<(usize, usize)> for Field2D {
    type Output = f64;

    #[inline]
    fn index(&self, index: (usize, usize)) -> &Self::Output {
        let k = self.idx(index.0, index.1);
        &self.data[k]
    }
}

impl IndexMut<(usize, usize)> for Field2D {
    #[inline]
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        let k = self.idx(index.0, index.1);
        &mut self.data[k]
    }
}

#[derive(Clone, Debug)]
pub struct Mask2D {
    nx: usize,
    ny: usize,
    data: Vec<bool>,
}

impl Mask2D {
    pub fn new(nx: usize, ny: usize, value: bool) -> Self {
        Self {
            nx,
            ny,
            data: vec![value; nx * ny],
        }
    }

    #[inline]
    pub fn idx(&self, i: usize, j: usize) -> usize {
        debug_assert!(i < self.nx && j < self.ny);
        i + self.nx * j
    }

    pub fn nx(&self) -> usize {
        self.nx
    }

    pub fn ny(&self) -> usize {
        self.ny
    }

    pub fn as_slice(&self) -> &[bool] {
        &self.data
    }
}

impl Index<(usize, usize)> for Mask2D {
    type Output = bool;

    #[inline]
    fn index(&self, index: (usize, usize)) -> &Self::Output {
        let k = self.idx(index.0, index.1);
        &self.data[k]
    }
}

impl IndexMut<(usize, usize)> for Mask2D {
    #[inline]
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        let k = self.idx(index.0, index.1);
        &mut self.data[k]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn field_indexing_is_row_major() {
        let mut f = Field2D::new(4, 3, 0.0);
        f[(2, 1)] = 7.0;
        assert_eq!(f.as_slice()[6], 7.0);
    }
}
