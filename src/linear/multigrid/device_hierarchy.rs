//! CPU setup and coefficient refresh for a GPU aggregation V-cycle. Only setup
//! runs here; restriction, smoothing, prolongation and coarse application are GPU kernels.
use super::aggregation;
use crate::{CsrMatrix, LinearAlgebraError};
pub(crate) struct DeviceLevel {
    pub matrix: CsrMatrix,
    pub parents: Option<Vec<usize>>,
    entries: Vec<usize>,
}
pub(crate) struct DeviceHierarchy {
    pub levels: Vec<DeviceLevel>,
}
impl DeviceHierarchy {
    pub fn new(matrix: &CsrMatrix) -> Result<Self, LinearAlgebraError> {
        let mut current = matrix.clone();
        let mut levels = Vec::new();
        loop {
            if current.nrows() <= 64 {
                levels.push(DeviceLevel {
                    matrix: current,
                    parents: None,
                    entries: Vec::new(),
                });
                break;
            }
            if levels.len() >= 24 {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "GPU AMG hierarchy did not coarsen to bounded terminal size",
                });
            }
            let (parents, count) = aggregation::pairs(&current, levels.len() % 2 == 0, None);
            if count >= current.nrows() {
                return Err(LinearAlgebraError::Breakdown {
                    reason: "GPU AMG aggregation stalled",
                });
            }
            let (coarse, entries) = aggregation::galerkin(&current, &parents, count)?;
            levels.push(DeviceLevel {
                matrix: current,
                parents: Some(parents),
                entries,
            });
            current = coarse;
        }
        Ok(Self { levels })
    }
    pub fn same_topology(&self, matrix: &CsrMatrix) -> bool {
        let first = &self.levels[0].matrix;
        first.nrows() == matrix.nrows()
            && first.row_offsets() == matrix.row_offsets()
            && first.column_indices() == matrix.column_indices()
    }
    pub fn update(&mut self, matrix: &CsrMatrix) -> Result<(), LinearAlgebraError> {
        if !self.same_topology(matrix) {
            return Err(LinearAlgebraError::Breakdown {
                reason: "GPU AMG topology changed",
            });
        }
        self.levels[0]
            .matrix
            .values_mut()
            .copy_from_slice(matrix.values());
        for index in 0..self.levels.len() - 1 {
            let (fine, coarse) = self.levels.split_at_mut(index + 1);
            let fine = &fine[index];
            let coarse = &mut coarse[0].matrix;
            coarse.values_mut().fill(0.);
            for (&value, &entry) in fine.matrix.values().iter().zip(&fine.entries) {
                coarse.values_mut()[entry] += value;
            }
        }
        Ok(())
    }
}
