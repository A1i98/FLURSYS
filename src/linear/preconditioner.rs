use super::{CsrMatrix, LinearAlgebraError};

/// Diagonal preconditioner storing reciprocal diagonal coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct JacobiPreconditioner {
    inverse_diagonal: Vec<f64>,
}

impl JacobiPreconditioner {
    pub fn new(matrix: &CsrMatrix) -> Result<Self, LinearAlgebraError> {
        let inverse_diagonal = matrix
            .diagonal()?
            .into_iter()
            .enumerate()
            .map(|(row, diagonal)| {
                let inverse = 1.0 / diagonal;
                if inverse.is_finite() {
                    Ok(inverse)
                } else {
                    Err(LinearAlgebraError::NonFiniteValue {
                        context: "inverse diagonal",
                        index: row,
                    })
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self { inverse_diagonal })
    }

    pub fn len(&self) -> usize {
        self.inverse_diagonal.len()
    }

    pub fn is_empty(&self) -> bool {
        self.inverse_diagonal.is_empty()
    }

    pub fn apply_into(
        &self,
        residual: &[f64],
        output: &mut [f64],
    ) -> Result<(), LinearAlgebraError> {
        if residual.len() != self.inverse_diagonal.len() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: self.inverse_diagonal.len(),
                actual: residual.len(),
            });
        }
        if output.len() != self.inverse_diagonal.len() {
            return Err(LinearAlgebraError::DimensionMismatch {
                expected: self.inverse_diagonal.len(),
                actual: output.len(),
            });
        }
        for (index, ((residual, inverse), output)) in residual
            .iter()
            .zip(&self.inverse_diagonal)
            .zip(output)
            .enumerate()
        {
            *output = residual * inverse;
            if !output.is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "preconditioned residual",
                    index,
                });
            }
        }
        Ok(())
    }
}
