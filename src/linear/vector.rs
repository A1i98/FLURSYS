use super::{CsrMatrix, LinearAlgebraError};
use crate::linear::csr::ensure_len;

pub fn dot(left: &[f64], right: &[f64]) -> Result<f64, LinearAlgebraError> {
    ensure_len(left.len(), right.len())?;
    let value: f64 = left
        .iter()
        .zip(right)
        .map(|(left, right)| left * right)
        .sum();
    if value.is_finite() {
        Ok(value)
    } else {
        Err(LinearAlgebraError::NonFiniteValue {
            context: "dot product",
            index: 0,
        })
    }
}

pub fn norm_l2(values: &[f64]) -> Result<f64, LinearAlgebraError> {
    let squared = dot(values, values)?;
    let norm = squared.sqrt();
    if norm.is_finite() {
        Ok(norm)
    } else {
        Err(LinearAlgebraError::NonFiniteValue {
            context: "L2 norm",
            index: 0,
        })
    }
}

pub fn norm_inf(values: &[f64]) -> Result<f64, LinearAlgebraError> {
    let mut maximum = 0.0_f64;
    for (index, value) in values.iter().enumerate() {
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "vector",
                index,
            });
        }
        maximum = maximum.max(value.abs());
    }
    Ok(maximum)
}

/// Computes `output += alpha * input` in place.
pub fn axpy(alpha: f64, input: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
    ensure_len(input.len(), output.len())?;
    if !alpha.is_finite() {
        return Err(LinearAlgebraError::NonFiniteValue {
            context: "AXPY scale",
            index: 0,
        });
    }
    for (index, (input, output)) in input.iter().zip(output).enumerate() {
        *output += alpha * input;
        if !output.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "AXPY output",
                index,
            });
        }
    }
    Ok(())
}

pub fn scale(alpha: f64, values: &mut [f64]) -> Result<(), LinearAlgebraError> {
    if !alpha.is_finite() {
        return Err(LinearAlgebraError::NonFiniteValue {
            context: "scale",
            index: 0,
        });
    }
    for (index, value) in values.iter_mut().enumerate() {
        *value *= alpha;
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "scaled vector",
                index,
            });
        }
    }
    Ok(())
}

pub fn copy_into(input: &[f64], output: &mut [f64]) -> Result<(), LinearAlgebraError> {
    ensure_len(input.len(), output.len())?;
    for (index, value) in input.iter().enumerate() {
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "input vector",
                index,
            });
        }
    }
    output.copy_from_slice(input);
    Ok(())
}

pub fn residual(
    matrix: &CsrMatrix,
    solution: &[f64],
    rhs: &[f64],
) -> Result<Vec<f64>, LinearAlgebraError> {
    let mut output = vec![0.0; matrix.nrows()];
    residual_into(matrix, solution, rhs, &mut output)?;
    Ok(output)
}

/// Overwrites `output` with `rhs - matrix * solution`.
pub fn residual_into(
    matrix: &CsrMatrix,
    solution: &[f64],
    rhs: &[f64],
    output: &mut [f64],
) -> Result<(), LinearAlgebraError> {
    ensure_len(matrix.nrows(), rhs.len())?;
    ensure_len(matrix.nrows(), output.len())?;
    matrix.spmv_into(solution, output)?;
    for (index, (output, rhs)) in output.iter_mut().zip(rhs).enumerate() {
        if !rhs.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "right-hand side",
                index,
            });
        }
        *output = rhs - *output;
        if !output.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "residual",
                index,
            });
        }
    }
    Ok(())
}
