use super::{CsrMatrix, LinearAlgebraError};
use crate::linear::csr::ensure_len;
use rayon::prelude::*;

pub fn dot(left: &[f64], right: &[f64]) -> Result<f64, LinearAlgebraError> {
    dot_with_scratch(left, right, &mut Vec::new())
}

pub(super) fn dot_with_scratch(
    left: &[f64],
    right: &[f64],
    partials: &mut Vec<f64>,
) -> Result<f64, LinearAlgebraError> {
    ensure_len(left.len(), right.len())?;
    // Fixed chunks keep the reduction order independent of worker scheduling.
    let chunk = |(a, b): (&[f64], &[f64])| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
    let value: f64 = if crate::compute::parallel_rows(left.len()) {
        partials.resize(left.len().div_ceil(1024), 0.0);
        partials
            .par_iter_mut()
            .zip(left.par_chunks(1024).zip(right.par_chunks(1024)))
            .for_each(|(output, pair)| *output = chunk(pair));
        // Keep the ordered serial sum identical to the public reduction.
        partials.iter().copied().sum()
    } else {
        left.chunks(1024).zip(right.chunks(1024)).map(chunk).sum()
    };
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
    norm_l2_with_scratch(values, &mut Vec::new())
}

pub(super) fn norm_l2_with_scratch(
    values: &[f64],
    partials: &mut Vec<f64>,
) -> Result<f64, LinearAlgebraError> {
    let squared = dot_with_scratch(values, values, partials)?;
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
    let apply =
        |(index, (input, output)): (usize, (&f64, &mut f64))| -> Result<(), LinearAlgebraError> {
            *output += alpha * input;
            if !output.is_finite() {
                return Err(LinearAlgebraError::NonFiniteValue {
                    context: "AXPY output",
                    index,
                });
            }
            Ok(())
        };
    if crate::compute::parallel_rows(output.len()) {
        input
            .par_iter()
            .zip(output.par_iter_mut())
            .enumerate()
            .try_for_each(apply)
    } else {
        input
            .iter()
            .zip(output.iter_mut())
            .enumerate()
            .try_for_each(apply)
    }
}

pub fn scale(alpha: f64, values: &mut [f64]) -> Result<(), LinearAlgebraError> {
    if !alpha.is_finite() {
        return Err(LinearAlgebraError::NonFiniteValue {
            context: "scale",
            index: 0,
        });
    }
    let apply = |(index, value): (usize, &mut f64)| -> Result<(), LinearAlgebraError> {
        *value *= alpha;
        if !value.is_finite() {
            return Err(LinearAlgebraError::NonFiniteValue {
                context: "scaled vector",
                index,
            });
        }
        Ok(())
    };
    if crate::compute::parallel_rows(values.len()) {
        values.par_iter_mut().enumerate().try_for_each(apply)
    } else {
        values.iter_mut().enumerate().try_for_each(apply)
    }
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

#[cfg(test)]
mod reduction_tests {
    use super::*;
    use crate::compute::{Backend, ComputeOptions, Execution, Scope};

    #[test]
    fn retained_reductions_preserve_order_across_sizes_backends_and_errors() {
        for (backend, threads) in [
            (Backend::Serial, 2),
            (Backend::Parallel, 1),
            (Backend::Parallel, 2),
            (Backend::Parallel, 3),
        ] {
            let runtime = Execution::new(ComputeOptions {
                backend,
                ..Default::default()
            })
            .unwrap();
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| {
                    let _scope = Scope::enter(runtime);
                    let mut scratch = Vec::new();
                    for n in [
                        0, 1, 1023, 1024, 1025, 4095, 4096, 4097, 9217, 17, 16385, 0, 4097,
                    ] {
                        let a: Vec<_> = (0..n)
                            .map(|i| [1e16, 1.0, -1e16, 0.125, -0.25, 0.5][i % 6])
                            .collect();
                        let b: Vec<_> = (0..n)
                            .map(|i| if i % 11 == 0 { -1.0 } else { 1.0 })
                            .collect();
                        let reference: f64 = a
                            .chunks(1024)
                            .zip(b.chunks(1024))
                            .map(|(a, b)| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>())
                            .sum();
                        assert_eq!(
                            dot_with_scratch(&a, &b, &mut scratch).unwrap().to_bits(),
                            reference.to_bits()
                        );
                        assert_eq!(dot(&a, &b).unwrap().to_bits(), reference.to_bits());
                        assert_eq!(
                            norm_l2_with_scratch(&a, &mut scratch).unwrap().to_bits(),
                            norm_l2(&a).unwrap().to_bits()
                        );
                    }
                    if backend == Backend::Serial || threads == 1 {
                        assert_eq!(scratch.capacity(), 0);
                    }
                    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::MAX] {
                        let a = vec![bad; 4097];
                        let b = vec![2.0; 4097];
                        assert_eq!(dot_with_scratch(&a, &b, &mut scratch), dot(&a, &b));
                        assert_eq!(norm_l2_with_scratch(&a, &mut scratch), norm_l2(&a));
                        assert_eq!(dot_with_scratch(&a, &[], &mut scratch), dot(&a, &[]));
                        let good = vec![1.0; 4097];
                        assert_eq!(
                            dot_with_scratch(&good, &good, &mut scratch).unwrap(),
                            4097.0
                        );
                    }
                    // Individual products and chunk sums are finite, final sum is not.
                    let a = vec![f64::MAX / 4.0; 8192];
                    let mut b = vec![0.0; 8192];
                    for i in (0..8192).step_by(1024) {
                        b[i] = 1.0;
                    }
                    assert_eq!(dot_with_scratch(&a, &b, &mut scratch), dot(&a, &b));
                });
        }
    }
}
