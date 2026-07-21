#!/usr/bin/env bash
set -euo pipefail

cargo run --release --bin flursys -- \
  cavity \
  --nx 64 --ny 64 \
  --re 100 \
  --dt 0.001 \
  --t-end 100 \
  --max-steps 100000 \
  --pressure-iters 600 \
  --pressure-tol 1e-5 \
  --print-every 500 \
  --output-every 100 \
  --frame-every 1000 \
  --out results/cavity
