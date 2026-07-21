#!/usr/bin/env bash
set -euo pipefail

cargo run --release --bin flursys -- \
  backward-step \
  --nx 700 --ny 40 \
  --length 35 --height 2 \
  --step-height 1 --step-x 5 \
  --rho 1 --u-mean 1 --re 100 \
  --dt 0.0025 --t-end 100 --max-steps 40000 \
  --pressure-iters 2500 \
  --pressure-tol 1e-5 \
  --print-every 500 \
  --output-every 100 \
  --frame-every 500 \
  --out results/backward-facing-step
