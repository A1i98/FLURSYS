#!/usr/bin/env bash
set -euo pipefail

# Benchmark mesh from the original specification: 500 x 200, dx=dy=0.05.
cargo run --release --bin flursys -- \
  cylinder \
  --nx 500 --ny 200 \
  --length 25 --height 10 \
  --diameter 1 --xc 5 --yc 5 \
  --rho 1 --u-inf 1 --re 100 \
  --convection central \
  --perturb 1e-3 \
  --dt 0.002 --t-end 100 --max-steps 50000 \
  --pressure-iters 2500 \
  --pressure-tol 1e-5 \
  --print-every 1000 \
  --output-every 100 \
  --frame-every 500 \
  --out results/cylinder-re100
