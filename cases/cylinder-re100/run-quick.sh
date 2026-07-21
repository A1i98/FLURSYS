#!/usr/bin/env bash
set -euo pipefail

cargo run --release --bin flursys -- \
  cylinder \
  --nx 250 --ny 100 \
  --length 25 --height 10 \
  --diameter 1 --xc 5 --yc 5 \
  --rho 1 --u-inf 1 --re 100 \
  --convection central \
  --perturb 1e-3 \
  --dt 0.004 --t-end 20 --max-steps 5000 \
  --pressure-iters 1200 \
  --print-every 100 \
  --output-every 50 \
  --frame-every 100 \
  --out results/cylinder-re100-quick
