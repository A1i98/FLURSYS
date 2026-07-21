#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <frames-directory> <speed|vorticity> [fps]"
  exit 1
fi

frames_dir="$1"
field="$2"
fps="${3:-12}"

ffmpeg -y -framerate "$fps" \
  -i "${frames_dir}/${field}_%05d.ppm" \
  -c:v libx264 -pix_fmt yuv420p -crf 18 \
  "${frames_dir%/}/${field}.mp4"
