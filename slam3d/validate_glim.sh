#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${GLIM_BUILD_DIR:-$ROOT/build/glim-cpu-runtime}"
CORE="$BUILD_DIR/libglim.so"
ODOMETRY_PLUGIN="$BUILD_DIR/libodometry_estimation_cpu.so"
SUBMAPPING_PLUGIN="$BUILD_DIR/libsub_mapping_passthrough.so"
GLOBAL_MAPPING_PLUGIN="$BUILD_DIR/libglobal_mapping_pose_graph.so"

for library in "$CORE" "$ODOMETRY_PLUGIN" "$SUBMAPPING_PLUGIN" "$GLOBAL_MAPPING_PLUGIN" "$BUILD_DIR/mujoco_glim_bridge"; do
  if [[ ! -f "$library" ]]; then
    printf 'Missing build output: %s\n' "$library" >&2
    exit 2
  fi
done

python3 - "$CORE" "$ODOMETRY_PLUGIN" "$SUBMAPPING_PLUGIN" "$GLOBAL_MAPPING_PLUGIN" <<'PY'
import ctypes
import os
import sys

ctypes.CDLL(os.path.abspath(sys.argv[1]), mode=os.RTLD_GLOBAL)
for library in sys.argv[2:]:
    ctypes.CDLL(os.path.abspath(library), mode=os.RTLD_NOW)
print("GLIM CPU odometry, passthrough submapping, and pose-graph modules loaded successfully")
PY
