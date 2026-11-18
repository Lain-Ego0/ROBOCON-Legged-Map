#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${GLIM_BUILD_DIR:-$ROOT/build/glim-cpu-runtime-no-cuda}"
CORE="$BUILD_DIR/libglim.so"
ODOMETRY_PLUGIN="$BUILD_DIR/libodometry_estimation_cpu.so"
SUBMAPPING_PLUGIN="$BUILD_DIR/libsub_mapping_passthrough.so"
GLOBAL_MAPPING_PLUGIN="$BUILD_DIR/libglobal_mapping_pose_graph.so"

for library in "$CORE" "$ODOMETRY_PLUGIN" "$SUBMAPPING_PLUGIN" "$GLOBAL_MAPPING_PLUGIN" "$BUILD_DIR/mujoco_glim_bridge"; do
  if [[ ! -f "$library" ]]; then
    printf 'Missing build output: %s\n' "$library" >&2
    exit 2
  fi
  if ldd "$library" | rg -qi 'lib(cuda|cudart|gtsam_points_cuda)'; then
    printf 'CPU GLIM output still depends on a CUDA library: %s\n' "$library" >&2
    ldd "$library" | rg -i 'lib(cuda|cudart|gtsam_points_cuda)' >&2 || true
    printf 'Rebuild gtsam_points with BUILD_WITH_CUDA=OFF, then rebuild GLIM.\n' >&2
    exit 3
  fi
done

python3 - "$CORE" "$ODOMETRY_PLUGIN" "$SUBMAPPING_PLUGIN" "$GLOBAL_MAPPING_PLUGIN" <<'PY'
import ctypes
import os
import sys

ctypes.CDLL(os.path.abspath(sys.argv[1]), mode=os.RTLD_GLOBAL)
for library in sys.argv[2:]:
    ctypes.CDLL(os.path.abspath(library), mode=os.RTLD_NOW)
print("GLIM CPU modules loaded successfully with no CUDA runtime dependencies")
PY
