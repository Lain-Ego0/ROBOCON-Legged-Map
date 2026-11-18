#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GLIM_SOURCE="${GLIM_SOURCE:-$ROOT/build/glim-upstream}"
BUILD_DIR="${GLIM_BUILD_DIR:-$ROOT/build/glim-runtime}"
PATCHED_SOURCE="${GLIM_PATCHED_SOURCE_DIR:-$ROOT/build/glim-source}"
COMPAT_PATCH="$ROOT/slam3d/patches/glim-conda-api-compat.patch"

missing=()
for tool in cmake nvcc; do
  command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
done
if [[ ! -f "$GLIM_SOURCE/CMakeLists.txt" ]]; then
  missing+=("GLIM source at $GLIM_SOURCE")
fi
if [[ ! -f "$COMPAT_PATCH" ]]; then
  missing+=("GLIM compatibility patch at $COMPAT_PATCH")
fi
if ((${#missing[@]})); then
  printf 'Cannot configure CUDA GLIM build; missing prerequisites:\n' >&2
  printf '  - %s\n' "${missing[@]}" >&2
  printf '\nInstall the CUDA-enabled gtsam_points and GLIM native dependencies first.\n' >&2
  exit 2
fi

mkdir -p "$PATCHED_SOURCE"
cp -a "$GLIM_SOURCE/." "$PATCHED_SOURCE/"
if ! patch -d "$PATCHED_SOURCE" -p1 --forward --silent < "$COMPAT_PATCH"; then
  if ! rg -q 'pred_T_odom_imu\.pose\(\)\.rotation\(\)' "$PATCHED_SOURCE/src/glim/odometry/robust_initial_state_estimation_odom.cpp" \
    || ! rg -q 'frames\.inner_size\(\)' "$PATCHED_SOURCE/src/glim/odometry/odometry_estimation_gpu.cpp"; then
  printf 'GLIM source does not match the expected compatibility patch context.\n' >&2
  printf 'Set GLIM_PATCHED_SOURCE_DIR to a fresh directory or update the patch.\n' >&2
  exit 3
  fi
fi

cmake -S "$ROOT/slam3d/native" -B "$BUILD_DIR" \
  -DGLIM_SOURCE_DIR="$PATCHED_SOURCE" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES=89 \
  -DBUILD_WITH_CUDA=ON
cmake --build "$BUILD_DIR" --parallel "${BUILD_JOBS:-4}"
