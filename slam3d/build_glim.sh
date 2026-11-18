#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GLIM_SOURCE="${GLIM_SOURCE:-$ROOT/build/glim-upstream}"
BUILD_DIR="${GLIM_BUILD_DIR:-$ROOT/build/glim-cpu-runtime-no-cuda}"
PATCHED_SOURCE="${GLIM_PATCHED_SOURCE_DIR:-$ROOT/build/glim-cpu-source}"
GTSAM_POINTS_DIR="${GTSAM_POINTS_DIR:-}"
COMPAT_PATCH="$ROOT/slam3d/patches/glim-conda-api-compat.patch"

missing=()
for tool in cmake; do
  command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
done
if [[ ! -f "$GLIM_SOURCE/CMakeLists.txt" ]]; then
  missing+=("GLIM source at $GLIM_SOURCE")
fi
if [[ ! -f "$COMPAT_PATCH" ]]; then
  missing+=("GLIM compatibility patch at $COMPAT_PATCH")
fi
if ((${#missing[@]})); then
  printf 'Cannot configure CPU GLIM build; missing prerequisites:\n' >&2
  printf '  - %s\n' "${missing[@]}" >&2
  printf '\nInstall GTSAM, gtsam_points, Eigen3, Boost serialization, OpenMP, spdlog, and GLIM native dependencies first.\n' >&2
  exit 2
fi

mkdir -p "$PATCHED_SOURCE"
cp -a "$GLIM_SOURCE/." "$PATCHED_SOURCE/"
if ! rg -q 'pred_T_odom_imu\.pose\(\)\.rotation\(\)' \
  "$PATCHED_SOURCE/src/glim/odometry/robust_initial_state_estimation_odom.cpp"; then
  if ! patch -d "$PATCHED_SOURCE" -p1 --forward --silent < "$COMPAT_PATCH"; then
    printf 'GLIM source does not match the expected compatibility patch context.\n' >&2
    printf 'Set GLIM_PATCHED_SOURCE_DIR to a fresh directory or update the patch.\n' >&2
    exit 3
  fi
fi

cmake_args=(
  -S "$ROOT/slam3d/native"
  -B "$BUILD_DIR"
  "-DGLIM_SOURCE_DIR=$PATCHED_SOURCE"
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_CUDA=OFF
)
if [[ -n "$GTSAM_POINTS_DIR" ]]; then
  cmake_args+=("-Dgtsam_points_DIR=$GTSAM_POINTS_DIR")
fi
cmake "${cmake_args[@]}"
cmake --build "$BUILD_DIR" --parallel "${BUILD_JOBS:-4}"
