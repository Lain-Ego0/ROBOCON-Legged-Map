#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_DIR="${GTSAM_POINTS_SOURCE_DIR:-$ROOT/build/gtsam-points-cpu-source/source}"
BUILD_DIR="${GTSAM_POINTS_BUILD_DIR:-$ROOT/build/gtsam-points-conda-build}"
INSTALL_PREFIX="${GTSAM_POINTS_INSTALL_PREFIX:-$ROOT/build/glim-conda-deps}"
VERSION_TAG="${GTSAM_POINTS_VERSION_TAG:-v1.2.2}"

if ! command -v cmake >/dev/null 2>&1; then
  printf 'CMake was not found. Activate the native dependency environment and retry.\n' >&2
  exit 127
fi

if [[ ! -f "$SOURCE_DIR/CMakeLists.txt" ]]; then
  if ! command -v git >/dev/null 2>&1; then
    printf 'Git is required to fetch gtsam_points %s.\n' "$VERSION_TAG" >&2
    exit 127
  fi
  mkdir -p "$(dirname "$SOURCE_DIR")"
  git clone --depth 1 --branch "$VERSION_TAG" \
    https://github.com/koide3/gtsam_points.git "$SOURCE_DIR"
fi

cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
  -DBUILD_WITH_CUDA=OFF \
  -DBUILD_WITH_TBB=ON \
  -DBUILD_WITH_OPENMP=ON \
  -DBUILD_WITH_MARCH_NATIVE=OFF \
  -DBUILD_TESTS=OFF \
  -DBUILD_DEMO=OFF \
  -DBUILD_EXAMPLE=OFF \
  -DBUILD_TOOLS=OFF
cmake --build "$BUILD_DIR" --parallel "${BUILD_JOBS:-4}"
cmake --install "$BUILD_DIR"

CONFIG="$INSTALL_PREFIX/lib/cmake/gtsam_points/gtsam_points-config.cmake"
if [[ ! -f "$CONFIG" ]] || rg -q 'set\(GTSAM_POINTS_USE_CUDA 1\)' "$CONFIG"; then
  printf 'Installed gtsam_points configuration does not confirm a CPU-only build.\n' >&2
  exit 4
fi

printf '\ngtsam_points CPU build installed at %s\n' "$INSTALL_PREFIX"
printf 'Build GLIM with GTSAM_POINTS_DIR=%s/lib/cmake/gtsam_points ./scripts/with_project_env.sh ./slam3d/build_glim.sh\n' "$INSTALL_PREFIX"
