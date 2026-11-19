#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="full"

usage() {
  cat <<'EOF'
Usage: ./scripts/setup_env.sh [--python-only]

Create or reuse the single project Conda environment. By default, build CPU GLIM
and validate its runtime as well.

Options:
  --python-only  Set up Python dependencies without building GLIM.
  -h, --help     Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --python-only) MODE="python-only" ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

cd "$ROOT"
./scripts/with_project_env.sh true

printf '\nChecking Python simulator dependencies...\n'
./scripts/with_project_env.sh python - <<'PY'
import importlib

for module in ("mujoco", "numpy", "PIL", "scipy", "torch"):
    importlib.import_module(module)

import mujoco
import torch

if torch.version.cuda is not None:
    raise RuntimeError("CPU-only PyTorch is required, but CUDA is available")

print(f"MuJoCo {mujoco.__version__}")
print(f"PyTorch {torch.__version__} (policy inference runs on CPU)")
PY

if [[ "$MODE" == "python-only" ]]; then
  printf '\nConda environment is ready. The GLIM backend was not built in --python-only mode.\n'
  exit 0
fi

NATIVE_PREFIX="${ROBOCON_ENV_PREFIX:-$ROOT/build/robocon-env}"

if ! command -v git >/dev/null 2>&1; then
  printf 'Git is required to fetch the pinned GLIM source.\n' >&2
  exit 127
fi

GLIM_SOURCE="${GLIM_SOURCE:-$ROOT/build/glim-upstream}"
GLIM_COMMIT="1a197c999e4865f5cab7c9cd25474128413ebbca"
if [[ ! -f "$GLIM_SOURCE/CMakeLists.txt" ]]; then
  mkdir -p "$(dirname "$GLIM_SOURCE")"
  git clone --filter=blob:none --no-checkout https://github.com/koide3/glim.git "$GLIM_SOURCE"
  git -C "$GLIM_SOURCE" fetch --depth 1 origin "$GLIM_COMMIT"
  git -C "$GLIM_SOURCE" checkout --detach "$GLIM_COMMIT"
elif [[ "$GLIM_SOURCE" == "$ROOT/build/glim-upstream" ]]; then
  git -C "$GLIM_SOURCE" fetch --depth 1 origin "$GLIM_COMMIT"
  git -C "$GLIM_SOURCE" checkout --detach "$GLIM_COMMIT"
fi

GTSAM_POINTS_INSTALL_PREFIX="${GTSAM_POINTS_INSTALL_PREFIX:-$ROOT/build/glim-conda-deps}"
GLIM_BUILD_DIR="${GLIM_BUILD_DIR:-$ROOT/build/glim-conda-runtime}"
GLIM_PATCHED_SOURCE_DIR="${GLIM_PATCHED_SOURCE_DIR:-$ROOT/build/glim-conda-source}"
printf '\nUsing the project Conda environment at %s...\n' "$NATIVE_PREFIX"
./scripts/with_project_env.sh true

printf '\nBuilding CPU-only gtsam_points dependency...\n'
GTSAM_POINTS_INSTALL_PREFIX="$GTSAM_POINTS_INSTALL_PREFIX" \
  ./scripts/with_project_env.sh ./scripts/build_gtsam_points_cpu.sh

printf '\nBuilding CPU GLIM at pinned commit %s...\n' "$GLIM_COMMIT"
GLIM_SOURCE="$GLIM_SOURCE" \
GLIM_PATCHED_SOURCE_DIR="$GLIM_PATCHED_SOURCE_DIR" \
GLIM_BUILD_DIR="$GLIM_BUILD_DIR" \
GTSAM_POINTS_DIR="$GTSAM_POINTS_INSTALL_PREFIX/lib/cmake/gtsam_points" \
  ./scripts/with_project_env.sh ./slam3d/build_glim.sh

printf '\nValidating CPU GLIM modules and runtime dependencies...\n'
GLIM_BUILD_DIR="$GLIM_BUILD_DIR" \
ROBOCON_GLIM_LIBRARY_DIRS="$NATIVE_PREFIX/lib:$GTSAM_POINTS_INSTALL_PREFIX/lib" \
  ./scripts/with_project_env.sh ./slam3d/validate_glim.sh

printf '\nRunning an end-to-end GLIM sensor replay...\n'
ROBOCON_GLIM_BUILD_DIR="$GLIM_BUILD_DIR" \
  ./scripts/with_project_env.sh python slam3d/replay_mujoco.py --duration 8

GLIM_BUILD_DIR_RELATIVE="$(realpath --relative-to="$ROOT" "$GLIM_BUILD_DIR")"
printf '\nEnvironment setup complete. Start the simulator with: ROBOCON_GLIM_BUILD_DIR="%s" ./run_sim.sh --no-open\n' "$GLIM_BUILD_DIR_RELATIVE"
