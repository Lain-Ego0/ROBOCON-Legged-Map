#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="full"

usage() {
  cat <<'EOF'
Usage: ./scripts/setup_env.sh [--python-only]

Prepare the locked Python environment. By default, also build and validate the
GLIM CPU backend when its source and native dependencies are available.

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

if ! command -v uv >/dev/null 2>&1; then
  if ! command -v curl >/dev/null 2>&1; then
    printf 'uv is missing and curl is unavailable. Install uv, then rerun this script.\n' >&2
    exit 127
  fi
  installer="$(mktemp)"
  trap 'rm -f "$installer"' EXIT
  curl --fail --location --silent --show-error https://astral.sh/uv/install.sh -o "$installer"
  sh "$installer"
  export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
  if ! command -v uv >/dev/null 2>&1; then
    printf 'uv installation completed, but uv was not found in the expected user paths.\n' >&2
    exit 127
  fi
fi

cd "$ROOT"
uv python install 3.12
uv sync --locked

printf '\nChecking Python simulator dependencies...\n'
uv run python - <<'PY'
import importlib

for module in ("mujoco", "numpy", "PIL", "scipy", "torch"):
    importlib.import_module(module)

import mujoco
import torch

if torch.version.cuda is not None:
    raise RuntimeError("A CPU-only PyTorch build is required, but this wheel includes CUDA")

print(f"MuJoCo {mujoco.__version__}")
print(f"PyTorch {torch.__version__} (policy inference runs on CPU)")
PY

if [[ "$MODE" == "python-only" ]]; then
  printf '\nPython environment is ready. Run the simulator with: uv run python -m sim.app --no-open\n'
  exit 0
fi

GLIM_SOURCE="${GLIM_SOURCE:-$ROOT/build/glim-upstream}"
missing=()
for tool in cmake; do
  command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
done
[[ -f "$GLIM_SOURCE/CMakeLists.txt" ]] || missing+=("GLIM source at $GLIM_SOURCE")

if ((${#missing[@]})); then
  printf '\nPython environment is ready, but the GLIM backend cannot be built yet.\n' >&2
  printf 'Missing native prerequisites:\n' >&2
  printf '  - %s\n' "${missing[@]}" >&2
  printf '\nInstall CMake, GTSAM, gtsam_points and GLIM native dependencies,\n' >&2
  printf 'then place a compatible GLIM checkout in build/glim-upstream/ or set GLIM_SOURCE.\n' >&2
  printf 'See sim/docs/reproduction.md for the native dependency list.\n' >&2
  exit 2
fi

GTSAM_POINTS_INSTALL_PREFIX="${GTSAM_POINTS_INSTALL_PREFIX:-$ROOT/build/glim-cpu-deps}"
GLIM_BUILD_DIR="${GLIM_BUILD_DIR:-$ROOT/build/glim-cpu-runtime-no-cuda}"
printf '\nBuilding CPU-only gtsam_points dependency...\n'
GTSAM_POINTS_INSTALL_PREFIX="$GTSAM_POINTS_INSTALL_PREFIX" ./scripts/build_gtsam_points_cpu.sh

printf '\nBuilding GLIM CPU backend...\n'
GLIM_SOURCE="$GLIM_SOURCE" \
GLIM_BUILD_DIR="$GLIM_BUILD_DIR" \
GTSAM_POINTS_DIR="$GTSAM_POINTS_INSTALL_PREFIX/lib/cmake/gtsam_points" \
./slam3d/build_glim.sh
printf '\nValidating GLIM CPU modules...\n'
GLIM_BUILD_DIR="$GLIM_BUILD_DIR" ./slam3d/validate_glim.sh

GLIM_BUILD_DIR_RELATIVE="$(realpath --relative-to="$ROOT" "$GLIM_BUILD_DIR")"
printf '\nEnvironment setup complete. Start the simulator with: ROBOCON_GLIM_BUILD_DIR="%s" uv run python -m sim.app --no-open\n' "$GLIM_BUILD_DIR_RELATIVE"
