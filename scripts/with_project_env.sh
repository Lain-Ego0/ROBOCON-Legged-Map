#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="${ROBOCON_ENV_PREFIX:-$ROOT/build/robocon-env}"
GTSAM_POINTS_PREFIX="${GTSAM_POINTS_INSTALL_PREFIX:-$ROOT/build/glim-conda-deps}"
if [[ "$ENV_PREFIX" != /* ]]; then
  ENV_PREFIX="$ROOT/$ENV_PREFIX"
fi
if [[ "$GTSAM_POINTS_PREFIX" != /* ]]; then
  GTSAM_POINTS_PREFIX="$ROOT/$GTSAM_POINTS_PREFIX"
fi
ENVIRONMENT_FILE="$ROOT/slam3d/native-environment.yml"

if [[ "${ROBOCON_GLIM_NATIVE_ACTIVE:-}" == "1" ]]; then
  exec "$@"
fi

if (($# == 0)); then
  printf 'Usage: ./scripts/with_project_env.sh command [args...]\n' >&2
  exit 2
fi

if [[ "$(uname -s)" != "Linux" || "$(uname -m)" != "x86_64" ]]; then
  printf 'The project Conda environment currently supports Linux x86-64.\n' >&2
  exit 2
fi

if ! command -v conda >/dev/null 2>&1; then
  printf 'Conda is required for the project Conda environment. Install Miniforge or Miniconda, then rerun this command.\n' >&2
  exit 127
fi

if [[ ! -f "$ENV_PREFIX/conda-meta/history" ]]; then
  conda env create --yes --prefix "$ENV_PREFIX" --file "$ENVIRONMENT_FILE"
fi

CONDA_BASE="$(conda info --base)"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "$ENV_PREFIX"

export ROBOCON_ENV_PREFIX="$ENV_PREFIX"
export ROBOCON_GLIM_NATIVE_PREFIX="$ENV_PREFIX"
export GTSAM_POINTS_INSTALL_PREFIX="$GTSAM_POINTS_PREFIX"
export CMAKE_PREFIX_PATH="$ENV_PREFIX:$GTSAM_POINTS_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export ROBOCON_GLIM_LIBRARY_DIRS="$ENV_PREFIX/lib:$GTSAM_POINTS_PREFIX/lib${ROBOCON_GLIM_LIBRARY_DIRS:+:$ROBOCON_GLIM_LIBRARY_DIRS}"
export LD_LIBRARY_PATH="$ENV_PREFIX/lib:$GTSAM_POINTS_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROBOCON_GLIM_NATIVE_ACTIVE=1

exec "$@"
