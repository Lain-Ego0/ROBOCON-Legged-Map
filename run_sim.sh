#!/usr/bin/env bash
# Launch the ROBOCON 2027 S-Dog MuJoCo WebUI.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${SIM_PYTHON:-/home/zero/Documents/sdog_rl_lab/.venv/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
  PYTHON="$(command -v python3)"
fi
cd "$ROOT"
exec "$PYTHON" -m sim.app "$@"
