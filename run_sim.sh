#!/usr/bin/env bash
# Launch the ROBOCON 2027 S-Dog MuJoCo WebUI.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${SIM_PYTHON:-$(command -v python3 || true)}"
if [[ -z "$PYTHON" || ! -x "$PYTHON" ]]; then
  printf 'Python 3 was not found. Install the simulator dependencies or set SIM_PYTHON to an executable path.\n' >&2
  exit 127
fi
cd "$ROOT"
exec "$PYTHON" -m sim.app "$@"
