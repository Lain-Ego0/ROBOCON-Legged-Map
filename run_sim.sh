#!/usr/bin/env bash
# Launch the ROBOCON 2027 S-Dog MuJoCo WebUI.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if [[ -n "${SIM_PYTHON:-}" ]]; then
  if [[ ! -x "$SIM_PYTHON" ]]; then
    printf 'SIM_PYTHON is not executable: %s\n' "$SIM_PYTHON" >&2
    exit 126
  fi
  exec "$SIM_PYTHON" -m sim.app "$@"
fi

if command -v uv >/dev/null 2>&1; then
  exec uv run --locked python -m sim.app "$@"
fi

if [[ -x "$ROOT/.venv/bin/python" ]]; then
  exec "$ROOT/.venv/bin/python" -m sim.app "$@"
fi

printf 'Project Python environment is unavailable. Run ./scripts/setup_env.sh first, or set SIM_PYTHON to an executable path.\n' >&2
exit 127
