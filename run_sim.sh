#!/usr/bin/env bash
# Launch the ROBOCON 2027 S-Dog MuJoCo WebUI.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

exec "$ROOT/scripts/with_project_env.sh" python -m sim.app "$@"
