#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/ros2_ws/install/setup.bash"
set -u

if ros2 node list 2>/dev/null | grep -qx '/encoder_velocity_controller'; then
  echo "Stop /encoder_velocity_controller before a bounded sweep." >&2
  exit 3
fi

exec python3 "$REPO_ROOT/scripts/quasistatic_encoder_sweep.py" "$@"
