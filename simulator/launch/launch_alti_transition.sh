#!/usr/bin/env bash
# Wrapper to launch alti_transition with forgiving arg parsing
set -euo pipefail

# Normalize arguments: accept both "wind_scenario=safe" and "wind_scenario:=safe"
NORMALIZED_ARGS=()
for a in "$@"; do
  if [[ "$a" == wind_scenario=* ]] && [[ "$a" != *":="* ]]; then
    NORMALIZED_ARGS+=("${a/=/:=}")
  else
    NORMALIZED_ARGS+=("$a")
  fi
done

# Ensure ROS env sourced if not already
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ARDU_WS="${OBDM_ARDU_WS:-${REPO_ROOT}/ardu_ws}"
if [ -f "${ARDU_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${ARDU_WS}/install/setup.bash"
fi
source /opt/ros/humble/setup.bash || true

exec ros2 launch ardupilot_gz_bringup alti_transition_runway.launch.py "${NORMALIZED_ARGS[@]}"
