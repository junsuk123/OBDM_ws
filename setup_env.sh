#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export OBDM_WS="${OBDM_WS:-${REPO_ROOT}}"
export OBDM_ARDU_WS="${OBDM_ARDU_WS:-${REPO_ROOT}/ardu_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export GZ_VERSION="${GZ_VERSION:-harmonic}"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if [ -f "${OBDM_ARDU_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1090
  set +u
  source "${OBDM_ARDU_WS}/install/setup.bash"
  set -u
fi

echo "OBDM_WS=${OBDM_WS}"
echo "OBDM_ARDU_WS=${OBDM_ARDU_WS}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "GZ_VERSION=${GZ_VERSION}"
