#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${OBDM_ARDU_WS:-${REPO_ROOT}/ardu_ws}"
PKG_PATH="${WORKSPACE_ROOT}/src/ros_gz/ros_gz_interfaces"

if [ ! -d "$WORKSPACE_ROOT" ]; then
  echo "Workspace root not found: $WORKSPACE_ROOT"
  exit 1
fi

if [ ! -d "$PKG_PATH" ]; then
  echo "Package path not found: $PKG_PATH"
  exit 1
fi

cat <<'MATLAB' > /tmp/gen_rosgzm_msgs.m
try
  workspaceRoot = getenv('WORKSPACE_ROOT');
  pkgPath = getenv('PKG_PATH');
  tempWorkspace = getenv('TEMP_WORKSPACE_ROOT');
  disp(['Attempting ros2genmsg from flattened workspace: ' tempWorkspace]);
  try
    ros2genmsg(tempWorkspace);
    disp('ros2genmsg completed from flattened workspace');
  catch rootME
    disp('Flattened-workspace ros2genmsg failed; retrying workspace root and package path for diagnostics.');
    disp(getReport(rootME));
    disp(['Retrying ros2genmsg from workspace root: ' workspaceRoot]);
    try
      ros2genmsg(workspaceRoot);
      disp('ros2genmsg completed from workspace root');
      exit(0);
    catch workspaceME
      disp(getReport(workspaceME));
    end
    disp(['Retrying ros2genmsg from package path: ' pkgPath]);
    ros2genmsg(pkgPath);
    disp('ros2genmsg completed from package path');
  end
catch ME
  disp(getReport(ME));
  exit(1);
end
exit(0);
MATLAB

export WORKSPACE_ROOT="$WORKSPACE_ROOT"
export PKG_PATH="$PKG_PATH"

TEMP_WORKSPACE_ROOT="$(mktemp -d /tmp/matlab_ros_gz_interfaces.XXXXXX)"
mkdir -p "$TEMP_WORKSPACE_ROOT/src"
cp -a "$PKG_PATH" "$TEMP_WORKSPACE_ROOT/src/ros_gz_interfaces"
export TEMP_WORKSPACE_ROOT

cleanup() {
  rm -f /tmp/gen_rosgzm_msgs.m
  rm -rf "$TEMP_WORKSPACE_ROOT"
}
trap cleanup EXIT

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "${WORKSPACE_ROOT}/install/setup.bash"
  set -u
fi

matlab -batch "run('/tmp/gen_rosgzm_msgs.m');"
