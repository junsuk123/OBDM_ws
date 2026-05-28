#!/bin/bash
# Quick wind topic diagnostics script

echo "═══════════════════════════════════════════════════════"
echo "Wind Topic Diagnostics"
echo "═══════════════════════════════════════════════════════"
echo ""

# Source ROS environment
export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic
source /opt/ros/humble/setup.bash 2>/dev/null || true
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ARDU_WS="${OBDM_ARDU_WS:-${REPO_ROOT}/ardu_ws}"
if [ -f "${ARDU_WS}/install/setup.bash" ]; then
  source "${ARDU_WS}/install/setup.bash" 2>/dev/null || true
fi

echo "[1] Checking ROS2 topics containing 'wind'"
echo "─────────────────────────────────────────"
ros2 topic list 2>/dev/null | grep -i wind || echo "⚠ No wind topics found"
echo ""

echo "[2] Checking Gazebo topics containing 'wind'"
echo "─────────────────────────────────────────"
gz topic -l 2>/dev/null | grep -i wind || echo "⚠ No wind topics found in Gazebo"
echo ""

echo "[3] Checking parameter_bridge status"
echo "─────────────────────────────────────────"
if pgrep -f parameter_bridge >/dev/null 2>&1; then
  echo "✓ parameter_bridge is running"
  pgrep -f parameter_bridge -a || true
else
  echo "✗ parameter_bridge is NOT running"
fi
echo ""

echo "[4] Attempting to publish wind command"
echo "─────────────────────────────────────────"
echo "Publishing gz.msgs.Wind to /world/runway/wind..."
gz topic -t /world/runway/wind -m gz.msgs.Wind \
  -p 'linear_velocity: {x: 10.0, y: 0.0, z: 0.0} enable_wind: true' 2>&1 | head -5 || echo "⚠ Publish failed"
sleep 1

echo ""
echo "[5] Checking if message was received by Gazebo"
echo "─────────────────────────────────────────"
echo "Reading from Gazebo /world/runway/wind topic (timeout 3s)..."
timeout 3 gz topic -e -t /world/runway/wind -n 1 2>&1 | head -10 || echo "⚠ No messages received"
echo ""

echo "[5b] Gazebo wind topic info"
echo "─────────────────────────────────────────"
gz topic -i -t /world/runway/wind 2>&1 | head -20 || echo "⚠ Topic info unavailable"
echo ""

echo "[6] Available ROS2 nodes"
echo "─────────────────────────────────────────"
ros2 node list 2>/dev/null | grep -i "bridge\|wind\|iris" || echo "⚠ No relevant nodes found"
echo ""

echo "[7] ROS2 service list (wind/bridge related)"
echo "─────────────────────────────────────────"
ros2 service list 2>/dev/null | grep -i "wind\|bridge" || echo "⚠ No wind/bridge services found"
echo ""

echo "═══════════════════════════════════════════════════════"
echo "Diagnostics complete"
echo "═══════════════════════════════════════════════════════"
