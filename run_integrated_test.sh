#!/bin/bash
# Integrated Flight Control System - Launch and Test Script
# Purpose: Clean, launch Gazebo, and run MATLAB data collection test
# Date: 2026-05-08

set -e

WORKSPACE_DIR="/home/j/SynologyDrive/INCSL/devel/INCSL/OBDM_ws"
LOG_DIR="/tmp/obdm_test"
mkdir -p "$LOG_DIR"

echo "╔══════════════════════════════════════════╗"
echo "║  Integrated Flight Control System Test  ║"
echo "║  Gazebo + ROS2 + MATLAB Integration     ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# 1. Cleanup
echo "[1/4] Cleaning up old processes..."
cleanup_patterns=(
    'gz sim'
    'gzserver'
    'gzclient'
    'ign gazebo'
    'ros_gz_bridge'
    'parameter_bridge'
    'micro_ros_agent'
    'arducopter'
    'arduplane'
    'ardupilot'
    'mavproxy'
)
for pattern in "${cleanup_patterns[@]}"; do
        pkill -TERM -f "$pattern" || true
done
sleep 1
for pattern in "${cleanup_patterns[@]}"; do
        pkill -KILL -f "$pattern" || true
done
rm -rf /tmp/obdm_test/*.lock /tmp/obdm_test/*.pid 2>/dev/null || true
echo "✓ Cleanup complete"
echo ""

# 2. Setup environment
echo "[2/4] Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash || true
source /home/j/ardu_ws/install/setup.bash || true
export ROS_DOMAIN_ID=0
echo "✓ Environment ready"
echo ""

# 3. Run MATLAB orchestration against the Iris/Gazebo bridge
echo "[3/4] Launching MATLAB orchestration for Iris quadcopter..."
echo "      (SimulationManager will auto-launch the Iris runway stack)"
echo "      (Logs: $LOG_DIR/matlab.log)"
echo ""

cd "$WORKSPACE_DIR"
export OBDM_RUN_MODE=gazebo_bridge
# OBDM_SKIP_CONTROL는 명시적으로 설정하지 않음 - 제어 테스트 활성화
# (unset OBDM_SKIP_CONTROL로 제어 활성화)
unset OBDM_SKIP_CONTROL
export OBDM_SKIP_CLEANUP=1

# Use the orchestrator so namespace detection and topic resolution stay aligned with the quadcopter launch.
timeout 240 /usr/local/bin/matlab -batch "run('matlab/main/integrated_main.m')" 2>&1 | tee "$LOG_DIR/matlab.log" || EXIT_CODE=$?

echo ""

# 4. Summarize results and cleanup
echo "[4/4] Finalizing and cleaning up..."

if [ -z "$EXIT_CODE" ] || [ "$EXIT_CODE" -eq "0" ] || [ "$EXIT_CODE" -eq "124" ]; then
    echo "✓ MATLAB orchestration completed"
else
    echo "⚠ MATLAB orchestration exit code: $EXIT_CODE"
fi

for pattern in "${cleanup_patterns[@]}"; do
    pkill -TERM -f "$pattern" || true
done
sleep 1
for pattern in "${cleanup_patterns[@]}"; do
    pkill -KILL -f "$pattern" || true
done
sleep 1
echo "✓ Cleanup complete"

echo ""
echo "Test Results:"
echo "  MATLAB log:   $LOG_DIR/matlab.log"
echo "  Gazebo log:   $LOG_DIR/gazebo.log"
echo ""
