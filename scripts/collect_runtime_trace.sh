#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/collect_runtime_trace.sh 10
# Records a short trace of important topics for N seconds (default 10).

DURATION=${1:-10}
OUTDIR=${2:-runtime_traces}
mkdir -p "$OUTDIR"

echo "Collecting topics for ${DURATION}s -> ${OUTDIR}"

timeout "$DURATION" ros2 topic echo --no-daemon /landing/reference_pose > "$OUTDIR/reference_pose.log" 2>&1 || true &
timeout "$DURATION" ros2 topic echo --no-daemon /landing/state > "$OUTDIR/landing_state.log" 2>&1 || true &
timeout "$DURATION" ros2 topic echo --no-daemon /cmd_vel > "$OUTDIR/cmd_vel.log" 2>&1 || true &
timeout "$DURATION" ros2 topic echo --no-daemon /obdm/guidance_setpoint geometry_msgs/msg/PoseStamped > "$OUTDIR/guidance_setpoint.log" 2>&1 || true &
timeout "$DURATION" ros2 topic echo --no-daemon /obdm/current_guidance_target geometry_msgs/msg/PoseStamped > "$OUTDIR/current_guidance_target.log" 2>&1 || true &
timeout "$DURATION" ros2 topic echo --no-daemon /iris/odometry nav_msgs/msg/Odometry > "$OUTDIR/iris_odometry.log" 2>&1 || true &
timeout "$DURATION" ros2 topic echo --no-daemon /landing_marker/odom nav_msgs/msg/Odometry > "$OUTDIR/landing_marker_odom.log" 2>&1 || true &
timeout "$DURATION" ros2 topic echo --no-daemon /landing_marker/ground_truth/odom nav_msgs/msg/Odometry > "$OUTDIR/landing_marker_ground_truth_odom.log" 2>&1 || true &

wait
echo "Done. Files in ${OUTDIR}"
