#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-hard}"
WORLD="${OBDM_GAZEBO_WORLD:-runway}"
RESET_LOG="${OBDM_RESET_LOG:-/tmp/obdm_reset.log}"
SOFT_READY_MARKER="/tmp/obdm_soft_reset_ready"

log_reset() {
  local method="$1"
  local request="$2"
  local ok="$3"
  local service_ms="$4"
  local pose_ms="$5"
  local cleanup_ms="$6"
  local message="$7"
  printf '%s method=%s world=%s request=%s ok=%s service_ms=%s pose_ms=nan cleanup_ms=%s message=%s\n' \
    "$(date '+%Y-%m-%dT%H:%M:%S.%3N')" "$method" "$WORLD" "$request" "$ok" "$service_ms" "$cleanup_ms" "$message" | \
    sed "s/pose_ms=nan/pose_ms=${pose_ms}/" >> "$RESET_LOG"
}

terminate_patterns() {
  local grace="$1"
  shift
  local self="$$"
  local parent="${PPID:-}"
  local targets=()
  local pid
  for pattern in "$@"; do
    while IFS= read -r pid; do
      [[ -z "$pid" ]] && continue
      [[ "$pid" == "$self" || "$pid" == "$parent" ]] && continue
      targets+=("$pid")
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done
  if [[ "${#targets[@]}" -eq 0 ]]; then
    sleep "$grace"
    return
  fi
  printf '%s\n' "${targets[@]}" | sort -u | xargs -r kill -TERM 2>/dev/null || true
  sleep "$grace"
  printf '%s\n' "${targets[@]}" | sort -u | xargs -r kill -KILL 2>/dev/null || true
}

if [[ "$MODE" == "soft" ]]; then
  log_reset "soft" "pose" "0" "nan" "nan" "nan" "script soft reset started"
  start_ms="$(date +%s%3N)"
  : > /tmp/obdm_kill_sim_soft_reset_service.log
  for req in \
    'name: "iris", position: {x: 0, y: 0, z: 0.2}, orientation: {w: 1}' \
    'name: "r1_rover", position: {x: 5.0, y: 0, z: 0.05}, orientation: {w: 1}'
  do
    gz service -s "/world/${WORLD}/set_pose" \
      --reqtype gz.msgs.Pose \
      --reptype gz.msgs.Boolean \
      --timeout 2000 \
      --req "$req" >>/tmp/obdm_kill_sim_soft_reset_service.log || MODE="hard"
  done
  if [[ "$MODE" == "hard" ]]; then
      end_ms="$(date +%s%3N)"
      log_reset "soft" "pose" "0" "$((end_ms - start_ms))" "nan" "nan" "script soft reset pose service failed"
      echo "[kill_sim] Soft reset failed; falling back to hard reset."
  fi
  if [[ "$MODE" == "soft" ]]; then
    end_ms="$(date +%s%3N)"
    pose_start_ms="$(date +%s%3N)"
    sleep 0.25
    if timeout 1.5 gz topic -e -n 1 -t "/world/${WORLD}/pose/info" >/tmp/obdm_kill_sim_soft_reset_pose.log 2>/dev/null && \
        grep -q 'name: "iris"' /tmp/obdm_kill_sim_soft_reset_pose.log && \
        grep -q 'name: "r1_rover"' /tmp/obdm_kill_sim_soft_reset_pose.log; then
      pose_end_ms="$(date +%s%3N)"
      log_reset "soft" "pose" "1" "$((end_ms - start_ms))" "$((pose_end_ms - pose_start_ms))" "nan" "script soft reset service and pose check completed"
    else
      pose_end_ms="$(date +%s%3N)"
      log_reset "soft" "pose" "0" "$((end_ms - start_ms))" "$((pose_end_ms - pose_start_ms))" "nan" "script soft reset pose check failed"
      echo "[kill_sim] Soft reset pose check failed; falling back to hard reset."
      MODE="hard"
    fi
  fi
fi

cleanup_start_ms="$(date +%s%3N)"
terminate_patterns 0.4 \
  "sim_vehicle.py" \
  "ontology_reasoner_node" \
  "path_generator_node" \
  "safety_layer_node" \
  "landing_controller_node" \
  "state_estimator_node" \
  "perception_adapter_node" \
  "ros2 launch" \
  "topic_tools" \
  "parameter_bridge" \
  "ros_gz_bridge" \
  "robot_state_publisher" \
  "ros_gz_sim create" \
  "aruco_landing_target_node.py" \
  "aruco_camera_detector_node.py" \
  "gz_virtual_wind_sensor_to_ros2.py" \
  "wind_arrow_marker_node.py" \
  "mavlink_guidance_streamer.py" \
  "realtime_ontology_trajectory_generator.py" \
  "guidance_path_visualizer.py" \
  "iris_rviz_tf_publisher.py" \
  "rover_cmd_vel_to_wheels.py"

if [[ "$MODE" == "hard" ]]; then
  rm -f "$SOFT_READY_MARKER"
  log_reset "hard" "process_cleanup_begin" "0" "nan" "nan" "nan" "script hard reset cleanup started"
  terminate_patterns 1.0 \
    "rviz2" \
    "gz sim" \
    "ros2 launch" \
    "arducopter" \
    "arduplane" \
    "mavproxy" \
    "micro_ros_agent"
  cleanup_end_ms="$(date +%s%3N)"
  log_reset "hard" "process_cleanup" "1" "nan" "nan" "$((cleanup_end_ms - cleanup_start_ms))" "script hard reset cleanup completed"
  echo "[kill_sim] Hard reset requested: Gazebo, RViz, ROS, and ArduPilot processes were signaled."
else
  terminate_patterns 0.4 \
    "arducopter" \
    "arduplane" \
    "mavproxy" \
    "micro_ros_agent"
  cleanup_end_ms="$(date +%s%3N)"
  printf '%s\n' "$WORLD" > "$SOFT_READY_MARKER"
  log_reset "soft" "ros_cleanup" "1" "nan" "nan" "$((cleanup_end_ms - cleanup_start_ms))" "script soft reset ROS cleanup completed"
  echo "[kill_sim] Soft reset requested: Gazebo world reset and ROS/ArduPilot processes were signaled; Gazebo/RViz were preserved."
fi
