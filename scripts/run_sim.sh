#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCENARIO="${1:-scenarios/moving_platform_no_wind.yaml}"

echo "[run_sim] Workspace: ${ROOT_DIR}"
echo "[run_sim] Scenario: ${SCENARIO}"
echo "[run_sim] Start the existing simulator first if Gazebo/ArduPilot are not already running:"
echo "          ${ROOT_DIR}/run_user_stack.sh"
echo "[run_sim] Then build/source the modular ROS 2 overlay and launch:"
echo "          cd ${ROOT_DIR} && colcon build --symlink-install --packages-select landing_interfaces landing_perception landing_state_estimation landing_ontology_reasoner landing_path_generator landing_safety_layer landing_controller uav_sim_bringup"
echo "          source install/setup.bash"
echo "          ros2 launch uav_sim_bringup landing_stack.launch.py"
