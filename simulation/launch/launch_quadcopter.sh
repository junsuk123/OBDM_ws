#!/bin/bash
# Launch Gazebo with the ArduPilot Iris quadcopter example
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS2_WS="/home/j/ardu_ws"

echo "Starting ArduCopter Gazebo launch..."

if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source "$ROS2_WS/install/setup.bash"
else
  echo "Warning: ROS2 workspace setup.bash not found: $ROS2_WS/install/setup.bash"
fi

export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$ROS2_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$ROS2_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}"
export SDF_PATH="$ROS2_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/models:$ROS2_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds:${SDF_PATH}"

# Launch the fixed Iris launcher (handles package:// URI resolution)
ros2 launch "$SCRIPT_DIR/iris_fixed.launch.py" use_gz_sim_gui:=true rviz:=false