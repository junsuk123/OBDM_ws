#!/bin/bash
# Launch Gazebo with only the ArduPlane (alti_transition) robot
set -e

ROS2_WS="${HOME}/ardu_ws"

echo "Starting ArduPlane Gazebo launch..."

if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source "$ROS2_WS/install/setup.bash"
else
  echo "Warning: ROS2 workspace setup.bash not found: $ROS2_WS/install/setup.bash"
fi

export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic

# Launch the alti_transition (plane-like) runway scenario
ros2 launch ardupilot_gz_bringup alti_transition_runway.launch.py use_gz_sim_gui:=true rviz:=false
