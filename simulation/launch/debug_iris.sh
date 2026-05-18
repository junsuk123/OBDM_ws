#!/bin/bash
# Simple debug launcher - runs each component separately with clear logs

set -e

export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic
ROS2_WS="${HOME}/ardu_ws"

source /opt/ros/humble/setup.bash
if [ -f "$ROS2_WS/install/setup.bash" ]; then
  source "$ROS2_WS/install/setup.bash"
fi

echo "=========================================="
echo "Starting Gazebo server..."
echo "=========================================="

# Start Gazebo server
gz sim -v4 -s -r "$ROS2_WS/install/ardupilot_gz_gazebo/share/ardupilot_gz_gazebo/worlds/runway.sdf" &
GZ_SERVER_PID=$!
echo "Gazebo server PID: $GZ_SERVER_PID"
sleep 5

echo ""
echo "=========================================="
echo "Checking Gazebo connectivity..."
echo "=========================================="
gz world list || echo "Gazebo not ready yet..."

echo ""
echo "=========================================="
echo "Starting Gazebo GUI..."
echo "=========================================="
gz sim -v4 -g &
GZ_GUI_PID=$!
echo "Gazebo GUI PID: $GZ_GUI_PID"
sleep 3

echo ""
echo "=========================================="
echo "Starting ArduCopter SITL..."
echo "=========================================="
cd "$ROS2_WS"
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
  command:=arducopter \
  model:=json \
  instance:=0 \
  synthetic_clock:=True \
  defaults:="$ROS2_WS/install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/copter.parm,$ROS2_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/config/gazebo-iris-gimbal.parm,$ROS2_WS/install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/dds_udp.parm,$ROS2_WS/install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/dds_use_ns.parm" \
  sim_address:=127.0.0.1 \
  micro_ros_agent_ns:=iris \
  transport:=udp4 \
  port:=2019 &
SITL_PID=$!
echo "ArduCopter SITL PID: $SITL_PID"
sleep 10

echo ""
echo "=========================================="
echo "Publishing robot description..."
echo "=========================================="
SDF_FILE="$ROS2_WS/install/ardupilot_gazebo/share/ardupilot_gazebo/models/iris_with_gimbal/model.sdf"
ros2 param set /robot_description "$(cat "$SDF_FILE")" &
PARAM_PID=$!
sleep 2

echo ""
echo "=========================================="
echo "Starting robot_state_publisher..."
echo "=========================================="
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -r __ns:=/iris \
  -p robot_description:="$(cat "$SDF_FILE")" &
RSP_PID=$!
echo "robot_state_publisher PID: $RSP_PID"
sleep 2

echo ""
echo "=========================================="
echo "Starting parameter bridge..."
echo "=========================================="
BRIDGE_CONFIG="$ROS2_WS/install/ardupilot_gz_bringup/share/ardupilot_gz_bringup/config/iris_bridge.yaml"
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -r __ns:=/iris \
  -p config_file:="$BRIDGE_CONFIG" &
BRIDGE_PID=$!
echo "parameter_bridge PID: $BRIDGE_PID"
sleep 3

echo ""
echo "=========================================="
echo "Spawning model..."
echo "=========================================="
ros2 run ros_gz_sim create \
  --ros-args -r __ns:=/iris \
  -p robot_description:="$(cat "$SDF_FILE")" \
  -- \
  -world "" \
  -param robot_description \
  -name iris \
  -x 0.0 -y 0.0 -z 0.2 \
  -R 0.0 -P 0.0 -Y 1.5707963267948966 &
CREATE_PID=$!
echo "create (spawn model) PID: $CREATE_PID"
sleep 5

echo ""
echo "=========================================="
echo "Checking ROS2 topics..."
echo "=========================================="
echo "All topics:"
ros2 topic list | head -20
echo ""
echo "Iris topics:"
ros2 topic list | grep iris || echo "No /iris topics found!"
echo ""
echo "IMU test (3 seconds):"
timeout 3 ros2 topic echo /iris/imu 2>/dev/null | head -3 || echo "No IMU data received"

echo ""
echo "Waiting... (Ctrl+C to stop)"
wait
