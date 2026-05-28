# Interface Spec

## Scenario Runner

Target command:

```bash
./run_sim.sh --scenario ontology_landing --user_stack ontology_landing
```

Current compatibility aliases:

```bash
./run_sim.sh --scenario moving_pad_landing --user_stack ontology_landing
./run_sim.sh --scenario wind_disturbance --user_stack ontology_landing
```

## Core ROS 2 Topics

| Topic | Type | Direction | Owner |
|---|---|---|---|
| `/iris/odometry` | `nav_msgs/msg/Odometry` | Simulator to user stack | ROS-Gazebo bridge |
| `/iris/camera/image` | `sensor_msgs/msg/Image` | Simulator to perception | ROS-Gazebo bridge |
| `/iris/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Simulator to perception | ROS-Gazebo bridge |
| `/iris/wind_world` | `geometry_msgs/msg/Vector3Stamped` | Bridge to visualization/user stack | `gz_virtual_wind_sensor_to_ros2.py` |
| `/iris/wind_sensor` | `geometry_msgs/msg/Vector3Stamped` | Bridge to visualization/user stack | `gz_virtual_wind_sensor_to_ros2.py` |
| `/iris/wind_arrow` | `visualization_msgs/msg/Marker` | Visualization | `wind_arrow_marker_node.py` |
| `/landing_marker/odom` | `nav_msgs/msg/Odometry` | Perception to guidance | `aruco_camera_detector_node.py` |
| `/landing_marker/visible` | `std_msgs/msg/Bool` | Perception to guidance | `aruco_camera_detector_node.py` |
| `/landing_marker/touchdown` | `std_msgs/msg/Bool` | Scenario termination | `aruco_landing_target_node.py` |
| `/landing_marker/touchdown_metrics` | `geometry_msgs/msg/Vector3Stamped` | Evaluation | `aruco_landing_target_node.py` |
| `/obdm/generated_trajectory_path` | `nav_msgs/msg/Path` or marker path | Visualization | `guidance_path_visualizer.py` |

## Gazebo Interfaces

| Interface | Type | Use |
|---|---|---|
| `/world/runway/wind` | `gz.msgs.Wind` | Wind command publication |
| `/world/runway/wind_info` | `gz.msgs.Empty -> gz.msgs.Wind` | Verify/read wind plugin endpoint |
| `/model/r1_rover/cmd_vel` | `geometry_msgs/msg/Twist` bridged to Gazebo | Landing pad rover motion |
| `/model/r1_rover/joint/motor_*/cmd_force` | `gz.msgs.Double` | Rover wheel force commands |

## Environment Variables

| Variable | Default | Use |
|---|---|---|
| `OBDM_WS` | repository root | Workspace root |
| `OBDM_ARDU_WS` | `OBDM_ws/ardu_ws` | ArduPilot colcon workspace |
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain |
| `ROS_LOCALHOST_ONLY` | `0` | ROS network visibility |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | ROS middleware |
| `GZ_VERSION` | `harmonic` | Gazebo CLI version |
| `OBDM_DISABLE_RVIZ` | unset | Disable RViz helper when `1` |
| `OBDM_ENABLE_MAVLINK_FALLBACK` | unset | Enable direct fallback MAVLink commands when `1` |

## Results Contract

Each integrated run should produce:

- `summary.txt`
- `metrics.csv`
- `landing_framework_results.mat`
- `altitude_attitude.png`
- `decision_timeline.png`
- `semantic_features.png`
- `wind_profile.png`
- optional `wind_bin_analysis.png`
- optional `confusion_matrix.png`
