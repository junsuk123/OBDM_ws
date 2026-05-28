# File Migration Plan

Status values:

- `active`: used by the current integrated run.
- `moved`: source has been moved and runtime references were updated.
- `candidate`: needs reference check or manual confirmation before moving.
- `runtime`: generated output; ignore or move under `results/`.

## Python and Shell

| Old path | New path | Reason | Status |
|---|---|---|---|
| `scripts/aruco_camera_detector_node.py` | `user_stack/perception/aruco/aruco_camera_detector_node.py` | ArUco perception and pad-center inference | active, moved |
| `scripts/aruco_landing_target_node.py` | `user_stack/perception/aruco/aruco_landing_target_node.py` | Moving landing target ground truth/touchdown publisher | active, moved |
| `scripts/aruco_follower_node.py` | `user_stack/guidance/aruco/aruco_follower_node.py` | ArUco guidance algorithm | moved |
| `scripts/realtime_ontology_trajectory_generator.py` | `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py` | Ontology landing trajectory generator | active, moved |
| `scripts/gimbal_controller_node.py` | `user_stack/control/gimbal/gimbal_controller_node.py` | Gimbal control | moved |
| `scripts/gimbal_init.py` | `user_stack/control/gimbal/gimbal_init.py` | Gimbal initialization | moved |
| `scripts/gz_airspeed_to_ros2.py` | `bridges/ros_gz/gz_airspeed_to_ros2.py` | Gazebo to ROS 2 airspeed bridge | moved |
| `scripts/gz_virtual_wind_sensor_to_ros2.py` | `bridges/ros_gz/gz_virtual_wind_sensor_to_ros2.py` | Gazebo wind bridge | active, moved |
| `scripts/mavlink_guidance_streamer.py` | `bridges/mavlink/mavlink_guidance_streamer.py` | MAVLink guidance bridge | active, moved |
| `scripts/mavproxy_supervisor.sh` | `bridges/mavlink/mavproxy_supervisor.sh` | MAVProxy supervisor | moved |
| `scripts/mavproxy_wrapper.sh` | `bridges/mavlink/mavproxy_wrapper.sh` | MAVProxy wrapper | moved |
| `scripts/rover_cmd_vel_to_wheels.py` | `simulator/models/rover/rover_cmd_vel_to_wheels.py` | Rover model actuation bridge | active, moved |
| `scripts/wind_arrow_marker_node.py` | `simulator/visualization/wind_arrow_marker_node.py` | RViz wind marker | active, moved |
| `scripts/guidance_path_visualizer.py` | `simulator/visualization/guidance_path_visualizer.py` | RViz trajectory marker/path | active, moved |
| `scripts/iris_rviz_tf_publisher.py` | `simulator/visualization/iris_rviz_tf_publisher.py` | RViz TF shim | active, moved |
| `scripts/diagnose_wind.sh` | `scripts/diagnostics/diagnose_wind.sh` | Wind diagnostics | moved |
| `scripts/read_ros_bool_once.py` | `scripts/diagnostics/read_ros_bool_once.py` | One-shot ROS 2 bool reader | active, moved |
| `scripts/read_ros_vector3_once.py` | `scripts/diagnostics/read_ros_vector3_once.py` | One-shot ROS 2 vector reader | active, moved |
| `scripts/test_rover_wheel_mapping.py` | `tests/python/test_rover_wheel_mapping.py` | Rover wheel test | moved |
| `scripts/test_rover_wheel_mapping_with_echo.py` | `tests/python/test_rover_wheel_mapping_with_echo.py` | Rover wheel test with echo | moved |

## MATLAB

| Old path | New path | Reason | Status |
|---|---|---|---|
| `matlab/modules/SimulationManager.m` | `matlab/core/SimulationManager.m` | Simulator lifecycle orchestration | active, moved |
| `matlab/modules/ROS2Manager.m` | `matlab/core/ROS2Manager.m` | ROS 2 namespace/topic helpers | active, moved |
| `matlab/modules/ProcessManager.m` | `matlab/core/ProcessManager.m` | Process cleanup | active, moved |
| `matlab/config/SimulationConfig.m` | `matlab/core/SimulationConfig.m` | Shared path/topic config | active, moved |
| `matlab/modules/MovingLandingTargetModule.m` | `matlab/modules/guidance/MovingLandingTargetModule.m` | Landing target runtime helpers | active, moved |
| `matlab/modules/environment/WindControlModule.m` | `matlab/modules/environment/WindControlModule.m` | Wind publishing module | candidate |
| `matlab/modules/environment/WindControlModuleAdapter.m` | `matlab/modules/environment/WindControlModuleAdapter.m` | Wind adapter | candidate |
| `matlab/modules/environment/AdverseEnvironmentModule.m` | `matlab/modules/environment/AdverseEnvironmentModule.m` | Environment disturbance module | candidate |
| `matlab/modules/control/ArucoLandingControlModule.m` | `matlab/modules/control/ArucoLandingControlModule.m` | Landing controller | legacy/candidate |
| `matlab/modules/logging/DataCollectionModule.m` | `matlab/modules/logging/DataCollectionModule.m` | Data collection | legacy/candidate |
| `matlab/modules/logging/DataLogger.m` | `matlab/modules/logging/DataLogger.m` | CSV logging | legacy/candidate |
| `matlab/modules/visualization/RealtimeVisualizer.m` | `matlab/modules/visualization/RealtimeVisualizer.m` | MATLAB visualization | legacy/candidate |
| `matlab/algorithms/circle_flight/circle_flight.m` | `matlab/algorithms/circle_flight/circle_flight.m` | Algorithm example | candidate |
| `matlab/algorithms/track_follow/track_follow_test.m` | `matlab/algorithms/track_follow/track_follow_test.m` | Algorithm example | candidate |
| `matlab/ontology_landing_framework/*` | `user_stack/decision/ontology_landing/matlab/*` | Replaceable ontology decision/guidance stack | active, moved |
| `matlab/tests/*` | `tests/matlab/*` | MATLAB diagnostics | candidate |

## Simulator Assets

| Old path | New path | Reason | Status |
|---|---|---|---|
| `simulation/rviz/ruiz.rviz` | `simulator/rviz/ruiz.rviz` | RViz config used by integrated run | active, moved |
| `simulation/launch/*` | `simulator/launch/*` | Manual launch helpers | moved |
| `ardu_ws/` | `external/ardu_ws/` | External colcon workspace | active, planned |
| `tools/Micro-XRCE-DDS-Gen/` | `tools/Micro-XRCE-DDS-Gen/` | External tooling | active as external dependency |

Active runtime references have been updated for the moved files. Legacy entry points remain present where practical, and `matlab/ontology_landing_framework/main.m` is now a compatibility wrapper.
