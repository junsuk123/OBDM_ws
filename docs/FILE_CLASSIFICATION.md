# File Classification

This classification is based on the current `user_stack/decision/ontology_landing/matlab/main.m` execution flow plus string references in MATLAB, Python, shell, launch, and documentation files.

## A. Simulator Core

- `ardu_ws/src/ardupilot_gz/ardupilot_gz_bringup`
- `ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/runway.sdf`
- `ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/models/aruco_landing_marker`
- `ardu_ws/src/ardupilot_gz/ardupilot_gazebo/models/iris_with_gimbal`
- `simulator/rviz/ruiz.rviz`
- `matlab/core/SimulationManager.m`
- `matlab/core/ROS2Manager.m`
- `matlab/core/ProcessManager.m`

## B. Bridge Layer

- `bridges/ros_gz/gz_virtual_wind_sensor_to_ros2.py`
- `bridges/ros_gz/gz_airspeed_to_ros2.py`
- `bridges/mavlink/mavlink_guidance_streamer.py`
- `bridges/mavlink/mavproxy_supervisor.sh`
- `bridges/mavlink/mavproxy_wrapper.sh`
- `user_stack/decision/ontology_landing/matlab/simulator_interface/*.m`
- ROS-Gazebo `parameter_bridge` command constructed by `MovingLandingTargetModule.m`

## C. User Algorithm Stack

- `user_stack/perception/aruco/aruco_camera_detector_node.py`
- `user_stack/perception/aruco/aruco_landing_target_node.py`
- `user_stack/guidance/aruco/aruco_follower_node.py`
- `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py`
- `user_stack/control/gimbal/gimbal_controller_node.py`
- `user_stack/control/gimbal/gimbal_init.py`
- `user_stack/decision/ontology_landing/matlab/ontology_features/*.m`
- `user_stack/decision/ontology_landing/matlab/labeling/*.m`

## D. Scenario Definitions

- `scenarios/ontology_landing/scenario.yaml`
- `scenarios/moving_pad_landing/scenario.yaml`
- `scenarios/wind_disturbance/scenario.yaml`
- `user_stack/decision/ontology_landing/matlab/config/landing_decision_config.m`
- `user_stack/decision/ontology_landing/matlab/config/default_config.m`

## E. Configuration

- `config/simulation/default.yaml`
- `config/evaluation/landing_thresholds.yaml`
- `matlab/core/SimulationConfig.m`
- ArduPilot defaults resolved from `ardupilot_sitl/config/default_params`
- Gazebo gimbal params resolved from `ardupilot_gazebo/config/gazebo-iris-gimbal.parm`

## F. Evaluation and Logging

- `user_stack/decision/ontology_landing/matlab/evaluation/*.m`
- `user_stack/decision/ontology_landing/matlab/visualization/*.m`
- `matlab/modules/logging/DataLogger.m`
- `matlab/modules/logging/DataLoggerModule.m`
- `matlab/modules/logging/DataCollectionModule.m`
- runtime outputs under `user_stack/decision/ontology_landing/matlab/results/<timestamp>/`

## G. Tests and Diagnostics

- `tests/matlab/*.m`
- `matlab/tests/*.m`
- `tests/python/test_simple_takeoff.py`
- `tests/python/test_rover_wheel_mapping.py`
- `tests/python/test_rover_wheel_mapping_with_echo.py`
- `scripts/diagnostics/diagnose_wind.sh`
- `scripts/diagnostics/read_ros_bool_once.py`
- `scripts/diagnostics/read_ros_vector3_once.py`
- `verify_error_handling.sh`
- `emergency_restart.sh`

## H. External Dependencies

- `ardu_ws/`
- `ardu_ws/src/ardupilot`
- `ardu_ws/src/ardupilot_gz`
- `ardu_ws/src/ardupilot_gazebo`
- `ardu_ws/src/ardupilot_sitl_models`
- `ardu_ws/src/ros_gz`
- `ardu_ws/src/micro_ros_agent`
- `tools/Micro-XRCE-DDS-Gen/`

## I. Unused or Temporary Files

Confirmed runtime-generated patterns:

- `*.tlog`
- `*.tlog.raw`
- `*.BIN`
- `eeprom.bin`
- `LASTLOG.TXT`
- `__pycache__/`
- `*.pyc`
- `*.asv`
- `dumpcore*`
- `dumpstack*`
- `build/`
- `install/`
- `log/`

Potentially duplicated but not auto-deleted:

- `mav.parm`
- `matlab/mav.parm`
- `matlab/main/mav.parm`
- `matlab/modules/mav.parm`
- `archive/unused_candidates/duplicated_params/mav.parm`
- `simulator/launch/mav.parm`
- `user_stack/decision/ontology_landing/matlab/mav.parm`
