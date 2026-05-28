# Cleanup Report

This report separates runtime files from source files. The first refactor pass updates `.gitignore` and documents candidates, but does not delete source or uncertain files.

## Definitely Used Files

| File path | Reason |
|---|---|
| `user_stack/decision/ontology_landing/matlab/main.m` | Primary integrated scenario entry |
| `user_stack/decision/ontology_landing/matlab/config/default_config.m` | Entry config loader |
| `user_stack/decision/ontology_landing/matlab/config/landing_decision_config.m` | Scenario/user-stack parameters |
| `user_stack/decision/ontology_landing/matlab/simulator_interface/*.m` | Simulator, ROS 2, Gazebo, MAVLink adapter |
| `user_stack/decision/ontology_landing/matlab/ontology_features/*.m` | Ontology feature extraction |
| `user_stack/decision/ontology_landing/matlab/feature_fusion/*.m` | Feature fusion |
| `user_stack/decision/ontology_landing/matlab/decision_model/*.m` | Decision model and baseline |
| `user_stack/decision/ontology_landing/matlab/evaluation/*.m` | Metrics |
| `user_stack/decision/ontology_landing/matlab/visualization/*.m` | Plot generation |
| `matlab/core/SimulationManager.m` | Launches Gazebo/ArduPilot |
| `matlab/core/ROS2Manager.m` | Namespace/topic discovery |
| `matlab/core/ProcessManager.m` | Process cleanup |
| `matlab/modules/guidance/MovingLandingTargetModule.m` | Starts helper nodes, RViz, wind visualization |
| `matlab/core/SimulationConfig.m` | Shared repo path/topic helper |
| `user_stack/perception/aruco/aruco_landing_target_node.py` | Moving pad and touchdown publisher |
| `user_stack/perception/aruco/aruco_camera_detector_node.py` | ArUco detector and pad-center estimator |
| `simulator/models/rover/rover_cmd_vel_to_wheels.py` | Rover wheel force bridge |
| `simulator/visualization/iris_rviz_tf_publisher.py` | TF for RViz |
| `bridges/mavlink/mavlink_guidance_streamer.py` | MAVLink setpoint streamer |
| `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py` | Realtime landing trajectory |
| `simulator/visualization/guidance_path_visualizer.py` | RViz guidance path |
| `bridges/ros_gz/gz_virtual_wind_sensor_to_ros2.py` | Gazebo wind to ROS 2 |
| `simulator/visualization/wind_arrow_marker_node.py` | RViz wind arrow marker |
| `scripts/diagnostics/read_ros_bool_once.py` | Touchdown bool reader |
| `scripts/diagnostics/read_ros_vector3_once.py` | Touchdown metrics reader |
| `simulator/rviz/ruiz.rviz` | RViz config used by integrated run |

## Possibly Used or Legacy Files

| File path | Reason | Confidence level |
|---|---|---|
| `matlab/main/integrated_main.m` | Older integrated flow and wrappers still reference it | Medium |
| `matlab/main/integrated_example.m` | Example flow | Medium |
| `matlab/demo/test_auto_launch.m` | Auto-launch demo | Medium |
| `scripts/system/run_simulation.sh` | Manual launch helper | Medium |
| `run_integrated_test.sh` | Legacy wrapper for `matlab/main/integrated_main.m` | Medium |
| `verify_error_handling.sh` | Legacy smoke/diagnostic wrapper | Medium |
| `emergency_restart.sh` | Manual recovery script | Medium |
| `bridges/ros_gz/gz_airspeed_to_ros2.py` | Airspeed bridge, not in current ontology flow | Medium |
| `user_stack/guidance/aruco/aruco_follower_node.py` | ArUco guidance, not in current ontology flow | Medium |
| `user_stack/control/gimbal/gimbal_controller_node.py` | Gimbal control helper, not directly called by current flow | Medium |
| `user_stack/control/gimbal/gimbal_init.py` | Gimbal init helper, not directly called by current flow | Medium |
| `user_stack/control/rover/rover_back_and_forth.py` | Rover diagnostic/helper | Medium |
| `simulator/launch/*.py`, `simulator/launch/*.sh` | Manual/legacy launch experiments | Medium |
| `matlab/algorithms/*/*.m` | Algorithm examples | Medium |
| `matlab/tests/*.m`, `tests/matlab/*.m` | Diagnostics/tests | Medium |

## Runtime-Generated Files

| File path | Reason for removal/archive | Confidence level |
|---|---|---|
| `mav.tlog`, `mav.tlog.raw` | SITL telemetry log | High |
| `eeprom.bin` | SITL runtime state | High |
| `logs/*.BIN`, `matlab/logs/*.BIN`, `user_stack/decision/ontology_landing/matlab/logs/*.BIN` | ArduPilot binary logs | High |
| `LASTLOG.TXT` | ArduPilot log index | High |
| `scripts/__pycache__/`, `*.pyc` | Python bytecode cache | High |
| `*.asv` | MATLAB autosave | High |
| `dumpcore*`, `dumpstack*` | Crash dumps | High |
| `ardu_ws/build/`, `ardu_ws/install/`, `ardu_ws/log/` | colcon outputs | High |
| `user_stack/decision/ontology_landing/matlab/results/<timestamp>/` | MATLAB run outputs | High |

## Duplicated Parameter Files

| File path | Reason for manual confirmation | Confidence level |
|---|---|---|
| `mav.parm` | Duplicate-looking ArduPilot parameter file; check launch references before archive | Medium |
| `matlab/mav.parm` | Duplicate-looking parameter file | Medium |
| `matlab/main/mav.parm` | Legacy MATLAB flow may reference it | Medium |
| `matlab/modules/mav.parm` | Legacy module location | Medium |
| `archive/unused_candidates/duplicated_params/mav.parm` | Former manual simulation location; archived pending confirmation | Medium |
| `simulator/launch/mav.parm` | Manual launch location | Medium |
| `user_stack/decision/ontology_landing/matlab/mav.parm` | Runtime/current working directory artifact unless explicitly referenced | Medium |

Do not delete `*.parm` files solely by name. Confirm shell, MATLAB, Python, launch, docs, and command examples first.

## Archive Policy

Confirmed unused files should be moved to:

```text
archive/unused_candidates/
```

Runtime files should normally be regenerated, ignored, or moved under:

```text
results/runtime/
results/ardupilot_logs/
results/matlab_logs/
```
