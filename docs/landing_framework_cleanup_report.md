# Cleanup Report

No files were deleted or moved. This report identifies candidates for later cleanup after the current simulator behavior is validated.

## Files Currently Used

Likely active files based on launch scripts, MATLAB wrappers, and topic references:

- `run_user_stack.sh`
- `run_integrated_test.sh`
- `setup_env.sh`
- `matlab/ontology_landing_framework/main.m`
- `user_stack/decision/ontology_landing/matlab/main.m`
- `matlab/main/integrated_main.m`
- `matlab/core/SimulationConfig.m`
- `matlab/core/SimulationManager.m`
- `matlab/modules/guidance/MovingLandingTargetModule.m`
- `matlab/modules/control/ArucoLandingControlModule.m`
- `matlab/modules/environment/WindControlModule.m`
- `matlab/modules/environment/AdverseEnvironmentModule.m`
- `matlab/modules/logging/DataLogger.m`
- `matlab/modules/logging/DataCollectionModule.m`
- `user_stack/perception/aruco/aruco_camera_detector_node.py`
- `user_stack/perception/aruco/aruco_landing_target_node.py`
- `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py`
- `simulator/launch/launch_quadcopter.sh`
- `simulator/launch/iris_runway_custom.launch.py`
- `simulator/launch/ardupilot/iris.launch.py`
- `simulator/rviz/ruiz.rviz`
- `simulator/visualization/guidance_path_visualizer.py`
- `simulator/visualization/wind_arrow_marker_node.py`
- `bridges/ros_gz/gz_virtual_wind_sensor_to_ros2.py`
- `bridges/ros_gz/gz_airspeed_to_ros2.py`

## Cleanup Candidates

Review before deleting:

- `matlab/main/integrated_main_backup.m`: backup copy; archive if no longer needed.
- `matlab/modules/CircleControlModule_DEBUG.m`: debug variant; archive after comparing with active module.
- `matlab/modules/SimulationManager.asv`: MATLAB autosave file.
- `matlab_ontology_landing_framework_main.m.bak`: backup wrapper.
- `matlab/**/logs/*.BIN`, `matlab/**/mav.tlog`, `matlab/**/mav.tlog.raw`, `matlab/**/eeprom.bin`: generated simulation logs/state. Archive outside source control.
- `simulator/visualization/__pycache__`: generated Python cache.
- Duplicate `mav.parm` files under root, `matlab/`, `matlab/main/`, `matlab/modules/`, and `simulator/launch/`: consolidate after confirming which one ArduPilot uses.
- Old scenario files in `scenarios/*/scenario.yaml`: keep until the new scenario naming is adopted.

## Suggested Refactor Strategy

1. Keep existing scripts stable as compatibility entry points.
2. Gradually move reusable ROS nodes into `src/*` packages.
3. Keep Gazebo/ArduPilot vendor/submodule code under `ardu_ws/src`.
4. Move generated logs and telemetry to `runs/` or `data/` with `.gitignore`.
5. After validation, replace ad hoc MATLAB/Python bridge launch logic with `uav_sim_bringup` launch files.
