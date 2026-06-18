# Dependency Map

## Primary Entry Point

`user_stack/decision/ontology_landing/matlab/main.m` is the current primary integrated scenario entry point.

## Startup Sequence

```text
main.m
  addpath(genpath(user_stack/decision/ontology_landing/matlab))
  addpath(genpath(matlab))
  default_config()
    landing_decision_config()
  initSimulator()
    init_simulator_interface()
    initialize_landing_simulation()
      SimulationManager('verbose', cfg.runtime.verbose)
      SimulationManager.initialize()
        ProcessManager.cleanup_all_simulation()
        ROS2Manager.Initialize()
      SimulationManager.start_simulation()
        ros2 launch ardupilot_gz_bringup iris_runway.launch.py
      ROS2Manager.auto_detect_namespace()
```

## Scenario Loop

```text
main.m
  resetScenario()
  performTakeoff()
  startLandingPadMotion()
    MovingLandingTargetModule.start()
      user_stack/perception/aruco/aruco_landing_target_node.py
      simulator/models/rover/rover_cmd_vel_to_wheels.py
      simulator/visualization/iris_rviz_tf_publisher.py
      bridges/mavlink/mavlink_guidance_streamer.py
      user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py
      simulator/visualization/guidance_path_visualizer.py
      bridges/ros_gz/gz_virtual_wind_sensor_to_ros2.py
      simulator/visualization/wind_arrow_marker_node.py
      ros_gz_bridge parameter_bridge for camera/odom/imu/navsat/battery
      user_stack/perception/aruco/aruco_camera_detector_node.py
      rviz2 -d simulator/rviz/ruiz.rviz
  generate_wind_process()
  read_uav_state()
  read_wind_condition()
    gz service /world/runway/wind_info
    gz topic /world/runway/wind
  read_landing_observation()
  compute_ontology_features()
  ontology_landing_decision()
  evaluate_landing_ontology_graph()
  send_decision_command()
    ROS 2 guidance topics
    optional run_mavlink_command()
  is_touchdown_on_marker()
    scripts/diagnostics/read_ros_bool_once.py
    scripts/diagnostics/read_ros_vector3_once.py
  stopLandingPadMotion()
  shutdownSimulator()
```

## Required MATLAB Files

- `user_stack/decision/ontology_landing/matlab/main.m`
- `user_stack/decision/ontology_landing/matlab/config/default_config.m`
- `user_stack/decision/ontology_landing/matlab/config/landing_decision_config.m`
- `user_stack/decision/ontology_landing/matlab/simulator_interface/*.m`
- `user_stack/decision/ontology_landing/matlab/stochastic_model/*.m`
- `user_stack/decision/ontology_landing/matlab/ontology_features/*.m`
- `user_stack/decision/ontology_landing/matlab/labeling/*.m`
- `user_stack/decision/ontology_landing/matlab/evaluation/*.m`
- `user_stack/decision/ontology_landing/matlab/visualization/*.m`
- `matlab/core/SimulationManager.m`
- `matlab/core/ROS2Manager.m`
- `matlab/core/ProcessManager.m`
- `matlab/modules/guidance/MovingLandingTargetModule.m`
- `matlab/modules/ModuleTemplates/ModuleBase.m`
- `matlab/core/SimulationConfig.m`

## Required Python/Shell Runtime Helpers

- `user_stack/perception/aruco/aruco_landing_target_node.py`
- `user_stack/perception/aruco/aruco_camera_detector_node.py`
- `simulator/models/rover/rover_cmd_vel_to_wheels.py`
- `simulator/visualization/iris_rviz_tf_publisher.py`
- `bridges/mavlink/mavlink_guidance_streamer.py`
- `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py`
- `simulator/visualization/guidance_path_visualizer.py`
- `bridges/ros_gz/gz_virtual_wind_sensor_to_ros2.py`
- `simulator/visualization/wind_arrow_marker_node.py`
- `scripts/diagnostics/read_ros_bool_once.py`
- `scripts/diagnostics/read_ros_vector3_once.py`

## Launch, World, Model, and Visualization Dependencies

- `ardu_ws/install/setup.bash`
- `ardupilot_gz_bringup/iris_runway.launch.py`
- `ardupilot_gazebo/models/iris_with_gimbal/model.sdf`
- `ardupilot_gazebo/models/aruco_landing_marker/model.sdf`
- `ardupilot_gazebo/models/aruco_landing_marker/materials/textures/aruco_multi_scale_pad.png`
- `ardupilot_gazebo/worlds/runway.sdf`
- `ardupilot_gazebo/config/gazebo-iris-gimbal.parm`
- `ardupilot_sitl/config/default_params/copter.parm`
- `ardupilot_sitl/config/default_params/dds_udp.parm`
- `ardupilot_sitl/config/default_params/dds_use_ns.parm`
- `simulator/rviz/ruiz.rviz`

## Legacy or Diagnostic Entry Points

- `run_integrated_test.sh`: legacy MATLAB integrated run wrapper for `matlab/main/integrated_main.m`.
- `verify_error_handling.sh`: legacy error-handling smoke wrapper.
- `emergency_restart.sh`: manual recovery and sensor probe script.
- `scripts/system/run_simulation.sh`: interactive manual Gazebo/ArduPilot launch script.
- `matlab/main/integrated_main.m`: older integrated MATLAB flow.
- `matlab/demo/test_auto_launch.m`: MATLAB auto-launch demo.

These should remain until the new scenario runner is validated.
