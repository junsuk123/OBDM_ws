# Architecture

## Design Rule

The simulator core should be reusable and stable. User algorithms should be replaceable. Scenarios should be config-driven. Runtime outputs should stay out of source directories.

## Current Integrated Runtime

```text
user_stack/decision/ontology_landing/matlab/main.m
  -> config/default_config.m
  -> simulator_interface/initSimulator.m
  -> simulator_interface/initialize_landing_simulation.m
  -> matlab/core/SimulationManager.m
  -> ros2 launch ardupilot_gz_bringup iris_runway.launch.py
  -> matlab/modules/guidance/MovingLandingTargetModule.m
  -> role-based Python helper nodes under user_stack/, bridges/, and simulator/
  -> ontology feature fusion, decision model, labeling, evaluation, plots
```

The integrated scenario currently combines simulator core, bridge layer, and user algorithm stack through MATLAB path lookup. This works, but the refactor target is to separate those roles.

## Current Component Boundaries

Simulator core:

- `ardu_ws/src/ardupilot_gz`: Gazebo worlds, models, bringup launch files.
- `simulator/rviz/ruiz.rviz`: RViz visualization configuration used by the integrated landing run.
- `matlab/core/SimulationManager.m`, `ROS2Manager.m`, `ProcessManager.m`: runtime orchestration and cleanup.

Bridge layer:

- `ros_gz_bridge parameter_bridge` launched from `MovingLandingTargetModule.m`.
- `bridges/ros_gz/gz_virtual_wind_sensor_to_ros2.py`: Gazebo wind and vehicle velocity to ROS 2 wind topics.
- `bridges/mavlink/mavlink_guidance_streamer.py`: ROS 2 guidance target to MAVLink control path.
- `user_stack/decision/ontology_landing/matlab/simulator_interface/*`: MATLAB to ROS 2/Gazebo/MAVLink adapter.

User algorithm stack:

- `user_stack/perception/aruco/aruco_camera_detector_node.py`: ArUco perception and inferred pad-center estimate.
- `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py`: landing trajectory generation.
- `user_stack/decision/ontology_landing/matlab/ontology_features`, `labeling`, `evaluation`: ontology landing decision stack.

Evaluation and logging:

- `user_stack/decision/ontology_landing/matlab/evaluation`
- `user_stack/decision/ontology_landing/matlab/visualization`
- per-run outputs under `user_stack/decision/ontology_landing/matlab/results/<timestamp>/`

## Target Boundary

```text
simulator/   Gazebo, models, launch, descriptions, RViz, reusable visualization
bridges/     ROS-Gazebo, MAVLink, ArduPilot, MATLAB adapters
user_stack/  Replaceable perception, decision, guidance, and control algorithms
scenarios/   YAML scenario definitions
config/      Vehicle, sensor, environment, autopilot, user-stack, and evaluation config
results/     Logs, rosbags, ArduPilot logs, MATLAB logs, plots, reports, runtime
```

The staged plan keeps the existing runtime paths intact first, then migrates file groups only after dependency references are updated.
