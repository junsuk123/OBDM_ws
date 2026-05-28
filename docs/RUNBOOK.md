# Runbook

## Environment

```bash
cd /home/j/SynologyDrive/junsuk/devel/INCSL/OBDM_ws
source ./setup_env.sh
```

`setup_env.sh` sources ROS 2 Humble and `ardu_ws/install/setup.bash` when available, and exports:

- `OBDM_WS`
- `OBDM_ARDU_WS`
- `ROS_DOMAIN_ID=0`
- `ROS_LOCALHOST_ONLY=0`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `GZ_VERSION=harmonic`

## Integrated Ontology Landing Run

```bash
./run_sim.sh --scenario ontology_landing --user_stack ontology_landing
```

Equivalent MATLAB command:

```bash
matlab -batch "run('user_stack/decision/ontology_landing/matlab/main.m')"
```

## Manual Simulator Launch

The current integrated run uses `SimulationManager.m`, which launches:

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py \
  use_gz_sim_gui:=true rviz:=false use_gz_tf:=true \
  rviz_config:=simulator/rviz/ruiz.rviz
```

RViz is started later by `MovingLandingTargetModule.m` after camera, wind, and marker topics are available.

## Diagnostics

```bash
source ./setup_env.sh
ros2 topic list | grep -E 'iris|landing_marker|wind|obdm'
ros2 topic echo /landing_marker/odom --once
ros2 topic echo /landing_marker/touchdown --once --qos-durability transient_local
ros2 topic echo /iris/wind_sensor --once
ros2 topic echo /iris/wind_arrow --once
gz topic -l | grep wind
gz service -s /world/runway/wind_info --reqtype gz.msgs.Empty --reptype gz.msgs.Wind --timeout 2000 --req ''
```

## Cleanup

The integrated run stops helper nodes and simulator processes on touchdown. For manual cleanup:

```bash
pkill -TERM -f 'gz sim|ros_gz_bridge|parameter_bridge|micro_ros_agent|arducopter|arduplane|mavproxy|rviz2' || true
pkill -TERM -f 'aruco_landing_target_node.py|aruco_camera_detector_node.py|gz_virtual_wind_sensor_to_ros2.py|wind_arrow_marker_node.py|mavlink_guidance_streamer.py|realtime_ontology_trajectory_generator.py|guidance_path_visualizer.py|iris_rviz_tf_publisher.py|rover_cmd_vel_to_wheels.py' || true
```

## Results

Ontology landing results are currently written to:

```text
user_stack/decision/ontology_landing/matlab/results/<timestamp>/
```

Expected files include `summary.txt`, `metrics.csv`, `landing_framework_results.mat`, `altitude_attitude.png`, `decision_timeline.png`, `semantic_features.png`, `wind_profile.png`, `wind_bin_analysis.png`, and `confusion_matrix.png`.
