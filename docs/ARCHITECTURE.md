# Architecture

## Runtime Stack

- Gazebo Sim 8 / Harmonic world and models live under `ardu_ws/src`.
- ROS 2 launch is provided by `ardupilot_gz_bringup`.
- ArduPilot SITL is controlled from MATLAB through MAVLink fallback helpers.
- MATLAB modules in `matlab/modules` coordinate simulation startup, data logging, wind control, target motion, visualization, and landing control.
- Python helper nodes in `scripts` bridge Gazebo wind/airspeed data and publish the moving ArUco landing target state.

## Active MATLAB Entry Points

- `matlab/main/integrated_main.m`: main integration run.
- `matlab/config/SimulationConfig.m`: repo-local paths and launch configuration.
- `matlab/modules/SimulationManager.m`: Gazebo / ROS 2 launch orchestration.
- `matlab/modules/ArucoLandingControlModule.m`: ArUco search, tracking, waypoint generation, landing sequence.
- `matlab/modules/AdverseEnvironmentModule.m`: wind scenario updates.
- `matlab/modules/MovingLandingTargetModule.m`: launches the moving target helper.
- `matlab/modules/DataLogger.m`: trajectory, wind, and target CSV logging.
- `matlab/modules/RealtimeVisualizer.m`: live trajectory plot with wind vector overlay.

## Data Flow

```text
Gazebo world wind
  -> Gazebo /world/runway/wind
  -> scripts/gz_virtual_wind_sensor_to_ros2.py
  -> /iris/wind_sensor
  -> MATLAB DataLogger / RealtimeVisualizer

Moving ArUco target
  -> scripts/aruco_landing_target_node.py
  -> /landing_marker/odom
  -> MATLAB logger and landing controller

Drone state
  -> /iris/odometry, /iris/navsat, /iris/imu, /iris/air_speed
  -> MATLAB logger, visualizer, controller
```

## Coordinate Convention

MATLAB path generation, logging, plotting, and landing target estimates use one world frame:

- `x`: Gazebo / ROS world X
- `y`: Gazebo / ROS world Y
- `z`: altitude up

Only the MAVLink command boundary converts this to ArduPilot `LOCAL_NED`:

- `north = world_y`
- `east = world_x`
- `down = -world_z`

The camera detector uses OpenCV optical coordinates internally (`x` right, `y` down, `z` forward), then converts the marker estimate back into the same world frame before publishing `/landing_marker/odom`.

The drone camera is configured as a fixed front-down camera. The detector assumes a 55 degree downward pitch when converting ArUco pose estimates to world coordinates.
