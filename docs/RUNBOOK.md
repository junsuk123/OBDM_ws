# Runbook

## Build

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/OBDM_ws/ardu_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Launch From MATLAB

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/OBDM_ws
matlab -batch "run('matlab/main/integrated_main.m')"
```

## Manual Wind Checks

```bash
gz topic -l | grep wind
gz topic -e -t /world/runway/wind
GZ_VERSION=harmonic gz topic -t "/world/runway/wind" -m gz.msgs.Wind -p "linear_velocity: {x: 3.0, y: 0.0, z: 0.0} enable_wind: true"
```

## ROS 2 Checks

```bash
source /opt/ros/humble/setup.bash
source ardu_ws/install/setup.bash
ros2 topic list | grep -E 'iris|landing_marker|wind'
ros2 topic echo /landing_marker/odom --once
ros2 topic echo /iris/wind_sensor --once
ros2 topic echo /iris/air_speed --once
```

## Runtime Artifacts

SITL and Gazebo produce `*.tlog`, `eeprom.bin`, `terrain/`, `logs/`, `build/`, `install/`, and `log/` outputs. These are ignored by git and can be deleted when cleaning the workspace.
