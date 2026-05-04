# ROS2 + Gazebo Simulation Launch Troubleshooting Guide

## Overview
This guide explains how to properly launch the ArduPilot + Gazebo simulation using ROS2 with MATLAB integration.

## Official Reference
- ArduPilot ROS2 Documentation: https://ardupilot.org/dev/docs/ros2-gazebo.html

## Key Requirements

### Environment Setup
1. **ROS 2 Humble** - Must be installed and sourced
2. **Gazebo Harmonic** - Recommended version
3. **ArduPilot Gazebo Workspace** - Located at `~/ardu_ws`
4. **Environment Variables**:
   ```bash
   export GZ_VERSION=harmonic
   export ROS_DOMAIN_ID=0
   ```

### Required Files
- ROS2 workspace: `~/ardu_ws/install/setup.bash`
- Gazebo launch package: `ardupilot_gz_bringup`
- Launch script location: `<workspace>/simulation/launch/launch_plane.sh`

## Launch Command

### Direct Command
```bash
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
export GZ_VERSION=harmonic
export ROS_DOMAIN_ID=0
ros2 launch ardupilot_gz_bringup alti_transition_runway.launch.py use_gz_sim_gui:=true rviz:=false
```

### Via Script
The `simulation/launch/launch_plane.sh` script automates this process:
```bash
bash simulation/launch/launch_plane.sh
```

## MATLAB Integration

### SimulationManager Improvements
The MATLAB SimulationManager has been updated to:

1. **Create a wrapper shell script** to properly inherit environment variables
2. **Use nohup** for robust background process execution
3. **Properly capture PID** of the launch process
4. **Better error handling** when PID capture fails

### Expected Workflow

```
1. Initialize ROS2Manager → Detects namespace (e.g., /alti_transition)
2. Start SimulationManager → Launches simulation via wrapper script
3. Wait for Topics → Checks for IMU/TF topic availability
4. Subscribe to Topics → Creates ROS2 subscribers
5. Collect Data → Receives data from available topics
```

## Troubleshooting Steps

### Issue: "Simulation launched (PID: NaN)"
**Cause**: PID file not created or contains invalid data
**Solution**:
1. Check log file: `/tmp/obdm_plane_launch.log`
2. Verify wrapper script was created: `/tmp/obdm_plane_launch.log.wrapper`
3. Check ROS2 workspace exists: `ls ~/ardu_ws/install/setup.bash`

### Issue: "Topics not detected after 30 seconds"
**Cause**: Gazebo not running or namespace mismatch
**Solution**:
1. Verify Gazebo launched:
   ```bash
   ps aux | grep gz
   ```
2. Check published topics:
   ```bash
   ros2 topic list
   ```
3. Check detected namespace in MATLAB output
4. Verify ROS_DOMAIN_ID is consistent (default: 0)

### Issue: "Topic not found: /alti_transition/gps"
**Cause**: GPS topic not published by simulation
**Solution**:
- This is normal - not all simulations publish GPS
- Script continues with available topics (IMU, TF)
- Check available topics:
  ```bash
  ros2 topic list | grep alti_transition
  ```

## Log Files

### Location
- Main launch log: `/tmp/obdm_plane_launch.log`
- PID file: `/tmp/obdm_plane_launch.log.pid`
- Wrapper script: `/tmp/obdm_plane_launch.log.wrapper`

### Viewing Logs
```bash
tail -f /tmp/obdm_plane_launch.log
```

## Verification Steps

### 1. Environment Check
```matlab
setenv('ROS_DOMAIN_ID', '0');
system('echo $GZ_VERSION');
system('ls ~/ardu_ws/install/setup.bash');
```

### 2. Manual Launch Test
```bash
cd ~/OBDM_ws
bash simulation/launch/launch_plane.sh
# Wait 30 seconds, then Ctrl+C
ros2 topic list
```

### 3. MATLAB Integration Test
```matlab
cd ~/OBDM_ws/matlab
run('main/integrated_main.m')
```

## Performance Tips

1. **Gazebo UI**: Disable GUI for faster launch with `use_gz_sim_gui:=false`
2. **RViz**: Disable to reduce overhead with `rviz:=false`
3. **Wait Time**: Default 30 seconds; adjust if needed
4. **Domain ID**: Keep consistent across all terminals

## References

- [ArduPilot ROS2 + Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/ros2-sitl.html)
