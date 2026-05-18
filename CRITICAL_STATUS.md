# 🚨 Critical Status Report - Gazebo Data Collection Failure

**Date**: 2026-05-08  
**Status**: ❌ SIMULATION CRASHED - DATA COLLECTION FAILED  
**Root Cause**: Gazebo process terminated (exit code -9 / SIGKILL after 5-15 seconds)

---

## Current Situation

### What Happened
The integrated MATLAB test ran but collected **0 messages** across all sensors:
```
[ 1/10] IMU: 0, ODOM: 0, TF: 0, BATTERY: 0
...
[10/10] IMU: 0, ODOM: 0, TF: 0, BATTERY: 0

Result: ALL FAILED (0 total messages)
```

### Why It Failed

**Root Cause Chain**:
1. Gazebo simulation was launched
2. After ~10 seconds, Gazebo process crashed (exit code -9 / SIGKILL)
3. When Gazebo crashed, supporting services stopped:
   - ❌ `parameter_bridge` (crashed/exited)
   - ❌ `micro_ros_agent` (crashed/exited)
   - ❌ IMU/ODOM sensor bridges (stopped)
4. Only `mavproxy.py` remained running (ArduPilot SITL layer)
5. MATLAB tried to collect data but found no sensor topics
6. All measurements returned 0 messages

### Process Status Verification

**Current Running Processes**:
```
✓ mavproxy.py      (PID: 1410885) - RUNNING
✗ gz sim           - NOT FOUND
✗ parameter_bridge - NOT FOUND
✗ micro_ros_agent  - NOT FOUND
```

**Available ROS2 Topics** (only partial):
```
/alti_transition/gz/tf
/alti_transition/joint_states
/alti_transition/robot_description
/alti_transition/tf
/alti_transition/tf_static
/clock
```

**Missing Sensor Topics** (should be publishing but aren't):
```
✗ /alti_transition/imu              (IMU data - MISSING)
✗ /alti_transition/odometry         (ODOM data - MISSING)
✗ /alti_transition/battery/state    (Battery - MISSING)
```

---

## Gazebo Crash Analysis

From previous diagnostics (analysis_report.md):

| Check | Result | Conclusion |
|-------|--------|-----------|
| Memory (OOM) | 23Gi available | ❌ NOT memory issue |
| Kernel kill | No entries in journalctl | ❌ NOT kernel OOM |
| Resource limits | All normal | ❌ NOT resource constraint |
| Thread issue | GZ_THREADS=1 still crashes | ❌ NOT threading issue |
| Simple SDF | Crashes even without plugins | ❌ NOT model-specific |

**Likely Causes**:
1. **Gazebo 8.11.0 Bug**: Ruby wrapper or internal crash during threading phase
2. **Environment Issue**: Container limits, watchdog timeout, or system constraints
3. **Launch Timeout**: ROS2 launch system forcibly killing gz sim after fixed duration

**Exit Code -9 Meaning**:
- `-9` = SIGKILL (forcible termination)
- Process was **externally killed**, not crashed internally
- Could be: launch timeout, watchdog process, OOM killer, or system timeout

---

## Recovery Action Plan

### Immediate (5 minutes)

**Run Emergency Recovery Script**:
```bash
chmod +x /home/j/SynologyDrive/INCSL/devel/INCSL/OBDM_ws/emergency_restart.sh
./emergency_restart.sh
```

This script will:
1. Clean up all crashed processes
2. Restart Gazebo with 12-second monitoring window
3. Check for sensor topics
4. Run quick MATLAB test
5. Log results for analysis

### If Emergency Script Fails

**Manual Diagnostics**:
```bash
# Check what topics are actually publishing
ros2 topic list

# Check Gazebo logs for crash reason
tail -f ~/.ros/log/*/gazebo*/

# Try raw gz sim command (should crash in 5-15 seconds)
timeout 20 gz sim -v1 -s -r /path/to/runway.sdf --force-version 8

# Check system resources during crash
watch -n 0.5 'ps aux | grep gz; free -h'
```

### If Gazebo Keeps Crashing

**Workaround Options**:
1. **Switch to ArduPilot Native ROS2** (no Gazebo):
   - Use `ardupilot_sitl sitl_dds_udp.launch.py`
   - Topics: `/ap/imu/experimental/data`, `/ap/pose/filtered`, etc.
   - Update `SimulationConfig.m` to use `/ap/` namespace

2. **Upgrade Gazebo**:
   - Current: Gazebo 8.11.0
   - Try: Gazebo 8.12 or Gazebo 9.x
   - Command: `sudo apt upgrade gazebo* --only-upgrade`

3. **Use Docker Container**:
   - Isolate environment variables
   - Different resource constraints
   - May fix timeout-related issue

4. **Run Gazebo Directly Without Launch System**:
   - Bypass ROS2 launch wrapper
   - Reduces external kill signals
   - Requires manual process management

---

## What To Expect If It Works

When Gazebo is stable and data bridges are working:

```
✓ Gazebo process running (no crash)
✓ parameter_bridge active
✓ micro_ros_agent active
✓ /alti_transition/imu publishing at 1-50 Hz
✓ /alti_transition/odometry publishing at 10 Hz
✓ /alti_transition/battery/state publishing at 2 Hz
✓ MATLAB collects 100+ messages in 10 seconds
```

---

## Files & References

**Recent Changes**:
- `model.sdf`: Added OdometryPublisher plugin
- `test_data_collection_enhanced.m`: Improved MATLAB test script
- `emergency_restart.sh`: Recovery automation script (THIS FILE)

**Key Diagnostic Files**:
- `/tmp/obdm_logs/`: Contains all recent logs
- `analysis_report.md`: Detailed crash analysis
- `INTEGRATION_STATUS.md`: System status and next steps

---

## Next Steps

### Immediate (Next 5 minutes)
1. ✅ Run emergency restart script
2. ✅ Check if Gazebo stays running for 12+ seconds
3. ✅ Verify sensor topics are published
4. ✅ Check MATLAB test results

### If Successful
- Document workaround (if any)
- Set up automated monitoring
- Prepare for production deployment

### If Still Failing
- Escalate Gazebo crash (file bug report or request upgrade)
- Implement workaround (switch to ArduPilot native ROS2 or other option)
- Consider Docker isolation

---

## Support Commands

```bash
# Monitor Gazebo in real-time
watch -n 1 'ps aux | grep "gz sim|parameter_bridge|mavproxy" | grep -v grep'

# Check Gazebo crash pattern
for i in {1..3}; do echo "=== Run $i ==="; timeout 20 gz sim -v1 -s -r /tmp/simple.sdf --force-version 8; echo "Exit: $?"; sleep 2; done

# View recent ROS2 logs
ls -lt ~/.ros/log | head -5

# Monitor ports for changes
watch -n 0.5 'netstat -tuln | grep -E "2019|5760|5501"'

# Memory/CPU during Gazebo launch
watch -n 0.1 'ps aux | grep gz; free -h; top -b -n 1 | head -20'
```

---

## Status Timeline

| Time | Event | Status |
|------|-------|--------|
| 09:06 | Gazebo launch in integrated_main.m | ✓ Started |
| ~09:16 | Gazebo process crash | ❌ Exit code -9 |
| ~09:17 | MATLAB test runs but finds no data | ❌ 0 messages |
| ~09:18 | Analysis identifies crash pattern | ℹ️ Documented |
| ~09:20 | Emergency recovery script created | ⏳ PENDING |

---

**Last Updated**: 2026-05-08 - URGENT STATUS  
**Action Required**: Run emergency recovery or implement workaround  
**Priority**: 🔴 CRITICAL - System non-functional  

