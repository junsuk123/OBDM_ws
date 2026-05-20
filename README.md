# OBDM_ws

Gazebo Sim 8 / ROS 2 Humble / ArduPilot / MATLAB workspace for wind-disturbed drone landing experiments.

## Layout

```text
ardu_ws/        ROS 2 colcon workspace: ArduPilot, Gazebo worlds, models, launch files
matlab/         MATLAB mission orchestration, modules, config, verification helpers
scripts/        Runtime bridge/helper nodes used by MATLAB and ROS 2
simulation/     Legacy/manual simulation launch helpers
tests/          Manual MATLAB and Python smoke tests
tools/          External build/codegen tools
docs/           Short project notes
```

## Main Run

```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/OBDM_ws
matlab -batch "run('matlab/main/integrated_main.m')"
```

The default mission is the ArUco landing scenario. It launches the repo-local `ardu_ws`, starts Gazebo wind publishing, runs the moving landing target helper, logs trajectory/wind/target data, and performs a best-effort landing at the end.

## Useful Checks

```bash
source /opt/ros/humble/setup.bash
source ardu_ws/install/setup.bash
ros2 topic list
gz topic -l | grep wind
```

See [docs/RUNBOOK.md](docs/RUNBOOK.md) and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the compact operating notes.
