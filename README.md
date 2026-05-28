# OBDM_ws

Reusable UAV simulation workspace for ROS 2 Humble, Gazebo Harmonic, RViz, ArduPilot SITL, MATLAB, and Python user algorithms.

The current integrated scenario is ontology-based precision landing on a moving ArUco pad under wind disturbance. It launches Gazebo/ArduPilot, starts the ROS 2 helper nodes, visualizes camera/trajectory/wind data in RViz, stops on touchdown, and writes MATLAB metrics and plots.

## Quick Start

```bash
cd /home/j/SynologyDrive/junsuk/devel/INCSL/OBDM_ws
source ./setup_env.sh
./run_sim.sh --scenario ontology_landing --user_stack ontology_landing
```

Compatibility scenario aliases are available while the full directory migration is staged:

```bash
./run_sim.sh --scenario moving_pad_landing --user_stack ontology_landing
./run_sim.sh --scenario wind_disturbance --user_stack ontology_landing
```

## Current Primary Entry Point

The active integrated entry point is:

```text
user_stack/decision/ontology_landing/matlab/main.m
```

Older scripts such as `run_integrated_test.sh`, `scripts/system/run_simulation.sh`, `matlab/main/integrated_main.m`, and `matlab/demo/test_auto_launch.m` remain useful for legacy/manual diagnostics, but they are not the current ontology landing scenario entry.

## Current Layout

```text
ardu_ws/        External ROS 2 colcon workspace for ArduPilot/Gazebo packages
matlab/         MATLAB orchestration, modules, algorithms, tests, and ontology landing framework
scripts/        Diagnostics, maintenance, and system helper scripts
simulator/      Launch helpers, RViz configs, visualization nodes, and simulator-facing models
scenarios/      Scenario descriptors for the staged framework interface
config/         YAML config seeds for simulation and evaluation
tests/          MATLAB/Python smoke tests
tools/          External code-generation/build tools
docs/           Architecture, runbook, dependency map, migration, and cleanup reports
archive/        Holding area for confirmed unused candidates
```

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Runbook](docs/RUNBOOK.md)
- [Interface Spec](docs/INTERFACE_SPEC.md)
- [Scenario Guide](docs/SCENARIO_GUIDE.md)
- [User Algorithm Guide](docs/USER_ALGORITHM_GUIDE.md)
- [Dependency Map](docs/DEPENDENCY_MAP.md)
- [File Classification](docs/FILE_CLASSIFICATION.md)
- [File Migration Plan](docs/FILE_MIGRATION_PLAN.md)
- [Cleanup Report](docs/CLEANUP_REPORT.md)
- [Target Directory Tree](docs/TARGET_DIRECTORY_TREE.md)

## Runtime Outputs

Runtime files must not be mixed with source. Generated ArduPilot and Gazebo files such as `*.tlog`, `*.tlog.raw`, `*.BIN`, `eeprom.bin`, `LASTLOG.TXT`, `dumpcore*`, `dumpstack*`, `build/`, `install/`, `log/`, and MATLAB result folders are ignored by `.gitignore`. Confirmed cleanup candidates are listed in [docs/CLEANUP_REPORT.md](docs/CLEANUP_REPORT.md).
