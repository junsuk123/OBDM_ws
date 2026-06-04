# UAV Landing Research Framework Architecture

This workspace now has an additive modular research stack under `src/` while preserving the existing simulator, MATLAB, Gazebo, ArduPilot, RViz, and user-stack scripts.

## Current Working Entry Points

The existing system is still driven by:

- `run_user_stack.sh`
- `run_integrated_test.sh`
- `matlab/ontology_landing_framework/main.m`
- `user_stack/decision/ontology_landing/matlab/main.m`
- `matlab/main/integrated_main.m`
- `simulator/launch/*.sh`
- `simulator/launch/*.launch.py`
- `simulator/rviz/ruiz.rviz`

The existing moving-pad and ArUco support is primarily in:

- `matlab/modules/guidance/MovingLandingTargetModule.m`
- `matlab/modules/control/ArucoLandingControlModule.m`
- `user_stack/perception/aruco/aruco_camera_detector_node.py`
- `user_stack/perception/aruco/aruco_landing_target_node.py`
- `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py`
- `simulator/visualization/guidance_path_visualizer.py`
- `simulator/visualization/wind_arrow_marker_node.py`

## Added Modular Stack

The new ROS 2 packages are:

- `landing_interfaces`: shared `LandingState`, `LandingDecision`, and `SafetyStatus` messages.
- `landing_perception`: adapter from existing marker topics into `/landing/*`.
- `landing_state_estimation`: aggregates UAV, platform, marker, wind, and battery signals into `/landing/state`.
- `landing_ontology_reasoner`: YAML rule-based semantic decision layer.
- `landing_path_generator`: mode-adaptive reference pose/path generator.
- `landing_safety_layer`: CBF-ready safety gate and descent permission publisher.
- `landing_controller`: safety-gated PID baseline command interface, replaceable by NMPC-CBF.
- `uav_sim_bringup`: launch/config wrapper for the modular stack.
- `uav_worlds`: placeholder package for reusable worlds, models, and wind profiles.
- `moving_platform_sim`: placeholder package for reusable rover/pad assets.

## Runtime Data Flow

1. Existing ArduPilot/Gazebo/MATLAB stack publishes UAV, marker, rover, and wind data.
2. `landing_perception` republishes compatibility marker topics to `/landing/marker_visible`, `/landing/marker_confidence`, `/landing/marker_pose`, and `/landing/pad_pose`.
3. `landing_state_estimation` publishes `/landing/state`.
4. `landing_ontology_reasoner` maps raw numeric state into semantic state and publishes `/landing/decision`.
5. `landing_path_generator` selects a reference behavior from the semantic mode and publishes `/landing/reference_pose` and `/landing/reference_path`.
6. `landing_safety_layer` checks wind, attitude, marker reliability, platform speed, and semantic descent permission. It publishes `/landing/safety_status` and `/landing/allowed_to_descend`.
7. `landing_controller` publishes a safety-gated velocity command. This is a baseline interface for later NMPC-CBF.

## Topic Contracts

- `/landing/state`: unified vehicle/platform/environment state.
- `/landing/decision`: semantic landing decision.
- `/landing/reference_pose`: immediate tracking target.
- `/landing/reference_path`: simple path preview.
- `/landing/safety_status`: runtime safety gate.
- `/landing/allowed_to_descend`: Boolean descent permission.
- `/cmd_vel`: default command output for simulator compatibility.

## NMPC-CBF Extension Point

The current controller is intentionally a simple PID-style velocity generator. Replace `landing_controller/controller_node.py` with an NMPC implementation that consumes the same `/landing/state`, `/landing/reference_pose`, and `/landing/safety_status` topics. CBF constraints should live either in `landing_safety_layer` or inside the NMPC optimization as hard/soft constraints:

- obstacle distance
- rover body exclusion zone
- pad boundary
- max descent rate
- max horizontal speed near pad
- max attitude error
- marker confidence gate
- wind risk limit
