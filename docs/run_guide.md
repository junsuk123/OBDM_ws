# Run Guide

## Existing Simulator

Use the existing workspace entry points first:

```bash
./run_user_stack.sh
```

or the MATLAB/integration flow already used by the project:

```bash
./run_integrated_test.sh
```

The MATLAB wrapper `matlab/ontology_landing_framework/main.m` forwards into `user_stack/decision/ontology_landing/matlab/main.m`.

## Build Modular Landing Stack

From the workspace root:

```bash
colcon build --symlink-install --packages-select \
  landing_interfaces \
  landing_perception \
  landing_state_estimation \
  landing_ontology_reasoner \
  landing_path_generator \
  landing_safety_layer \
  landing_controller \
  uav_sim_bringup
source install/setup.bash
```

Then launch:

```bash
ros2 launch uav_sim_bringup landing_stack.launch.py
```

## Convenience Helpers

```bash
./scripts/check_workspace.sh
./scripts/run_sim.sh scenarios/moving_platform_no_wind.yaml
./scripts/kill_sim.sh
```

`run_sim.sh` currently prints the recommended command sequence rather than hiding simulator startup inside a new wrapper. This keeps backward compatibility with the existing ArduPilot/Gazebo flow.

## Soft Reset Failures

Soft reset can fail in two places. First, the Gazebo reset service itself can fail or return an unexpected response. Second, the reset can succeed but the post-reset pose check can still fail if the models have not settled or the pose sample does not arrive in time.

The most common causes are:

missing or slow `/world/<world>/pose/info` samples during reset verification
model position drift larger than the configured tolerance after reset
Gazebo service timeout or control service failure
startup load that makes the pose settle window too short for the current machine

When this happens, the code falls back from soft reset to hard reset automatically. Check `/tmp/obdm_reset.log` for the exact `serviceOk`, `poseOk`, and failure message values. If the log says `No Gazebo pose sample`, increase the pose sample timeout or settle time; if it says a model distance exceeds tolerance, increase the reset tolerance or verify that the world is actually returning the model to the expected coordinates.

## Real-time Graph Visualization

To inspect live node-to-topic relationships during a running stack, use the runtime ROS graph viewer:

```bash
source .venv/bin/activate
python simulator/visualization/ros_graph_visualizer.py
```

The viewer draws ROS nodes on the left and topics on the right, with arrows showing publisher and subscriber relations. Use `--include-regex 'landing|iris|obdm'` to focus on the landing stack. It depends on `matplotlib` in the active Python environment.

For the MATLAB ontology landing flow, set `cfg.visualization.showOntologyGraph = true` and keep `cfg.visualization.visible = true` to open the live ontology relation plot from `user_stack/decision/ontology_landing/matlab/simulator_interface/send_decision_command.m`.

## Guidance XY Debugging

If the generated trajectory looks too short in RViz or the UAV barely moves in XY, inspect the active guidance logs:

```bash
python scripts/analyze_guidance_xy_logs.py \
  /tmp/obdm_mavlink_guidance_streamer.log \
  /tmp/obdm_realtime_ontology_trajectory_generator.log
```

Healthy ontology guidance should show nonzero `north/east` samples and nonzero XY radius. If the analyzer reports all XY samples as `0.000`, the MAVLink streamer is receiving vertical-only setpoints and the issue is upstream of vehicle control.

The integrated moving-pad run starts `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py` by default. This is the normal XY guidance publisher for `/obdm/guidance_setpoint`, and it evaluates the landing ontology inference values (`LandingIntent`, `OperationalRisk`, `TrackQuality`, `DescentAuthority`) before choosing lead time, XY step size, and descent rate. Disable it only for MATLAB-only trajectory debugging:

```bash
export OBDM_ENABLE_REALTIME_TRAJECTORY_GENERATOR=0
```

When trajectory generation is healthy, `/tmp/obdm_realtime_ontology_trajectory_generator.log` should contain `setpoint#... local_ned=(north,east,z)` lines with nonzero XY values and an `ontology=mode:...,intent:...,risk:...,track:...,step:...,descent:...` suffix. The mode should progress from `INTERCEPT` to `PAD_CAPTURE` and then `FINAL_DESCENT`; near the pad, `lead_time` should be close to `0.12` s and `step` should be small rather than staying at the global maximum.

The realtime trajectory generator predicts future rover pad motion only during intercept/capture, then shifts to damped pad-center landing during final descent. During final approach it reasons over marker position/velocity, UAV velocity/acceleration, altitude above pad, descent rate, relative distance, relative speed, and wind from `/iris/wind_world`. The terminal controller estimates touchdown-time pad and UAV positions, compensates for inertia and wind drift with a bounded correction, and uses that predicted touchdown error for center readiness. Close to the pad, the final aim point is constrained inside a small pad-center corridor so prediction and inertia compensation cannot create a lateral "magnetic push" around the landing pad. The integrated run commands the UAV through `/obdm/guidance_velocity` and `/obdm/guidance_accel`; `/obdm/guidance_setpoint` remains available for RViz/debugging and as a fallback. The ontology now also reasons about overshoot risk from relative distance, relative velocity, measured UAV acceleration, wind, drag, track quality, and provider confidence, then adapts velocity gain, damping, max speed, acceleration gain, acceleration damping, and max acceleration before each command is published. Relative distance and relative velocity also act directly inside the command equation: close high-relative-speed states increase damping, reduce desired command speed, and add braking acceleration opposite the drone-to-pad relative velocity. During far intercept, relative velocity no longer suppresses speed by itself; distance authority keeps the UAV fast enough to catch the rover. The command also adds a center-crossing velocity so the UAV can move through the landing-pad center instead of merely matching pad speed; the command speed floor is at least the moving pad speed plus crossing margin, and this crossing term fades when center and relative-speed readiness are satisfied for touchdown. This follows the same control split described by ArduPilot's ROS 2 interface: velocity control is separate from the high-level waypoint goal interface. The log line includes `intercept_lead`, `descent_center_time`, `center_error`, `rel_speed`, `aim_corridor`, `terminal_t`, `inertia_t`, `wind`, `cmd_vel`, `cmd_acc`, `cmd_limits`, `center_ready`, and `rel_ready`; safe center landing should drive `center_error`, `rel_speed`, `rel_dist_risk`, `rel_vel_risk`, and `overshoot` down before the mode reaches `LAND_READY`.

If the camera marker is lost during search/recovery and is not reacquired for 60 seconds, the realtime trajectory generator publishes `/obdm/guidance_land` to request LAND instead of searching indefinitely. The timeout resets whenever a usable camera marker is seen again, and the log reports `marker search timeout ... requesting LAND` when it fires.

When the camera cannot observe the marker, the realtime trajectory generator switches the landing-pad position provider from camera odometry to a noisy abstract rover-GPS estimate from `/landing_marker/ground_truth/odom`. This fallback is labeled `marker_source=rover_gps` in `/tmp/obdm_realtime_ontology_trajectory_generator.log`, uses configurable position/velocity noise, and lets the UAV keep tracking the moving charging pad until the camera marker is visible again. Camera detections remain primary and immediately replace the rover-GPS estimate when they return.

The moving-pad scenario models a charging/maintenance pad mounted on a road vehicle. The rover follows a car-like route: it drives mostly straight down the road, makes smooth lane changes, returns to the original lane, and does not loop back toward the start. The integrated MATLAB launch passes `--no-waypoint-loop` and a long waypoint route so the rover behaves like normal road traffic during the landing attempt.

The UAV guidance is tuned to be faster than the rover during intercept, but the mission objective is pad capture and touchdown, not continuous rover tracking. The MAVLink streamer sends XY velocity commands up to `1.8 m/s` with acceleration limiting, while the realtime trajectory generator uses faster intercept lead/step settings and then tightens to smooth current pad-center velocity and adaptive acceleration control for final descent. In `PAD_CAPTURE` and `FINAL_DESCENT`, `aim_corridor=(offset/limit)` in the generator log shows whether the controller is clipping an over-predicted aim point back toward the pad center. If the marker drops out while the drone is already close and low, the system commits to LAND instead of climbing back into search.

## MATLAB End-to-End Research Pipeline

The MATLAB ontology landing entry point can run data collection, training, and validation from one replay:

```bash
matlab -batch "run('matlab/ontology_landing_framework/main.m')"
```

Edit the top of `user_stack/decision/ontology_landing/matlab/main.m` before launching:

```matlab
MAIN_PIPELINE.collectData = true;
MAIN_PIPELINE.trainModel = true;
MAIN_PIPELINE.trainOntologyWeights = true;
MAIN_PIPELINE.useHistoricalTrainingData = true;
MAIN_PIPELINE.maxHistoricalRuns = 20;
MAIN_PIPELINE.validateModel = true;
```

Each stage can be disabled independently. When `collectData=false`, set `MAIN_PIPELINE.inputResultsMat` to a previous `landing_framework_results.mat`, or leave it empty to load the latest saved run automatically. When `useHistoricalTrainingData=true`, each new simulation run trains the Gaussian NB model and ontology weight artifact from the newly collected samples plus up to `maxHistoricalRuns` previous result files. Validation is still reported on the current run, and `summary.txt` includes previous-model accuracy on the current data plus the accuracy gain. The same top block contains the main tuning parameters applied to `cfg`, including scenario count, duration, pad motion, ontology thresholds, and final LAND readiness limits.

The run saves:

```text
landing_framework_results.mat
main_pipeline_config.mat
ontology_policy.mat
ontology_policy.json
summary.txt
metrics.csv
path_generation_metrics.csv
reinforcement_training_rewards.csv
```

`ontology_policy.json` is an interpretable warm-start artifact for learned semantic weights. It preserves the ontology graph and stores additive weights for `OperationalRisk`, `TrackQuality`, and `LandingIntent`, plus multiplicative exponents for `DescentAuthority`.

The ontology semantic vector explicitly includes the landing geometry and vehicle/platform state used for learning and guidance:

```text
relativeDistanceState
altitudeState
droneMotionStability
roverMotionStability
```

These are computed from UAV-to-pad horizontal distance, altitude above the pad, UAV velocity/attitude/descent stability, and landing-pad rover/platform velocity. They are appended to the semantic feature vector, included in the baseline score, and represented as nodes in `LandingOntologyTrajectoryGraph`.

Path-generation evaluation is saved with each run. `path_generation_metrics.csv` reports per-scenario tracking error, final error, path length, straight-line efficiency, generated target smoothness, command coverage, marker coverage, approach step size, navigation gain, track quality, risk, and timing to capture/final radius. The plotting stage also saves:

```text
path_generation_metric_dashboard.png
path_tracking_time_series.png
path_topdown_comparison.png
reinforcement_reward_smoothness.png
```

For reinforcement-learning experiments, `reinforcement_training_rewards.csv` is the smoothness-aware reward dataset. Each step includes the total reward plus separate tracking, smoothness, efficiency, coverage, track-quality, risk, and terminal reward components. The smoothness component penalizes large second differences in the generated target trajectory, so RL training is pushed toward paths that are both successful and dynamically gentle.

The MATLAB trajectory generator now writes a per-command trace to:

```text
/tmp/obdm_matlab_trajectory_debug.log
```

Use this when XY motion is missing. The key fields are `hasMarker`, `ekfInit`, `uavMock`, `obsMock`, `marker`, `ekf`, `target`, `local`, `range`, and `approachStep`. If `hasMarker=0` or `obsMock=1` persists, the ontology graph cannot generate the ArUco/pad-following trajectory and will stay in search/hold guidance.

Keep MATLAB and ROS on the same DDS middleware settings. The landing MATLAB helpers preserve `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, and `FASTRTPS_DEFAULT_PROFILES_FILE`; avoid launching MATLAB from a shell with different ROS middleware variables than the running Gazebo/ROS stack.

The MATLAB main script exposes the data-collection repeat count at the top:

```matlab
MAIN_TUNING.dataCollectionRepeats = 8;
MAIN_TUNING.visualizationVisible = true;
MAIN_TUNING.contactArmingAltitude = 1.50;
MAIN_TUNING.contactMinFlightTime = 3.00;
MAIN_TUNING.contactMinSamples = 10;
MAIN_TUNING.windDifficulty = 1.00;
MAIN_TUNING.windMeanVelocity = [3.0, 0.8, 0.0];
MAIN_TUNING.windGustStd = [1.6, 1.1, 0.4];
MAIN_TUNING.windMaxSpeed = 14.0;
MAIN_TUNING.windPublishPeriod = 0.5;
```

Each repeat performs a full simulator lifecycle. Gazebo, RViz, rover/pad helpers, guidance streaming, and trajectory visualization are stopped after the repeat and restarted for the next one.

`main.m` starts with `close all force`, so every new execution clears plots from the previous run before generating new result figures. With `MAIN_TUNING.visualizationVisible=true`, the saved result plots are also opened as MATLAB figure windows at the end of the run.

Touchdown/contact detection is delayed until both `MAIN_TUNING.contactMinFlightTime` and `MAIN_TUNING.contactMinSamples` are satisfied. The defaults are shorter and more permissive so the system can recognize a valid touchdown earlier and stop the vehicle sooner after pad contact, while still avoiding stale Gazebo/ROS touchdown data at `t=0`.

During integrated runs, the Gazebo GUI is launched with:

```text
simulator/gazebo/iris_follow_gui.config
```

This enables the Gazebo `CameraTracking` plugin with `follow_target=/iris`, so the 3D view follows the Iris drone model during flight. The MATLAB helper also sends a follow command after startup in case the GUI loads before the model spawns. Set `OBDM_GZ_FOLLOW_CAMERA=0` before launching MATLAB to use the stock Gazebo camera behavior instead.

Marker-search altitude is capped at the initial takeoff altitude. This prevents marker-loss recovery from climbing above the camera's useful ArUco resolution range. The realtime trajectory generator logs both the requested search altitude and the applied ceiling at startup; the MAVLink streamer also clamps outgoing guidance setpoints to the takeoff altitude.

`MAIN_TUNING.windDifficulty` scales the generated wind before it is published to Gazebo WindEffects through `/world/<world>/wind`. Use `0.0` for near-calm collection, `1.0` for the configured baseline, and values above `1.0` for harsher wind scenarios.

Touch/contact termination is now split into two cases after takeoff. Contact detection is armed only after the UAV has climbed above `MAIN_TUNING.contactArmingAltitude`, which prevents the normal initial climb from being misclassified as ground contact. Successful landing-pad touchdown is gated by the Gazebo contact sensor on the rover pad collision (`/landing_pad/contact`) and published through `/landing_marker/touchdown`; altitude/XY thresholds are kept as a ground-contact failure fallback. Set `OBDM_ALLOW_TOUCHDOWN_THRESHOLD_FALLBACK=1` only when you explicitly want to allow the old pad-threshold success behavior without Gazebo contact.

- Landing-pad contact: treated as a successful landing, immediately requests vehicle disarm, stops pad motion, and cleans up the simulator processes.
- Ground contact away from the pad: treated as a failed landing, immediately requests vehicle disarm, stops pad motion, and cleans up the simulator processes.



## Domain-neutral Ontology Visualization

The reusable ontology graph viewer is under `ontology_viz/`. It accepts a domain-neutral JSON snapshot with nodes, edges, inferred facts, selection state, and optional reasoning traces. The included demo graph now uses the landing ontology names from `LandingOntologyTrajectoryGraph`, such as `VehicleState`, `PadTrack`, `OperationalRisk`, `TrackQuality`, `LandingIntent`, `DescentAuthority`, and `FinalLandReadiness`.

Validate a snapshot:

```bash
python ontology_viz/backend/schema_validation.py ontology_viz/frontend/data/demo_graph.json
```

Open the static browser UI:

```bash
cd ontology_viz/frontend
python -m http.server 8080
```

Then open `http://127.0.0.1:8080`. The page loads the static demo once and starts `Auto Live` by default. If the live backend is not running yet, the browser keeps waiting and reconnects automatically when `http://127.0.0.1:8765/events` becomes available.

Serve snapshots through the lightweight backend adapter:

```bash
python ontology_viz/backend/graph_provider.py \
  --snapshot ontology_viz/frontend/data/demo_graph.json \
  --port 8765
```

Use `http://127.0.0.1:8765/snapshot` for REST polling or `http://127.0.0.1:8765/events` for a live Server-Sent Events stream. Runtime logs can be stored as JSONL in:

```text
data/runtime/ontology_snapshots.jsonl
data/runtime/ontology_events.jsonl
```

To test real-time updates without a project backend, run the landing ontology demo JSONL writer and serve that log:

```bash
python ontology_viz/tools/write_demo_stream.py --truncate
python ontology_viz/backend/graph_provider.py \
  --snapshot data/runtime/ontology_snapshots.jsonl \
  --port 8765
```

Open `http://127.0.0.1:8080`. The graph updates as new snapshots are appended without pressing `Live`; the `Auto Live` button is only needed to manually disconnect or reconnect. Press `Poll` to refresh the matching `/snapshot` URL once per second instead.

Analyze a snapshot or JSONL log:

```bash
python ontology_viz/tools/analyze_ontology_logs.py ontology_viz/frontend/data/demo_graph.json
```

## Assumptions

- Existing ArduPilot/Gazebo/MATLAB launch paths remain the source of truth for simulation startup.
- Existing marker topics can be bridged from `/landing_marker/visible` and `/landing_marker/pose`.
- Existing wind data is available on `/iris/wind_sensor`.
- The baseline controller publishes `/cmd_vel`; change `src/landing_controller/config/controller.yaml` for MAVROS setpoint topics.

## Missing or Future Work

- Full NMPC-CBF optimization is not implemented yet.
- Obstacle distance is represented in the safety architecture but not yet connected to perception.
- Scenario YAML files define test intent; they are not yet a full scenario orchestrator.
- Formal OWL/RDF ontology support is deferred until the lightweight rule layer is validated.
