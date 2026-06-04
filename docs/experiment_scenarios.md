# Experiment Scenarios

Scenario configs were added under `scenarios/`.

## Scenarios

- `static_platform_no_wind.yaml`: static platform, no wind, no obstacle.
- `moving_platform_no_wind.yaml`: moving rover platform, no wind.
- `moving_platform_constant_wind.yaml`: moving platform with constant crosswind.
- `moving_platform_gust_wind.yaml`: moving platform with gust wind.
- `moving_platform_marker_loss.yaml`: temporary marker loss.
- `moving_platform_obstacle_near_path.yaml`: obstacle near landing path.
- `high_wind_reject_direct.yaml`: direct descent should be rejected due to high wind.
- `marker_lost_low_altitude.yaml`: marker loss below 2 m should trigger ascend-and-research.

## Wind Profiles

Reusable wind profiles are in:

- `src/uav_worlds/wind/wind_profiles.yaml`

The current YAML files define intent and expected decisions. Hooking them into Gazebo wind publishing should be the next step. The existing wind path is already present through `WindControlModule` and the virtual wind sensor bridge.

## Expected Evaluation

For each scenario, record:

- selected semantic mode over time
- wind speed and marker visibility
- path/reference pose
- safety status and descent permission
- touchdown error and descent rate
- abort/research events

This creates the core data needed for comparing ontology-guided behavior against a baseline direct landing policy.
