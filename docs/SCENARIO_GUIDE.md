# Scenario Guide

## Current Scenarios

| Scenario | YAML | Status |
|---|---|---|
| `ontology_landing` | `scenarios/ontology_landing/scenario.yaml` | Implemented via `user_stack/decision/ontology_landing/matlab/main.m` |
| `moving_pad_landing` | `scenarios/moving_pad_landing/scenario.yaml` | Alias/staged descriptor |
| `wind_disturbance` | `scenarios/wind_disturbance/scenario.yaml` | Alias/staged descriptor |

## Run

```bash
./run_sim.sh --scenario ontology_landing --user_stack ontology_landing
```

## Scenario YAML Shape

```yaml
name: ontology_landing
entrypoint: user_stack/decision/ontology_landing/matlab/main.m
user_stack: ontology_landing
world: runway
vehicle:
  model: iris
  namespace: /iris
landing_pad:
  model: r1_rover
wind:
  gazebo_topic: /world/runway/wind
evaluation:
  stop_on_touchdown: true
```

## Adding a Scenario

1. Create `scenarios/<name>/scenario.yaml`.
2. Declare the world, vehicle namespace, required bridges, user stack, and evaluation outputs.
3. Add any scenario-specific configs beside the YAML, such as `wind_profile.yaml` or `rover_motion.yaml`.
4. Extend `run_sim.sh` only after the scenario entry point exists and has a validation command.

The target design is config-driven. MATLAB hard-coded values should gradually move into `config/` and `scenarios/` YAML files, then be loaded by a small configuration adapter.
