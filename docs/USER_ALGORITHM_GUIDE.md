# User Algorithm Guide

## Replaceable Stack Boundary

User algorithms should consume simulator/bridge interfaces and publish guidance/control outputs without modifying Gazebo worlds, vehicle models, or core launch files.

Current ontology landing stack components:

- Perception: `user_stack/perception/aruco/aruco_camera_detector_node.py`
- Guidance: `user_stack/guidance/ontology_landing/realtime_ontology_trajectory_generator.py`
- Decision: `user_stack/decision/ontology_landing/matlab/ontology_features`, `feature_fusion`, `decision_model`, `labeling`
- Control bridge: `bridges/mavlink/mavlink_guidance_streamer.py`
- Evaluation: `user_stack/decision/ontology_landing/matlab/evaluation`, `visualization`

## Target Layout

```text
user_stack/
  perception/
    aruco/
  navigation/
  decision/
    ontology_landing/
      matlab/
      python/
      config/
      ontology_features/
      feature_fusion/
      decision_model/
      stochastic_model/
      labeling/
      evaluation/
      visualization/
  guidance/
    ontology_landing/
  control/
```

## Adding a User Stack

1. Create `user_stack/<role>/<stack_name>/`.
2. Define input topics and output topics in a stack config file.
3. Avoid hard-coded workspace paths. Use `OBDM_WS`, `OBDM_ARDU_WS`, and scenario config values.
4. Add a scenario binding under `scenarios/<scenario>/user_stack.yaml`.
5. Add validation steps to `docs/RUNBOOK.md`.

## Required Behavior

A user stack should:

- tolerate topic startup delay,
- log enough state for evaluation,
- shut down cleanly when the scenario ends,
- keep runtime outputs under `results/`,
- avoid modifying simulator core files.
