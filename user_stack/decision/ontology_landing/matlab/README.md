# Ontology-Based UAV Landing Framework

This folder contains a modular MATLAB template for ontology-based landing decisions under wind disturbance and sensor uncertainty.

## Core idea

The framework combines noisy observations and ontology-derived semantic features, then evaluates a semantic landing-intent score to decide between `LAND` and `HOVER`.

The observation model is written as:

```text
x_obs(t) = x_true(t) + n(t)
```

where `n(t)` is zero-mean Gaussian noise.

Wind disturbance is represented as a stochastic process with:

```text
w(t) = w_mean(t) + w_gust(t) + w_turb(t)
```

The ontology landing-intent evidence is a normalized weighted semantic score:

```text
LandingIntentEvidence =
  f(AlignmentState, RelativeDistanceState, AltitudeState,
    VisualStability, DescentStability, MotionStability,
    EnergyEfficiency, WindRisk, DragRisk)
```

For online guidance, the landing controller now evaluates one explicit ontology network:

```text
raw state / marker / wind evidence
  -> semantic nodes
     WindRisk, DragRisk, VisualStability, AlignmentState,
     AttitudeStability, DescentStability, TimeEfficiency, EnergyEfficiency
  -> inference nodes
     OperationalRisk, TrackQuality, LandingIntent
  -> trajectory nodes
     FuturePadPrediction, ApproachAimPoint, HorizontalGuidance,
     DescentAuthority, FinalLandReadiness
  -> command
     local setpoint or final LAND/DISARM
```

The graph is evaluated by `ontology_features/evaluate_landing_ontology_graph.m`.
`send_decision_command.m` consumes the graph values directly when generating
the landing trajectory, so lead time, future pad prediction, horizontal step,
descent rate, and final landing readiness all come from the same semantic
network instead of separate ad hoc guidance rules.

The realtime Python trajectory generator is the default guidance setpoint
publisher for the integrated Gazebo/ArduPilot run. The MATLAB ontology graph is
still evaluated and logged, but the realtime generator keeps the XY guidance
stream moving while the MATLAB graph trajectory path is being validated. Set
`OBDM_ENABLE_REALTIME_TRAJECTORY_GENERATOR=0` or module config
`useRealtimeTrajectoryGenerator=false` only for MATLAB-only trajectory debugging.

## Folder layout

```text
ontology_landing_framework/
  main.m
  README.md
  config/
  simulator_interface/
  stochastic_model/
  ontology_features/
  feature_fusion/
  labeling/
  evaluation/
  visualization/
  results/
```

## Run

From the MATLAB root:

```matlab
cd user_stack/decision/ontology_landing/matlab
main
```

The script writes metrics, figures, and logs into `results/<timestamp>/`.

## Optional auto launch

To mirror the `integrated_main` flow, set `OBDM_ENABLE_AUTO_SIM_EXECUTION=1` before running `main`. The project then attempts to start `SimulationManager`, detect the ROS 2 namespace, and wait for topics before continuing. If the launch path is unavailable, the framework falls back to the mock backend when fallback is enabled.
