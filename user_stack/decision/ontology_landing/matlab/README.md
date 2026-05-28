# Ontology-Based Probabilistic UAV Landing Framework

This folder contains a modular MATLAB template for ontology-based probabilistic landing decisions under wind disturbance and sensor uncertainty.

## Core idea

The framework combines noisy observations and ontology-derived semantic features, then uses a Gaussian Naive Bayes classifier to decide between `LAND` and `HOVER`.

The observation model is written as:

```text
x_obs(t) = x_true(t) + n(t)
```

where `n(t)` is zero-mean Gaussian noise.

Wind disturbance is represented as a stochastic process with:

```text
w(t) = w_mean(t) + w_gust(t) + w_turb(t)
```

The Gaussian Naive Bayes posterior is:

```text
p(y = c | x) proportional to p(y = c) * product_i N(x_i | mu_ci, sigma_ci^2)
```

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
  decision_model/
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
