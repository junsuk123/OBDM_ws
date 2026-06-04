#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

echo "[check_workspace] ROS packages under src:"
find src -maxdepth 2 -name package.xml -print | sort

echo
echo "[check_workspace] Python syntax check:"
python3 -m py_compile \
  src/landing_ontology_reasoner/landing_ontology_reasoner/ontology_reasoner_node.py \
  src/landing_path_generator/landing_path_generator/path_generator_node.py \
  src/landing_safety_layer/landing_safety_layer/safety_layer_node.py \
  src/landing_controller/landing_controller/controller_node.py \
  src/landing_state_estimation/landing_state_estimation/state_estimator_node.py \
  src/landing_perception/landing_perception/perception_adapter_node.py

echo "[check_workspace] OK"
