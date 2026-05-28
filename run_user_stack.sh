#!/usr/bin/env bash
set -euo pipefail

STACK="${1:-ontology_landing}"

case "${STACK}" in
  ontology_landing)
    echo "The ontology landing stack is launched by user_stack/decision/ontology_landing/matlab/main.m."
    echo "Use: ./run_sim.sh --scenario ontology_landing --user_stack ontology_landing"
    ;;
  *)
    echo "Unknown user stack: ${STACK}" >&2
    exit 2
    ;;
esac
