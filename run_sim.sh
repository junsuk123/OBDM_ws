#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCENARIO="ontology_landing"
USER_STACK="ontology_landing"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --scenario)
      SCENARIO="${2:?missing value for --scenario}"
      shift 2
      ;;
    --user_stack|--user-stack)
      USER_STACK="${2:?missing value for --user_stack}"
      shift 2
      ;;
    --help|-h)
      cat <<'USAGE'
Usage:
  ./run_sim.sh --scenario ontology_landing --user_stack ontology_landing

Current implemented scenario:
  ontology_landing

Compatibility aliases:
  moving_pad_landing and wind_disturbance currently route through the
  ontology_landing MATLAB entry because that is the integrated scenario
  using moving pad, wind, ArUco perception, RViz, and evaluation.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

case "${SCENARIO}:${USER_STACK}" in
  ontology_landing:ontology_landing|moving_pad_landing:ontology_landing|wind_disturbance:ontology_landing)
    ;;
  *)
    echo "Unsupported combination: scenario=${SCENARIO}, user_stack=${USER_STACK}" >&2
    echo "See scenarios/*/scenario.yaml and docs/SCENARIO_GUIDE.md." >&2
    exit 2
    ;;
esac

cd "${REPO_ROOT}"
export OBDM_SCENARIO="${SCENARIO}"
export OBDM_USER_STACK="${USER_STACK}"
exec matlab -batch "run('user_stack/decision/ontology_landing/matlab/main.m')"
