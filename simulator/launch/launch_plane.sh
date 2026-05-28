#!/bin/bash
# Backward-compatible wrapper for the quadcopter launch script.
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
exec "$SCRIPT_DIR/launch_quadcopter.sh" "$@"
