#!/usr/bin/env bash
set -euo pipefail

LOGDIR=/tmp/mavproxy_supervised_logs
mkdir -p "$LOGDIR"

MAVPROXY_CMD=(/usr/bin/python3 /home/j/.local/bin/mavproxy.py --out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --non-interactive)

echo "mavproxy supervisor started at $(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$LOGDIR/supervisor.log"

while true; do
  TS=$(date +%Y%m%d_%H%M%S)
  LOG="$LOGDIR/mavproxy_$TS.log"
  echo "Starting mavproxy at $(date +%Y-%m-%dT%H:%M:%S) -> $LOG" >> "$LOGDIR/supervisor.log"
  "${MAVPROXY_CMD[@]}" >> "$LOG" 2>&1 || true
  EXIT=$?
  echo "mavproxy exited with code $EXIT at $(date +%Y-%m-%dT%H:%M:%S)" >> "$LOGDIR/supervisor.log"
  # short backoff
  sleep 2
done
