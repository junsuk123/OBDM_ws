#!/usr/bin/env bash
# Simple wrapper to run mavproxy and log received signals and exit codes.
LOGDIR=/tmp/mavproxy_wrapper_logs
mkdir -p "$LOGDIR"
LOG="$LOGDIR/wrapper.log"
echo "----- wrapper start $(date -u +%Y-%m-%dT%H:%M:%SZ) -----" >> "$LOG"

MAVPY=/home/j/.local/bin/mavproxy.py
ARGS=(--out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --non-interactive)

trap 'echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) - wrapper received SIGTERM/INT/QUIT" >> "$LOG"; if [ -n "${child:-}" ]; then echo "Killing child $child" >> "$LOG"; kill -TERM "$child" 2>/dev/null || true; fi; exit 0' TERM INT QUIT
trap 'echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) - wrapper received SIGUSR1/USR2" >> "$LOG"' USR1 USR2

echo "Starting: $MAVPY ${ARGS[*]}" >> "$LOG"
"/usr/bin/python3" "$MAVPY" "${ARGS[@]}" >> "$LOG" 2>&1 &
child=$!
echo "child pid=$child" >> "$LOG"

wait $child
RC=$?
echo "$(date -u +%Y-%m-%dT%H:%M:%SZ) - mavproxy exited with code ${RC}" >> "$LOG"
echo "----- wrapper end $(date -u +%Y-%m-%dT%H:%M:%SZ) -----" >> "$LOG"

exit $RC
