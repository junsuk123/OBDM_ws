#!/usr/bin/env python3
"""Parse -p ros2 topic echo logs and suggest kp_xy and max_xy_rate_mps2.

Usage: python3 scripts/analyze_trace_and_suggest_tuning.py <trace_dir>
Expects files: landing_state_p.log, reference_pose_p.log, cmd_vel_p.log
"""
import sys
import ast
import os
import math
from statistics import median


def iter_msgs(path):
    if not os.path.exists(path):
        return
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # ros2 -p prints python literal per message; try parse
            try:
                obj = ast.literal_eval(line)
                yield obj
            except Exception:
                continue


def extract_posexy(msg):
    # flexible extraction for PoseStamped-like dict
    try:
        pose = msg['pose'] if 'pose' in msg else msg.get('data', {}).get('pose', {})
        pos = pose['position']
        return float(pos['x']), float(pos['y'])
    except Exception:
        # try nested header/pose
        try:
            return float(msg['pose']['position']['x']), float(msg['pose']['position']['y'])
        except Exception:
            return None


def extract_cmd(msg):
    try:
        lin = msg['linear']
        return float(lin.get('x', 0.0)), float(lin.get('y', 0.0))
    except Exception:
        try:
            # nested geometry
            return float(msg['twist']['linear']['x']), float(msg['twist']['linear']['y'])
        except Exception:
            return None


def extract_state_xy(msg):
    # landing_state message structure varies; look for uav_pose
    try:
        up = msg['uav_pose']
        pos = up['pose']['position']
        return float(pos['x']), float(pos['y'])
    except Exception:
        # fallback to top-level pose
        return None


def main(trace_dir):
    ref_path = os.path.join(trace_dir, 'reference_pose_p.log')
    state_path = os.path.join(trace_dir, 'landing_state_p.log')
    cmd_path = os.path.join(trace_dir, 'cmd_vel_p.log')

    refs = list(iter_msgs(ref_path))
    states = list(iter_msgs(state_path))
    cmds = list(iter_msgs(cmd_path))

    # make tuples of timeseries by simple index alignment (best-effort)
    n = min(len(refs), len(states), len(cmds))
    if n == 0:
        print('No aligned messages found. Ensure logs exist and contain -p output.')
        return

    kps = []
    deltas = []
    rate_estimates = []

    prev_cmd_x = None
    prev_cmd_y = None
    # assume uniform dt ~ total_duration / n if header stamp not available
    for i in range(n):
        r = refs[i]
        s = states[i]
        c = cmds[i]
        rxy = extract_posexy(r.get('pose', r)) or extract_posexy(r)
        sxy = extract_state_xy(s)
        if rxy is None or sxy is None:
            continue
        rx, ry = rxy
        sx, sy = sxy
        err_x = rx - sx
        err_y = ry - sy
        cmd = extract_cmd(c)
        if cmd is None:
            continue
        cx, cy = cmd
        # estimate kp per sample where error not tiny
        if abs(err_x) > 1e-3:
            kps.append(cx / err_x)
        if abs(err_y) > 1e-3:
            kps.append(cy / err_y)
        if prev_cmd_x is not None:
            # approximate dt unknown; use index step as proxy
            rate_estimates.append(abs(cx - prev_cmd_x))
            rate_estimates.append(abs(cy - prev_cmd_y))
        prev_cmd_x = cx; prev_cmd_y = cy

    if not kps:
        print('Could not estimate kp from data (no valid samples).')
    else:
        kp_med = median(kps)
        kp_90 = sorted(kps)[int(len(kps)*0.9)-1] if len(kps) >= 10 else max(kps)
        print(f'Suggested kp_xy ~ median {kp_med:.3f}, 90pct {kp_90:.3f}')

    if not rate_estimates:
        print('Could not estimate rate (insufficient command samples).')
    else:
        # assume sample rate ~ 10Hz for conversion to m/s^2; we will ask user to provide dt if different
        median_delta = median(rate_estimates)
        suggested_rate = median_delta * 10.0
        print(f'Observed median per-step cmd change {median_delta:.3f} m/s; suggested max_xy_rate_mps2 ≈ {suggested_rate:.3f} (assuming 10Hz sampling).')

    print('\nNotes:')
    print('- This is a best-effort suggestion based on aligned -p logs. For accurate tuning, provide timestamps or CSV with time, ref_x, ref_y, uav_x, uav_y, cmd_x, cmd_y.')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: analyze_trace_and_suggest_tuning.py <trace_dir>')
        sys.exit(1)
    main(sys.argv[1])
