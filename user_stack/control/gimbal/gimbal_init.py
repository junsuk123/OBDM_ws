#!/usr/bin/env python3
"""Set initial gimbal orientation using gz service set_pose via CLI.

This sets the gimbal link pose so the camera optical axis points downward
relative to the vehicle. It uses the `gz` CLI; ensure `gz` is in PATH.
"""
import argparse
import math
import subprocess
import sys


def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="runway")
    parser.add_argument("--model-name", default="iris_with_gimbal")
    parser.add_argument("--link-name", default="gimbal_link")
    parser.add_argument("--pitch-deg", type=float, default=-90.0)
    parser.add_argument("--roll-deg", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    args = parser.parse_args()

    roll = math.radians(args.roll_deg)
    pitch = math.radians(args.pitch_deg)
    yaw = math.radians(args.yaw_deg)
    qw, qx, qy, qz = euler_to_quat(roll, pitch, yaw)

    # Entity name: model::link
    name = f"{args.model_name}::{args.link_name}"
    payload = (
        f'name: "{name}" '
        f'position {{x: 0.0 y: 0.0 z: 0.0}} '
        f'orientation {{w: {qw:.6f} x: {qx:.6f} y: {qy:.6f} z: {qz:.6f}}}'
    )
    service = f"/world/{args.world}/set_pose"
    try:
        result = subprocess.run([
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            payload,
        ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=False)
        ok = result.returncode == 0 and "data: true" in result.stdout
        print(f"set_pose returned: ok={ok}, stdout={result.stdout.strip()}, stderr={result.stderr.strip()}")
        if not ok:
            sys.exit(1)
    except Exception as e:
        print(f"Exception calling gz service: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
