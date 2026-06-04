#!/usr/bin/env python3
"""Summarize OBDM guidance XY movement from text logs.

Usage:
  python scripts/analyze_guidance_xy_logs.py \
    /tmp/obdm_mavlink_guidance_streamer.log \
    /tmp/obdm_realtime_ontology_trajectory_generator.log
"""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path


PATTERNS = [
    re.compile(r"setpoint LOCAL_NED north=([-+0-9.eE]+), east=([-+0-9.eE]+), z_up=([-+0-9.eE]+)"),
    re.compile(r"stream target LOCAL_NED north=([-+0-9.eE]+), east=([-+0-9.eE]+), .*?cmd_z_up=([-+0-9.eE]+)"),
    re.compile(r"local_ned=\(([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\)"),
]


def iter_points(path: Path):
    if not path.exists():
        return
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            for pattern in PATTERNS:
                match = pattern.search(line)
                if match:
                    north, east, z_up = (float(match.group(i)) for i in range(1, 4))
                    yield north, east, z_up
                    break


def summarize(path: Path) -> None:
    points = list(iter_points(path))
    print(f"\n{path}")
    if not points:
        print("  no guidance setpoints found")
        return

    xy = [math.hypot(north, east) for north, east, _ in points]
    deltas = [
        math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        for i in range(1, len(points))
    ]
    nonzero = sum(1 for value in xy if value >= 0.05)
    print(f"  samples: {len(points)}")
    print(f"  nonzero xy samples >= 0.05m: {nonzero}")
    print(f"  xy radius min/mean/max: {min(xy):.3f} / {sum(xy) / len(xy):.3f} / {max(xy):.3f} m")
    if deltas:
        print(f"  xy step mean/max: {sum(deltas) / len(deltas):.3f} / {max(deltas):.3f} m")
    print(f"  first: north={points[0][0]:.3f}, east={points[0][1]:.3f}, z_up={points[0][2]:.3f}")
    print(f"  last:  north={points[-1][0]:.3f}, east={points[-1][1]:.3f}, z_up={points[-1][2]:.3f}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze XY movement in OBDM guidance logs.")
    parser.add_argument("paths", nargs="+", type=Path)
    args = parser.parse_args()
    for path in args.paths:
        summarize(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
