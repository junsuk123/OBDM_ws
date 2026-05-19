#!/usr/bin/env python3
"""Relay Gazebo AirSpeed samples to a ROS 2 Vector3Stamped topic.

Gazebo Harmonic publishes gz.msgs.AirSpeed with differential pressure and
temperature. ros_gz_bridge does not provide the ardupilot_msgs/Airspeed
conversion used by this launch file, so MATLAB never sees /iris/air_speed.
This relay keeps the MATLAB side simple by publishing a forward airspeed
vector [tas, 0, 0] on /<namespace>/air_speed.
"""

import argparse
import math
import re
import subprocess
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped


DIFF_PRESSURE_RE = re.compile(r"diff_pressure:\s*([-+0-9.eE]+)")


def read_diff_pressure(topic: str, timeout_s: float):
    cmd = [
        "timeout",
        f"{timeout_s:.3f}",
        "gz",
        "topic",
        "-e",
        "-t",
        topic,
        "-n",
        "1",
    ]
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=timeout_s + 0.5,
        )
    except subprocess.SubprocessError:
        return None

    match = DIFF_PRESSURE_RE.search(proc.stdout or "")
    if not match:
        return None

    try:
        value = float(match.group(1))
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def diff_pressure_to_tas(diff_pressure: float, rho: float) -> float:
    # Sign depends on sensor orientation; magnitude is the useful airspeed.
    return math.sqrt(max(0.0, 2.0 * abs(diff_pressure) / rho))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/iris")
    parser.add_argument("--world", default="runway")
    parser.add_argument("--robot", default="iris")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--rho", type=float, default=1.225)
    parser.add_argument("--gz-timeout", type=float, default=0.35)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    namespace = args.namespace.strip() or "/iris"
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    namespace = namespace.rstrip("/")

    gz_topic = (
        f"/world/{args.world}/model/{args.robot}/link/base_link/"
        "sensor/air_speed_sensor/air_speed"
    )
    ros_topic = f"{namespace}/air_speed"

    rclpy.init()
    node = rclpy.create_node("gz_airspeed_to_ros2")
    pub = node.create_publisher(Vector3Stamped, ros_topic, 10)
    period_s = 1.0 / max(args.rate, 0.1)

    node.get_logger().info(f"Relaying {gz_topic} -> {ros_topic}")
    try:
        while rclpy.ok():
            diff_pressure = read_diff_pressure(gz_topic, args.gz_timeout)
            if diff_pressure is not None:
                msg = Vector3Stamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = f"{args.robot}/base_link"
                msg.vector.x = diff_pressure_to_tas(diff_pressure, args.rho)
                msg.vector.y = 0.0
                msg.vector.z = 0.0
                try:
                    pub.publish(msg)
                except Exception:
                    break
                if args.once:
                    break
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception:
                break
            time.sleep(period_s)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
