#!/usr/bin/env python3
"""Publish a virtual wind sensor from Gazebo wind and vehicle velocity.

Outputs:
  /<namespace>/wind_world   geometry_msgs/Vector3Stamped, world-frame wind
  /<namespace>/wind_sensor  geometry_msgs/Vector3Stamped, apparent wind

The apparent wind is approximated as:
  relative_wind_world = wind_world - vehicle_linear_velocity

This is a virtual sensor for data collection and visualization. It gives a
wind vector with direction, unlike a differential-pressure airspeed sensor.
"""

import argparse
import math
import re
import subprocess
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry


LINEAR_VELOCITY_RE = re.compile(r"linear_velocity\s*\{(?P<body>.*?)\}", re.S)
COMPONENT_RE = re.compile(r"\b(?P<axis>[xyz]):\s*(?P<value>[-+0-9.eE]+)")


def parse_wind_vector(text: str):
    match = LINEAR_VELOCITY_RE.search(text or "")
    if not match:
        return None

    values = {"x": 0.0, "y": 0.0, "z": 0.0}
    for comp in COMPONENT_RE.finditer(match.group("body")):
        try:
            values[comp.group("axis")] = float(comp.group("value"))
        except ValueError:
            return None

    vec = (values["x"], values["y"], values["z"])
    return vec if all(math.isfinite(v) for v in vec) else None


def read_world_wind(world: str, timeout_s: float):
    service = f"/world/{world}/wind_info"
    cmd = [
        "timeout",
        f"{timeout_s:.3f}",
        "gz",
        "service",
        "-s",
        service,
        "--reqtype",
        "gz.msgs.Empty",
        "--reptype",
        "gz.msgs.Wind",
        "--timeout",
        str(int(max(timeout_s * 1000.0, 1.0))),
        "--req",
        "",
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
    return parse_wind_vector(proc.stdout)


def make_msg(node, frame_id: str, vec):
    msg = Vector3Stamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.vector.x = float(vec[0])
    msg.vector.y = float(vec[1])
    msg.vector.z = float(vec[2])
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/iris")
    parser.add_argument("--world", default="runway")
    parser.add_argument("--robot", default="iris")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--gz-timeout", type=float, default=0.35)
    args = parser.parse_args()

    namespace = args.namespace.strip() or "/iris"
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    namespace = namespace.rstrip("/")

    odom_topic = f"{namespace}/odometry"
    wind_world_topic = f"{namespace}/wind_world"
    wind_sensor_topic = f"{namespace}/wind_sensor"

    rclpy.init()
    node = rclpy.create_node("gz_virtual_wind_sensor")
    world_pub = node.create_publisher(Vector3Stamped, wind_world_topic, 10)
    sensor_pub = node.create_publisher(Vector3Stamped, wind_sensor_topic, 10)
    velocity = {"x": 0.0, "y": 0.0, "z": 0.0}

    def odom_cb(msg: Odometry):
        velocity["x"] = float(msg.twist.twist.linear.x)
        velocity["y"] = float(msg.twist.twist.linear.y)
        velocity["z"] = float(msg.twist.twist.linear.z)

    node.create_subscription(Odometry, odom_topic, odom_cb, 10)
    period_s = 1.0 / max(args.rate, 0.1)

    node.get_logger().info(
        f"Publishing virtual wind sensor from /world/{args.world}/wind_info "
        f"and {odom_topic}"
    )

    try:
        while rclpy.ok():
            wind = read_world_wind(args.world, args.gz_timeout)
            if wind is not None:
                rel = (
                    wind[0] - velocity["x"],
                    wind[1] - velocity["y"],
                    wind[2] - velocity["z"],
                )
                try:
                    world_pub.publish(make_msg(node, "world", wind))
                    sensor_pub.publish(make_msg(node, f"{args.robot}/base_link", rel))
                except Exception:
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
