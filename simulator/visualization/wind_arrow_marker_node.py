#!/usr/bin/env python3
"""Publish RViz arrow markers from virtual wind vectors.

Subscribes:
  /<namespace>/wind_sensor (geometry_msgs/Vector3Stamped)

Publishes:
  /<namespace>/wind_arrow (visualization_msgs/Marker)
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import Point, Vector3Stamped
from visualization_msgs.msg import Marker


class WindArrowMarkerNode:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("wind_arrow_marker")

        ns = args.namespace.strip() or "/iris"
        if not ns.startswith("/"):
            ns = "/" + ns
        ns = ns.rstrip("/")

        self.wind_topic = f"{ns}/wind_sensor"
        self.marker_topic = f"{ns}/wind_arrow"

        self.pub = self.node.create_publisher(Marker, self.marker_topic, 10)
        self.sub = self.node.create_subscription(
            Vector3Stamped, self.wind_topic, self.on_wind, 10
        )

        self.scale = max(args.scale, 0.01)
        self.max_len = max(args.max_len, 0.1)
        self.min_len = max(args.min_len, 0.01)

        self.node.get_logger().info(
            f"Publishing wind arrow markers: {self.wind_topic} -> {self.marker_topic}"
        )

    def on_wind(self, msg: Vector3Stamped):
        vx = float(msg.vector.x)
        vy = float(msg.vector.y)
        vz = float(msg.vector.z)

        mag = math.sqrt(vx * vx + vy * vy + vz * vz)
        if not math.isfinite(mag):
            return

        length = min(self.max_len, max(self.min_len, mag * self.scale))
        if mag > 1e-6:
            ux, uy, uz = vx / mag, vy / mag, vz / mag
        else:
            ux, uy, uz = 1.0, 0.0, 0.0

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id or self.args.default_frame
        marker.ns = "wind"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=ux * length, y=uy * length, z=uz * length),
        ]

        marker.scale.x = 0.06
        marker.scale.y = 0.12
        marker.scale.z = 0.16

        marker.color.r = 0.15
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.95

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(1e9 / max(self.args.rate, 1.0))

        self.pub.publish(marker)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/iris")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--scale", type=float, default=0.6)
    parser.add_argument("--max-len", type=float, default=3.0)
    parser.add_argument("--min-len", type=float, default=0.15)
    parser.add_argument("--default-frame", default="iris/base_link")
    args = parser.parse_args()

    rclpy.init()
    node = WindArrowMarkerNode(args)

    try:
        rclpy.spin(node.node)
    finally:
        node.node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
