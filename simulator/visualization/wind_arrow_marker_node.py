#!/usr/bin/env python3
"""Publish RViz particle markers from virtual wind vectors.

Subscribes:
  /<namespace>/wind_sensor (geometry_msgs/Vector3Stamped)

Publishes:
  /<namespace>/wind_arrow (visualization_msgs/Marker)
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import Point, Vector3Stamped
from std_msgs.msg import ColorRGBA
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
        self.particle_count = max(args.particle_count, 12)
        self.radius = max(args.radius, 0.01)
        self.alpha = min(1.0, max(args.alpha, 0.01))
        self.point_size = max(args.point_size, 0.01)

        self.node.get_logger().info(
            f"Publishing wind particle markers: {self.wind_topic} -> {self.marker_topic}"
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
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.points = []
        marker.colors = []

        basis_a, basis_b = self.orthonormal_basis(ux, uy, uz)
        count = self.particle_count
        swirl = 2.399963229728653
        for idx in range(count):
            t = 0.0 if count == 1 else idx / float(count - 1)
            angle = idx * swirl
            wake_radius = self.radius * (0.35 + 0.9 * t)
            radial = wake_radius * math.sin(math.pi * (0.2 + 0.8 * t))
            offset_a = math.cos(angle) * radial
            offset_b = math.sin(angle) * radial
            distance = length * t

            marker.points.append(
                Point(
                    x=ux * distance + basis_a[0] * offset_a + basis_b[0] * offset_b,
                    y=uy * distance + basis_a[1] * offset_a + basis_b[1] * offset_b,
                    z=uz * distance + basis_a[2] * offset_a + basis_b[2] * offset_b,
                )
            )
            marker.colors.append(self.rainbow_color(t, self.alpha * (1.0 - 0.55 * t)))

        marker.scale.x = self.point_size
        marker.scale.y = self.point_size
        marker.scale.z = self.point_size
        marker.color.a = self.alpha

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(1e9 / max(self.args.rate, 1.0))

        self.pub.publish(marker)

    @staticmethod
    def orthonormal_basis(ux, uy, uz):
        if abs(uz) < 0.9:
            ax, ay, az = -uy, ux, 0.0
        else:
            ax, ay, az = 0.0, -uz, uy
        norm_a = math.sqrt(ax * ax + ay * ay + az * az)
        ax, ay, az = ax / norm_a, ay / norm_a, az / norm_a
        bx = uy * az - uz * ay
        by = uz * ax - ux * az
        bz = ux * ay - uy * ax
        return (ax, ay, az), (bx, by, bz)

    @staticmethod
    def rainbow_color(t, alpha):
        phase = max(0.0, min(1.0, t)) * 0.82
        r = 0.5 + 0.5 * math.sin(2.0 * math.pi * (phase + 0.00))
        g = 0.5 + 0.5 * math.sin(2.0 * math.pi * (phase + 0.33))
        b = 0.5 + 0.5 * math.sin(2.0 * math.pi * (phase + 0.66))
        return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(alpha))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/iris")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--scale", type=float, default=0.6)
    parser.add_argument("--max-len", type=float, default=3.0)
    parser.add_argument("--min-len", type=float, default=0.15)
    parser.add_argument("--particle-count", type=int, default=96)
    parser.add_argument("--radius", type=float, default=0.32)
    parser.add_argument("--alpha", type=float, default=0.55)
    parser.add_argument("--point-size", type=float, default=0.075)
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
