#!/usr/bin/env python3
"""Publish RViz paths for generated guidance setpoints and actual drone motion."""

import argparse
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy


def finite3(values):
    return all(math.isfinite(float(v)) for v in values)


class GuidancePathVisualizer:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("obdm_guidance_path_visualizer")
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.generated_path = Path()
        self.generated_path.header.frame_id = args.fixed_frame
        self.actual_path = Path()
        self.actual_path.header.frame_id = args.fixed_frame
        self.origin = None
        self.z_offset = 0.0
        self.last_generated = None
        self.last_actual = None
        self.generated_pub = self.node.create_publisher(Path, args.generated_path_topic, 10)
        self.actual_pub = self.node.create_publisher(Path, args.actual_path_topic, 10)
        self.target_pub = self.node.create_publisher(PoseStamped, args.current_target_topic, 10)
        self.node.create_subscription(PoseStamped, args.setpoint_topic, self.on_setpoint, 10)
        self.node.create_subscription(PoseStamped, args.origin_topic, self.on_origin, 10)
        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, qos)
        self.node.get_logger().info(
            f"guidance path visualizer: setpoints={args.setpoint_topic}, origin={args.origin_topic}, "
            f"odom={args.odom_topic}, frame={args.fixed_frame}"
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.origin is None:
            self.origin = (float(p.x), float(p.y))
            self.z_offset = max(0.0, float(p.z) - self.args.takeoff_altitude)
            self.node.get_logger().info(
                f"path origin initialized x={self.origin[0]:.2f}, y={self.origin[1]:.2f}, z_offset={self.z_offset:.2f}"
            )
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.args.fixed_frame
        pose.pose = msg.pose.pose
        if self.should_append(self.last_actual, pose.pose.position):
            self.actual_path.poses.append(pose)
            self.actual_path.poses = self.actual_path.poses[-self.args.max_actual_points :]
            self.last_actual = (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            self.publish_path(self.actual_pub, self.actual_path)

    def on_origin(self, msg):
        p = msg.pose.position
        origin = (float(p.x), float(p.y))
        z_offset = float(p.z)
        if self.origin is None:
            self.node.get_logger().info(
                f"guidance origin received x={origin[0]:.2f}, y={origin[1]:.2f}, z_offset={z_offset:.2f}"
            )
        self.origin = origin
        self.z_offset = z_offset

    def on_setpoint(self, msg):
        north = float(msg.pose.position.x)
        east = float(msg.pose.position.y)
        z_up = float(msg.pose.position.z)
        if not finite3((north, east, z_up)):
            return
        if self.origin is None:
            ox, oy = 0.0, 0.0
            z_offset = 0.0
        else:
            ox, oy = self.origin
            z_offset = self.z_offset
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.args.fixed_frame
        # Project LOCAL_NED setpoint back into the Gazebo/ROS odom convention:
        # local north -> world y, local east -> world x, z_up -> z.
        pose.pose.position.x = ox + east
        pose.pose.position.y = oy + north
        pose.pose.position.z = z_offset + z_up
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)
        if self.should_append(self.last_generated, pose.pose.position):
            self.generated_path.poses.append(pose)
            self.generated_path.poses = self.generated_path.poses[-self.args.max_generated_points :]
            self.last_generated = (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            self.publish_path(self.generated_pub, self.generated_path)

    def should_append(self, last, position):
        if last is None:
            return True
        dx = float(position.x) - last[0]
        dy = float(position.y) - last[1]
        dz = float(position.z) - last[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) >= self.args.min_point_spacing

    def publish_path(self, publisher, path):
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = self.args.fixed_frame
        publisher.publish(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--setpoint-topic", default="/obdm/guidance_setpoint")
    parser.add_argument("--origin-topic", default="/obdm/guidance_origin")
    parser.add_argument("--odom-topic", default="/iris/odometry")
    parser.add_argument("--generated-path-topic", default="/obdm/generated_trajectory_path")
    parser.add_argument("--actual-path-topic", default="/obdm/drone_actual_path")
    parser.add_argument("--current-target-topic", default="/obdm/current_guidance_target")
    parser.add_argument("--fixed-frame", default="odom")
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--max-points", type=int, default=450)
    parser.add_argument("--max-generated-points", type=int, default=300)
    parser.add_argument("--max-actual-points", type=int, default=450)
    parser.add_argument("--min-point-spacing", type=float, default=0.03)
    args = parser.parse_args()

    rclpy.init()
    node = GuidancePathVisualizer(args)
    try:
        rclpy.spin(node.node)
    finally:
        node.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
