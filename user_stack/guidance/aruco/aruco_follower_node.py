#!/usr/bin/env python3
"""Follower node: average marker poses, follow, search when lost, predict near landing.

Publishes velocity commands to `/cmd_vel` by default and a goal pose to
`/landing_pad/goal` for higher-level controllers.
"""
import argparse
import collections
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class ArucoFollower:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("aruco_follower")

        self.marker_sub = self.node.create_subscription(
            PoseStamped, args.marker_pose_topic, self.on_marker_pose, 10
        )
        self.visible_sub = self.node.create_subscription(
            Bool, args.marker_visible_topic, self.on_visible, 10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, args.drone_odom_topic, self.on_drone_odom, 10
        )

        self.cmd_pub = self.node.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.goal_pub = self.node.create_publisher(PoseStamped, "/landing_pad/goal", 10)

        self.alpha = args.alpha
        self.avg_x = None
        self.avg_y = None
        self.avg_z = None
        self.last_marker_t = None
        self.last_marker_vx = 0.0
        self.last_marker_vy = 0.0
        self.last_marker_vz = 0.0
        self.visible = False
        self.drone = None

        self.timer = self.node.create_timer(1.0 / args.rate, self.step)

    def on_marker_pose(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        t = self.node.get_clock().now().nanoseconds * 1e-9
        if self.avg_x is None:
            self.avg_x, self.avg_y, self.avg_z = x, y, z
        else:
            self.avg_x = (1.0 - self.alpha) * self.avg_x + self.alpha * x
            self.avg_y = (1.0 - self.alpha) * self.avg_y + self.alpha * y
            self.avg_z = (1.0 - self.alpha) * self.avg_z + self.alpha * z

        if self.last_marker_t is not None:
            dt = max(1e-6, t - self.last_marker_t)
            self.last_marker_vx = (x - self.prev_x) / dt
            self.last_marker_vy = (y - self.prev_y) / dt
            self.last_marker_vz = (z - self.prev_z) / dt

        self.prev_x, self.prev_y, self.prev_z = x, y, z
        self.last_marker_t = t

    def on_visible(self, msg: Bool):
        self.visible = bool(msg.data)

    def on_drone_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.drone = {"x": p.x, "y": p.y, "z": p.z, "vx": v.x, "vy": v.y, "vz": v.z}

    def step(self):
        if self.avg_x is None or self.drone is None:
            return

        target_x = self.avg_x
        target_y = self.avg_y
        target_z = self.avg_z

        # If marker not visible recently, search near averaged position
        now = time.time()
        if not self.visible and self.last_marker_t is not None and now - self.last_marker_t > self.args.lost_timeout:
            # simple search: small circular sweep around averaged point
            phase = (now % self.args.search_period) / self.args.search_period
            radius = self.args.search_radius
            target_x = self.avg_x + radius * math.cos(2.0 * math.pi * phase)
            target_y = self.avg_y + radius * math.sin(2.0 * math.pi * phase)

        # Near landing: if marker missing, use predicted position from last velocity
        if not self.visible and self.last_marker_t is not None:
            dt = now - self.last_marker_t
            if dt < self.args.prediction_max_dt:
                target_x = target_x + self.last_marker_vx * dt
                target_y = target_y + self.last_marker_vy * dt
                target_z = target_z + self.last_marker_vz * dt

        # Publish goal pose
        goal = PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = "odom"
        goal.pose.position.x = float(target_x)
        goal.pose.position.y = float(target_y)
        goal.pose.position.z = float(target_z)
        self.goal_pub.publish(goal)

        # Simple proportional velocity controller towards goal in world frame
        dx = target_x - self.drone["x"]
        dy = target_y - self.drone["y"]
        dz = target_z - self.drone["z"]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < 0.05:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # scale speed
        speed = min(self.args.max_speed, self.args.kp * dist)
        vx = speed * dx / max(dist, 1e-6)
        vy = speed * dy / max(dist, 1e-6)
        vz = speed * dz / max(dist, 1e-6)

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.linear.z = float(vz)
        self.cmd_pub.publish(cmd)

    def destroy(self):
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--marker-pose-topic", default="/landing_marker/pose")
    parser.add_argument("--marker-visible-topic", default="/landing_marker/visible")
    parser.add_argument("--drone-odom-topic", default="/iris/odometry")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--alpha", type=float, default=0.2)
    parser.add_argument("--kp", type=float, default=0.8)
    parser.add_argument("--max-speed", type=float, default=1.0)
    parser.add_argument("--lost-timeout", type=float, default=1.0)
    parser.add_argument("--search-period", type=float, default=8.0)
    parser.add_argument("--search-radius", type=float, default=1.5)
    parser.add_argument("--prediction-max-dt", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = ArucoFollower(args)
    try:
        rclpy.spin(node.node)
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
