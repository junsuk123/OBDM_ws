#!/usr/bin/env python3
"""Safety-gated PID baseline controller.

This node is intentionally simple. It provides the same command boundary an
NMPC-CBF implementation can replace later.
"""

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node

from landing_interfaces.msg import LandingState, SafetyStatus


class LandingControllerNode(Node):
    def __init__(self):
        super().__init__("landing_controller_node")
        default_config = (
            Path(get_package_share_directory("landing_controller"))
            / "config"
            / "controller.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        with Path(self.get_parameter("config_file").value).open("r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream).get("controller", {})
        self.kp_xy = float(cfg.get("kp_xy", 0.8))
        self.kp_z = float(cfg.get("kp_z", 0.6))
        self.max_xy = float(cfg.get("max_xy_speed_mps", 2.0))
        self.max_z = float(cfg.get("max_z_speed_mps", 0.7))
        # maximum allowed change in xy speed per second (m/s^2)
        self.max_xy_rate = float(cfg.get("max_xy_rate_mps2", 1.0))
        self.state = None
        self.reference = None
        self.safe = False
        self._prev_cmd = None
        self._prev_time = self.get_clock().now()
        self.cmd_pub = self.create_publisher(Twist, str(cfg.get("command_topic", "/cmd_vel")), 10)
        self.create_subscription(LandingState, "/landing/state", self.on_state, 10)
        self.create_subscription(PoseStamped, str(cfg.get("reference_topic", "/landing/reference_pose")), self.on_ref, 10)
        self.create_subscription(SafetyStatus, str(cfg.get("safety_topic", "/landing/safety_status")), self.on_safety, 10)
        self.timer = self.create_timer(0.05, self.publish_command)

    def on_state(self, msg):
        self.state = msg

    def on_ref(self, msg):
        self.reference = msg

    def on_safety(self, msg):
        self.safe = msg.safe

    def publish_command(self):
        cmd = Twist()
        if self.state is not None and self.reference is not None and self.safe:
            uav = self.state.uav_pose.pose.position
            ref = self.reference.pose.position
            desired_x = self.clamp((ref.x - uav.x) * self.kp_xy, self.max_xy)
            desired_y = self.clamp((ref.y - uav.y) * self.kp_xy, self.max_xy)
            # apply slew-rate limiter
            now = self.get_clock().now()
            dt = (now - self._prev_time).nanoseconds * 1e-9 if self._prev_time is not None else 0.05
            dt = max(dt, 1e-6)
            if self._prev_cmd is None:
                prev_x = 0.0; prev_y = 0.0
            else:
                prev_x = self._prev_cmd.linear.x; prev_y = self._prev_cmd.linear.y
            max_delta = self.max_xy_rate * dt
            delta_x = desired_x - prev_x
            delta_y = desired_y - prev_y
            if delta_x > max_delta:
                cmd.linear.x = prev_x + max_delta
            elif delta_x < -max_delta:
                cmd.linear.x = prev_x - max_delta
            else:
                cmd.linear.x = desired_x
            if delta_y > max_delta:
                cmd.linear.y = prev_y + max_delta
            elif delta_y < -max_delta:
                cmd.linear.y = prev_y - max_delta
            else:
                cmd.linear.y = desired_y
            self._prev_time = now
            # store for next iteration
            self._prev_cmd = Twist()
            self._prev_cmd.linear.x = cmd.linear.x
            self._prev_cmd.linear.y = cmd.linear.y
            cmd.linear.z = self.clamp((ref.z - uav.z) * self.kp_z, self.max_z)
            xy = math.sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y)
            if xy > self.max_xy:
                scale = self.max_xy / xy
                cmd.linear.x *= scale
                cmd.linear.y *= scale
        self.cmd_pub.publish(cmd)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))


def main():
    rclpy.init()
    node = LandingControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
