#!/usr/bin/env python3
"""Safety checks before descent/controller execution."""

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool

from landing_interfaces.msg import LandingDecision, LandingState, SafetyStatus


class SafetyLayerNode(Node):
    def __init__(self):
        super().__init__("safety_layer_node")
        default_config = (
            Path(get_package_share_directory("landing_safety_layer"))
            / "config"
            / "safety_limits.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        with Path(self.get_parameter("config_file").value).open("r", encoding="utf-8") as stream:
            self.limits = yaml.safe_load(stream).get("limits", {})
        self.state = None
        self.decision = None
        self.status_pub = self.create_publisher(SafetyStatus, "/landing/safety_status", 10)
        self.descend_pub = self.create_publisher(Bool, "/landing/allowed_to_descend", 10)
        self.create_subscription(LandingState, "/landing/state", self.on_state, 10)
        self.create_subscription(LandingDecision, "/landing/decision", self.on_decision, 10)
        self.timer = self.create_timer(0.1, self.publish_status)

    def on_state(self, msg):
        self.state = msg

    def on_decision(self, msg):
        self.decision = msg

    def publish_status(self):
        if self.state is None:
            return
        platform_speed = self.speed(self.state.platform_velocity.twist.linear)
        status = SafetyStatus()
        status.header = self.state.header
        status.obstacle_clear = True
        status.marker_reliable = (
            self.state.marker_visible
            and self.state.marker_confidence
            >= float(self.limits.get("final_marker_confidence_min", 0.75))
        )
        status.wind_safe = self.state.wind_speed <= float(self.limits.get("wind_speed_max_mps", 8.0))
        status.platform_motion_safe = platform_speed <= float(
            self.limits.get("platform_speed_max_mps", 1.0)
        )
        status.attitude_safe = self.state.attitude_error <= float(
            self.limits.get("max_attitude_error_deg", 12.0)
        )
        decision_allows = True if self.decision is None else self.decision.allowed_to_descend
        status.safe = all(
            [
                status.obstacle_clear,
                status.wind_safe,
                status.platform_motion_safe,
                status.attitude_safe,
                decision_allows,
            ]
        )
        status.safety_margin = min(
            float(self.limits.get("wind_speed_max_mps", 8.0)) - self.state.wind_speed,
            float(self.limits.get("max_attitude_error_deg", 12.0)) - self.state.attitude_error,
        )
        reasons = []
        if not decision_allows:
            reasons.append("decision_blocks_descent")
        if not status.wind_safe:
            reasons.append("wind_limit")
        if not status.platform_motion_safe:
            reasons.append("platform_motion_limit")
        if not status.attitude_safe:
            reasons.append("attitude_limit")
        status.reason = ",".join(reasons) if reasons else "safe"
        self.status_pub.publish(status)
        self.descend_pub.publish(Bool(data=status.safe))

    @staticmethod
    def speed(vec):
        return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)


def main():
    rclpy.init()
    node = SafetyLayerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
