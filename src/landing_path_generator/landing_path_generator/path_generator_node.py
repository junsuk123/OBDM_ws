#!/usr/bin/env python3
"""Generate landing reference poses from semantic decisions."""

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node

from landing_interfaces.msg import LandingDecision, LandingState


class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__("path_generator_node")
        default_config = (
            Path(get_package_share_directory("landing_path_generator"))
            / "config"
            / "path_modes.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        with Path(self.get_parameter("config_file").value).open("r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        self.state = None
        self.decision = None
        self.pose_pub = self.create_publisher(PoseStamped, "/landing/reference_pose", 10)
        self.path_pub = self.create_publisher(PathMsg, "/landing/reference_path", 10)
        self.create_subscription(LandingState, "/landing/state", self.on_state, 10)
        self.create_subscription(LandingDecision, "/landing/decision", self.on_decision, 10)
        self.timer = self.create_timer(0.1, self.publish_reference)

    def on_state(self, msg):
        self.state = msg

    def on_decision(self, msg):
        self.decision = msg

    def publish_reference(self):
        if self.state is None or self.decision is None:
            return

        mode = self.decision.selected_mode
        cfg = self.config.get("modes", {}).get(mode, {})
        target = PoseStamped()
        target.header = self.state.header
        target.pose = self.state.platform_pose.pose

        horizontal_gain = float(cfg.get("horizontal_gain", 0.0))
        lookahead_s = float(cfg.get("lookahead_s", 0.0))
        platform_velocity = self.state.platform_velocity.twist
        target.pose.position.x += horizontal_gain * lookahead_s * float(platform_velocity.linear.x)
        target.pose.position.y += horizontal_gain * lookahead_s * float(platform_velocity.linear.y)

        altitude = float(self.state.altitude)
        if mode == "ASCEND_AND_RESEARCH":
            target.pose.position.z = max(altitude, float(cfg.get("search_altitude_m", 4.0)))
        elif mode == "ABORT_LANDING":
            target.pose.position.z = max(altitude, float(cfg.get("abort_altitude_m", 8.0)))
        elif mode == "HOVER_AND_ALIGN":
            target.pose.position.z = altitude
        else:
            descent = float(cfg.get("vertical_rate_mps", 0.15)) * 0.5
            target.pose.position.z = max(0.2, altitude - descent)

        if mode == "CONSERVATIVE_APPROACH":
            target.pose.position.y += float(cfg.get("lateral_offset_m", 1.5))

        path = PathMsg()
        path.header = target.header
        path.poses = [self.state.uav_pose, target]
        self.pose_pub.publish(target)
        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = PathGeneratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
