#!/usr/bin/env python3
"""Bridge existing marker topics into the /landing namespace."""

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class PerceptionAdapterNode(Node):
    def __init__(self):
        super().__init__("perception_adapter_node")
        default_config = (
            Path(get_package_share_directory("landing_perception"))
            / "config"
            / "perception_topics.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        with Path(self.get_parameter("config_file").value).open("r", encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream)
        topics = cfg.get("topics", {})
        compat = cfg.get("compatibility", {})
        self.visible_pub = self.create_publisher(Bool, topics.get("marker_visible", "/landing/marker_visible"), 10)
        self.conf_pub = self.create_publisher(Float32, topics.get("marker_confidence", "/landing/marker_confidence"), 10)
        self.pose_pub = self.create_publisher(PoseStamped, topics.get("marker_pose", "/landing/marker_pose"), 10)
        self.pad_pub = self.create_publisher(PoseStamped, topics.get("pad_pose", "/landing/pad_pose"), 10)
        self.create_subscription(Bool, compat.get("existing_visible_topic", "/landing_marker/visible"), self.on_visible, 10)
        self.create_subscription(PoseStamped, compat.get("existing_pose_topic", "/landing_marker/pose"), self.on_pose, 10)

    def on_visible(self, msg):
        self.visible_pub.publish(msg)
        self.conf_pub.publish(Float32(data=1.0 if msg.data else 0.0))

    def on_pose(self, msg):
        self.pose_pub.publish(msg)
        self.pad_pub.publish(msg)


def main():
    rclpy.init()
    node = PerceptionAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
