#!/usr/bin/env python3
"""Aggregate raw landing signals into LandingState."""

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32

from landing_interfaces.msg import LandingState


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__("state_estimator_node")
        default_config = (
            Path(get_package_share_directory("landing_state_estimation"))
            / "config"
            / "state_topics.yaml"
        )
        self.declare_parameter("config_file", str(default_config))
        with Path(self.get_parameter("config_file").value).open("r", encoding="utf-8") as stream:
            topics = yaml.safe_load(stream).get("topics", {})

        self.uav_pose = PoseStamped()
        self.uav_velocity = TwistStamped()
        self.platform_pose = PoseStamped()
        self.platform_velocity = TwistStamped()
        self.marker_visible = False
        self.marker_confidence = 0.0
        self.wind_speed = 0.0
        self.wind_direction = 0.0
        self.attitude_error = 0.0
        self.battery_level = 100.0

        self.pub = self.create_publisher(LandingState, topics.get("output_state", "/landing/state"), 10)
        self.create_subscription(PoseStamped, topics.get("uav_pose", "/mavros/local_position/pose"), self.on_uav_pose, 10)
        self.create_subscription(TwistStamped, topics.get("uav_velocity", "/mavros/local_position/velocity_local"), self.on_uav_velocity, 10)
        self.create_subscription(PoseStamped, topics.get("platform_pose", "/landing/pad_pose"), self.on_platform_pose, 10)
        self.create_subscription(TwistStamped, topics.get("platform_velocity", "/landing/platform_velocity"), self.on_platform_velocity, 10)
        self.create_subscription(Bool, topics.get("marker_visible", "/landing/marker_visible"), self.on_marker_visible, 10)
        self.create_subscription(Float32, topics.get("marker_confidence", "/landing/marker_confidence"), self.on_marker_confidence, 10)
        self.create_subscription(Vector3Stamped, topics.get("wind_sensor", "/iris/wind_sensor"), self.on_wind, 10)
        self.create_subscription(BatteryState, topics.get("battery", "/mavros/battery"), self.on_battery, 10)
        self.timer = self.create_timer(0.1, self.publish_state)

    def on_uav_pose(self, msg):
        self.uav_pose = msg

    def on_uav_velocity(self, msg):
        self.uav_velocity = msg

    def on_platform_pose(self, msg):
        self.platform_pose = msg

    def on_platform_velocity(self, msg):
        self.platform_velocity = msg

    def on_marker_visible(self, msg):
        self.marker_visible = bool(msg.data)

    def on_marker_confidence(self, msg):
        self.marker_confidence = float(msg.data)

    def on_wind(self, msg):
        vx = float(msg.vector.x)
        vy = float(msg.vector.y)
        vz = float(msg.vector.z)
        self.wind_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        self.wind_direction = math.degrees(math.atan2(vy, vx))

    def on_battery(self, msg):
        if msg.percentage >= 0.0:
            self.battery_level = float(msg.percentage) * 100.0

    def publish_state(self):
        state = LandingState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = self.uav_pose.header.frame_id or "map"
        state.uav_pose = self.uav_pose
        state.uav_velocity = self.uav_velocity
        state.platform_pose = self.platform_pose
        state.platform_velocity = self.platform_velocity
        state.relative_pose = PoseStamped()
        state.relative_pose.header = state.header
        state.relative_pose.pose.position.x = self.platform_pose.pose.position.x - self.uav_pose.pose.position.x
        state.relative_pose.pose.position.y = self.platform_pose.pose.position.y - self.uav_pose.pose.position.y
        state.relative_pose.pose.position.z = self.platform_pose.pose.position.z - self.uav_pose.pose.position.z
        dx = state.relative_pose.pose.position.x
        dy = state.relative_pose.pose.position.y
        dz = state.relative_pose.pose.position.z
        state.relative_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        state.marker_visible = self.marker_visible
        state.marker_confidence = self.marker_confidence
        state.wind_speed = self.wind_speed
        state.wind_direction = self.wind_direction
        # Compute attitude error (max of absolute roll/pitch in degrees) from UAV orientation
        try:
            q = self.uav_pose.pose.orientation
            x = float(q.x); y = float(q.y); z = float(q.z); w = float(q.w)
            # roll (x-axis rotation)
            sinr_cosp = 2.0 * (w * x + y * z)
            cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            # pitch (y-axis rotation)
            sinp = 2.0 * (w * y - z * x)
            if sinp >= 1.0:
                pitch = math.pi / 2.0
            elif sinp <= -1.0:
                pitch = -math.pi / 2.0
            else:
                pitch = math.asin(sinp)
            # express attitude error as max(|roll|, |pitch|) in degrees
            self.attitude_error = max(abs(roll), abs(pitch)) * 180.0 / math.pi
        except Exception:
            # fallback to previously stored value
            pass
        state.attitude_error = self.attitude_error
        state.altitude = float(self.uav_pose.pose.position.z)
        state.battery_level = self.battery_level
        self.pub.publish(state)


def main():
    rclpy.init()
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
