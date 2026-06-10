#!/usr/bin/env python3
"""Generate landing reference poses from semantic decisions."""

import copy
import math
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
        target.pose = copy.deepcopy(self.state.platform_pose.pose)

        horizontal_gain = float(cfg.get("horizontal_gain", 0.0))
        lookahead_s = float(cfg.get("lookahead_s", 0.0))
        platform_velocity = self.state.platform_velocity.twist
        precision = self.config.get("precision_landing", {})
        range_xy = float(self.state.relative_distance)
        approach_radius = float(precision.get("approach_radius_m", 2.5))
        final_alignment_radius = float(precision.get("final_alignment_radius_m", 0.28))
        precision_blend = self._clamp(
            1.0 - (range_xy - final_alignment_radius) / max(approach_radius - final_alignment_radius, 1.0e-6),
            0.0,
            1.0,
        )
        lead_scale = 1.0 - 0.85 * precision_blend
        target.pose.position.x += lead_scale * horizontal_gain * lookahead_s * float(platform_velocity.linear.x)
        target.pose.position.y += lead_scale * horizontal_gain * lookahead_s * float(platform_velocity.linear.y)

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

        self._apply_precision_landing_limits(target, mode, cfg, precision, precision_blend)

        path = PathMsg()
        path.header = target.header
        path.poses = [self.state.uav_pose, target]
        self.pose_pub.publish(target)
        self.path_pub.publish(path)

    def _apply_precision_landing_limits(self, target, mode, cfg, precision, precision_blend):
        if precision_blend <= 0.0:
            return

        ux = float(self.state.uav_pose.pose.position.x)
        uy = float(self.state.uav_pose.pose.position.y)
        dx = float(target.pose.position.x) - ux
        dy = float(target.pose.position.y) - uy
        dist = math.hypot(dx, dy)
        max_step = self._lerp(
            float(precision.get("max_step_m", 0.85)),
            float(precision.get("final_max_step_m", 0.22)),
            precision_blend,
        )
        if dist > max_step > 0.0:
            scale = max_step / dist
            target.pose.position.x = ux + dx * scale
            target.pose.position.y = uy + dy * scale

        uav_twist = self.state.uav_velocity.twist
        platform_twist = self.state.platform_velocity.twist
        horizontal_speed = math.hypot(float(uav_twist.linear.x), float(uav_twist.linear.y))
        relative_speed = math.hypot(
            float(uav_twist.linear.x) - float(platform_twist.linear.x),
            float(uav_twist.linear.y) - float(platform_twist.linear.y),
        )
        aligned = float(self.state.relative_distance) <= float(precision.get("alignment_radius_m", 0.65))
        slow = (
            horizontal_speed <= float(precision.get("max_horizontal_speed_mps", 0.70))
            and relative_speed <= float(precision.get("max_relative_speed_mps", 0.45))
        )
        precision_ready = aligned and slow

        platform_z = float(self.state.platform_pose.pose.position.z)
        hold_z = platform_z + float(precision.get("hold_height_above_pad_m", 0.70))
        if not precision_ready:
            target.pose.position.z = max(float(target.pose.position.z), hold_z)
        elif mode in ("DIRECT_DESCENT", "EMERGENCY_LAND"):
            descent = float(precision.get("final_vertical_rate_mps", cfg.get("vertical_rate_mps", 0.16))) * 0.5
            target.pose.position.z = max(0.2, float(self.state.altitude) - descent)

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    def _lerp(self, a, b, t):
        return a + (b - a) * self._clamp(t, 0.0, 1.0)


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
