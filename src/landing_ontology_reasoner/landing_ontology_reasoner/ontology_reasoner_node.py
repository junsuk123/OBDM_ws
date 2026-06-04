#!/usr/bin/env python3
"""Rule-based semantic landing decision node."""

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from landing_interfaces.msg import LandingDecision, LandingState


class OntologyReasonerNode(Node):
    def __init__(self):
        super().__init__("ontology_reasoner_node")
        default_rules = (
            Path(get_package_share_directory("landing_ontology_reasoner"))
            / "rules"
            / "landing_rules.yaml"
        )
        self.declare_parameter("rules_file", str(default_rules))
        self.rules = self.load_rules(Path(self.get_parameter("rules_file").value))
        self.decision_pub = self.create_publisher(LandingDecision, "/landing/decision", 10)
        self.state_sub = self.create_subscription(
            LandingState, "/landing/state", self.on_state, 10
        )
        self.get_logger().info("Ontology reasoner publishing /landing/decision")

    @staticmethod
    def load_rules(path: Path):
        with path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def on_state(self, state: LandingState):
        thresholds = self.rules.get("thresholds", {})
        context = {
            **thresholds,
            "marker_visible": bool(state.marker_visible),
            "marker_confidence": float(state.marker_confidence),
            "relative_distance": float(state.relative_distance),
            "platform_speed": self.twist_speed(state.platform_velocity.twist),
            "wind_speed": float(state.wind_speed),
            "wind_direction": float(state.wind_direction),
            "attitude_error": float(state.attitude_error),
            "altitude": float(state.altitude),
            "battery_level": float(state.battery_level),
            "obstacle_distance": 999.0,
        }

        selected = self.rules.get("default", {})
        for rule in self.rules.get("rules", []):
            if self.safe_eval(rule.get("when", "False"), context):
                selected = rule
                break

        msg = LandingDecision()
        msg.header = state.header
        msg.landing_phase = str(selected.get("landing_phase", "HOLD"))
        msg.selected_mode = str(selected.get("selected_mode", "HOVER_AND_ALIGN"))
        msg.risk_level = str(selected.get("risk_level", "MEDIUM"))
        msg.allowed_to_descend = bool(selected.get("allowed_to_descend", False))
        msg.abort_required = bool(selected.get("abort_required", False))
        msg.explanation = str(selected.get("explanation", "No explanation available."))
        self.decision_pub.publish(msg)

    @staticmethod
    def twist_speed(twist):
        vx = float(twist.linear.x)
        vy = float(twist.linear.y)
        vz = float(twist.linear.z)
        return math.sqrt(vx * vx + vy * vy + vz * vz)

    @staticmethod
    def safe_eval(expression: str, context: dict) -> bool:
        allowed_names = {"abs": abs, "min": min, "max": max, "math": math}
        return bool(eval(expression, {"__builtins__": {}}, {**allowed_names, **context}))


def main():
    rclpy.init()
    node = OntologyReasonerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
