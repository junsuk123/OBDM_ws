#!/usr/bin/env python3
"""Read one geometry_msgs/Vector3Stamped sample without using the ros2 CLI daemon."""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--timeout", type=float, default=0.6)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("obdm_read_vector3_once")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    value = {"seen": False, "data": (0.0, 0.0, 0.0)}

    def on_msg(msg):
        value["seen"] = True
        value["data"] = (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))

    node.create_subscription(Vector3Stamped, args.topic, on_msg, qos)
    deadline = time.monotonic() + max(0.05, args.timeout)
    try:
        while rclpy.ok() and not value["seen"] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not value["seen"]:
        return 1
    print(f"{value['data'][0]:.12g} {value['data'][1]:.12g} {value['data'][2]:.12g}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
