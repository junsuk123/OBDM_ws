#!/usr/bin/env python3
"""Read one std_msgs/Bool sample without using the ros2 CLI daemon."""

import argparse
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--timeout", type=float, default=0.6)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("obdm_read_bool_once")
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    value = {"seen": False, "data": False}

    def on_msg(msg):
        value["seen"] = True
        value["data"] = bool(msg.data)

    node.create_subscription(Bool, args.topic, on_msg, qos)
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
    print("true" if value["data"] else "false")
    return 0


if __name__ == "__main__":
    sys.exit(main())
