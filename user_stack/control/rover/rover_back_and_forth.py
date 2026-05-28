#!/usr/bin/env python3
"""Publish /model/r1_rover/cmd_vel to make rover go forward then return repeatedly.

Environment variables:
- OBDM_BAF_SPEED (m/s) default 0.2
- OBDM_BAF_FORWARD_SEC default 2.0
- OBDM_BAF_RETURN_SEC default 2.0
- OBDM_BAF_PAUSE_SEC default 0.5
- OBDM_BAF_REPEAT default 0 (0=infinite)
- Use Ctrl+C to stop when running in foreground.
"""
import os
import time
import signal
import sys

import rclpy
from geometry_msgs.msg import Twist

running = True

def handle_sigint(signum, frame):
    global running
    running = False

signal.signal(signal.SIGINT, handle_sigint)

SPEED = float(os.getenv('OBDM_BAF_SPEED', '0.2'))
FORWARD_SEC = float(os.getenv('OBDM_BAF_FORWARD_SEC', '2.0'))
RETURN_SEC = float(os.getenv('OBDM_BAF_RETURN_SEC', '2.0'))
PAUSE_SEC = float(os.getenv('OBDM_BAF_PAUSE_SEC', '0.5'))
REPEAT = int(os.getenv('OBDM_BAF_REPEAT', '0'))
RATE = int(os.getenv('OBDM_BAF_RATE', '10'))


def publish_for_seconds(pub, duration, linear_x):
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    steps = max(1, int(duration * RATE))
    for _ in range(steps):
        if not running:
            break
        pub.publish(msg)
        time.sleep(1.0 / RATE)
    # send zero to stop
    msg.linear.x = 0.0
    pub.publish(msg)


def main():
    rclpy.init()
    node = rclpy.create_node('rover_back_and_forth')
    pub = node.create_publisher(Twist, '/model/r1_rover/cmd_vel', 10)
    try:
        iter_count = 0
        while running:
            if REPEAT and iter_count >= REPEAT:
                break
            # forward
            publish_for_seconds(pub, FORWARD_SEC, SPEED)
            time.sleep(PAUSE_SEC)
            # return
            publish_for_seconds(pub, RETURN_SEC, -SPEED)
            time.sleep(PAUSE_SEC)
            iter_count += 1
    except Exception as e:
        node.get_logger().error('Exception in rover_back_and_forth: %s' % str(e))
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
