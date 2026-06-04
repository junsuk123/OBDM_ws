#!/usr/bin/env python3
"""Publish /model/r1_rover/cmd_vel to make rover go forward then return repeatedly.

Environment variables:
- OBDM_BAF_SPEED (m/s) default 0.2
- OBDM_BAF_FORWARD_SEC default 2.0
- OBDM_BAF_RETURN_SEC default 2.0
- OBDM_BAF_PAUSE_SEC default 0.5
- OBDM_BAF_RANDOM default 1 (set 0 for deterministic back-and-forth)
- OBDM_BAF_SPEED_JITTER default 0.35
- OBDM_BAF_DURATION_JITTER default 0.40
- OBDM_BAF_TURN_MAX (rad/s) default 0.25
- OBDM_BAF_REPEAT default 0 (0=infinite)
- Use Ctrl+C to stop when running in foreground.
"""
import os
import random
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
RANDOMIZE = os.getenv('OBDM_BAF_RANDOM', '1').lower() not in ('0', 'false', 'no')
SPEED_JITTER = max(0.0, float(os.getenv('OBDM_BAF_SPEED_JITTER', '0.35')))
DURATION_JITTER = max(0.0, float(os.getenv('OBDM_BAF_DURATION_JITTER', '0.40')))
TURN_MAX = max(0.0, float(os.getenv('OBDM_BAF_TURN_MAX', '0.25')))
RNG = random.Random()


def jittered(value, fraction, minimum=0.0):
    if not RANDOMIZE or fraction <= 0.0:
        return value
    scale = RNG.uniform(max(0.0, 1.0 - fraction), 1.0 + fraction)
    return max(minimum, value * scale)


def publish_for_seconds(pub, duration, linear_x, angular_z=0.0):
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(angular_z)
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
            forward_speed = jittered(SPEED, SPEED_JITTER)
            return_speed = jittered(SPEED, SPEED_JITTER)
            forward_sec = jittered(FORWARD_SEC, DURATION_JITTER, 0.1)
            return_sec = jittered(RETURN_SEC, DURATION_JITTER, 0.1)
            forward_turn = RNG.uniform(-TURN_MAX, TURN_MAX) if RANDOMIZE else 0.0
            return_turn = RNG.uniform(-TURN_MAX, TURN_MAX) if RANDOMIZE else 0.0
            publish_for_seconds(pub, forward_sec, forward_speed, forward_turn)
            time.sleep(jittered(PAUSE_SEC, DURATION_JITTER, 0.0))
            publish_for_seconds(pub, return_sec, -return_speed, return_turn)
            time.sleep(jittered(PAUSE_SEC, DURATION_JITTER, 0.0))
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
