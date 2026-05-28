#!/usr/bin/env python3
"""Subscribe to /model/r1_rover/cmd_vel and publish per-wheel forces.

The r1_rover model uses opposite joint axes on the left and right sides.
To make the rover drive forward instead of fighting itself, the right-side
wheels must receive the opposite signed force from the left-side wheels.
"""
import subprocess
import os
import rclpy
from geometry_msgs.msg import Twist
import traceback


def _log_debug(line: str):
    try:
        with open('/tmp/rover_cmd_vel_bridge_debug.log', 'a') as f:
            f.write(line + '\n')
    except Exception:
        pass


def publish_force(wheel_topic, force):
    # Use gz topic -p to publish gz.msgs.Double
    cmd = [
        'gz', 'topic', '-t', wheel_topic, '-m', 'gz.msgs.Double', '-p', f'data: {float(force)}'
    ]
    try:
        subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


class RoverCmdVelBridge:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('rover_cmd_vel_to_wheels')
        self.sub = self.node.create_subscription(Twist, '/model/r1_rover/cmd_vel', self.cb, 10)
        self.force_gain = float(os.getenv('OBDM_ROVER_FORCE_GAIN', '1.2'))
        self.turn_gain = float(os.getenv('OBDM_ROVER_TURN_GAIN', '1.8'))
        # Maximum absolute force to apply per wheel (clamps output)
        self.force_max = float(os.getenv('OBDM_ROVER_FORCE_MAX', '3.0'))
        # Maximum accepted cmd_vel linear.x (m/s) to avoid excessive commanded speeds
        self.cmdvel_max = float(os.getenv('OBDM_ROVER_CMDVEL_MAX', '0.5'))
        self.yawrate_max = float(os.getenv('OBDM_ROVER_YAWRATE_MAX', '1.2'))
        # wheel joint topics in gz
        self.wheels = [
            ('/model/r1_rover/joint/motor_0/cmd_force', 1.0),
            ('/model/r1_rover/joint/motor_1/cmd_force', 1.0),
            ('/model/r1_rover/joint/motor_2/cmd_force', -1.0),
            ('/model/r1_rover/joint/motor_3/cmd_force', -1.0),
        ]

    def cb(self, msg: Twist):
        lin = float(msg.linear.x)
        if lin > self.cmdvel_max:
            lin = self.cmdvel_max
        elif lin < -self.cmdvel_max:
            lin = -self.cmdvel_max
        yaw = float(msg.angular.z)
        if yaw > self.yawrate_max:
            yaw = self.yawrate_max
        elif yaw < -self.yawrate_max:
            yaw = -self.yawrate_max

        left_raw = self.force_gain * lin - self.turn_gain * yaw
        right_raw = self.force_gain * lin + self.turn_gain * yaw
        left_force = max(-self.force_max, min(self.force_max, left_raw))
        right_force = max(-self.force_max, min(self.force_max, right_raw))

        wheel_forces = {
            '/model/r1_rover/joint/motor_0/cmd_force': left_force,
            '/model/r1_rover/joint/motor_1/cmd_force': left_force,
            '/model/r1_rover/joint/motor_2/cmd_force': -right_force,
            '/model/r1_rover/joint/motor_3/cmd_force': -right_force,
        }
        for wheel_topic, _ in self.wheels:
            publish_force(wheel_topic, wheel_forces[wheel_topic])
        # debug log to file for reliable capture
        _log_debug(
            f'[rover_bridge] linear.x_in={msg.linear.x:.3f} linear.x_used={lin:.3f} '
            f'angular.z_in={msg.angular.z:.3f} angular.z_used={yaw:.3f} '
            f'left_force={left_force:.3f} right_force={right_force:.3f}'
        )

    def spin(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                # ignore shutdown errors when context already shutdown
                pass


def main():
    bridge = RoverCmdVelBridge()
    try:
        bridge.spin()
    except Exception as e:
        _log_debug('Exception in bridge: ' + repr(e))
        _log_debug(traceback.format_exc())
        raise


if __name__ == '__main__':
    main()
