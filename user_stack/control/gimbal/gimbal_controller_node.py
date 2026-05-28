#!/usr/bin/env python3
"""ROS2 node to control gimbal orientation via gz set_pose service.

Subscribes to `/gimbal/angle_cmd` (geometry_msgs/Vector3) where x=roll,y=pitch,z=yaw in degrees.
Calls `gz service /world/<world>/set_pose` to update the gimbal link pose.
"""
import argparse
import math
import shlex
import subprocess
import sys

import rclpy
from geometry_msgs.msg import Vector3


def euler_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


class GimbalController:
    def __init__(self, args):
        self.args = args
        rclpy.init()
        self.node = rclpy.create_node('gimbal_controller')
        self.sub = self.node.create_subscription(Vector3, args.cmd_topic, self.cb, 10)

    def cb(self, msg: Vector3):
        roll = math.radians(msg.x)
        pitch = math.radians(msg.y)
        yaw = math.radians(msg.z)
        qw, qx, qy, qz = euler_to_quat(roll, pitch, yaw)
        name = f"{self.args.model_name}::{self.args.link_name}"
        payload = (
            f'name: "{name}" '
            f'position {{x: 0.0 y: 0.0 z: 0.0}} '
            f'orientation {{w: {qw:.6f} x: {qx:.6f} y: {qy:.6f} z: {qz:.6f}}}'
        )
        service = f"/world/{self.args.world}/set_pose"
        try:
            result = subprocess.run([
                'gz', 'service', '-s', service,
                '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000', '--req', payload
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            ok = result.returncode == 0 and 'data: true' in result.stdout
            if not ok and self.args.verbose:
                self.node.get_logger().warn(f"set_pose failed: stdout={result.stdout.strip()}, stderr={result.stderr.strip()}")
        except Exception as e:
            self.node.get_logger().error(f"Exception calling gz service: {e}")

    def spin(self):
        try:
            rclpy.spin(self.node)
        finally:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='runway')
    parser.add_argument('--model-name', default='iris_with_gimbal')
    parser.add_argument('--link-name', default='gimbal_link')
    parser.add_argument('--cmd-topic', default='/gimbal/angle_cmd')
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    controller = GimbalController(args)
    controller.spin()


if __name__ == '__main__':
    main()
