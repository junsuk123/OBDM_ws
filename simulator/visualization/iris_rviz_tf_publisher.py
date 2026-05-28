#!/usr/bin/env python3
"""Publish the minimal TF tree needed by the RViz Iris RobotModel."""

import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class IrisRvizTfPublisher:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("iris_rviz_tf_publisher")
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.static_broadcaster = StaticTransformBroadcaster(self.node)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, qos)
        self.publish_static_transforms()
        self.node.get_logger().info(f"Publishing RViz TF from {args.odom_topic}")

    def make_transform(self, parent, child, xyz, quat=(0.0, 0.0, 0.0, 1.0)):
        tf = TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation.x = float(quat[0])
        tf.transform.rotation.y = float(quat[1])
        tf.transform.rotation.z = float(quat[2])
        tf.transform.rotation.w = float(quat[3])
        return tf

    def publish_static_transforms(self):
        static_tfs = [
            self.make_transform("base_link", "imu_link", (0.0, 0.0, 0.02)),
            self.make_transform("base_link", "gimbal_link", (0.10, 0.0, -0.08)),
            self.make_transform("gimbal_link", "yaw_link", (0.0, 0.0, -0.04)),
            self.make_transform("yaw_link", "roll_link", (0.0, 0.0, -0.04)),
            self.make_transform("roll_link", "pitch_link", (0.0, 0.0, -0.04), (0.7071068, 0.0, 0.0, 0.7071068)),
            self.make_transform("base_link", "rotor_0", (0.13, -0.22, 0.023)),
            self.make_transform("base_link", "rotor_1", (-0.13, 0.20, 0.023)),
            self.make_transform("base_link", "rotor_2", (0.13, 0.22, 0.023)),
            self.make_transform("base_link", "rotor_3", (-0.13, -0.20, 0.023)),
        ]
        self.static_broadcaster.sendTransform(static_tfs)

    def on_odom(self, msg):
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.args.fixed_frame
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-topic", default="/iris/odometry")
    parser.add_argument("--fixed-frame", default="odom")
    args = parser.parse_args()

    rclpy.init()
    pub = IrisRvizTfPublisher(args)
    try:
        rclpy.spin(pub.node)
    finally:
        pub.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
