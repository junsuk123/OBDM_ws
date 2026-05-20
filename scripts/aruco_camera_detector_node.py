#!/usr/bin/env python3
"""Detect the landing ArUco marker from the drone camera image.

Publishes the same landing target contract consumed by MATLAB:
  /landing_marker/odom     nav_msgs/Odometry, world-frame marker estimate
  /landing_marker/pose     geometry_msgs/PoseStamped
  /landing_marker/visible  std_msgs/Bool

Coordinate convention:
  ROS / Gazebo / MATLAB use world x, world y, z-up internally.
  OpenCV camera optical frame is x-right, y-down, z-forward.
  This node compensates a fixed front-down camera pitch, then converts the
  marker estimate back into world coordinates. Only vehicle yaw is used for
  horizontal heading, which matches the current fixed-gimbal experiment.
"""

import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArucoCameraDetector:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("aruco_camera_detector")
        self.bridge = CvBridge()

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        ns = args.namespace if args.namespace.startswith("/") else "/" + args.namespace
        ns = ns.rstrip("/")
        image_topic = args.image_topic or f"{ns}/camera/image"
        camera_info_topic = args.camera_info_topic or f"{ns}/camera/camera_info"
        odom_topic = args.odom_topic or f"{ns}/odometry"

        self.image_sub = self.node.create_subscription(Image, image_topic, self.on_image, qos)
        self.info_sub = self.node.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, qos)
        self.odom_sub = self.node.create_subscription(Odometry, odom_topic, self.on_odom, qos)

        self.odom_pub = self.node.create_publisher(Odometry, args.marker_odom_topic, 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, args.marker_pose_topic, 10)
        self.visible_pub = self.node.create_publisher(Bool, args.marker_visible_topic, 10)
        self.timer = self.node.create_timer(0.2, self.publish_visible_timeout)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.drone = None
        self.last_seen_t = 0.0
        self.last_pos = None
        self.last_pos_t = None

        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        else:
            self.detector = None

        self.node.get_logger().info(
            f"Camera ArUco detector: image={image_topic}, info={camera_info_topic}, "
            f"odom={odom_topic}, marker_id={args.marker_id}"
        )

    def on_camera_info(self, msg):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.drone = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "vx": float(v.x),
            "vy": float(v.y),
            "vz": float(v.z),
            "yaw": yaw,
        }

    def on_image(self, msg):
        if self.camera_matrix is None or self.drone is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if frame.ndim == 2:
                pass
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)

        if ids is None or len(ids) == 0:
            return

        ids_flat = ids.flatten()
        matches = np.where(ids_flat == self.args.marker_id)[0]
        if len(matches) == 0:
            return

        marker_corners = corners[int(matches[0])][0].astype(np.float64)
        t = self.estimate_marker_translation(marker_corners)
        if t is None:
            return
        pos = self.camera_to_world(t)
        self.publish_marker(pos, msg.header.stamp)

    def estimate_marker_translation(self, image_corners):
        half = self.args.marker_size_m * 0.5
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        ok, _, tvec = cv2.solvePnP(
            object_points,
            image_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            ok, _, tvec = cv2.solvePnP(
                object_points,
                image_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        if not ok:
            return None
        return tvec.reshape(3).astype(float)

    def camera_to_world(self, tvec):
        raw_right_cam_m = float(tvec[0])
        raw_down_cam_m = float(tvec[1])
        forward_cam_m = float(tvec[2])
        pitch_down = math.radians(self.args.camera_pitch_down_deg)
        roll = math.radians(self.args.camera_roll_deg)

        right_cam_m = raw_right_cam_m * math.cos(roll) - raw_down_cam_m * math.sin(roll)
        down_cam_m = raw_right_cam_m * math.sin(roll) + raw_down_cam_m * math.cos(roll)

        yaw = self.drone["yaw"] + self.args.yaw_offset_rad
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
        right = np.array([math.sin(yaw), -math.cos(yaw)], dtype=np.float64)

        forward_horizontal_m = forward_cam_m * math.cos(pitch_down) - down_cam_m * math.sin(pitch_down)
        down_world_m = forward_cam_m * math.sin(pitch_down) + down_cam_m * math.cos(pitch_down)

        xy = (
            np.array([self.drone["x"], self.drone["y"]], dtype=np.float64)
            + forward * forward_horizontal_m
            + right * right_cam_m
        )
        z = self.drone["z"] - down_world_m + self.args.z_offset_m
        if self.args.clamp_ground_z:
            z = max(self.args.marker_ground_z, min(z, self.drone["z"]))
        return np.array([xy[0], xy[1], z], dtype=np.float64)

    def publish_marker(self, pos, stamp):
        now = time.monotonic()
        vel = np.zeros(3, dtype=np.float64)
        if self.last_pos is not None and self.last_pos_t is not None:
            dt = max(1e-3, now - self.last_pos_t)
            vel = (pos - self.last_pos) / dt

        self.last_pos = pos
        self.last_pos_t = now
        self.last_seen_t = now

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "aruco_landing_marker_camera"
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        visible = Bool()
        visible.data = True
        self.visible_pub.publish(visible)

    def publish_visible_timeout(self):
        visible = Bool()
        visible.data = (time.monotonic() - self.last_seen_t) <= self.args.visible_timeout_s
        self.visible_pub.publish(visible)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/iris")
    parser.add_argument("--image-topic", default=None)
    parser.add_argument("--camera-info-topic", default=None)
    parser.add_argument("--odom-topic", default=None)
    parser.add_argument("--marker-odom-topic", default="/landing_marker/odom")
    parser.add_argument("--marker-pose-topic", default="/landing_marker/pose")
    parser.add_argument("--marker-visible-topic", default="/landing_marker/visible")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--marker-size-m", type=float, default=1.0)
    parser.add_argument("--visible-timeout-s", type=float, default=0.5)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=55.0)
    parser.add_argument("--camera-roll-deg", type=float, default=135.0)
    parser.add_argument("--yaw-offset-rad", type=float, default=0.0)
    parser.add_argument("--z-offset-m", type=float, default=0.0)
    parser.add_argument("--marker-ground-z", type=float, default=0.03)
    parser.add_argument("--clamp-ground-z", action="store_true", default=True)
    args = parser.parse_args()

    rclpy.init()
    detector = ArucoCameraDetector(args)
    try:
        rclpy.spin(detector.node)
    finally:
        detector.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
