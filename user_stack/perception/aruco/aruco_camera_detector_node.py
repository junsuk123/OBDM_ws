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
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_marker_sizes(spec, default_id, default_size):
    marker_sizes = {}
    if spec:
        for item in spec.split(","):
            item = item.strip()
            if not item:
                continue
            if ":" not in item:
                raise ValueError(f"invalid marker size entry: {item}")
            marker_id_text, size_text = item.split(":", 1)
            marker_id = int(marker_id_text.strip())
            size_m = float(size_text.strip())
            if marker_id < 0 or size_m <= 0.0:
                raise ValueError(f"invalid marker size entry: {item}")
            marker_sizes[marker_id] = size_m
    if not marker_sizes:
        marker_sizes[int(default_id)] = float(default_size)
    return marker_sizes


def parse_marker_center_offsets(spec):
    offsets = {}
    if spec:
        for item in spec.split(","):
            item = item.strip()
            if not item:
                continue
            parts = [part.strip() for part in item.split(":")]
            if len(parts) != 3:
                raise ValueError(f"invalid marker center offset entry: {item}")
            marker_id = int(parts[0])
            offsets[marker_id] = np.array([float(parts[1]), float(parts[2]), 0.0], dtype=np.float64)
    offsets.setdefault(0, np.zeros(3, dtype=np.float64))
    return offsets


class ArucoCameraDetector:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("aruco_camera_detector")

        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos = QoSProfile(depth=10)
        odom_qos.reliability = ReliabilityPolicy.RELIABLE

        ns = args.namespace if args.namespace.startswith("/") else "/" + args.namespace
        ns = ns.rstrip("/")
        image_topic = args.image_topic or f"{ns}/camera/image"
        camera_info_topic = args.camera_info_topic or f"{ns}/camera/camera_info"
        odom_topic = args.odom_topic or f"{ns}/odometry"

        self.image_sub = self.node.create_subscription(Image, image_topic, self.on_image, qos)
        self.info_sub = self.node.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, qos)
        self.odom_sub = self.node.create_subscription(Odometry, odom_topic, self.on_odom, odom_qos)
        self.truth_sub = None
        if args.enable_fov_fallback and args.fov_fallback_marker_topic:
            self.truth_sub = self.node.create_subscription(
                Odometry, args.fov_fallback_marker_topic, self.on_truth_marker, qos
            )

        self.odom_pub = self.node.create_publisher(Odometry, args.marker_odom_topic, 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, args.marker_pose_topic, 10)
        self.visible_pub = self.node.create_publisher(Bool, args.marker_visible_topic, 10)
        self.bbox_pub = self.node.create_publisher(Detection2DArray, args.bbox_topic, 10)
        self.bbox_marker_pub = self.node.create_publisher(MarkerArray, args.bbox_marker_topic, 10)
        self.error_pub = self.node.create_publisher(Vector3Stamped, args.servo_error_topic, 10)
        self.annotated_image_pub = self.node.create_publisher(Image, args.annotated_image_topic, 10)
        self.timer = self.node.create_timer(0.2, self.publish_visible_timeout)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = ""
        self.drone = None
        self.truth_marker = None
        self.last_seen_t = 0.0
        self.last_pos = None
        self.last_pos_t = None
        self.last_marker_odom = None
        self.last_marker_pose = None
        self.last_bbox_frame_id = ""
        self.image_count = 0
        self.camera_info_count = 0
        self.odom_count = 0
        self.detection_count = 0
        self.last_diag_t = time.monotonic()
        self.marker_sizes = parse_marker_sizes(args.marker_sizes, args.marker_id, args.marker_size_m)
        self.marker_center_offsets = parse_marker_center_offsets(args.marker_center_offsets)

        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        else:
            self.detector = None

        self.node.get_logger().info(
            f"Camera ArUco detector: image={image_topic}, info={camera_info_topic}, "
            f"odom={odom_topic}, marker_sizes={self.marker_sizes}, "
            f"center_offsets={{{', '.join(f'{k}:({v[0]:.3f},{v[1]:.3f})' for k, v in sorted(self.marker_center_offsets.items()))}}}"
        )

    def on_camera_info(self, msg):
        self.camera_info_count += 1
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        if msg.header.frame_id:
            self.camera_frame_id = msg.header.frame_id

    def on_odom(self, msg):
        self.odom_count += 1
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

    def on_truth_marker(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.truth_marker = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "vx": float(v.x),
            "vy": float(v.y),
            "vz": float(v.z),
        }

    def on_image(self, msg):
        self.image_count += 1
        try:
            frame = self.image_msg_to_cv_image(msg)
        except Exception as exc:
            self.node.get_logger().warn(f"image conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)

        if ids is None or len(ids) == 0:
            self.publish_empty_bbox(msg.header.stamp, msg.header.frame_id)
            self.clear_bbox_markers()
            self.publish_annotated_image(frame, None, None, msg)
            self.log_diagnostics()
            return

        ids_flat = ids.flatten()
        matches = [idx for idx, marker_id in enumerate(ids_flat) if int(marker_id) in self.marker_sizes]
        if len(matches) == 0:
            self.publish_empty_bbox(msg.header.stamp, msg.header.frame_id)
            self.clear_bbox_markers()
            self.publish_annotated_image(frame, None, None, msg)
            self.log_diagnostics()
            return

        selected = max(matches, key=lambda idx: abs(cv2.contourArea(corners[int(idx)][0].astype(np.float32))))
        marker_corners = corners[int(selected)][0].astype(np.float64)
        marker_id = int(ids_flat[int(selected)])
        self.publish_annotated_image(frame, marker_corners, marker_id, msg)
        self.publish_bbox(marker_corners, marker_id, msg.header.stamp, msg.header.frame_id, msg.width, msg.height)
        self.publish_servo_error(marker_corners, msg.header.stamp, msg.header.frame_id, msg.width, msg.height)

        if self.camera_matrix is None or self.drone is None:
            self.log_diagnostics()
            return

        pos = self.estimate_pad_center_from_markers(corners, ids_flat, matches)
        if pos is None:
            return
        self.detection_count += 1
        self.publish_marker(pos, msg.header.stamp)

    def image_msg_to_cv_image(self, msg):
        enc = msg.encoding.lower()
        channels_by_encoding = {
            "mono8": 1,
            "8uc1": 1,
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
        }
        if enc not in channels_by_encoding:
            raise ValueError(f"unsupported image encoding: {msg.encoding}")

        channels = channels_by_encoding[enc]
        row_bytes = int(msg.step)
        expected_row_bytes = int(msg.width) * channels
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size < row_bytes * int(msg.height):
            raise ValueError("image data shorter than height*step")

        rows = data[: row_bytes * int(msg.height)].reshape((int(msg.height), row_bytes))
        pixels = rows[:, :expected_row_bytes]
        if channels == 1:
            return pixels.reshape((int(msg.height), int(msg.width))).copy()

        frame = pixels.reshape((int(msg.height), int(msg.width), channels))
        if enc == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif enc == "rgba8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif enc == "bgra8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame.copy()

    def log_diagnostics(self):
        now = time.monotonic()
        if now - self.last_diag_t < 2.0:
            return
        self.last_diag_t = now
        self.node.get_logger().info(
            "Detector status: "
            f"images={self.image_count}, camera_info={self.camera_info_count}, "
            f"odom={self.odom_count}, detections={self.detection_count}, "
            f"has_intrinsics={self.camera_matrix is not None}, has_odom={self.drone is not None}"
        )

    def estimate_marker_pose(self, image_corners, marker_size_m):
        half = marker_size_m * 0.5
        object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                image_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None
        return tvec.reshape(3).astype(float), rvec.reshape(3).astype(float)

    def estimate_pad_center_from_markers(self, corners, ids_flat, matches):
        center_candidates = []
        weights = []
        for idx in matches:
            marker_id = int(ids_flat[int(idx)])
            center_offset = self.marker_center_offsets.get(marker_id)
            if center_offset is None:
                self.node.get_logger().warn(
                    f"detected marker {marker_id} has no center offset; skipping it for pad-center fusion",
                    throttle_duration_sec=2.0,
                )
                continue
            marker_corners = corners[int(idx)][0].astype(np.float64)
            marker_size_m = self.marker_sizes[marker_id]
            pose = self.estimate_marker_pose(marker_corners, marker_size_m)
            if pose is None:
                continue
            tvec, rvec = pose
            rotation, _ = cv2.Rodrigues(rvec.reshape(3, 1))
            center_tvec = tvec + rotation.dot(center_offset)
            center_candidates.append(self.camera_to_world(center_tvec))
            area_px = abs(cv2.contourArea(marker_corners.astype(np.float32)))
            weights.append(max(1.0, area_px))

        if not center_candidates:
            return None

        centers = np.vstack(center_candidates)
        weights = np.array(weights, dtype=np.float64)
        pos = np.average(centers, axis=0, weights=weights)
        if self.args.clamp_ground_z:
            pos[2] = self.args.marker_ground_z
        return pos

    def publish_bbox(self, marker_corners, marker_id, stamp, frame_id, image_width, image_height):
        frame_id = frame_id or self.camera_frame_id or "camera"

        min_xy = np.min(marker_corners, axis=0)
        max_xy = np.max(marker_corners, axis=0)
        center_xy = 0.5 * (min_xy + max_xy)
        width_px = max(1.0, float(max_xy[0] - min_xy[0]))
        height_px = max(1.0, float(max_xy[1] - min_xy[1]))

        detection = Detection2D()
        detection.header.stamp = stamp
        detection.header.frame_id = frame_id
        detection.bbox = BoundingBox2D()
        detection.bbox.center.position.x = float(center_xy[0])
        detection.bbox.center.position.y = float(center_xy[1])
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = width_px
        detection.bbox.size_y = height_px

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(marker_id)
        hypothesis.hypothesis.score = 1.0
        detection.results = [hypothesis]

        array = Detection2DArray()
        array.header.stamp = stamp
        array.header.frame_id = frame_id
        array.detections = [detection]
        self.bbox_pub.publish(array)

        if self.camera_matrix is not None:
            marker_array = MarkerArray()
            marker_array.markers = self.build_bbox_markers(marker_corners, marker_id, stamp, frame_id)
            self.bbox_marker_pub.publish(marker_array)
            self.last_bbox_frame_id = frame_id

    def publish_empty_bbox(self, stamp, frame_id):
        array = Detection2DArray()
        array.header.stamp = stamp
        array.header.frame_id = frame_id or self.camera_frame_id or "camera"
        array.detections = []
        self.bbox_pub.publish(array)

    def publish_servo_error(self, marker_corners, stamp, frame_id, image_width, image_height):
        min_xy = np.min(marker_corners, axis=0)
        max_xy = np.max(marker_corners, axis=0)
        center_xy = 0.5 * (min_xy + max_xy)
        width = max(1.0, float(image_width))
        height = max(1.0, float(image_height))

        err = Vector3Stamped()
        err.header.stamp = stamp
        err.header.frame_id = frame_id or self.camera_frame_id or "camera"
        err.vector.x = float((center_xy[0] - 0.5 * width) / (0.5 * width))
        err.vector.y = float((center_xy[1] - 0.5 * height) / (0.5 * height))
        err.vector.z = float(((max_xy[0] - min_xy[0]) * (max_xy[1] - min_xy[1])) / (width * height))
        self.error_pub.publish(err)

    def publish_annotated_image(self, frame, marker_corners, marker_id, source_msg):
        annotated = frame.copy()
        if annotated.ndim == 2:
            annotated = cv2.cvtColor(annotated, cv2.COLOR_GRAY2BGR)

        cv2.drawMarker(
            annotated,
            (int(source_msg.width * 0.5), int(source_msg.height * 0.5)),
            (255, 0, 0),
            cv2.MARKER_CROSS,
            18,
            2,
        )

        if marker_corners is not None:
            pts = marker_corners.reshape((-1, 1, 2)).astype(np.int32)
            cv2.polylines(annotated, [pts], True, (0, 255, 0), 2)
            center = np.mean(marker_corners, axis=0).astype(int)
            cv2.drawMarker(annotated, tuple(center), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(
                annotated,
                f"aruco {marker_id}",
                (max(0, center[0] + 8), max(20, center[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.putText(
                annotated,
                "aruco not detected",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        msg = Image()
        msg.header = source_msg.header
        msg.height = annotated.shape[0]
        msg.width = annotated.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(annotated.shape[1] * 3)
        msg.data = annotated.tobytes()
        self.annotated_image_pub.publish(msg)

    def build_bbox_markers(self, marker_corners, marker_id, stamp, frame_id):
        depth_m = 1.0
        points = [self.pixel_to_camera_point(corner[0], corner[1], depth_m) for corner in marker_corners]
        closed_points = points + [points[0]]
        lifetime = Duration(sec=0, nanosec=350000000)

        border = Marker()
        border.header.stamp = stamp
        border.header.frame_id = frame_id
        border.ns = "aruco_bbox"
        border.id = marker_id
        border.type = Marker.LINE_STRIP
        border.action = Marker.ADD
        border.scale.x = 0.02
        border.color.r = 0.1
        border.color.g = 0.95
        border.color.b = 0.2
        border.color.a = 0.95
        border.points = [self.point_msg(point) for point in closed_points]
        border.frame_locked = True
        border.lifetime = lifetime

        center = Marker()
        center.header.stamp = stamp
        center.header.frame_id = frame_id
        center.ns = "aruco_bbox"
        center.id = marker_id + 1000
        center.type = Marker.SPHERE
        center.action = Marker.ADD
        center.pose.position.x = float(np.mean([point[0] for point in points]))
        center.pose.position.y = float(np.mean([point[1] for point in points]))
        center.pose.position.z = float(np.mean([point[2] for point in points]))
        center.pose.orientation.w = 1.0
        center.scale.x = 0.06
        center.scale.y = 0.06
        center.scale.z = 0.06
        center.color.r = 1.0
        center.color.g = 0.2
        center.color.b = 0.2
        center.color.a = 0.9
        center.frame_locked = True
        center.lifetime = lifetime

        label = Marker()
        label.header.stamp = stamp
        label.header.frame_id = frame_id
        label.ns = "aruco_bbox"
        label.id = marker_id + 2000
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = center.pose.position.x
        label.pose.position.y = center.pose.position.y
        label.pose.position.z = center.pose.position.z + 0.12
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 0.95
        label.text = f"aruco {marker_id}"
        label.frame_locked = True
        label.lifetime = lifetime

        return [border, center, label]

    def pixel_to_camera_point(self, u, v, depth_m):
        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        x = ((float(u) - cx) / fx) * depth_m
        y = ((float(v) - cy) / fy) * depth_m
        return np.array([x, y, depth_m], dtype=np.float64)

    @staticmethod
    def point_msg(point):
        msg = Point()
        msg.x = float(point[0])
        msg.y = float(point[1])
        msg.z = float(point[2])
        return msg

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
        odom.header.frame_id = "odom"
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
        self.last_marker_odom = odom
        self.last_marker_pose = pose

        visible = Bool()
        visible.data = True
        self.visible_pub.publish(visible)

    def publish_visible_timeout(self):
        visible = Bool()
        visible.data = (time.monotonic() - self.last_seen_t) <= self.args.visible_timeout_s
        if not visible.data and self.args.enable_fov_fallback:
            visible.data = self.try_publish_fov_fallback()
        if visible.data and self.last_marker_odom is not None:
            stamp = self.node.get_clock().now().to_msg()
            self.last_marker_odom.header.stamp = stamp
            self.last_marker_pose.header.stamp = stamp
            self.odom_pub.publish(self.last_marker_odom)
            self.pose_pub.publish(self.last_marker_pose)
        self.visible_pub.publish(visible)
        if not visible.data and self.last_bbox_frame_id:
            self.clear_bbox_markers()

    def clear_bbox_markers(self):
        if self.last_bbox_frame_id:
            self.bbox_marker_pub.publish(self.delete_all_markers(self.last_bbox_frame_id))
            self.last_bbox_frame_id = ""

    @staticmethod
    def delete_all_markers(frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.action = Marker.DELETEALL
        array = MarkerArray()
        array.markers = [marker]
        return array

    def try_publish_fov_fallback(self):
        if self.drone is None or self.truth_marker is None:
            return False

        rel = np.array(
            [
                self.truth_marker["x"] - self.drone["x"],
                self.truth_marker["y"] - self.drone["y"],
                self.truth_marker["z"] - self.drone["z"],
            ],
            dtype=np.float64,
        )
        horizontal = float(np.linalg.norm(rel[:2]))
        distance = float(np.linalg.norm(rel))
        if horizontal < 1e-3 or distance > self.args.fov_fallback_max_range_m:
            return False

        yaw = self.drone["yaw"] + self.args.yaw_offset_rad
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float64)
        right = np.array([math.sin(yaw), -math.cos(yaw)], dtype=np.float64)
        forward_m = float(np.dot(rel[:2], forward))
        right_m = float(np.dot(rel[:2], right))
        if forward_m <= self.args.fov_fallback_min_forward_m:
            return False

        bearing_rad = math.atan2(right_m, max(1e-6, forward_m))
        pitch_down_rad = math.atan2(self.drone["z"] - self.truth_marker["z"], horizontal)

        half_h = self.args.camera_horizontal_fov_rad * 0.5
        half_v = 0.5 * 2.0 * math.atan(
            math.tan(self.args.camera_horizontal_fov_rad * 0.5)
            * (self.args.camera_image_height / self.args.camera_image_width)
        )
        boresight_pitch_down = math.radians(self.args.camera_pitch_down_deg)
        in_fixed_fov = (
            abs(bearing_rad) <= half_h * self.args.fov_fallback_margin
            and abs(pitch_down_rad - boresight_pitch_down) <= half_v * self.args.fov_fallback_margin
        )

        if not in_fixed_fov:
            return False

        pos = np.array([self.truth_marker["x"], self.truth_marker["y"], self.truth_marker["z"]], dtype=np.float64)
        stamp = self.node.get_clock().now().to_msg()
        self.publish_marker(pos, stamp)
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/iris")
    parser.add_argument("--image-topic", default=None)
    parser.add_argument("--camera-info-topic", default=None)
    parser.add_argument("--odom-topic", default=None)
    parser.add_argument("--marker-odom-topic", default="/landing_marker/odom")
    parser.add_argument("--marker-pose-topic", default="/landing_marker/pose")
    parser.add_argument("--marker-visible-topic", default="/landing_marker/visible")
    parser.add_argument("--bbox-topic", default="/landing_marker/bbox")
    parser.add_argument("--bbox-marker-topic", default="/landing_marker/bbox_markers")
    parser.add_argument("--servo-error-topic", default="/landing_marker/visual_servo_error")
    parser.add_argument("--annotated-image-topic", default="/landing_marker/annotated_image")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--marker-size-m", type=float, default=1.0)
    parser.add_argument(
        "--marker-sizes",
        default="",
        help="Comma-separated ArUco physical sizes, for example '0:0.42,1:0.20,5:0.12'.",
    )
    parser.add_argument(
        "--marker-center-offsets",
        default="0:0.000:0.000,1:0.420:0.000,2:-0.420:0.000,3:0.000:-0.420,4:0.000:0.420,5:0.471:-0.471,6:-0.471:-0.471,7:0.471:0.471,8:-0.471:0.471",
        help="Comma-separated offsets from each detected marker center to pad center in marker-local meters: id:x:y.",
    )
    parser.add_argument("--visible-timeout-s", type=float, default=5.0)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=55.0)
    parser.add_argument("--camera-roll-deg", type=float, default=135.0)
    parser.add_argument("--yaw-offset-rad", type=float, default=0.0)
    parser.add_argument("--z-offset-m", type=float, default=0.0)
    parser.add_argument("--marker-ground-z", type=float, default=0.03)
    parser.add_argument("--clamp-ground-z", action="store_true", default=True)
    parser.add_argument("--enable-fov-fallback", action="store_true")
    parser.add_argument("--fov-fallback-marker-topic", default="/landing_marker/ground_truth/odom")
    parser.add_argument("--fov-fallback-max-range-m", type=float, default=25.0)
    parser.add_argument("--fov-fallback-min-forward-m", type=float, default=0.2)
    parser.add_argument("--fov-fallback-margin", type=float, default=0.95)
    parser.add_argument("--camera-horizontal-fov-rad", type=float, default=2.0)
    parser.add_argument("--camera-image-width", type=float, default=640.0)
    parser.add_argument("--camera-image-height", type=float, default=480.0)
    args = parser.parse_args()

    rclpy.init()
    detector = ArucoCameraDetector(args)
    try:
        rclpy.spin(detector.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        detector.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
