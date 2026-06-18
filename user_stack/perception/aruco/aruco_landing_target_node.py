#!/usr/bin/env python3
"""Moving ArUco-style landing target for Gazebo / ROS 2 experiments.

The node owns the target kinematics used by MATLAB:
  * initial target position is 10 m in front of the first received vehicle odom
  * target can move along a line, circle, waypoint route, or randomized bounded wander path
  * speed and acceleration/yaw-rate commands are randomized periodically
  * target state is published as nav_msgs/Odometry
  * Gazebo visual pose is updated through gz transport when enabled

In camera-detection mode this node publishes ground-truth marker motion on
/landing_marker/ground_truth/* while aruco_camera_detector_node.py publishes
the camera-based /landing_marker/* estimate consumed by MATLAB.
"""

import argparse
import math
import os
import random
import re
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def parse_pose_v_model_pose(text, model_name):
    needle = f'name: "{model_name}"'
    index = text.find(needle)
    if index < 0:
        return None
    next_pose = text.find("\npose {", index + len(needle))
    segment = text[index:] if next_pose < 0 else text[index:next_pose]
    position = parse_vector_block(segment, "position")
    orientation = parse_vector_block(segment, "orientation")
    if position is None:
        return None
    if orientation is None:
        yaw = 0.0
    else:
        yaw = quat_to_yaw(
            orientation.get("x", 0.0),
            orientation.get("y", 0.0),
            orientation.get("z", 0.0),
            orientation.get("w", 1.0),
        )
    return (position.get("x", 0.0), position.get("y", 0.0), position.get("z", 0.0), yaw)


def parse_vector_block(text, field):
    match = re.search(rf"\b{re.escape(field)}\s*\{{(?P<body>.*?)\}}", text, re.S)
    if not match:
        return None
    values = {}
    for axis, value in re.findall(r"\b([xyzw])\s*:\s*([-+0-9.eE]+)", match.group("body")):
        try:
            parsed = float(value)
        except ValueError:
            return None
        if not math.isfinite(parsed):
            return None
        values[axis] = parsed
    return values


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_waypoints(spec):
    waypoints = []
    for item in spec.split(";"):
        item = item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split(",")]
        if len(parts) != 2:
            raise ValueError(f"Invalid waypoint {item!r}; expected forward,lateral")
        forward_m = float(parts[0])
        lateral_m = float(parts[1])
        if not math.isfinite(forward_m) or not math.isfinite(lateral_m):
            raise ValueError(f"Invalid waypoint {item!r}; values must be finite")
        waypoints.append((forward_m, lateral_m))
    if len(waypoints) < 2:
        raise ValueError("At least two waypoints are required")
    return waypoints


class MovingLandingTarget:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("aruco_landing_target")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.odom_sub = self.node.create_subscription(
            Odometry, args.drone_odom_topic, self.on_drone_odom, qos
        )
        self.odom_pub = self.node.create_publisher(Odometry, args.marker_odom_topic, 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, args.marker_pose_topic, 10)
        self.visible_pub = self.node.create_publisher(Bool, args.marker_visible_topic, 10)
        self.touchdown_pub = self.node.create_publisher(Bool, args.touchdown_topic, latched_qos)
        self.touchdown_metrics_pub = self.node.create_publisher(Vector3Stamped, args.touchdown_metrics_topic, latched_qos)
        self.box_marker_pub = self.node.create_publisher(MarkerArray, args.box_marker_topic, 10)
        # Publish Twist on ROS2 so ros_gz bridge can forward to /model/.../cmd_vel
        self.gz_twist_pub = self.node.create_publisher(Twist, f"/model/{args.model_name}/cmd_vel", 10)
        self.motion_enable_sub = self.node.create_subscription(
            Bool, args.motion_enable_topic, self.on_motion_enable, latched_qos
        )

        self.drone = None
        self.initialized = False
        self.center_x = 0.0
        self.center_y = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.center_forward_offset = 0.0
        self.forward_x = 1.0
        self.forward_y = 0.0
        self.lateral_x = 0.0
        self.lateral_y = 1.0
        self.yaw = 0.0
        self.motion_s = 0.0
        self.circle_theta = 0.0
        self.circle_omega = 0.0
        self.circle_alpha = 0.0
        self.waypoints_local = parse_waypoints(args.waypoints)
        self.waypoints_world = []
        self.waypoint_index = 1
        self.waypoint_yaw_rate = 0.0
        self.random_target_speed = 0.0
        self.random_target_yaw_rate = 0.0
        self.random_yaw_rate = 0.0
        self.z = args.marker_z
        self.x = 0.0
        self.y = 0.0
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_z = args.marker_z
        self.actual_yaw = 0.0
        self.actual_pose_source = "commanded"
        self.last_actual_pose_read_t = 0.0
        self.vs = 0.0
        self.accel_s = 0.0
        self.last_t = time.monotonic()
        self.last_accel_change_t = self.last_t
        self.last_random_change_t = self.last_t
        self.last_gz_t = 0.0
        self.gz_pose_initialized = False
        self.gz_initial_pose_sent = False
        self.gz_pose_updates = 0
        self.gz_pose_failures = 0
        self.last_gz_log_t = 0.0
        self.gz_velocity_updates = 0
        self.gz_velocity_failures = 0
        self.last_gz_velocity_log_t = 0.0
        self.gz_velocity_warned = False
        self.wheel_force_gain = float(os.getenv("OBDM_ROVER_FORCE_GAIN", "2.2"))
        self.wheel_turn_gain = float(os.getenv("OBDM_ROVER_TURN_GAIN", "1.8"))
        self.wheel_force_max = float(os.getenv("OBDM_ROVER_FORCE_MAX", "6.0"))
        self.wheel_cmdvel_max = float(os.getenv("OBDM_ROVER_CMDVEL_MAX", "1.6"))
        self.wheel_yawrate_max = float(os.getenv("OBDM_ROVER_YAWRATE_MAX", "1.2"))
        self.start_t = time.monotonic()
        self.takeoff_detected = False
        self.contact_armed = False
        self.contact_arming_z_threshold = max(
            args.contact_arming_z_threshold,
            args.takeoff_z_threshold,
            args.touchdown_ground_z_threshold + 0.5,
        )
        self.motion_enabled = not args.wait_for_takeoff
        self.motion_release_received = not args.wait_for_motion_enable
        self.touchdown_detected = False
        self.touchdown_metrics = (math.nan, math.nan, math.nan)
        self.gazebo_contact_samples = 0
        self.last_zero_cmd_t = 0.0
        self.rng = random.Random(args.seed if args.seed >= 0 else None)
        self.timer = self.node.create_timer(1.0 / args.rate, self.step)

        # Optional timer to detect Gazebo readiness and auto-release rover motion
        self.gz_ready_timer = None
        self.start_on_gazebo_up = getattr(args, "start_on_gazebo_up", False)
        if self.start_on_gazebo_up:
            # check Gazebo readiness at 2 Hz
            self.gz_ready_timer = self.node.create_timer(0.5, self.check_gazebo_ready)

        self.node.get_logger().info(
            f"Moving ArUco target publishing {args.marker_odom_topic}; "
            f"world={args.world}, model={args.model_name}, wait_for_takeoff={args.wait_for_takeoff}, "
            f"wait_for_motion_enable={args.wait_for_motion_enable}, motion_enable_topic={args.motion_enable_topic}"
        )

        if args.initial_x is not None and args.initial_y is not None:
            self.initialize_from_pose(args.initial_x, args.initial_y, args.initial_yaw_rad, "configured MATLAB initial position")

    def on_motion_enable(self, msg):
        if bool(msg.data):
            if not self.motion_release_received:
                self.node.get_logger().info(
                    f"External rover motion release received on {self.args.motion_enable_topic}"
                )
            self.motion_release_received = True
            if self.takeoff_detected or not self.args.wait_for_takeoff:
                self.enable_motion_if_ready("external release")
        else:
            self.node.get_logger().info(
                f"External rover motion hold received on {self.args.motion_enable_topic}; stopping target motion"
            )
            self.motion_release_received = False
            self.stop_motion()

    def on_drone_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        self.drone = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "vx": float(v.x),
            "vy": float(v.y),
            "vz": float(v.z),
            "yaw": yaw,
        }
        if self.args.one_way_on_takeoff and (not self.takeoff_detected):
            if self.drone["z"] >= self.args.takeoff_z_threshold:
                self.mark_takeoff_detected()
        if not self.initialized:
            self.initialize_from_pose(self.drone["x"], self.drone["y"], yaw, "vehicle odom")
        if (not self.takeoff_detected) and self.drone["z"] >= self.args.takeoff_z_threshold:
            self.mark_takeoff_detected()
        if (not self.contact_armed) and self.drone["z"] >= self.contact_arming_z_threshold:
            self.contact_armed = True
            self.node.get_logger().info(
                f"Landing contact detection armed "
                f"(z={self.drone['z']:.2f} >= {self.contact_arming_z_threshold:.2f})"
            )
        self.check_touchdown()

    def mark_takeoff_detected(self):
        self.takeoff_detected = True
        self.enable_motion_if_ready("takeoff detected")

    def enable_motion_if_ready(self, reason):
        if self.args.wait_for_takeoff and not self.takeoff_detected:
            return
        if self.args.wait_for_motion_enable and not self.motion_release_received:
            if reason == "takeoff detected":
                self.node.get_logger().info(
                    f"Takeoff detected (z={self.drone['z']:.2f} >= {self.args.takeoff_z_threshold:.2f}); "
                    f"holding target motion until external release on {self.args.motion_enable_topic}"
                )
            return
        if self.motion_enabled:
            return
        self.motion_enabled = True
        mode = "forward-only mode" if self.args.one_way_on_takeoff else "target motion"
        self.node.get_logger().info(
            f"Enabling {mode} after {reason}"
        )

    def check_touchdown(self):
        if self.touchdown_detected or not self.initialized or self.drone is None:
            return
        if not self.takeoff_detected:
            return
        if not self.contact_armed:
            return
        self.update_actual_pose_from_gazebo()
        dx = self.drone["x"] - self.actual_x
        dy = self.drone["y"] - self.actual_y
        horizontal_error = math.sqrt(dx * dx + dy * dy)
        gazebo_contact_ok = self.read_gazebo_pad_contact()
        if gazebo_contact_ok:
            self.gazebo_contact_samples += 1
        else:
            self.gazebo_contact_samples = 0
        gazebo_contact_confirmed = self.gazebo_contact_samples >= self.args.touchdown_contact_min_samples
        altitude_ok = self.drone["z"] <= self.args.touchdown_z_threshold
        position_ok = horizontal_error <= self.args.touchdown_xy_tolerance
        speed_ok = abs(self.drone["vz"]) <= self.args.touchdown_max_abs_vz
        ground_position_ok = horizontal_error <= self.args.touchdown_ground_xy_tolerance
        ground_contact_ok = (
            self.drone["z"] <= self.args.touchdown_ground_z_threshold
            and ground_position_ok
        )
        threshold_pad_contact_ok = (
            not self.args.touchdown_require_gazebo_contact
            and altitude_ok
            and position_ok
            and speed_ok
        )
        detected_contact = gazebo_contact_confirmed or threshold_pad_contact_ok
        if not self.args.touchdown_require_gazebo_contact:
            detected_contact = detected_contact or ground_contact_ok
        if detected_contact:
            self.touchdown_detected = True
            self.touchdown_metrics = (self.drone["z"], horizontal_error, self.drone["vz"])
            contact_kind = "landing pad" if gazebo_contact_confirmed or position_ok else "ground"
            contact_source = "gazebo_contact" if gazebo_contact_confirmed else "threshold"
            self.node.get_logger().info(
                f"{contact_kind} contact detected: z={self.drone['z']:.2f}, "
                f"xy_error={horizontal_error:.2f}, vz={self.drone['vz']:.2f}, "
                f"gazebo_contact={gazebo_contact_confirmed}, source={contact_source}, "
                f"ground_contact={ground_contact_ok}, ground_xy_ok={ground_position_ok}; "
                "stopping target motion"
            )
            self.stop_motion()

    def read_gazebo_pad_contact(self):
        if not self.args.touchdown_use_gazebo_contact:
            return False
        topic = self.args.touchdown_contact_topic
        try:
            result = subprocess.run(
                [
                    "timeout",
                    f"{max(self.args.touchdown_contact_read_timeout_s, 0.03):.3f}",
                    "gz",
                    "topic",
                    "-e",
                    "-t",
                    topic,
                    "-n",
                    "1",
                ],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=max(0.1, self.args.touchdown_contact_read_timeout_s + 0.1),
            )
        except Exception:
            return False
        if result.returncode != 0 or not result.stdout:
            return False
        text = result.stdout
        pad_token = self.args.touchdown_contact_pad_collision
        vehicle_tokens = [
            token.strip()
            for token in self.args.touchdown_contact_vehicle_filter.split(",")
            if token.strip()
        ]
        if pad_token and pad_token not in text:
            return False
        return any(token in text for token in vehicle_tokens)

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        return quat_to_yaw(x, y, z, w)

    @staticmethod
    def yaw_to_quat(yaw):
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        return 0.0, 0.0, qz, qw

    def position_from_motion(self):
        if self.args.motion_mode == "circle":
            radius = self.args.circle_radius_m
            radial_x = math.cos(self.circle_theta) * self.forward_x + math.sin(self.circle_theta) * self.lateral_x
            radial_y = math.cos(self.circle_theta) * self.forward_y + math.sin(self.circle_theta) * self.lateral_y
            return (
                self.center_x + radius * radial_x,
                self.center_y + radius * radial_y,
            )
        if self.args.motion_mode in ("random", "waypoints"):
            return (self.x, self.y)
        return (
            self.center_x + self.motion_s * self.forward_x,
            self.center_y + self.motion_s * self.forward_y,
        )

    def initialize_from_pose(self, x, y, yaw, source):
        self.yaw = yaw
        self.forward_x = math.cos(yaw)
        self.forward_y = math.sin(yaw)
        self.lateral_x = -math.sin(yaw)
        self.lateral_y = math.cos(yaw)
        self.origin_x = float(x)
        self.origin_y = float(y)
        self.center_forward_offset = clamp(
            self.args.spawn_forward_m,
            self.args.min_forward_offset_m,
            self.args.max_forward_offset_m,
        )
        self.center_x = x + self.center_forward_offset * self.forward_x
        self.center_y = y + self.center_forward_offset * self.forward_y
        self.motion_s = 0.0
        self.circle_theta = math.radians(self.args.circle_start_deg)
        self.circle_omega = self.args.circle_angular_speed
        self.circle_alpha = 0.0
        self.waypoints_world = [
            (
                self.center_x + forward_m * self.forward_x + lateral_m * self.lateral_x,
                self.center_y + forward_m * self.forward_y + lateral_m * self.lateral_y,
            )
            for forward_m, lateral_m in self.waypoints_local
        ]
        self.waypoint_index = 1
        self.waypoint_yaw_rate = 0.0
        self.random_target_speed = self.rng.uniform(self.args.random_min_speed, self.args.random_max_speed)
        self.random_target_yaw_rate = self.rng.uniform(-self.args.random_yawrate_max, self.args.random_yawrate_max)
        self.random_yaw_rate = 0.0
        self.last_random_change_t = self.last_t
        min_s, max_s = self.motion_limits()
        self.motion_s = clamp(self.motion_s, min_s, max_s)
        if self.args.motion_mode == "waypoints":
            self.x, self.y = self.waypoints_world[0]
            if len(self.waypoints_world) > 1:
                next_x, next_y = self.waypoints_world[1]
                self.yaw = math.atan2(next_y - self.y, next_x - self.x)
        else:
            self.x = self.center_x + self.motion_s * self.forward_x
            self.y = self.center_y + self.motion_s * self.forward_y
        self.x, self.y = self.position_from_motion()
        self.actual_x = self.x
        self.actual_y = self.y
        self.actual_z = self.z
        self.actual_yaw = yaw
        self.actual_pose_source = "commanded"
        if self.args.motion_mode == "random":
            self.vs = self.rng.uniform(self.args.random_min_speed, self.args.random_max_speed)
        elif self.args.motion_mode == "waypoints":
            initial_waypoint_speed = self.random_target_speed if self.args.randomize_waypoint_speed else self.args.waypoint_speed
            self.vs = clamp(initial_waypoint_speed, 0.0, self.args.max_speed)
        else:
            self.vs = self.rng.uniform(-self.args.initial_speed, self.args.initial_speed)
        self.accel_s = self.random_accel()
        self.initialized = True
        self.node.get_logger().info(
            "Target initialized "
            f"from {source} at x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
            f"yaw={math.degrees(yaw):.1f}deg, forward=({self.forward_x:.2f},{self.forward_y:.2f}), "
            f"motion_mode={self.args.motion_mode}, forward_offset_limits=[{self.args.min_forward_offset_m:.2f},{self.args.max_forward_offset_m:.2f}]m"
        )

    def motion_limits(self):
        configured_range = min(abs(self.args.motion_range_m), abs(self.args.max_motion_range_m))
        min_s = max(-configured_range, self.args.min_forward_offset_m - self.center_forward_offset)
        max_s = min(configured_range, self.args.max_forward_offset_m - self.center_forward_offset)
        if min_s > max_s:
            min_s = max_s = 0.0
        return min_s, max_s

    def random_accel(self):
        mag = self.rng.uniform(self.args.accel_min, self.args.accel_max)
        sign = -1.0 if self.rng.random() < 0.5 else 1.0
        return sign * mag

    def update_circle_motion(self, dt):
        now = time.monotonic()
        if self.args.circle_accel_change_period > 0.0 and now - self.last_accel_change_t > self.args.circle_accel_change_period:
            self.circle_alpha = self.rng.uniform(-self.args.circle_angular_accel_max, self.args.circle_angular_accel_max)
            self.last_accel_change_t = now
        desired = self.args.circle_angular_speed
        restoring = self.args.circle_speed_gain * (desired - self.circle_omega)
        self.circle_omega = clamp(
            self.circle_omega + (restoring + self.circle_alpha) * dt,
            -abs(self.args.circle_max_angular_speed),
            abs(self.args.circle_max_angular_speed),
        )
        if abs(self.circle_omega) < self.args.circle_min_angular_speed:
            self.circle_omega = math.copysign(self.args.circle_min_angular_speed, desired if desired != 0.0 else 1.0)
        self.circle_theta = (self.circle_theta + self.circle_omega * dt) % (2.0 * math.pi)
        self.vs = abs(self.args.circle_radius_m * self.circle_omega)
        tangent_sign = 1.0 if self.circle_omega >= 0.0 else -1.0
        tangent_x = tangent_sign * (-math.sin(self.circle_theta) * self.forward_x + math.cos(self.circle_theta) * self.lateral_x)
        tangent_y = tangent_sign * (-math.sin(self.circle_theta) * self.forward_y + math.cos(self.circle_theta) * self.lateral_y)
        self.yaw = math.atan2(tangent_y, tangent_x)
        self.x, self.y = self.position_from_motion()

    def update_random_motion(self, dt):
        now = time.monotonic()
        if self.args.random_change_period > 0.0 and now - self.last_random_change_t > self.args.random_change_period:
            self.random_target_speed = self.rng.uniform(self.args.random_min_speed, self.args.random_max_speed)
            self.random_target_yaw_rate = self.rng.uniform(-self.args.random_yawrate_max, self.args.random_yawrate_max)
            self.last_random_change_t = now

        dx = self.x - self.center_x
        dy = self.y - self.center_y
        forward_offset = dx * self.forward_x + dy * self.forward_y
        lateral_offset = dx * self.lateral_x + dy * self.lateral_y
        min_s, max_s = self.motion_limits()
        lateral_limit = max(0.05, self.args.random_lateral_range_m)

        over_forward = forward_offset < min_s or forward_offset > max_s
        over_lateral = abs(lateral_offset) > lateral_limit
        if over_forward or over_lateral:
            desired_heading = math.atan2(self.center_y - self.y, self.center_x - self.x)
            heading_error = self.wrap_angle(desired_heading - self.yaw)
            yaw_target = clamp(
                self.args.random_boundary_yaw_gain * heading_error,
                -self.args.random_yawrate_max,
                self.args.random_yawrate_max,
            )
        else:
            yaw_target = self.random_target_yaw_rate

        speed_alpha = clamp(self.args.random_speed_smoothing * dt, 0.0, 1.0)
        yaw_alpha = clamp(self.args.random_yawrate_smoothing * dt, 0.0, 1.0)
        self.vs += (self.random_target_speed - self.vs) * speed_alpha
        self.random_yaw_rate += (yaw_target - self.random_yaw_rate) * yaw_alpha
        self.vs = clamp(self.vs, 0.0, self.args.random_max_speed)
        self.random_yaw_rate = clamp(self.random_yaw_rate, -self.args.random_yawrate_max, self.args.random_yawrate_max)

        self.yaw = self.wrap_angle(self.yaw + self.random_yaw_rate * dt)
        self.x += self.vs * math.cos(self.yaw) * dt
        self.y += self.vs * math.sin(self.yaw) * dt

        dx = self.x - self.center_x
        dy = self.y - self.center_y
        forward_offset = clamp(dx * self.forward_x + dy * self.forward_y, min_s, max_s)
        lateral_offset = clamp(dx * self.lateral_x + dy * self.lateral_y, -lateral_limit, lateral_limit)
        self.x = self.center_x + forward_offset * self.forward_x + lateral_offset * self.lateral_x
        self.y = self.center_y + forward_offset * self.forward_y + lateral_offset * self.lateral_y

    def update_waypoint_motion(self, dt):
        if len(self.waypoints_world) < 2:
            self.vs = 0.0
            self.waypoint_yaw_rate = 0.0
            return

        if self.args.randomize_waypoint_speed:
            now = time.monotonic()
            if self.args.random_change_period > 0.0 and now - self.last_random_change_t > self.args.random_change_period:
                self.random_target_speed = self.rng.uniform(self.args.random_min_speed, self.args.random_max_speed)
                self.last_random_change_t = now

        target_x, target_y = self.waypoints_world[self.waypoint_index]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)
        while distance <= self.args.waypoint_reached_radius and len(self.waypoints_world) > 1:
            if self.waypoint_index >= len(self.waypoints_world) - 1:
                if self.args.waypoint_loop:
                    self.waypoint_index = 0
                else:
                    self.vs = 0.0
                    self.waypoint_yaw_rate = 0.0
                    return
            else:
                self.waypoint_index += 1
            target_x, target_y = self.waypoints_world[self.waypoint_index]
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx)
        heading_error = self.wrap_angle(desired_yaw - self.yaw)
        desired_yaw_rate = clamp(
            self.args.waypoint_heading_gain * heading_error,
            -self.args.waypoint_yawrate_max,
            self.args.waypoint_yawrate_max,
        )
        yaw_alpha = clamp(self.args.waypoint_yawrate_smoothing * dt, 0.0, 1.0)
        self.waypoint_yaw_rate += (desired_yaw_rate - self.waypoint_yaw_rate) * yaw_alpha
        self.yaw = self.wrap_angle(self.yaw + self.waypoint_yaw_rate * dt)

        turn_slowdown = clamp(1.0 - 0.35 * abs(heading_error), 0.55, 1.0)
        base_speed = self.random_target_speed if self.args.randomize_waypoint_speed else self.args.waypoint_speed
        target_speed = clamp(base_speed * turn_slowdown, 0.0, self.args.max_speed)
        speed_alpha = clamp(self.args.waypoint_speed_smoothing * dt, 0.0, 1.0)
        self.vs += (target_speed - self.vs) * speed_alpha
        self.vs = clamp(self.vs, 0.0, self.args.max_speed)
        self.x += self.vs * math.cos(self.yaw) * dt
        self.y += self.vs * math.sin(self.yaw) * dt

    def step(self):
        if not self.initialized:
            if time.monotonic() - self.start_t >= self.args.odom_init_timeout_s:
                self.initialize_from_pose(0.0, 0.0, 0.0, "origin fallback")
            else:
                return

        now = time.monotonic()
        dt = clamp(now - self.last_t, 0.0, 0.25)
        self.last_t = now

        self.check_touchdown()
        if self.touchdown_detected or not self.motion_enabled:
            self.vs = 0.0
            visible = self.is_visible()
            self.publish_state(visible)
            if self.args.update_gazebo and now - self.last_gz_t >= 1.0 / self.args.gz_rate:
                if not self.gz_initial_pose_sent:
                    self.publish_gazebo_pose()
                    self.gz_pose_initialized = True
                    self.gz_initial_pose_sent = True
                self.publish_gazebo_velocity(0.0, 0.0)
                self.last_gz_t = now
            return

        if self.args.motion_mode == "circle":
            self.update_circle_motion(dt)
        elif self.args.motion_mode == "random":
            self.update_random_motion(dt)
        elif self.args.motion_mode == "waypoints":
            self.update_waypoint_motion(dt)
        elif now - self.last_accel_change_t > self.args.accel_change_period:
            self.accel_s = self.random_accel()
            self.last_accel_change_t = now

        if self.args.motion_mode not in ("circle", "random", "waypoints"):
            one_way_active = self.args.one_way_on_takeoff and self.takeoff_detected
            min_s, max_s = self.motion_limits()
            if one_way_active:
                # After takeoff, keep rover motion forward-only (no return path).
                self.accel_s = abs(self.accel_s)
                self.vs = abs(clamp(self.vs + self.accel_s * dt, 0.0, self.args.max_speed))
                self.motion_s = self.motion_s + self.vs * dt
                if self.motion_s > max_s:
                    self.motion_s = max_s
                    self.vs = 0.0
                    self.accel_s = 0.0
            else:
                self.vs = clamp(self.vs + self.accel_s * dt, -self.args.max_speed, self.args.max_speed)
                self.motion_s = self.motion_s + self.vs * dt

                if self.motion_s < min_s:
                    self.motion_s = min_s
                    self.vs = abs(self.vs) + self.rng.uniform(0.05, 0.25)
                    self.accel_s = abs(self.random_accel())
                elif self.motion_s > max_s:
                    self.motion_s = max_s
                    self.vs = -abs(self.vs) - self.rng.uniform(0.05, 0.25)
                    self.accel_s = -abs(self.random_accel())
            self.x, self.y = self.position_from_motion()

        visible = self.is_visible()
        self.publish_state(visible)

        if self.args.update_gazebo and now - self.last_gz_t >= 1.0 / self.args.gz_rate:
            if not self.gz_initial_pose_sent:
                self.publish_gazebo_pose()
                self.gz_pose_initialized = True
                self.gz_initial_pose_sent = True
            self.publish_gazebo_velocity()
            self.last_gz_t = now

    def is_visible(self):
        self.update_actual_pose_from_gazebo()
        if self.drone is None:
            return False
        dx = self.actual_x - self.drone["x"]
        dy = self.actual_y - self.drone["y"]
        dz = self.actual_z - self.drone["z"]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        reference_forward_x = math.cos(self.drone["yaw"])
        reference_forward_y = math.sin(self.drone["yaw"])
        reference_lateral_x = -reference_forward_y
        reference_lateral_y = reference_forward_x
        forward = dx * reference_forward_x + dy * reference_forward_y
        lateral = dx * reference_lateral_x + dy * reference_lateral_y
        return (
            forward >= self.args.visible_min_forward
            and forward <= self.args.visible_max_forward
            and abs(lateral) <= self.args.visible_half_width
            and distance <= self.args.visible_range
        )

    def publish_state(self, visible):
        self.update_actual_pose_from_gazebo()
        stamp = self.node.get_clock().now().to_msg()
        if self.actual_pose_source == "gazebo":
            marker_yaw = self.actual_yaw
        else:
            marker_yaw = self.current_motion_yaw()
        qx, qy, qz, qw = self.yaw_to_quat(marker_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "aruco_landing_marker"
        odom.pose.pose.position.x = float(self.actual_x)
        odom.pose.pose.position.y = float(self.actual_y)
        odom.pose.pose.position.z = float(self.actual_z)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        if self.args.motion_mode == "circle":
            vx = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_x + math.cos(self.circle_theta) * self.lateral_x
            )
            vy = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_y + math.cos(self.circle_theta) * self.lateral_y
            )
        elif self.args.motion_mode == "random":
            vx = self.vs * math.cos(self.yaw)
            vy = self.vs * math.sin(self.yaw)
        elif self.args.motion_mode == "waypoints":
            vx = self.vs * math.cos(self.yaw)
            vy = self.vs * math.sin(self.yaw)
        else:
            vx = self.vs * self.forward_x
            vy = self.vs * self.forward_y
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = 0.0
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        vis = Bool()
        vis.data = bool(visible)
        self.visible_pub.publish(vis)

        touchdown = Bool()
        touchdown.data = bool(self.touchdown_detected)
        self.touchdown_pub.publish(touchdown)

        metrics = Vector3Stamped()
        metrics.header.stamp = stamp
        metrics.header.frame_id = "landing_marker"
        metrics.vector.x = float(self.touchdown_metrics[0])
        metrics.vector.y = float(self.touchdown_metrics[1])
        metrics.vector.z = float(self.touchdown_metrics[2])
        self.touchdown_metrics_pub.publish(metrics)

        self.box_marker_pub.publish(self.build_box_markers(stamp, odom.pose.pose.orientation))

    def update_actual_pose_from_gazebo(self):
        if not self.args.use_gazebo_pose_for_rviz:
            self.actual_x = self.x
            self.actual_y = self.y
            self.actual_z = self.z
            self.actual_yaw = self.current_motion_yaw()
            self.actual_pose_source = "commanded"
            return
        now = time.monotonic()
        if now - self.last_actual_pose_read_t < 1.0 / max(self.args.gz_pose_read_rate, 0.1):
            return
        self.last_actual_pose_read_t = now
        pose = self.read_gazebo_model_pose()
        if pose is None:
            if self.actual_pose_source != "gazebo":
                self.actual_x = self.x
                self.actual_y = self.y
                self.actual_z = self.z
                self.actual_yaw = self.current_motion_yaw()
            return
        self.actual_x, self.actual_y, model_z, self.actual_yaw = pose
        self.actual_z = model_z + max(0.0, self.args.marker_z - self.args.model_z)
        self.actual_pose_source = "gazebo"

    def read_gazebo_model_pose(self):
        topic = f"/world/{self.args.world}/pose/info"
        try:
            result = subprocess.run(
                [
                    "timeout",
                    f"{max(self.args.gz_pose_read_timeout_s, 0.05):.3f}",
                    "gz",
                    "topic",
                    "-e",
                    "-t",
                    topic,
                    "-n",
                    "1",
                ],
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=max(0.2, self.args.gz_pose_read_timeout_s + 0.2),
            )
        except Exception:
            return None
        if result.returncode != 0 or not result.stdout:
            return None
        return parse_pose_v_model_pose(result.stdout, self.args.model_name)

    def current_motion_yaw(self):
        if self.args.motion_mode == "circle":
            vx = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_x + math.cos(self.circle_theta) * self.lateral_x
            )
            vy = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_y + math.cos(self.circle_theta) * self.lateral_y
            )
            if math.hypot(vx, vy) > 1e-6:
                return math.atan2(vy, vx)
        elif self.args.motion_mode == "random":
            return self.yaw
        elif self.args.motion_mode == "waypoints":
            return self.yaw
        elif abs(self.vs) > 1e-6:
            return self.yaw if self.vs >= 0.0 else self.wrap_angle(self.yaw + math.pi)
        return self.yaw

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def build_box_markers(self, stamp, orientation):
        marker_array = MarkerArray()
        marker_array.markers = [
            self.make_box_marker(
                stamp,
                marker_id=1,
                ns="landing_target_rover",
                z=float(self.actual_z - self.args.marker_z + self.args.model_z + 0.5 * self.args.rover_box_height_m),
                sx=float(self.args.rover_box_length_m),
                sy=float(self.args.rover_box_width_m),
                sz=float(self.args.rover_box_height_m),
                rgba=(0.10, 0.45, 0.95, 0.80),
                orientation=orientation,
            ),
            self.make_box_marker(
                stamp,
                marker_id=2,
                ns="landing_target_pad",
                z=float(self.actual_z + 0.5 * self.args.landing_pad_box_height_m),
                sx=float(self.args.landing_pad_box_length_m),
                sy=float(self.args.landing_pad_box_width_m),
                sz=float(self.args.landing_pad_box_height_m),
                rgba=(0.15, 0.90, 0.25, 0.88),
                orientation=orientation,
            ),
        ]
        return marker_array

    def make_box_marker(self, stamp, marker_id, ns, z, sx, sy, sz, rgba, orientation):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "odom"
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.actual_x)
        marker.pose.position.y = float(self.actual_y)
        marker.pose.position.z = float(z)
        marker.pose.orientation = orientation
        marker.scale.x = max(0.01, sx)
        marker.scale.y = max(0.01, sy)
        marker.scale.z = max(0.01, sz)
        marker.color.r = float(rgba[0])
        marker.color.g = float(rgba[1])
        marker.color.b = float(rgba[2])
        marker.color.a = float(rgba[3])
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def publish_gazebo_motion(self):
        if not self.gz_initial_pose_sent:
            self.publish_gazebo_pose()
            self.gz_pose_initialized = True
            self.gz_initial_pose_sent = True
        self.publish_gazebo_velocity()

    def check_gazebo_ready(self):
        # If Gazebo already has an initial pose available for the model, treat
        # that as the simulator being up and allow rover motion to start.
        try:
            pose = self.read_gazebo_model_pose()
        except Exception:
            pose = None
        if pose is not None:
            # mark that Gazebo provided a pose and release motion if configured
            if not self.motion_release_received:
                self.node.get_logger().info("Gazebo is ready: auto-releasing rover motion")
            self.motion_release_received = True
            # If configured to start on Gazebo up, bypass takeoff gating and enable motion
            if getattr(self, 'start_on_gazebo_up', False):
                if not self.motion_enabled:
                    mode = "forward-only mode" if self.args.one_way_on_takeoff else "target motion"
                    self.motion_enabled = True
                    # Ensure a small non-zero initial speed so Gazebo receives motion commands
                    try:
                        min_speed = float(getattr(self.args, 'initial_speed', 0.05))
                    except Exception:
                        min_speed = 0.05
                    self.vs = max(self.vs, min_speed)
                    self.node.get_logger().info(f"Enabling {mode} after gazebo ready (auto-start); initial_vs={self.vs:.3f}")
            elif self.takeoff_detected or not self.args.wait_for_takeoff:
                self.enable_motion_if_ready("gazebo ready")
            # stop the ready timer
            if self.gz_ready_timer is not None:
                try:
                    self.gz_ready_timer.cancel()
                except Exception:
                    pass
                self.gz_ready_timer = None

    def publish_gazebo_pose(self):
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        payload = (
            f'name: "{self.args.model_name}" '
            f'position {{x: {self.x:.6f} y: {self.y:.6f} z: {self.args.model_z:.6f}}} '
            f'orientation {{z: {qz:.9f} w: {qw:.9f}}}'
        )
        service = f"/world/{self.args.world}/set_pose"
        try:
            result = subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    service,
                    "--reqtype",
                    "gz.msgs.Pose",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    str(int(self.args.gz_service_timeout_ms)),
                    "--req",
                    payload,
                ],
                timeout=max(0.5, self.args.gz_service_timeout_ms / 1000.0 + 0.5),
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            accepted = result.returncode == 0 and "data: true" in result.stdout
            if accepted:
                self.gz_pose_updates += 1
            else:
                self.gz_pose_failures += 1
        except Exception as exc:
            self.gz_pose_failures += 1
            if self.args.verbose_gz:
                self.node.get_logger().warn(f"Gazebo marker pose update exception: {exc}")
            return

        now = time.monotonic()
        if self.args.verbose_gz or now - self.last_gz_log_t >= self.args.gz_log_period:
            status = "accepted" if accepted else "failed"
            detail = ""
            if not accepted:
                detail = f", stdout={result.stdout.strip()!r}, stderr={result.stderr.strip()!r}"
            self.node.get_logger().info(
                f"Gazebo model pose {status}: x={self.x:.2f}, y={self.y:.2f}, model_z={self.args.model_z:.2f}, marker_z={self.z:.2f}, "
                f"ok={self.gz_pose_updates}, fail={self.gz_pose_failures}{detail}"
            )
            self.last_gz_log_t = now

    def publish_gazebo_velocity(self, linear_override=None, angular_override=None):
        if linear_override is not None or angular_override is not None:
            linear_cmd = 0.0 if linear_override is None else float(linear_override)
            angular_cmd = 0.0 if angular_override is None else float(angular_override)
            world_vx = 0.0
            world_vy = 0.0
        elif self.args.motion_mode == "circle":
            linear_cmd = abs(self.args.circle_radius_m * self.circle_omega)
            angular_cmd = self.circle_omega
            world_vx = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_x + math.cos(self.circle_theta) * self.lateral_x
            )
            world_vy = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_y + math.cos(self.circle_theta) * self.lateral_y
            )
        elif self.args.motion_mode == "random":
            linear_cmd = max(0.0, self.vs)
            angular_cmd = self.random_yaw_rate
            world_vx = self.vs * math.cos(self.yaw)
            world_vy = self.vs * math.sin(self.yaw)
        elif self.args.motion_mode == "waypoints":
            linear_cmd = max(0.0, self.vs)
            angular_cmd = self.waypoint_yaw_rate
            world_vx = self.vs * math.cos(self.yaw)
            world_vy = self.vs * math.sin(self.yaw)
        else:
            linear_cmd = self.vs
            angular_cmd = 0.0
            world_vx = self.vs * self.forward_x
            world_vy = self.vs * self.forward_y
        # Publish model-frame forward speed plus yaw rate. The wheel bridge
        # converts these into differential wheel forces for the Gazebo rover.
        sent = False
        try:
            twist = Twist()
            twist.linear.x = float(linear_cmd)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(angular_cmd)
            self.gz_twist_pub.publish(twist)
            self.gz_velocity_updates += 1
            sent = True
            if self.args.verbose_gz:
                self.node.get_logger().info(
                    f"Published ROS2 Twist: model_vx={linear_cmd:.3f}, yaw_rate={angular_cmd:.3f}, world_v=({world_vx:.3f},{world_vy:.3f})"
                )
        except Exception as exc:
            self.gz_velocity_failures += 1
            if not self.gz_velocity_warned or self.args.verbose_gz:
                self.node.get_logger().warn(f"ROS2 Twist publish failed: {exc}")
                self.gz_velocity_warned = True
        self.publish_wheel_forces(linear_cmd, angular_cmd)

        now = time.monotonic()
        if self.args.verbose_gz or now - self.last_gz_velocity_log_t >= self.args.gz_log_period:
            status = "published" if sent else "failed"
            self.node.get_logger().info(
                f"Gazebo marker velocity {status}: model_vx={linear_cmd:.2f}, "
                f"yaw_rate={angular_cmd:.2f}, world_v=({world_vx:.2f},{world_vy:.2f}), ok={self.gz_velocity_updates}, fail={self.gz_velocity_failures}"
            )
            self.last_gz_velocity_log_t = now

    def stop_motion(self):
        self.motion_enabled = False
        self.vs = 0.0
        self.accel_s = 0.0
        self.publish_gazebo_velocity(0.0, 0.0)
        self.publish_wheel_forces_zero()

    def publish_wheel_forces(self, linear_cmd, angular_cmd):
        lin = clamp(float(linear_cmd), -self.wheel_cmdvel_max, self.wheel_cmdvel_max)
        yaw = clamp(float(angular_cmd), -self.wheel_yawrate_max, self.wheel_yawrate_max)
        left_force = clamp(self.wheel_force_gain * lin - self.wheel_turn_gain * yaw, -self.wheel_force_max, self.wheel_force_max)
        right_force = clamp(self.wheel_force_gain * lin + self.wheel_turn_gain * yaw, -self.wheel_force_max, self.wheel_force_max)
        wheel_forces = {
            "/model/r1_rover/joint/motor_0/cmd_force": left_force,
            "/model/r1_rover/joint/motor_1/cmd_force": left_force,
            "/model/r1_rover/joint/motor_2/cmd_force": -right_force,
            "/model/r1_rover/joint/motor_3/cmd_force": -right_force,
        }
        for topic, force in wheel_forces.items():
            try:
                subprocess.run(
                    ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {float(force)}"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=0.5,
                )
            except Exception:
                pass

    def publish_wheel_forces_zero(self):
        now = time.monotonic()
        if now - self.last_zero_cmd_t < 0.2:
            return
        self.last_zero_cmd_t = now
        wheel_topics = [
            "/model/r1_rover/joint/motor_0/cmd_force",
            "/model/r1_rover/joint/motor_1/cmd_force",
            "/model/r1_rover/joint/motor_2/cmd_force",
            "/model/r1_rover/joint/motor_3/cmd_force",
        ]
        for topic in wheel_topics:
            try:
                subprocess.run(
                    ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", "data: 0.0"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=0.5,
                )
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespace", default="/iris")
    parser.add_argument("--world", default="runway")
    parser.add_argument("--model-name", default="aruco_landing_marker")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--gz-rate", type=float, default=5.0)
    parser.add_argument("--update-gazebo", action="store_true")
    parser.add_argument("--gz-service-timeout-ms", type=float, default=3000.0)
    parser.add_argument("--gz-topic-timeout-ms", type=float, default=500.0)
    parser.add_argument("--gz-log-period", type=float, default=2.0)
    parser.add_argument("--use-gazebo-pose-for-rviz", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--gz-pose-read-rate", type=float, default=10.0)
    parser.add_argument("--gz-pose-read-timeout-s", type=float, default=0.18)
    parser.add_argument("--verbose-gz", action="store_true")
    parser.add_argument("--spawn-forward-m", type=float, default=2.5)
    parser.add_argument("--spawn-lateral-m", type=float, default=0.0)
    parser.add_argument("--initial-x", type=float, default=None)
    parser.add_argument("--initial-y", type=float, default=None)
    parser.add_argument("--initial-yaw-rad", type=float, default=0.0)
    parser.add_argument("--marker-z", type=float, default=0.04)
    parser.add_argument("--model-z", type=float, default=0.04)
    # Keep the target path bounded so it can physically interact with the
    # vehicle and remain interceptable.
    parser.add_argument("--motion-mode", choices=["line", "circle", "random", "waypoints"], default="line")
    parser.add_argument("--motion-range-m", type=float, default=1.0)
    parser.add_argument("--max-motion-range-m", type=float, default=2.0)
    parser.add_argument("--circle-radius-m", type=float, default=0.6)
    parser.add_argument("--circle-start-deg", type=float, default=0.0)
    parser.add_argument("--circle-angular-speed", type=float, default=0.24)
    parser.add_argument("--circle-min-angular-speed", type=float, default=0.12)
    parser.add_argument("--circle-max-angular-speed", type=float, default=0.45)
    parser.add_argument("--circle-angular-accel-max", type=float, default=0.015)
    parser.add_argument("--circle-accel-change-period", type=float, default=3.0)
    parser.add_argument("--circle-speed-gain", type=float, default=0.8)
    parser.add_argument("--random-min-speed", type=float, default=0.18)
    parser.add_argument("--random-max-speed", type=float, default=0.42)
    parser.add_argument("--random-yawrate-max", type=float, default=0.28)
    parser.add_argument("--random-change-period", type=float, default=2.5)
    parser.add_argument("--random-lateral-range-m", type=float, default=0.8)
    parser.add_argument("--random-boundary-yaw-gain", type=float, default=1.6)
    parser.add_argument("--random-speed-smoothing", type=float, default=1.8)
    parser.add_argument("--random-yawrate-smoothing", type=float, default=1.4)
    parser.add_argument("--waypoints", default="0,0;10,0;18,0.4;26,1.6;42,1.6;50,1.2;58,0;80,0;100,0")
    parser.add_argument("--waypoint-speed", type=float, default=1.35)
    parser.add_argument("--waypoint-reached-radius", type=float, default=0.55)
    parser.add_argument("--waypoint-heading-gain", type=float, default=1.1)
    parser.add_argument("--waypoint-yawrate-max", type=float, default=0.35)
    parser.add_argument("--waypoint-speed-smoothing", type=float, default=2.4)
    parser.add_argument("--waypoint-yawrate-smoothing", type=float, default=2.2)
    parser.add_argument("--waypoint-loop", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--randomize-waypoint-speed", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--min-forward-offset-m", type=float, default=0.0)
    parser.add_argument("--max-forward-offset-m", type=float, default=2.0)
    parser.add_argument("--initial-speed", type=float, default=0.05)
    parser.add_argument("--max-speed", type=float, default=1.6)
    parser.add_argument("--accel-min", type=float, default=0.05)
    parser.add_argument("--accel-max", type=float, default=0.2)
    parser.add_argument("--accel-change-period", type=float, default=1.5)
    parser.add_argument("--visible-min-forward", type=float, default=0.0)
    parser.add_argument("--visible-max-forward", type=float, default=15.0)
    parser.add_argument("--visible-half-width", type=float, default=6.0)
    parser.add_argument("--visible-range", type=float, default=18.0)
    parser.add_argument("--seed", type=int, default=-1)
    parser.add_argument("--odom-init-timeout-s", type=float, default=3.0)
    parser.add_argument(
        "--one-way-on-takeoff",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--takeoff-z-threshold",
        type=float,
        default=float(os.getenv("OBDM_TAKEOFF_Z_THRESHOLD", "0.8")),
    )
    parser.add_argument(
        "--contact-arming-z-threshold",
        type=float,
        default=float(os.getenv("OBDM_CONTACT_ARMING_Z_THRESHOLD", "1.5")),
    )
    parser.add_argument(
        "--wait-for-takeoff",
        action=argparse.BooleanOptionalAction,
        default=os.getenv("OBDM_WAIT_FOR_TAKEOFF_BEFORE_ROVER", "1") != "0",
    )
    parser.add_argument(
        "--wait-for-motion-enable",
        action=argparse.BooleanOptionalAction,
        default=os.getenv("OBDM_WAIT_FOR_EXTERNAL_ROVER_RELEASE", "0") == "1",
    )
    parser.add_argument(
        "--start-on-gazebo-up",
        action="store_true",
        default=False,
        help="Automatically release rover motion when Gazebo world/model pose becomes available",
    )
    parser.add_argument("--motion-enable-topic", default="/obdm/start_pad_motion")
    parser.add_argument(
        "--touchdown-z-threshold",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_Z_THRESHOLD", "0.80")),
    )
    parser.add_argument(
        "--touchdown-xy-tolerance",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_XY_TOLERANCE", "1.30")),
    )
    parser.add_argument(
        "--touchdown-max-abs-vz",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_MAX_ABS_VZ", "3.00")),
    )
    parser.add_argument(
        "--touchdown-ground-z-threshold",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_GROUND_Z_THRESHOLD", "1.00")),
    )
    parser.add_argument(
        "--touchdown-ground-xy-tolerance",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_GROUND_XY_TOLERANCE", "1.55")),
    )
    parser.add_argument(
        "--touchdown-use-gazebo-contact",
        action=argparse.BooleanOptionalAction,
        default=os.getenv("OBDM_TOUCHDOWN_USE_GAZEBO_CONTACT", "1") != "0",
    )
    parser.add_argument(
        "--touchdown-require-gazebo-contact",
        action=argparse.BooleanOptionalAction,
        default=os.getenv("OBDM_TOUCHDOWN_REQUIRE_GAZEBO_CONTACT", "1") != "0",
    )
    parser.add_argument("--touchdown-contact-topic", default=os.getenv("OBDM_TOUCHDOWN_CONTACT_TOPIC", "/landing_pad/contact"))
    parser.add_argument(
        "--touchdown-contact-pad-collision",
        default=os.getenv("OBDM_TOUCHDOWN_PAD_COLLISION", "aruco_landing_pad_collision"),
    )
    parser.add_argument(
        "--touchdown-contact-vehicle-filter",
        default=os.getenv(
            "OBDM_TOUCHDOWN_VEHICLE_FILTER",
            "iris,base_collision,front_left_leg_collision,front_right_leg_collision,rear_left_leg_collision,rear_right_leg_collision",
        ),
    )
    parser.add_argument("--touchdown-contact-min-samples", type=int, default=int(os.getenv("OBDM_TOUCHDOWN_CONTACT_MIN_SAMPLES", "3")))
    parser.add_argument(
        "--touchdown-contact-read-timeout-s",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_CONTACT_READ_TIMEOUT_S", "0.06")),
    )
    parser.add_argument("--drone-odom-topic", default=None)
    parser.add_argument("--marker-odom-topic", default="/landing_marker/odom")
    parser.add_argument("--marker-pose-topic", default="/landing_marker/pose")
    parser.add_argument("--marker-visible-topic", default="/landing_marker/visible")
    parser.add_argument("--box-marker-topic", default="/landing_marker/ground_truth/box_markers")
    parser.add_argument("--touchdown-topic", default="/landing_marker/touchdown")
    parser.add_argument("--touchdown-metrics-topic", default="/landing_marker/touchdown_metrics")
    parser.add_argument("--rover-box-length-m", type=float, default=0.90)
    parser.add_argument("--rover-box-width-m", type=float, default=0.55)
    parser.add_argument("--rover-box-height-m", type=float, default=0.18)
    parser.add_argument("--landing-pad-box-length-m", type=float, default=0.72)
    parser.add_argument("--landing-pad-box-width-m", type=float, default=0.72)
    parser.add_argument("--landing-pad-box-height-m", type=float, default=0.035)
    args = parser.parse_args()

    ns = args.namespace if args.namespace.startswith("/") else "/" + args.namespace
    ns = ns.rstrip("/")
    if args.drone_odom_topic is None:
        args.drone_odom_topic = f"{ns}/odometry"

    rclpy.init()
    node = MovingLandingTarget(args)
    try:
        rclpy.spin(node.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop_motion()
        node.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
