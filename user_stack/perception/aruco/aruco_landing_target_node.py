#!/usr/bin/env python3
"""Moving ArUco-style landing target for Gazebo / ROS 2 experiments.

The node owns the target kinematics used by MATLAB:
  * initial target position is 10 m in front of the first received vehicle odom
  * target moves back and forth along the initial vehicle-heading axis within +/- range
  * acceleration is randomized at each boundary bounce and periodically in-flight
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
import subprocess
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


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
        # Publish Twist on ROS2 so ros_gz bridge can forward to /model/.../cmd_vel
        self.gz_twist_pub = self.node.create_publisher(Twist, f"/model/{args.model_name}/cmd_vel", 10)

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
        self.z = args.marker_z
        self.x = 0.0
        self.y = 0.0
        self.vs = 0.0
        self.accel_s = 0.0
        self.last_t = time.monotonic()
        self.last_accel_change_t = self.last_t
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
        self.wheel_force_gain = float(os.getenv("OBDM_ROVER_FORCE_GAIN", "1.2"))
        self.wheel_turn_gain = float(os.getenv("OBDM_ROVER_TURN_GAIN", "1.8"))
        self.wheel_force_max = float(os.getenv("OBDM_ROVER_FORCE_MAX", "3.0"))
        self.wheel_cmdvel_max = float(os.getenv("OBDM_ROVER_CMDVEL_MAX", "0.5"))
        self.wheel_yawrate_max = float(os.getenv("OBDM_ROVER_YAWRATE_MAX", "1.2"))
        self.start_t = time.monotonic()
        self.takeoff_detected = False
        self.motion_enabled = not args.wait_for_takeoff
        self.touchdown_detected = False
        self.touchdown_metrics = (0.0, 0.0, 0.0)
        self.last_zero_cmd_t = 0.0
        self.rng = random.Random(args.seed if args.seed >= 0 else None)
        self.timer = self.node.create_timer(1.0 / args.rate, self.step)

        self.node.get_logger().info(
            f"Moving ArUco target publishing {args.marker_odom_topic}; "
            f"world={args.world}, model={args.model_name}, wait_for_takeoff={args.wait_for_takeoff}"
        )

        if args.initial_x is not None and args.initial_y is not None:
            self.initialize_from_pose(args.initial_x, args.initial_y, args.initial_yaw_rad, "configured MATLAB initial position")

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
        self.check_touchdown()

    def mark_takeoff_detected(self):
        self.takeoff_detected = True
        self.motion_enabled = True
        mode = "forward-only mode" if self.args.one_way_on_takeoff else "target motion"
        self.node.get_logger().info(
            f"Takeoff detected (z={self.drone['z']:.2f} >= {self.args.takeoff_z_threshold:.2f}); "
            f"enabling {mode}"
        )

    def check_touchdown(self):
        if self.touchdown_detected or not self.initialized or self.drone is None:
            return
        if not self.takeoff_detected:
            return
        dx = self.drone["x"] - self.x
        dy = self.drone["y"] - self.y
        horizontal_error = math.sqrt(dx * dx + dy * dy)
        altitude_ok = self.drone["z"] <= self.args.touchdown_z_threshold
        position_ok = horizontal_error <= self.args.touchdown_xy_tolerance
        speed_ok = abs(self.drone["vz"]) <= self.args.touchdown_max_abs_vz
        if altitude_ok and position_ok and speed_ok:
            self.touchdown_detected = True
            self.touchdown_metrics = (self.drone["z"], horizontal_error, self.drone["vz"])
            self.node.get_logger().info(
                f"Touchdown on marker detected: z={self.drone['z']:.2f}, "
                f"xy_error={horizontal_error:.2f}, vz={self.drone['vz']:.2f}; stopping target motion"
            )
            self.stop_motion()

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def position_from_motion(self):
        if self.args.motion_mode == "circle":
            radius = self.args.circle_radius_m
            radial_x = math.cos(self.circle_theta) * self.forward_x + math.sin(self.circle_theta) * self.lateral_x
            radial_y = math.cos(self.circle_theta) * self.forward_y + math.sin(self.circle_theta) * self.lateral_y
            return (
                self.center_x + radius * radial_x,
                self.center_y + radius * radial_y,
            )
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
        min_s, max_s = self.motion_limits()
        self.motion_s = clamp(self.motion_s, min_s, max_s)
        self.x, self.y = self.position_from_motion()
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
        elif now - self.last_accel_change_t > self.args.accel_change_period:
            self.accel_s = self.random_accel()
            self.last_accel_change_t = now

        if self.args.motion_mode != "circle":
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
        if self.drone is None:
            return False
        dx = self.x - self.drone["x"]
        dy = self.y - self.drone["y"]
        dz = self.z - self.drone["z"]
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
        stamp = self.node.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "aruco_landing_marker"
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = float(self.z)
        odom.pose.pose.orientation.w = 1.0
        if self.args.motion_mode == "circle":
            vx = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_x + math.cos(self.circle_theta) * self.lateral_x
            )
            vy = self.args.circle_radius_m * self.circle_omega * (
                -math.sin(self.circle_theta) * self.forward_y + math.cos(self.circle_theta) * self.lateral_y
            )
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

    def publish_gazebo_motion(self):
        if not self.gz_initial_pose_sent:
            self.publish_gazebo_pose()
            self.gz_pose_initialized = True
            self.gz_initial_pose_sent = True
        self.publish_gazebo_velocity()

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
    parser.add_argument("--verbose-gz", action="store_true")
    parser.add_argument("--spawn-forward-m", type=float, default=2.5)
    parser.add_argument("--spawn-lateral-m", type=float, default=0.0)
    parser.add_argument("--initial-x", type=float, default=None)
    parser.add_argument("--initial-y", type=float, default=None)
    parser.add_argument("--initial-yaw-rad", type=float, default=0.0)
    parser.add_argument("--marker-z", type=float, default=0.04)
    parser.add_argument("--model-z", type=float, default=0.04)
    # Make the target move slower and over a longer path by default so it
    # can physically interact with the vehicle and be interceptable.
    parser.add_argument("--motion-mode", choices=["line", "circle"], default="line")
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
    parser.add_argument("--min-forward-offset-m", type=float, default=0.0)
    parser.add_argument("--max-forward-offset-m", type=float, default=2.0)
    parser.add_argument("--initial-speed", type=float, default=0.05)
    parser.add_argument("--max-speed", type=float, default=0.5)
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
        "--wait-for-takeoff",
        action=argparse.BooleanOptionalAction,
        default=os.getenv("OBDM_WAIT_FOR_TAKEOFF_BEFORE_ROVER", "1") != "0",
    )
    parser.add_argument(
        "--touchdown-z-threshold",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_Z_THRESHOLD", "0.35")),
    )
    parser.add_argument(
        "--touchdown-xy-tolerance",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_XY_TOLERANCE", "0.75")),
    )
    parser.add_argument(
        "--touchdown-max-abs-vz",
        type=float,
        default=float(os.getenv("OBDM_TOUCHDOWN_MAX_ABS_VZ", "1.0")),
    )
    parser.add_argument("--drone-odom-topic", default=None)
    parser.add_argument("--marker-odom-topic", default="/landing_marker/odom")
    parser.add_argument("--marker-pose-topic", default="/landing_marker/pose")
    parser.add_argument("--marker-visible-topic", default="/landing_marker/visible")
    parser.add_argument("--touchdown-topic", default="/landing_marker/touchdown")
    parser.add_argument("--touchdown-metrics-topic", default="/landing_marker/touchdown_metrics")
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
