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
import random
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class MovingLandingTarget:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("aruco_landing_target")

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.odom_sub = self.node.create_subscription(
            Odometry, args.drone_odom_topic, self.on_drone_odom, qos
        )
        self.odom_pub = self.node.create_publisher(Odometry, args.marker_odom_topic, 10)
        self.pose_pub = self.node.create_publisher(PoseStamped, args.marker_pose_topic, 10)
        self.visible_pub = self.node.create_publisher(Bool, args.marker_visible_topic, 10)
        # Publish Twist on ROS2 so ros_gz bridge can forward to /model/.../cmd_vel
        self.gz_twist_pub = self.node.create_publisher(Twist, f"/model/{args.model_name}/cmd_vel", 10)

        self.drone = None
        self.initialized = False
        self.center_x = 0.0
        self.center_y = 0.0
        self.forward_x = 1.0
        self.forward_y = 0.0
        self.lateral_x = 0.0
        self.lateral_y = 1.0
        self.motion_s = 0.0
        self.z = args.marker_z
        self.x = 0.0
        self.y = 0.0
        self.vs = 0.0
        self.accel_s = 0.0
        self.last_t = time.monotonic()
        self.last_accel_change_t = self.last_t
        self.last_gz_t = 0.0
        self.gz_pose_initialized = False
        self.gz_pose_updates = 0
        self.gz_pose_failures = 0
        self.last_gz_log_t = 0.0
        self.gz_velocity_updates = 0
        self.gz_velocity_failures = 0
        self.last_gz_velocity_log_t = 0.0
        self.rng = random.Random(args.seed if args.seed >= 0 else None)
        self.timer = self.node.create_timer(1.0 / args.rate, self.step)

        self.node.get_logger().info(
            f"Moving ArUco target publishing {args.marker_odom_topic}; "
            f"world={args.world}, model={args.model_name}"
        )

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
        if not self.initialized:
            self.forward_x = math.cos(yaw)
            self.forward_y = math.sin(yaw)
            self.lateral_x = -math.sin(yaw)
            self.lateral_y = math.cos(yaw)
            self.center_x = (
                self.drone["x"]
                + self.args.spawn_forward_m * self.forward_x
                + self.args.spawn_lateral_m * self.lateral_x
            )
            self.center_y = (
                self.drone["y"]
                + self.args.spawn_forward_m * self.forward_y
                + self.args.spawn_lateral_m * self.lateral_y
            )
            self.motion_s = 0.0
            self.x, self.y = self.position_from_motion()
            self.vs = self.rng.uniform(-self.args.initial_speed, self.args.initial_speed)
            self.accel_s = self.random_accel()
            self.initialized = True
            self.node.get_logger().info(
                "Target initialized "
                f"at x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
                f"yaw={math.degrees(yaw):.1f}deg, "
                f"forward=({self.forward_x:.2f},{self.forward_y:.2f})"
            )

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def position_from_motion(self):
        return (
            self.center_x + self.motion_s * self.forward_x,
            self.center_y + self.motion_s * self.forward_y,
        )

    def random_accel(self):
        mag = self.rng.uniform(self.args.accel_min, self.args.accel_max)
        sign = -1.0 if self.rng.random() < 0.5 else 1.0
        return sign * mag

    def step(self):
        if not self.initialized:
            return

        now = time.monotonic()
        dt = clamp(now - self.last_t, 0.0, 0.25)
        self.last_t = now

        if now - self.last_accel_change_t > self.args.accel_change_period:
            self.accel_s = self.random_accel()
            self.last_accel_change_t = now

        self.vs = clamp(self.vs + self.accel_s * dt, -self.args.max_speed, self.args.max_speed)
        self.motion_s = self.motion_s + self.vs * dt

        min_s = -self.args.motion_range_m
        max_s = self.args.motion_range_m
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
            self.publish_gazebo_motion()
            self.last_gz_t = now

    def is_visible(self):
        if self.drone is None:
            return False
        dx = self.x - self.drone["x"]
        dy = self.y - self.drone["y"]
        dz = self.z - self.drone["z"]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        forward = dx * self.forward_x + dy * self.forward_y
        lateral = dx * self.lateral_x + dy * self.lateral_y
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
        odom.header.frame_id = "map"
        odom.child_frame_id = "aruco_landing_marker"
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = float(self.z)
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = float(self.vs * self.forward_x)
        odom.twist.twist.linear.y = float(self.vs * self.forward_y)
        odom.twist.twist.linear.z = 0.0
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        vis = Bool()
        vis.data = bool(visible)
        self.visible_pub.publish(vis)

    def publish_gazebo_motion(self):
        # Use pose update for correct placement, and also command velocity so
        # Gazebo applies physical contact forces (so the pad can push the
        # vehicle). Velocity publish has a fallback to ROS2 topic publish if
        # the gz CLI fails in this environment.
        self.publish_gazebo_pose()
        self.gz_pose_initialized = True
        self.publish_gazebo_velocity()

    def publish_gazebo_pose(self):
        payload = (
            f'name: "{self.args.model_name}" '
            f'position {{x: {self.x:.6f} y: {self.y:.6f} z: {self.z:.6f}}} '
            'orientation {w: 1.0}'
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
                f"Gazebo marker pose {status}: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, "
                f"ok={self.gz_pose_updates}, fail={self.gz_pose_failures}{detail}"
            )
            self.last_gz_log_t = now

    def publish_gazebo_velocity(self):
        vx = self.vs * self.forward_x
        vy = self.vs * self.forward_y
        # Publish directly over ROS2 to ensure bridge receives Twist commands
        try:
            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.gz_twist_pub.publish(twist)
            if self.args.verbose_gz:
                self.node.get_logger().info(f"Published ROS2 Twist: vx={vx:.3f}, vy={vy:.3f}")
        except Exception:
            pass
        payload = f"linear {{x: {vx:.6f} y: {vy:.6f} z: 0.0}} angular {{x: 0.0 y: 0.0 z: 0.0}}"
        topic = f"/model/{self.args.model_name}/cmd_vel"
        sent = False
        # Try gz CLI first (fast, in-process)
        try:
            subprocess.run(
                ["gz", "topic", "-t", topic, "-m", "gz.msgs.Twist", "-p", payload],
                timeout=max(0.5, self.args.gz_topic_timeout_ms / 1000.0),
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.gz_velocity_updates += 1
            sent = True
        except Exception:
            self.gz_velocity_failures += 1

        # Fallback: publish a single ROS2 Twist message (mapped by ros_gz bridge)
        if not sent:
            try:
                ros_payload = (
                    f"{{\"linear\": {{\"x\": {vx:.6f}, \"y\": {vy:.6f}, \"z\": 0.0}}, "
                    f"\"angular\": {{\"x\": 0.0, \"y\": 0.0, \"z\": 0.0}}}}"
                )
                subprocess.run(
                    [
                        "bash",
                        "-lc",
                        f"ros2 topic pub -1 {topic} geometry_msgs/msg/Twist '{ros_payload}' >/dev/null 2>&1 || true",
                    ],
                    timeout=1.0,
                    check=False,
                )
                self.gz_velocity_updates += 1
                sent = True
            except Exception:
                self.gz_velocity_failures += 1

        now = time.monotonic()
        if self.args.verbose_gz or now - self.last_gz_velocity_log_t >= self.args.gz_log_period:
            self.node.get_logger().info(
                f"Gazebo marker velocity commanded: vx={vx:.2f}, vy={vy:.2f}, ok={self.gz_velocity_updates}, fail={self.gz_velocity_failures}"
            )
            self.last_gz_velocity_log_t = now


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
    parser.add_argument("--spawn-forward-m", type=float, default=10.0)
    parser.add_argument("--spawn-lateral-m", type=float, default=0.0)
    parser.add_argument("--marker-z", type=float, default=0.04)
    # Make the target move slower and over a longer path by default so it
    # can physically interact with the vehicle and be interceptable.
    parser.add_argument("--motion-range-m", type=float, default=8.0)
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
    parser.add_argument("--drone-odom-topic", default=None)
    parser.add_argument("--marker-odom-topic", default="/landing_marker/odom")
    parser.add_argument("--marker-pose-topic", default="/landing_marker/pose")
    parser.add_argument("--marker-visible-topic", default="/landing_marker/visible")
    args = parser.parse_args()

    ns = args.namespace if args.namespace.startswith("/") else "/" + args.namespace
    ns = ns.rstrip("/")
    if args.drone_odom_topic is None:
        args.drone_odom_topic = f"{ns}/odometry"

    rclpy.init()
    node = MovingLandingTarget(args)
    try:
        rclpy.spin(node.node)
    finally:
        node.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
