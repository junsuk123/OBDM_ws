#!/usr/bin/env python3
"""Generate landing guidance setpoints continuously from odom and marker tracks."""

import argparse
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class RealtimeOntologyTrajectoryGenerator:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("obdm_realtime_ontology_trajectory_generator")
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        reliable = QoSProfile(depth=20)
        reliable.reliability = ReliabilityPolicy.RELIABLE

        self.drone = None
        self.marker = None
        self.marker_t = 0.0
        self.camera_marker_t = 0.0
        self.visible = False
        self.origin = (float(args.origin_x), float(args.origin_y))
        self.z_offset = float(args.origin_z_offset)
        self.last_target_world = None
        self.landing_enabled = False
        self.ekf_state = None
        self.ekf_t = 0.0
        self.last_log_t = 0.0
        self.last_diag_t = 0.0
        self.odom_count = 0
        self.marker_count = 0
        self.fallback_marker_count = 0
        self.setpoint_count = 0

        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, qos)
        self.node.create_subscription(Odometry, args.marker_topic, self.on_marker, reliable)
        if args.fallback_marker_topic:
            self.node.create_subscription(Odometry, args.fallback_marker_topic, self.on_fallback_marker, reliable)
        self.node.create_subscription(Bool, args.visible_topic, self.on_visible, reliable)
        self.setpoint_pub = self.node.create_publisher(PoseStamped, args.setpoint_topic, 10)
        self.origin_pub = self.node.create_publisher(PoseStamped, args.origin_topic, 10)
        self.timer = self.node.create_timer(1.0 / args.rate, self.step)
        self.node.get_logger().info(
            f"realtime ontology trajectory generator: odom={args.odom_topic}, marker={args.marker_topic}, "
            f"fallback_marker={args.fallback_marker_topic}, "
            f"setpoint={args.setpoint_topic}, origin=({self.origin[0]:.2f},{self.origin[1]:.2f}), "
            f"rate={args.rate:.1f}Hz"
        )

    def on_odom(self, msg):
        self.odom_count += 1
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.drone = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "vx": float(v.x),
            "vy": float(v.y),
            "vz": float(v.z),
        }
        if self.args.use_first_odom_origin and self.odom_count == 1:
            self.origin = (self.drone["x"], self.drone["y"])
            self.z_offset = max(0.0, self.drone["z"] - self.args.takeoff_altitude)
            self.node.get_logger().info(
                f"guidance origin initialized world=({self.origin[0]:.2f},{self.origin[1]:.2f}), z_offset={self.z_offset:.2f}"
            )

    def on_marker(self, msg):
        self.marker_count += 1
        self.camera_marker_t = time.monotonic()
        self.accept_marker(msg, self.camera_marker_t)

    def on_fallback_marker(self, msg):
        self.fallback_marker_count += 1
        now = time.monotonic()
        if self.camera_marker_t > 0.0 and (now - self.camera_marker_t) < self.args.fallback_after:
            return
        self.accept_marker(msg, now)

    def accept_marker(self, msg, now):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.marker = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "vx": float(v.x),
            "vy": float(v.y),
            "vz": float(v.z),
        }
        self.marker_t = now
        self.update_marker_ekf(self.marker_t, self.marker)
        self.visible = True

    def on_visible(self, msg):
        self.visible = bool(msg.data) or (time.monotonic() - self.marker_t) <= self.args.marker_timeout

    def step(self):
        if self.drone is None or self.origin is None or self.z_offset is None:
            self.log_diag()
            return
        self.publish_origin()
        if not self.landing_enabled:
            if self.drone["z"] >= self.args.landing_start_altitude:
                self.landing_enabled = True
                self.last_target_world = None
                self.node.get_logger().info(
                    f"landing trajectory enabled at drone_z={self.drone['z']:.2f} "
                    f"(threshold={self.args.landing_start_altitude:.2f})"
                )
            else:
                self.log_diag(
                    f"waiting for takeoff altitude: drone_z={self.drone['z']:.2f}, "
                    f"threshold={self.args.landing_start_altitude:.2f}"
                )
                return

        now = time.monotonic()
        marker_age = now - self.marker_t if self.marker_t > 0.0 else float("inf")
        camera_marker_age = now - self.camera_marker_t if self.camera_marker_t > 0.0 else float("inf")
        if self.marker is None or marker_age > self.args.marker_lost_timeout:
            self.publish_marker_lost_climb(marker_age, camera_marker_age)
            return

        marker_age = max(0.0, marker_age)
        mx, my, mz, mvx, mvy, _ = self.predicted_marker_state(now)
        dx = mx - self.drone["x"]
        dy = my - self.drone["y"]
        dist = math.hypot(dx, dy)
        lead_time = clamp(
            self.args.min_lead_time + dist * self.args.lead_distance_gain + marker_age,
            self.args.min_lead_time,
            self.args.max_lead_time,
        )
        mx = mx + mvx * lead_time
        my = my + mvy * lead_time
        dx = mx - self.drone["x"]
        dy = my - self.drone["y"]
        dist = math.hypot(dx, dy)

        if dist > 1.0e-6:
            distance_boost = clamp(
                (dist - self.args.fast_track_radius) * self.args.fast_track_gain,
                0.0,
                self.args.fast_track_extra_step,
            )
            max_step = clamp(
                self.args.min_step + self.args.step_gain * dist + distance_boost,
                self.args.min_step,
                self.args.max_step,
            )
            if dist <= self.args.final_radius:
                step = dist
            else:
                step = min(max_step, dist)
            tx = self.drone["x"] + dx / dist * step
            ty = self.drone["y"] + dy / dist * step
        else:
            tx = self.drone["x"]
            ty = self.drone["y"]

        if self.last_target_world is not None:
            a = clamp(self.args.xy_smoothing, 0.0, 1.0)
            tx = (1.0 - a) * self.last_target_world[0] + a * tx
            ty = (1.0 - a) * self.last_target_world[1] + a * ty

        descent_rate = self.args.descent_rate
        if dist > self.args.descent_start_radius:
            descent_rate *= self.args.far_descent_scale
        elif dist <= self.args.fast_descent_radius:
            descent_rate = min(self.args.max_descent_rate_near, descent_rate * self.args.near_descent_boost)
        target_z = max(self.args.touchdown_altitude, self.drone["z"] - descent_rate / max(self.args.rate, 1.0e-3))
        if self.last_target_world is not None:
            target_z = min(target_z, self.last_target_world[2])

        self.last_target_world = (tx, ty, target_z)
        north = ty - self.origin[1]
        east = tx - self.origin[0]
        z_up = max(0.0, target_z - self.z_offset)

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "local_ned"
        msg.pose.position.x = float(north)
        msg.pose.position.y = float(east)
        msg.pose.position.z = float(z_up)
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)
        self.setpoint_count += 1

        now = time.monotonic()
        if now - self.last_log_t >= self.args.log_period:
            self.node.get_logger().info(
                f"setpoint#{self.setpoint_count} local_ned=({north:.2f},{east:.2f},{z_up:.2f}) "
                f"drone=({self.drone['x']:.2f},{self.drone['y']:.2f},{self.drone['z']:.2f}) "
                f"marker=({mx:.2f},{my:.2f},{mz:.2f}) dist={dist:.2f} "
                f"lead_time={lead_time:.2f} marker_age={marker_age:.2f}"
            )
            self.last_log_t = now

    def publish_marker_lost_climb(self, marker_age, camera_marker_age):
        target_z = min(
            self.args.search_altitude,
            max(self.drone["z"], self.args.landing_start_altitude)
            + self.args.marker_lost_climb_rate / max(self.args.rate, 1.0e-3),
        )
        if self.last_target_world is not None:
            tx, ty = self.last_target_world[0], self.last_target_world[1]
        else:
            tx, ty = self.drone["x"], self.drone["y"]
        self.last_target_world = (tx, ty, target_z)

        north = ty - self.origin[1]
        east = tx - self.origin[0]
        z_up = max(0.0, target_z - self.z_offset)

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "local_ned"
        msg.pose.position.x = float(north)
        msg.pose.position.y = float(east)
        msg.pose.position.z = float(z_up)
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)
        self.setpoint_count += 1

        now = time.monotonic()
        if now - self.last_log_t >= self.args.log_period:
            self.node.get_logger().info(
                f"marker lost/search climb setpoint#{self.setpoint_count} "
                f"local_ned=({north:.2f},{east:.2f},{z_up:.2f}) "
                f"drone=({self.drone['x']:.2f},{self.drone['y']:.2f},{self.drone['z']:.2f}) "
                f"marker_age={marker_age:.2f} camera_marker_age={camera_marker_age:.2f}"
            )
            self.last_log_t = now

    def publish_origin(self):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x = float(self.origin[0])
        msg.pose.position.y = float(self.origin[1])
        msg.pose.position.z = float(self.z_offset)
        msg.pose.orientation.w = 1.0
        self.origin_pub.publish(msg)

    def update_marker_ekf(self, now, marker):
        meas = [
            marker["x"],
            marker["y"],
            marker["z"],
            marker["vx"],
            marker["vy"],
            marker["vz"],
        ]
        if self.ekf_state is None:
            self.ekf_state = meas
            self.ekf_t = now
            return

        dt = clamp(now - self.ekf_t, 0.0, 1.0)
        pred = self.ekf_state[:]
        pred[0] += pred[3] * dt
        pred[1] += pred[4] * dt
        pred[2] += pred[5] * dt

        pos_alpha = clamp(self.args.ekf_position_gain, 0.0, 1.0)
        vel_alpha = clamp(self.args.ekf_velocity_gain, 0.0, 1.0)
        self.ekf_state = [
            pred[0] + pos_alpha * (meas[0] - pred[0]),
            pred[1] + pos_alpha * (meas[1] - pred[1]),
            pred[2] + pos_alpha * (meas[2] - pred[2]),
            pred[3] + vel_alpha * (meas[3] - pred[3]),
            pred[4] + vel_alpha * (meas[4] - pred[4]),
            pred[5] + vel_alpha * (meas[5] - pred[5]),
        ]
        self.ekf_t = now

    def predicted_marker_state(self, now):
        if self.ekf_state is None:
            return (
                self.marker["x"],
                self.marker["y"],
                self.marker["z"],
                self.marker["vx"],
                self.marker["vy"],
                self.marker["vz"],
            )
        dt = clamp(now - self.ekf_t, 0.0, self.args.marker_hold_timeout)
        return (
            self.ekf_state[0] + self.ekf_state[3] * dt,
            self.ekf_state[1] + self.ekf_state[4] * dt,
            self.ekf_state[2] + self.ekf_state[5] * dt,
            self.ekf_state[3],
            self.ekf_state[4],
            self.ekf_state[5],
        )

    def log_diag(self, extra=""):
        now = time.monotonic()
        if now - self.last_diag_t < self.args.log_period:
            return
        self.last_diag_t = now
        prefix = f"{extra}; " if extra else ""
        self.node.get_logger().info(
            f"{prefix}waiting: odom_count={self.odom_count}, marker_count={self.marker_count}, "
            f"fallback_marker_count={self.fallback_marker_count}, "
            f"has_drone={self.drone is not None}, has_marker={self.marker is not None}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--odom-topic", default="/iris/odometry")
    parser.add_argument("--marker-topic", default="/landing_marker/odom")
    parser.add_argument("--fallback-marker-topic", default="/landing_marker/ground_truth/odom")
    parser.add_argument("--visible-topic", default="/landing_marker/visible")
    parser.add_argument("--setpoint-topic", default="/obdm/guidance_setpoint")
    parser.add_argument("--origin-topic", default="/obdm/guidance_origin")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z-offset", type=float, default=0.0)
    parser.add_argument("--use-first-odom-origin", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--search-altitude", type=float, default=7.0)
    parser.add_argument("--landing-start-altitude", type=float, default=4.0)
    parser.add_argument("--touchdown-altitude", type=float, default=0.35)
    parser.add_argument("--descent-rate", type=float, default=0.35)
    parser.add_argument("--max-descent-rate-near", type=float, default=0.55)
    parser.add_argument("--near-descent-boost", type=float, default=1.35)
    parser.add_argument("--fast-descent-radius", type=float, default=1.2)
    parser.add_argument("--far-descent-scale", type=float, default=0.35)
    parser.add_argument("--descent-start-radius", type=float, default=2.2)
    parser.add_argument("--final-radius", type=float, default=0.35)
    parser.add_argument("--min-step", type=float, default=0.12)
    parser.add_argument("--max-step", type=float, default=2.20)
    parser.add_argument("--step-gain", type=float, default=0.75)
    parser.add_argument("--fast-track-radius", type=float, default=0.75)
    parser.add_argument("--fast-track-gain", type=float, default=0.45)
    parser.add_argument("--fast-track-extra-step", type=float, default=0.55)
    parser.add_argument("--xy-smoothing", type=float, default=0.85)
    parser.add_argument("--min-lead-time", type=float, default=0.8)
    parser.add_argument("--lead-distance-gain", type=float, default=0.35)
    parser.add_argument("--max-lead-time", type=float, default=3.5)
    parser.add_argument("--ekf-position-gain", type=float, default=0.55)
    parser.add_argument("--ekf-velocity-gain", type=float, default=0.20)
    parser.add_argument("--marker-timeout", type=float, default=0.5)
    parser.add_argument("--marker-lost-timeout", type=float, default=1.2)
    parser.add_argument("--marker-lost-climb-rate", type=float, default=1.20)
    parser.add_argument("--marker-hold-timeout", type=float, default=5.0)
    parser.add_argument("--fallback-after", type=float, default=2.0)
    parser.add_argument("--log-period", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = RealtimeOntologyTrajectoryGenerator(args)
    try:
        rclpy.spin(node.node)
    finally:
        node.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
