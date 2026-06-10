#!/usr/bin/env python3
"""Generate landing guidance setpoints continuously from odom and marker tracks."""

import argparse
import math
import random
import time

from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def lerp(a, b, t):
    return a + (b - a) * clamp(t, 0.0, 1.0)


def limit_vector(x, y, max_norm):
    norm = math.hypot(x, y)
    if norm > max_norm > 0.0:
        scale = max_norm / norm
        return x * scale, y * scale
    return x, y


def quat_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


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
        self.marker_source = "none"
        self.marker_t = 0.0
        self.camera_marker_t = 0.0
        self.last_camera_marker_raw = None
        self.last_camera_marker_raw_t = 0.0
        self.rejected_camera_marker_count = 0
        self.visible = False
        self.origin = (float(args.origin_x), float(args.origin_y))
        self.z_offset = float(args.origin_z_offset)
        self.last_target_world = None
        self.landing_enabled = False
        self.ekf_state = None
        self.ekf_t = 0.0
        self.last_log_t = 0.0
        self.last_diag_t = 0.0
        self.last_marker_reject_log_t = 0.0
        self.last_ontology = None
        self.guidance_mode = "SEARCH"
        self.direct_land_since = None
        self.land_requested = False
        self.last_odom_t = 0.0
        self.wind = {"x": 0.0, "y": 0.0, "z": 0.0, "speed": 0.0}
        self.wind_t = 0.0
        self.marker_lost_since = None
        self.search_timeout_sent = False
        self.odom_count = 0
        self.marker_count = 0
        self.fallback_marker_count = 0
        self.setpoint_count = 0
        self.noise_rng = random.Random(args.fallback_marker_noise_seed)
        self.started_t = time.monotonic()
        self.search_altitude_ceiling = (
            args.takeoff_altitude
            if args.max_search_altitude is None
            else min(args.max_search_altitude, args.takeoff_altitude)
        )
        self.effective_search_altitude = min(args.search_altitude, self.search_altitude_ceiling)

        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, qos)
        self.node.create_subscription(Odometry, args.marker_topic, self.on_marker, reliable)
        if args.wind_topic:
            self.node.create_subscription(Vector3Stamped, args.wind_topic, self.on_wind, qos)
        if args.fallback_marker_topic:
            self.node.create_subscription(Odometry, args.fallback_marker_topic, self.on_fallback_marker, reliable)
        self.node.create_subscription(Bool, args.visible_topic, self.on_visible, reliable)
        self.setpoint_pub = self.node.create_publisher(PoseStamped, args.setpoint_topic, 10)
        self.velocity_pub = self.node.create_publisher(TwistStamped, args.velocity_topic, 10)
        self.accel_pub = self.node.create_publisher(AccelStamped, args.accel_topic, 10)
        self.land_pub = self.node.create_publisher(Bool, args.land_topic, 10)
        self.origin_pub = self.node.create_publisher(PoseStamped, args.origin_topic, 10)
        self.timer = self.node.create_timer(1.0 / args.rate, self.step)
        self.node.get_logger().info(
            f"realtime ontology trajectory generator: odom={args.odom_topic}, marker={args.marker_topic}, "
            f"fallback_marker={args.fallback_marker_topic}, fallback_source={args.fallback_marker_source_name}, "
            f"setpoint={args.setpoint_topic}, velocity={args.velocity_topic}, accel={args.accel_topic}, "
            f"origin=({self.origin[0]:.2f},{self.origin[1]:.2f}), "
            f"rate={args.rate:.1f}Hz, search_altitude={self.effective_search_altitude:.2f}m "
            f"(requested={args.search_altitude:.2f}m, ceiling={self.search_altitude_ceiling:.2f}m)"
        )

    def on_odom(self, msg):
        self.odom_count += 1
        now = time.monotonic()
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        vx = float(v.x)
        vy = float(v.y)
        vz = float(v.z)
        ax = 0.0
        ay = 0.0
        az = 0.0
        if self.drone is not None and self.last_odom_t > 0.0:
            dt = max(1.0e-3, now - self.last_odom_t)
            ax = (vx - self.drone["vx"]) / dt
            ay = (vy - self.drone["vy"]) / dt
            az = (vz - self.drone["vz"]) / dt
            accel_alpha = clamp(self.args.odom_accel_smoothing, 0.0, 1.0)
            ax = (1.0 - accel_alpha) * self.drone.get("ax", 0.0) + accel_alpha * ax
            ay = (1.0 - accel_alpha) * self.drone.get("ay", 0.0) + accel_alpha * ay
            az = (1.0 - accel_alpha) * self.drone.get("az", 0.0) + accel_alpha * az
        self.last_odom_t = now
        self.drone = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "ax": ax,
            "ay": ay,
            "az": az,
            "yaw": quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w)),
        }
        if self.args.use_first_odom_origin and self.odom_count == 1:
            self.origin = (self.drone["x"], self.drone["y"])
            self.z_offset = max(0.0, self.drone["z"] - self.args.takeoff_altitude)
            self.node.get_logger().info(
                f"guidance origin initialized world=({self.origin[0]:.2f},{self.origin[1]:.2f}), z_offset={self.z_offset:.2f}"
            )

    def on_marker(self, msg):
        self.marker_count += 1
        now = time.monotonic()
        if not self.camera_marker_measurement_ok(msg, now):
            return
        self.camera_marker_t = now
        self.accept_marker(msg, self.camera_marker_t, "camera")
        p = msg.pose.pose.position
        self.last_camera_marker_raw = (float(p.x), float(p.y), float(p.z))
        self.last_camera_marker_raw_t = now

    def on_fallback_marker(self, msg):
        self.fallback_marker_count += 1
        if not self.args.allow_fallback_marker_for_guidance:
            return
        now = time.monotonic()
        if self.camera_marker_t > 0.0 and (now - self.camera_marker_t) < self.args.fallback_after:
            return
        self.accept_marker(msg, now, self.args.fallback_marker_source_name)

    def accept_marker(self, msg, now, source):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        nx = ny = nz = nvx = nvy = nvz = 0.0
        if source == self.args.fallback_marker_source_name:
            nx = self.noise_rng.gauss(0.0, self.args.fallback_marker_horizontal_noise_std)
            ny = self.noise_rng.gauss(0.0, self.args.fallback_marker_horizontal_noise_std)
            nz = self.noise_rng.gauss(0.0, self.args.fallback_marker_vertical_noise_std)
            nvx = self.noise_rng.gauss(0.0, self.args.fallback_marker_velocity_noise_std)
            nvy = self.noise_rng.gauss(0.0, self.args.fallback_marker_velocity_noise_std)
            nvz = self.noise_rng.gauss(0.0, self.args.fallback_marker_velocity_noise_std)
        self.marker = {
            "x": float(p.x) + nx,
            "y": float(p.y) + ny,
            "z": float(p.z) + nz,
            "vx": float(v.x) + nvx,
            "vy": float(v.y) + nvy,
            "vz": float(v.z) + nvz,
        }
        self.marker_source = source
        self.marker_t = now
        self.update_marker_ekf(self.marker_t, self.marker)
        if source == "camera":
            self.visible = True

    def camera_marker_measurement_ok(self, msg, now):
        p = msg.pose.pose.position
        cur = (float(p.x), float(p.y), float(p.z))
        if not all(math.isfinite(v) for v in cur):
            return False
        if self.last_camera_marker_raw is None or self.last_camera_marker_raw_t <= 0.0:
            return True

        dt = max(1.0e-3, now - self.last_camera_marker_raw_t)
        if dt > self.args.camera_marker_outlier_window:
            return True
        horizontal_jump = math.hypot(
            cur[0] - self.last_camera_marker_raw[0],
            cur[1] - self.last_camera_marker_raw[1],
        )
        vertical_jump = abs(cur[2] - self.last_camera_marker_raw[2])
        horizontal_speed = horizontal_jump / dt
        jump_ok = horizontal_jump <= self.args.camera_marker_max_position_jump
        speed_ok = horizontal_speed <= self.args.camera_marker_max_horizontal_speed
        vertical_ok = vertical_jump <= self.args.camera_marker_max_vertical_jump
        if jump_ok and speed_ok and vertical_ok:
            return True

        self.rejected_camera_marker_count += 1
        if now - self.last_marker_reject_log_t >= self.args.log_period:
            self.node.get_logger().warn(
                f"rejected camera marker outlier #{self.rejected_camera_marker_count}: "
                f"jump_xy={horizontal_jump:.2f}m, jump_z={vertical_jump:.2f}m, "
                f"speed_xy={horizontal_speed:.2f}m/s, dt={dt:.2f}s"
            )
            self.last_marker_reject_log_t = now
        return False

    def on_visible(self, msg):
        self.visible = bool(msg.data) or (time.monotonic() - self.marker_t) <= self.args.marker_timeout

    def on_wind(self, msg):
        vx = float(msg.vector.x)
        vy = float(msg.vector.y)
        vz = float(msg.vector.z)
        self.wind = {
            "x": vx,
            "y": vy,
            "z": vz,
            "speed": math.sqrt(vx * vx + vy * vy + vz * vz),
        }
        self.wind_t = time.monotonic()

    def step(self):
        if self.drone is None or self.origin is None or self.z_offset is None:
            self.log_diag()
            return
        self.publish_origin()
        if not self.landing_enabled:
            wait_elapsed = max(0.0, time.monotonic() - self.started_t)
            camera_marker_ready = (
                self.camera_marker_t > 0.0
                and (time.monotonic() - self.camera_marker_t) <= self.args.marker_hold_timeout
            )
            marker_recovery_ready = (
                camera_marker_ready
                and self.drone["z"] >= self.args.mid_approach_recovery_altitude
            )
            timeout_ready = (
                self.args.takeoff_wait_timeout > 0.0
                and wait_elapsed >= self.args.takeoff_wait_timeout
                and self.drone["z"] >= self.args.landing_start_altitude
                and self.odom_count >= max(3, int(self.args.rate))
            )
            if self.drone["z"] >= self.args.landing_start_altitude or marker_recovery_ready or timeout_ready:
                self.landing_enabled = True
                self.last_target_world = None
                reason = "takeoff_altitude"
                if marker_recovery_ready:
                    reason = "marker_recovery"
                elif timeout_ready:
                    reason = "takeoff_wait_timeout"
                self.node.get_logger().info(
                    f"landing trajectory enabled at drone_z={self.drone['z']:.2f} "
                    f"(threshold={self.args.landing_start_altitude:.2f}, recovery={marker_recovery_ready}, "
                    f"timeout={timeout_ready}, reason={reason}, waited={wait_elapsed:.1f}s)"
                )
            else:
                self.log_diag(
                    f"waiting for takeoff altitude: drone_z={self.drone['z']:.2f}, "
                    f"threshold={self.args.landing_start_altitude:.2f}, "
                    f"recovery_threshold={self.args.mid_approach_recovery_altitude:.2f}, "
                    f"timeout={self.args.takeoff_wait_timeout:.1f}s, waited={wait_elapsed:.1f}s"
                )
                return

        now = time.monotonic()
        marker_age = now - self.marker_t if self.marker_t > 0.0 else float("inf")
        camera_marker_age = now - self.camera_marker_t if self.camera_marker_t > 0.0 else float("inf")
        camera_marker_fresh = camera_marker_age <= self.args.camera_marker_guidance_timeout
        fallback_marker_fresh = (
            self.args.allow_fallback_marker_for_guidance
            and self.marker is not None
            and self.marker_source == self.args.fallback_marker_source_name
            and marker_age <= self.args.fallback_marker_guidance_timeout
        )
        terminal_prediction_allowed = False
        if self.marker is not None and self.marker_source == "camera":
            altitude_above_pad = self.drone["z"] - self.marker["z"]
            terminal_prediction_allowed = (
                altitude_above_pad <= self.args.terminal_marker_prediction_altitude_above_pad
                and camera_marker_age <= self.args.terminal_marker_prediction_timeout
            )
        if self.marker is None or marker_age > self.args.marker_hold_timeout:
            if self.handle_marker_search_timeout(now, marker_age, camera_marker_age, "marker_hold_timeout"):
                return
            self.publish_marker_lost_climb(marker_age, camera_marker_age)
            return
        if (
            self.args.require_camera_marker_for_descent
            and (not camera_marker_fresh or self.marker_source != "camera")
            and not fallback_marker_fresh
            and not terminal_prediction_allowed
        ):
            if self.commit_terminal_landing_on_marker_loss(now, marker_age, camera_marker_age):
                return
            if self.handle_marker_search_timeout(now, marker_age, camera_marker_age, "camera_marker_timeout"):
                return
            self.publish_marker_lost_climb(marker_age, camera_marker_age)
            return

        self.marker_lost_since = None
        self.search_timeout_sent = False
        marker_age = max(0.0, marker_age)
        mx, my, mz, mvx, mvy, _ = self.predicted_marker_state(now)
        dx = mx - self.drone["x"]
        dy = my - self.drone["y"]
        current_pad_dist = math.hypot(dx, dy)
        altitude_above_pad = max(0.0, self.drone["z"] - mz)
        ontology = self.evaluate_ontology(
            marker=(mx, my, mz, mvx, mvy),
            current_pad_dist=current_pad_dist,
            marker_age=marker_age,
        )
        self.last_ontology = ontology
        mode = ontology["values"]["guidanceMode"]
        lead_time = ontology["values"]["leadTime"]
        rel_vx = self.drone["vx"] - mvx
        rel_vy = self.drone["vy"] - mvy
        relative_speed = math.hypot(rel_vx, rel_vy)
        intercept_lead = clamp(
            current_pad_dist / max(self.args.intercept_nominal_speed, 1.0e-3),
            self.args.min_lead_time,
            self.args.max_lead_time,
        )
        if mode == "INTERCEPT":
            lead_time = lerp(lead_time, intercept_lead, self.args.intercept_lead_blend)
        aim_x, aim_y, future_time = self.predicted_approach_point(now, lead_time)
        marker_speed = math.hypot(mvx, mvy)
        if mode == "PAD_CAPTURE":
            capture_blend = ontology["values"]["captureBlend"]
            aim_x = capture_blend * mx + (1.0 - capture_blend) * aim_x
            aim_y = capture_blend * my + (1.0 - capture_blend) * aim_y
        elif mode in ("FINAL_DESCENT", "LAND_READY"):
            terminal = self.terminal_landing_solution(now, mz, descent_rate=ontology["values"]["descentRate"])
            aim_x = terminal["aim_x"]
            aim_y = terminal["aim_y"]
            future_time = terminal["prediction_time"]
        damping_time = self.velocity_damping_time(mode)
        aim_damping_time = 0.0
        if mode == "INTERCEPT":
            aim_damping_time = damping_time
        elif mode == "PAD_CAPTURE":
            capture_span = max(self.args.pad_capture_radius - self.args.final_descent_radius, 1.0e-3)
            capture_progress = clamp(
                (current_pad_dist - self.args.final_descent_radius) / capture_span,
                0.0,
                1.0,
            )
            aim_damping_time = (
                damping_time
                * clamp(self.args.capture_aim_damping_scale, 0.0, 1.0)
                * capture_progress
            )
        if aim_damping_time > 0.0:
            aim_x -= rel_vx * aim_damping_time
            aim_y -= rel_vy * aim_damping_time
        aim_x, aim_y, aim_corridor_offset, aim_corridor_limit = self.constrain_approach_aim(
            now,
            mode,
            aim_x,
            aim_y,
            current_pad_dist,
            future_time,
        )
        descent_center_time = min(future_time, self.args.descent_center_prediction_time)
        if mode in ("FINAL_DESCENT", "LAND_READY"):
            terminal = self.terminal_landing_solution(now, mz, descent_rate=ontology["values"]["descentRate"])
            center_error = terminal["touchdown_error"]
            wind_speed = terminal["wind_speed"]
            terminal_prediction_time = terminal["prediction_time"]
            terminal_inertia_time = terminal["inertia_time"]
        else:
            center_x, center_y, _, _, _, _ = self.predicted_marker_state(now, descent_center_time)
            center_error = math.hypot(center_x - self.drone["x"], center_y - self.drone["y"])
            wind_speed = self.current_wind_speed(now)
            terminal_prediction_time = 0.0
            terminal_inertia_time = 0.0
        dx = aim_x - self.drone["x"]
        dy = aim_y - self.drone["y"]
        dist = math.hypot(dx, dy)

        if dist > 1.0e-6:
            max_step = clamp(
                ontology["values"]["maxHorizontalStep"],
                self.args.min_step,
                self.args.max_step,
            )
            if mode in ("FINAL_DESCENT", "LAND_READY"):
                max_step = min(max_step, max(self.args.min_step, current_pad_dist * self.args.final_step_gain))
            elif mode == "PAD_CAPTURE":
                max_step = min(max_step, max(self.args.min_step, current_pad_dist * self.args.capture_step_gain))
            if mode in ("FINAL_DESCENT", "LAND_READY"):
                step = min(max_step, dist)
            elif current_pad_dist <= self.args.final_radius:
                step = min(max(self.args.min_step, current_pad_dist * self.args.final_step_gain), dist)
            else:
                step = min(max_step, dist)
            tx = self.drone["x"] + dx / dist * step
            ty = self.drone["y"] + dy / dist * step
        else:
            tx = self.drone["x"]
            ty = self.drone["y"]

        if self.last_target_world is not None:
            a = clamp(self.args.xy_smoothing, 0.0, 1.0)
            if mode == "PAD_CAPTURE":
                a = clamp(self.args.capture_xy_smoothing, 0.0, 1.0)
            elif mode in ("FINAL_DESCENT", "LAND_READY"):
                a = clamp(self.args.final_xy_smoothing, 0.0, 1.0)
            tx = (1.0 - a) * self.last_target_world[0] + a * tx
            ty = (1.0 - a) * self.last_target_world[1] + a * ty

        descent_rate = ontology["values"]["descentRate"]
        descent_start_radius = ontology["values"]["descentStartRadius"]
        center_ready = center_error <= self.args.centering_error_for_descent
        relative_speed_ready = relative_speed <= self.args.max_relative_speed_for_descent
        if mode in ("FINAL_DESCENT", "LAND_READY"):
            descent_rate = min(self.args.max_descent_rate_near, max(descent_rate, self.args.final_descent_rate))
        elif mode == "PAD_CAPTURE":
            descent_rate = min(self.args.max_descent_rate_near, max(descent_rate, self.args.capture_descent_rate))
        elif dist > descent_start_radius:
            descent_rate *= self.args.far_descent_scale
        elif dist <= self.args.fast_descent_radius:
            descent_rate = min(self.args.max_descent_rate_near, descent_rate * self.args.near_descent_boost)
        if mode in ("FINAL_DESCENT", "LAND_READY"):
            touchdown_z = max(0.0, mz + self.args.terminal_touchdown_height_above_pad)
        else:
            touchdown_z = max(self.args.touchdown_altitude, mz + self.args.touchdown_height_above_pad)
        if not center_ready or not relative_speed_ready:
            descent_rate *= self.args.unsafe_descent_scale
            touchdown_z = max(touchdown_z, mz + self.args.centering_hold_height_above_pad)
        target_z = max(touchdown_z, self.drone["z"] - descent_rate / max(self.args.rate, 1.0e-3))
        if self.last_target_world is not None:
            target_z = min(target_z, self.last_target_world[2])

        self.last_target_world = (tx, ty, target_z)
        north = ty - self.origin[1]
        east = tx - self.origin[0]
        z_up = max(0.0, target_z - self.z_offset)
        cmd = self.compute_velocity_accel_command(
            ontology=ontology,
            mode=mode,
            aim_x=aim_x,
            aim_y=aim_y,
            target_z=target_z,
            marker_vx=mvx,
            marker_vy=mvy,
            rel_vx=rel_vx,
            rel_vy=rel_vy,
            altitude_above_pad=altitude_above_pad,
            center_error=center_error,
            relative_speed=relative_speed,
            center_ready=center_ready,
            relative_speed_ready=relative_speed_ready,
        )

        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "local_ned"
        msg.pose.position.x = float(north)
        msg.pose.position.y = float(east)
        msg.pose.position.z = float(z_up)
        yaw = self.compute_setpoint_yaw(north, east)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.setpoint_pub.publish(msg)
        self.publish_velocity_accel_command(msg.header.stamp, cmd)
        self.setpoint_count += 1

        direct_land_ready = self.should_direct_land(now, current_pad_dist, mz, marker_speed, relative_speed)
        if direct_land_ready:
            self.publish_land_request(current_pad_dist, mz)

        now = time.monotonic()
        if now - self.last_log_t >= self.args.log_period:
            self.node.get_logger().info(
                f"setpoint#{self.setpoint_count} local_ned=({north:.2f},{east:.2f},{z_up:.2f}) "
                f"drone=({self.drone['x']:.2f},{self.drone['y']:.2f},{self.drone['z']:.2f}) "
                f"marker=({mx:.2f},{my:.2f},{mz:.2f}) aim=({aim_x:.2f},{aim_y:.2f}) "
                f"aim_dist={dist:.2f} pad_dist={current_pad_dist:.2f} "
                f"aim_corridor=({aim_corridor_offset:.2f}/{aim_corridor_limit:.2f}) "
                f"lead_time={lead_time:.2f} intercept_lead={intercept_lead:.2f} future_time={future_time:.2f} "
                f"descent_center_time={descent_center_time:.2f} "
                f"center_error={center_error:.2f} rel_speed={relative_speed:.2f} "
                f"terminal_t={terminal_prediction_time:.2f} inertia_t={terminal_inertia_time:.2f} "
                f"wind={wind_speed:.2f} alt_above_pad={altitude_above_pad:.2f} "
                f"cmd_vel=({cmd['vn']:.2f},{cmd['ve']:.2f},{cmd['vz_up']:.2f}) "
                f"cmd_acc=({cmd['an']:.2f},{cmd['ae']:.2f},{cmd['az_up']:.2f}) "
                f"cmd_limits=(speed:{ontology['values']['velocityMaxSpeed']:.2f},accel:{ontology['values']['accelMax']:.2f},"
                f"cross:{ontology['values']['centerCrossingSpeed']:.2f},"
                f"overshoot:{ontology['values']['overshootRisk']:.2f},"
                f"rel_dist_risk:{ontology['values']['relativeDistanceRisk']:.2f},"
                f"rel_vel_risk:{ontology['values']['relativeVelocityRisk']:.2f},"
                f"intercept_auth:{ontology['values']['interceptDistanceAuthority']:.2f}) "
                f"center_ready={center_ready} rel_ready={relative_speed_ready} marker_age={marker_age:.2f} "
                f"camera_marker_age={camera_marker_age:.2f} marker_source={self.marker_source} "
                f"ontology=mode:{mode},intent:{ontology['values']['landingIntent']:.2f},"
                f"risk:{ontology['values']['operationalRisk']:.2f},"
                f"track:{ontology['values']['trackQuality']:.2f},"
                f"step:{ontology['values']['maxHorizontalStep']:.2f},"
                f"direct_land_ready:{direct_land_ready}"
            )
            self.last_log_t = now

    def should_direct_land(self, now, dist, pad_z, marker_speed, relative_speed):
        altitude_above_pad = self.drone["z"] - pad_z
        horizontal_speed = math.hypot(self.drone["vx"], self.drone["vy"])
        aligned = dist <= self.args.direct_land_radius
        low_enough = altitude_above_pad <= self.args.direct_land_altitude_above_pad
        slow_enough = horizontal_speed <= self.args.direct_land_max_horizontal_speed
        relative_slow_enough = relative_speed <= self.args.direct_land_max_relative_speed
        marker_speed_ok = marker_speed >= self.args.direct_land_min_marker_speed
        if aligned and low_enough and slow_enough and relative_slow_enough and marker_speed_ok:
            if self.direct_land_since is None:
                self.direct_land_since = now
            if self.land_requested:
                return True
            return (now - self.direct_land_since) >= self.args.direct_land_stable_time
        self.direct_land_since = None
        self.land_requested = False
        return False

    def terminal_landing_solution(self, now, pad_z, descent_rate):
        altitude_above_pad = max(0.0, self.drone["z"] - pad_z)
        descent_time = altitude_above_pad / max(abs(descent_rate), 0.10)
        inertia_speed = math.hypot(self.drone["vx"], self.drone["vy"])
        accel_mag = math.hypot(self.drone.get("ax", 0.0), self.drone.get("ay", 0.0))
        wind_speed = self.current_wind_speed(now)
        inertia_time = clamp(
            self.args.terminal_response_time
            + self.args.terminal_inertia_speed_gain * inertia_speed
            + self.args.terminal_inertia_accel_gain * accel_mag
            + self.args.terminal_wind_response_gain * wind_speed,
            self.args.terminal_min_prediction_time,
            self.args.terminal_max_prediction_time,
        )
        prediction_time = clamp(
            descent_time + inertia_time,
            self.args.terminal_min_prediction_time,
            self.args.terminal_max_prediction_time,
        )
        pad_x, pad_y, _, _, _, _ = self.predicted_marker_state(now, prediction_time)
        drone_x = (
            self.drone["x"]
            + self.drone["vx"] * inertia_time
            + 0.5 * self.drone.get("ax", 0.0) * inertia_time * inertia_time
            + self.wind["x"] * self.args.terminal_wind_drift_time
        )
        drone_y = (
            self.drone["y"]
            + self.drone["vy"] * inertia_time
            + 0.5 * self.drone.get("ay", 0.0) * inertia_time * inertia_time
            + self.wind["y"] * self.args.terminal_wind_drift_time
        )
        correction_x = -(drone_x - self.drone["x"])
        correction_y = -(drone_y - self.drone["y"])
        correction_norm = math.hypot(correction_x, correction_y)
        max_correction = max(0.0, self.args.terminal_max_inertia_correction)
        if correction_norm > max_correction > 0.0:
            correction_x = correction_x / correction_norm * max_correction
            correction_y = correction_y / correction_norm * max_correction
        aim_x = pad_x + correction_x
        aim_y = pad_y + correction_y
        touchdown_error = math.hypot(pad_x - drone_x, pad_y - drone_y)
        return {
            "aim_x": aim_x,
            "aim_y": aim_y,
            "pad_x": pad_x,
            "pad_y": pad_y,
            "drone_x": drone_x,
            "drone_y": drone_y,
            "touchdown_error": touchdown_error,
            "prediction_time": prediction_time,
            "descent_time": descent_time,
            "inertia_time": inertia_time,
            "wind_speed": wind_speed,
            "accel": accel_mag,
        }

    def constrain_approach_aim(self, now, mode, aim_x, aim_y, current_pad_dist, future_time):
        if mode not in ("PAD_CAPTURE", "FINAL_DESCENT", "LAND_READY"):
            return aim_x, aim_y, 0.0, 0.0

        if mode == "PAD_CAPTURE":
            center_time = min(
                max(0.0, future_time),
                max(0.0, self.args.capture_center_corridor_prediction_time),
            )
            corridor_limit = max(
                self.args.min_step,
                min(
                    self.args.capture_center_corridor_radius,
                    current_pad_dist * self.args.capture_center_corridor_gain,
                ),
            )
        else:
            center_time = min(
                max(0.0, future_time),
                max(0.0, self.args.final_center_corridor_prediction_time),
            )
            corridor_limit = max(
                self.args.min_step,
                min(
                    self.args.final_center_corridor_radius,
                    max(self.args.min_step, current_pad_dist * self.args.final_center_corridor_gain),
                ),
            )

        center_x, center_y, _, _, _, _ = self.predicted_marker_state(now, center_time)
        offset_x = aim_x - center_x
        offset_y = aim_y - center_y
        offset_norm = math.hypot(offset_x, offset_y)
        if offset_norm > corridor_limit > 0.0:
            offset_x = offset_x / offset_norm * corridor_limit
            offset_y = offset_y / offset_norm * corridor_limit
            aim_x = center_x + offset_x
            aim_y = center_y + offset_y
        return aim_x, aim_y, offset_norm, corridor_limit

    def current_wind_speed(self, now):
        if self.wind_t <= 0.0 or (now - self.wind_t) > self.args.wind_timeout:
            return 0.0
        return self.wind["speed"]

    def compute_velocity_accel_command(
        self,
        ontology,
        mode,
        aim_x,
        aim_y,
        target_z,
        marker_vx,
        marker_vy,
        rel_vx,
        rel_vy,
        altitude_above_pad,
        center_error,
        relative_speed,
        center_ready,
        relative_speed_ready,
    ):
        final_mode = mode in ("FINAL_DESCENT", "LAND_READY")
        ex = aim_x - self.drone["x"]
        ey = aim_y - self.drone["y"]
        values = ontology["values"]
        kp = values["velocityKp"]
        kd = values["velocityKd"]
        max_speed = values["velocityMaxSpeed"]
        relative_distance_risk = values["relativeDistanceRisk"]
        relative_velocity_risk = values["relativeVelocityRisk"]
        terminal_relative_motion_risk = values["terminalRelativeMotionRisk"]
        kd += self.args.relative_motion_command_damping_gain * terminal_relative_motion_risk
        max_speed *= 1.0 - self.args.relative_motion_command_speed_scale * terminal_relative_motion_risk
        max_speed = max(self.args.ontology_min_velocity_speed, max_speed)
        if final_mode and altitude_above_pad <= self.args.terminal_marker_prediction_altitude_above_pad:
            close_scale = clamp(
                center_error / max(self.args.centering_error_for_descent, 1.0e-3),
                0.35,
                1.0,
            )
            max_speed *= close_scale

        aim_dist = math.hypot(ex, ey)
        crossing_vx = 0.0
        crossing_vy = 0.0
        if aim_dist > 1.0e-6:
            crossing_speed = values["centerCrossingSpeed"]
            if center_ready and relative_speed_ready:
                crossing_speed *= self.args.center_crossing_ready_scale
            crossing_vx = ex / aim_dist * crossing_speed
            crossing_vy = ey / aim_dist * crossing_speed
        marker_speed = math.hypot(marker_vx, marker_vy)
        moving_pad_speed_floor = min(
            self.args.velocity_control_max_speed,
            marker_speed + crossing_speed + self.args.moving_pad_speed_margin,
        )
        max_speed = max(max_speed, moving_pad_speed_floor)

        desired_vx = marker_vx + crossing_vx + kp * ex - kd * rel_vx
        desired_vy = marker_vy + crossing_vy + kp * ey - kd * rel_vy
        desired_vx, desired_vy = limit_vector(desired_vx, desired_vy, max_speed)

        if final_mode and not (center_ready and relative_speed_ready):
            # Keep final descent lateral motion calm while still correcting toward pad center.
            terminal_speed_floor = min(
                self.args.velocity_control_max_speed,
                marker_speed + crossing_speed + self.args.moving_pad_speed_margin,
            )
            desired_vx, desired_vy = limit_vector(
                desired_vx,
                desired_vy,
                max(
                    self.args.ontology_min_velocity_speed,
                    0.75 * values["velocityMaxSpeed"],
                    terminal_speed_floor,
                ),
            )

        z_error = target_z - self.drone["z"]
        vz_up = clamp(
            values["verticalVelocityKp"] * z_error,
            -values["verticalMaxDown"],
            self.args.vertical_velocity_control_max_up,
        )
        if final_mode and not (center_ready and relative_speed_ready):
            vz_up = max(vz_up, -values["verticalMaxDown"] * self.args.unsafe_descent_scale)

        wind_x = self.wind["x"] if self.current_wind_speed(time.monotonic()) > 0.0 else 0.0
        wind_y = self.wind["y"] if self.current_wind_speed(time.monotonic()) > 0.0 else 0.0
        ax_cmd = (
            values["accelKp"] * (desired_vx - self.drone["vx"])
            - values["accelKd"] * rel_vx
            - self.args.accel_control_inertia_gain * self.drone.get("ax", 0.0)
            - self.args.accel_control_wind_gain * wind_x
        )
        ay_cmd = (
            values["accelKp"] * (desired_vy - self.drone["vy"])
            - values["accelKd"] * rel_vy
            - self.args.accel_control_inertia_gain * self.drone.get("ay", 0.0)
            - self.args.accel_control_wind_gain * wind_y
        )
        brake_gain = self.args.relative_motion_command_brake_gain * terminal_relative_motion_risk
        ax_cmd -= brake_gain * rel_vx
        ay_cmd -= brake_gain * rel_vy
        moving_pad_accel_floor = min(
            self.args.accel_control_max_accel,
            self.args.moving_pad_speed_accel_gain * marker_speed,
        )
        max_accel = max(values["accelMax"], moving_pad_accel_floor)
        ax_cmd, ay_cmd = limit_vector(ax_cmd, ay_cmd, max_accel)
        az_up_cmd = clamp(
            self.args.accel_control_vertical_gain * (vz_up - self.drone["vz"])
            - self.args.accel_control_inertia_gain * self.drone.get("az", 0.0),
            -max_accel,
            max_accel,
        )

        return {
            "vn": desired_vy,
            "ve": desired_vx,
            "vz_up": vz_up,
            "an": ay_cmd,
            "ae": ax_cmd,
            "az_up": az_up_cmd,
            "relative_speed": relative_speed,
            "relative_distance_risk": relative_distance_risk,
            "relative_velocity_risk": relative_velocity_risk,
            "terminal_relative_motion_risk": terminal_relative_motion_risk,
            "center_crossing_speed": values["centerCrossingSpeed"],
        }

    def publish_velocity_accel_command(self, stamp, cmd):
        vel = TwistStamped()
        vel.header.stamp = stamp
        vel.header.frame_id = "local_ned_z_up"
        vel.twist.linear.x = float(cmd["vn"])
        vel.twist.linear.y = float(cmd["ve"])
        vel.twist.linear.z = float(cmd["vz_up"])
        self.velocity_pub.publish(vel)

        accel = AccelStamped()
        accel.header.stamp = stamp
        accel.header.frame_id = "local_ned_z_up"
        accel.accel.linear.x = float(cmd["an"])
        accel.accel.linear.y = float(cmd["ae"])
        accel.accel.linear.z = float(cmd["az_up"])
        self.accel_pub.publish(accel)

    def velocity_damping_time(self, mode):
        if mode in ("FINAL_DESCENT", "LAND_READY"):
            return max(0.0, self.args.final_velocity_damping_time)
        if mode == "PAD_CAPTURE":
            return max(0.0, self.args.capture_velocity_damping_time)
        return max(0.0, self.args.intercept_velocity_damping_time)

    def handle_marker_search_timeout(self, now, marker_age, camera_marker_age, reason):
        if self.args.marker_search_timeout <= 0.0:
            return False
        if self.marker_lost_since is None:
            self.marker_lost_since = now
            self.search_timeout_sent = False
            return False
        lost_duration = now - self.marker_lost_since
        if lost_duration < self.args.marker_search_timeout:
            return False
        if not self.search_timeout_sent:
            msg = Bool()
            msg.data = True
            self.land_pub.publish(msg)
            self.land_requested = True
            self.search_timeout_sent = True
            self.node.get_logger().warn(
                f"marker search timeout after {lost_duration:.1f}s: reason={reason}, "
                f"marker_age={marker_age:.2f}, camera_marker_age={camera_marker_age:.2f}; requesting LAND"
            )
        return True

    def commit_terminal_landing_on_marker_loss(self, now, marker_age, camera_marker_age):
        if self.marker is None or self.drone is None:
            return False
        mx, my, mz, _, _, _ = self.predicted_marker_state(now)
        pad_dist = math.hypot(mx - self.drone["x"], my - self.drone["y"])
        altitude_above_pad = self.drone["z"] - mz
        terminal_close = pad_dist <= self.args.terminal_marker_loss_land_radius
        terminal_low = altitude_above_pad <= self.args.terminal_marker_loss_land_altitude_above_pad
        recent_enough = camera_marker_age <= self.args.terminal_marker_loss_land_timeout
        if not (terminal_close and terminal_low and recent_enough):
            return False
        self.node.get_logger().warn(
            f"terminal marker loss while landing: pad_dist={pad_dist:.2f}, "
            f"altitude_above_pad={altitude_above_pad:.2f}, marker_age={marker_age:.2f}, "
            f"camera_marker_age={camera_marker_age:.2f}; committing to LAND"
        )
        self.publish_land_request(pad_dist, mz)
        return True

    def publish_land_request(self, dist, pad_z):
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)
        self.land_requested = True
        now = time.monotonic()
        if now - self.last_log_t >= 0.5:
            altitude_above_pad = self.drone["z"] - pad_z
            self.node.get_logger().warn(
                f"direct LAND requested: dist={dist:.2f}, altitude_above_pad={altitude_above_pad:.2f}, "
                f"stable_time={self.args.direct_land_stable_time:.2f}"
            )
            self.last_log_t = now

    def publish_marker_lost_climb(self, marker_age, camera_marker_age):
        target_z = min(
            self.effective_search_altitude,
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
        yaw = self.compute_setpoint_yaw(north, east)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
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

    def compute_setpoint_yaw(self, north, east):
        if self.drone is None:
            return 0.0
        drone_north = self.drone["y"] - self.origin[1]
        drone_east = self.drone["x"] - self.origin[0]
        step_north = north - drone_north
        step_east = east - drone_east
        if abs(step_north) < 1.0e-6 and abs(step_east) < 1.0e-6:
            return float(self.drone.get("yaw", 0.0))
        return math.atan2(step_east, step_north)

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

    def predicted_approach_point(self, now, lead_time):
        lead_time = clamp(lead_time, self.args.min_lead_time, self.args.max_lead_time)
        future_time = clamp(
            lead_time + self.args.prediction_steps * self.args.prediction_step_time,
            lead_time,
            self.args.prediction_horizon,
        )
        lead = self.predicted_marker_state(now, lead_time)
        future = self.predicted_marker_state(now, future_time)
        blend = clamp(self.args.future_point_blend, 0.0, 1.0)
        aim_x = (1.0 - blend) * lead[0] + blend * future[0]
        aim_y = (1.0 - blend) * lead[1] + blend * future[1]
        return aim_x, aim_y, future_time

    def predicted_marker_state(self, now, extra_time=0.0):
        if self.ekf_state is None:
            extra_time = max(0.0, extra_time)
            return (
                self.marker["x"] + self.marker["vx"] * extra_time,
                self.marker["y"] + self.marker["vy"] * extra_time,
                self.marker["z"] + self.marker["vz"] * extra_time,
                self.marker["vx"],
                self.marker["vy"],
                self.marker["vz"],
            )
        dt = clamp(now - self.ekf_t + max(0.0, extra_time), 0.0, self.args.marker_hold_timeout)
        return (
            self.ekf_state[0] + self.ekf_state[3] * dt,
            self.ekf_state[1] + self.ekf_state[4] * dt,
            self.ekf_state[2] + self.ekf_state[5] * dt,
            self.ekf_state[3],
            self.ekf_state[4],
            self.ekf_state[5],
        )

    def evaluate_ontology(self, marker, current_pad_dist, marker_age):
        _, _, _, mvx, mvy = marker
        horizontal_speed = math.hypot(self.drone["vx"], self.drone["vy"])
        vertical_speed = abs(self.drone["vz"])
        marker_speed = math.hypot(mvx, mvy)
        relative_speed = math.hypot(self.drone["vx"] - mvx, self.drone["vy"] - mvy)
        accel_mag = math.hypot(self.drone.get("ax", 0.0), self.drone.get("ay", 0.0))
        wind_speed = self.current_wind_speed(time.monotonic())
        altitude_above_pad = max(0.0, self.drone["z"] - marker[2])

        position_alignment = 1.0 - clamp(
            current_pad_dist / max(self.args.ontology_position_error_reference, 1.0e-6),
            0.0,
            1.0,
        )
        marker_freshness = 1.0 - clamp(
            marker_age / max(self.args.marker_lost_timeout, 1.0e-6),
            0.0,
            1.0,
        )
        motion_stability = 1.0 - clamp(
            marker_speed / max(self.args.ontology_platform_speed_reference, 1.0e-6),
            0.0,
            1.0,
        )
        visual_stability = clamp(0.70 * marker_freshness + 0.30 * motion_stability, 0.0, 1.0)
        attitude_stability = 1.0 - clamp(
            accel_mag / max(self.args.ontology_inertial_accel_reference, 1.0e-6),
            0.0,
            1.0,
        )
        descent_stability = 1.0 - clamp(
            abs(vertical_speed - self.args.descent_rate)
            / max(self.args.ontology_descent_tolerance, 1.0e-6),
            0.0,
            1.0,
        )
        altitude_state = 1.0 - clamp(
            altitude_above_pad / max(self.args.ontology_altitude_reference, 1.0e-6),
            0.0,
            1.0,
        )
        path_length = math.hypot(current_pad_dist, max(self.drone["z"] - self.args.touchdown_altitude, 0.0))
        time_to_target = path_length / max(horizontal_speed, 0.25)
        time_efficiency = 1.0 - clamp(
            time_to_target / max(self.args.ontology_time_to_target_reference, 1.0e-6),
            0.0,
            1.0,
        )
        drag_risk = clamp(
            0.55 * horizontal_speed / max(self.args.ontology_horizontal_speed_reference, 1.0e-6)
            + 0.45 * marker_speed / max(self.args.ontology_platform_speed_reference, 1.0e-6),
            0.0,
            1.0,
        )
        wind_risk = clamp(
            self.args.ontology_nominal_wind_risk
            + 0.35 * drag_risk
            + 0.45 * wind_speed / max(self.args.ontology_wind_speed_reference, 1.0e-6),
            0.0,
            1.0,
        )
        energy_efficiency = 1.0 - clamp(
            path_length / max(self.args.ontology_path_length_reference, 1.0e-6),
            0.0,
            1.0,
        )
        relative_motion_stability = 1.0 - clamp(
            relative_speed / max(self.args.max_relative_speed_for_descent, 1.0e-6),
            0.0,
            1.0,
        )
        relative_distance_risk = clamp(
            1.0 - current_pad_dist / max(self.args.ontology_relative_distance_reference, 1.0e-6),
            0.0,
            1.0,
        )
        relative_velocity_risk = clamp(
            relative_speed / max(self.args.ontology_relative_velocity_reference, 1.0e-6),
            0.0,
            1.0,
        )
        intercept_distance_authority = clamp(
            (current_pad_dist - self.args.pad_capture_radius)
            / max(self.args.ontology_intercept_distance_reference - self.args.pad_capture_radius, 1.0e-6),
            0.0,
            1.0,
        )
        terminal_relative_motion_risk = clamp(
            relative_distance_risk * relative_velocity_risk,
            0.0,
            1.0,
        )
        provider_confidence = 1.0
        if self.marker_source == self.args.fallback_marker_source_name:
            provider_confidence = self.args.fallback_marker_ontology_confidence
        alignment_state = clamp(
            0.50 * position_alignment
            + 0.20 * motion_stability
            + 0.30 * relative_motion_stability * (1.0 - 0.35 * relative_distance_risk),
            0.0,
            1.0,
        )

        landing_posterior = clamp(
            provider_confidence * (0.55 * visual_stability + 0.45 * alignment_state),
            0.0,
            1.0,
        )
        landing_intent = max(self.args.ontology_landing_intent_floor, landing_posterior)
        operational_risk = clamp(
            0.30 * wind_risk
            + 0.25 * drag_risk
            + 0.15 * terminal_relative_motion_risk
            + 0.20 * (1.0 - energy_efficiency)
            + 0.10 * (1.0 - attitude_stability),
            0.0,
            1.0,
        )
        track_quality = clamp(
            provider_confidence
            * (0.45 * alignment_state + 0.35 * visual_stability + 0.20 * attitude_stability),
            0.0,
            1.0,
        )
        efficiency = clamp(0.55 * energy_efficiency + 0.45 * time_efficiency, 0.0, 1.0)
        altitude_above_pad = self.drone["z"] - marker[2]
        previous_mode = self.guidance_mode
        if current_pad_dist <= self.args.direct_land_radius and altitude_above_pad <= self.args.direct_land_altitude_above_pad:
            guidance_mode = "LAND_READY"
        elif current_pad_dist <= self.args.final_descent_radius:
            guidance_mode = "FINAL_DESCENT"
        elif previous_mode in ("FINAL_DESCENT", "LAND_READY") and current_pad_dist <= self.args.final_descent_exit_radius:
            guidance_mode = "FINAL_DESCENT"
        elif current_pad_dist <= self.args.pad_capture_radius:
            guidance_mode = "PAD_CAPTURE"
        elif previous_mode == "PAD_CAPTURE" and current_pad_dist <= self.args.pad_capture_exit_radius:
            guidance_mode = "PAD_CAPTURE"
        else:
            guidance_mode = "INTERCEPT"
        self.guidance_mode = guidance_mode

        lead_time_max = lerp(
            self.args.min_lead_time,
            self.args.max_lead_time,
            track_quality * efficiency * (1.0 - 0.5 * operational_risk),
        )
        closing_speed = max(0.2, horizontal_speed + 0.35 * marker_speed)
        lead_time = min(
            lead_time_max,
            max(
                self.args.min_lead_time,
                current_pad_dist / max(closing_speed, 1.0e-6)
                + self.args.lead_distance_gain * current_pad_dist
                + marker_age,
            ),
        )
        if guidance_mode == "PAD_CAPTURE":
            capture_progress = 1.0 - clamp(
                (current_pad_dist - self.args.final_descent_radius)
                / max(self.args.pad_capture_radius - self.args.final_descent_radius, 1.0e-6),
                0.0,
                1.0,
            )
            lead_time = min(lead_time, lerp(self.args.capture_max_lead_time, self.args.final_descent_lead_time, capture_progress))
        elif guidance_mode in ("FINAL_DESCENT", "LAND_READY"):
            capture_progress = 1.0
            lead_time = min(lead_time, self.args.final_descent_lead_time)
        else:
            capture_progress = 0.0

        navigation_gain = lerp(
            self.args.ontology_min_navigation_gain,
            self.args.ontology_max_navigation_gain,
            (0.45 + 0.70 * track_quality + 0.25 * time_efficiency)
            * (0.65 + 0.35 * landing_intent)
            * (1.0 - 0.30 * operational_risk),
        )
        max_horizontal_step = lerp(
            self.args.min_step,
            self.args.max_step,
            (0.40 + 0.65 * track_quality + 0.30 * efficiency)
            * (0.80 + 0.20 * landing_intent)
            * (1.0 - 0.20 * operational_risk),
        )
        if guidance_mode == "PAD_CAPTURE":
            max_horizontal_step = min(
                max_horizontal_step,
                max(self.args.min_step, current_pad_dist * self.args.capture_step_gain),
            )
        elif guidance_mode in ("FINAL_DESCENT", "LAND_READY"):
            max_horizontal_step = min(
                max_horizontal_step,
                max(self.args.min_step, current_pad_dist * self.args.final_step_gain),
            )

        descent_authority = clamp(
            landing_intent
            * alignment_state
            * visual_stability
            * attitude_stability
            * descent_stability
            * lerp(0.60, 1.0, altitude_state)
            * energy_efficiency
            * (1.0 - operational_risk),
            0.0,
            1.0,
        )
        descent_rate = lerp(
            self.args.ontology_min_descent_rate,
            self.args.ontology_max_descent_rate,
            descent_authority,
        )
        if guidance_mode == "PAD_CAPTURE":
            descent_rate = max(descent_rate, self.args.capture_descent_rate)
        elif guidance_mode in ("FINAL_DESCENT", "LAND_READY"):
            descent_rate = max(descent_rate, self.args.final_descent_rate)

        overshoot_risk = clamp(
            0.25 * terminal_relative_motion_risk
            + 0.20 * terminal_relative_motion_risk
            + 0.25 * (1.0 - attitude_stability)
            + 0.20 * wind_risk
            + 0.10 * drag_risk
            + 0.10 * (1.0 - provider_confidence),
            0.0,
            1.0,
        )
        velocity_authority = clamp(
            (0.20 + 0.80 * track_quality)
            * (0.35 + 0.65 * alignment_state)
            * (0.35 + 0.65 * relative_motion_stability)
            * (1.0 - 0.60 * terminal_relative_motion_risk)
            * (1.0 - 0.55 * overshoot_risk)
            * (1.0 - 0.35 * operational_risk),
            0.0,
            1.0,
        )
        if guidance_mode == "INTERCEPT":
            velocity_authority = max(
                velocity_authority,
                self.args.ontology_intercept_min_velocity_authority * intercept_distance_authority,
            )
        accel_authority = clamp(
            (0.25 + 0.75 * attitude_stability)
            * (0.30 + 0.70 * relative_motion_stability)
            * (1.0 - 0.65 * terminal_relative_motion_risk)
            * (1.0 - 0.70 * overshoot_risk)
            * (1.0 - 0.35 * operational_risk),
            0.0,
            1.0,
        )
        if guidance_mode == "INTERCEPT":
            accel_authority = max(
                accel_authority,
                self.args.ontology_intercept_min_accel_authority * intercept_distance_authority,
            )
        if guidance_mode == "INTERCEPT":
            mode_speed_scale = 1.0
            mode_accel_scale = 1.0
        elif guidance_mode == "PAD_CAPTURE":
            mode_speed_scale = self.args.ontology_capture_velocity_scale
            mode_accel_scale = self.args.ontology_capture_accel_scale
        else:
            mode_speed_scale = self.args.ontology_final_velocity_scale
            mode_accel_scale = self.args.ontology_final_accel_scale

        velocity_max_speed = mode_speed_scale * lerp(
            self.args.ontology_min_velocity_speed,
            self.args.velocity_control_max_speed,
            velocity_authority,
        )
        accel_max = mode_accel_scale * lerp(
            self.args.ontology_min_accel,
            self.args.accel_control_max_accel,
            accel_authority,
        )
        if guidance_mode in ("FINAL_DESCENT", "LAND_READY"):
            velocity_max_speed = min(velocity_max_speed, self.args.velocity_control_final_max_speed)
            accel_max = min(accel_max, self.args.accel_control_final_max_accel)
        elif guidance_mode == "PAD_CAPTURE":
            velocity_max_speed = min(velocity_max_speed, self.args.velocity_control_capture_max_speed)
            accel_max = min(accel_max, self.args.accel_control_capture_max_accel)
        velocity_kp = lerp(
            self.args.velocity_control_final_kp,
            self.args.velocity_control_kp,
            velocity_authority,
        )
        velocity_kd = lerp(
            self.args.velocity_control_kd,
            self.args.ontology_max_velocity_kd,
            clamp(overshoot_risk + 0.5 * (1.0 - relative_motion_stability), 0.0, 1.0),
        )
        if guidance_mode in ("FINAL_DESCENT", "LAND_READY"):
            velocity_kp = min(velocity_kp, self.args.velocity_control_final_kp)
            velocity_kd = max(velocity_kd, self.args.velocity_control_final_kd)
        accel_kp = lerp(
            self.args.ontology_min_accel_kp,
            self.args.accel_control_kp,
            accel_authority,
        )
        accel_kd = lerp(
            self.args.accel_control_kd,
            self.args.ontology_max_accel_kd,
            overshoot_risk,
        )
        vertical_max_down = lerp(
            self.args.ontology_min_vertical_down_speed,
            self.args.vertical_velocity_control_max_down,
            descent_authority * (1.0 - 0.45 * overshoot_risk),
        )
        if guidance_mode == "INTERCEPT":
            center_crossing_limit = self.args.center_crossing_intercept_speed
        elif guidance_mode == "PAD_CAPTURE":
            center_crossing_limit = self.args.center_crossing_capture_speed
        else:
            center_crossing_limit = self.args.center_crossing_final_speed
        center_slowdown = clamp(
            current_pad_dist / max(self.args.center_crossing_slow_radius, 1.0e-6),
            0.0,
            1.0,
        )
        center_crossing_speed = lerp(
            self.args.center_crossing_min_speed,
            center_crossing_limit,
            clamp(velocity_authority * center_slowdown * (1.0 - 0.50 * terminal_relative_motion_risk), 0.0, 1.0),
        )

        descent_start_radius = max(
            self.args.fast_descent_radius,
            self.args.descent_start_radius
            * lerp(0.65, 1.15, track_quality)
            * lerp(1.05, 0.70, operational_risk),
        )
        final_land_readiness = clamp(landing_intent * track_quality * (1.0 - operational_risk), 0.0, 1.0)

        return {
            "name": "LandingOntologyTrajectoryGraph",
            "values": {
                "landingPosterior": landing_posterior,
                "landingIntent": landing_intent,
                "operationalRisk": operational_risk,
                "trackQuality": track_quality,
                "efficiency": efficiency,
                "leadTime": lead_time,
                "navigationGain": navigation_gain,
                "maxHorizontalStep": max_horizontal_step,
                "descentAuthority": descent_authority,
                "descentRate": descent_rate,
                "descentStartRadius": descent_start_radius,
                "finalLandReadiness": final_land_readiness,
                "guidanceMode": guidance_mode,
                "captureBlend": clamp(0.30 + 0.70 * capture_progress, 0.0, 1.0),
                "overshootRisk": overshoot_risk,
                "relativeDistanceRisk": relative_distance_risk,
                "relativeVelocityRisk": relative_velocity_risk,
                "interceptDistanceAuthority": intercept_distance_authority,
                "terminalRelativeMotionRisk": terminal_relative_motion_risk,
                "velocityAuthority": velocity_authority,
                "accelAuthority": accel_authority,
                "velocityMaxSpeed": velocity_max_speed,
                "velocityKp": velocity_kp,
                "velocityKd": velocity_kd,
                "accelMax": accel_max,
                "accelKp": accel_kp,
                "accelKd": accel_kd,
                "verticalVelocityKp": self.args.vertical_velocity_control_kp,
                "verticalMaxDown": vertical_max_down,
                "centerCrossingSpeed": center_crossing_speed,
            },
            "semantic": {
                "windRisk": wind_risk,
                "dragRisk": drag_risk,
                "visualStability": visual_stability,
                "alignmentState": alignment_state,
                "attitudeStability": attitude_stability,
                "descentStability": descent_stability,
                "relativeMotionStability": relative_motion_stability,
                "relativeDistance": current_pad_dist,
                "relativeSpeed": relative_speed,
                "altitudeState": altitude_state,
                "providerConfidence": provider_confidence,
                "windSpeed": wind_speed,
                "inertialAcceleration": accel_mag,
                "timeEfficiency": time_efficiency,
                "energyEfficiency": energy_efficiency,
            },
        }

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
    parser.add_argument("--wind-topic", default="/iris/wind_world")
    parser.add_argument("--setpoint-topic", default="/obdm/guidance_setpoint")
    parser.add_argument("--velocity-topic", default="/obdm/guidance_velocity")
    parser.add_argument("--accel-topic", default="/obdm/guidance_accel")
    parser.add_argument("--land-topic", default="/obdm/guidance_land")
    parser.add_argument("--origin-topic", default="/obdm/guidance_origin")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z-offset", type=float, default=0.0)
    parser.add_argument("--use-first-odom-origin", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--odom-accel-smoothing", type=float, default=0.30)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--search-altitude", type=float, default=7.0)
    parser.add_argument("--max-search-altitude", type=float, default=None)
    parser.add_argument("--landing-start-altitude", type=float, default=4.0)
    parser.add_argument("--mid-approach-recovery-altitude", type=float, default=1.20)
    parser.add_argument("--takeoff-wait-timeout", type=float, default=12.0)
    parser.add_argument("--require-camera-marker-for-descent", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--touchdown-altitude", type=float, default=0.35)
    parser.add_argument("--touchdown-height-above-pad", type=float, default=0.15)
    parser.add_argument("--terminal-touchdown-height-above-pad", type=float, default=0.05)
    parser.add_argument("--descent-rate", type=float, default=0.35)
    parser.add_argument("--max-descent-rate-near", type=float, default=0.55)
    parser.add_argument("--near-descent-boost", type=float, default=1.35)
    parser.add_argument("--fast-descent-radius", type=float, default=1.2)
    parser.add_argument("--far-descent-scale", type=float, default=0.35)
    parser.add_argument("--descent-start-radius", type=float, default=2.2)
    parser.add_argument("--final-radius", type=float, default=0.35)
    parser.add_argument("--direct-land-radius", type=float, default=0.18)
    parser.add_argument("--direct-land-altitude-above-pad", type=float, default=0.15)
    parser.add_argument("--direct-land-stable-time", type=float, default=0.80)
    parser.add_argument("--direct-land-max-horizontal-speed", type=float, default=0.35)
    parser.add_argument("--direct-land-max-relative-speed", type=float, default=0.25)
    parser.add_argument("--direct-land-min-marker-speed", type=float, default=0.0)
    parser.add_argument("--pad-capture-radius", type=float, default=3.00)
    parser.add_argument("--pad-capture-exit-radius", type=float, default=3.60)
    parser.add_argument("--final-descent-radius", type=float, default=0.90)
    parser.add_argument("--final-descent-exit-radius", type=float, default=1.35)
    parser.add_argument("--capture-max-lead-time", type=float, default=0.60)
    parser.add_argument("--final-descent-lead-time", type=float, default=0.12)
    parser.add_argument("--capture-step-gain", type=float, default=0.42)
    parser.add_argument("--final-step-gain", type=float, default=0.22)
    parser.add_argument("--capture-descent-rate", type=float, default=0.28)
    parser.add_argument("--final-descent-rate", type=float, default=0.22)
    parser.add_argument("--capture-xy-smoothing", type=float, default=0.70)
    parser.add_argument("--final-xy-smoothing", type=float, default=0.90)
    parser.add_argument("--intercept-nominal-speed", type=float, default=1.60)
    parser.add_argument("--intercept-lead-blend", type=float, default=0.65)
    parser.add_argument("--intercept-velocity-damping-time", type=float, default=0.40)
    parser.add_argument("--capture-velocity-damping-time", type=float, default=0.55)
    parser.add_argument("--final-velocity-damping-time", type=float, default=0.75)
    parser.add_argument("--capture-aim-damping-scale", type=float, default=0.25)
    parser.add_argument("--capture-center-corridor-radius", type=float, default=0.90)
    parser.add_argument("--capture-center-corridor-gain", type=float, default=0.40)
    parser.add_argument("--capture-center-corridor-prediction-time", type=float, default=0.25)
    parser.add_argument("--final-center-corridor-radius", type=float, default=0.40)
    parser.add_argument("--final-center-corridor-gain", type=float, default=0.18)
    parser.add_argument("--final-center-corridor-prediction-time", type=float, default=0.08)
    parser.add_argument("--centering-error-for-descent", type=float, default=0.45)
    parser.add_argument("--max-relative-speed-for-descent", type=float, default=0.45)
    parser.add_argument("--centering-hold-height-above-pad", type=float, default=0.70)
    parser.add_argument("--unsafe-descent-scale", type=float, default=0.25)
    parser.add_argument("--descent-center-prediction-time", type=float, default=0.35)
    parser.add_argument("--terminal-response-time", type=float, default=0.25)
    parser.add_argument("--terminal-min-prediction-time", type=float, default=0.15)
    parser.add_argument("--terminal-max-prediction-time", type=float, default=1.60)
    parser.add_argument("--terminal-inertia-speed-gain", type=float, default=0.10)
    parser.add_argument("--terminal-inertia-accel-gain", type=float, default=0.05)
    parser.add_argument("--terminal-wind-response-gain", type=float, default=0.03)
    parser.add_argument("--terminal-wind-drift-time", type=float, default=0.08)
    parser.add_argument("--terminal-max-inertia-correction", type=float, default=0.55)
    parser.add_argument("--velocity-control-max-speed", type=float, default=1.80)
    parser.add_argument("--velocity-control-capture-max-speed", type=float, default=0.90)
    parser.add_argument("--velocity-control-final-max-speed", type=float, default=0.45)
    parser.add_argument("--velocity-control-kp", type=float, default=0.75)
    parser.add_argument("--velocity-control-kd", type=float, default=0.55)
    parser.add_argument("--velocity-control-final-kp", type=float, default=0.45)
    parser.add_argument("--velocity-control-final-kd", type=float, default=1.10)
    parser.add_argument("--accel-control-kp", type=float, default=1.00)
    parser.add_argument("--accel-control-kd", type=float, default=0.40)
    parser.add_argument("--accel-control-inertia-gain", type=float, default=0.20)
    parser.add_argument("--accel-control-vertical-gain", type=float, default=0.80)
    parser.add_argument("--accel-control-max-accel", type=float, default=1.20)
    parser.add_argument("--accel-control-capture-max-accel", type=float, default=0.80)
    parser.add_argument("--accel-control-final-max-accel", type=float, default=0.55)
    parser.add_argument("--accel-control-wind-gain", type=float, default=0.08)
    parser.add_argument("--vertical-velocity-control-kp", type=float, default=0.80)
    parser.add_argument("--vertical-velocity-control-max-down", type=float, default=0.55)
    parser.add_argument("--vertical-velocity-control-max-up", type=float, default=0.25)
    parser.add_argument("--ontology-min-velocity-speed", type=float, default=0.20)
    parser.add_argument("--ontology-capture-velocity-scale", type=float, default=0.80)
    parser.add_argument("--ontology-final-velocity-scale", type=float, default=0.60)
    parser.add_argument("--ontology-max-velocity-kd", type=float, default=1.35)
    parser.add_argument("--ontology-min-accel", type=float, default=0.18)
    parser.add_argument("--ontology-capture-accel-scale", type=float, default=0.75)
    parser.add_argument("--ontology-final-accel-scale", type=float, default=0.55)
    parser.add_argument("--ontology-min-accel-kp", type=float, default=0.35)
    parser.add_argument("--ontology-max-accel-kd", type=float, default=0.90)
    parser.add_argument("--ontology-min-vertical-down-speed", type=float, default=0.10)
    parser.add_argument("--relative-motion-command-damping-gain", type=float, default=0.45)
    parser.add_argument("--relative-motion-command-brake-gain", type=float, default=0.55)
    parser.add_argument("--relative-motion-command-speed-scale", type=float, default=0.45)
    parser.add_argument("--center-crossing-min-speed", type=float, default=0.08)
    parser.add_argument("--center-crossing-intercept-speed", type=float, default=1.00)
    parser.add_argument("--center-crossing-capture-speed", type=float, default=0.32)
    parser.add_argument("--center-crossing-final-speed", type=float, default=0.12)
    parser.add_argument("--center-crossing-slow-radius", type=float, default=0.65)
    parser.add_argument("--center-crossing-ready-scale", type=float, default=0.15)
    parser.add_argument("--moving-pad-speed-margin", type=float, default=0.20)
    parser.add_argument("--moving-pad-speed-accel-gain", type=float, default=0.80)
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
    parser.add_argument("--prediction-steps", type=int, default=8)
    parser.add_argument("--prediction-step-time", type=float, default=0.35)
    parser.add_argument("--prediction-horizon", type=float, default=6.0)
    parser.add_argument("--future-point-blend", type=float, default=0.55)
    parser.add_argument("--ontology-position-error-reference", type=float, default=2.5)
    parser.add_argument("--ontology-platform-speed-reference", type=float, default=2.0)
    parser.add_argument("--ontology-horizontal-speed-reference", type=float, default=4.0)
    parser.add_argument("--ontology-path-length-reference", type=float, default=8.0)
    parser.add_argument("--ontology-time-to-target-reference", type=float, default=12.0)
    parser.add_argument("--ontology-descent-tolerance", type=float, default=0.75)
    parser.add_argument("--ontology-nominal-wind-risk", type=float, default=0.10)
    parser.add_argument("--ontology-wind-speed-reference", type=float, default=8.0)
    parser.add_argument("--ontology-inertial-accel-reference", type=float, default=3.0)
    parser.add_argument("--ontology-altitude-reference", type=float, default=5.0)
    parser.add_argument("--ontology-relative-distance-reference", type=float, default=1.50)
    parser.add_argument("--ontology-relative-velocity-reference", type=float, default=1.20)
    parser.add_argument("--ontology-intercept-distance-reference", type=float, default=20.0)
    parser.add_argument("--ontology-intercept-min-velocity-authority", type=float, default=0.70)
    parser.add_argument("--ontology-intercept-min-accel-authority", type=float, default=0.55)
    parser.add_argument("--ontology-landing-intent-floor", type=float, default=0.45)
    parser.add_argument("--ontology-min-navigation-gain", type=float, default=1.50)
    parser.add_argument("--ontology-max-navigation-gain", type=float, default=4.80)
    parser.add_argument("--ontology-min-descent-rate", type=float, default=0.15)
    parser.add_argument("--ontology-max-descent-rate", type=float, default=0.80)
    parser.add_argument("--ekf-position-gain", type=float, default=0.55)
    parser.add_argument("--ekf-velocity-gain", type=float, default=0.20)
    parser.add_argument("--marker-timeout", type=float, default=0.5)
    parser.add_argument("--marker-lost-timeout", type=float, default=1.2)
    parser.add_argument("--camera-marker-guidance-timeout", type=float, default=1.0)
    parser.add_argument("--camera-marker-max-position-jump", type=float, default=1.25)
    parser.add_argument("--camera-marker-max-horizontal-speed", type=float, default=8.0)
    parser.add_argument("--camera-marker-max-vertical-jump", type=float, default=0.35)
    parser.add_argument("--camera-marker-outlier-window", type=float, default=1.0)
    parser.add_argument("--terminal-marker-prediction-altitude-above-pad", type=float, default=0.80)
    parser.add_argument("--terminal-marker-prediction-timeout", type=float, default=2.50)
    parser.add_argument("--terminal-marker-loss-land-radius", type=float, default=0.90)
    parser.add_argument("--terminal-marker-loss-land-altitude-above-pad", type=float, default=0.80)
    parser.add_argument("--terminal-marker-loss-land-timeout", type=float, default=4.0)
    parser.add_argument("--marker-lost-climb-rate", type=float, default=1.20)
    parser.add_argument("--marker-hold-timeout", type=float, default=5.0)
    parser.add_argument("--marker-search-timeout", type=float, default=60.0)
    parser.add_argument("--fallback-after", type=float, default=2.0)
    parser.add_argument("--allow-fallback-marker-for-guidance", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--fallback-marker-source-name", default="rover_gps")
    parser.add_argument("--fallback-marker-guidance-timeout", type=float, default=2.0)
    parser.add_argument("--fallback-marker-horizontal-noise-std", type=float, default=0.25)
    parser.add_argument("--fallback-marker-vertical-noise-std", type=float, default=0.05)
    parser.add_argument("--fallback-marker-velocity-noise-std", type=float, default=0.08)
    parser.add_argument("--fallback-marker-noise-seed", type=int, default=17)
    parser.add_argument("--fallback-marker-ontology-confidence", type=float, default=0.72)
    parser.add_argument("--wind-timeout", type=float, default=2.0)
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
