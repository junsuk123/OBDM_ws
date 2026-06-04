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


def lerp(a, b, t):
    return a + (b - a) * clamp(t, 0.0, 1.0)


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
        self.last_ontology = None
        self.guidance_mode = "SEARCH"
        self.direct_land_since = None
        self.land_requested = False
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
        self.land_pub = self.node.create_publisher(Bool, args.land_topic, 10)
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
            marker_recovery_ready = (
                self.marker is not None
                and self.drone["z"] >= self.args.mid_approach_recovery_altitude
            )
            if self.drone["z"] >= self.args.landing_start_altitude or marker_recovery_ready:
                self.landing_enabled = True
                self.last_target_world = None
                self.node.get_logger().info(
                    f"landing trajectory enabled at drone_z={self.drone['z']:.2f} "
                    f"(threshold={self.args.landing_start_altitude:.2f}, recovery={marker_recovery_ready})"
                )
            else:
                self.log_diag(
                    f"waiting for takeoff altitude: drone_z={self.drone['z']:.2f}, "
                    f"threshold={self.args.landing_start_altitude:.2f}, "
                    f"recovery_threshold={self.args.mid_approach_recovery_altitude:.2f}"
                )
                return

        now = time.monotonic()
        marker_age = now - self.marker_t if self.marker_t > 0.0 else float("inf")
        camera_marker_age = now - self.camera_marker_t if self.camera_marker_t > 0.0 else float("inf")
        if self.marker is None or marker_age > self.args.marker_hold_timeout:
            self.publish_marker_lost_climb(marker_age, camera_marker_age)
            return

        marker_age = max(0.0, marker_age)
        mx, my, mz, mvx, mvy, _ = self.predicted_marker_state(now)
        dx = mx - self.drone["x"]
        dy = my - self.drone["y"]
        current_pad_dist = math.hypot(dx, dy)
        ontology = self.evaluate_ontology(
            marker=(mx, my, mz, mvx, mvy),
            current_pad_dist=current_pad_dist,
            marker_age=marker_age,
        )
        self.last_ontology = ontology
        mode = ontology["values"]["guidanceMode"]
        lead_time = ontology["values"]["leadTime"]
        aim_x, aim_y, future_time = self.predicted_approach_point(now, lead_time)
        if mode == "PAD_CAPTURE":
            capture_blend = ontology["values"]["captureBlend"]
            aim_x = capture_blend * mx + (1.0 - capture_blend) * aim_x
            aim_y = capture_blend * my + (1.0 - capture_blend) * aim_y
        elif mode in ("FINAL_DESCENT", "LAND_READY"):
            final_lead = self.args.final_descent_lead_time
            aim_x = mx + mvx * final_lead
            aim_y = my + mvy * final_lead
            future_time = final_lead
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
            if current_pad_dist <= self.args.final_radius or mode in ("FINAL_DESCENT", "LAND_READY"):
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
            if mode == "PAD_CAPTURE":
                a = max(a, self.args.capture_xy_smoothing)
            elif mode in ("FINAL_DESCENT", "LAND_READY"):
                a = max(a, self.args.final_xy_smoothing)
            tx = (1.0 - a) * self.last_target_world[0] + a * tx
            ty = (1.0 - a) * self.last_target_world[1] + a * ty

        descent_rate = ontology["values"]["descentRate"]
        descent_start_radius = ontology["values"]["descentStartRadius"]
        if mode in ("FINAL_DESCENT", "LAND_READY"):
            descent_rate = min(self.args.max_descent_rate_near, max(descent_rate, self.args.final_descent_rate))
        elif mode == "PAD_CAPTURE":
            descent_rate = min(self.args.max_descent_rate_near, max(descent_rate, self.args.capture_descent_rate))
        elif dist > descent_start_radius:
            descent_rate *= self.args.far_descent_scale
        elif dist <= self.args.fast_descent_radius:
            descent_rate = min(self.args.max_descent_rate_near, descent_rate * self.args.near_descent_boost)
        touchdown_z = max(self.args.touchdown_altitude, mz + self.args.touchdown_height_above_pad)
        target_z = max(touchdown_z, self.drone["z"] - descent_rate / max(self.args.rate, 1.0e-3))
        if self.last_target_world is not None:
            target_z = min(target_z, self.last_target_world[2])

        if self.should_direct_land(now, current_pad_dist, mz):
            self.publish_land_request(current_pad_dist, mz)
            return

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
                f"marker=({mx:.2f},{my:.2f},{mz:.2f}) aim=({aim_x:.2f},{aim_y:.2f}) "
                f"aim_dist={dist:.2f} pad_dist={current_pad_dist:.2f} "
                f"lead_time={lead_time:.2f} future_time={future_time:.2f} marker_age={marker_age:.2f} "
                f"ontology=mode:{mode},intent:{ontology['values']['landingIntent']:.2f},"
                f"risk:{ontology['values']['operationalRisk']:.2f},"
                f"track:{ontology['values']['trackQuality']:.2f},"
                f"step:{ontology['values']['maxHorizontalStep']:.2f},"
                f"descent:{ontology['values']['descentRate']:.2f}"
            )
            self.last_log_t = now

    def should_direct_land(self, now, dist, pad_z):
        altitude_above_pad = self.drone["z"] - pad_z
        horizontal_speed = math.hypot(self.drone["vx"], self.drone["vy"])
        aligned = dist <= self.args.direct_land_radius
        low_enough = altitude_above_pad <= self.args.direct_land_altitude_above_pad
        slow_enough = horizontal_speed <= self.args.direct_land_max_horizontal_speed
        if aligned and low_enough and slow_enough:
            if self.direct_land_since is None:
                self.direct_land_since = now
            if self.land_requested:
                return True
            return (now - self.direct_land_since) >= self.args.direct_land_stable_time
        self.direct_land_since = None
        self.land_requested = False
        return False

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
        attitude_stability = 1.0
        descent_stability = 1.0 - clamp(
            abs(vertical_speed - self.args.descent_rate)
            / max(self.args.ontology_descent_tolerance, 1.0e-6),
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
        wind_risk = clamp(self.args.ontology_nominal_wind_risk + 0.35 * drag_risk, 0.0, 1.0)
        energy_efficiency = 1.0 - clamp(
            path_length / max(self.args.ontology_path_length_reference, 1.0e-6),
            0.0,
            1.0,
        )
        alignment_state = clamp(0.75 * position_alignment + 0.25 * motion_stability, 0.0, 1.0)

        landing_posterior = clamp(0.55 * visual_stability + 0.45 * alignment_state, 0.0, 1.0)
        landing_intent = max(self.args.ontology_landing_intent_floor, landing_posterior)
        operational_risk = clamp(
            0.30 * wind_risk
            + 0.25 * drag_risk
            + 0.20 * (1.0 - energy_efficiency)
            + 0.15 * (1.0 - attitude_stability)
            + 0.10 * (1.0 - visual_stability),
            0.0,
            1.0,
        )
        track_quality = clamp(
            0.45 * alignment_state + 0.35 * visual_stability + 0.20 * attitude_stability,
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
            },
            "semantic": {
                "windRisk": wind_risk,
                "dragRisk": drag_risk,
                "visualStability": visual_stability,
                "alignmentState": alignment_state,
                "attitudeStability": attitude_stability,
                "descentStability": descent_stability,
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
    parser.add_argument("--setpoint-topic", default="/obdm/guidance_setpoint")
    parser.add_argument("--land-topic", default="/obdm/guidance_land")
    parser.add_argument("--origin-topic", default="/obdm/guidance_origin")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--origin-z-offset", type=float, default=0.0)
    parser.add_argument("--use-first-odom-origin", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--search-altitude", type=float, default=7.0)
    parser.add_argument("--landing-start-altitude", type=float, default=4.0)
    parser.add_argument("--mid-approach-recovery-altitude", type=float, default=1.20)
    parser.add_argument("--touchdown-altitude", type=float, default=0.35)
    parser.add_argument("--touchdown-height-above-pad", type=float, default=0.15)
    parser.add_argument("--descent-rate", type=float, default=0.35)
    parser.add_argument("--max-descent-rate-near", type=float, default=0.55)
    parser.add_argument("--near-descent-boost", type=float, default=1.35)
    parser.add_argument("--fast-descent-radius", type=float, default=1.2)
    parser.add_argument("--far-descent-scale", type=float, default=0.35)
    parser.add_argument("--descent-start-radius", type=float, default=2.2)
    parser.add_argument("--final-radius", type=float, default=0.35)
    parser.add_argument("--direct-land-radius", type=float, default=0.65)
    parser.add_argument("--direct-land-altitude-above-pad", type=float, default=1.20)
    parser.add_argument("--direct-land-stable-time", type=float, default=0.70)
    parser.add_argument("--direct-land-max-horizontal-speed", type=float, default=1.20)
    parser.add_argument("--pad-capture-radius", type=float, default=3.00)
    parser.add_argument("--pad-capture-exit-radius", type=float, default=3.60)
    parser.add_argument("--final-descent-radius", type=float, default=0.90)
    parser.add_argument("--final-descent-exit-radius", type=float, default=1.35)
    parser.add_argument("--capture-max-lead-time", type=float, default=0.60)
    parser.add_argument("--final-descent-lead-time", type=float, default=0.12)
    parser.add_argument("--capture-step-gain", type=float, default=0.60)
    parser.add_argument("--final-step-gain", type=float, default=0.30)
    parser.add_argument("--capture-descent-rate", type=float, default=0.38)
    parser.add_argument("--final-descent-rate", type=float, default=0.58)
    parser.add_argument("--capture-xy-smoothing", type=float, default=0.70)
    parser.add_argument("--final-xy-smoothing", type=float, default=0.90)
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
    parser.add_argument("--ontology-landing-intent-floor", type=float, default=0.45)
    parser.add_argument("--ontology-min-navigation-gain", type=float, default=1.50)
    parser.add_argument("--ontology-max-navigation-gain", type=float, default=4.80)
    parser.add_argument("--ontology-min-descent-rate", type=float, default=0.15)
    parser.add_argument("--ontology-max-descent-rate", type=float, default=0.80)
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
