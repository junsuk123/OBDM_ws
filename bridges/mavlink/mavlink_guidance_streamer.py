#!/usr/bin/env python3
"""Continuously stream local setpoints from ROS topics to ArduPilot MAVLink."""

import argparse
import time

from geometry_msgs.msg import PoseStamped
from pymavlink import mavutil
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool


class GuidanceStreamer:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("obdm_mavlink_guidance_streamer")
        self.target = None
        self.filtered_target = None
        self.target_t = 0.0
        self.target_descending = False
        self.land_requested = False
        self.land_sent = False
        self.last_log_t = 0.0
        self.last_stream_log_t = 0.0
        self.current_z_up = None
        self.current_mode = ""
        self.armed = None
        self.last_takeoff_request_t = 0.0
        self.mav = None
        self.node.create_subscription(PoseStamped, args.setpoint_topic, self.on_setpoint, 10)
        self.node.create_subscription(Bool, args.land_topic, self.on_land, 10)
        self.timer = self.node.create_timer(1.0 / args.rate, self.step)

    def on_setpoint(self, msg):
        previous_target = self.target
        self.target = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        self.target_descending = (
            previous_target is not None
            and self.target[2] < previous_target[2] - self.args.descent_deadband
        )
        self.target_t = time.monotonic()
        self.land_requested = False
        now = time.monotonic()
        if now - self.last_log_t > 1.0:
            self.node.get_logger().info(
                f"setpoint LOCAL_NED north={self.target[0]:.2f}, east={self.target[1]:.2f}, z_up={self.target[2]:.2f}"
            )
            self.last_log_t = now

    def on_land(self, msg):
        self.land_requested = bool(msg.data)

    def connect(self):
        if self.mav is not None:
            return True
        last_err = ""
        for uri in self.args.uris.split(","):
            uri = uri.strip()
            if not uri:
                continue
            try:
                mav = mavutil.mavlink_connection(uri, source_system=252, timeout=2)
                heartbeat = mav.wait_heartbeat(timeout=4)
                if heartbeat is None:
                    raise RuntimeError("heartbeat timeout")
                self.mav = mav
                self.current_mode = str(getattr(mav, "flightmode", "") or "")
                self.node.get_logger().info(
                    f"MAVLink guidance connected uri={uri} target={mav.target_system}:{mav.target_component}"
                )
                return True
            except Exception as exc:
                last_err = f"{uri}: {exc}"
        self.node.get_logger().warn(f"MAVLink guidance connect failed: {last_err}", throttle_duration_sec=3.0)
        return False

    def heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
            3,
        )

    def update_vehicle_state(self):
        for _ in range(20):
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "LOCAL_POSITION_NED":
                self.current_z_up = -float(msg.z)
            elif msg_type == "GLOBAL_POSITION_INT":
                self.current_z_up = float(msg.relative_alt) / 1000.0
            elif msg_type == "HEARTBEAT":
                self.current_mode = str(self.mav.flightmode)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg_type == "COMMAND_ACK":
                self.node.get_logger().info(
                    f"COMMAND_ACK command={msg.command} result={msg.result}", throttle_duration_sec=1.0
                )
            elif msg_type == "STATUSTEXT":
                text = getattr(msg, "text", "")
                self.node.get_logger().warn(f"STATUSTEXT: {text}", throttle_duration_sec=2.0)

    def ensure_guided(self):
        if str(self.current_mode).upper() == "GUIDED":
            return
        try:
            mapping = self.mav.mode_mapping() or {}
            if "GUIDED" in mapping:
                self.mav.mav.set_mode_send(
                    self.mav.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mapping["GUIDED"],
                )
            else:
                self.mav.set_mode("GUIDED")
        except Exception as exc:
            self.node.get_logger().warn(f"GUIDED mode request failed: {exc}", throttle_duration_sec=3.0)

    def ensure_airborne_for_climb(self, target_z_up):
        if not self.args.auto_takeoff_on_climb:
            return
        if self.current_z_up is None or self.current_z_up > self.args.auto_takeoff_max_altitude:
            return
        if self.armed is True:
            return
        now = time.monotonic()
        if now - self.last_takeoff_request_t < self.args.takeoff_request_period:
            return
        self.last_takeoff_request_t = now
        self.ensure_guided()
        try:
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            takeoff_alt = min(max(target_z_up, self.args.auto_takeoff_min_altitude), self.args.auto_takeoff_target_altitude)
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                takeoff_alt,
            )
            self.node.get_logger().warn(
                f"auto takeoff requested for climb: target_z_up={target_z_up:.2f}, takeoff_alt={takeoff_alt:.2f}"
            )
        except Exception as exc:
            self.node.get_logger().warn(f"auto takeoff request failed: {exc}", throttle_duration_sec=3.0)

    def step(self):
        if not self.connect():
            return
        try:
            self.heartbeat()
            self.update_vehicle_state()
            if self.land_requested and not self.land_sent:
                self.mav.set_mode("LAND")
                self.land_sent = True
                self.node.get_logger().info("LAND mode requested")
                return
            if self.target is None:
                return
            target_age = time.monotonic() - self.target_t
            if target_age > self.args.target_timeout:
                if self.args.hold_last_target:
                    self.node.get_logger().warn(
                        f"holding stale target age={target_age:.2f}s", throttle_duration_sec=3.0
                    )
                else:
                    return
            if target_age > self.args.target_timeout and not self.args.hold_last_target:
                return
            self.ensure_guided()
            if self.filtered_target is None:
                self.filtered_target = self.target
            else:
                xy_alpha = max(0.0, min(1.0, self.args.tracking_alpha))
                z_alpha = max(0.0, min(1.0, self.args.z_tracking_alpha))
                self.filtered_target = (
                    self.filtered_target[0] + xy_alpha * (self.target[0] - self.filtered_target[0]),
                    self.filtered_target[1] + xy_alpha * (self.target[1] - self.filtered_target[1]),
                    self.filtered_target[2] + z_alpha * (self.target[2] - self.filtered_target[2]),
                )
            north, east, z_up = self.filtered_target
            commanded_z_up = z_up
            climb_limited = False
            vz_down = 0.0
            if self.current_z_up is None:
                vz_down = self.args.descent_rate if self.target_descending else 0.0
            elif commanded_z_up > self.current_z_up + self.args.climb_deadband:
                climb_error = commanded_z_up - self.current_z_up
                self.ensure_airborne_for_climb(commanded_z_up)
                commanded_z_up = min(
                    commanded_z_up,
                    self.current_z_up + self.args.climb_target_lookahead,
                )
                vz_down = -min(
                    self.args.max_climb_rate,
                    max(self.args.min_climb_rate, 0.6 * climb_error),
                )
                climb_limited = True
            elif commanded_z_up < self.current_z_up - self.args.descent_deadband:
                error = self.current_z_up - commanded_z_up
                vz_down = min(self.args.max_descent_rate, max(self.args.min_descent_rate, 0.6 * error))
            z_down = -commanded_z_up
            type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            )
            self.mav.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                north,
                east,
                z_down,
                0,
                0,
                vz_down,
                0,
                0,
                0,
                0,
                0,
            )
            now = time.monotonic()
            if now - self.last_stream_log_t > 2.0:
                current = float("nan") if self.current_z_up is None else self.current_z_up
                self.node.get_logger().info(
                    f"stream target LOCAL_NED north={north:.2f}, east={east:.2f}, "
                    f"raw_z_up={self.target[2]:.2f}, filtered_z_up={z_up:.2f}, "
                    f"cmd_z_up={commanded_z_up:.2f}, z_down={z_down:.2f}, "
                    f"current_z_up={current:.2f}, vz_down={vz_down:.2f}, climb_limited={climb_limited}, "
                    f"target_age={target_age:.2f}, mode={self.current_mode}"
                )
                self.last_stream_log_t = now
        except Exception as exc:
            self.node.get_logger().warn(f"MAVLink guidance stream error: {exc}")
            self.mav = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--setpoint-topic", default="/obdm/guidance_setpoint")
    parser.add_argument("--land-topic", default="/obdm/guidance_land")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--target-timeout", type=float, default=2.0)
    parser.add_argument("--tracking-alpha", type=float, default=0.25)
    parser.add_argument("--z-tracking-alpha", type=float, default=0.85)
    parser.add_argument("--hold-last-target", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--descent-rate", type=float, default=0.35)
    parser.add_argument("--min-descent-rate", type=float, default=0.10)
    parser.add_argument("--max-descent-rate", type=float, default=0.80)
    parser.add_argument("--descent-deadband", type=float, default=0.03)
    parser.add_argument("--min-climb-rate", type=float, default=0.10)
    parser.add_argument("--max-climb-rate", type=float, default=0.40)
    parser.add_argument("--climb-target-lookahead", type=float, default=0.80)
    parser.add_argument("--climb-deadband", type=float, default=0.15)
    parser.add_argument("--auto-takeoff-on-climb", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--auto-takeoff-max-altitude", type=float, default=0.8)
    parser.add_argument("--auto-takeoff-min-altitude", type=float, default=2.0)
    parser.add_argument("--auto-takeoff-target-altitude", type=float, default=5.0)
    parser.add_argument("--takeoff-request-period", type=float, default=5.0)
    parser.add_argument(
        "--uris",
        default="udpin:127.0.0.1:14550,udpin:127.0.0.1:14551,tcp:127.0.0.1:5760",
    )
    args = parser.parse_args()

    rclpy.init()
    streamer = GuidanceStreamer(args)
    try:
        rclpy.spin(streamer.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        streamer.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
