#!/usr/bin/env python3
"""Publish RViz paths and a top-down image for guidance debugging."""

import argparse
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def finite3(values):
    return all(math.isfinite(float(v)) for v in values)


class GuidancePathVisualizer:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("obdm_guidance_path_visualizer")
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.generated_path = Path()
        self.generated_path.header.frame_id = args.fixed_frame
        self.actual_path = Path()
        self.actual_path.header.frame_id = args.fixed_frame
        self.origin = None
        self.z_offset = 0.0
        self.last_generated = None
        self.last_actual = None
        self.latest_actual_pose = None
        self.latest_marker_pose = None
        self.latest_target_pose = None
        self.generated_pub = self.node.create_publisher(Path, args.generated_path_topic, 10)
        self.actual_pub = self.node.create_publisher(Path, args.actual_path_topic, 10)
        self.target_pub = self.node.create_publisher(PoseStamped, args.current_target_topic, 10)
        self.topdown_pub = self.node.create_publisher(Image, args.topdown_image_topic, 2)
        self.node.create_subscription(PoseStamped, args.setpoint_topic, self.on_setpoint, 10)
        self.node.create_subscription(PoseStamped, args.origin_topic, self.on_origin, 10)
        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, qos)
        self.node.create_subscription(Odometry, args.marker_topic, self.on_marker, qos)
        self.image_timer = self.node.create_timer(1.0 / max(args.topdown_rate, 0.1), self.publish_topdown_image)
        self.node.get_logger().info(
            f"guidance path visualizer: setpoints={args.setpoint_topic}, origin={args.origin_topic}, "
            f"odom={args.odom_topic}, marker={args.marker_topic}, frame={args.fixed_frame}, "
            f"topdown_image={args.topdown_image_topic}"
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.origin is None:
            self.origin = (float(p.x), float(p.y))
            self.z_offset = max(0.0, float(p.z) - self.args.takeoff_altitude)
            self.node.get_logger().info(
                f"path origin initialized x={self.origin[0]:.2f}, y={self.origin[1]:.2f}, z_offset={self.z_offset:.2f}"
            )
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.args.fixed_frame
        pose.pose = msg.pose.pose
        self.latest_actual_pose = pose
        if self.should_append(self.last_actual, pose.pose.position):
            self.actual_path.poses.append(pose)
            self.actual_path.poses = self.actual_path.poses[-self.args.max_actual_points :]
            self.last_actual = (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            self.publish_path(self.actual_pub, self.actual_path)

    def on_marker(self, msg):
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.args.fixed_frame
        pose.pose = msg.pose.pose
        self.latest_marker_pose = pose

    def on_origin(self, msg):
        p = msg.pose.position
        origin = (float(p.x), float(p.y))
        z_offset = float(p.z)
        if self.origin is None:
            self.node.get_logger().info(
                f"guidance origin received x={origin[0]:.2f}, y={origin[1]:.2f}, z_offset={z_offset:.2f}"
            )
        self.origin = origin
        self.z_offset = z_offset

    def on_setpoint(self, msg):
        north = float(msg.pose.position.x)
        east = float(msg.pose.position.y)
        z_up = float(msg.pose.position.z)
        if not finite3((north, east, z_up)):
            return
        if self.origin is None:
            ox, oy = 0.0, 0.0
            z_offset = 0.0
        else:
            ox, oy = self.origin
            z_offset = self.z_offset
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.args.fixed_frame
        # Project LOCAL_NED setpoint back into the Gazebo/ROS odom convention:
        # local north -> world y, local east -> world x, z_up -> z.
        pose.pose.position.x = ox + east
        pose.pose.position.y = oy + north
        pose.pose.position.z = z_offset + z_up
        pose.pose.orientation.w = 1.0
        self.latest_target_pose = pose
        self.target_pub.publish(pose)
        if self.args.current_generated_only:
            self.generated_path.poses = []
            if self.latest_actual_pose is not None:
                self.generated_path.poses.append(self.latest_actual_pose)
            self.generated_path.poses.append(pose)
            self.publish_path(self.generated_pub, self.generated_path)
            self.last_generated = (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
        elif self.should_append(self.last_generated, pose.pose.position):
            if self.should_reset_generated_path(pose.pose.position):
                self.generated_path.poses = []
            self.generated_path.poses.append(pose)
            self.generated_path.poses = self.generated_path.poses[-self.args.max_generated_points :]
            self.last_generated = (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.position.z),
            )
            self.publish_path(self.generated_pub, self.generated_path)

    def should_append(self, last, position):
        if last is None:
            return True
        dx = float(position.x) - last[0]
        dy = float(position.y) - last[1]
        dz = float(position.z) - last[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) >= self.args.min_point_spacing

    def should_reset_generated_path(self, position):
        if self.last_generated is None or self.args.max_generated_jump <= 0.0:
            return False
        dx = float(position.x) - self.last_generated[0]
        dy = float(position.y) - self.last_generated[1]
        dz = float(position.z) - self.last_generated[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz) > self.args.max_generated_jump

    def publish_path(self, publisher, path):
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = self.args.fixed_frame
        publisher.publish(path)

    def publish_topdown_image(self):
        width = int(self.args.topdown_width)
        height = int(self.args.topdown_height)
        if width <= 0 or height <= 0:
            return
        data = bytearray([246, 248, 250] * width * height)
        points = self.collect_topdown_points()
        if not points:
            self.topdown_pub.publish(self.make_image(data, width, height))
            return

        min_x, max_x, min_y, max_y = bounds_for_points(points, self.args.topdown_min_span)
        world_to_px = make_projection(min_x, max_x, min_y, max_y, width, height, self.args.topdown_padding_px)
        draw_grid(data, width, height, min_x, max_x, min_y, max_y, world_to_px, self.args.topdown_grid_m)

        actual = [(p.pose.position.x, p.pose.position.y) for p in self.actual_path.poses]
        generated = [(p.pose.position.x, p.pose.position.y) for p in self.generated_path.poses]
        draw_polyline(data, width, height, actual, world_to_px, (35, 110, 220), 3)
        draw_polyline(data, width, height, generated, world_to_px, (235, 126, 35), 3)

        if self.latest_marker_pose is not None:
            p = self.latest_marker_pose.pose.position
            cx, cy = world_to_px(float(p.x), float(p.y))
            draw_square(data, width, height, cx, cy, 8, (35, 170, 80))
        if self.latest_target_pose is not None:
            p = self.latest_target_pose.pose.position
            cx, cy = world_to_px(float(p.x), float(p.y))
            draw_cross(data, width, height, cx, cy, 10, (190, 55, 150))
        if self.latest_actual_pose is not None:
            p = self.latest_actual_pose.pose.position
            cx, cy = world_to_px(float(p.x), float(p.y))
            draw_circle(data, width, height, cx, cy, 8, (30, 95, 210))

        draw_legend(data, width, height)
        self.topdown_pub.publish(self.make_image(data, width, height))

    def collect_topdown_points(self):
        points = []
        for path in (self.actual_path, self.generated_path):
            for pose in path.poses:
                p = pose.pose.position
                if math.isfinite(float(p.x)) and math.isfinite(float(p.y)):
                    points.append((float(p.x), float(p.y)))
        for pose in (self.latest_actual_pose, self.latest_marker_pose, self.latest_target_pose):
            if pose is None:
                continue
            p = pose.pose.position
            if math.isfinite(float(p.x)) and math.isfinite(float(p.y)):
                points.append((float(p.x), float(p.y)))
        return points

    def make_image(self, data, width, height):
        msg = Image()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.args.fixed_frame
        msg.height = height
        msg.width = width
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = bytes(data)
        return msg


def bounds_for_points(points, min_span):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, float(min_span))
    span_y = max(max_y - min_y, float(min_span))
    center_x = 0.5 * (min_x + max_x)
    center_y = 0.5 * (min_y + max_y)
    span = max(span_x, span_y)
    margin = 0.15 * span
    return (
        center_x - 0.5 * span - margin,
        center_x + 0.5 * span + margin,
        center_y - 0.5 * span - margin,
        center_y + 0.5 * span + margin,
    )


def make_projection(min_x, max_x, min_y, max_y, width, height, padding):
    usable_w = max(1, width - 2 * padding)
    usable_h = max(1, height - 2 * padding)

    def project(x, y):
        u = padding + int(round((x - min_x) / max(max_x - min_x, 1.0e-9) * usable_w))
        v = height - padding - int(round((y - min_y) / max(max_y - min_y, 1.0e-9) * usable_h))
        return u, v

    return project


def set_pixel(data, width, height, x, y, color):
    if x < 0 or y < 0 or x >= width or y >= height:
        return
    offset = (y * width + x) * 3
    data[offset : offset + 3] = bytes(color)


def draw_line(data, width, height, x0, y0, x1, y1, color, thickness=1):
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    radius = max(0, thickness // 2)
    while True:
        for ox in range(-radius, radius + 1):
            for oy in range(-radius, radius + 1):
                set_pixel(data, width, height, x0 + ox, y0 + oy, color)
        if x0 == x1 and y0 == y1:
            break
        err2 = 2 * err
        if err2 >= dy:
            err += dy
            x0 += sx
        if err2 <= dx:
            err += dx
            y0 += sy


def draw_polyline(data, width, height, points, project, color, thickness):
    if len(points) < 2:
        return
    projected = [project(float(x), float(y)) for x, y in points]
    for idx in range(1, len(projected)):
        draw_line(data, width, height, projected[idx - 1][0], projected[idx - 1][1], projected[idx][0], projected[idx][1], color, thickness)


def draw_circle(data, width, height, cx, cy, radius, color):
    r2 = radius * radius
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2:
                set_pixel(data, width, height, x, y, color)


def draw_square(data, width, height, cx, cy, half_size, color):
    for y in range(cy - half_size, cy + half_size + 1):
        for x in range(cx - half_size, cx + half_size + 1):
            set_pixel(data, width, height, x, y, color)


def draw_cross(data, width, height, cx, cy, size, color):
    draw_line(data, width, height, cx - size, cy, cx + size, cy, color, 3)
    draw_line(data, width, height, cx, cy - size, cx, cy + size, color, 3)


def draw_grid(data, width, height, min_x, max_x, min_y, max_y, project, spacing):
    spacing = max(float(spacing), 0.25)
    grid_color = (222, 228, 232)
    axis_color = (180, 190, 198)
    x = math.floor(min_x / spacing) * spacing
    while x <= max_x:
        px0, py0 = project(x, min_y)
        px1, py1 = project(x, max_y)
        draw_line(data, width, height, px0, py0, px1, py1, axis_color if abs(x) < 1.0e-6 else grid_color, 1)
        x += spacing
    y = math.floor(min_y / spacing) * spacing
    while y <= max_y:
        px0, py0 = project(min_x, y)
        px1, py1 = project(max_x, y)
        draw_line(data, width, height, px0, py0, px1, py1, axis_color if abs(y) < 1.0e-6 else grid_color, 1)
        y += spacing


def draw_legend(data, width, height):
    items = [
        ((35, 110, 220), 16),
        ((235, 126, 35), 36),
        ((35, 170, 80), 56),
        ((190, 55, 150), 76),
    ]
    for color, y in items:
        draw_square(data, width, height, 18, y, 6, color)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--setpoint-topic", default="/obdm/guidance_setpoint")
    parser.add_argument("--origin-topic", default="/obdm/guidance_origin")
    parser.add_argument("--odom-topic", default="/iris/odometry")
    parser.add_argument("--marker-topic", default="/landing_marker/ground_truth/odom")
    parser.add_argument("--generated-path-topic", default="/obdm/generated_trajectory_path")
    parser.add_argument("--actual-path-topic", default="/obdm/drone_actual_path")
    parser.add_argument("--current-target-topic", default="/obdm/current_guidance_target")
    parser.add_argument("--topdown-image-topic", default="/obdm/topdown_projection_image")
    parser.add_argument("--fixed-frame", default="odom")
    parser.add_argument("--takeoff-altitude", type=float, default=5.0)
    parser.add_argument("--max-points", type=int, default=450)
    parser.add_argument("--max-generated-points", type=int, default=24)
    parser.add_argument("--max-actual-points", type=int, default=160)
    parser.add_argument("--min-point-spacing", type=float, default=0.03)
    parser.add_argument("--max-generated-jump", type=float, default=2.5)
    parser.add_argument("--current-generated-only", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--topdown-width", type=int, default=640)
    parser.add_argument("--topdown-height", type=int, default=640)
    parser.add_argument("--topdown-rate", type=float, default=5.0)
    parser.add_argument("--topdown-min-span", type=float, default=6.0)
    parser.add_argument("--topdown-grid-m", type=float, default=1.0)
    parser.add_argument("--topdown-padding-px", type=int, default=32)
    args = parser.parse_args()

    rclpy.init()
    node = GuidancePathVisualizer(args)
    try:
        rclpy.spin(node.node)
    finally:
        node.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
