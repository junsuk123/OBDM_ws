#!/usr/bin/env python3
"""Visualize live ROS 2 node-topic relationships in a matplotlib window.

This is a lightweight runtime graph viewer for checking how the current ROS 2
nodes are connected through topics. It is intended for quick inspection of
publisher/subscriber relations during simulation.
"""

import argparse
import re
from dataclasses import dataclass
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
import rclpy


@dataclass(frozen=True)
class GraphEdge:
    source: str
    target: str
    kind: str


def format_node_label(namespace: str, name: str) -> str:
    namespace = namespace.rstrip("/")
    if not namespace or namespace == "/":
        return f"/{name}"
    return f"{namespace}/{name}" if namespace.startswith("/") else f"/{namespace}/{name}"


class RosGraphVisualizer:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("ros_graph_visualizer")
        self.regex = re.compile(args.include_regex) if args.include_regex else None
        self.figure, self.axes = plt.subplots(figsize=(14, 8))
        self.figure.canvas.manager.set_window_title("ROS 2 Runtime Graph")
        self.last_snapshot = None
        plt.ion()
        self.node.get_logger().info(
            "ROS graph visualizer started. Showing live node-topic relations."
        )

    def matches_filter(self, *values: str) -> bool:
        if self.regex is None:
            return True
        return any(self.regex.search(value) is not None for value in values)

    def collect_snapshot(self) -> Tuple[List[str], List[str], List[GraphEdge]]:
        nodes = set()
        topics = set()
        edges: List[GraphEdge] = []

        topic_names_and_types = self.node.get_topic_names_and_types()
        for topic_name, _ in sorted(topic_names_and_types, key=lambda item: item[0]):
            publishers = self.node.get_publishers_info_by_topic(topic_name)
            subscribers = self.node.get_subscriptions_info_by_topic(topic_name)

            for info in publishers:
                node_label = format_node_label(info.node_namespace, info.node_name)
                if not self.matches_filter(node_label, topic_name):
                    continue
                nodes.add(node_label)
                topics.add(topic_name)
                edges.append(GraphEdge(node_label, topic_name, "publisher"))

            for info in subscribers:
                node_label = format_node_label(info.node_namespace, info.node_name)
                if not self.matches_filter(node_label, topic_name):
                    continue
                nodes.add(node_label)
                topics.add(topic_name)
                edges.append(GraphEdge(topic_name, node_label, "subscriber"))

        return sorted(nodes), sorted(topics), edges

    @staticmethod
    def build_positions(items: List[str], x_value: float) -> Dict[str, Tuple[float, float]]:
        if not items:
            return {}
        count = len(items)
        positions = {}
        for index, item in enumerate(items):
            if count == 1:
                y_value = 0.0
            else:
                y_value = 1.0 - (2.0 * index / (count - 1))
            positions[item] = (x_value, y_value)
        return positions

    def draw(self):
        nodes, topics, edges = self.collect_snapshot()
        self.last_snapshot = (nodes, topics, edges)

        self.axes.clear()
        self.axes.set_axis_off()
        self.axes.set_xlim(-0.35, 1.35)

        total_rows = max(len(nodes), len(topics), 1)
        height = max(6.0, min(18.0, 0.35 * total_rows + 2.0))
        self.figure.set_size_inches(14.0, height, forward=True)

        node_positions = self.build_positions(nodes, 0.0)
        topic_positions = self.build_positions(topics, 1.0)
        all_positions = {**node_positions, **topic_positions}

        for node_label, (x_value, y_value) in node_positions.items():
            self.axes.scatter([x_value], [y_value], s=1100, color="#1f77b4", alpha=0.92, zorder=3)
            self.axes.text(
                x_value,
                y_value,
                node_label,
                ha="center",
                va="center",
                fontsize=9,
                color="white",
                fontweight="bold",
                zorder=4,
            )

        for topic_label, (x_value, y_value) in topic_positions.items():
            self.axes.scatter([x_value], [y_value], s=850, marker="s", color="#5f5f5f", alpha=0.9, zorder=3)
            self.axes.text(
                x_value,
                y_value,
                topic_label,
                ha="center",
                va="center",
                fontsize=8,
                color="white",
                zorder=4,
            )

        for edge in edges:
            start = all_positions.get(edge.source)
            end = all_positions.get(edge.target)
            if start is None or end is None:
                continue
            color = "#2ca02c" if edge.kind == "publisher" else "#ff7f0e"
            rad = 0.08 if edge.kind == "publisher" else -0.08
            arrow = FancyArrowPatch(
                start,
                end,
                arrowstyle="-",
                connectionstyle=f"arc3,rad={rad}",
                linewidth=1.4,
                color=color,
                alpha=0.28,
                zorder=1,
            )
            self.axes.add_patch(arrow)

        self.axes.text(
            0.0,
            1.05,
            "ROS 2 Runtime Graph",
            transform=self.axes.transAxes,
            ha="left",
            va="bottom",
            fontsize=14,
            fontweight="bold",
        )
        self.axes.text(
            0.0,
            1.01,
            f"nodes: {len(nodes)}  topics: {len(topics)}  edges: {len(edges)}",
            transform=self.axes.transAxes,
            ha="left",
            va="bottom",
            fontsize=10,
            color="#444444",
        )

        legend_items = [
            Line2D([0], [0], color="#2ca02c", lw=2, label="publisher"),
            Line2D([0], [0], color="#ff7f0e", lw=2, label="subscriber"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor="#1f77b4", markersize=10, label="ROS node"),
            Line2D([0], [0], marker="s", color="w", markerfacecolor="#5f5f5f", markersize=9, label="topic"),
        ]
        self.axes.legend(handles=legend_items, loc="lower center", ncol=4, frameon=False)
        self.axes.set_title(
            "Live node-topic relationships. Close the window to stop the viewer.",
            fontsize=10,
            color="#555555",
            pad=18,
        )

        self.figure.tight_layout()
        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()
        plt.pause(0.001)


def main():
    parser = argparse.ArgumentParser(description="Visualize live ROS 2 node-topic relations.")
    parser.add_argument(
        "--include-regex",
        default="",
        help="Optional regex filter applied to node labels and topic names.",
    )
    parser.add_argument(
        "--update-rate",
        type=float,
        default=1.0,
        help="Refresh rate in Hz.",
    )
    args = parser.parse_args()

    if args.update_rate <= 0.0:
        raise ValueError("--update-rate must be greater than zero")

    rclpy.init()
    viewer = RosGraphVisualizer(args)
    period = 1.0 / args.update_rate

    try:
        while rclpy.ok() and plt.fignum_exists(viewer.figure.number):
            rclpy.spin_once(viewer.node, timeout_sec=period)
            viewer.draw()
    finally:
        viewer.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()