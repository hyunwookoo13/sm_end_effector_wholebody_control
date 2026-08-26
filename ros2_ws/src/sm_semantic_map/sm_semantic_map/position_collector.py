from __future__ import annotations

import argparse
import json
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sm_semantic_map.geometry import Point3, summarize_points, world_to_map


class PositionCollector(Node):
    def __init__(self, topic: str, objects: list[str]) -> None:
        super().__init__("semantic_object_position_collector")
        self.expected = {name.strip().lower() for name in objects if name.strip()}
        self.points: dict[str, list[Point3]] = {name: [] for name in self.expected}
        self.confidences: dict[str, list[float]] = {name: [] for name in self.expected}
        self.create_subscription(String, topic, self._on_detections, 10)

    def _on_detections(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        for detection in payload.get("objects", []):
            if not isinstance(detection, dict):
                continue
            name = str(detection.get("object_name", "")).strip().lower()
            position = detection.get("position_target_frame")
            if name not in self.points or not isinstance(position, dict):
                continue
            if str(position.get("frame_id", "")) != "world":
                continue
            try:
                point = Point3(
                    float(position["x"]),
                    float(position["y"]),
                    float(position["z"]),
                )
                confidence = float(detection.get("confidence", 0.0))
            except (KeyError, TypeError, ValueError):
                continue
            self.points[name].append(point)
            self.confidences[name].append(confidence)

    def complete(self, sample_count: int) -> bool:
        return bool(self.points) and all(
            len(values) >= sample_count for values in self.points.values()
        )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--objects", nargs="+", required=True)
    parser.add_argument("--samples", type=int, default=12)
    parser.add_argument("--timeout", type=float, default=40.0)
    parser.add_argument("--map-x-in-world", type=float, default=2.0666)
    parser.add_argument("--map-y-in-world", type=float, default=-3.6084)
    parser.add_argument("--map-yaw-in-world", type=float, default=-0.052598)
    return parser.parse_args(argv)


def build_result(node: PositionCollector, args: argparse.Namespace) -> dict[str, object]:
    result: dict[str, object] = {}
    for name in sorted(node.points):
        world_summary = summarize_points(node.points[name])
        map_points = [
            world_to_map(
                point,
                args.map_x_in_world,
                args.map_y_in_world,
                args.map_yaw_in_world,
            )
            for point in node.points[name]
        ]
        result[name] = {
            "world": world_summary,
            "map": summarize_points(map_points),
            "confidence_median": (
                statistics.median(node.confidences[name])
                if node.confidences[name]
                else None
            ),
        }
    return result


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    rclpy.init()
    node = PositionCollector(args.topic, args.objects)
    deadline = time.monotonic() + max(0.1, args.timeout)
    try:
        while time.monotonic() < deadline and not node.complete(max(1, args.samples)):
            rclpy.spin_once(node, timeout_sec=0.2)
        print(json.dumps(build_result(node, args), indent=2, sort_keys=True), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
