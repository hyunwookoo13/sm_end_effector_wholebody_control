from __future__ import annotations

import json
import math
from pathlib import Path
import sqlite3
from typing import Any

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


OBJECT_COLORS = {
    "red_can": (0.95, 0.08, 0.08),
    "orange": (1.00, 0.45, 0.05),
    "yellow_box": (1.00, 0.90, 0.05),
    "apple": (0.55, 0.90, 0.15),
    "blue_can": (0.10, 0.35, 1.00),
    "pink_box": (1.00, 0.25, 0.65),
}

ROLE_COLORS = {
    "PICK": (0.95, 0.08, 0.12),
    "PLACE": (1.00, 0.12, 0.62),
}

STATE_LABELS = {
    "IDLE": "READY",
    "RESOLVED": "RESOLVED",
    "NAV_TO_PICK": "NAV_TO_PICK",
    "PICK": "PICKING",
    "NAV_TO_PLACE": "NAV_TO_PLACE",
    "PLACE": "PLACING",
    "DONE": "COMPLETE",
}

ZONE_LABELS = {
    "first_table": ("STATION_A", "FIRST_TABLE", 2.0),
    "second_table": ("STATION_B", "SECOND_TABLE", -2.0),
}

OBJECT_ORDER = {
    "red_can": 0,
    "orange": 1,
    "yellow_box": 2,
    "apple": 3,
    "blue_can": 4,
    "pink_box": 5,
}


def load_objects_read_only(db_path: str | Path) -> list[dict[str, Any]]:
    """Load marker fields without creating or migrating the semantic database."""
    path = Path(db_path).expanduser().resolve()
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    connection.row_factory = sqlite3.Row
    try:
        rows = connection.execute(
            """
            SELECT object_id, canonical_name, semantic_class, capabilities_json,
                   zone, status, frame_id,
                   object_x, object_y, object_z,
                   approach_x, approach_y, approach_yaw
            FROM objects
            ORDER BY zone, object_id
            """
        ).fetchall()
    finally:
        connection.close()

    objects: list[dict[str, Any]] = []
    for row in rows:
        item = dict(row)
        item["capabilities"] = json.loads(item.pop("capabilities_json"))
        objects.append(item)
    return objects


def parse_selected_objects(text: str) -> tuple[str, str]:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError):
        return "", ""
    if not isinstance(payload, dict) or payload.get("ok") is not True:
        return "", ""
    pick = payload.get("pick")
    place = payload.get("place")
    if not isinstance(pick, dict) or not isinstance(place, dict):
        return "", ""
    return (
        str(pick.get("object_id", "")).strip(),
        str(place.get("object_id", "")).strip(),
    )


def parse_mission_state(text: str) -> str:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError):
        return ""
    if not isinstance(payload, dict):
        return ""
    return str(payload.get("state", "")).strip().upper()


def _set_color(marker: Marker, rgb: tuple[float, float, float], alpha: float) -> None:
    marker.color.r = float(rgb[0])
    marker.color.g = float(rgb[1])
    marker.color.b = float(rgb[2])
    marker.color.a = float(alpha)


def _marker(frame_id: str, stamp, namespace: str, marker_id: int, kind: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = str(frame_id)
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = int(kind)
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _circle_points(center_x: float, center_y: float, radius: float) -> list[Point]:
    points: list[Point] = []
    for index in range(49):
        angle = 2.0 * math.pi * index / 48.0
        point = Point()
        point.x = center_x + radius * math.cos(angle)
        point.y = center_y + radius * math.sin(angle)
        point.z = 0.08
        points.append(point)
    return points


def _zone_markers(
    objects: list[dict[str, Any]],
    stamp,
) -> list[Marker]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for item in objects:
        grouped.setdefault(str(item["zone"]), []).append(item)

    markers: list[Marker] = []
    for index, (zone, zone_objects) in enumerate(grouped.items()):
        zone_objects.sort(key=lambda item: OBJECT_ORDER.get(str(item["object_id"]), 99))
        frame_id = str(zone_objects[0].get("frame_id") or "map")
        center_x = sum(float(item["object_x"]) for item in zone_objects) / len(zone_objects)
        center_y = sum(float(item["object_y"]) for item in zone_objects) / len(zone_objects)

        boundary = _marker(frame_id, stamp, "semantic_stations", 8000 + index * 2, Marker.LINE_STRIP)
        boundary.scale.x = 0.055
        boundary.points = _circle_points(center_x, center_y, 0.82)
        _set_color(boundary, (0.03, 0.48, 0.62), 0.88)
        markers.append(boundary)

        title = _marker(frame_id, stamp, "semantic_station_labels", 8001 + index * 2, Marker.TEXT_VIEW_FACING)
        title.pose.position.x = center_x
        station_code, station_name, label_offset = ZONE_LABELS.get(
            zone,
            (f"STATION_{index + 1}", zone.upper(), 2.0),
        )
        title.pose.position.y = center_y + label_offset
        title.pose.position.z = 0.35
        title.scale.z = 0.28
        object_names = "\n".join(str(item["object_id"]).upper() for item in zone_objects)
        title.text = f"{station_code}\n{station_name}\n{object_names}"
        _set_color(title, (0.04, 0.12, 0.20), 1.0)
        markers.append(title)
    return markers


def build_marker_array(
    objects: list[dict[str, Any]],
    stamp,
    selected_pick: str = "",
    selected_place: str = "",
    mission_state: str = "IDLE",
    status_frame: str = "chassis_link",
) -> MarkerArray:
    markers = MarkerArray()
    markers.markers.extend(_zone_markers(objects, stamp))

    for index, item in enumerate(objects):
        object_id = str(item["object_id"])
        frame_id = str(item.get("frame_id") or "map")
        selected_role = ""
        if object_id == selected_pick:
            selected_role = "PICK"
        elif object_id == selected_place:
            selected_role = "PLACE"
        selected = bool(selected_role)
        base_id = index * 10

        selection_color = ROLE_COLORS.get(selected_role, (0.03, 0.48, 0.62))
        if selected:
            halo = _marker(frame_id, stamp, "semantic_selection", base_id, Marker.LINE_STRIP)
            halo.scale.x = 0.085
            halo.points = _circle_points(
                float(item["object_x"]),
                float(item["object_y"]),
                0.48,
            )
            _set_color(halo, selection_color, 1.0)
            markers.markers.append(halo)

        symbol = _marker(frame_id, stamp, "semantic_objects", base_id + 1, Marker.CUBE)
        symbol.pose.position.x = float(item["object_x"])
        symbol.pose.position.y = float(item["object_y"])
        symbol.pose.position.z = 0.12
        if str(item["semantic_class"]) == "box":
            symbol.scale.x = 0.32
            symbol.scale.y = 0.24
            symbol.scale.z = 0.14
        else:
            symbol.type = Marker.CYLINDER if str(item["semantic_class"]) == "can" else Marker.SPHERE
            symbol.scale.x = 0.24
            symbol.scale.y = 0.24
            symbol.scale.z = 0.24
        if selected:
            symbol.scale.x *= 1.25
            symbol.scale.y *= 1.25
            symbol.scale.z *= 1.25
        _set_color(symbol, OBJECT_COLORS.get(object_id, (0.4, 0.8, 1.0)), 0.95)
        markers.markers.append(symbol)

        if selected:
            yaw = float(item["approach_yaw"])
            approach = _marker(frame_id, stamp, "semantic_approaches", base_id + 2, Marker.ARROW)
            approach.pose.position.x = float(item["approach_x"])
            approach.pose.position.y = float(item["approach_y"])
            approach.pose.position.z = 0.10
            approach.pose.orientation.z = math.sin(yaw * 0.5)
            approach.pose.orientation.w = math.cos(yaw * 0.5)
            approach.scale.x = 0.76
            approach.scale.y = 0.14
            approach.scale.z = 0.14
            _set_color(approach, selection_color, 1.0)
            markers.markers.append(approach)

            label = _marker(frame_id, stamp, "semantic_labels", base_id + 3, Marker.TEXT_VIEW_FACING)
            label.pose.position.x = float(item["approach_x"]) - 0.70 * math.cos(yaw)
            label.pose.position.y = float(item["approach_y"]) - 0.70 * math.sin(yaw)
            label.pose.position.z = 0.42
            label.scale.z = 0.38
            label.text = f"{selected_role}\n{object_id.upper()}"
            _set_color(label, selection_color, 1.0)
            markers.markers.append(label)

    status = _marker(status_frame, stamp, "semantic_mission", 9000, Marker.TEXT_VIEW_FACING)
    status.pose.position.x = -3.0
    status.pose.position.y = 2.0
    status.pose.position.z = 0.45
    status.scale.z = 0.40
    task_text = "NO_TASK"
    if selected_pick or selected_place:
        task_text = f"{selected_pick.upper()}>{selected_place.upper()}"
    normalized_state = str(mission_state or "IDLE").upper()
    state_label = STATE_LABELS.get(normalized_state, normalized_state.replace("_", " "))
    status.text = f"MISSION\n{state_label}\n{task_text}"
    state_color = (0.02, 0.55, 0.12) if normalized_state == "DONE" else (0.05, 0.18, 0.85)
    _set_color(status, state_color, 1.0)
    markers.markers.append(status)
    return markers


class SemanticMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("semantic_map_marker_publisher")
        self.declare_parameter("db_path", "")
        self.declare_parameter("marker_topic", "/semantic_map/markers")
        self.declare_parameter(
            "resolved_task_topic", "/semantic_lookup/resolved_task"
        )
        self.declare_parameter("status_topic", "/semantic_mvp/status")
        self.declare_parameter("status_frame", "chassis_link")
        self.declare_parameter("republish_period_sec", 1.0)

        self.db_path = str(self.get_parameter("db_path").value).strip()
        self.status_frame = str(self.get_parameter("status_frame").value).strip()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("resolved_task_topic").value),
            self.on_resolved_task,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self.on_status,
            10,
        )

        self.objects: list[dict[str, Any]] = []
        self.selected_pick = ""
        self.selected_place = ""
        self.mission_state = "IDLE"
        self.load_error = ""
        period = max(
            0.2,
            float(self.get_parameter("republish_period_sec").value),
        )
        self.timer = self.create_timer(period, self.publish_markers)
        self.load_database()
        self.publish_markers()

    def load_database(self) -> bool:
        if not self.db_path:
            self.load_error = "db_path is empty"
            self.get_logger().error(self.load_error)
            return False
        try:
            self.objects = load_objects_read_only(self.db_path)
        except (OSError, sqlite3.Error, json.JSONDecodeError) as exc:
            error = f"waiting for semantic DB: {exc}"
            if error != self.load_error:
                self.get_logger().warn(error)
                self.load_error = error
            return False
        self.load_error = ""
        self.get_logger().info(
            f"Semantic RViz markers ready: db={self.db_path}, "
            f"objects={len(self.objects)}"
        )
        return True

    def on_resolved_task(self, message: String) -> None:
        selected_pick, selected_place = parse_selected_objects(message.data)
        if not selected_pick or not selected_place:
            return
        self.selected_pick = selected_pick
        self.selected_place = selected_place
        self.mission_state = "RESOLVED"
        self.publish_markers()

    def on_status(self, message: String) -> None:
        state = parse_mission_state(message.data)
        if not state:
            return
        self.mission_state = state
        self.publish_markers()

    def publish_markers(self) -> None:
        if not self.objects and not self.load_database():
            return
        markers = build_marker_array(
            self.objects,
            self.get_clock().now().to_msg(),
            selected_pick=self.selected_pick,
            selected_place=self.selected_place,
            mission_state=self.mission_state,
            status_frame=self.status_frame,
        )
        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
