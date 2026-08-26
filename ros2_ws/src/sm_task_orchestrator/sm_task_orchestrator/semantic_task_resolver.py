from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sm_semantic_map_interfaces.srv import FindObject


def parse_task_payload(text: str) -> tuple[str, str] | None:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict):
        return None
    pick = str(payload.get("pick", "")).strip()
    place = str(payload.get("place", "")).strip()
    return (pick, place) if pick and place and pick != place else None


def quaternion_yaw(orientation: Any) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def object_response_to_dict(response: FindObject.Response) -> dict[str, object]:
    object_position = response.object_pose.pose.position
    approach_position = response.approach_pose.pose.position
    return {
        "object_id": response.object_id,
        "canonical_name": response.canonical_name,
        "semantic_class": response.semantic_class,
        "capabilities": list(response.capabilities),
        "status": response.status,
        "zone": response.zone,
        "source_sensor": response.source_sensor,
        "object_pose": {
            "frame_id": response.object_pose.header.frame_id,
            "x": float(object_position.x),
            "y": float(object_position.y),
            "z": float(object_position.z),
        },
        "approach_pose": {
            "frame_id": response.approach_pose.header.frame_id,
            "x": float(approach_position.x),
            "y": float(approach_position.y),
            "yaw": quaternion_yaw(response.approach_pose.pose.orientation),
        },
        "approach_clearance_m": float(response.approach_clearance_m),
    }


class SemanticTaskResolver(Node):
    def __init__(self) -> None:
        super().__init__("semantic_task_resolver")
        self.declare_parameter("task_topic", "/semantic_lookup/task")
        self.declare_parameter("resolved_task_topic", "/semantic_lookup/resolved_task")
        self.declare_parameter("status_topic", "/semantic_lookup/status")
        self.declare_parameter("find_service", "/semantic_map/find_object")
        self.declare_parameter("required_pick_status", "AVAILABLE")
        self.declare_parameter("required_place_status", "AVAILABLE")

        self.required_pick_status = str(
            self.get_parameter("required_pick_status").value
        ).strip().upper()
        self.required_place_status = str(
            self.get_parameter("required_place_status").value
        ).strip().upper()
        self.resolved_pub = self.create_publisher(
            String,
            str(self.get_parameter("resolved_task_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("task_topic").value),
            self._on_task,
            10,
        )
        self.client = self.create_client(
            FindObject,
            str(self.get_parameter("find_service").value),
        )
        self.generation = 0
        self.get_logger().info("Semantic task resolver ready; navigation output is disabled")

    def _publish_status(self, ok: bool, reason: str, **extra: object) -> None:
        payload = {"ok": bool(ok), "reason": str(reason), **extra}
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _on_task(self, message: String) -> None:
        parsed = parse_task_payload(message.data)
        if parsed is None:
            self._publish_status(False, "invalid pick/place task")
            return
        if not self.client.service_is_ready():
            self._publish_status(False, "semantic map service is unavailable")
            return

        self.generation += 1
        generation = self.generation
        pick_query, place_query = parsed
        context: dict[str, object] = {
            "input": {"pick": pick_query, "place": place_query},
        }
        request = FindObject.Request()
        request.query = pick_query
        request.required_capability = "pick"
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda completed: self._on_pick_result(completed, generation, context)
        )
        self._publish_status(True, "resolving", **context)

    def _on_pick_result(self, future, generation: int, context: dict[str, object]) -> None:
        if generation != self.generation:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status(False, f"pick lookup failed: {exc}", **context)
            return
        if not response.found:
            self._publish_status(False, response.message, stage="pick", **context)
            return
        if self.required_pick_status and response.status.upper() != self.required_pick_status:
            self._publish_status(
                False,
                f"pick object status is {response.status}",
                stage="pick",
                **context,
            )
            return
        context["pick"] = object_response_to_dict(response)

        request = FindObject.Request()
        request.query = str(context["input"]["place"])
        request.required_capability = "place"
        place_future = self.client.call_async(request)
        place_future.add_done_callback(
            lambda completed: self._on_place_result(completed, generation, context)
        )

    def _on_place_result(self, future, generation: int, context: dict[str, object]) -> None:
        if generation != self.generation:
            return
        try:
            response = future.result()
        except Exception as exc:
            self._publish_status(False, f"place lookup failed: {exc}", **context)
            return
        if not response.found:
            self._publish_status(False, response.message, stage="place", **context)
            return
        if self.required_place_status and response.status.upper() != self.required_place_status:
            self._publish_status(
                False,
                f"place object status is {response.status}",
                stage="place",
                **context,
            )
            return
        context["place"] = object_response_to_dict(response)
        payload = {"ok": True, **context}
        self.resolved_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self._publish_status(True, "resolved", **context)
        self.get_logger().warn(
            f"Semantic task resolved: pick={context['pick']['object_id']}, "
            f"place={context['place']['object_id']}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticTaskResolver()
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
