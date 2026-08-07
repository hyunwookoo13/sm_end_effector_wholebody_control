import json
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class FlorenceTargetTfBridge(Node):
    """Florence detection 중심점을 wholebody controller용 target TF로 발행한다."""

    def __init__(self) -> None:
        super().__init__("florence_target_tf_bridge")

        self.declare_parameter("detections_topic", "/sm_florence_2_vlm/detections")
        self.declare_parameter("parent_frame", "chassis_link")
        self.declare_parameter("target_frame", "target_vision")
        self.declare_parameter("object_name", "")
        self.declare_parameter("object_index", 0)
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("timeout_sec", 0.5)
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)

        self.detections_topic = self.get_parameter("detections_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.object_name = str(self.get_parameter("object_name").value).strip().lower()
        self.object_index = int(self.get_parameter("object_index").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_transform: TransformStamped | None = None
        self.latest_update_time = self.get_clock().now()
        self.last_debug = "waiting for detection"

        self.create_subscription(String, self.detections_topic, self.on_detections, 10)
        self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.create_timer(1.0, self.on_log_timer)

        self.get_logger().info(
            f"Publishing Florence target TF: {self.parent_frame} -> {self.target_frame}, "
            f"source={self.detections_topic}, object={self.object_name or '<first>'}"
        )

    def on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.last_debug = f"invalid detection json: {exc}"
            return

        objects = payload.get("objects", [])
        if not isinstance(objects, list):
            self.last_debug = "invalid detection payload: objects is not a list"
            return

        obj = self.select_object(objects)
        if obj is None:
            self.last_debug = "no matching object"
            return

        pos = obj.get("position_target_frame")
        if not isinstance(pos, dict):
            self.last_debug = "selected object has no position_target_frame"
            return

        frame_id = str(pos.get("frame_id", ""))
        if frame_id and frame_id != self.parent_frame:
            self.last_debug = f"position frame mismatch: {frame_id} != {self.parent_frame}"
            return

        try:
            x = float(pos["x"]) + self.offset_x
            y = float(pos["y"]) + self.offset_y
            z = float(pos["z"]) + self.offset_z
        except (KeyError, TypeError, ValueError) as exc:
            self.last_debug = f"invalid position: {exc}"
            return

        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0

        self.latest_transform = transform
        self.latest_update_time = self.get_clock().now()
        self.last_debug = (
            f"{obj.get('object_id', '<object>')}: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            f"conf={float(obj.get('confidence', 0.0)):.2f}"
        )

    def select_object(self, objects: list[Any]) -> dict[str, Any] | None:
        matches: list[dict[str, Any]] = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            confidence = float(obj.get("confidence", 0.0))
            if confidence < self.min_confidence:
                continue
            name = str(obj.get("object_name", "")).strip().lower()
            if self.object_name and name != self.object_name:
                continue
            matches.append(obj)

        if not matches:
            return None
        index = max(0, min(self.object_index, len(matches) - 1))
        return matches[index]

    def on_timer(self) -> None:
        if self.latest_transform is None:
            return

        age = (self.get_clock().now() - self.latest_update_time).nanoseconds * 1e-9
        if age > self.timeout_sec:
            return

        self.latest_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.latest_transform)

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FlorenceTargetTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
