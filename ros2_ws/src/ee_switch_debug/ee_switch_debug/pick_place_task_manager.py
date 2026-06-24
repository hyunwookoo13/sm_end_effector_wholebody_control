import json
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def normalize_quaternion_xyzw(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def multiply_quaternions(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize_quaternion_xyzw(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(
    vector: tuple[float, float, float],
    q: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    x, y, z = vector
    qx, qy, qz, qw = normalize_quaternion_xyzw(q)
    # q * v * q^-1, expanded to avoid extra dependencies.
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


class PickPlaceTaskManager(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_task_manager")

        self.declare_parameter("task_topic", "/pick_place_task")
        self.declare_parameter("target_objects_topic", "/sm_florence_2_vlm/target_objects")
        self.declare_parameter("detections_topic", "/sm_florence_2_vlm/detections")
        self.declare_parameter("grasp_topic", "/sm_grasping/grasp_best")
        self.declare_parameter("arm_task_command_topic", "/arm_task_command")
        self.declare_parameter("arm_task_state_topic", "/arm_task_state")
        self.declare_parameter("task_state_topic", "/pick_place_task_state")
        self.declare_parameter("parent_frame", "chassis_link")
        self.declare_parameter("target_frame", "pick_place_target")
        self.declare_parameter("pick_object", "can")
        self.declare_parameter("place_object", "dish")
        self.declare_parameter("autostart", False)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("target_objects_publish_period", 1.0)
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("min_place_confidence", 0.0)
        self.declare_parameter("pick_offset_x", 0.0)
        self.declare_parameter("pick_offset_y", 0.0)
        self.declare_parameter("pick_offset_z", 0.0)
        self.declare_parameter("place_offset_x", 0.0)
        self.declare_parameter("place_offset_y", 0.0)
        self.declare_parameter("place_offset_z", 0.0)

        self.task_topic = str(self.get_parameter("task_topic").value)
        self.target_objects_topic = str(self.get_parameter("target_objects_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.grasp_topic = str(self.get_parameter("grasp_topic").value)
        self.arm_task_command_topic = str(self.get_parameter("arm_task_command_topic").value)
        self.arm_task_state_topic = str(self.get_parameter("arm_task_state_topic").value)
        self.task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.pick_object = str(self.get_parameter("pick_object").value).strip()
        self.place_object = str(self.get_parameter("place_object").value).strip()
        self.autostart = bool(self.get_parameter("autostart").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.target_objects_publish_period = float(
            self.get_parameter("target_objects_publish_period").value
        )
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.min_place_confidence = float(self.get_parameter("min_place_confidence").value)
        self.pick_offset = (
            float(self.get_parameter("pick_offset_x").value),
            float(self.get_parameter("pick_offset_y").value),
            float(self.get_parameter("pick_offset_z").value),
        )
        self.place_offset = (
            float(self.get_parameter("place_offset_x").value),
            float(self.get_parameter("place_offset_y").value),
            float(self.get_parameter("place_offset_z").value),
        )

        self.phase = "IDLE"
        self.arm_task_state = ""
        self.current_perception_object = ""
        self.last_target_objects_publish_ns = 0
        self.last_debug = "idle"
        self.pick_transform: TransformStamped | None = None
        self.place_transform: TransformStamped | None = None
        self.current_transform: TransformStamped | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(String, self.task_topic, self.on_task_command, 10)
        self.create_subscription(PoseStamped, self.grasp_topic, self.on_grasp_best, 10)
        self.create_subscription(String, self.detections_topic, self.on_detections, 10)
        self.create_subscription(String, self.arm_task_state_topic, self.on_arm_task_state, 10)
        self.target_objects_pub = self.create_publisher(String, self.target_objects_topic, 10)
        self.arm_task_command_pub = self.create_publisher(String, self.arm_task_command_topic, 10)
        self.task_state_pub = self.create_publisher(String, self.task_state_topic, 10)
        self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.create_timer(1.0, self.on_log_timer)

        if self.autostart and self.pick_object and self.place_object:
            self.start_task(self.pick_object, self.place_object)

        self.get_logger().info(
            f"Pick/place manager target={self.target_frame}, parent={self.parent_frame}, "
            f"task_topic={self.task_topic}"
        )

    def on_task_command(self, msg: String) -> None:
        parsed = self.parse_task(msg.data)
        if parsed is None:
            self.get_logger().warn(
                "Invalid pick/place task. Use 'can,dish' or "
                "'{\"pick\":\"can\", \"place\":\"dish\"}'."
            )
            return
        pick_object, place_object = parsed
        self.start_task(pick_object, place_object)

    def parse_task(self, text: str) -> tuple[str, str] | None:
        raw = (text or "").strip()
        if not raw:
            return None
        if raw.startswith("{") or raw.startswith("["):
            try:
                payload: Any = json.loads(raw)
            except Exception:
                return None
            if isinstance(payload, dict):
                pick = str(payload.get("pick", "")).strip()
                place = str(payload.get("place", "")).strip()
                return (pick, place) if pick and place else None
            if isinstance(payload, list) and len(payload) >= 2:
                pick = str(payload[0]).strip()
                place = str(payload[1]).strip()
                return (pick, place) if pick and place else None
            return None

        tokens = [token.strip() for token in raw.split(",") if token.strip()]
        if len(tokens) < 2:
            return None
        return tokens[0], tokens[1]

    def start_task(self, pick_object: str, place_object: str) -> None:
        self.pick_object = pick_object
        self.place_object = place_object
        self.phase = "FIND_PICK"
        self.arm_task_state = ""
        self.current_perception_object = pick_object
        self.last_target_objects_publish_ns = 0
        self.pick_transform = None
        self.place_transform = None
        self.current_transform = None
        self.publish_target_object(force=True)
        self.publish_arm_command("PICK")
        self.publish_task_state()
        self.get_logger().warn(f"Pick/place task started: pick={pick_object}, place={place_object}")

    def on_grasp_best(self, msg: PoseStamped) -> None:
        if self.phase != "FIND_PICK":
            return
        transform = self.pose_to_parent_transform(
            source_frame=msg.header.frame_id,
            position=(
                float(msg.pose.position.x) + self.pick_offset[0],
                float(msg.pose.position.y) + self.pick_offset[1],
                float(msg.pose.position.z) + self.pick_offset[2],
            ),
            orientation=(
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ),
        )
        if transform is None:
            return
        self.pick_transform = transform
        self.current_transform = transform

    def on_detections(self, msg: String) -> None:
        if self.phase != "FIND_PLACE":
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.last_debug = f"invalid detections json: {exc}"
            return

        objects = payload.get("objects", [])
        if not isinstance(objects, list):
            self.last_debug = "invalid detections payload"
            return

        obj = self.select_detection(objects, self.place_object)
        if obj is None:
            self.last_debug = f"waiting for place object: {self.place_object}"
            return

        pos = obj.get("position_target_frame")
        if not isinstance(pos, dict):
            self.last_debug = "place object has no position_target_frame"
            return

        try:
            transform = self.pose_to_parent_transform(
                source_frame=str(pos.get("frame_id", "")),
                position=(
                    float(pos["x"]) + self.place_offset[0],
                    float(pos["y"]) + self.place_offset[1],
                    float(pos["z"]) + self.place_offset[2],
                ),
                orientation=(0.0, 0.0, 0.0, 1.0),
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.last_debug = f"invalid place position: {exc}"
            return

        if transform is None:
            return
        self.place_transform = transform
        self.current_transform = transform

    def select_detection(self, objects: list[Any], target_name: str) -> dict[str, Any] | None:
        target = target_name.strip().lower()
        matches: list[dict[str, Any]] = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            confidence = float(obj.get("confidence", 0.0))
            if confidence < self.min_place_confidence:
                continue
            name = str(obj.get("object_name", "")).strip().lower()
            if target and target not in name and name not in target:
                continue
            matches.append(obj)
        if not matches:
            return None
        matches.sort(key=lambda item: float(item.get("confidence", 0.0)), reverse=True)
        return matches[0]

    def pose_to_parent_transform(
        self,
        source_frame: str,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> TransformStamped | None:
        if not source_frame:
            self.last_debug = "source frame is empty"
            return None

        target_point = position
        target_orientation = normalize_quaternion_xyzw(orientation)
        if source_frame != self.parent_frame:
            try:
                parent_from_source = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
            except TransformException as exc:
                self.last_debug = f"waiting for transform {self.parent_frame} <- {source_frame}: {exc}"
                return None

            rotation_msg = parent_from_source.transform.rotation
            rotation = (
                float(rotation_msg.x),
                float(rotation_msg.y),
                float(rotation_msg.z),
                float(rotation_msg.w),
            )
            rotated = rotate_vector(position, rotation)
            translation = parent_from_source.transform.translation
            target_point = (
                rotated[0] + float(translation.x),
                rotated[1] + float(translation.y),
                rotated[2] + float(translation.z),
            )
            target_orientation = multiply_quaternions(rotation, target_orientation)

        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = target_point[0]
        transform.transform.translation.y = target_point[1]
        transform.transform.translation.z = target_point[2]
        transform.transform.rotation.x = target_orientation[0]
        transform.transform.rotation.y = target_orientation[1]
        transform.transform.rotation.z = target_orientation[2]
        transform.transform.rotation.w = target_orientation[3]
        return transform

    def on_arm_task_state(self, msg: String) -> None:
        self.arm_task_state = msg.data.strip().upper()

    def on_timer(self) -> None:
        self.publish_target_object(force=False)
        self.advance_phase()
        if self.current_transform is not None:
            self.current_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.current_transform)
        self.publish_task_state()

    def advance_phase(self) -> None:
        if self.phase == "IDLE":
            return

        if self.phase == "FIND_PICK":
            if self.pick_transform is None:
                self.last_debug = f"waiting for pick grasp: {self.pick_object}"
                return
            self.current_transform = self.pick_transform
            self.publish_arm_command("PICK")
            self.phase = "PICK"
            self.get_logger().warn("Task phase: FIND_PICK -> PICK")

        elif self.phase == "PICK":
            if self.arm_task_state == "PICK:HOLD":
                self.current_perception_object = self.place_object
                self.last_target_objects_publish_ns = 0
                self.publish_target_object(force=True)
                self.phase = "FIND_PLACE"
                self.get_logger().warn("Task phase: PICK -> FIND_PLACE")

        elif self.phase == "FIND_PLACE":
            if self.place_transform is None:
                self.last_debug = f"holding object; waiting for place target: {self.place_object}"
                return
            self.current_transform = self.place_transform
            self.publish_arm_command("PLACE")
            self.phase = "PLACE"
            self.get_logger().warn("Task phase: FIND_PLACE -> PLACE")

        elif self.phase == "PLACE":
            if self.arm_task_state == "PLACE:HOLD":
                self.phase = "DONE"
                self.get_logger().warn("Task phase: PLACE -> DONE")

        elif self.phase == "DONE":
            self.last_debug = "pick/place task done"

    def publish_target_object(self, force: bool) -> None:
        if not self.current_perception_object:
            return
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.last_target_objects_publish_ns) * 1e-9
        if not force and elapsed < self.target_objects_publish_period:
            return
        self.target_objects_pub.publish(String(data=self.current_perception_object))
        self.last_target_objects_publish_ns = now_ns

    def publish_arm_command(self, command: str) -> None:
        self.arm_task_command_pub.publish(String(data=command))

    def publish_task_state(self) -> None:
        self.task_state_pub.publish(
            String(
                data=(
                    f"{self.phase}: pick={self.pick_object}, place={self.place_object}, "
                    f"arm={self.arm_task_state or '<none>'}"
                )
            )
        )

    def on_log_timer(self) -> None:
        if self.current_transform is not None:
            t = self.current_transform.transform.translation
            target = f"target=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})"
        else:
            target = "target=<none>"
        self.get_logger().info(
            f"phase={self.phase}, pick={self.pick_object}, place={self.place_object}, "
            f"perception={self.current_perception_object or '<none>'}, "
            f"arm={self.arm_task_state or '<none>'}, {target}, {self.last_debug}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceTaskManager()
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
