import json
from math import cos, hypot, sin
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .navigation_geometry import (
    compute_standoff_pose,
    hybrid_blend_weight,
    rear_sector_clearance,
    should_skip_navigation,
)


COLOR_WORDS = {"red", "blue", "yellow", "green", "pink"}
CLASS_ALIASES = {
    "can": {"can", "tin", "soup can", "tomato soup can"},
    "box": {"box", "bin", "container", "carton", "package", "tray", "plastic tray"},
    "dish": {"dish", "plate", "tray"},
    "mug": {"mug", "cup", "coffee cup", "coffee mug", "teacup", "tea cup"},
    "cup": {"cup", "mug", "coffee cup", "coffee mug", "teacup", "tea cup"},
    "bottle": {"bottle", "water bottle", "drink bottle"},
}


def split_semantic_target(name: str) -> tuple[str | None, str]:
    tokens = str(name).strip().lower().split()
    if tokens and tokens[0] in COLOR_WORDS:
        return tokens[0], " ".join(tokens[1:]) or tokens[0]
    return None, " ".join(tokens)


def class_aliases_for(name: str) -> set[str]:
    _, base_class = split_semantic_target(name)
    aliases = set(CLASS_ALIASES.get(base_class, {base_class}))
    aliases.add(base_class)
    return {alias for alias in aliases if alias}


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
        self.declare_parameter("place_detections_topic", "")
        self.declare_parameter("grasp_topic", "/sm_grasping/grasp_best")
        self.declare_parameter("extra_grasp_topics", [""])
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
        self.declare_parameter("fresh_grasp_delay_sec", 0.35)
        self.declare_parameter("min_place_confidence", 0.0)
        self.declare_parameter("pick_offset_x", 0.0)
        self.declare_parameter("pick_offset_y", 0.0)
        self.declare_parameter("pick_offset_z", 0.0)
        self.declare_parameter("place_offset_x", 0.0)
        self.declare_parameter("place_offset_y", 0.0)
        self.declare_parameter("place_offset_z", 0.0)
        self.declare_parameter("enable_navigation", False)
        self.declare_parameter("navigation_action_name", "/navigate_to_pose")
        self.declare_parameter("navigation_base_frame", "chassis_link")
        self.declare_parameter("pick_standoff_m", 0.70)
        self.declare_parameter("place_standoff_m", 0.70)
        self.declare_parameter("navigation_goal_skip_distance_m", 0.20)
        self.declare_parameter("place_direct_approach_distance_m", 1.20)
        self.declare_parameter("enable_hybrid_handoff", True)
        self.declare_parameter("hybrid_outer_distance_m", 1.40)
        self.declare_parameter("hybrid_inner_distance_m", 0.85)
        self.declare_parameter("hybrid_blend_topic", "/base_control_blend")
        self.declare_parameter("base_control_mode_topic", "/base_control_mode")
        self.declare_parameter("retreat_cmd_topic", "/cmd_vel_retreat")
        self.declare_parameter("rear_scan_topic", "/laser_scan_2")
        self.declare_parameter("retreat_distance_m", 0.40)
        self.declare_parameter("retreat_speed_mps", 0.25)
        self.declare_parameter("retreat_min_clearance_m", 0.50)
        self.declare_parameter("retreat_scan_ignore_below_m", 0.45)
        self.declare_parameter("retreat_sector_half_angle_rad", 0.70)
        self.declare_parameter("retreat_scan_timeout_sec", 0.50)
        self.declare_parameter("retreat_timeout_sec", 5.0)
        self.declare_parameter("transport_settle_sec", 1.0)

        self.task_topic = str(self.get_parameter("task_topic").value)
        self.target_objects_topic = str(self.get_parameter("target_objects_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.place_detections_topic = (
            str(self.get_parameter("place_detections_topic").value).strip()
            or self.detections_topic
        )
        self.grasp_topic = str(self.get_parameter("grasp_topic").value)
        self.grasp_topics = self._load_topic_list("extra_grasp_topics", [self.grasp_topic])
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
        self.fresh_grasp_delay_sec = max(
            0.0,
            float(self.get_parameter("fresh_grasp_delay_sec").value),
        )
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
        self.enable_navigation = bool(self.get_parameter("enable_navigation").value)
        self.navigation_action_name = str(self.get_parameter("navigation_action_name").value)
        self.navigation_base_frame = str(self.get_parameter("navigation_base_frame").value)
        self.pick_standoff_m = max(0.0, float(self.get_parameter("pick_standoff_m").value))
        self.place_standoff_m = max(0.0, float(self.get_parameter("place_standoff_m").value))
        self.navigation_goal_skip_distance_m = max(
            0.0,
            float(self.get_parameter("navigation_goal_skip_distance_m").value),
        )
        self.place_direct_approach_distance_m = max(
            0.0,
            float(self.get_parameter("place_direct_approach_distance_m").value),
        )
        self.enable_hybrid_handoff = bool(
            self.get_parameter("enable_hybrid_handoff").value
        )
        self.hybrid_outer_distance_m = max(
            0.0, float(self.get_parameter("hybrid_outer_distance_m").value)
        )
        self.hybrid_inner_distance_m = max(
            0.0, float(self.get_parameter("hybrid_inner_distance_m").value)
        )
        if self.hybrid_inner_distance_m > self.hybrid_outer_distance_m:
            self.hybrid_inner_distance_m, self.hybrid_outer_distance_m = (
                self.hybrid_outer_distance_m,
                self.hybrid_inner_distance_m,
            )
        self.retreat_distance_m = max(0.0, float(self.get_parameter("retreat_distance_m").value))
        self.retreat_speed_mps = max(0.0, float(self.get_parameter("retreat_speed_mps").value))
        self.retreat_min_clearance_m = max(
            0.0, float(self.get_parameter("retreat_min_clearance_m").value)
        )
        self.retreat_scan_ignore_below_m = max(
            0.0, float(self.get_parameter("retreat_scan_ignore_below_m").value)
        )
        self.retreat_sector_half_angle_rad = max(
            0.0, float(self.get_parameter("retreat_sector_half_angle_rad").value)
        )
        self.retreat_scan_timeout_ns = int(
            max(0.05, float(self.get_parameter("retreat_scan_timeout_sec").value)) * 1e9
        )
        self.retreat_timeout_ns = int(
            max(0.5, float(self.get_parameter("retreat_timeout_sec").value)) * 1e9
        )
        self.transport_settle_ns = int(
            max(0.0, float(self.get_parameter("transport_settle_sec").value)) * 1e9
        )

        self.phase = "IDLE"
        self.arm_task_state = ""
        self.current_perception_object = ""
        self.last_target_objects_publish_ns = 0
        self.last_debug = "idle"
        self.pick_transform: TransformStamped | None = None
        self.place_transform: TransformStamped | None = None
        self.current_transform: TransformStamped | None = None
        self.pick_orientation: tuple[float, float, float, float] | None = None
        self.pick_orientation_frame = ""
        self.pick_orientation_by_source: dict[str, tuple[str, tuple[float, float, float, float]]] = {}
        self.selected_pick_source = ""
        self.selected_place_source = ""
        self.task_start_ns = 0
        self.grasp_accept_after_ns = 0
        self.navigation_result: str | None = None
        self.navigation_goal_handle = None
        self.navigation_goal_kind = ""
        self.navigation_cancel_requested = False
        self.hybrid_kind = ""
        self.hybrid_weight = 0.0
        self.retreat_phase_start_ns = 0
        self.retreat_start_xy: tuple[float, float] | None = None
        self.rear_clearance_m: float | None = None
        self.rear_scan_received_ns = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(String, self.task_topic, self.on_task_command, 10)
        for topic in self.grasp_topics:
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, source=topic: self.on_grasp_best(msg, source),
                10,
            )
        for topic in self._unique_topics([self.detections_topic, self.place_detections_topic]):
            self.create_subscription(
                String,
                topic,
                lambda msg, source=topic: self.on_detections(msg, source),
                10,
            )
        self.create_subscription(String, self.arm_task_state_topic, self.on_arm_task_state, 10)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("rear_scan_topic").value),
            self.on_rear_scan,
            10,
        )
        self.target_objects_pub = self.create_publisher(String, self.target_objects_topic, 10)
        self.arm_task_command_pub = self.create_publisher(String, self.arm_task_command_topic, 10)
        self.task_state_pub = self.create_publisher(String, self.task_state_topic, 10)
        self.base_control_mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("base_control_mode_topic").value),
            10,
        )
        self.hybrid_blend_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("hybrid_blend_topic").value),
            10,
        )
        self.retreat_cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("retreat_cmd_topic").value),
            10,
        )
        self.navigation_client = (
            ActionClient(self, NavigateToPose, self.navigation_action_name)
            if self.enable_navigation
            else None
        )
        self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.create_timer(1.0, self.on_log_timer)

        if self.autostart and self.pick_object and self.place_object:
            self.start_task(self.pick_object, self.place_object)

        self.get_logger().info(
            f"Pick/place manager target={self.target_frame}, parent={self.parent_frame}, "
            f"task_topic={self.task_topic}, detections={self._unique_topics([self.detections_topic, self.place_detections_topic])}, "
            f"grasp_topics={self.grasp_topics}"
        )

    def _load_topic_list(self, parameter_name: str, defaults: list[str]) -> list[str]:
        values = [topic for topic in defaults if topic]
        parameter_value = self.get_parameter(parameter_name).value
        if isinstance(parameter_value, (list, tuple)):
            values.extend(str(item).strip() for item in parameter_value if str(item).strip())
        elif parameter_value:
            values.extend(token.strip() for token in str(parameter_value).split(",") if token.strip())
        return self._unique_topics(values)

    @staticmethod
    def _unique_topics(topics: list[str]) -> list[str]:
        unique = []
        for topic in topics:
            normalized = str(topic).strip()
            if normalized and normalized not in unique:
                unique.append(normalized)
        return unique

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
        self.current_perception_object = self.build_perception_label(pick_object, place_object)
        now_ns = self.get_clock().now().nanoseconds
        self.task_start_ns = now_ns
        self.grasp_accept_after_ns = now_ns + int(self.fresh_grasp_delay_sec * 1e9)
        self.last_target_objects_publish_ns = 0
        self.pick_transform = None
        self.place_transform = None
        self.current_transform = None
        self.pick_orientation = None
        self.pick_orientation_frame = ""
        self.pick_orientation_by_source = {}
        self.selected_pick_source = ""
        self.selected_place_source = ""
        self.navigation_result = None
        self.navigation_goal_kind = ""
        self.navigation_cancel_requested = False
        self.hybrid_kind = ""
        self.hybrid_weight = 0.0
        self.retreat_phase_start_ns = 0
        self.retreat_start_xy = None
        self.rear_clearance_m = None
        self.rear_scan_received_ns = 0
        self.publish_target_object(force=True)
        if self.enable_navigation:
            self.publish_arm_command("RESET")
            self.publish_base_mode("STOP")
        else:
            self.publish_arm_command("PICK")
            self.publish_base_mode("MANIPULATION")
        self.publish_task_state()
        self.get_logger().warn(f"Pick/place task started: pick={pick_object}, place={place_object}")

    def on_grasp_best(self, msg: PoseStamped, source_topic: str = "") -> None:
        if self.phase != "FIND_PICK":
            return
        now_ns = self.get_clock().now().nanoseconds
        if not self.is_fresh_grasp_message(msg, now_ns):
            self.last_debug = (
                f"ignored stale/early grasp from {source_topic or '<unknown>'}; "
                f"waiting for fresh {self.pick_object} grasp"
            )
            return
        orientation = (
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        )
        orientation_frame = None
        cached_orientation = self.pick_orientation_for_grasp_source(source_topic)
        if cached_orientation is not None:
            orientation_frame, orientation = cached_orientation
        transform = self.pose_to_parent_transform(
            source_frame=msg.header.frame_id,
            position=(
                float(msg.pose.position.x) + self.pick_offset[0],
                float(msg.pose.position.y) + self.pick_offset[1],
                float(msg.pose.position.z) + self.pick_offset[2],
            ),
            orientation=orientation,
            orientation_frame=orientation_frame,
        )
        if transform is None:
            return
        self.pick_transform = transform
        self.current_transform = transform
        self.selected_pick_source = source_topic

    def is_fresh_grasp_message(self, msg: PoseStamped, now_ns: int) -> bool:
        if int(now_ns) < int(self.grasp_accept_after_ns):
            return False
        stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)
        if stamp_ns > 0 and self.task_start_ns > 0 and stamp_ns < self.task_start_ns:
            return False
        return True

    @staticmethod
    def stamp_to_nanoseconds(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def on_detections(self, msg: String, source_topic: str = "") -> None:
        if self.phase not in ("FIND_PICK", "PICK", "FIND_PLACE"):
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

        if self.phase in ("FIND_PICK", "PICK"):
            self.cache_pick_orientation(objects, source_topic)

        obj = self.select_detection(objects, self.place_object)
        if obj is None:
            if self.phase == "FIND_PLACE":
                self.last_debug = f"waiting for place object: {self.place_object}"
            return

        pos = obj.get("position_target_frame")
        if not isinstance(pos, dict):
            if self.phase == "FIND_PLACE":
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
        self.selected_place_source = source_topic
        if self.phase == "FIND_PLACE":
            self.current_transform = transform
        else:
            self.last_debug = (
                f"cached place target: {self.place_object} via {source_topic or '<unknown>'}"
            )

    def select_detection(self, objects: list[Any], target_name: str) -> dict[str, Any] | None:
        target = target_name.strip().lower()
        target_color, target_class = split_semantic_target(target)
        target_aliases = class_aliases_for(target)
        matches: list[dict[str, Any]] = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            confidence = float(obj.get("confidence", 0.0))
            if confidence < self.min_place_confidence:
                continue
            name = str(obj.get("object_name", "")).strip().lower()
            semantic_class = str(obj.get("semantic_class", "")).strip().lower()
            class_match = (
                not target
                or target in name
                or name in target
                or semantic_class == target_class
                or any(alias and alias in name for alias in target_aliases)
            )
            if not class_match:
                continue
            if target_color:
                observed_color = str(obj.get("observed_color", "")).strip().lower()
                color_scores = obj.get("color_scores", {})
                color_score = 0.0
                if isinstance(color_scores, dict):
                    color_score = float(color_scores.get(target_color, 0.0))
                if observed_color != target_color and color_score < 0.08:
                    continue
            matches.append(obj)
        if not matches:
            return None
        matches.sort(key=lambda item: float(item.get("confidence", 0.0)), reverse=True)
        return matches[0]

    def cache_pick_orientation(self, objects: list[Any], source_topic: str = "") -> None:
        obj = self.select_detection(objects, self.pick_object)
        if obj is None:
            return
        orientation = self.detection_orientation(obj)
        if orientation is None:
            return
        frame_id, quaternion = orientation
        source = str(source_topic or "").strip()
        if source:
            self.pick_orientation_by_source[source] = (frame_id, quaternion)
        self.pick_orientation_frame = frame_id
        self.pick_orientation = quaternion
        self.last_debug = (
            f"cached pick orientation: {self.pick_object} via {source_topic or '<unknown>'}"
        )

    def pick_orientation_for_grasp_source(
        self,
        source_topic: str = "",
    ) -> tuple[str, tuple[float, float, float, float]] | None:
        detection_topic = self.detection_topic_for_grasp_source(source_topic)
        if detection_topic in self.pick_orientation_by_source:
            return self.pick_orientation_by_source[detection_topic]
        if str(source_topic or "").strip():
            return None
        if self.pick_orientation is not None and self.pick_orientation_frame:
            return self.pick_orientation_frame, self.pick_orientation
        return None

    def detection_topic_for_grasp_source(self, source_topic: str = "") -> str:
        source = str(source_topic or "").strip()
        if source == "/sm_grasping_place/grasp_best":
            return self.place_detections_topic
        return self.detections_topic

    @staticmethod
    def detection_orientation(
        obj: dict[str, Any],
    ) -> tuple[str, tuple[float, float, float, float]] | None:
        for key in ("orientation_target_frame", "orientation_camera_frame"):
            orientation = obj.get(key)
            if not isinstance(orientation, dict):
                continue
            frame_id = str(orientation.get("frame_id", "")).strip()
            if not frame_id:
                continue
            try:
                quaternion = normalize_quaternion_xyzw(
                    (
                        float(orientation["x"]),
                        float(orientation["y"]),
                        float(orientation["z"]),
                        float(orientation["w"]),
                    )
                )
            except (KeyError, TypeError, ValueError):
                continue
            return frame_id, quaternion
        return None

    def pose_to_parent_transform(
        self,
        source_frame: str,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
        orientation_frame: str | None = None,
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

        orientation_source_frame = str(orientation_frame or source_frame).strip()
        if not orientation_source_frame:
            orientation_source_frame = source_frame
        if orientation_source_frame != self.parent_frame:
            try:
                parent_from_orientation = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    orientation_source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_timeout_sec),
                )
            except TransformException as exc:
                self.last_debug = (
                    f"waiting for orientation transform {self.parent_frame} <- "
                    f"{orientation_source_frame}: {exc}"
                )
                return None

            rotation_msg = parent_from_orientation.transform.rotation
            rotation = (
                float(rotation_msg.x),
                float(rotation_msg.y),
                float(rotation_msg.z),
                float(rotation_msg.w),
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

    def on_rear_scan(self, msg: LaserScan) -> None:
        clearance = rear_sector_clearance(
            ranges=list(msg.ranges),
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            half_angle=self.retreat_sector_half_angle_rad,
            ignore_below=self.retreat_scan_ignore_below_m,
        )
        self.rear_clearance_m = float(msg.range_max) if clearance is None else clearance
        self.rear_scan_received_ns = self.get_clock().now().nanoseconds

    def on_timer(self) -> None:
        self.publish_target_object(force=False)
        self.advance_phase()
        if self.current_transform is not None:
            self.current_transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.current_transform)
        self.publish_task_state()

    def request_navigation(self, kind: str, target: TransformStamped) -> str:
        if self.navigation_client is None:
            return "not_needed"
        try:
            parent_from_base = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.navigation_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.last_debug = f"waiting for navigation base TF: {exc}"
            return "waiting"

        base = parent_from_base.transform.translation
        obj = target.transform.translation
        standoff = self.pick_standoff_m if kind == "pick" else self.place_standoff_m
        planar_goal = compute_standoff_pose(
            (float(base.x), float(base.y)),
            (float(obj.x), float(obj.y)),
            standoff,
        )
        skip_navigation, skip_reason = should_skip_navigation(
            kind=kind,
            base_xy=(float(base.x), float(base.y)),
            object_xy=(float(obj.x), float(obj.y)),
            goal_xy=(planar_goal.x, planar_goal.y),
            goal_skip_distance_m=self.navigation_goal_skip_distance_m,
            place_direct_approach_distance_m=self.place_direct_approach_distance_m,
        )
        if skip_navigation:
            self.last_debug = f"skipping Nav2 {kind}: {skip_reason}"
            return "not_needed"
        if not self.navigation_client.server_is_ready():
            self.last_debug = f"waiting for Nav2 action server: {self.navigation_action_name}"
            return "waiting"

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.parent_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = planar_goal.x
        goal.pose.pose.position.y = planar_goal.y
        goal.pose.pose.orientation.z = sin(planar_goal.yaw * 0.5)
        goal.pose.pose.orientation.w = cos(planar_goal.yaw * 0.5)
        self.current_transform = None
        self.navigation_result = None
        self.navigation_goal_kind = kind
        self.navigation_goal_handle = None
        self.navigation_cancel_requested = False
        self.hybrid_kind = ""
        self.publish_hybrid_weight(0.0)
        self.publish_base_mode("NAVIGATION")
        future = self.navigation_client.send_goal_async(goal)
        future.add_done_callback(self.on_navigation_goal_response)
        self.last_debug = (
            f"sent Nav2 {kind} goal=({planar_goal.x:.3f}, {planar_goal.y:.3f}, "
            f"yaw={planar_goal.yaw:.3f})"
        )
        return "started"

    def place_direct_approach_status(self) -> tuple[bool, str]:
        """Return whether the cached place target is already reachable directly."""
        if self.place_transform is None or self.place_direct_approach_distance_m <= 0.0:
            return False, "place target is not cached"
        try:
            parent_from_base = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.navigation_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            return False, f"place direct-approach TF unavailable: {exc}"

        base = parent_from_base.transform.translation
        obj = self.place_transform.transform.translation
        distance = hypot(float(obj.x) - float(base.x), float(obj.y) - float(base.y))
        if distance <= self.place_direct_approach_distance_m:
            return True, (
                f"place target {distance:.3f}m away is inside direct approach range "
                f"{self.place_direct_approach_distance_m:.3f}m"
            )
        return False, (
            f"place target {distance:.3f}m away exceeds direct approach range "
            f"{self.place_direct_approach_distance_m:.3f}m"
        )

    def target_planar_distance(self, target: TransformStamped) -> float | None:
        try:
            parent_from_base = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.navigation_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.last_debug = f"waiting for hybrid base TF: {exc}"
            return None
        base = parent_from_base.transform.translation
        obj = target.transform.translation
        return hypot(float(obj.x) - float(base.x), float(obj.y) - float(base.y))

    def maybe_start_hybrid_handoff(self, kind: str, target: TransformStamped) -> bool:
        if not self.enable_hybrid_handoff:
            return False
        distance = self.target_planar_distance(target)
        if distance is None or distance > self.hybrid_outer_distance_m:
            return False
        self.hybrid_kind = kind
        self.current_transform = target
        self.publish_arm_command("PICK" if kind == "pick" else "PLACE")
        self.publish_base_mode("HYBRID")
        self.phase = "HYBRID_PICK" if kind == "pick" else "HYBRID_PLACE"
        self.update_hybrid_handoff(target)
        self.get_logger().warn(
            f"Task phase: NAVIGATE_{kind.upper()} -> {self.phase}; "
            f"hybrid handoff started at {distance:.3f}m"
        )
        return True

    def update_hybrid_handoff(self, target: TransformStamped) -> None:
        distance = self.target_planar_distance(target)
        if distance is None:
            return
        weight = hybrid_blend_weight(
            distance,
            self.hybrid_outer_distance_m,
            self.hybrid_inner_distance_m,
        )
        self.publish_hybrid_weight(weight)
        self.last_debug = (
            f"hybrid {self.hybrid_kind}: distance={distance:.3f}m, "
            f"precision_weight={weight:.3f}"
        )
        if distance <= self.hybrid_inner_distance_m:
            self.finish_hybrid_handoff()

    def finish_hybrid_handoff(self) -> None:
        self.navigation_cancel_requested = True
        if self.navigation_goal_handle is not None:
            self.navigation_goal_handle.cancel_goal_async()
        self.publish_hybrid_weight(1.0)
        self.publish_base_mode("MANIPULATION")
        self.phase = "PICK" if self.hybrid_kind == "pick" else "PLACE"
        self.navigation_result = "HANDOFF"
        self.get_logger().warn(
            f"Hybrid handoff complete: precision control owns {self.hybrid_kind} approach"
        )

    def publish_hybrid_weight(self, weight: float) -> None:
        self.hybrid_weight = max(0.0, min(1.0, float(weight)))
        self.hybrid_blend_pub.publish(Float32(data=self.hybrid_weight))

    def on_navigation_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.navigation_result = "FAILED"
            self.last_debug = f"Nav2 goal request failed: {exc}"
            return
        if not goal_handle.accepted:
            self.navigation_result = "FAILED"
            self.last_debug = "Nav2 goal rejected"
            return
        self.navigation_goal_handle = goal_handle
        if self.navigation_cancel_requested:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_navigation_result)

    def on_navigation_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:
            self.navigation_result = "FAILED"
            self.last_debug = f"Nav2 result failed: {exc}"
            return
        if self.navigation_cancel_requested:
            self.navigation_result = "HANDOFF"
        else:
            self.navigation_result = (
                "SUCCEEDED" if status == GoalStatus.STATUS_SUCCEEDED else "FAILED"
            )
        self.last_debug = f"Nav2 {self.navigation_goal_kind} result={self.navigation_result}"

    def advance_phase(self) -> None:
        if self.phase == "IDLE":
            return

        if self.phase == "FIND_PICK":
            if self.pick_transform is None:
                self.last_debug = f"waiting for pick grasp: {self.pick_object}"
                return
            navigation = self.request_navigation("pick", self.pick_transform)
            if navigation == "waiting":
                return
            if navigation == "started":
                self.phase = "NAVIGATE_PICK"
                self.get_logger().warn("Task phase: FIND_PICK -> NAVIGATE_PICK")
                return
            self.begin_pick_manipulation()
            self.get_logger().warn(
                f"Task phase: FIND_PICK -> PICK via {self.selected_pick_source or '<unknown>'}"
            )

        elif self.phase == "NAVIGATE_PICK":
            if self.maybe_start_hybrid_handoff("pick", self.pick_transform):
                return
            if self.navigation_result is None:
                return
            if self.navigation_result != "SUCCEEDED":
                self.phase = "ERROR"
                self.publish_base_mode("STOP")
                return
            self.begin_pick_manipulation()
            self.get_logger().warn("Task phase: NAVIGATE_PICK -> PICK")

        elif self.phase == "HYBRID_PICK":
            self.update_hybrid_handoff(self.pick_transform)

        elif self.phase == "PICK":
            if self.arm_task_state == "PICK:HOLD":
                self.current_perception_object = self.build_perception_label(
                    self.pick_object,
                    self.place_object,
                )
                self.last_target_objects_publish_ns = 0
                self.publish_target_object(force=True)
                if self.enable_navigation:
                    self.current_transform = None
                    direct_place, direct_reason = self.place_direct_approach_status()
                    if direct_place:
                        self.last_debug = f"skipping safe retreat: {direct_reason}"
                        self.get_logger().warn(
                            "Task phase: PICK -> FIND_PLACE; "
                            f"skipping SAFE_RETREAT: {direct_reason}"
                        )
                    else:
                        self.publish_arm_command("TRANSPORT")
                        self.publish_base_mode("STOP")
                        if self.retreat_distance_m > 0.0:
                            self.start_safe_retreat()
                            self.get_logger().warn(
                                f"Task phase: PICK -> SAFE_RETREAT; {direct_reason}"
                            )
                            return
                self.phase = "FIND_PLACE"
                self.get_logger().warn("Task phase: PICK -> FIND_PLACE")

        elif self.phase == "SAFE_RETREAT":
            self.advance_safe_retreat()

        elif self.phase == "FIND_PLACE":
            if self.place_transform is None:
                self.last_debug = f"holding object; waiting for place target: {self.place_object}"
                return
            navigation = self.request_navigation("place", self.place_transform)
            if navigation == "waiting":
                return
            if navigation == "started":
                self.phase = "NAVIGATE_PLACE"
                self.get_logger().warn("Task phase: FIND_PLACE -> NAVIGATE_PLACE")
                return
            self.begin_place_manipulation()
            self.get_logger().warn(
                f"Task phase: FIND_PLACE -> PLACE via {self.selected_place_source or '<unknown>'}"
            )

        elif self.phase == "NAVIGATE_PLACE":
            if self.maybe_start_hybrid_handoff("place", self.place_transform):
                return
            if self.navigation_result is None:
                return
            if self.navigation_result != "SUCCEEDED":
                self.phase = "ERROR"
                self.publish_base_mode("STOP")
                return
            self.begin_place_manipulation()
            self.get_logger().warn("Task phase: NAVIGATE_PLACE -> PLACE")

        elif self.phase == "HYBRID_PLACE":
            self.update_hybrid_handoff(self.place_transform)

        elif self.phase == "PLACE":
            if self.arm_task_state == "PLACE:HOLD":
                self.phase = "DONE"
                self.get_logger().warn("Task phase: PLACE -> DONE")

        elif self.phase == "DONE":
            self.last_debug = "pick/place task done"

        elif self.phase == "ERROR":
            self.last_debug = f"pick/place task stopped: navigation {self.navigation_result}"

    def begin_pick_manipulation(self) -> None:
        self.current_transform = self.pick_transform
        self.publish_base_mode("MANIPULATION")
        self.publish_arm_command("PICK")
        self.phase = "PICK"

    def begin_place_manipulation(self) -> None:
        self.current_transform = self.place_transform
        self.publish_base_mode("MANIPULATION")
        self.publish_arm_command("PLACE")
        self.phase = "PLACE"

    def start_safe_retreat(self) -> None:
        self.phase = "SAFE_RETREAT"
        self.retreat_phase_start_ns = self.get_clock().now().nanoseconds
        self.retreat_start_xy = None
        self.publish_retreat_command(0.0)
        self.publish_base_mode("STOP")

    def advance_safe_retreat(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed_ns = now_ns - self.retreat_phase_start_ns
        if elapsed_ns < self.transport_settle_ns:
            self.publish_retreat_command(0.0)
            self.last_debug = "waiting for transport pose before retreat"
            return
        if elapsed_ns > self.retreat_timeout_ns:
            self.fail_safe_retreat("retreat timed out")
            return
        if (
            self.rear_scan_received_ns <= 0
            or now_ns - self.rear_scan_received_ns > self.retreat_scan_timeout_ns
        ):
            self.publish_retreat_command(0.0)
            self.publish_base_mode("STOP")
            self.last_debug = "waiting for fresh rear scan before retreat"
            return
        if self.rear_clearance_m is None or self.rear_clearance_m <= self.retreat_min_clearance_m:
            clearance = "unknown" if self.rear_clearance_m is None else f"{self.rear_clearance_m:.2f}m"
            self.fail_safe_retreat(f"rear path blocked at {clearance}")
            return

        base_xy = self.navigation_base_xy()
        if base_xy is None:
            self.publish_retreat_command(0.0)
            return
        if self.retreat_start_xy is None:
            self.retreat_start_xy = base_xy
        traveled = hypot(
            base_xy[0] - self.retreat_start_xy[0],
            base_xy[1] - self.retreat_start_xy[1],
        )
        if traveled >= self.retreat_distance_m:
            self.publish_retreat_command(0.0)
            self.publish_base_mode("STOP")
            self.phase = "FIND_PLACE"
            self.last_debug = f"safe retreat complete: {traveled:.2f}m"
            self.get_logger().warn("Task phase: SAFE_RETREAT -> FIND_PLACE")
            return

        self.publish_base_mode("RETREAT")
        self.publish_retreat_command(-self.retreat_speed_mps)
        self.last_debug = (
            f"safe retreat {traveled:.2f}/{self.retreat_distance_m:.2f}m, "
            f"rear clearance={self.rear_clearance_m:.2f}m"
        )

    def navigation_base_xy(self) -> tuple[float, float] | None:
        try:
            parent_from_base = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.navigation_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self.last_debug = f"waiting for navigation base TF: {exc}"
            return None
        translation = parent_from_base.transform.translation
        return float(translation.x), float(translation.y)

    def fail_safe_retreat(self, reason: str) -> None:
        self.publish_retreat_command(0.0)
        self.publish_base_mode("STOP")
        self.phase = "ERROR"
        self.navigation_result = "RETREAT_FAILED"
        self.last_debug = reason
        self.get_logger().error(f"Safe retreat failed: {reason}")

    def publish_retreat_command(self, linear_x: float) -> None:
        command = Twist()
        command.linear.x = float(linear_x)
        self.retreat_cmd_pub.publish(command)

    def publish_target_object(self, force: bool) -> None:
        payload = self.build_perception_payload(self.pick_object, self.place_object)
        if not payload:
            return
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (now_ns - self.last_target_objects_publish_ns) * 1e-9
        if not force and elapsed < self.target_objects_publish_period:
            return
        self.target_objects_pub.publish(String(data=payload))
        self.last_target_objects_publish_ns = now_ns

    @staticmethod
    def build_perception_label(pick_object: str, place_object: str) -> str:
        return ", ".join(PickPlaceTaskManager.unique_nonempty([pick_object, place_object]))

    @staticmethod
    def build_perception_payload(pick_object: str, place_object: str) -> str:
        targets = PickPlaceTaskManager.unique_nonempty([pick_object, place_object])
        roi_targets = PickPlaceTaskManager.unique_nonempty([pick_object])
        if not targets:
            return ""
        return json.dumps(
            {
                "target_objects": targets,
                "roi_target_objects": roi_targets,
            },
            ensure_ascii=False,
        )

    @staticmethod
    def unique_nonempty(values: list[str]) -> list[str]:
        unique: list[str] = []
        for value in values:
            normalized = str(value).strip()
            if normalized and normalized not in unique:
                unique.append(normalized)
        return unique

    def publish_arm_command(self, command: str) -> None:
        self.arm_task_command_pub.publish(String(data=command))

    def publish_base_mode(self, mode: str) -> None:
        self.base_control_mode_pub.publish(String(data=str(mode).strip().upper()))

    def publish_task_state(self) -> None:
        self.task_state_pub.publish(
            String(
                data=(
                    f"{self.phase}: pick={self.pick_object}, place={self.place_object}, "
                    f"arm={self.arm_task_state or '<none>'}"
                    f", navigation={self.navigation_result or '<none>'}"
                    f", blend={self.hybrid_weight:.3f}"
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
            f"arm={self.arm_task_state or '<none>'}, {target}, "
            f"pick_source={self.selected_pick_source or '<none>'}, "
            f"place_source={self.selected_place_source or '<none>'}, {self.last_debug}"
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
