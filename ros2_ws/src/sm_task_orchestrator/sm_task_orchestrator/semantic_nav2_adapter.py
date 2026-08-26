from __future__ import annotations

import json
import math
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class SemanticNavigationGoal:
    role: str
    object_id: str
    frame_id: str
    x: float
    y: float
    yaw: float

    @property
    def fingerprint(self) -> tuple[str, str, str, float, float, float]:
        return (
            self.role,
            self.object_id,
            self.frame_id,
            self.x,
            self.y,
            self.yaw,
        )


def parse_semantic_navigation_goal(
    text: str,
    role: str = "pick",
    expected_frame: str = "map",
) -> SemanticNavigationGoal:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("resolved task is not valid JSON") from exc
    if not isinstance(payload, dict) or payload.get("ok") is not True:
        raise ValueError("resolved task is not successful")

    normalized_role = str(role).strip().lower()
    target = payload.get(normalized_role)
    if not isinstance(target, dict):
        raise ValueError(f"resolved task has no {normalized_role} object")
    pose = target.get("approach_pose")
    if not isinstance(pose, dict):
        raise ValueError(f"{normalized_role} object has no approach_pose")

    object_id = str(target.get("object_id", "")).strip()
    frame_id = str(pose.get("frame_id", "")).strip()
    if not object_id:
        raise ValueError(f"{normalized_role} object_id is empty")
    if not frame_id:
        raise ValueError("approach_pose frame_id is empty")
    normalized_expected_frame = str(expected_frame).strip()
    if normalized_expected_frame and frame_id != normalized_expected_frame:
        raise ValueError(
            f"approach_pose frame is {frame_id}, expected {normalized_expected_frame}"
        )

    try:
        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose["yaw"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("approach_pose must contain numeric x, y, yaw") from exc
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("approach_pose contains a non-finite value")

    return SemanticNavigationGoal(
        role=normalized_role,
        object_id=object_id,
        frame_id=frame_id,
        x=x,
        y=y,
        yaw=yaw,
    )


def goal_to_pose(goal: SemanticNavigationGoal, stamp) -> PoseStamped:
    pose = PoseStamped()
    if stamp is not None:
        pose.header.stamp = stamp
    pose.header.frame_id = goal.frame_id
    pose.pose.position.x = goal.x
    pose.pose.position.y = goal.y
    pose.pose.orientation.z = math.sin(goal.yaw * 0.5)
    pose.pose.orientation.w = math.cos(goal.yaw * 0.5)
    return pose


def handoff_feedback_state(
    distance_remaining: float,
    handoff_distance_m: float,
    armed: bool,
) -> tuple[bool, bool]:
    """Return (armed, ready) for a non-zero Nav2 approach handoff."""
    distance = float(distance_remaining)
    threshold = float(handoff_distance_m)
    # Nav2 can transiently report exactly zero when a replanning action aborts,
    # so zero is not accepted as proof of arrival. A real zero-distance goal is
    # covered by the normal SUCCEEDED fallback.
    if threshold <= 0.0 or not math.isfinite(distance) or distance <= 0.0:
        return bool(armed), False
    if distance > threshold:
        return True, False
    return bool(armed), bool(armed)


def parse_navigation_request(
    text: str,
    fallback_resolved_task: str = "",
) -> tuple[str, str, str]:
    """Return role, resolved-task JSON, and optional expected object ID."""
    raw = str(text).strip()
    role = raw.lower()
    expected_object_id = ""
    resolved_task = str(fallback_resolved_task).strip()
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        payload = None
    if isinstance(payload, dict):
        role = str(payload.get("role", "")).strip().lower()
        expected_object_id = str(payload.get("object_id", "")).strip()
        embedded_task = payload.get("resolved_task")
        if isinstance(embedded_task, dict):
            resolved_task = json.dumps(embedded_task, ensure_ascii=False)
        elif isinstance(embedded_task, str) and embedded_task.strip():
            resolved_task = embedded_task.strip()
    if role not in {"pick", "place"}:
        raise ValueError("navigation request role must be pick or place")
    if not resolved_task:
        raise ValueError("navigation request has no resolved task")
    return role, resolved_task, expected_object_id


class SemanticNav2Adapter(Node):
    def __init__(self) -> None:
        super().__init__("semantic_nav2_adapter")
        self.declare_parameter("resolved_task_topic", "/semantic_lookup/resolved_task")
        self.declare_parameter("request_topic", "/semantic_navigation/request")
        self.declare_parameter("goal_preview_topic", "/semantic_navigation/goal_pose")
        self.declare_parameter("status_topic", "/semantic_navigation/status")
        self.declare_parameter("navigation_action_name", "/navigate_to_pose")
        self.declare_parameter("base_mode_topic", "/base_control_mode")
        self.declare_parameter("target_role", "pick")
        self.declare_parameter("auto_start_on_resolved_task", True)
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("navigation_enabled", False)
        self.declare_parameter("prevent_duplicate_goal", True)
        self.declare_parameter("server_retry_period_sec", 0.25)
        self.declare_parameter("handoff_distance_m", 0.0)
        self.declare_parameter("navigation_base_frame", "chassis_link")
        self.declare_parameter("handoff_check_period_sec", 0.05)
        self.declare_parameter("handoff_transform_timeout_sec", 0.02)

        self.target_role = str(self.get_parameter("target_role").value).strip().lower()
        self.auto_start_on_resolved_task = bool(
            self.get_parameter("auto_start_on_resolved_task").value
        )
        self.expected_frame = str(self.get_parameter("expected_frame").value).strip()
        self.navigation_enabled = bool(
            self.get_parameter("navigation_enabled").value
        )
        self.prevent_duplicate_goal = bool(
            self.get_parameter("prevent_duplicate_goal").value
        )
        self.handoff_distance_m = max(
            0.0,
            float(self.get_parameter("handoff_distance_m").value),
        )
        self.navigation_base_frame = str(
            self.get_parameter("navigation_base_frame").value
        ).strip()
        self.handoff_transform_timeout_sec = max(
            0.0,
            float(self.get_parameter("handoff_transform_timeout_sec").value),
        )
        self.preview_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("goal_preview_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.base_mode_pub = self.create_publisher(
            String,
            str(self.get_parameter("base_mode_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("resolved_task_topic").value),
            self._on_resolved_task,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("request_topic").value),
            self._on_navigation_request,
            10,
        )
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigation_action_name").value),
        )
        retry_period = max(
            0.05,
            float(self.get_parameter("server_retry_period_sec").value),
        )
        self.create_timer(retry_period, self._try_send_pending_goal)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        handoff_check_period = max(
            0.02,
            float(self.get_parameter("handoff_check_period_sec").value),
        )
        self.create_timer(handoff_check_period, self._check_handoff_distance)

        self.pending_goal: SemanticNavigationGoal | None = None
        self.active_goal: SemanticNavigationGoal | None = None
        self.goal_handle = None
        self.last_fingerprint = None
        self.reported_waiting = False
        self.handoff_armed = False
        self.handoff_requested = False
        self.latest_resolved_task = ""
        mode = "enabled" if self.navigation_enabled else "dry-run"
        self.get_logger().info(
            f"Semantic Nav2 adapter ready: role={self.target_role}, mode={mode}, "
            f"auto_start={self.auto_start_on_resolved_task}, "
            f"handoff_distance={self.handoff_distance_m:.2f}m"
        )

    def _publish_status(
        self,
        state: str,
        goal: SemanticNavigationGoal | None = None,
        message: str = "",
    ) -> None:
        payload: dict[str, object] = {
            "state": str(state),
            "navigation_enabled": self.navigation_enabled,
        }
        if goal is not None:
            payload.update(
                {
                    "role": goal.role,
                    "object_id": goal.object_id,
                    "goal": {
                        "frame_id": goal.frame_id,
                        "x": goal.x,
                        "y": goal.y,
                        "yaw": goal.yaw,
                    },
                }
            )
        if message:
            payload["message"] = str(message)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_base_mode(self, mode: str) -> None:
        self.base_mode_pub.publish(String(data=str(mode).strip().upper()))

    def _on_resolved_task(self, message: String) -> None:
        self.latest_resolved_task = message.data
        if not self.auto_start_on_resolved_task:
            self._publish_status("task_cached")
            return
        try:
            goal = parse_semantic_navigation_goal(
                message.data,
                role=self.target_role,
                expected_frame=self.expected_frame,
            )
        except ValueError as exc:
            self._publish_status("rejected", message=str(exc))
            self.get_logger().error(f"Rejected semantic navigation task: {exc}")
            return

        self._queue_goal(goal)

    def _on_navigation_request(self, message: String) -> None:
        try:
            role, resolved_task, expected_object_id = parse_navigation_request(
                message.data,
                fallback_resolved_task=self.latest_resolved_task,
            )
            goal = parse_semantic_navigation_goal(
                resolved_task,
                role=role,
                expected_frame=self.expected_frame,
            )
        except ValueError as exc:
            self._publish_status("rejected", message=str(exc))
            self.get_logger().error(f"Rejected semantic navigation request: {exc}")
            return
        if expected_object_id and expected_object_id != goal.object_id:
            message_text = (
                f"navigation request expected {expected_object_id}, "
                f"resolved {goal.object_id}"
            )
            self._publish_status("rejected", goal, message_text)
            self.get_logger().error(message_text)
            return

        self._queue_goal(goal)

    def _queue_goal(self, goal: SemanticNavigationGoal) -> None:
        if self.active_goal is not None or self.pending_goal is not None:
            self._publish_status("busy", goal, "another navigation goal is active")
            return
        if self.prevent_duplicate_goal and goal.fingerprint == self.last_fingerprint:
            self._publish_status("duplicate_ignored", goal)
            return

        pose = goal_to_pose(goal, self.get_clock().now().to_msg())
        self.preview_pub.publish(pose)
        self._publish_status("previewed", goal)
        if not self.navigation_enabled:
            self.last_fingerprint = goal.fingerprint
            self._publish_status("dry_run", goal, "Nav2 goal was not sent")
            self.get_logger().warn(
                f"Dry-run {goal.role} goal for {goal.object_id}: "
                f"({goal.x:.3f}, {goal.y:.3f}, yaw={goal.yaw:.3f}) {goal.frame_id}"
            )
            return

        self.pending_goal = goal
        self.reported_waiting = False
        self._try_send_pending_goal()

    def _try_send_pending_goal(self) -> None:
        goal = self.pending_goal
        if goal is None or self.active_goal is not None:
            return
        if not self.navigation_client.server_is_ready():
            if not self.reported_waiting:
                self._publish_status("waiting_for_nav2", goal)
                self.reported_waiting = True
            return

        self.pending_goal = None
        self.active_goal = goal
        self.last_fingerprint = goal.fingerprint
        self.handoff_armed = False
        self.handoff_requested = False
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_to_pose(goal, self.get_clock().now().to_msg())
        self._publish_status("goal_sent", goal)
        future = self.navigation_client.send_goal_async(
            nav_goal,
            feedback_callback=self._on_navigation_feedback,
        )
        future.add_done_callback(self._on_goal_response)
        self.get_logger().warn(
            f"Sent Nav2 {goal.role} goal for {goal.object_id}: "
            f"({goal.x:.3f}, {goal.y:.3f}, yaw={goal.yaw:.3f}) {goal.frame_id}"
        )

    def _on_goal_response(self, future) -> None:
        goal = self.active_goal
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish("failed", f"Nav2 goal request failed: {exc}")
            return
        if not goal_handle.accepted:
            self._finish("rejected", "Nav2 rejected the goal")
            return
        self.goal_handle = goal_handle
        self._publish_base_mode("NAVIGATION")
        self._publish_status("goal_accepted", goal)
        goal_handle.get_result_async().add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback_message) -> None:
        goal = self.active_goal
        if (
            goal is None
            or self.goal_handle is None
            or self.handoff_requested
            or self.handoff_distance_m <= 0.0
        ):
            return
        try:
            distance_remaining = float(feedback_message.feedback.distance_remaining)
        except (AttributeError, TypeError, ValueError):
            return
        self._consider_handoff_distance(distance_remaining, "Nav2 feedback")

    def _check_handoff_distance(self) -> None:
        goal = self.active_goal
        if (
            goal is None
            or self.goal_handle is None
            or self.handoff_requested
            or self.handoff_distance_m <= 0.0
        ):
            return
        try:
            goal_from_base = self.tf_buffer.lookup_transform(
                goal.frame_id,
                self.navigation_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.handoff_transform_timeout_sec),
            )
        except TransformException:
            return
        translation = goal_from_base.transform.translation
        distance = math.hypot(
            goal.x - float(translation.x),
            goal.y - float(translation.y),
        )
        self._consider_handoff_distance(distance, "map/base TF")

    def _consider_handoff_distance(self, distance: float, source: str) -> None:
        goal = self.active_goal
        if goal is None or self.goal_handle is None or self.handoff_requested:
            return
        self.handoff_armed, ready = handoff_feedback_state(
            distance,
            self.handoff_distance_m,
            self.handoff_armed,
        )
        if not ready:
            return

        self.handoff_requested = True
        self._publish_status(
            "handoff_ready",
            goal,
            f"distance={distance:.3f}m, source={source}",
        )
        self.get_logger().warn(
            f"Semantic Nav2 handoff ready for {goal.object_id}: "
            f"distance={distance:.3f}m via {source}"
        )
        self.goal_handle.cancel_goal_async()

    def _on_navigation_result(self, future) -> None:
        try:
            status = int(future.result().status)
        except Exception as exc:
            self._finish("failed", f"Nav2 result failed: {exc}")
            return
        states = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_CANCELED: "canceled",
            GoalStatus.STATUS_ABORTED: "aborted",
        }
        self._finish(states.get(status, "failed"), f"Nav2 status={status}")

    def _finish(self, state: str, message: str = "") -> None:
        goal = self.active_goal
        handed_off = self.handoff_requested
        if not handed_off:
            self._publish_base_mode("STOP")
        self._publish_status(state, goal, message)
        if handed_off and goal is not None:
            self.get_logger().warn(
                f"Nav2 released {goal.object_id} without STOP; "
                "the existing manipulation pipeline owns base control"
            )
        elif state == "succeeded" and goal is not None:
            self.get_logger().warn(
                f"Nav2 reached semantic {goal.role} approach for {goal.object_id}"
            )
        elif state not in {"canceled"}:
            self.get_logger().error(f"Semantic navigation ended: {state}; {message}")
        # Duplicate suppression is only meant to prevent an already completed
        # task from being sent twice.  An aborted/canceled goal must remain
        # retryable after recovery actions such as clearing a costmap or
        # backing away from an obstacle.
        if state != "succeeded" and not handed_off:
            self.last_fingerprint = None
        self.goal_handle = None
        self.active_goal = None
        self.handoff_armed = False
        self.handoff_requested = False

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self._publish_base_mode("STOP")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticNav2Adapter()
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
