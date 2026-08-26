from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class SemanticMissionTask:
    pick_object_id: str
    place_object_id: str
    pick_label: str
    place_label: str
    pick_zone: str
    place_zone: str
    resolved_task: dict[str, Any]

    @property
    def same_workspace(self) -> bool:
        return bool(self.pick_zone) and self.pick_zone == self.place_zone

    def existing_task_payload(self) -> str:
        return json.dumps(
            {"pick": self.pick_label, "place": self.place_label},
            ensure_ascii=False,
        )

    def navigation_request_payload(self, role: str) -> str:
        normalized_role = str(role).strip().lower()
        if normalized_role not in {"pick", "place"}:
            raise ValueError("navigation role must be pick or place")
        object_id = (
            self.pick_object_id if normalized_role == "pick" else self.place_object_id
        )
        return json.dumps(
            {
                "role": normalized_role,
                "object_id": object_id,
                "resolved_task": self.resolved_task,
            },
            ensure_ascii=False,
        )


def _mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value


def parse_resolved_task(text: str) -> SemanticMissionTask:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("resolved task is not valid JSON") from exc
    payload = _mapping(payload, "payload")
    if payload.get("ok") is not True:
        raise ValueError("semantic lookup was not successful")

    task_input = _mapping(payload.get("input"), "input")
    pick = _mapping(payload.get("pick"), "pick")
    place = _mapping(payload.get("place"), "place")
    pick_id = str(pick.get("object_id", "")).strip()
    place_id = str(place.get("object_id", "")).strip()
    pick_label = str(
        pick.get("canonical_name") or task_input.get("pick") or pick_id
    ).strip()
    place_label = str(
        place.get("canonical_name") or task_input.get("place") or place_id
    ).strip()
    if not pick_id or not place_id or not pick_label or not place_label:
        raise ValueError("resolved task is missing object identifiers or labels")

    return SemanticMissionTask(
        pick_object_id=pick_id,
        place_object_id=place_id,
        pick_label=pick_label,
        place_label=place_label,
        pick_zone=str(pick.get("zone", "")).strip(),
        place_zone=str(place.get("zone", "")).strip(),
        resolved_task=payload,
    )


def task_phase(text: str) -> str:
    return str(text).partition(":")[0].strip().upper()


def navigation_status_for_task(
    text: str,
    task: SemanticMissionTask,
    role: str,
) -> str:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError):
        return ""
    if not isinstance(payload, dict):
        return ""
    normalized_role = str(role).strip().lower()
    object_id = task.pick_object_id if normalized_role == "pick" else task.place_object_id
    if (
        str(payload.get("role", "")).strip().lower() != normalized_role
        or str(payload.get("object_id", "")).strip() != object_id
    ):
        return ""
    return str(payload.get("state", "")).strip().lower()


class SemanticMissionOrchestrator(Node):
    """Coordinate two DB-driven Nav2 legs around the unchanged manipulation flow."""

    def __init__(self) -> None:
        super().__init__("semantic_mission_orchestrator")
        self.declare_parameter(
            "resolved_task_topic", "/semantic_lookup/resolved_task"
        )
        self.declare_parameter(
            "navigation_request_topic", "/semantic_navigation/request"
        )
        self.declare_parameter(
            "navigation_status_topic", "/semantic_navigation/status"
        )
        self.declare_parameter("existing_task_topic", "/pick_place_task")
        self.declare_parameter("task_state_topic", "/pick_place_task_state")
        self.declare_parameter(
            "phase_command_topic", "/pick_place_phase_command"
        )
        self.declare_parameter("status_topic", "/semantic_mvp/status")

        gp = self.get_parameter
        self.navigation_request_pub = self.create_publisher(
            String, str(gp("navigation_request_topic").value), 10
        )
        self.task_pub = self.create_publisher(
            String, str(gp("existing_task_topic").value), 10
        )
        self.phase_command_pub = self.create_publisher(
            String, str(gp("phase_command_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(gp("status_topic").value), 10
        )
        self.create_subscription(
            String,
            str(gp("resolved_task_topic").value),
            self._on_resolved_task,
            10,
        )
        self.create_subscription(
            String,
            str(gp("navigation_status_topic").value),
            self._on_navigation_status,
            10,
        )
        self.create_subscription(
            String,
            str(gp("task_state_topic").value),
            self._on_task_state,
            10,
        )

        self.task: SemanticMissionTask | None = None
        self.state = "IDLE"
        self.task_dispatched = False
        self.place_navigation_requested = False
        self.place_resumed = False
        self.last_task_phase = ""
        self.current_workspace = ""
        self.departure_retreat_seen = False
        self.task_started_seen = False
        self.get_logger().info(
            "Semantic mission orchestrator ready: DB Nav2 pick -> existing Pick -> "
            "DB Nav2 place -> existing Place"
        )

    def _publish_status(self, state: str, message: str = "") -> None:
        task = self.task
        payload: dict[str, object] = {"state": str(state)}
        if task is not None:
            payload.update(
                {
                    "pick_object_id": task.pick_object_id,
                    "place_object_id": task.place_object_id,
                    "pick_zone": task.pick_zone,
                    "place_zone": task.place_zone,
                }
            )
        if message:
            payload["message"] = str(message)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _on_resolved_task(self, message: String) -> None:
        if self.state not in {"IDLE", "DONE", "ERROR"}:
            self._publish_status("busy", f"mission is currently {self.state}")
            return
        try:
            task = parse_resolved_task(message.data)
        except ValueError as exc:
            self.task = None
            self.state = "ERROR"
            self._publish_status("rejected", str(exc))
            return

        previous_task_phase = self.last_task_phase
        previous_workspace = self.current_workspace
        self.task = task
        self.task_dispatched = False
        self.place_navigation_requested = False
        self.place_resumed = False
        self.departure_retreat_seen = False
        self.task_started_seen = False

        if previous_task_phase == "DONE" and previous_workspace:
            if previous_workspace == task.pick_zone:
                self._dispatch_pick_task(
                    f"pick workspace {task.pick_zone} is already active"
                )
                return
            self.state = "WAIT_DEPARTURE_RETREAT"
            self.phase_command_pub.publish(String(data="START_DEPARTURE_RETREAT"))
            self._publish_status(
                "departure_retreat",
                f"leaving {previous_workspace} before pick Nav2",
            )
            self.get_logger().warn(
                f"Consecutive mission is leaving {previous_workspace} before "
                f"DB Nav2 to {task.pick_zone}"
            )
            return
        self._request_navigation("pick")

    def _dispatch_pick_task(self, reason: str = "") -> None:
        task = self.task
        if task is None or self.task_dispatched:
            return
        self.task_pub.publish(String(data=task.existing_task_payload()))
        self.task_dispatched = True
        self.task_started_seen = False
        self.state = "PICK"
        self._publish_status("pick_dispatched", reason)
        self.get_logger().warn(
            "Dispatched the unchanged existing task: "
            f"{task.pick_label} -> {task.place_label}"
            + (f"; {reason}" if reason else "")
        )

    def _request_navigation(self, role: str) -> None:
        task = self.task
        if task is None:
            return
        self.current_workspace = ""
        self.navigation_request_pub.publish(
            String(data=task.navigation_request_payload(role))
        )
        self.state = "NAV_TO_PICK" if role == "pick" else "NAV_TO_PLACE"
        self._publish_status(self.state.lower())
        object_id = task.pick_object_id if role == "pick" else task.place_object_id
        self.get_logger().warn(
            f"Mission requested DB Nav2 {role} approach for {object_id}"
        )

    def _on_navigation_status(self, message: String) -> None:
        task = self.task
        if task is None:
            return
        if self.state == "NAV_TO_PICK":
            role = "pick"
        elif self.state == "NAV_TO_PLACE":
            role = "place"
        else:
            return
        status = navigation_status_for_task(message.data, task, role)
        if not status:
            return
        if status in {"failed", "rejected", "aborted"}:
            self.state = "ERROR"
            self._publish_status("error", f"{role} navigation {status}")
            return
        if status not in {"handoff_ready", "succeeded"}:
            return

        if role == "pick" and not self.task_dispatched:
            self.current_workspace = task.pick_zone
            self._dispatch_pick_task("pick DB approach complete")
        elif role == "place" and not self.place_resumed:
            self.current_workspace = task.place_zone
            self._resume_place("place DB approach complete")

    def _on_task_state(self, message: String) -> None:
        task = self.task
        if task is None:
            return
        if not self.task_dispatched and self.state != "WAIT_DEPARTURE_RETREAT":
            return
        phase = task_phase(message.data)
        phase_changed = phase != self.last_task_phase
        if phase_changed:
            self.last_task_phase = phase
            self.get_logger().info(f"Existing task manager phase: {phase}")

        if self.state == "WAIT_DEPARTURE_RETREAT":
            if phase == "SAFE_RETREAT":
                self.departure_retreat_seen = True
            elif phase == "DONE" and self.departure_retreat_seen:
                self._request_navigation("pick")
            elif phase == "ERROR":
                self.state = "ERROR"
                self._publish_status(
                    "error", "departure retreat before pick navigation failed"
                )
            return

        if phase not in {"", "IDLE", "DONE", "ERROR"}:
            self.task_started_seen = True
        if phase == "PREPARE_PLACE_NAVIGATION":
            self.state = "PREPARE_TRANSPORT"
            if phase_changed:
                self._publish_status("prepare_transport")
                if not task.same_workspace:
                    self.phase_command_pub.publish(
                        String(data="START_PLACE_RETREAT")
                    )
                    self.get_logger().warn(
                        "Cross-workspace task requested the existing safe retreat "
                        "before place Nav2"
                    )
            return
        if (
            phase == "WAIT_PLACE_NAVIGATION"
            and not self.place_navigation_requested
            and not self.place_resumed
        ):
            if task.same_workspace:
                self._resume_place("pick and place share one workspace")
            else:
                self.place_navigation_requested = True
                self._request_navigation("place")
            return
        if phase == "DONE" and self.task_started_seen and self.state != "DONE":
            self.current_workspace = task.place_zone
            self.state = "DONE"
            self._publish_status("done")
        elif phase == "ERROR" and self.task_started_seen and self.state != "ERROR":
            self.state = "ERROR"
            self._publish_status("error", "existing Pick & Place reported ERROR")

    def _resume_place(self, reason: str) -> None:
        self.phase_command_pub.publish(String(data="RESUME_PLACE"))
        self.place_navigation_requested = True
        self.place_resumed = True
        self.state = "PLACE"
        self._publish_status("place_dispatched", reason)
        self.get_logger().warn(f"Place approach complete; resumed existing Place: {reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMissionOrchestrator()
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
