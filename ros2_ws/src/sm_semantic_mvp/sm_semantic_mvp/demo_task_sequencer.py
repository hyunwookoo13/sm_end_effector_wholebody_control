from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_MISSIONS = json.dumps(
    [
        {
            "command": "빨간 캔을 분홍 박스에 넣어줘",
            "pick": "red_can",
            "place": "pink_box",
        },
        {
            "command": "파란 캔을 노란 박스에 넣어줘",
            "pick": "blue_can",
            "place": "yellow_box",
        },
    ],
    ensure_ascii=False,
)


@dataclass(frozen=True)
class DemoMission:
    command: str
    pick: str
    place: str


def parse_mission_plan(text: str) -> list[DemoMission]:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("demo mission plan must be valid JSON") from exc
    if not isinstance(payload, list) or not payload:
        raise ValueError("demo mission plan must be a non-empty list")

    missions: list[DemoMission] = []
    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValueError(f"demo mission {index} must be an object")
        command = str(item.get("command", "")).strip()
        pick = str(item.get("pick", "")).strip()
        place = str(item.get("place", "")).strip()
        if not command or not pick or not place:
            raise ValueError(
                f"demo mission {index} requires command, pick, and place"
            )
        missions.append(DemoMission(command=command, pick=pick, place=place))
    return missions


def parse_mission_status(text: str) -> dict[str, str]:
    try:
        payload = json.loads(str(text))
    except (TypeError, json.JSONDecodeError):
        return {}
    if not isinstance(payload, dict):
        return {}
    return {
        "state": str(payload.get("state", "")).strip().lower(),
        "pick": str(payload.get("pick_object_id", "")).strip(),
        "place": str(payload.get("place_object_id", "")).strip(),
    }


def status_event_for_mission(
    text: str,
    mission: DemoMission,
    completion_armed: bool,
) -> tuple[str, bool]:
    status = parse_mission_status(text)
    if (
        status.get("pick") != mission.pick
        or status.get("place") != mission.place
    ):
        return "ignore", completion_armed

    state = status.get("state", "")
    if state == "done":
        return ("done", True) if completion_armed else ("ignore", False)
    if state in {"error", "rejected"}:
        return "error", completion_armed
    if state and state != "busy":
        return "active", True
    return "ignore", completion_armed


class DemoTaskSequencer(Node):
    """Publish a demonstration mission and advance only after its verified DONE."""

    def __init__(self) -> None:
        super().__init__("semantic_demo_task_sequencer")
        self.declare_parameter("task_topic", "/natural_language_task")
        self.declare_parameter("mission_status_topic", "/semantic_mvp/status")
        self.declare_parameter(
            "sequence_status_topic", "/semantic_demo_sequence/status"
        )
        self.declare_parameter("missions_json", DEFAULT_MISSIONS)
        self.declare_parameter("initial_delay_sec", 10.0)
        self.declare_parameter("inter_task_delay_sec", 3.0)

        self.missions = parse_mission_plan(
            str(self.get_parameter("missions_json").value)
        )
        self.initial_delay_ns = int(
            max(0.0, float(self.get_parameter("initial_delay_sec").value)) * 1e9
        )
        self.inter_task_delay_ns = int(
            max(0.0, float(self.get_parameter("inter_task_delay_sec").value))
            * 1e9
        )
        self.task_pub = self.create_publisher(
            String,
            str(self.get_parameter("task_topic").value),
            10,
        )
        self.sequence_status_pub = self.create_publisher(
            String,
            str(self.get_parameter("sequence_status_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_status_topic").value),
            self.on_mission_status,
            10,
        )

        self.mission_index = 0
        self.state = "WAIT_INITIAL_DELAY"
        self.completion_armed = False
        self.next_publish_ns = (
            self.get_clock().now().nanoseconds + self.initial_delay_ns
        )
        self.timer = self.create_timer(0.2, self.tick)
        self.publish_sequence_status("ready")
        self.get_logger().warn(
            f"Automatic demo sequence armed: missions={len(self.missions)}, "
            f"initial_delay={self.initial_delay_ns / 1e9:.1f}s"
        )

    @property
    def current_mission(self) -> DemoMission | None:
        if self.mission_index >= len(self.missions):
            return None
        return self.missions[self.mission_index]

    def publish_sequence_status(self, event: str, message: str = "") -> None:
        mission = self.current_mission
        payload: dict[str, Any] = {
            "event": str(event),
            "state": self.state,
            "mission_index": self.mission_index,
            "mission_count": len(self.missions),
        }
        if mission is not None:
            payload.update(
                {
                    "command": mission.command,
                    "pick": mission.pick,
                    "place": mission.place,
                }
            )
        if message:
            payload["message"] = str(message)
        self.sequence_status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def tick(self) -> None:
        if self.state not in {"WAIT_INITIAL_DELAY", "WAIT_INTER_TASK_DELAY"}:
            return
        if self.get_clock().now().nanoseconds < self.next_publish_ns:
            return
        if self.task_pub.get_subscription_count() < 1:
            return
        mission = self.current_mission
        if mission is None:
            self.state = "COMPLETE"
            self.publish_sequence_status("complete")
            return

        self.task_pub.publish(String(data=mission.command))
        self.state = "WAIT_MISSION_DONE"
        self.completion_armed = False
        self.publish_sequence_status("published")
        self.get_logger().warn(
            f"Auto demo mission {self.mission_index + 1}/{len(self.missions)}: "
            f"{mission.command}"
        )

    def on_mission_status(self, message: String) -> None:
        mission = self.current_mission
        if mission is None or self.state != "WAIT_MISSION_DONE":
            return
        event, self.completion_armed = status_event_for_mission(
            message.data,
            mission,
            self.completion_armed,
        )
        if event in {"ignore", "active"}:
            return
        if event == "error":
            self.state = "ERROR"
            self.publish_sequence_status("error", "mission reported an error")
            self.get_logger().error(
                f"Auto demo stopped on mission {self.mission_index + 1}"
            )
            return

        self.publish_sequence_status("mission_done")
        self.mission_index += 1
        self.completion_armed = False
        if self.mission_index >= len(self.missions):
            self.state = "COMPLETE"
            self.publish_sequence_status("complete")
            self.get_logger().warn("Automatic demo sequence COMPLETE")
            return
        self.state = "WAIT_INTER_TASK_DELAY"
        self.next_publish_ns = (
            self.get_clock().now().nanoseconds + self.inter_task_delay_ns
        )
        self.publish_sequence_status("waiting_next")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoTaskSequencer()
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
