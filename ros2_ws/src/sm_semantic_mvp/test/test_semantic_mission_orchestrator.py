import json

from std_msgs.msg import String

from sm_semantic_mvp.semantic_mission_orchestrator import (
    SemanticMissionOrchestrator,
    navigation_status_for_task,
    parse_resolved_task,
    task_phase,
)


def resolved_payload(place_zone="second_table"):
    return json.dumps(
        {
            "ok": True,
            "input": {"pick": "red can", "place": "pink box"},
            "pick": {
                "object_id": "red_can",
                "canonical_name": "red can",
                "zone": "first_table",
                "source_sensor": "rsd455",
                "approach_pose": {
                    "frame_id": "map",
                    "x": 3.549,
                    "y": 4.321,
                    "yaw": 0.0,
                },
            },
            "place": {
                "object_id": "pink_box",
                "canonical_name": "pink box",
                "zone": place_zone,
                "source_sensor": "rsd455_2",
                "approach_pose": {
                    "frame_id": "map",
                    "x": -0.6,
                    "y": -6.101,
                    "yaw": -1.570796,
                },
            },
        }
    )


def test_multi_zone_task_preserves_existing_input_contract():
    task = parse_resolved_task(resolved_payload())

    assert not task.same_workspace
    assert json.loads(task.existing_task_payload()) == {
        "pick": "red can",
        "place": "pink box",
    }


def test_navigation_requests_embed_both_db_poses_without_global_state_race():
    task = parse_resolved_task(resolved_payload())

    pick_request = json.loads(task.navigation_request_payload("pick"))
    place_request = json.loads(task.navigation_request_payload("place"))

    assert pick_request["object_id"] == "red_can"
    assert pick_request["resolved_task"]["pick"]["approach_pose"]["x"] == 3.549
    assert place_request["object_id"] == "pink_box"
    assert place_request["resolved_task"]["place"]["approach_pose"]["y"] == -6.101


def test_same_workspace_is_detected_for_direct_place_resume():
    task = parse_resolved_task(resolved_payload(place_zone="first_table"))

    assert task.same_workspace


def test_only_matching_navigation_status_advances_each_role():
    task = parse_resolved_task(resolved_payload())
    place_ready = json.dumps(
        {"state": "handoff_ready", "role": "place", "object_id": "pink_box"}
    )
    wrong_object = json.dumps(
        {"state": "handoff_ready", "role": "place", "object_id": "yellow_box"}
    )

    assert navigation_status_for_task(place_ready, task, "place") == "handoff_ready"
    assert navigation_status_for_task(place_ready, task, "pick") == ""
    assert navigation_status_for_task(wrong_object, task, "place") == ""


def test_task_manager_state_prefix_is_parsed_without_coupling_to_debug_text():
    state = "WAIT_PLACE_NAVIGATION: pick=red can, place=pink box, arm=TRANSPORT:HOLD"

    assert task_phase(state) == "WAIT_PLACE_NAVIGATION"


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class Logger:
    def info(self, message):
        pass

    def warn(self, message):
        pass


def mission_orchestrator(place_zone="second_table"):
    node = SemanticMissionOrchestrator.__new__(SemanticMissionOrchestrator)
    node.task = parse_resolved_task(resolved_payload(place_zone=place_zone))
    node.state = "PICK"
    node.task_dispatched = True
    node.place_navigation_requested = False
    node.place_resumed = False
    node.last_task_phase = "PICK"
    node.current_workspace = "first_table"
    node.departure_retreat_seen = False
    node.task_started_seen = True
    node.phase_command_pub = Publisher()
    node.navigation_request_pub = Publisher()
    node.task_pub = Publisher()
    node._publish_status = lambda *args: None
    node.get_logger = lambda: Logger()
    return node


def blue_to_yellow_payload():
    payload = json.loads(resolved_payload())
    payload["input"] = {"pick": "blue can", "place": "yellow box"}
    payload["pick"].update(
        {
            "object_id": "blue_can",
            "canonical_name": "blue can",
            "zone": "second_table",
        }
    )
    payload["place"].update(
        {
            "object_id": "yellow_box",
            "canonical_name": "yellow box",
            "zone": "first_table",
        }
    )
    return json.dumps(payload)


def test_cross_workspace_requests_safe_retreat_during_transport_preparation():
    node = mission_orchestrator()

    node._on_task_state(String(data="PREPARE_PLACE_NAVIGATION: holding red can"))

    assert node.phase_command_pub.messages == ["START_PLACE_RETREAT"]


def test_same_workspace_does_not_request_nav2_departure_retreat():
    node = mission_orchestrator(place_zone="first_table")

    node._on_task_state(String(data="PREPARE_PLACE_NAVIGATION: holding red can"))

    assert node.phase_command_pub.messages == []


def test_consecutive_task_in_current_workspace_skips_redundant_nav2():
    node = mission_orchestrator()
    node.state = "DONE"
    node.last_task_phase = "DONE"
    node.current_workspace = "second_table"

    node._on_resolved_task(String(data=blue_to_yellow_payload()))

    assert node.navigation_request_pub.messages == []
    assert node.phase_command_pub.messages == []
    assert json.loads(node.task_pub.messages[0]) == {
        "pick": "blue can",
        "place": "yellow box",
    }
    assert node.state == "PICK"


def test_stale_done_is_ignored_until_consecutive_task_actually_starts():
    node = mission_orchestrator()
    node.state = "DONE"
    node.last_task_phase = "DONE"
    node.current_workspace = "second_table"
    node._on_resolved_task(String(data=blue_to_yellow_payload()))

    node._on_task_state(String(data="DONE: previous task"))
    assert node.state == "PICK"

    node._on_task_state(String(data="FIND_PICK: blue can"))
    node._on_task_state(String(data="DONE: blue can -> yellow box"))
    assert node.state == "DONE"
    assert node.current_workspace == "first_table"


def test_consecutive_task_in_other_workspace_retreats_before_nav2():
    node = mission_orchestrator()
    node.state = "DONE"
    node.last_task_phase = "DONE"
    node.current_workspace = "first_table"

    node._on_resolved_task(String(data=blue_to_yellow_payload()))

    assert node.state == "WAIT_DEPARTURE_RETREAT"
    assert node.phase_command_pub.messages == ["START_DEPARTURE_RETREAT"]
    assert node.navigation_request_pub.messages == []

    node._on_task_state(String(data="SAFE_RETREAT: leaving first_table"))
    node._on_task_state(String(data="DONE: departure complete"))

    assert node.state == "NAV_TO_PICK"
    request = json.loads(node.navigation_request_pub.messages[0])
    assert request["role"] == "pick"
    assert request["object_id"] == "blue_can"
