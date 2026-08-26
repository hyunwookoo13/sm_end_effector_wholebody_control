import json

import pytest

from sm_semantic_mvp.demo_task_sequencer import (
    DEFAULT_MISSIONS,
    DemoMission,
    parse_mission_plan,
    parse_mission_status,
    status_event_for_mission,
)


def test_default_plan_contains_two_requested_missions_in_order():
    missions = parse_mission_plan(DEFAULT_MISSIONS)

    assert missions == [
        DemoMission(
            command="빨간 캔을 분홍 박스에 넣어줘",
            pick="red_can",
            place="pink_box",
        ),
        DemoMission(
            command="파란 캔을 노란 박스에 넣어줘",
            pick="blue_can",
            place="yellow_box",
        ),
    ]


@pytest.mark.parametrize("payload", ["not json", "[]", '[{"command": "x"}]'])
def test_invalid_plans_are_rejected(payload):
    with pytest.raises(ValueError):
        parse_mission_plan(payload)


def test_status_parser_ignores_invalid_payload():
    assert parse_mission_status("not json") == {}


def test_old_done_cannot_skip_a_new_mission():
    mission = DemoMission("command", "blue_can", "yellow_box")
    old_done = json.dumps(
        {
            "state": "done",
            "pick_object_id": "red_can",
            "place_object_id": "pink_box",
        }
    )
    matching_done = json.dumps(
        {
            "state": "done",
            "pick_object_id": "blue_can",
            "place_object_id": "yellow_box",
        }
    )

    assert status_event_for_mission(old_done, mission, False) == ("ignore", False)
    assert status_event_for_mission(matching_done, mission, False) == (
        "ignore",
        False,
    )


def test_matching_active_then_done_advances_mission():
    mission = DemoMission("command", "blue_can", "yellow_box")
    active = json.dumps(
        {
            "state": "nav_to_pick",
            "pick_object_id": "blue_can",
            "place_object_id": "yellow_box",
        }
    )
    done = json.dumps(
        {
            "state": "done",
            "pick_object_id": "blue_can",
            "place_object_id": "yellow_box",
        }
    )

    assert status_event_for_mission(active, mission, False) == ("active", True)
    assert status_event_for_mission(done, mission, True) == ("done", True)
