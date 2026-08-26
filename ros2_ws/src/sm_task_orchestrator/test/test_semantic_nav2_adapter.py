import json
import math

import pytest

from sm_task_orchestrator.semantic_nav2_adapter import (
    goal_to_pose,
    handoff_feedback_state,
    parse_navigation_request,
    parse_semantic_navigation_goal,
)


def resolved_task() -> str:
    return json.dumps(
        {
            "ok": True,
            "pick": {
                "object_id": "blue_can",
                "approach_pose": {
                    "frame_id": "map",
                    "x": -0.917,
                    "y": -6.111,
                    "yaw": -math.pi / 2.0,
                },
            },
            "place": {
                "object_id": "pink_box",
                "approach_pose": {
                    "frame_id": "map",
                    "x": -0.6,
                    "y": -6.101,
                    "yaw": -math.pi / 2.0,
                },
            },
        }
    )


def test_parses_pick_approach_pose_only():
    goal = parse_semantic_navigation_goal(resolved_task())

    assert goal.role == "pick"
    assert goal.object_id == "blue_can"
    assert goal.frame_id == "map"
    assert goal.x == pytest.approx(-0.917)
    assert goal.yaw == pytest.approx(-math.pi / 2.0)


def test_rejects_failed_resolution_and_wrong_frame():
    with pytest.raises(ValueError, match="not successful"):
        parse_semantic_navigation_goal('{"ok": false}')

    payload = json.loads(resolved_task())
    payload["pick"]["approach_pose"]["frame_id"] = "odom"
    with pytest.raises(ValueError, match="expected map"):
        parse_semantic_navigation_goal(json.dumps(payload))


def test_builds_nav2_pose_quaternion():
    goal = parse_semantic_navigation_goal(resolved_task())
    pose = goal_to_pose(goal, stamp=None)

    assert pose.header.frame_id == "map"
    assert pose.pose.position.x == pytest.approx(-0.917)
    assert pose.pose.position.y == pytest.approx(-6.111)
    assert pose.pose.orientation.z == pytest.approx(math.sin(-math.pi / 4.0))
    assert pose.pose.orientation.w == pytest.approx(math.cos(-math.pi / 4.0))


def test_request_selects_place_pose_from_embedded_resolved_task():
    payload = {
        "role": "place",
        "object_id": "pink_box",
        "resolved_task": json.loads(resolved_task()),
    }

    role, task_json, expected_object_id = parse_navigation_request(
        json.dumps(payload)
    )
    goal = parse_semantic_navigation_goal(task_json, role=role)

    assert expected_object_id == "pink_box"
    assert goal.role == "place"
    assert goal.object_id == "pink_box"
    assert goal.x == pytest.approx(-0.6)
    assert goal.y == pytest.approx(-6.101)


def test_plain_role_request_uses_cached_resolved_task():
    role, task_json, expected_object_id = parse_navigation_request(
        "place",
        fallback_resolved_task=resolved_task(),
    )

    assert role == "place"
    assert json.loads(task_json)["place"]["object_id"] == "pink_box"
    assert expected_object_id == ""


def test_handoff_requires_real_approach_before_threshold_crossing():
    armed, ready = handoff_feedback_state(0.0, 0.20, armed=False)
    assert not armed
    assert not ready

    armed, ready = handoff_feedback_state(0.0, 0.20, armed=True)
    assert armed
    assert not ready

    armed, ready = handoff_feedback_state(3.0, 0.20, armed=False)
    assert armed
    assert not ready

    armed, ready = handoff_feedback_state(0.19, 0.20, armed=armed)
    assert armed
    assert ready


def test_zero_handoff_distance_disables_early_handoff():
    armed, ready = handoff_feedback_state(0.10, 0.0, armed=True)
    assert armed
    assert not ready
