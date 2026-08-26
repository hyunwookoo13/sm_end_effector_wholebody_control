import math

import pytest

from sm_semantic_map_interfaces.srv import FindObject
from sm_task_orchestrator.semantic_task_resolver import (
    object_response_to_dict,
    parse_task_payload,
)


def test_parse_task_payload_accepts_parser_output():
    assert parse_task_payload('{"pick":"blue can","place":"yellow box"}') == (
        "blue can",
        "yellow box",
    )


def test_parse_task_payload_rejects_missing_or_equal_targets():
    assert parse_task_payload('{"pick":"blue can"}') is None
    assert parse_task_payload('{"pick":"apple","place":"apple"}') is None
    assert parse_task_payload("not json") is None


def test_object_response_serializes_map_approach_pose():
    response = FindObject.Response()
    response.found = True
    response.object_id = "blue_can"
    response.canonical_name = "blue can"
    response.semantic_class = "can"
    response.capabilities = ["pick"]
    response.status = "AVAILABLE"
    response.zone = "second_table"
    response.source_sensor = "rsd455_2"
    response.object_pose.header.frame_id = "map"
    response.object_pose.pose.position.x = -0.917
    response.object_pose.pose.position.y = -7.111
    response.object_pose.pose.position.z = 0.827
    response.approach_pose.header.frame_id = "map"
    response.approach_pose.pose.position.x = -0.917
    response.approach_pose.pose.position.y = -6.111
    yaw = -math.pi / 2.0
    response.approach_pose.pose.orientation.z = math.sin(yaw * 0.5)
    response.approach_pose.pose.orientation.w = math.cos(yaw * 0.5)
    response.approach_clearance_m = 0.802

    payload = object_response_to_dict(response)

    assert payload["object_id"] == "blue_can"
    assert payload["approach_pose"]["frame_id"] == "map"
    assert payload["approach_pose"]["yaw"] == pytest.approx(yaw)
