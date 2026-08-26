import json
from pathlib import Path

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from sm_semantic_map.semantic_db import SemanticMapDB
from sm_semantic_map.semantic_marker_publisher import (
    build_marker_array,
    load_objects_read_only,
    parse_mission_state,
    parse_selected_objects,
)


SEED = Path(__file__).resolve().parents[1] / "config" / "warehouse_objects.yaml"


def resolved_task():
    return json.dumps(
        {
            "ok": True,
            "pick": {"object_id": "red_can"},
            "place": {"object_id": "pink_box"},
        }
    )


def test_database_is_loaded_through_read_only_visualization_adapter(tmp_path):
    path = tmp_path / "objects.sqlite3"
    database = SemanticMapDB(path)
    try:
        database.load_seed(SEED)
    finally:
        database.close()

    objects = load_objects_read_only(path)

    assert len(objects) == 6
    assert {item["object_id"] for item in objects} == {
        "red_can",
        "orange",
        "yellow_box",
        "apple",
        "blue_can",
        "pink_box",
    }


def test_resolved_task_and_mission_state_parsers_ignore_invalid_payloads():
    assert parse_selected_objects(resolved_task()) == ("red_can", "pink_box")
    assert parse_selected_objects("not json") == ("", "")
    assert parse_mission_state('{"state": "nav_to_pick"}') == "NAV_TO_PICK"
    assert parse_mission_state("not json") == ""


def test_marker_array_contains_objects_labels_approaches_and_selection(tmp_path):
    path = tmp_path / "objects.sqlite3"
    database = SemanticMapDB(path)
    try:
        database.load_seed(SEED)
    finally:
        database.close()
    objects = load_objects_read_only(path)

    result = build_marker_array(
        objects,
        Time(),
        selected_pick="red_can",
        selected_place="pink_box",
        mission_state="NAV_TO_PICK",
    )

    assert len(result.markers) == 17
    assert all(
        marker.header.frame_id == "map"
        for marker in result.markers
        if marker.ns != "semantic_mission"
    )
    assert any(
        marker.ns == "semantic_labels" and marker.text == "PICK\nRED_CAN"
        for marker in result.markers
    )
    assert any(
        marker.ns == "semantic_labels" and marker.text == "PLACE\nPINK_BOX"
        for marker in result.markers
    )
    assert sum(marker.type == Marker.ARROW for marker in result.markers) == 2
    assert sum(marker.ns == "semantic_station_labels" for marker in result.markers) == 2
    status = next(marker for marker in result.markers if marker.ns == "semantic_mission")
    assert status.header.frame_id == "chassis_link"
    assert not status.frame_locked
    assert status.text.startswith("MISSION\nNAV_TO_PICK")
