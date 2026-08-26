from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config" / "warehouse_objects.yaml"


def test_seed_contains_six_unique_map_objects():
    payload = yaml.safe_load(CONFIG.read_text())
    objects = payload["objects"]
    assert payload["frame_id"] == "map"
    assert len(objects) == 6
    assert len({item["object_id"] for item in objects}) == 6


def test_seed_objects_have_query_and_navigation_fields():
    payload = yaml.safe_load(CONFIG.read_text())
    for item in payload["objects"]:
        assert item["canonical_name"]
        assert item["aliases"]
        assert item["capabilities"]
        assert item["status"] == "AVAILABLE"
        assert set(item["object_pose"]) == {"x", "y", "z"}
        assert set(item["approach_pose"]) == {"x", "y", "yaw"}
        assert item["approach_clearance_m"] >= 0.60
