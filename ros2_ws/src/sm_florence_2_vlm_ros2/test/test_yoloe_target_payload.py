import json

from sm_florence_2_vlm.yoloe_vlm_node import YOLOEVLMNode


def test_parse_target_payload_separates_detection_and_roi_targets():
    payload = json.dumps(
        {
            "target_objects": ["blue can", "yellow box"],
            "roi_target_objects": ["blue can"],
        }
    )

    target_objects, roi_target_objects = YOLOEVLMNode._parse_target_payload(payload)

    assert target_objects == ["blue can", "yellow box"]
    assert roi_target_objects == ["blue can"]


def test_parse_target_payload_clears_roi_filter_for_legacy_string_input():
    target_objects, roi_target_objects = YOLOEVLMNode._parse_target_payload("blue can, yellow box")

    assert target_objects == ["blue can", "yellow box"]
    assert roi_target_objects == []


def test_roi_target_filter_keeps_only_requested_pick_object():
    assert YOLOEVLMNode._is_roi_target_object("blue can", ["blue can"])
    assert not YOLOEVLMNode._is_roi_target_object("yellow box", ["blue can"])
    assert YOLOEVLMNode._is_roi_target_object("yellow box", [])
