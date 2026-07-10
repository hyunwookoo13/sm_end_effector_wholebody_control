import json

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from ee_switch_debug.pick_place_task_manager import PickPlaceTaskManager


def make_manager_without_ros_node():
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.place_object = "yellow box"
    manager.place_offset = (0.0, 0.0, 0.0)
    manager.min_place_confidence = 0.0
    manager.place_transform = None
    manager.current_transform = None
    manager.selected_place_source = ""
    manager.last_debug = ""
    manager.pose_to_parent_transform = (
        lambda source_frame, position, orientation: {
            "source_frame": source_frame,
            "position": position,
            "orientation": orientation,
        }
    )
    return manager


def detection_payload():
    return {
        "objects": [
            {
                "object_name": "yellow box",
                "semantic_class": "box",
                "requested_color": "yellow",
                "observed_color": "yellow",
                "confidence": 0.77,
                "position_target_frame": {
                    "frame_id": "world",
                    "x": 1.2,
                    "y": -0.4,
                    "z": 0.8,
                },
            }
        ]
    }


def test_place_target_is_cached_while_pick_is_in_progress():
    manager = make_manager_without_ros_node()
    manager.phase = "PICK"

    manager.on_detections(String(data=json.dumps(detection_payload())), "/sm_florence_2_vlm_place/detections")

    assert manager.place_transform == {
        "source_frame": "world",
        "position": (1.2, -0.4, 0.8),
        "orientation": (0.0, 0.0, 0.0, 1.0),
    }
    assert manager.selected_place_source == "/sm_florence_2_vlm_place/detections"


def test_perception_payload_detects_pick_and_place_but_limits_roi_to_pick():
    payload = PickPlaceTaskManager.build_perception_payload("blue can", "yellow box")

    assert json.loads(payload) == {
        "target_objects": ["blue can", "yellow box"],
        "roi_target_objects": ["blue can"],
    }


def test_fresh_grasp_gate_rejects_early_and_old_stamped_grasps():
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.task_start_ns = 10_000
    manager.grasp_accept_after_ns = 15_000

    zero_stamp = PoseStamped()
    assert not manager.is_fresh_grasp_message(zero_stamp, now_ns=14_000)
    assert manager.is_fresh_grasp_message(zero_stamp, now_ns=16_000)

    old_stamped = PoseStamped()
    old_stamped.header.stamp.nanosec = 9_000
    assert not manager.is_fresh_grasp_message(old_stamped, now_ns=16_000)

    new_stamped = PoseStamped()
    new_stamped.header.stamp.nanosec = 11_000
    assert manager.is_fresh_grasp_message(new_stamped, now_ns=16_000)
