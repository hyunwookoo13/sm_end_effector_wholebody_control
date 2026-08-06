import json

from geometry_msgs.msg import PoseStamped
from pytest import approx
from std_msgs.msg import String

from ee_switch_debug.pick_place_task_manager import PickPlaceTaskManager


def make_manager_without_ros_node():
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.pick_object = "blue can"
    manager.place_object = "yellow box"
    manager.detections_topic = "/sm_florence_2_vlm/detections"
    manager.place_detections_topic = "/sm_florence_2_vlm_place/detections"
    manager.pick_offset = (0.0, 0.0, 0.0)
    manager.place_offset = (0.0, 0.0, 0.0)
    manager.min_place_confidence = 0.0
    manager.pick_orientation = None
    manager.pick_orientation_frame = ""
    manager.pick_orientation_by_source = {}
    manager.place_transform = None
    manager.current_transform = None
    manager.selected_pick_source = ""
    manager.selected_place_source = ""
    manager.last_debug = ""
    manager.pose_to_parent_transform = (
        lambda source_frame, position, orientation, orientation_frame=None: {
            "source_frame": source_frame,
            "position": position,
            "orientation": orientation,
            "orientation_frame": orientation_frame,
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


def pick_detection_payload():
    return {
        "objects": [
            {
                "object_name": "blue can",
                "semantic_class": "can",
                "requested_color": "blue",
                "observed_color": "blue",
                "confidence": 0.81,
                "orientation_camera_frame": {
                    "frame_id": "rsd455_color_optical_frame",
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.5,
                    "w": 0.8660254,
                    "angle_rad": 1.0472,
                    "confidence": 0.92,
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
        "orientation_frame": None,
    }
    assert manager.selected_place_source == "/sm_florence_2_vlm_place/detections"


def test_perception_payload_detects_pick_and_place_but_limits_roi_to_pick():
    payload = PickPlaceTaskManager.build_perception_payload("blue can", "yellow box")

    assert json.loads(payload) == {
        "target_objects": ["blue can", "yellow box"],
        "roi_target_objects": ["blue can"],
    }


def test_open_vocabulary_queries_reach_perception_without_rewriting():
    payload = PickPlaceTaskManager.build_perception_payload("orange", "pink box")

    assert json.loads(payload) == {
        "target_objects": ["orange", "pink box"],
        "roi_target_objects": ["orange"],
    }


def test_find_pick_does_not_navigate_without_fresh_grounding():
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.phase = "FIND_PICK"
    manager.pick_object = "orange"
    manager.pick_transform = None
    manager.last_debug = ""
    manager.request_navigation = lambda *args: (_ for _ in ()).throw(
        AssertionError("navigation requested before grounding")
    )

    manager.advance_phase()

    assert manager.phase == "FIND_PICK"
    assert manager.last_debug == "waiting for pick grasp: orange"


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


def test_pick_detection_orientation_overrides_grasp_orientation():
    manager = make_manager_without_ros_node()
    manager.phase = "FIND_PICK"

    manager.on_detections(String(data=json.dumps(pick_detection_payload())), "/sm_florence_2_vlm/detections")

    assert manager.pick_orientation == approx((0.0, 0.0, 0.5, 0.8660254))
    assert manager.pick_orientation_frame == "rsd455_color_optical_frame"

    grasp = PoseStamped()
    grasp.header.frame_id = "chassis_link"
    grasp.pose.position.x = 0.4
    grasp.pose.position.y = 0.1
    grasp.pose.position.z = 0.7
    grasp.pose.orientation.w = 1.0
    manager.is_fresh_grasp_message = lambda msg, now_ns: True
    manager.get_clock = lambda: type("Clock", (), {"now": lambda self: type("Now", (), {"nanoseconds": 20_000})()})()

    manager.on_grasp_best(grasp, "/sm_grasping/grasp_best")

    assert manager.pick_transform == {
        "source_frame": "chassis_link",
        "position": (0.4, 0.1, 0.7),
        "orientation": approx((0.0, 0.0, 0.5, 0.8660254)),
        "orientation_frame": "rsd455_color_optical_frame",
    }
