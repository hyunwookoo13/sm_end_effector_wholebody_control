from geometry_msgs.msg import TransformStamped

from ee_switch_debug.pick_place_task_manager import (
    PickPlaceTaskManager,
    departure_retreat_for_new_task,
)


def make_retreat_manager():
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.departure_retreat_pending = True
    manager.retreat_distance_m = 0.40
    manager.events = []
    manager.publish_arm_command = lambda command: manager.events.append(
        ("arm", command)
    )
    manager.start_safe_retreat = lambda next_phase="FIND_PLACE": manager.events.append(
        ("retreat", next_phase)
    )
    return manager


def test_only_consecutive_done_task_marks_departure_retreat():
    assert departure_retreat_for_new_task("DONE", enable_navigation=True)
    assert not departure_retreat_for_new_task("IDLE", enable_navigation=True)
    assert not departure_retreat_for_new_task("DONE", enable_navigation=False)


def test_close_pick_clears_pending_retreat_without_moving():
    manager = make_retreat_manager()

    started = manager.maybe_start_departure_retreat("pick", skip_navigation=True)

    assert started is False
    assert manager.departure_retreat_pending is False
    assert manager.events == []


def test_remote_pick_starts_transport_retreat_before_navigation():
    manager = make_retreat_manager()

    started = manager.maybe_start_departure_retreat("pick", skip_navigation=False)

    assert started is True
    assert manager.departure_retreat_pending is False
    assert manager.events == [
        ("arm", "TRANSPORT"),
        ("retreat", "FIND_PICK"),
    ]


def make_completed_retreat_manager(next_phase):
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    now = type("Now", (), {"nanoseconds": 2_000_000_000})()
    manager.get_clock = lambda: type(
        "Clock",
        (),
        {"now": lambda self: now},
    )()
    manager.retreat_phase_start_ns = 1_000_000_000
    manager.transport_settle_ns = 0
    manager.retreat_timeout_ns = 5_000_000_000
    manager.rear_scan_received_ns = 2_000_000_000
    manager.retreat_scan_timeout_ns = 1_000_000_000
    manager.rear_clearance_m = 1.0
    manager.retreat_min_clearance_m = 0.50
    manager.retreat_start_xy = (0.0, 0.0)
    manager.retreat_distance_m = 0.40
    manager.retreat_speed_mps = 0.25
    manager.retreat_next_phase = next_phase
    manager.navigation_base_xy = lambda: (0.40, 0.0)
    manager.publish_retreat_command = lambda speed: None
    manager.publish_base_mode = lambda mode: None
    manager.get_logger = lambda: type(
        "Logger",
        (),
        {"warn": lambda self, message: None},
    )()
    return manager


def test_departure_retreat_completion_returns_to_find_pick():
    manager = make_completed_retreat_manager("FIND_PICK")

    manager.advance_safe_retreat()

    assert manager.phase == "FIND_PICK"


def test_existing_retreat_completion_still_returns_to_find_place():
    manager = make_completed_retreat_manager("FIND_PLACE")

    manager.advance_safe_retreat()

    assert manager.phase == "FIND_PLACE"


def test_remote_pick_retreats_before_submitting_nav2_goal():
    manager = make_retreat_manager()
    manager.parent_frame = "odom"
    manager.navigation_base_frame = "chassis_link"
    manager.tf_timeout_sec = 0.1
    manager.pick_standoff_m = 0.70
    manager.place_standoff_m = 0.70
    manager.navigation_goal_skip_distance_m = 0.20
    manager.place_direct_approach_distance_m = 1.20

    base_transform = TransformStamped()
    manager.tf_buffer = type(
        "Buffer",
        (),
        {"lookup_transform": lambda self, *args, **kwargs: base_transform},
    )()
    manager.navigation_client = type(
        "NavigationClient",
        (),
        {
            "server_is_ready": lambda self: True,
            "send_goal_async": lambda self, goal: (_ for _ in ()).throw(
                AssertionError("Nav2 goal submitted before retreat")
            ),
        },
    )()

    target = TransformStamped()
    target.transform.translation.x = 3.0

    result = manager.request_navigation("pick", target)

    assert result == "retreating"
    assert manager.events[-1] == ("retreat", "FIND_PICK")


def test_find_pick_does_not_fall_through_while_retreating():
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.phase = "FIND_PICK"
    manager.pick_transform = TransformStamped()
    manager.pick_object = "apple"
    manager.request_navigation = lambda kind, target: "retreating"
    manager.begin_pick_manipulation = lambda: (_ for _ in ()).throw(
        AssertionError("pick manipulation started during retreat")
    )

    manager.advance_phase()

    assert manager.phase == "FIND_PICK"
