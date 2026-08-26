from std_msgs.msg import String

from sm_task_orchestrator.pick_place_task_manager import PickPlaceTaskManager


class Clock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return self


class Logger:
    def warn(self, message):
        pass


def make_manager(now_ns=1_000_000_000):
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.phase = "PICK"
    manager.arm_task_state = "PICK:HOLD"
    manager.external_place_navigation = True
    manager.pick_object = "red can"
    manager.place_object = "pink box"
    manager.current_perception_object = ""
    manager.last_target_objects_publish_ns = 1
    manager.current_transform = object()
    manager.transport_settle_ns = 1_000_000_000
    manager.external_place_ready_ns = 0
    manager.events = []
    manager.get_clock = lambda: Clock(now_ns)
    manager.get_logger = lambda: Logger()
    manager.publish_target_object = lambda force: manager.events.append(("targets", force))
    manager.publish_arm_command = lambda command: manager.events.append(("arm", command))
    manager.publish_base_mode = lambda mode: manager.events.append(("base", mode))
    manager.publish_task_state = lambda: manager.events.append(("state", manager.phase))
    return manager


def test_pick_hold_enters_transport_preparation_without_changing_default_algorithm():
    manager = make_manager()

    manager.advance_phase()

    assert manager.phase == "PREPARE_PLACE_NAVIGATION"
    assert manager.external_place_ready_ns == 2_000_000_000
    assert ("arm", "TRANSPORT") in manager.events
    assert ("base", "STOP") in manager.events


def test_transport_settle_gate_exposes_place_navigation_ready_phase():
    manager = make_manager(now_ns=2_000_000_000)
    manager.phase = "PREPARE_PLACE_NAVIGATION"
    manager.external_place_ready_ns = 2_000_000_000

    manager.advance_phase()

    assert manager.phase == "WAIT_PLACE_NAVIGATION"


def test_cross_workspace_command_reuses_safe_retreat_before_place_nav2():
    manager = make_manager()
    manager.phase = "PREPARE_PLACE_NAVIGATION"

    def start_safe_retreat(next_phase):
        manager.events.append(("retreat", next_phase))
        manager.phase = "SAFE_RETREAT"

    manager.start_safe_retreat = start_safe_retreat

    manager.on_phase_command(String(data="START_PLACE_RETREAT"))

    assert manager.phase == "SAFE_RETREAT"
    assert ("retreat", "WAIT_PLACE_NAVIGATION") in manager.events


def test_resume_place_discards_transit_cache_and_waits_for_fresh_detection():
    manager = make_manager()
    manager.phase = "WAIT_PLACE_NAVIGATION"
    manager.place_transform = object()
    manager.selected_place_source = "/cached/detections"

    manager.on_phase_command(String(data="RESUME_PLACE"))

    assert manager.phase == "FIND_PLACE"
    assert manager.place_transform is None
    assert manager.selected_place_source == ""
    assert manager.current_transform is None
    assert ("base", "MANIPULATION") in manager.events


def test_external_phase_command_is_ignored_by_default():
    manager = make_manager()
    manager.external_place_navigation = False
    manager.phase = "WAIT_PLACE_NAVIGATION"

    manager.on_phase_command(String(data="RESUME_PLACE"))

    assert manager.phase == "WAIT_PLACE_NAVIGATION"
