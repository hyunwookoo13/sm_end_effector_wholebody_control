from copy import deepcopy

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, String

from ee_switch_debug.pick_perception_snapshot import PickPerceptionSnapshot


class FakeTime:
    def __init__(self, nanoseconds: int):
        self.nanoseconds = nanoseconds

    def to_msg(self) -> Time:
        return Time(
            sec=self.nanoseconds // 1_000_000_000,
            nanosec=self.nanoseconds % 1_000_000_000,
        )


class FakeClock:
    def __init__(self):
        self.nanoseconds = 1_000_000_000

    def now(self) -> FakeTime:
        return FakeTime(self.nanoseconds)

    def advance(self, seconds: float) -> None:
        self.nanoseconds += int(seconds * 1_000_000_000)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(deepcopy(msg))


def make_snapshot():
    node = PickPerceptionSnapshot.__new__(PickPerceptionSnapshot)
    node.state = PickPerceptionSnapshot.LIVE
    node.grasp_settle_ns = 400_000_000
    node.locked_roi = None
    node.locked_grasp = None
    node.locked_opening = None
    node.pending_opening = None
    node.roi_snapshot_time_ns = 0
    node.clock = FakeClock()
    node.get_clock = lambda: node.clock
    node.roi_pub = FakePublisher()
    node.grasp_pub = FakePublisher()
    node.opening_pub = FakePublisher()
    node.logs = []
    node.get_logger = lambda: type(
        "Logger",
        (),
        {
            "info": lambda self, message: node.logs.append(message),
            "warn": lambda self, message: node.logs.append(message),
            "debug": lambda self, message: node.logs.append(message),
        },
    )()
    return node


def make_cloud(value: int = 1, *, empty: bool = False) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header.frame_id = "rsd455_color_optical_frame"
    cloud.height = 0 if empty else 1
    cloud.width = 0 if empty else 1
    cloud.point_step = 4
    cloud.row_step = 0 if empty else 4
    cloud.data = [] if empty else [value, value, value, value]
    return cloud


def make_grasp(x: float, *, frame_id: str = "chassis_link") -> PoseStamped:
    grasp = PoseStamped()
    grasp.header.frame_id = frame_id
    grasp.pose.position.x = x
    grasp.pose.orientation.w = 1.0
    return grasp


def make_opening(value: float) -> Float32MultiArray:
    return Float32MultiArray(data=[value])


def begin_and_capture_roi(node, value: int = 7) -> None:
    node.on_task_command(String(data="PICK"))
    node.on_live_roi(make_cloud(value))


def lock_bundle(node, x: float = 0.4, opening: float = 0.05) -> None:
    begin_and_capture_roi(node)
    node.on_live_opening(make_opening(opening))
    node.clock.advance(0.5)
    node.on_live_grasp(make_grasp(x))


def test_live_messages_pass_through_to_canonical_topics():
    node = make_snapshot()

    node.on_live_roi(make_cloud(2))
    node.on_live_opening(make_opening(0.04))
    node.on_live_grasp(make_grasp(0.3))

    assert list(node.roi_pub.messages[-1].data) == [2, 2, 2, 2]
    assert node.opening_pub.messages[-1].data == pytest.approx([0.04])
    assert node.grasp_pub.messages[-1].pose.position.x == pytest.approx(0.3)


def test_pick_invalidates_previous_bundle_and_waits_for_first_fresh_roi():
    node = make_snapshot()
    node.locked_roi = make_cloud(3)
    node.locked_grasp = make_grasp(0.2)
    node.locked_opening = make_opening(0.03)

    node.on_task_command(String(data="PICK"))

    assert node.state == PickPerceptionSnapshot.WAITING_ROI
    assert node.locked_roi is None
    assert node.locked_grasp is None
    assert node.locked_opening is None


def test_first_nonempty_roi_after_pick_is_captured_and_later_roi_is_ignored():
    node = make_snapshot()
    node.on_task_command(String(data="PICK"))

    node.on_live_roi(make_cloud(empty=True))
    assert node.state == PickPerceptionSnapshot.WAITING_ROI

    node.on_live_roi(make_cloud(4))
    node.on_live_roi(make_cloud(9))

    assert node.state == PickPerceptionSnapshot.WAITING_GRASP
    assert list(node.locked_roi.data) == [4, 4, 4, 4]


def test_grasp_before_roi_or_during_settle_window_is_ignored():
    node = make_snapshot()
    node.on_task_command(String(data="PICK"))
    node.on_live_grasp(make_grasp(0.1))
    assert node.locked_grasp is None

    node.on_live_roi(make_cloud(5))
    node.on_live_grasp(make_grasp(0.2))

    assert node.state == PickPerceptionSnapshot.WAITING_GRASP
    assert node.locked_grasp is None
    assert node.grasp_pub.messages == []


def test_first_post_settle_grasp_and_opening_lock_and_later_updates_are_ignored():
    node = make_snapshot()
    begin_and_capture_roi(node, value=6)
    node.on_live_opening(make_opening(0.06))
    node.clock.advance(0.5)

    node.on_live_grasp(make_grasp(0.4))
    node.on_live_opening(make_opening(0.12))
    node.on_live_grasp(make_grasp(0.9))

    assert node.state == PickPerceptionSnapshot.LOCKED
    assert node.locked_grasp.pose.position.x == pytest.approx(0.4)
    assert node.locked_opening.data == pytest.approx([0.06])


def test_invalid_grasp_does_not_consume_snapshot():
    node = make_snapshot()
    begin_and_capture_roi(node)
    node.clock.advance(0.5)

    node.on_live_grasp(make_grasp(0.4, frame_id=""))

    assert node.state == PickPerceptionSnapshot.WAITING_GRASP
    assert node.locked_grasp is None


def test_timer_republishes_identical_locked_bundle_with_fresh_stamps():
    node = make_snapshot()
    lock_bundle(node, x=0.45, opening=0.055)
    node.clock.advance(1.0)

    node.on_timer()

    assert list(node.roi_pub.messages[-1].data) == [7, 7, 7, 7]
    assert node.grasp_pub.messages[-1].pose.position.x == pytest.approx(0.45)
    assert node.opening_pub.messages[-1].data == pytest.approx([0.055])
    assert node.roi_pub.messages[-1].header.stamp.sec == 2
    assert node.grasp_pub.messages[-1].header.stamp.sec == 2


@pytest.mark.parametrize("state", ["PICK:HOLD", "PICK:DONE"])
def test_pick_completion_releases_bundle(state):
    node = make_snapshot()
    lock_bundle(node)

    node.on_task_state(String(data=state))

    assert node.state == PickPerceptionSnapshot.LIVE
    assert node.locked_roi is None
    assert node.locked_grasp is None


def test_reset_and_repeated_pick_start_a_new_snapshot_boundary():
    node = make_snapshot()
    lock_bundle(node)

    node.on_task_command(String(data="RESET"))
    assert node.state == PickPerceptionSnapshot.LIVE

    lock_bundle(node, x=0.5)
    node.on_task_command(String(data="PICK"))
    assert node.state == PickPerceptionSnapshot.WAITING_ROI
    assert node.locked_grasp is None
