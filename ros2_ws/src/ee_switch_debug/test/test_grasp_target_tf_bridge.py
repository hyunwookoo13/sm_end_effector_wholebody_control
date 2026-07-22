import pytest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from ee_switch_debug.grasp_target_tf_bridge import GraspTargetTfBridge


class FakeBroadcaster:
    def __init__(self):
        self.broadcasts = []

    def sendTransform(self, transform):
        self.broadcasts.append(transform)


def make_bridge():
    bridge = GraspTargetTfBridge.__new__(GraspTargetTfBridge)
    bridge.parent_frame = "chassis_link"
    bridge.target_frame = "grasp_position_target"
    bridge.offset_x = 0.0
    bridge.offset_y = 0.0
    bridge.offset_z = 0.0
    bridge.latch_target = False
    bridge.target_latched = False
    bridge.latest_transform = object()
    bridge.latest_update_time = None
    bridge.last_debug = "x=0.4, y=0.1, z=0.7"
    bridge.target_frozen = False
    bridge.pick_freeze_requested = False
    bridge.pick_snapshot_active = False
    bridge.control_state = ""
    bridge.tf_broadcaster = FakeBroadcaster()
    bridge.broadcasts = bridge.tf_broadcaster.broadcasts
    messages = []
    bridge.get_logger = lambda: type(
        "Logger",
        (),
        {
            "info": lambda self, message: messages.append(message),
            "warn": lambda self, message: messages.append(message),
            "debug": lambda self, message: messages.append(message),
        },
    )()
    bridge.get_clock = lambda: type(
        "Clock", (), {"now": lambda self: "now"}
    )()
    return bridge, messages


def make_pose(
    *,
    x: float,
    y: float,
    z: float,
    qx: float = 0.0,
    qy: float = 0.0,
    qz: float = 0.0,
    qw: float = 1.0,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "chassis_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def lock_first_post_pick_grasp(bridge) -> None:
    bridge.on_task_command(String(data="PICK"))
    bridge.on_grasp_best(make_pose(x=0.4, y=0.1, z=0.7))


def test_pick_command_discards_existing_grasp_and_waits_for_fresh_sample():
    bridge, _ = make_bridge()
    stale = bridge.latest_transform

    bridge.on_task_command(String(data="PICK"))

    assert bridge.latest_transform is None
    assert bridge.pick_freeze_requested
    assert not bridge.pick_snapshot_active
    assert not bridge.target_frozen
    assert stale is not bridge.latest_transform


def test_pick_command_waits_for_first_grasp_when_none_exists():
    bridge, _ = make_bridge()
    bridge.latest_transform = None

    bridge.on_task_command(String(data="PICK"))

    assert bridge.pick_freeze_requested
    assert not bridge.pick_snapshot_active


def test_first_valid_grasp_after_pick_is_locked_and_later_grasps_are_ignored():
    bridge, _ = make_bridge()
    bridge.on_task_command(String(data="PICK"))

    first = make_pose(x=0.40, y=0.10, z=0.70, qz=0.1, qw=0.995)
    second = make_pose(x=0.20, y=-0.30, z=0.55, qz=0.7, qw=0.714)
    bridge.on_grasp_best(first)
    locked = bridge.latest_transform
    bridge.on_grasp_best(second)

    assert bridge.latest_transform is locked
    assert bridge.latest_transform.transform.translation.x == pytest.approx(0.40)
    assert bridge.latest_transform.transform.translation.y == pytest.approx(0.10)
    assert bridge.latest_transform.transform.translation.z == pytest.approx(0.70)
    assert bridge.latest_transform.transform.rotation.z == pytest.approx(
        first.pose.orientation.z, abs=1e-4
    )
    assert bridge.pick_snapshot_active
    assert bridge.target_frozen


def test_invalid_first_grasp_does_not_consume_pick_snapshot():
    bridge, _ = make_bridge()
    bridge.on_task_command(String(data="PICK"))
    invalid = make_pose(x=0.4, y=0.1, z=0.7)
    invalid.header.frame_id = ""

    bridge.on_grasp_best(invalid)

    assert bridge.latest_transform is None
    assert bridge.pick_freeze_requested
    assert not bridge.pick_snapshot_active
    assert not bridge.target_frozen


def test_timer_does_not_broadcast_old_target_while_waiting_for_first_pick_grasp():
    bridge, _ = make_bridge()
    bridge.on_task_command(String(data="PICK"))

    bridge.on_timer()

    assert bridge.broadcasts == []


def test_return_home_does_not_release_active_pick_snapshot():
    bridge, _ = make_bridge()
    bridge.target_frozen = True
    bridge.pick_snapshot_active = True
    bridge.control_state = "ARM_TRACK"

    bridge.on_control_state(String(data="RETURN_HOME"))

    assert bridge.target_frozen
    assert bridge.pick_snapshot_active


@pytest.mark.parametrize("state", ["PICK:HOLD", "PICK:DONE"])
def test_pick_completion_releases_snapshot_after_grasp_sequence(state):
    bridge, _ = make_bridge()
    lock_first_post_pick_grasp(bridge)

    bridge.on_task_state(String(data=state))

    assert not bridge.target_frozen
    assert not bridge.pick_snapshot_active


def test_reset_releases_snapshot_after_grasp_sequence():
    bridge, _ = make_bridge()
    lock_first_post_pick_grasp(bridge)

    bridge.on_task_command(String(data="RESET"))

    assert not bridge.target_frozen
    assert not bridge.pick_snapshot_active
    assert not bridge.pick_freeze_requested


def test_repeated_pick_discards_previous_locked_snapshot():
    bridge, _ = make_bridge()
    lock_first_post_pick_grasp(bridge)
    previous = bridge.latest_transform

    bridge.on_task_command(String(data="PICK"))

    assert bridge.latest_transform is None
    assert previous is not bridge.latest_transform
    assert bridge.pick_freeze_requested
    assert not bridge.pick_snapshot_active
