from math import cos, pi, sin
from pathlib import Path

import ee_switch_debug.arm_yaw_rho_z_position_controller as controller
import rclpy

from ee_switch_debug.arm_yaw_rho_z_position_controller import (
    compute_wrist_targets_from_orientation,
    is_wrist_aligned_for_phase,
)


def multiply(A, B):
    return [
        [sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def rot_y(theta):
    return [
        [cos(theta), 0.0, sin(theta)],
        [0.0, 1.0, 0.0],
        [-sin(theta), 0.0, cos(theta)],
    ]


def rot_z(theta):
    return [
        [cos(theta), -sin(theta), 0.0],
        [sin(theta), cos(theta), 0.0],
        [0.0, 0.0, 1.0],
    ]


def yzy(q4, q5, q6):
    return multiply(multiply(rot_y(q4), rot_z(q5)), rot_y(q6))


def object_target_rotation(closing_angle):
    desired_closing = [cos(closing_angle), 0.0, -sin(closing_angle)]
    object_major = [sin(closing_angle), 0.0, cos(closing_angle)]
    approach_axis = [0.0, 1.0, 0.0]
    return [
        [object_major[0], desired_closing[0], approach_axis[0]],
        [object_major[1], desired_closing[1], approach_axis[1]],
        [object_major[2], desired_closing[2], approach_axis[2]],
    ]


def test_pick_approach_clearance_keeps_final_grasp_height_unchanged():
    approach, descend_target = controller.pick_z_offsets(
        grasp_offset_z=0.03,
        descend_depth=0.07,
        approach_clearance_z=0.03,
    )

    assert approach == 0.06
    assert abs(descend_target + 0.04) < 1e-9


def test_link6_axis_alignment_rotates_90_degrees_toward_object_short_axis():
    target, delta, valid = controller.compute_link6_axis_target(
        target_rotation=object_target_rotation(pi / 2.0),
        ee_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        current_q6=0.0,
        lower_limit=-pi,
        upper_limit=pi,
        link6_axis="y",
        gripper_closing_axis="x",
        target_closing_axis="y",
    )

    assert valid
    assert abs(abs(delta) - pi / 2.0) < 1e-6
    assert abs(abs(target) - pi / 2.0) < 1e-6


def test_link6_axis_alignment_keeps_continuous_diagonal_angle():
    expected = 0.63
    target, delta, valid = controller.compute_link6_axis_target(
        target_rotation=object_target_rotation(expected),
        ee_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        current_q6=0.0,
        lower_limit=-pi,
        upper_limit=pi,
        link6_axis="y",
        gripper_closing_axis="x",
        target_closing_axis="y",
    )

    assert valid
    assert abs(delta - expected) < 1e-6
    assert abs(target - expected) < 1e-6


def test_link6_axis_alignment_treats_reversed_object_axis_as_equivalent():
    expected = 0.47
    target_rotation = object_target_rotation(expected)
    reversed_rotation = [
        [-row[0], -row[1], row[2]]
        for row in target_rotation
    ]

    forward = controller.compute_link6_axis_target(
        target_rotation=target_rotation,
        ee_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        current_q6=0.0,
        lower_limit=-pi,
        upper_limit=pi,
        link6_axis="y",
        gripper_closing_axis="x",
        target_closing_axis="y",
    )
    reversed_axis = controller.compute_link6_axis_target(
        target_rotation=reversed_rotation,
        ee_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        current_q6=0.0,
        lower_limit=-pi,
        upper_limit=pi,
        link6_axis="y",
        gripper_closing_axis="x",
        target_closing_axis="y",
    )

    assert forward[2] and reversed_axis[2]
    assert abs(forward[0] - reversed_axis[0]) < 1e-6


def test_link6_axis_alignment_selects_exact_equivalent_inside_joint_limits():
    current_q6 = 2.9
    correction = 0.5
    target, delta, valid = controller.compute_link6_axis_target(
        target_rotation=object_target_rotation(correction),
        ee_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        current_q6=current_q6,
        lower_limit=-pi,
        upper_limit=pi,
        link6_axis="y",
        gripper_closing_axis="x",
        target_closing_axis="y",
    )

    assert valid
    assert abs(delta - correction) < 1e-6
    assert abs(target - (current_q6 + correction - pi)) < 1e-6


def test_link6_axis_alignment_reports_unreachable_target():
    target, _, valid = controller.compute_link6_axis_target(
        target_rotation=object_target_rotation(1.0),
        ee_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        current_q6=0.0,
        lower_limit=-0.2,
        upper_limit=0.2,
        link6_axis="y",
        gripper_closing_axis="x",
        target_closing_axis="y",
    )

    assert not valid
    assert abs(target - 0.2) < 1e-6


def test_link6_mode_uses_grasp_orientation_while_keeping_default_wrist_shape():
    fallback = [-0.5236, 1.5708, -1.5708]
    target_link6 = 0.42
    targets = compute_wrist_targets_from_orientation(
        target_rotation=yzy(0.2, 1.1, target_link6),
        link3_rotation=yzy(0.2, 1.1, 0.0),
        fallback_targets=fallback,
        joint_lower_limits=[-3.1416] * 6,
        joint_upper_limits=[3.1416] * 6,
        mode="link6",
        link6_offset=0.0,
    )

    assert targets[0] == fallback[0]
    assert targets[1] == fallback[1]
    assert abs(targets[2] - target_link6) < 1e-6


def test_full_mode_can_follow_all_wrist_angles_from_grasp_orientation():
    targets = compute_wrist_targets_from_orientation(
        target_rotation=yzy(-0.4, 1.2, 0.7),
        link3_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        fallback_targets=[0.0, 0.0, -1.5708],
        joint_lower_limits=[-3.1416] * 6,
        joint_upper_limits=[3.1416] * 6,
        mode="full",
        link6_offset=0.0,
    )

    assert abs(targets[0] - -0.4) < 1e-6
    assert abs(targets[1] - 1.2) < 1e-6
    assert abs(targets[2] - 0.7) < 1e-6


def test_off_mode_keeps_existing_fixed_wrist_targets():
    fallback = [-0.5236, 1.5708, -1.5708]
    targets = compute_wrist_targets_from_orientation(
        target_rotation=yzy(0.5, 1.0, 0.3),
        link3_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        fallback_targets=fallback,
        joint_lower_limits=[-3.1416] * 6,
        joint_upper_limits=[3.1416] * 6,
        mode="off",
        link6_offset=0.0,
    )

    assert targets == fallback


def test_pick_approach_requires_wrist_alignment_before_descend():
    assert not is_wrist_aligned_for_phase(
        task_mode="PICK",
        grasp_phase="APPROACH",
        wrist_errors=[0.0, 0.0, 0.4],
        tolerance=0.12,
        require_alignment=True,
    )
    assert is_wrist_aligned_for_phase(
        task_mode="PICK",
        grasp_phase="APPROACH",
        wrist_errors=[0.0, 0.0, 0.08],
        tolerance=0.12,
        require_alignment=True,
    )
    assert is_wrist_aligned_for_phase(
        task_mode="PLACE",
        grasp_phase="APPROACH",
        wrist_errors=[0.0, 0.0, 0.4],
        tolerance=0.12,
        require_alignment=True,
    )


def test_pick_approach_rejects_invalid_object_axis_alignment():
    assert not is_wrist_aligned_for_phase(
        task_mode="PICK",
        grasp_phase="APPROACH",
        wrist_errors=[0.0, 0.0, 0.0],
        tolerance=0.12,
        require_alignment=True,
        orientation_valid=False,
    )


def test_controller_axis_parameters_load_as_strings_from_yaml():
    config_path = Path(__file__).parents[1] / "config" / "arm_position_target_in.yaml"
    rclpy.init(args=["--ros-args", "--params-file", str(config_path)])
    node = None
    try:
        node = controller.ArmYawRhoZPositionController()
        assert node.grasp_link6_axis == "y"
        assert node.grasp_gripper_closing_axis == "x"
        assert node.grasp_target_closing_axis == "y"
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_transport_sequence_retracts_arm_and_keeps_gripper_closed():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.gripper_close_position = 0.8
    node.place_wrist_hold_positions = [0.1, 0.2, 0.3]
    node.previous_velocity = [1.0] * 6
    states = []
    node.set_control_state = states.append
    node.publish_task_state = lambda: None
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: None}
    )()

    node.start_transport_sequence()

    assert node.task_mode == "TRANSPORT"
    assert node.force_safety_pose
    assert node.grasp_phase == "HOLD"
    assert node.gripper_position == 0.8
    assert node.place_wrist_hold_positions is None
    assert node.previous_velocity == [0.0] * 6
    assert states == ["RETURN_HOME"]
