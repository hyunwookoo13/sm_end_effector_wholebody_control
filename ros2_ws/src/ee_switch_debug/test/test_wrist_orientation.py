from math import cos, sin

from ee_switch_debug.arm_yaw_rho_z_position_controller import (
    compute_wrist_targets_from_orientation,
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
