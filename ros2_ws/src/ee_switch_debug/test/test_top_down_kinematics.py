from math import pi

import numpy as np

from ee_switch_debug.top_down_kinematics import (
    apply_inward_radial_offset,
    build_group_blended_approach,
    build_group_sequential_approach,
    forward_kinematics,
    grasp_marker_rotation_to_ee_rotation,
    interpolate_rotation,
    plan_blended_top_down_sequence,
    plan_fixed_orientation_top_down_sequence,
    plan_top_down_sequence,
    rotation_distance,
    solve_joint23_rho_z,
    solve_pose_ik,
)
from ee_switch_debug.group_blended_trajectory import (
    sample_group_blended_trajectory,
)


HOME = np.array([0.0, -1.5708, 2.0944, -0.5236, 1.5708, 0.0])
LOWER = np.array([-0.6981, -1.5708, -1.0472, -pi, -pi, -pi])
UPPER = np.array([0.6981, 1.2217, 2.7925, pi, pi, pi])
LIVE_HOME = np.array([-0.0006, -1.5766, 2.1063, -0.5303, 1.5710, 0.0])
LIVE_GRASP_POSITION = np.array([0.399, -0.194, 0.295])
LIVE_MARKER_ROTATION = np.array(
    [
        [0.002, 0.999, -0.042],
        [0.002, -0.042, -0.999],
        [-1.000, 0.002, -0.002],
    ]
)


def test_forward_kinematics_matches_live_home_tf():
    position, rotation = forward_kinematics(HOME)

    assert np.allclose(position, [0.1065, -0.1130, 0.4765], atol=7e-4)
    assert np.allclose(
        rotation,
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        atol=4e-3,
    )


def test_grasp_marker_axes_are_converted_to_actual_omy_ee_axes():
    marker_rotation = np.eye(3)

    ee_rotation = grasp_marker_rotation_to_ee_rotation(marker_rotation)

    # Marker +X is palm-to-TCP approach; OMY palm-to-TCP is EE -Y.
    assert np.allclose(-ee_rotation[:, 1], marker_rotation[:, 0])
    # Marker +Y and OMY EE +X are both gripper closing directions.
    assert np.allclose(ee_rotation[:, 0], marker_rotation[:, 1])


def test_pose_ik_reconstructs_a_reachable_six_joint_pose():
    expected_joints = np.array([0.18, -1.12, 1.72, -0.61, 1.34, 0.27])
    target_position, target_rotation = forward_kinematics(expected_joints)

    result = solve_pose_ik(
        target_position,
        target_rotation,
        seed=HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
    )

    solved_position, solved_rotation = forward_kinematics(result.joints)
    assert result.success, result.message
    assert np.linalg.norm(target_position - solved_position) < 0.004
    assert rotation_distance(target_rotation, solved_rotation) < 0.04


def test_top_down_plan_places_pregrasp_ten_centimetres_above_grasp():
    grasp_joints = np.array([0.12, -1.02, 1.62, -0.60, 1.45, -0.16])
    grasp_position, grasp_rotation = forward_kinematics(grasp_joints)

    plan = plan_top_down_sequence(
        grasp_position,
        grasp_rotation,
        current_joints=HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        waypoint_spacing=0.02,
    )

    pregrasp_position, pregrasp_rotation = forward_kinematics(plan.pregrasp_joints)
    assert plan.success, plan.message
    assert np.allclose(
        pregrasp_position,
        grasp_position + np.array([0.0, 0.0, 0.10]),
        atol=0.005,
    )
    assert rotation_distance(grasp_rotation, pregrasp_rotation) < 0.04
    assert len(plan.descent_waypoints) == 5


def test_descent_waypoints_follow_world_z_without_changing_orientation():
    grasp_joints = np.array([-0.10, -0.95, 1.58, -0.63, 1.38, 0.22])
    grasp_position, grasp_rotation = forward_kinematics(grasp_joints)

    plan = plan_top_down_sequence(
        grasp_position,
        grasp_rotation,
        current_joints=HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        waypoint_spacing=0.02,
    )

    assert plan.success, plan.message
    positions = []
    for joints in plan.descent_waypoints:
        position, rotation = forward_kinematics(joints)
        positions.append(position)
        assert np.linalg.norm(position[:2] - grasp_position[:2]) < 0.005
        assert rotation_distance(grasp_rotation, rotation) < 0.04
    assert all(positions[index][2] > positions[index + 1][2] for index in range(4))
    assert abs(positions[-1][2] - grasp_position[2]) < 0.005


def test_unreachable_top_down_target_returns_failure_without_waypoints():
    plan = plan_top_down_sequence(
        [2.0, 0.0, 2.0],
        np.eye(3),
        current_joints=HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        waypoint_spacing=0.02,
    )

    assert not plan.success
    assert plan.descent_waypoints == []


def test_inward_radial_offset_moves_toward_link0_without_changing_height():
    compensated = apply_inward_radial_offset(LIVE_GRASP_POSITION, 0.025)

    assert np.isclose(np.linalg.norm(LIVE_GRASP_POSITION[:2] - compensated[:2]), 0.025)
    assert np.linalg.norm(compensated[:2]) < np.linalg.norm(LIVE_GRASP_POSITION[:2])
    assert compensated[2] == LIVE_GRASP_POSITION[2]


def test_rotation_interpolation_has_exact_endpoints_and_valid_midpoint():
    _, start_rotation = forward_kinematics(LIVE_HOME)
    end_rotation = grasp_marker_rotation_to_ee_rotation(LIVE_MARKER_ROTATION)

    assert np.allclose(interpolate_rotation(start_rotation, end_rotation, 0.0), start_rotation)
    assert np.allclose(interpolate_rotation(start_rotation, end_rotation, 1.0), end_rotation)
    midpoint = interpolate_rotation(start_rotation, end_rotation, 0.5)
    assert np.allclose(midpoint.T @ midpoint, np.eye(3), atol=1e-8)
    assert np.isclose(np.linalg.det(midpoint), 1.0, atol=1e-8)


def test_live_current_distance_target_has_safe_blended_plan():
    grasp_rotation = grasp_marker_rotation_to_ee_rotation(LIVE_MARKER_ROTATION)
    compensated = apply_inward_radial_offset(LIVE_GRASP_POSITION, 0.025)

    plan = plan_blended_top_down_sequence(
        LIVE_GRASP_POSITION,
        grasp_rotation,
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        waypoint_spacing=0.01,
        approach_joint_step=0.12,
        minimum_approach_clearance=0.08,
    )

    assert plan.success, plan.message
    assert plan.approach_waypoints
    assert len(plan.descent_waypoints) == 10
    for joints in plan.approach_waypoints:
        position, _ = forward_kinematics(joints)
        assert np.all(joints >= LOWER)
        assert np.all(joints <= UPPER)
        assert position[2] >= compensated[2] + 0.08

    first_position, first_rotation = forward_kinematics(plan.descent_waypoints[0])
    final_position, final_rotation = forward_kinematics(plan.descent_waypoints[-1])
    _, start_rotation = forward_kinematics(LIVE_HOME)
    assert first_position[2] < compensated[2] + 0.10
    assert rotation_distance(start_rotation, first_rotation) > 0.01
    assert rotation_distance(grasp_rotation, first_rotation) > 0.01
    assert np.linalg.norm(final_position - compensated) <= 0.005
    assert rotation_distance(grasp_rotation, final_rotation) <= 0.03


def test_live_target_selects_one_reachable_fixed_orientation_for_descent():
    requested_rotation = grasp_marker_rotation_to_ee_rotation(LIVE_MARKER_ROTATION)

    plan = plan_fixed_orientation_top_down_sequence(
        LIVE_GRASP_POSITION,
        requested_rotation,
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        minimum_orientation_fraction=0.50,
        orientation_search_steps=20,
        maximum_xy_deviation=0.015,
        maximum_orientation_deviation=0.035,
    )

    assert plan.success, plan.message
    assert plan.orientation_fraction >= 0.75
    assert plan.grasp_joints.shape == (6,)
    pregrasp_position, pregrasp_rotation = forward_kinematics(plan.pregrasp_joints)
    grasp_position, grasp_rotation = forward_kinematics(plan.grasp_joints)
    assert pregrasp_position[2] > grasp_position[2]
    assert rotation_distance(pregrasp_rotation, grasp_rotation) <= 0.035
    assert rotation_distance(plan.selected_rotation, pregrasp_rotation) <= 0.035
    assert rotation_distance(plan.selected_rotation, grasp_rotation) <= 0.035


def test_fixed_orientation_joint_descent_is_monotonic_and_nearly_vertical():
    requested_rotation = grasp_marker_rotation_to_ee_rotation(LIVE_MARKER_ROTATION)
    plan = plan_fixed_orientation_top_down_sequence(
        LIVE_GRASP_POSITION,
        requested_rotation,
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        minimum_orientation_fraction=0.50,
        orientation_search_steps=20,
        maximum_xy_deviation=0.015,
        maximum_orientation_deviation=0.035,
    )

    assert plan.success, plan.message
    samples = [
        plan.pregrasp_joints
        + fraction * (plan.grasp_joints - plan.pregrasp_joints)
        for fraction in np.linspace(0.0, 1.0, 21)
    ]
    poses = [forward_kinematics(joints) for joints in samples]
    positions = np.asarray([position for position, _ in poses])
    rotations = [rotation for _, rotation in poses]
    final_xy = positions[-1, :2]
    assert np.all(np.diff(positions[:, 2]) < 0.0)
    assert np.max(np.linalg.norm(positions[:, :2] - final_xy, axis=1)) <= 0.015
    assert max(
        rotation_distance(plan.selected_rotation, rotation)
        for rotation in rotations
    ) <= 0.035


def test_fixed_orientation_plan_fails_when_required_fraction_is_unreachable():
    requested_rotation = grasp_marker_rotation_to_ee_rotation(LIVE_MARKER_ROTATION)

    plan = plan_fixed_orientation_top_down_sequence(
        LIVE_GRASP_POSITION,
        requested_rotation,
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        minimum_orientation_fraction=0.90,
        orientation_search_steps=20,
    )

    assert not plan.success
    assert "common fixed orientation" in plan.message


def test_joint23_solver_keeps_yaw_and_wrist_fixed():
    seed = np.array([0.2, -1.2, 1.8, -0.4, 1.0, -0.7])
    target_joints = np.array([0.2, -0.8, 1.3, -0.4, 1.0, -0.7])
    target_position, _ = forward_kinematics(target_joints)

    result = solve_joint23_rho_z(
        target_position,
        seed,
        [-3.14] * 6,
        [3.14] * 6,
    )

    assert result.success, result.message
    assert np.allclose(
        result.joints[[0, 3, 4, 5]],
        seed[[0, 3, 4, 5]],
    )
    solved_position, _ = forward_kinematics(result.joints)
    assert abs(
        np.hypot(*solved_position[:2]) - np.hypot(*target_position[:2])
    ) < 0.005
    assert abs(solved_position[2] - target_position[2]) < 0.005


def test_joint23_solver_rejects_invalid_inactive_joint_without_changing_it():
    seed = np.array([3.5, -1.2, 1.8, -0.4, 1.0, -0.7])

    result = solve_joint23_rho_z(
        [0.3, 0.0, 0.4],
        seed,
        [-3.14] * 6,
        [3.14] * 6,
    )

    assert not result.success
    assert result.joints[0] == seed[0]
    assert np.allclose(result.joints[3:], seed[3:])


def test_joint23_solver_rejects_unreachable_target_within_active_limits():
    result = solve_joint23_rho_z(
        [5.0, 0.0, 5.0],
        [0.0] * 6,
        [-1.0] * 6,
        [1.0] * 6,
    )

    assert not result.success
    assert np.all(result.joints[1:3] >= -1.0)
    assert np.all(result.joints[1:3] <= 1.0)


def test_group_sequential_approach_assigns_only_requested_joint_groups():
    requested_rotation = grasp_marker_rotation_to_ee_rotation(
        LIVE_MARKER_ROTATION
    )
    fixed_plan = plan_fixed_orientation_top_down_sequence(
        LIVE_GRASP_POSITION,
        requested_rotation,
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        minimum_orientation_fraction=0.50,
        orientation_search_steps=20,
        maximum_xy_deviation=0.015,
        maximum_orientation_deviation=0.035,
    )
    assert fixed_plan.success, fixed_plan.message

    plan = build_group_sequential_approach(
        fixed_plan,
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        minimum_clearance=0.08,
    )

    assert plan.success, plan.message
    assert plan.yaw_joints[0] == plan.pregrasp_joints[0]
    assert np.allclose(plan.yaw_joints[1:], LIVE_HOME[1:])
    assert plan.arm_joints[0] == plan.yaw_joints[0]
    assert np.allclose(plan.arm_joints[3:], LIVE_HOME[3:])
    assert plan.pregrasp_joints[0] == plan.yaw_joints[0]
    assert not np.allclose(plan.arm_joints[1:3], plan.pregrasp_joints[1:3])


def _live_fixed_plan():
    return plan_fixed_orientation_top_down_sequence(
        LIVE_GRASP_POSITION,
        grasp_marker_rotation_to_ee_rotation(LIVE_MARKER_ROTATION),
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        minimum_orientation_fraction=0.50,
        orientation_search_steps=20,
        maximum_xy_deviation=0.015,
        maximum_orientation_deviation=0.035,
    )


def _live_group_blended_plan(minimum_clearance=0.08):
    return build_group_blended_approach(
        _live_fixed_plan(),
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        velocity_limits=[0.70, 1.05, 1.20, 0.95, 0.95, 0.95],
        acceleration_limit=1.6,
        minimum_duration=0.6,
        minimum_clearance=minimum_clearance,
    )


def test_group_blended_approach_accepts_live_start_and_ends_at_pregrasp():
    plan = _live_group_blended_plan()

    assert plan.success, plan.message
    assert plan.blended_approach is not None
    start, _ = sample_group_blended_trajectory(plan.blended_approach, 0.0)
    finish, complete = sample_group_blended_trajectory(
        plan.blended_approach,
        plan.blended_approach.duration,
    )
    assert np.allclose(start, LIVE_HOME)
    assert complete
    assert np.allclose(finish, plan.pregrasp_joints)


def test_group_blended_approach_rejects_impossible_clearance():
    plan = _live_group_blended_plan(minimum_clearance=10.0)

    assert not plan.success
    assert "clearance" in plan.message


def test_group_blended_live_path_preserves_ownership_and_clearance():
    plan = _live_group_blended_plan()

    assert plan.success, plan.message
    assert np.allclose(plan.yaw_joints[1:], LIVE_HOME[1:])
    assert np.allclose(
        plan.arm_joints[[0, 3, 4, 5]],
        plan.yaw_joints[[0, 3, 4, 5]],
    )
    grasp, _ = forward_kinematics(plan.grasp_joints)
    for elapsed in np.linspace(0.0, plan.blended_approach.duration, 101):
        joints, _ = sample_group_blended_trajectory(
            plan.blended_approach,
            elapsed,
        )
        position, _ = forward_kinematics(joints)
        assert position[2] >= grasp[2] + 0.08 - 1e-9
