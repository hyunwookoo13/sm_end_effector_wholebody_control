import numpy as np

from ee_switch_debug.group_blended_trajectory import (
    compose_group_blended_path,
    plan_group_blended_trajectory,
    sample_group_blended_trajectory,
)


START = np.array([0.0, -1.3, 1.9, -0.5, 1.4, 0.0])
YAW = np.array([0.3, -1.3, 1.9, -0.5, 1.4, 0.0])
ARM = np.array([0.3, -0.8, 1.5, -0.5, 1.4, 0.0])
PREGRASP = np.array([0.3, -0.65, 1.35, -1.0, 1.8, 0.4])


def test_composed_path_has_exact_endpoints():
    start, _, _ = compose_group_blended_path(START, YAW, ARM, PREGRASP, 0.0)
    finish, _, _ = compose_group_blended_path(
        START, YAW, ARM, PREGRASP, 1.0
    )

    assert np.allclose(start, START)
    assert np.allclose(finish, PREGRASP)


def test_endpoint_perturbations_change_only_owned_joints():
    baseline, _, _ = compose_group_blended_path(
        START, YAW, ARM, PREGRASP, 0.6
    )
    changed_yaw = YAW.copy()
    changed_arm_for_yaw = ARM.copy()
    changed_pregrasp_for_yaw = PREGRASP.copy()
    changed_yaw[0] += 0.1
    changed_arm_for_yaw[0] += 0.1
    changed_pregrasp_for_yaw[0] += 0.1
    changed, _, _ = compose_group_blended_path(
        START,
        changed_yaw,
        changed_arm_for_yaw,
        changed_pregrasp_for_yaw,
        0.6,
    )
    assert np.flatnonzero(np.abs(changed - baseline) > 1e-9).tolist() == [0]

    changed_arm = ARM.copy()
    changed_arm[1:3] += [0.1, -0.1]
    changed, _, _ = compose_group_blended_path(
        START, YAW, changed_arm, PREGRASP, 0.6
    )
    assert np.flatnonzero(np.abs(changed - baseline) > 1e-9).tolist() == [1, 2]


def test_joint23_compensation_overlaps_overhead_motion():
    progress = 0.60
    blended, _, _ = compose_group_blended_path(
        START, YAW, ARM, PREGRASP, progress
    )
    u = progress / 0.75
    arm_scale = u * u * (3.0 - 2.0 * u)
    arm_only = START[1:3] + arm_scale * (ARM[1:3] - START[1:3])

    assert not np.allclose(blended[1:3], arm_only)


def test_motion_has_no_internal_full_robot_stop():
    for progress in np.linspace(0.01, 0.99, 99):
        _, derivative, _ = compose_group_blended_path(
            START, YAW, ARM, PREGRASP, progress
        )
        assert np.linalg.norm(derivative) > 1e-6


def test_duration_respects_velocity_and_acceleration_limits():
    limits = np.array([0.70, 1.05, 1.20, 0.95, 0.95, 0.95])
    trajectory = plan_group_blended_trajectory(
        START,
        YAW,
        ARM,
        PREGRASP,
        limits,
        acceleration_limit=1.6,
        minimum_duration=0.6,
    )

    for progress in np.linspace(0.0, 1.0, 1001):
        _, first, second = compose_group_blended_path(
            START, YAW, ARM, PREGRASP, progress
        )
        assert np.all(
            np.abs(first) / trajectory.duration <= limits + 1e-9
        )
        assert np.all(
            np.abs(second) / trajectory.duration**2 <= 1.6 + 1e-9
        )
    finish, complete = sample_group_blended_trajectory(
        trajectory, trajectory.duration
    )
    assert complete
    assert np.allclose(finish, PREGRASP)
