import numpy as np

from ee_switch_debug.semantic_joint_trajectory import (
    compact_smoothstep,
    minimum_compact_duration,
    minimum_quintic_duration,
    quintic_smoothstep,
    sample_compact_joint_positions,
    sample_quintic_joint_positions,
)


def test_quintic_smoothstep_has_exact_endpoints_and_midpoint():
    assert quintic_smoothstep(-1.0) == 0.0
    assert quintic_smoothstep(0.0) == 0.0
    assert quintic_smoothstep(0.5) == 0.5
    assert quintic_smoothstep(1.0) == 1.0
    assert quintic_smoothstep(2.0) == 1.0


def test_quintic_smoothstep_starts_and_ends_with_zero_velocity():
    epsilon = 1e-5
    start_slope = (quintic_smoothstep(epsilon) - quintic_smoothstep(0.0)) / epsilon
    end_slope = (quintic_smoothstep(1.0) - quintic_smoothstep(1.0 - epsilon)) / epsilon

    assert abs(start_slope) < 1e-6
    assert abs(end_slope) < 1e-6


def test_duration_respects_quintic_velocity_and_acceleration_bounds():
    start = np.zeros(3)
    goal = np.array([1.0, -0.4, 0.2])
    velocity_limits = np.array([0.5, 1.0, 2.0])

    duration = minimum_quintic_duration(
        start,
        goal,
        velocity_limits,
        acceleration_limit=1.2,
        minimum_duration=0.8,
    )

    expected_velocity_duration = 1.875 * 1.0 / 0.5
    expected_acceleration_duration = np.sqrt(5.7736 * 1.0 / 1.2)
    assert duration >= expected_velocity_duration
    assert duration >= expected_acceleration_duration
    assert duration >= 0.8


def test_joint_trajectory_samples_exact_endpoints_and_completion():
    start = np.array([0.0, -1.0, 2.0])
    goal = np.array([1.0, 0.0, -1.0])

    before, complete_before = sample_quintic_joint_positions(start, goal, -1.0, 4.0)
    midpoint, complete_midpoint = sample_quintic_joint_positions(start, goal, 2.0, 4.0)
    after, complete_after = sample_quintic_joint_positions(start, goal, 8.0, 4.0)

    assert np.array_equal(before, start)
    assert not complete_before
    assert np.allclose(midpoint, 0.5 * (start + goal))
    assert not complete_midpoint
    assert np.array_equal(after, goal)
    assert complete_after


def test_compact_smoothstep_has_exact_endpoints_and_moves_early():
    assert compact_smoothstep(-1.0) == 0.0
    assert compact_smoothstep(0.0) == 0.0
    assert compact_smoothstep(0.5) == 0.5
    assert compact_smoothstep(1.0) == 1.0
    assert compact_smoothstep(2.0) == 1.0
    assert compact_smoothstep(0.2) > quintic_smoothstep(0.2)


def test_compact_smoothstep_has_zero_endpoint_velocity():
    epsilon = 1e-6
    start_slope = (
        compact_smoothstep(epsilon) - compact_smoothstep(0.0)
    ) / epsilon
    end_slope = (
        compact_smoothstep(1.0) - compact_smoothstep(1.0 - epsilon)
    ) / epsilon

    assert abs(start_slope) < 4e-6
    assert abs(end_slope) < 4e-6


def test_compact_duration_respects_exact_velocity_and_acceleration_bounds():
    duration = minimum_compact_duration(
        np.zeros(3),
        np.array([1.0, -0.4, 0.2]),
        np.array([0.5, 1.0, 2.0]),
        acceleration_limit=1.2,
        minimum_duration=0.8,
    )

    assert duration >= 1.5 * 1.0 / 0.5
    assert duration >= np.sqrt(6.0 * 1.0 / 1.2)
    assert duration >= 0.8


def test_compact_joint_trajectory_samples_exact_endpoints_and_completion():
    start = np.array([0.0, -1.0, 2.0])
    goal = np.array([1.0, 0.0, -1.0])

    before, complete_before = sample_compact_joint_positions(
        start, goal, -1.0, 4.0
    )
    midpoint, complete_midpoint = sample_compact_joint_positions(
        start, goal, 2.0, 4.0
    )
    after, complete_after = sample_compact_joint_positions(
        start, goal, 8.0, 4.0
    )

    assert np.array_equal(before, start)
    assert not complete_before
    assert np.allclose(midpoint, 0.5 * (start + goal))
    assert not complete_midpoint
    assert np.array_equal(after, goal)
    assert complete_after
