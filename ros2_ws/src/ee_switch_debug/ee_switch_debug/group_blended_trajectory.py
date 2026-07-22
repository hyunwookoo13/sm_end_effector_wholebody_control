"""Pure composition and time scaling for joint-group blended approaches."""

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class GroupBlendedTrajectory:
    start: np.ndarray
    yaw: np.ndarray
    arm: np.ndarray
    pregrasp: np.ndarray
    duration: float
    arm_completion: float = 0.75
    compensation_start: float = 0.10


def _compact_window(
    progress: float,
    start: float,
    end: float,
) -> tuple[float, float, float]:
    """Return compact position and derivatives with respect to global progress."""
    if not 0.0 <= start < end <= 1.0:
        raise ValueError("compact window must satisfy 0 <= start < end <= 1")
    value = float(progress)
    if value < start:
        return 0.0, 0.0, 0.0
    if value > end:
        return 1.0, 0.0, 0.0
    width = end - start
    local = (value - start) / width
    return (
        local * local * (3.0 - 2.0 * local),
        (6.0 * local - 6.0 * local * local) / width,
        (6.0 - 12.0 * local) / (width * width),
    )


def _group_endpoint_arrays(
    start: Sequence[float],
    yaw: Sequence[float],
    arm: Sequence[float],
    pregrasp: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    arrays = tuple(
        np.asarray(value, dtype=float).copy()
        for value in (start, yaw, arm, pregrasp)
    )
    if any(value.shape != (6,) for value in arrays):
        raise ValueError("all group endpoints must contain six joints")
    if not all(np.all(np.isfinite(value)) for value in arrays):
        raise ValueError("group endpoints must be finite")
    start_array, yaw_array, arm_array, pregrasp_array = arrays
    if not np.allclose(yaw_array[1:], start_array[1:]):
        raise ValueError("yaw endpoint changed a non-Joint1 value")
    inactive_arm = [0, 3, 4, 5]
    if not np.allclose(
        arm_array[inactive_arm],
        yaw_array[inactive_arm],
    ):
        raise ValueError("arm endpoint changed a joint outside Joint2/3")
    if not np.isclose(pregrasp_array[0], yaw_array[0]):
        raise ValueError("pregrasp changed Joint1 after yaw planning")
    return start_array, yaw_array, arm_array, pregrasp_array


def compose_group_blended_path(
    start: Sequence[float],
    yaw: Sequence[float],
    arm: Sequence[float],
    pregrasp: Sequence[float],
    progress: float,
    arm_completion: float = 0.75,
    compensation_start: float = 0.10,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compose owned joint-group contributions on one normalized timeline."""
    start_array, yaw_array, arm_array, pregrasp_array = _group_endpoint_arrays(
        start,
        yaw,
        arm,
        pregrasp,
    )
    whole = _compact_window(progress, 0.0, 1.0)
    overhead = _compact_window(progress, 0.0, float(arm_completion))
    compensation = _compact_window(
        progress,
        float(compensation_start),
        1.0,
    )
    position = start_array.copy()
    first = np.zeros(6, dtype=float)
    second = np.zeros(6, dtype=float)

    for index in (0, 3, 4, 5):
        goal = yaw_array[index] if index == 0 else pregrasp_array[index]
        delta = goal - start_array[index]
        position[index] += whole[0] * delta
        first[index] += whole[1] * delta
        second[index] += whole[2] * delta

    for index in (1, 2):
        overhead_delta = arm_array[index] - start_array[index]
        compensation_delta = pregrasp_array[index] - arm_array[index]
        position[index] += (
            overhead[0] * overhead_delta
            + compensation[0] * compensation_delta
        )
        first[index] += (
            overhead[1] * overhead_delta
            + compensation[1] * compensation_delta
        )
        second[index] += (
            overhead[2] * overhead_delta
            + compensation[2] * compensation_delta
        )
    return position, first, second


def plan_group_blended_trajectory(
    start: Sequence[float],
    yaw: Sequence[float],
    arm: Sequence[float],
    pregrasp: Sequence[float],
    velocity_limits: Sequence[float],
    acceleration_limit: float,
    minimum_duration: float,
    derivative_samples: int = 1001,
) -> GroupBlendedTrajectory:
    """Time-scale the composed path against joint velocity and acceleration."""
    endpoints = _group_endpoint_arrays(start, yaw, arm, pregrasp)
    limits = np.asarray(velocity_limits, dtype=float)
    if limits.shape != (6,) or np.any(~np.isfinite(limits)):
        raise ValueError("velocity limits must contain six finite values")
    if np.any(limits <= 0.0):
        raise ValueError("velocity limits must be positive")
    acceleration = float(acceleration_limit)
    if not np.isfinite(acceleration) or acceleration <= 0.0:
        raise ValueError("acceleration limit must be positive")

    sample_count = max(3, int(derivative_samples))
    velocity_duration = 0.0
    acceleration_duration = 0.0
    for progress in np.linspace(0.0, 1.0, sample_count):
        _, first, second = compose_group_blended_path(
            *endpoints,
            progress,
        )
        velocity_duration = max(
            velocity_duration,
            float(np.max(np.abs(first) / limits)),
        )
        acceleration_duration = max(
            acceleration_duration,
            float(np.sqrt(np.max(np.abs(second)) / acceleration)),
        )
    duration = max(
        max(0.0, float(minimum_duration)),
        velocity_duration,
        acceleration_duration,
        1e-6,
    )
    return GroupBlendedTrajectory(
        start=endpoints[0],
        yaw=endpoints[1],
        arm=endpoints[2],
        pregrasp=endpoints[3],
        duration=duration,
    )


def sample_group_blended_trajectory(
    trajectory: GroupBlendedTrajectory,
    elapsed: float,
) -> tuple[np.ndarray, bool]:
    """Sample one immutable trajectory and report planned completion."""
    duration = max(float(trajectory.duration), 1e-9)
    complete = float(elapsed) >= duration
    progress = float(np.clip(float(elapsed) / duration, 0.0, 1.0))
    position, _, _ = compose_group_blended_path(
        trajectory.start,
        trajectory.yaw,
        trajectory.arm,
        trajectory.pregrasp,
        progress,
        trajectory.arm_completion,
        trajectory.compensation_start,
    )
    return position, complete
