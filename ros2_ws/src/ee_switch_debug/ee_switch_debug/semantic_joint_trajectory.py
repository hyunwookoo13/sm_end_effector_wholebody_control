"""Pure time scaling for smooth semantic arm motion segments."""

from typing import Sequence

import numpy as np


QUINTIC_MAX_NORMALIZED_VELOCITY = 1.875
QUINTIC_MAX_NORMALIZED_ACCELERATION = 5.7736
COMPACT_MAX_NORMALIZED_VELOCITY = 1.5
COMPACT_MAX_NORMALIZED_ACCELERATION = 6.0


def quintic_smoothstep(fraction: float) -> float:
    """Return minimum-jerk position scaling with zero endpoint derivatives."""
    value = float(np.clip(fraction, 0.0, 1.0))
    return value**3 * (10.0 - 15.0 * value + 6.0 * value**2)


def compact_smoothstep(fraction: float) -> float:
    """Return cubic position scaling with shorter low-motion endpoint regions."""
    value = float(np.clip(fraction, 0.0, 1.0))
    return value**2 * (3.0 - 2.0 * value)


def minimum_quintic_duration(
    start: Sequence[float],
    goal: Sequence[float],
    velocity_limits: Sequence[float],
    acceleration_limit: float,
    minimum_duration: float,
) -> float:
    """Choose a duration that respects quintic peak velocity and acceleration."""
    start_array = np.asarray(start, dtype=float)
    goal_array = np.asarray(goal, dtype=float)
    limits = np.asarray(velocity_limits, dtype=float)
    if start_array.shape != goal_array.shape or start_array.shape != limits.shape:
        raise ValueError("start, goal, and velocity limits must have matching shapes")
    if np.any(limits <= 0.0):
        raise ValueError("velocity limits must be positive")
    acceleration = float(acceleration_limit)
    if acceleration <= 0.0:
        raise ValueError("acceleration limit must be positive")

    delta = np.abs(goal_array - start_array)
    velocity_duration = float(
        np.max(QUINTIC_MAX_NORMALIZED_VELOCITY * delta / limits)
    )
    acceleration_duration = float(
        np.max(np.sqrt(QUINTIC_MAX_NORMALIZED_ACCELERATION * delta / acceleration))
    )
    return max(float(minimum_duration), velocity_duration, acceleration_duration)


def minimum_compact_duration(
    start: Sequence[float],
    goal: Sequence[float],
    velocity_limits: Sequence[float],
    acceleration_limit: float,
    minimum_duration: float,
) -> float:
    """Choose a duration that respects compact-profile motion bounds."""
    start_array = np.asarray(start, dtype=float)
    goal_array = np.asarray(goal, dtype=float)
    limits = np.asarray(velocity_limits, dtype=float)
    if start_array.shape != goal_array.shape or start_array.shape != limits.shape:
        raise ValueError("start, goal, and velocity limits must have matching shapes")
    if np.any(limits <= 0.0):
        raise ValueError("velocity limits must be positive")
    acceleration = float(acceleration_limit)
    if acceleration <= 0.0:
        raise ValueError("acceleration limit must be positive")

    delta = np.abs(goal_array - start_array)
    velocity_duration = float(
        np.max(COMPACT_MAX_NORMALIZED_VELOCITY * delta / limits)
    )
    acceleration_duration = float(
        np.max(np.sqrt(COMPACT_MAX_NORMALIZED_ACCELERATION * delta / acceleration))
    )
    return max(float(minimum_duration), velocity_duration, acceleration_duration)


def sample_quintic_joint_positions(
    start: Sequence[float],
    goal: Sequence[float],
    elapsed: float,
    duration: float,
) -> tuple[np.ndarray, bool]:
    """Sample one joint segment and report whether planned time has elapsed."""
    start_array = np.asarray(start, dtype=float)
    goal_array = np.asarray(goal, dtype=float)
    if start_array.shape != goal_array.shape:
        raise ValueError("start and goal must have matching shapes")
    safe_duration = max(float(duration), 1e-9)
    complete = float(elapsed) >= safe_duration
    fraction = float(np.clip(float(elapsed) / safe_duration, 0.0, 1.0))
    scale = quintic_smoothstep(fraction)
    return start_array + scale * (goal_array - start_array), complete


def sample_compact_joint_positions(
    start: Sequence[float],
    goal: Sequence[float],
    elapsed: float,
    duration: float,
) -> tuple[np.ndarray, bool]:
    """Sample one compact joint segment and report planned completion."""
    start_array = np.asarray(start, dtype=float)
    goal_array = np.asarray(goal, dtype=float)
    if start_array.shape != goal_array.shape:
        raise ValueError("start and goal must have matching shapes")
    safe_duration = max(float(duration), 1e-9)
    complete = float(elapsed) >= safe_duration
    fraction = float(np.clip(float(elapsed) / safe_duration, 0.0, 1.0))
    scale = compact_smoothstep(fraction)
    return start_array + scale * (goal_array - start_array), complete
