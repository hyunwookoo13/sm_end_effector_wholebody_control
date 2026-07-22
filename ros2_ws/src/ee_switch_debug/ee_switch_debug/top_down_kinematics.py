"""OMY-F3M kinematics used to plan a fixed top-down grasp sequence."""

from dataclasses import dataclass, field
from math import acos, atan2, ceil, cos, pi, sin
from typing import Sequence

import numpy as np


JOINT_AXES = ("z", "y", "y", "y", "z", "y")
JOINT_TRANSLATIONS = np.array(
    [
        [0.0, 0.0, 0.1715],
        [0.0, -0.1215, 0.0],
        [0.0, 0.0, 0.2470],
        [0.0, 0.1215, 0.2195],
        [0.0, -0.1130, 0.0],
        [0.0, 0.0, 0.1155],
    ],
    dtype=float,
)
TOOL_TRANSLATION = np.array([0.0, -0.244030948, 0.0], dtype=float)
GRASP_MARKER_TO_EE_ROTATION = np.array(
    [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    dtype=float,
)


@dataclass(frozen=True)
class IKResult:
    success: bool
    joints: np.ndarray
    position_error: float
    orientation_error: float
    iterations: int
    message: str = ""


@dataclass(frozen=True)
class TopDownPlan:
    success: bool
    pregrasp_joints: np.ndarray = field(
        default_factory=lambda: np.zeros(6, dtype=float)
    )
    approach_waypoints: list[np.ndarray] = field(default_factory=list)
    descent_waypoints: list[np.ndarray] = field(default_factory=list)
    grasp_joints: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=float))
    selected_rotation: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=float))
    orientation_fraction: float = 0.0
    message: str = ""


@dataclass(frozen=True)
class FixedOrientationCandidate:
    success: bool
    fraction: float = 0.0
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=float))
    pregrasp_joints: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=float))
    grasp_joints: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=float))
    message: str = ""


def _translation(vector: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, 3] = np.asarray(vector, dtype=float)
    return transform


def _rotation(axis: str, angle: float) -> np.ndarray:
    c = cos(float(angle))
    s = sin(float(angle))
    if axis == "y":
        matrix = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    elif axis == "z":
        matrix = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    else:
        raise ValueError(f"Unsupported joint axis: {axis}")
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = matrix
    return transform


def forward_kinematics(joints: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """Return link0-frame position and rotation of end_effector_link."""
    values = np.asarray(joints, dtype=float)
    if values.shape != (6,):
        raise ValueError("Expected exactly six arm joint positions")

    transform = np.eye(4, dtype=float)
    for translation, axis, angle in zip(
        JOINT_TRANSLATIONS, JOINT_AXES, values
    ):
        transform = transform @ _translation(translation) @ _rotation(axis, angle)
    transform = transform @ _translation(TOOL_TRANSLATION)
    return transform[:3, 3].copy(), transform[:3, :3].copy()


def grasp_marker_rotation_to_ee_rotation(
    marker_rotation: Sequence[Sequence[float]],
) -> np.ndarray:
    """Convert sm_grasping marker axes to OMY end_effector_link axes.

    The marker uses +X from palm to TCP and +Y as its closing direction.  The
    OMY model uses -Y from palm to TCP and +X as its closing direction.
    """
    return np.asarray(marker_rotation, dtype=float) @ GRASP_MARKER_TO_EE_ROTATION


def _matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized quaternion in w, x, y, z order."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array(
            [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2.0 * np.sqrt(max(1e-12, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]))
            quaternion = np.array(
                [
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                ]
            )
        elif index == 1:
            scale = 2.0 * np.sqrt(max(1e-12, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]))
            quaternion = np.array(
                [
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                ]
            )
        else:
            scale = 2.0 * np.sqrt(max(1e-12, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]))
            quaternion = np.array(
                [
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    norm = float(np.linalg.norm(quaternion))
    return quaternion / max(norm, 1e-12)


def _quaternion_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm([w, x, y, z]))
    w, x, y, z = np.asarray([w, x, y, z], dtype=float) / max(norm, 1e-12)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def interpolate_rotation(
    start_rotation: Sequence[Sequence[float]],
    end_rotation: Sequence[Sequence[float]],
    fraction: float,
) -> np.ndarray:
    """Shortest-path spherical interpolation between two rotations."""
    fraction = float(np.clip(fraction, 0.0, 1.0))
    start = np.asarray(start_rotation, dtype=float)
    end = np.asarray(end_rotation, dtype=float)
    if fraction <= 0.0:
        return start.copy()
    if fraction >= 1.0:
        return end.copy()

    start_quaternion = _matrix_to_quaternion(start)
    end_quaternion = _matrix_to_quaternion(end)
    dot = float(np.dot(start_quaternion, end_quaternion))
    if dot < 0.0:
        end_quaternion = -end_quaternion
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        blended = start_quaternion + fraction * (end_quaternion - start_quaternion)
    else:
        angle = acos(dot)
        denominator = sin(angle)
        blended = (
            sin((1.0 - fraction) * angle) / denominator * start_quaternion
            + sin(fraction * angle) / denominator * end_quaternion
        )
    return _quaternion_to_matrix(blended)


def apply_inward_radial_offset(
    position: Sequence[float], offset: float
) -> np.ndarray:
    """Move an arm-base-frame target radially toward link0 in the XY plane."""
    compensated = np.asarray(position, dtype=float).copy()
    radial_norm = float(np.linalg.norm(compensated[:2]))
    offset = max(0.0, float(offset))
    if radial_norm > 1e-9:
        compensated[:2] -= min(offset, radial_norm) * compensated[:2] / radial_norm
    return compensated


def structured_pregrasp_seed(
    current_joints: Sequence[float], target_position: Sequence[float]
) -> np.ndarray:
    """Return a bent-elbow seed suited to distant, above-table targets."""
    current = np.asarray(current_joints, dtype=float)
    target = np.asarray(target_position, dtype=float)
    return np.array(
        [atan2(float(target[1]), float(target[0])), 0.20, 2.00, -2.20, 2.00, current[5]],
        dtype=float,
    )


def interpolate_joint_waypoints(
    start: Sequence[float], end: Sequence[float], maximum_step: float
) -> list[np.ndarray]:
    """Interpolate every joint together, excluding start and including end."""
    start_array = np.asarray(start, dtype=float)
    end_array = np.asarray(end, dtype=float)
    maximum_step = max(0.01, float(maximum_step))
    step_count = max(1, int(ceil(float(np.max(np.abs(end_array - start_array))) / maximum_step)))
    return [
        start_array + (end_array - start_array) * (step / step_count)
        for step in range(1, step_count + 1)
    ]


def _quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def rotation_error_vector(
    target_rotation: Sequence[Sequence[float]],
    current_rotation: Sequence[Sequence[float]],
) -> np.ndarray:
    """Shortest world-frame rotation vector from current to target."""
    target = _matrix_to_quaternion(np.asarray(target_rotation, dtype=float))
    current = _matrix_to_quaternion(np.asarray(current_rotation, dtype=float))
    current_inverse = current * np.array([1.0, -1.0, -1.0, -1.0])
    error = _quaternion_multiply(target, current_inverse)
    if error[0] < 0.0:
        error = -error
    vector_norm = float(np.linalg.norm(error[1:]))
    if vector_norm < 1e-10:
        return 2.0 * error[1:]
    angle = 2.0 * atan2(vector_norm, max(float(error[0]), 0.0))
    return error[1:] * (angle / vector_norm)


def rotation_distance(
    target_rotation: Sequence[Sequence[float]],
    current_rotation: Sequence[Sequence[float]],
) -> float:
    return float(np.linalg.norm(rotation_error_vector(target_rotation, current_rotation)))


def _pose_error(
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    joints: np.ndarray,
    orientation_weight: float,
) -> tuple[np.ndarray, float, float]:
    current_position, current_rotation = forward_kinematics(joints)
    position_error = target_position - current_position
    orientation_error = rotation_error_vector(target_rotation, current_rotation)
    weighted = np.concatenate((position_error, orientation_weight * orientation_error))
    return weighted, float(np.linalg.norm(position_error)), float(np.linalg.norm(orientation_error))


def _numeric_jacobian(joints: np.ndarray, orientation_weight: float) -> np.ndarray:
    position, rotation = forward_kinematics(joints)
    epsilon = 1e-5
    jacobian = np.zeros((6, 6), dtype=float)
    for index in range(6):
        perturbed = joints.copy()
        perturbed[index] += epsilon
        next_position, next_rotation = forward_kinematics(perturbed)
        jacobian[:3, index] = (next_position - position) / epsilon
        jacobian[3:, index] = (
            orientation_weight
            * rotation_error_vector(next_rotation, rotation)
            / epsilon
        )
    return jacobian


def solve_pose_ik(
    target_position: Sequence[float],
    target_rotation: Sequence[Sequence[float]],
    seed: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    *,
    max_iterations: int = 240,
    position_tolerance: float = 0.003,
    orientation_tolerance: float = 0.025,
    damping: float = 0.025,
    orientation_weight: float = 0.22,
) -> IKResult:
    """Solve one bounded pose without changing the target during iteration."""
    target_position_array = np.asarray(target_position, dtype=float)
    target_rotation_array = np.asarray(target_rotation, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    joints = np.clip(np.asarray(seed, dtype=float), lower, upper)
    if target_position_array.shape != (3,) or target_rotation_array.shape != (3, 3):
        raise ValueError("Target pose must contain a 3-vector and a 3x3 rotation")
    if joints.shape != (6,) or lower.shape != (6,) or upper.shape != (6,):
        raise ValueError("IK seed and limits must each contain six values")
    if np.any(lower > upper):
        return IKResult(False, joints, float("inf"), float("inf"), 0, "invalid joint limits")

    last_position_error = float("inf")
    last_orientation_error = float("inf")
    for iteration in range(max_iterations + 1):
        error, last_position_error, last_orientation_error = _pose_error(
            target_position_array,
            target_rotation_array,
            joints,
            orientation_weight,
        )
        if (
            last_position_error <= position_tolerance
            and last_orientation_error <= orientation_tolerance
        ):
            return IKResult(
                True,
                joints.copy(),
                last_position_error,
                last_orientation_error,
                iteration,
                "converged",
            )
        if iteration == max_iterations:
            break

        jacobian = _numeric_jacobian(joints, orientation_weight)
        augmented_matrix = np.vstack((jacobian, damping * np.eye(6)))
        augmented_error = np.concatenate((error, np.zeros(6)))
        step, *_ = np.linalg.lstsq(augmented_matrix, augmented_error, rcond=None)
        maximum = float(np.max(np.abs(step)))
        if maximum > 0.18:
            step *= 0.18 / maximum

        current_score = float(np.linalg.norm(error))
        accepted = False
        for scale in (1.0, 0.5, 0.25, 0.1):
            candidate = np.clip(joints + scale * step, lower, upper)
            candidate_error, _, _ = _pose_error(
                target_position_array,
                target_rotation_array,
                candidate,
                orientation_weight,
            )
            if float(np.linalg.norm(candidate_error)) < current_score - 1e-10:
                joints = candidate
                accepted = True
                break
        if not accepted:
            break

    return IKResult(
        False,
        joints.copy(),
        last_position_error,
        last_orientation_error,
        iteration,
        (
            "IK did not converge: "
            f"position_error={last_position_error:.4f} m, "
            f"orientation_error={last_orientation_error:.4f} rad"
        ),
    )


def plan_top_down_sequence(
    grasp_position: Sequence[float],
    grasp_rotation: Sequence[Sequence[float]],
    current_joints: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    *,
    clearance: float = 0.10,
    waypoint_spacing: float = 0.02,
) -> TopDownPlan:
    """Plan one immutable pre-grasp and vertical descent joint sequence."""
    clearance = max(0.0, float(clearance))
    waypoint_spacing = max(0.005, float(waypoint_spacing))
    grasp_position_array = np.asarray(grasp_position, dtype=float)
    grasp_rotation_array = np.asarray(grasp_rotation, dtype=float)
    pregrasp_position = grasp_position_array + np.array([0.0, 0.0, clearance])

    pregrasp = solve_pose_ik(
        pregrasp_position,
        grasp_rotation_array,
        seed=current_joints,
        lower_limits=lower_limits,
        upper_limits=upper_limits,
    )
    if not pregrasp.success:
        return TopDownPlan(False, message=f"pregrasp {pregrasp.message}")

    step_count = max(1, int(ceil(clearance / waypoint_spacing)))
    previous = pregrasp.joints
    waypoints: list[np.ndarray] = []
    for step in range(1, step_count + 1):
        fraction = step / step_count
        waypoint_position = pregrasp_position.copy()
        waypoint_position[2] -= clearance * fraction
        result = solve_pose_ik(
            waypoint_position,
            grasp_rotation_array,
            seed=previous,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
        )
        if not result.success:
            return TopDownPlan(
                False,
                pregrasp_joints=pregrasp.joints,
                message=f"descent waypoint {step}/{step_count} {result.message}",
            )
        previous = result.joints
        waypoints.append(result.joints.copy())

    return TopDownPlan(
        True,
        pregrasp_joints=pregrasp.joints.copy(),
        descent_waypoints=waypoints,
        message="planned",
    )


def solve_blended_descent_waypoints(
    start_position: Sequence[float],
    end_position: Sequence[float],
    start_rotation: Sequence[Sequence[float]],
    end_rotation: Sequence[Sequence[float]],
    seed: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    waypoint_spacing: float,
) -> tuple[list[np.ndarray], str]:
    """Solve an immutable Cartesian descent while gradually rotating the wrist."""
    start_position_array = np.asarray(start_position, dtype=float)
    end_position_array = np.asarray(end_position, dtype=float)
    distance = float(np.linalg.norm(end_position_array - start_position_array))
    spacing = max(0.005, float(waypoint_spacing))
    step_count = max(1, int(ceil(distance / spacing - 1e-9)))
    previous = np.asarray(seed, dtype=float)
    waypoints: list[np.ndarray] = []
    for step in range(1, step_count + 1):
        fraction = step / step_count
        target_position = start_position_array + fraction * (
            end_position_array - start_position_array
        )
        target_rotation = interpolate_rotation(start_rotation, end_rotation, fraction)
        result = solve_pose_ik(
            target_position,
            target_rotation,
            seed=previous,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            max_iterations=500,
            position_tolerance=0.005,
            orientation_tolerance=0.03,
        )
        if not result.success:
            return [], f"descent waypoint {step}/{step_count} {result.message}"
        previous = result.joints
        waypoints.append(result.joints.copy())
    return waypoints, "planned"


def validate_and_build_plan(
    grasp_position: Sequence[float],
    approach_waypoints: list[np.ndarray],
    pregrasp_joints: Sequence[float],
    descent_waypoints: list[np.ndarray],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    minimum_approach_clearance: float,
) -> TopDownPlan:
    """Reject paths that leave limits or dip near the object before descent."""
    grasp = np.asarray(grasp_position, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    minimum_z = float(grasp[2]) + max(0.0, float(minimum_approach_clearance))
    for index, waypoint in enumerate(approach_waypoints):
        joints = np.asarray(waypoint, dtype=float)
        if np.any(joints < lower - 1e-9) or np.any(joints > upper + 1e-9):
            return TopDownPlan(False, message=f"approach waypoint {index + 1} exceeds joint limits")
        position, _ = forward_kinematics(joints)
        if float(position[2]) < minimum_z:
            return TopDownPlan(
                False,
                message=(
                    f"approach waypoint {index + 1} height {position[2]:.4f} m "
                    f"is below safe height {minimum_z:.4f} m"
                ),
            )
    for index, waypoint in enumerate(descent_waypoints):
        joints = np.asarray(waypoint, dtype=float)
        if np.any(joints < lower - 1e-9) or np.any(joints > upper + 1e-9):
            return TopDownPlan(False, message=f"descent waypoint {index + 1} exceeds joint limits")
    if not approach_waypoints or not descent_waypoints:
        return TopDownPlan(False, message="planned path contains no executable waypoints")
    return TopDownPlan(
        True,
        pregrasp_joints=np.asarray(pregrasp_joints, dtype=float).copy(),
        approach_waypoints=[np.asarray(value, dtype=float).copy() for value in approach_waypoints],
        descent_waypoints=[np.asarray(value, dtype=float).copy() for value in descent_waypoints],
        message="planned blended current-distance top-down path",
    )


def plan_blended_top_down_sequence(
    grasp_position: Sequence[float],
    grasp_rotation: Sequence[Sequence[float]],
    current_joints: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    *,
    clearance: float = 0.10,
    radial_inward_offset: float = 0.025,
    waypoint_spacing: float = 0.01,
    approach_joint_step: float = 0.12,
    minimum_approach_clearance: float = 0.08,
) -> TopDownPlan:
    """Plan a fixed-base synchronized approach and orientation-blended descent."""
    current = np.asarray(current_joints, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    compensated = apply_inward_radial_offset(grasp_position, radial_inward_offset)
    pregrasp_position = compensated + np.array([0.0, 0.0, max(0.0, float(clearance))])
    _, current_rotation = forward_kinematics(current)

    candidates = (
        np.clip(current, lower, upper),
        np.clip(structured_pregrasp_seed(current, compensated), lower, upper),
    )
    pregrasp = None
    failures: list[str] = []
    for seed in candidates:
        result = solve_pose_ik(
            pregrasp_position,
            current_rotation,
            seed=seed,
            lower_limits=lower,
            upper_limits=upper,
            max_iterations=500,
            position_tolerance=0.005,
            orientation_tolerance=0.03,
        )
        if result.success:
            pregrasp = result
            break
        failures.append(result.message)
    if pregrasp is None:
        return TopDownPlan(False, message="pregrasp " + "; ".join(failures))

    approach = interpolate_joint_waypoints(
        np.clip(current, lower, upper), pregrasp.joints, approach_joint_step
    )
    descent, descent_message = solve_blended_descent_waypoints(
        pregrasp_position,
        compensated,
        current_rotation,
        grasp_rotation,
        pregrasp.joints,
        lower,
        upper,
        waypoint_spacing,
    )
    if not descent:
        return TopDownPlan(
            False,
            pregrasp_joints=pregrasp.joints.copy(),
            approach_waypoints=approach,
            message=descent_message,
        )
    return validate_and_build_plan(
        compensated,
        approach,
        pregrasp.joints,
        descent,
        lower,
        upper,
        minimum_approach_clearance,
    )


def find_common_reachable_orientation(
    grasp_position: Sequence[float],
    requested_rotation: Sequence[Sequence[float]],
    current_joints: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    *,
    clearance: float,
    minimum_orientation_fraction: float,
    orientation_search_steps: int,
) -> FixedOrientationCandidate:
    """Find the most requested rotation reachable at pre-grasp and grasp."""
    grasp = np.asarray(grasp_position, dtype=float)
    current = np.asarray(current_joints, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    _, current_rotation = forward_kinematics(current)
    minimum_fraction = float(np.clip(minimum_orientation_fraction, 0.0, 1.0))
    search_steps = max(1, int(orientation_search_steps))
    pregrasp_position = grasp + np.array([0.0, 0.0, max(0.0, float(clearance))])
    seed = np.clip(structured_pregrasp_seed(current, grasp), lower, upper)

    last_message = "no orientation samples evaluated"
    for fraction in np.linspace(1.0, minimum_fraction, search_steps + 1):
        rotation = interpolate_rotation(current_rotation, requested_rotation, float(fraction))
        pregrasp = solve_pose_ik(
            pregrasp_position,
            rotation,
            seed=seed,
            lower_limits=lower,
            upper_limits=upper,
            max_iterations=500,
            position_tolerance=0.005,
            orientation_tolerance=0.03,
        )
        if not pregrasp.success:
            last_message = f"fraction={fraction:.3f} pregrasp {pregrasp.message}"
            continue
        grasp_result = solve_pose_ik(
            grasp,
            rotation,
            seed=pregrasp.joints,
            lower_limits=lower,
            upper_limits=upper,
            max_iterations=500,
            position_tolerance=0.005,
            orientation_tolerance=0.03,
        )
        if not grasp_result.success:
            last_message = f"fraction={fraction:.3f} grasp {grasp_result.message}"
            continue
        return FixedOrientationCandidate(
            True,
            fraction=float(fraction),
            rotation=rotation.copy(),
            pregrasp_joints=pregrasp.joints.copy(),
            grasp_joints=grasp_result.joints.copy(),
            message="common fixed orientation found",
        )

    return FixedOrientationCandidate(
        False,
        message=(
            "no common fixed orientation at or above "
            f"fraction {minimum_fraction:.3f}; {last_message}"
        ),
    )


def validate_fixed_orientation_descent(
    pregrasp_joints: Sequence[float],
    grasp_joints: Sequence[float],
    selected_rotation: Sequence[Sequence[float]],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    *,
    maximum_xy_deviation: float,
    maximum_orientation_deviation: float,
    sample_count: int = 21,
) -> tuple[bool, str]:
    """Validate a joint segment as a nearly vertical, fixed-pose descent."""
    pregrasp = np.asarray(pregrasp_joints, dtype=float)
    grasp = np.asarray(grasp_joints, dtype=float)
    selected = np.asarray(selected_rotation, dtype=float)
    lower = np.asarray(lower_limits, dtype=float)
    upper = np.asarray(upper_limits, dtype=float)
    samples = max(3, int(sample_count))
    positions: list[np.ndarray] = []
    maximum_rotation_error = 0.0
    for fraction in np.linspace(0.0, 1.0, samples):
        joints = pregrasp + float(fraction) * (grasp - pregrasp)
        if np.any(joints < lower - 1e-9) or np.any(joints > upper + 1e-9):
            return False, "fixed-orientation descent exceeds joint limits"
        position, rotation = forward_kinematics(joints)
        positions.append(position)
        maximum_rotation_error = max(
            maximum_rotation_error,
            rotation_distance(selected, rotation),
        )
    position_array = np.asarray(positions)
    if np.any(np.diff(position_array[:, 2]) >= -1e-6):
        return False, "fixed-orientation descent is not monotonically decreasing in Z"
    xy_deviation = float(
        np.max(np.linalg.norm(position_array[:, :2] - position_array[-1, :2], axis=1))
    )
    if xy_deviation > max(0.0, float(maximum_xy_deviation)):
        return False, f"fixed-orientation descent XY deviation {xy_deviation:.4f} m"
    if maximum_rotation_error > max(0.0, float(maximum_orientation_deviation)):
        return False, (
            "fixed-orientation descent rotation deviation "
            f"{maximum_rotation_error:.4f} rad"
        )
    return True, "fixed-orientation descent validated"


def plan_fixed_orientation_top_down_sequence(
    grasp_position: Sequence[float],
    grasp_rotation: Sequence[Sequence[float]],
    current_joints: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    *,
    clearance: float = 0.10,
    radial_inward_offset: float = 0.025,
    minimum_orientation_fraction: float = 0.50,
    orientation_search_steps: int = 20,
    maximum_xy_deviation: float = 0.015,
    maximum_orientation_deviation: float = 0.035,
) -> TopDownPlan:
    """Plan semantic endpoints with one rotation shared by descent and lift."""
    compensated = apply_inward_radial_offset(grasp_position, radial_inward_offset)
    candidate = find_common_reachable_orientation(
        compensated,
        grasp_rotation,
        current_joints,
        lower_limits,
        upper_limits,
        clearance=clearance,
        minimum_orientation_fraction=minimum_orientation_fraction,
        orientation_search_steps=orientation_search_steps,
    )
    if not candidate.success:
        return TopDownPlan(False, message=candidate.message)
    valid, validation_message = validate_fixed_orientation_descent(
        candidate.pregrasp_joints,
        candidate.grasp_joints,
        candidate.rotation,
        lower_limits,
        upper_limits,
        maximum_xy_deviation=maximum_xy_deviation,
        maximum_orientation_deviation=maximum_orientation_deviation,
    )
    if not valid:
        return TopDownPlan(
            False,
            pregrasp_joints=candidate.pregrasp_joints,
            grasp_joints=candidate.grasp_joints,
            selected_rotation=candidate.rotation,
            orientation_fraction=candidate.fraction,
            message=validation_message,
        )
    return TopDownPlan(
        True,
        pregrasp_joints=candidate.pregrasp_joints,
        grasp_joints=candidate.grasp_joints,
        selected_rotation=candidate.rotation,
        orientation_fraction=candidate.fraction,
        message=(
            "planned fixed-orientation semantic path; "
            f"orientation_fraction={candidate.fraction:.3f}"
        ),
    )
