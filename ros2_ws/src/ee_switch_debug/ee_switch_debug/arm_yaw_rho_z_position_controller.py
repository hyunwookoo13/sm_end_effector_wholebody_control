from math import acos, atan2, cos, hypot, pi, sin, tanh

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from ee_switch_debug.semantic_joint_trajectory import (
    minimum_compact_duration,
    minimum_quintic_duration,
    sample_compact_joint_positions,
    sample_quintic_joint_positions,
)
from ee_switch_debug.top_down_kinematics import (
    build_group_sequential_approach,
    forward_kinematics,
    grasp_marker_rotation_to_ee_rotation,
    plan_blended_top_down_sequence,
    plan_fixed_orientation_top_down_sequence,
    plan_top_down_sequence,
    rotation_distance,
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def top_down_stage_mask(stage: str) -> list[bool]:
    """Return the only joints allowed to move in a fixed-target stage."""
    normalized = str(stage).strip().upper()
    masks = {
        "APPROACH": [True] * 6,
        "YAW": [True, False, False, False, False, False],
        "ARM": [False, True, True, False, False, False],
        "ARM_POSITION": [False, True, True, False, False, False],
        "WRIST": [False, False, False, True, True, True],
        "WRIST_ALIGN": [False, True, True, True, True, True],
        "DESCEND": [True] * 6,
        "LIFT": [True] * 6,
        "HOME": [True] * 6,
    }
    return list(masks.get(normalized, [False] * 6))


def semantic_top_down_stage_goal(stage, plan, home_positions):
    """Return the only endpoint associated with a semantic motion stage."""
    normalized = str(stage).strip().upper()
    if normalized == "YAW":
        return plan.yaw_joints
    if normalized == "ARM_POSITION":
        return plan.arm_joints
    if normalized == "WRIST_ALIGN":
        return plan.pregrasp_joints
    if normalized in ("APPROACH", "LIFT"):
        return plan.pregrasp_joints
    if normalized == "DESCEND":
        return plan.grasp_joints
    if normalized == "HOME":
        return home_positions
    raise ValueError(f"No semantic endpoint for stage {stage!r}")


def semantic_stage_joint_tolerance(
    stage: str,
    strict_tolerance: float,
    transit_tolerance: float,
) -> float:
    """Keep grasp descent strict while allowing safe transit stages to advance."""
    normalized = str(stage).strip().upper()
    if normalized in ("APPROACH", "LIFT", "HOME"):
        return max(0.0, float(transit_tolerance))
    return max(0.0, float(strict_tolerance))


def semantic_stage_trajectory_profile(stage: str) -> str:
    """Use compact easing only for non-contact semantic transit stages."""
    normalized = str(stage).strip().upper()
    if normalized in (
        "APPROACH",
        "YAW",
        "ARM_POSITION",
        "WRIST_ALIGN",
        "LIFT",
        "HOME",
    ):
        return "compact"
    return "quintic"


def vertical_lift_clearance_reached(
    measured_ee_z: float,
    grasp_ee_z: float,
    required_clearance: float,
) -> bool:
    """Return whether measured EE height has cleared the grasp pose."""
    clearance = max(0.0, float(required_clearance))
    return float(measured_ee_z) - float(grasp_ee_z) >= clearance - 1e-9


def measured_pregrasp_pose_valid(
    current_joints,
    plan,
    position_tolerance: float,
    orientation_tolerance: float,
) -> bool:
    """Verify the measured EE reached the immutable pregrasp pose."""
    current_position, current_rotation = forward_kinematics(current_joints)
    target_position, _ = forward_kinematics(plan.pregrasp_joints)
    return (
        float(np.linalg.norm(current_position - target_position))
        <= max(0.0, float(position_tolerance))
        and rotation_distance(plan.selected_rotation, current_rotation)
        <= max(0.0, float(orientation_tolerance))
    )


def fixed_target_joint_velocities(
    current: list[float],
    target: list[float],
    active_mask: list[bool],
    kp: float,
    velocity_limits: list[float],
    tolerance: float,
) -> tuple[list[float], bool]:
    """P-control one selected joint group toward an immutable joint target."""
    velocities = [0.0] * 6
    active_errors = []
    for index, active in enumerate(active_mask):
        if not active:
            continue
        error = normalize_angle(float(target[index]) - float(current[index]))
        active_errors.append(abs(error))
        if abs(error) > float(tolerance):
            limit = abs(float(velocity_limits[index]))
            velocities[index] = clamp(float(kp) * error, -limit, limit)
    aligned = bool(active_errors) and max(active_errors) <= float(tolerance)
    return velocities, aligned


def pick_z_offsets(
    grasp_offset_z: float,
    descend_depth: float,
    approach_clearance_z: float,
) -> tuple[float, float]:
    """Return the pre-grasp and final descend offsets for a pick."""
    return (
        float(grasp_offset_z) + max(0.0, float(approach_clearance_z)),
        float(grasp_offset_z) - max(0.0, float(descend_depth)),
    )


def quaternion_to_matrix(q) -> list[list[float]]:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = (x*x + y*y + z*z + w*w)**0.5
    if norm > 1e-9:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
    else:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    return [
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z), 2.0*(x*z + w*y)],
        [2.0*(x*y + w*z), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x)],
        [2.0*(x*z - w*y), 2.0*(y*z + w*x), 1.0 - 2.0*(x*x + y*y)]
    ]


def transpose_matrix(M: list[list[float]]) -> list[list[float]]:
    return [
        [M[0][0], M[1][0], M[2][0]],
        [M[0][1], M[1][1], M[2][1]],
        [M[0][2], M[1][2], M[2][2]]
    ]


def multiply_matrices(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    C = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            C[i][j] = sum(A[i][k] * B[k][j] for k in range(3))
    return C


def rotation_error_angle(
    target_rotation: list[list[float]],
    current_rotation: list[list[float]],
) -> float:
    """Return the shortest SO(3) angle between current and target frames."""
    current_from_target = multiply_matrices(
        transpose_matrix(current_rotation),
        target_rotation,
    )
    trace = (
        current_from_target[0][0]
        + current_from_target[1][1]
        + current_from_target[2][2]
    )
    return acos(clamp((trace - 1.0) * 0.5, -1.0, 1.0))


def update_wrist_position_recovery(
    active: bool,
    rho_error: float,
    z_error: float,
    enter_threshold: float,
    exit_threshold: float,
) -> bool:
    error = max(abs(float(rho_error)), abs(float(z_error)))
    if active:
        return error > float(exit_threshold)
    return error > float(enter_threshold)


def axis_vector(axis_name: str) -> tuple[float, float, float] | None:
    normalized = str(axis_name).strip().lower()
    sign = -1.0 if normalized.startswith("-") else 1.0
    name = normalized[1:] if normalized.startswith(("+", "-")) else normalized
    axes = {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "z": (0.0, 0.0, 1.0),
    }
    axis = axes.get(name)
    if axis is None:
        return None
    return sign * axis[0], sign * axis[1], sign * axis[2]


def transform_vector(
    rotation: list[list[float]],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(float(rotation[row][column]) * vector[column] for column in range(3))
        for row in range(3)
    )


def dot_vectors(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sum(a[index] * b[index] for index in range(3))


def cross_vectors(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def normalized_vector(
    vector: tuple[float, float, float],
    epsilon: float = 1e-8,
) -> tuple[float, float, float] | None:
    norm = dot_vectors(vector, vector) ** 0.5
    if norm <= epsilon:
        return None
    return tuple(component / norm for component in vector)


def project_axis_onto_rotation_plane(
    vector: tuple[float, float, float],
    rotation_axis: tuple[float, float, float],
) -> tuple[float, float, float] | None:
    axis = normalized_vector(rotation_axis)
    if axis is None:
        return None
    parallel_scale = dot_vectors(vector, axis)
    projected = tuple(
        vector[index] - parallel_scale * axis[index]
        for index in range(3)
    )
    return normalized_vector(projected)


def compute_link6_axis_target(
    target_rotation: list[list[float]],
    ee_rotation: list[list[float]],
    current_q6: float,
    lower_limit: float,
    upper_limit: float,
    link6_axis: str = "y",
    gripper_closing_axis: str = "x",
    target_closing_axis: str = "y",
    link6_offset: float = 0.0,
) -> tuple[float, float, bool]:
    local_joint_axis = axis_vector(link6_axis)
    local_gripper_axis = axis_vector(gripper_closing_axis)
    local_target_axis = axis_vector(target_closing_axis)
    if local_joint_axis is None or local_gripper_axis is None or local_target_axis is None:
        return clamp(float(current_q6), lower_limit, upper_limit), 0.0, False

    joint_axis = normalized_vector(transform_vector(ee_rotation, local_joint_axis))
    if joint_axis is None:
        return clamp(float(current_q6), lower_limit, upper_limit), 0.0, False
    current_closing = project_axis_onto_rotation_plane(
        transform_vector(ee_rotation, local_gripper_axis),
        joint_axis,
    )
    desired_closing = project_axis_onto_rotation_plane(
        transform_vector(target_rotation, local_target_axis),
        joint_axis,
    )
    if current_closing is None or desired_closing is None:
        return clamp(float(current_q6), lower_limit, upper_limit), 0.0, False

    signed_sine = dot_vectors(joint_axis, cross_vectors(current_closing, desired_closing))
    signed_cosine = clamp(dot_vectors(current_closing, desired_closing), -1.0, 1.0)
    delta = atan2(signed_sine, signed_cosine)
    if delta > pi / 2.0:
        delta -= pi
    elif delta < -pi / 2.0:
        delta += pi

    base_target = float(current_q6) + delta + float(link6_offset)
    candidates = [
        base_target + multiple * pi
        for multiple in range(-4, 5)
        if lower_limit - 1e-9 <= base_target + multiple * pi <= upper_limit + 1e-9
    ]
    if candidates:
        target = min(candidates, key=lambda candidate: abs(candidate - float(current_q6)))
        return clamp(target, lower_limit, upper_limit), delta, True

    return clamp(base_target, lower_limit, upper_limit), delta, False


def decompose_yzy(R: list[list[float]]) -> tuple[float, float, float]:
    r11, r12, r13 = R[0][0], R[0][1], R[0][2]
    r21, r22, r23 = R[1][0], R[1][1], R[1][2]
    r31, r32, r33 = R[2][0], R[2][1], R[2][2]
    
    cos_beta = clamp(r22, -1.0, 1.0)
    beta = acos(cos_beta)
    sin_beta = sin(beta)
    
    if abs(sin_beta) > 1e-6:
        alpha = atan2(r32, -r12)
        gamma = atan2(r23, r21)
    else:
        alpha = 0.0
        if cos_beta > 0.0:
            gamma = atan2(r13, r11)
        else:
            gamma = atan2(-r13, -r11)
            
    return alpha, beta, gamma


def compute_wrist_targets_from_orientation(
    target_rotation: list[list[float]],
    link3_rotation: list[list[float]],
    fallback_targets: list[float],
    joint_lower_limits: list[float],
    joint_upper_limits: list[float],
    mode: str = "link6",
    link6_offset: float = 0.0,
    reference_targets: list[float] | None = None,
) -> list[float]:
    normalized_mode = str(mode).strip().lower()
    if normalized_mode in ("", "false", "none", "off"):
        return list(fallback_targets)

    link3_from_world = transpose_matrix(link3_rotation)
    link3_from_target = multiply_matrices(link3_from_world, target_rotation)
    q4_target, q5_target, q6_target = decompose_yzy(link3_from_target)
    q6_target = normalize_angle(q6_target + float(link6_offset))

    if normalized_mode == "full":
        decompositions = [
            [q4_target, q5_target, q6_target],
            [q4_target + pi, -q5_target, q6_target + pi],
        ]
        references = list(reference_targets or fallback_targets)
        candidates: list[list[float]] = []
        for decomposition in decompositions:
            axis_candidates: list[list[float]] = []
            for index, value in enumerate(decomposition):
                lower = float(joint_lower_limits[index + 3])
                upper = float(joint_upper_limits[index + 3])
                equivalents = [
                    float(value) + multiple * 2.0 * pi
                    for multiple in range(-2, 3)
                    if lower - 1e-9 <= float(value) + multiple * 2.0 * pi <= upper + 1e-9
                ]
                if not equivalents:
                    equivalents = [clamp(float(value), lower, upper)]
                axis_candidates.append(equivalents)
            for q4 in axis_candidates[0]:
                for q5 in axis_candidates[1]:
                    for q6 in axis_candidates[2]:
                        candidates.append([q4, q5, q6])
        targets = min(
            candidates,
            key=lambda candidate: sum(
                normalize_angle(candidate[index] - references[index]) ** 2
                for index in range(3)
            ),
        )
    else:
        targets = [fallback_targets[0], fallback_targets[1], q6_target]

    return [
        clamp(targets[index], joint_lower_limits[index + 3], joint_upper_limits[index + 3])
        for index in range(3)
    ]


def is_wrist_aligned_for_phase(
    task_mode: str,
    grasp_phase: str,
    wrist_errors: list[float],
    tolerance: float,
    require_alignment: bool,
    orientation_valid: bool = True,
) -> bool:
    if not require_alignment:
        return True
    if str(task_mode).strip().upper() != "PICK":
        return True
    if str(grasp_phase).strip().upper() not in ("APPROACH", "DESCEND"):
        return True
    if not orientation_valid:
        return False
    return max(abs(float(error)) for error in wrist_errors) <= float(tolerance)


def update_pick_approach_stage(
    task_mode: str,
    grasp_phase: str,
    current_stage: str,
    position_aligned: bool,
) -> str:
    """Gate wrist motion until the pre-grasp position has been reached."""
    if str(task_mode).strip().upper() != "PICK":
        return "POSITION"
    if str(grasp_phase).strip().upper() != "APPROACH":
        return "WRIST"
    normalized_stage = str(current_stage).strip().upper()
    if normalized_stage in ("WRIST", "PREGRASP", "HOLD"):
        return normalized_stage
    return "WRIST" if position_aligned else "POSITION"


class ArmYawRhoZPositionController(Node):
    def __init__(self) -> None:
        super().__init__("arm_yaw_rho_z_position_controller")

        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("base_frame", "chassis_control_frame")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_position_topic", "/joint_position_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("state_topic", "/arm_safety_state")
        self.declare_parameter("task_command_topic", "/arm_task_command")
        self.declare_parameter("task_state_topic", "/arm_task_state")
        self.declare_parameter("rate_hz", 40.0)
        self.declare_parameter("yaw_error_source", "joint")
        self.declare_parameter("joint1_sign", 1.0)
        self.declare_parameter("joint2_sign", 1.0)
        self.declare_parameter("joint3_sign", 1.0)
        self.declare_parameter("joint2_rho_per_rad", 0.2715)
        self.declare_parameter("joint2_z_per_rad", -0.1972)
        self.declare_parameter("joint3_rho_per_rad", 0.1521)
        self.declare_parameter("joint3_z_per_rad", -0.2631)
        self.declare_parameter("k_yaw", 2.5)
        self.declare_parameter("k_rho", 2.0)
        self.declare_parameter("k_z", 2.5)
        self.declare_parameter("max_yaw_velocity", 0.8)
        self.declare_parameter("max_rho_velocity", 0.22)
        self.declare_parameter("max_z_velocity", 0.22)
        self.declare_parameter("max_joint_velocity", 1.5)
        self.declare_parameter("max_joint2_velocity", 1.0)
        self.declare_parameter("max_joint3_velocity", 1.5)
        self.declare_parameter("max_joint_acceleration", 3.0)
        self.declare_parameter("approach_slowdown_distance", 0.08)
        self.declare_parameter("approach_min_scale", 0.35)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("rho_tolerance", 0.04)
        self.declare_parameter("z_tolerance", 0.04)
        self.declare_parameter("singularity_epsilon", 0.01)
        self.declare_parameter(
            "home_positions",
            [0.0, -1.5708, 2.0944, -0.5236, 1.5708, 0.0],
        )
        self.declare_parameter("home_kp", 2.0)
        self.declare_parameter("home_max_joint_velocity", 1.0)
        self.declare_parameter("home_position_tolerance", 0.03)
        self.declare_parameter("safety_enter_rho_min", 0.20)
        self.declare_parameter("safety_enter_rho_max", 0.55)
        self.declare_parameter("safety_exit_rho_min", 0.15)
        self.declare_parameter("safety_exit_rho_max", 0.65)
        self.declare_parameter("safety_enter_z_min", 0.35)
        self.declare_parameter("safety_enter_z_max", 0.78)
        self.declare_parameter("safety_exit_z_min", 0.30)
        self.declare_parameter("safety_exit_z_max", 0.85)
        self.declare_parameter("safety_enter_yaw_max", 1.0472)
        self.declare_parameter("safety_exit_yaw_max", 1.3090)
        self.declare_parameter("base_stop_rho", 0.50)
        self.declare_parameter("switching_rho", 0.55)
        self.declare_parameter("switching_alpha", 10.0)
        self.declare_parameter("arm_switching_rho", 0.75)
        self.declare_parameter("arm_switching_alpha", 10.0)
        self.declare_parameter("arm_blend_exponent", 1.0)
        self.declare_parameter("base_yaw_tolerance", 0.08)
        self.declare_parameter("k_base_yaw", 3.2)
        self.declare_parameter("k_base_linear", 1.2)
        self.declare_parameter("max_base_yaw_rate", 1.4)
        self.declare_parameter("max_base_linear", 0.5)
        self.declare_parameter("enable_base_motion", False)
        self.declare_parameter(
            "joint_lower_limits",
            [-3.1416, -3.1416, -3.1416, -3.1416, -3.1416, -3.1416],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.1416, 3.1416, 3.1416, 3.1416, 3.1416, 3.1416],
        )
        self.declare_parameter("k_wrist", 2.0)
        self.declare_parameter("max_wrist_velocity", 1.0)
        self.declare_parameter("position_stage_yaw_velocity", 0.35)
        self.declare_parameter("position_stage_joint2_velocity", 0.60)
        self.declare_parameter("position_stage_joint3_velocity", 0.80)
        self.declare_parameter("position_stage_wrist_velocity", 0.30)
        self.declare_parameter("wrist_stage_yaw_velocity", 0.25)
        self.declare_parameter("wrist_stage_joint2_velocity", 0.25)
        self.declare_parameter("wrist_stage_joint3_velocity", 0.35)
        self.declare_parameter("wrist_stage_wrist_velocity", 0.35)
        self.declare_parameter("wrist_recovery_joint2_velocity", 0.30)
        self.declare_parameter("wrist_recovery_joint3_velocity", 0.42)
        self.declare_parameter("wrist_recovery_yaw_velocity", 0.22)
        self.declare_parameter("wrist_recovery_wrist_velocity", 0.08)
        self.declare_parameter("wrist_recovery_acceleration", 0.75)
        self.declare_parameter("wrist_recovery_enter_error_m", 0.05)
        self.declare_parameter("wrist_recovery_exit_error_m", 0.025)
        self.declare_parameter("max_wrist_target_clamp_rad", 0.35)
        self.declare_parameter("pregrasp_stage_yaw_velocity", 0.18)
        self.declare_parameter("pregrasp_stage_joint2_velocity", 0.20)
        self.declare_parameter("pregrasp_stage_joint3_velocity", 0.28)
        self.declare_parameter("pregrasp_stage_wrist_velocity", 0.22)
        self.declare_parameter("descend_stage_yaw_velocity", 0.12)
        self.declare_parameter("descend_stage_joint2_velocity", 0.08)
        self.declare_parameter("descend_stage_joint3_velocity", 0.12)
        self.declare_parameter("descend_stage_wrist_velocity", 0.12)
        self.declare_parameter("position_stage_acceleration", 1.4)
        self.declare_parameter("wrist_stage_acceleration", 0.9)
        self.declare_parameter("pregrasp_stage_acceleration", 0.65)
        self.declare_parameter("descend_stage_acceleration", 0.30)
        self.declare_parameter("joint_soft_limit_margin", 0.12)
        self.declare_parameter(
            "pick_max_joint_excursion",
            [0.65, 1.80, 2.00, 1.40, 1.40, 2.00],
        )
        self.declare_parameter("hold_wrist_during_place", True)
        self.declare_parameter("grasp_orientation_wrist_mode", "link6")
        self.declare_parameter("grasp_link6_orientation_offset", 0.0)
        self.declare_parameter("grasp_link6_axis", "y")
        self.declare_parameter("grasp_gripper_closing_axis", "x")
        self.declare_parameter("grasp_target_closing_axis", "y")
        self.declare_parameter("require_wrist_alignment_before_descend", True)
        self.declare_parameter("wrist_orientation_tolerance", 0.12)
        self.declare_parameter("link1", 0.247)
        self.declare_parameter("link2", 0.45)

        # Grasp sequence parameters
        self.declare_parameter("grasp_descend_speed", 0.3)
        self.declare_parameter("grasp_lift_speed", 0.3)
        self.declare_parameter("grasp_lift_height", 0.12)
        self.declare_parameter("grasp_descend_depth", 0.08)
        self.declare_parameter("grasp_approach_clearance_z", 0.0)
        self.declare_parameter("enable_staged_top_down_approach", False)
        self.declare_parameter("enable_precomputed_top_down_sequence", False)
        self.declare_parameter("top_down_use_fixed_reachable_orientation", False)
        self.declare_parameter("top_down_use_group_sequential_approach", False)
        self.declare_parameter("top_down_blend_orientation_during_descent", False)
        self.declare_parameter("top_down_pregrasp_clearance", 0.10)
        self.declare_parameter("top_down_waypoint_spacing", 0.02)
        self.declare_parameter("top_down_radial_inward_offset", 0.025)
        self.declare_parameter("top_down_approach_joint_step", 0.12)
        self.declare_parameter("top_down_minimum_approach_clearance", 0.08)
        self.declare_parameter("top_down_minimum_orientation_fraction", 0.50)
        self.declare_parameter("top_down_orientation_search_steps", 20)
        self.declare_parameter("top_down_descend_max_xy_deviation", 0.015)
        self.declare_parameter("top_down_descend_max_orientation_deviation", 0.035)
        self.declare_parameter("top_down_segment_min_duration", 0.8)
        self.declare_parameter("top_down_settle_max_command_offset", 0.10)
        self.declare_parameter("top_down_joint_kp", 2.2)
        self.declare_parameter("top_down_joint_tolerance", 0.025)
        self.declare_parameter("top_down_transit_joint_tolerance", 0.025)
        self.declare_parameter("top_down_stage_acceleration", 0.8)
        self.declare_parameter("top_down_lift_home_clearance", 0.06)
        self.declare_parameter(
            "top_down_alignment_velocity_limits",
            [0.25, 0.50, 0.65, 0.45, 0.45, 0.45],
        )
        self.declare_parameter(
            "top_down_path_velocity_limits",
            [0.16, 0.22, 0.28, 0.22, 0.16, 0.22],
        )
        self.declare_parameter("grasp_safe_clearance_z", 0.15)
        self.declare_parameter("grasp_pregrasp_clearance_z", 0.04)
        self.declare_parameter("grasp_final_offset_z", 0.0)
        self.declare_parameter("require_descend_confirmation", False)
        self.declare_parameter("grasp_close_duration", 0.1)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_close_position", 0.8)
        self.declare_parameter("grasp_offset_z", 0.08)
        self.declare_parameter("place_offset_z", 0.10)
        self.declare_parameter("place_descend_depth", 0.08)
        self.declare_parameter("place_open_duration", 0.8)
        self.declare_parameter("return_home_after_place", True)
        self.declare_parameter("return_home_after_pick", False)
        self.declare_parameter("gripper_joint_names", ["rh_l1", "rh_r1_joint"])

        self.arm_base_frame = str(self.get_parameter("arm_base_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.link1 = float(self.get_parameter("link1").value)
        self.link2 = float(self.get_parameter("link2").value)
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        joint_position_topic = str(self.get_parameter("joint_position_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        task_command_topic = str(self.get_parameter("task_command_topic").value)
        task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.yaw_error_source = str(self.get_parameter("yaw_error_source").value)
        self.joint1_sign = float(self.get_parameter("joint1_sign").value)
        self.joint2_sign = float(self.get_parameter("joint2_sign").value)
        self.joint3_sign = float(self.get_parameter("joint3_sign").value)
        self.joint2_rho_per_rad = float(self.get_parameter("joint2_rho_per_rad").value)
        self.joint2_z_per_rad = float(self.get_parameter("joint2_z_per_rad").value)
        self.joint3_rho_per_rad = float(self.get_parameter("joint3_rho_per_rad").value)
        self.joint3_z_per_rad = float(self.get_parameter("joint3_z_per_rad").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.k_rho = float(self.get_parameter("k_rho").value)
        self.k_z = float(self.get_parameter("k_z").value)
        self.max_yaw_velocity = float(self.get_parameter("max_yaw_velocity").value)
        self.max_rho_velocity = float(self.get_parameter("max_rho_velocity").value)
        self.max_z_velocity = float(self.get_parameter("max_z_velocity").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self.max_joint2_velocity = float(
            self.get_parameter("max_joint2_velocity").value
        )
        self.max_joint3_velocity = float(
            self.get_parameter("max_joint3_velocity").value
        )
        self.max_joint_acceleration = float(self.get_parameter("max_joint_acceleration").value)
        self.approach_slowdown_distance = max(
            float(self.get_parameter("approach_slowdown_distance").value),
            0.001,
        )
        self.approach_min_scale = clamp(
            float(self.get_parameter("approach_min_scale").value),
            0.0,
            1.0,
        )
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.rho_tolerance = float(self.get_parameter("rho_tolerance").value)
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.singularity_epsilon = float(self.get_parameter("singularity_epsilon").value)
        self.home_positions = list(self.get_parameter("home_positions").value)
        self.home_kp = float(self.get_parameter("home_kp").value)
        self.home_max_joint_velocity = float(
            self.get_parameter("home_max_joint_velocity").value
        )
        self.home_position_tolerance = float(
            self.get_parameter("home_position_tolerance").value
        )
        self.safety_enter_rho_min = float(
            self.get_parameter("safety_enter_rho_min").value
        )
        self.safety_enter_rho_max = float(
            self.get_parameter("safety_enter_rho_max").value
        )
        self.safety_exit_rho_min = float(
            self.get_parameter("safety_exit_rho_min").value
        )
        self.safety_exit_rho_max = float(
            self.get_parameter("safety_exit_rho_max").value
        )
        self.safety_enter_z_min = float(
            self.get_parameter("safety_enter_z_min").value
        )
        self.safety_enter_z_max = float(
            self.get_parameter("safety_enter_z_max").value
        )
        self.safety_exit_z_min = float(
            self.get_parameter("safety_exit_z_min").value
        )
        self.safety_exit_z_max = float(
            self.get_parameter("safety_exit_z_max").value
        )
        self.safety_enter_yaw_max = float(
            self.get_parameter("safety_enter_yaw_max").value
        )
        self.safety_exit_yaw_max = float(
            self.get_parameter("safety_exit_yaw_max").value
        )
        self.base_stop_rho = float(self.get_parameter("base_stop_rho").value)
        self.switching_rho = float(self.get_parameter("switching_rho").value)
        self.switching_alpha = float(
            self.get_parameter("switching_alpha").value
        )
        self.arm_switching_rho = float(
            self.get_parameter("arm_switching_rho").value
        )
        self.arm_switching_alpha = float(
            self.get_parameter("arm_switching_alpha").value
        )
        self.arm_blend_exponent = max(
            float(self.get_parameter("arm_blend_exponent").value),
            0.01,
        )
        self.base_yaw_tolerance = float(
            self.get_parameter("base_yaw_tolerance").value
        )
        self.k_base_yaw = float(self.get_parameter("k_base_yaw").value)
        self.k_base_linear = float(self.get_parameter("k_base_linear").value)
        self.max_base_yaw_rate = float(
            self.get_parameter("max_base_yaw_rate").value
        )
        self.max_base_linear = float(self.get_parameter("max_base_linear").value)
        self.enable_base_motion = bool(
            self.get_parameter("enable_base_motion").value
        )
        self.joint_lower_limits = list(self.get_parameter("joint_lower_limits").value)
        self.joint_upper_limits = list(self.get_parameter("joint_upper_limits").value)
        self.k_wrist = float(self.get_parameter("k_wrist").value)
        self.max_wrist_velocity = float(self.get_parameter("max_wrist_velocity").value)
        self.stage_velocity_limits = {
            "POSITION": [
                float(self.get_parameter("position_stage_yaw_velocity").value),
                float(self.get_parameter("position_stage_joint2_velocity").value),
                float(self.get_parameter("position_stage_joint3_velocity").value),
                float(self.get_parameter("position_stage_wrist_velocity").value),
            ],
            "WRIST": [
                float(self.get_parameter("wrist_stage_yaw_velocity").value),
                float(self.get_parameter("wrist_stage_joint2_velocity").value),
                float(self.get_parameter("wrist_stage_joint3_velocity").value),
                float(self.get_parameter("wrist_stage_wrist_velocity").value),
            ],
            "RECOVER": [
                float(self.get_parameter("wrist_recovery_yaw_velocity").value),
                float(self.get_parameter("wrist_recovery_joint2_velocity").value),
                float(self.get_parameter("wrist_recovery_joint3_velocity").value),
                float(self.get_parameter("wrist_recovery_wrist_velocity").value),
            ],
            "PREGRASP": [
                float(self.get_parameter("pregrasp_stage_yaw_velocity").value),
                float(self.get_parameter("pregrasp_stage_joint2_velocity").value),
                float(self.get_parameter("pregrasp_stage_joint3_velocity").value),
                float(self.get_parameter("pregrasp_stage_wrist_velocity").value),
            ],
            "DESCEND": [
                float(self.get_parameter("descend_stage_yaw_velocity").value),
                float(self.get_parameter("descend_stage_joint2_velocity").value),
                float(self.get_parameter("descend_stage_joint3_velocity").value),
                float(self.get_parameter("descend_stage_wrist_velocity").value),
            ],
        }
        self.stage_acceleration_limits = {
            "POSITION": float(self.get_parameter("position_stage_acceleration").value),
            "WRIST": float(self.get_parameter("wrist_stage_acceleration").value),
            "RECOVER": float(self.get_parameter("wrist_recovery_acceleration").value),
            "PREGRASP": float(self.get_parameter("pregrasp_stage_acceleration").value),
            "DESCEND": float(self.get_parameter("descend_stage_acceleration").value),
        }
        self.joint_soft_limit_margin = max(
            0.0, float(self.get_parameter("joint_soft_limit_margin").value)
        )
        self.pick_max_joint_excursion = [
            max(0.0, float(value))
            for value in self.get_parameter("pick_max_joint_excursion").value
        ]
        self.wrist_recovery_enter_error_m = max(
            0.0, float(self.get_parameter("wrist_recovery_enter_error_m").value)
        )
        self.wrist_recovery_exit_error_m = min(
            self.wrist_recovery_enter_error_m,
            max(0.0, float(self.get_parameter("wrist_recovery_exit_error_m").value)),
        )
        self.max_wrist_target_clamp_rad = max(
            0.0, float(self.get_parameter("max_wrist_target_clamp_rad").value)
        )
        self.hold_wrist_during_place = bool(
            self.get_parameter("hold_wrist_during_place").value
        )
        self.grasp_orientation_wrist_mode = str(
            self.get_parameter("grasp_orientation_wrist_mode").value
        ).strip().lower()
        if self.grasp_orientation_wrist_mode not in ("off", "link6", "full"):
            self.get_logger().warn(
                "grasp_orientation_wrist_mode must be off, link6, or full; using link6"
            )
            self.grasp_orientation_wrist_mode = "link6"
        self.grasp_link6_orientation_offset = float(
            self.get_parameter("grasp_link6_orientation_offset").value
        )
        self.grasp_link6_axis = str(
            self.get_parameter("grasp_link6_axis").value
        ).strip().lower()
        self.grasp_gripper_closing_axis = str(
            self.get_parameter("grasp_gripper_closing_axis").value
        ).strip().lower()
        self.grasp_target_closing_axis = str(
            self.get_parameter("grasp_target_closing_axis").value
        ).strip().lower()
        self.require_wrist_alignment_before_descend = bool(
            self.get_parameter("require_wrist_alignment_before_descend").value
        )
        self.wrist_orientation_tolerance = max(
            0.0,
            float(self.get_parameter("wrist_orientation_tolerance").value),
        )

        # Grasp sequence parameters
        self.grasp_descend_speed = float(self.get_parameter("grasp_descend_speed").value)
        self.grasp_lift_speed = float(self.get_parameter("grasp_lift_speed").value)
        self.grasp_lift_height = float(self.get_parameter("grasp_lift_height").value)
        self.grasp_descend_depth = max(
            0.0,
            float(self.get_parameter("grasp_descend_depth").value),
        )
        self.grasp_approach_clearance_z = max(
            0.0,
            float(self.get_parameter("grasp_approach_clearance_z").value),
        )
        self.enable_staged_top_down_approach = bool(
            self.get_parameter("enable_staged_top_down_approach").value
        )
        self.enable_precomputed_top_down_sequence = bool(
            self.get_parameter("enable_precomputed_top_down_sequence").value
        )
        self.top_down_use_fixed_reachable_orientation = bool(
            self.get_parameter("top_down_use_fixed_reachable_orientation").value
        )
        self.top_down_use_group_sequential_approach = bool(
            self.get_parameter("top_down_use_group_sequential_approach").value
        )
        self.top_down_blend_orientation_during_descent = bool(
            self.get_parameter("top_down_blend_orientation_during_descent").value
        )
        self.top_down_pregrasp_clearance = max(
            0.0, float(self.get_parameter("top_down_pregrasp_clearance").value)
        )
        self.top_down_waypoint_spacing = max(
            0.005, float(self.get_parameter("top_down_waypoint_spacing").value)
        )
        self.top_down_radial_inward_offset = max(
            0.0, float(self.get_parameter("top_down_radial_inward_offset").value)
        )
        self.top_down_approach_joint_step = max(
            0.01, float(self.get_parameter("top_down_approach_joint_step").value)
        )
        self.top_down_minimum_approach_clearance = max(
            0.0,
            float(self.get_parameter("top_down_minimum_approach_clearance").value),
        )
        self.top_down_minimum_orientation_fraction = clamp(
            float(self.get_parameter("top_down_minimum_orientation_fraction").value),
            0.0,
            1.0,
        )
        self.top_down_orientation_search_steps = max(
            1, int(self.get_parameter("top_down_orientation_search_steps").value)
        )
        self.top_down_descend_max_xy_deviation = max(
            0.0, float(self.get_parameter("top_down_descend_max_xy_deviation").value)
        )
        self.top_down_descend_max_orientation_deviation = max(
            0.0,
            float(self.get_parameter("top_down_descend_max_orientation_deviation").value),
        )
        self.top_down_segment_min_duration = max(
            0.1, float(self.get_parameter("top_down_segment_min_duration").value)
        )
        self.top_down_settle_max_command_offset = max(
            0.0,
            float(
                self.get_parameter("top_down_settle_max_command_offset").value
            ),
        )
        self.top_down_joint_kp = max(
            0.0, float(self.get_parameter("top_down_joint_kp").value)
        )
        self.top_down_joint_tolerance = max(
            0.001, float(self.get_parameter("top_down_joint_tolerance").value)
        )
        self.top_down_transit_joint_tolerance = max(
            0.001,
            float(self.get_parameter("top_down_transit_joint_tolerance").value),
        )
        self.top_down_stage_acceleration = max(
            0.0, float(self.get_parameter("top_down_stage_acceleration").value)
        )
        self.top_down_lift_home_clearance = max(
            0.0,
            float(self.get_parameter("top_down_lift_home_clearance").value),
        )
        self.top_down_alignment_velocity_limits = [
            max(0.0, float(value))
            for value in self.get_parameter("top_down_alignment_velocity_limits").value
        ]
        self.top_down_path_velocity_limits = [
            max(0.0, float(value))
            for value in self.get_parameter("top_down_path_velocity_limits").value
        ]
        if (
            len(self.top_down_alignment_velocity_limits) != 6
            or len(self.top_down_path_velocity_limits) != 6
        ):
            raise ValueError("top-down velocity limit parameters must contain six values")
        self.grasp_safe_clearance_z = max(
            0.0, float(self.get_parameter("grasp_safe_clearance_z").value)
        )
        self.grasp_pregrasp_clearance_z = max(
            0.0, float(self.get_parameter("grasp_pregrasp_clearance_z").value)
        )
        self.grasp_final_offset_z = float(
            self.get_parameter("grasp_final_offset_z").value
        )
        self.require_descend_confirmation = bool(
            self.get_parameter("require_descend_confirmation").value
        )
        if self.grasp_safe_clearance_z < self.grasp_pregrasp_clearance_z:
            self.grasp_safe_clearance_z = self.grasp_pregrasp_clearance_z
        self.grasp_close_duration = float(self.get_parameter("grasp_close_duration").value)
        self.gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self.gripper_close_position = float(self.get_parameter("gripper_close_position").value)
        self.grasp_offset_z = float(self.get_parameter("grasp_offset_z").value)
        self.place_offset_z = float(self.get_parameter("place_offset_z").value)
        self.place_descend_depth = max(
            0.0,
            float(self.get_parameter("place_descend_depth").value),
        )
        self.place_open_duration = max(
            0.0,
            float(self.get_parameter("place_open_duration").value),
        )
        self.return_home_after_place = bool(
            self.get_parameter("return_home_after_place").value
        )
        self.return_home_after_pick = bool(
            self.get_parameter("return_home_after_pick").value
        )
        self.gripper_joint_names = list(self.get_parameter("gripper_joint_names").value)

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joints: dict[str, float] = {}
        self.position_command: list[float] | None = None
        self.previous_velocity = [0.0] * 6
        self.control_state: str | None = None
        self.target_received = False
        self.target_tracking_enabled = False
        self.target_accept_after_ns = 0
        self.last_update_time = None
        self.last_debug = "waiting for joint states"

        # Grasp sequence state
        self.task_mode = "PICK"
        self.force_safety_pose = False
        self.grasp_phase = "APPROACH"  # APPROACH, DESCEND, GRASP, LIFT, HOLD
        self.pick_approach_stage = "POSITION"  # POSITION -> WRIST -> DESCEND
        self.descend_confirmed = False
        self.pregrasp_hold_positions: list[float] | None = None
        self.pick_start_positions: list[float] | None = None
        self.wrist_position_recovery = False
        self.grasp_z_offset, _ = pick_z_offsets(
            self.grasp_offset_z,
            self.grasp_descend_depth,
            self.grasp_approach_clearance_z,
        )
        if self.enable_staged_top_down_approach:
            self.grasp_z_offset = self.grasp_safe_clearance_z
        self.grasp_phase_start_time = None
        self.gripper_position = self.gripper_open_position
        self.lift_z_accumulated = 0.0
        self.place_wrist_hold_positions: list[float] | None = None
        self.top_down_plan = None
        self.top_down_plan_attempted = False
        self.top_down_plan_error = ""
        self.top_down_stage = "WAIT_TARGET"
        self.top_down_waypoint_index = 0
        self.top_down_segment_stage = ""
        self.top_down_segment_start = None
        self.top_down_segment_goal = None
        self.top_down_segment_start_time = None
        self.top_down_segment_duration = 0.0
        self.top_down_segment_planned_complete = False
        self.top_down_continuous_home = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.create_subscription(String, task_command_topic, self.on_task_command, 10)
        self.position_pub = self.create_publisher(JointState, joint_position_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.task_state_pub = self.create_publisher(String, task_state_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(self.rate_hz, 0.1), 0.01), self.on_timer)
        self.log_timer = self.create_timer(1.0, self.on_log_timer)

        self.get_logger().info(
            f"Arm position controller: {self.arm_base_frame} -> {self.target_frame}, "
            f"base frame={self.base_frame}, publishing {joint_position_topic}, "
            f"link6 axes=[joint {self.grasp_link6_axis}, "
            f"gripper {self.grasp_gripper_closing_axis}, "
            f"target {self.grasp_target_closing_axis}]"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))
        if self.position_command is None and all(name in self.current_joints for name in self.joint_names):
            self.position_command = [self.current_joints[name] for name in self.joint_names]
            self.last_update_time = self.get_clock().now()
            self.get_logger().info("Position command initialized from current joint states")

    def on_task_command(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "PICK":
            self.start_pick_sequence()
            self.enable_fresh_target_tracking()
        elif command == "PLACE":
            self.start_place_sequence()
            self.enable_fresh_target_tracking()
        elif command == "TRANSPORT":
            self.disable_target_tracking()
            self.start_transport_sequence()
        elif command == "RESET":
            self.release_pregrasp_hold()
            self.disable_target_tracking()
            self.start_pick_sequence()
            self.set_control_state("RETURN_HOME")
        elif command == "DESCEND":
            if (
                self.task_mode == "PICK"
                and self.enable_staged_top_down_approach
                and self.grasp_phase == "APPROACH"
            ):
                self.descend_confirmed = True
                self.previous_velocity = [0.0] * 6
                if self.pick_approach_stage == "HOLD":
                    self.release_pregrasp_hold()
                    self.pick_approach_stage = "PREGRASP"
                    self.get_logger().warn(
                        "Arm task command: DESCEND accepted; released pre-grasp hold"
                    )
                else:
                    self.get_logger().warn(
                        "Arm task command: DESCEND queued; "
                        f"current stage={self.pick_approach_stage}"
                    )
            else:
                self.get_logger().warn(
                    "DESCEND ignored: no active staged PICK approach"
                )
        else:
            self.get_logger().warn(f"Unknown arm task command: {msg.data!r}")

    def enable_fresh_target_tracking(self) -> None:
        self.target_tracking_enabled = True
        self.target_received = False
        self.target_accept_after_ns = self.get_clock().now().nanoseconds

    def disable_target_tracking(self) -> None:
        self.target_tracking_enabled = False
        self.target_received = False
        self.target_accept_after_ns = 0
        self.previous_velocity = [0.0] * 6

    def start_pick_sequence(self) -> None:
        self.task_mode = "PICK"
        self.force_safety_pose = False
        self.place_wrist_hold_positions = None
        self.grasp_phase = "APPROACH"
        self.pick_approach_stage = "POSITION"
        self.descend_confirmed = False
        self.pregrasp_hold_positions = None
        self.pick_start_positions = self.capture_current_arm_positions()
        self.wrist_position_recovery = False
        self.reset_precomputed_top_down_state()
        self.grasp_z_offset, _ = pick_z_offsets(
            self.grasp_offset_z,
            self.grasp_descend_depth,
            self.grasp_approach_clearance_z,
        )
        if self.enable_staged_top_down_approach:
            self.grasp_z_offset = self.grasp_safe_clearance_z
        self.grasp_phase_start_time = None
        self.gripper_position = self.gripper_open_position
        self.lift_z_accumulated = 0.0
        self.previous_velocity = [0.0] * 6
        self.publish_task_state()
        self.get_logger().warn("Arm task command: PICK")

    def capture_current_arm_positions(self) -> list[float] | None:
        if all(name in self.current_joints for name in self.joint_names):
            return [self.current_joints[name] for name in self.joint_names]
        if self.position_command is not None:
            return list(self.position_command[:6])
        return None

    def start_place_sequence(self) -> None:
        self.task_mode = "PLACE"
        self.force_safety_pose = False
        self.place_wrist_hold_positions = self.capture_current_wrist_positions()
        if self.position_command is not None and self.place_wrist_hold_positions is not None:
            self.position_command[3:6] = self.place_wrist_hold_positions
        self.grasp_phase = "APPROACH"
        self.grasp_z_offset = self.place_offset_z
        self.grasp_phase_start_time = None
        self.gripper_position = self.gripper_close_position
        self.lift_z_accumulated = 0.0
        self.previous_velocity = [0.0] * 6
        self.publish_task_state()
        self.get_logger().warn("Arm task command: PLACE")

    def start_transport_sequence(self) -> None:
        """Retract the arm while retaining the grasp during base navigation."""
        self.task_mode = "TRANSPORT"
        self.force_safety_pose = True
        self.grasp_phase = "HOLD"
        self.gripper_position = self.gripper_close_position
        self.place_wrist_hold_positions = None
        self.previous_velocity = [0.0] * 6
        self.set_control_state("RETURN_HOME")
        self.publish_task_state()
        self.get_logger().warn("Arm task command: TRANSPORT")

    def capture_current_wrist_positions(self) -> list[float] | None:
        wrist_names = self.joint_names[3:6]
        if all(name in self.current_joints for name in wrist_names):
            return [self.current_joints[name] for name in wrist_names]
        if self.position_command is not None:
            return list(self.position_command[3:6])
        return None

    def publish_task_state(self) -> None:
        if (
            self.task_mode == "PICK"
            and getattr(self, "enable_precomputed_top_down_sequence", False)
            and getattr(self, "top_down_stage", "WAIT_TARGET") not in ("WAIT_TARGET", "")
        ):
            state = f"PICK:{self.top_down_stage}"
        elif (
            self.task_mode == "PICK"
            and self.grasp_phase == "APPROACH"
            and self.pick_approach_stage == "HOLD"
        ):
            state = "PICK:PREGRASP_HOLD"
        else:
            state = f"{self.task_mode}:{self.grasp_phase}"
        self.task_state_pub.publish(String(data=state))

    def reset_precomputed_top_down_state(self) -> None:
        self.top_down_plan = None
        self.top_down_plan_attempted = False
        self.top_down_plan_error = ""
        self.top_down_stage = "WAIT_TARGET"
        self.top_down_waypoint_index = 0
        self.top_down_continuous_home = False
        self.clear_top_down_segment_state()

    def clear_top_down_segment_state(self) -> None:
        self.top_down_segment_stage = ""
        self.top_down_segment_start = None
        self.top_down_segment_goal = None
        self.top_down_segment_start_time = None
        self.top_down_segment_duration = 0.0
        self.top_down_segment_planned_complete = False

    def precomputed_top_down_joint_bounds(self) -> tuple[list[float], list[float]]:
        lower = [float(value) for value in self.joint_lower_limits]
        upper = [float(value) for value in self.joint_upper_limits]
        start = self.pick_start_positions or [
            float(self.current_joints[name]) for name in self.joint_names
        ]
        for index, excursion in enumerate(self.pick_max_joint_excursion):
            lower[index] = max(lower[index], float(start[index]) - float(excursion))
            upper[index] = min(upper[index], float(start[index]) + float(excursion))
        return lower, upper

    def ensure_precomputed_top_down_plan(
        self,
        grasp_position: list[float],
        grasp_rotation: list[list[float]],
    ) -> bool:
        """Compute exactly one fixed plan for the current PICK command."""
        if self.top_down_plan is not None:
            return True
        if self.top_down_plan_attempted:
            return False

        self.top_down_plan_attempted = True
        lower, upper = self.precomputed_top_down_joint_bounds()
        current = [float(self.current_joints[name]) for name in self.joint_names]
        final_position = [float(value) for value in grasp_position]
        final_position[2] += float(getattr(self, "grasp_final_offset_z", 0.0))
        converted_rotation = grasp_marker_rotation_to_ee_rotation(grasp_rotation)
        fixed_orientation = bool(
            getattr(self, "top_down_use_fixed_reachable_orientation", False)
        )
        if fixed_orientation:
            plan = plan_fixed_orientation_top_down_sequence(
                final_position,
                converted_rotation,
                current_joints=current,
                lower_limits=lower,
                upper_limits=upper,
                clearance=self.top_down_pregrasp_clearance,
                radial_inward_offset=self.top_down_radial_inward_offset,
                minimum_orientation_fraction=self.top_down_minimum_orientation_fraction,
                orientation_search_steps=self.top_down_orientation_search_steps,
                maximum_xy_deviation=self.top_down_descend_max_xy_deviation,
                maximum_orientation_deviation=(
                    self.top_down_descend_max_orientation_deviation
                ),
            )
            if (
                plan.success
                and getattr(self, "top_down_use_group_sequential_approach", False)
            ):
                plan = build_group_sequential_approach(
                    plan,
                    current_joints=current,
                    lower_limits=lower,
                    upper_limits=upper,
                    minimum_clearance=self.top_down_minimum_approach_clearance,
                )
        elif getattr(self, "top_down_blend_orientation_during_descent", False):
            plan = plan_blended_top_down_sequence(
                final_position,
                converted_rotation,
                current_joints=current,
                lower_limits=lower,
                upper_limits=upper,
                clearance=self.top_down_pregrasp_clearance,
                radial_inward_offset=self.top_down_radial_inward_offset,
                waypoint_spacing=self.top_down_waypoint_spacing,
                approach_joint_step=self.top_down_approach_joint_step,
                minimum_approach_clearance=self.top_down_minimum_approach_clearance,
            )
        else:
            plan = plan_top_down_sequence(
                final_position,
                converted_rotation,
                current_joints=current,
                lower_limits=lower,
                upper_limits=upper,
                clearance=self.top_down_pregrasp_clearance,
                waypoint_spacing=self.top_down_waypoint_spacing,
            )
        if not plan.success:
            self.top_down_plan_error = plan.message
            self.top_down_stage = "PLAN_FAILED"
            self.get_logger().error(
                f"Top-down PICK plan rejected; arm held: {plan.message}"
            )
            return False

        self.top_down_plan = plan
        blended = bool(getattr(plan, "approach_waypoints", []))
        group_sequential = bool(
            fixed_orientation
            and getattr(self, "top_down_use_group_sequential_approach", False)
        )
        self.top_down_stage = (
            "YAW"
            if group_sequential
            else "APPROACH"
            if fixed_orientation or blended
            else "YAW"
        )
        self.top_down_waypoint_index = 0
        self.pick_approach_stage = self.top_down_stage
        self.previous_velocity = [0.0] * 6
        self.get_logger().warn(
            "Top-down PICK plan latched once: "
            f"clearance={self.top_down_pregrasp_clearance:.3f} m, "
            f"approach_waypoints={len(getattr(plan, 'approach_waypoints', []))}, "
            f"descent_waypoints={len(plan.descent_waypoints)}, "
            f"orientation_fraction={getattr(plan, 'orientation_fraction', 0.0):.3f}"
        )
        return True

    def command_precomputed_joint_target(
        self,
        target: list[float],
        stage: str,
        dt: float,
    ) -> bool:
        current = [float(self.current_joints[name]) for name in self.joint_names]
        mask = top_down_stage_mask(stage)
        limits = (
            self.top_down_path_velocity_limits
            if stage in ("DESCEND", "LIFT")
            else self.top_down_alignment_velocity_limits
        )
        desired_velocity, aligned = fixed_target_joint_velocities(
            current=current,
            target=list(target),
            active_mask=mask,
            kp=self.top_down_joint_kp,
            velocity_limits=limits,
            tolerance=self.top_down_joint_tolerance,
        )
        if aligned:
            for index, active in enumerate(mask):
                if active:
                    self.position_command[index] = float(target[index])
            self.previous_velocity = [0.0] * 6
            return True

        desired_velocity = self.apply_joint_limit_slowdown(desired_velocity)
        limited_velocity = self.limit_acceleration(
            desired_velocity,
            dt,
            max_acceleration=self.top_down_stage_acceleration,
        )
        self.integrate_position_command(limited_velocity, dt)
        return False

    def command_semantic_top_down_segment(
        self,
        goal: list[float],
        stage: str,
        dt: float,
    ) -> bool:
        """Execute a smooth segment, then close residual measured-joint error."""
        current = [float(self.current_joints[name]) for name in self.joint_names]
        normalized_stage = str(stage).strip().upper()
        group_stage = normalized_stage in ("YAW", "ARM_POSITION", "WRIST_ALIGN")
        active_mask = (
            top_down_stage_mask(normalized_stage) if group_stage else [True] * 6
        )
        requested_goal = [float(value) for value in goal]
        stage_goal = [
            requested if active else measured
            for requested, measured, active in zip(
                requested_goal,
                current,
                active_mask,
            )
        ]
        profile = semantic_stage_trajectory_profile(normalized_stage)
        duration_function = (
            minimum_compact_duration
            if profile == "compact"
            else minimum_quintic_duration
        )
        sample_function = (
            sample_compact_joint_positions
            if profile == "compact"
            else sample_quintic_joint_positions
        )
        limits = (
            self.top_down_path_velocity_limits
            if normalized_stage in ("DESCEND", "LIFT")
            else self.top_down_alignment_velocity_limits
        )
        if self.top_down_segment_stage != normalized_stage:
            self.top_down_segment_stage = normalized_stage
            self.top_down_segment_start = list(current)
            self.top_down_segment_goal = stage_goal
            self.top_down_segment_start_time = self.get_clock().now()
            self.top_down_segment_planned_complete = False
            self.top_down_segment_duration = duration_function(
                self.top_down_segment_start,
                self.top_down_segment_goal,
                limits,
                acceleration_limit=max(self.top_down_stage_acceleration, 0.01),
                minimum_duration=self.top_down_segment_min_duration,
            )
            self.get_logger().warn(
                f"Top-down {normalized_stage} trajectory started: "
                f"profile={profile}, "
                f"duration={self.top_down_segment_duration:.2f} s"
            )

        elapsed = (
            self.get_clock().now() - self.top_down_segment_start_time
        ).nanoseconds * 1e-9
        position, planned_complete = sample_function(
            self.top_down_segment_start,
            self.top_down_segment_goal,
            elapsed,
            self.top_down_segment_duration,
        )
        if not planned_complete:
            previous_command = list(self.position_command[:6])
            safe_dt = max(float(dt), 1e-6)
            self.previous_velocity = [
                clamp(
                    (float(value) - float(previous)) / safe_dt,
                    -abs(float(limit)),
                    abs(float(limit)),
                )
                for value, previous, limit in zip(
                    position,
                    previous_command,
                    limits,
                )
            ]
            self.position_command[:6] = [float(value) for value in position]
            return False

        if not self.top_down_segment_planned_complete:
            self.position_command[:6] = list(self.top_down_segment_goal)
            self.top_down_segment_planned_complete = True

        stage_tolerance = semantic_stage_joint_tolerance(
            normalized_stage,
            self.top_down_joint_tolerance,
            self.top_down_transit_joint_tolerance,
        )
        desired_velocity, aligned = fixed_target_joint_velocities(
            current=current,
            target=list(self.top_down_segment_goal),
            active_mask=active_mask,
            kp=self.top_down_joint_kp,
            velocity_limits=limits,
            tolerance=stage_tolerance,
        )
        if aligned:
            self.previous_velocity = [0.0] * 6
            return True

        desired_velocity = self.apply_joint_limit_slowdown(desired_velocity)
        limited_velocity = self.limit_acceleration(
            desired_velocity,
            dt,
            max_acceleration=self.top_down_stage_acceleration,
        )
        self.integrate_position_command(limited_velocity, dt)
        maximum_offset = self.top_down_settle_max_command_offset
        for index, endpoint in enumerate(self.top_down_segment_goal):
            self.position_command[index] = clamp(
                self.position_command[index],
                float(endpoint) - maximum_offset,
                float(endpoint) + maximum_offset,
            )
        return False

    def command_continuous_top_down_home(
        self,
        goal: list[float],
        dt: float,
    ) -> bool:
        """Drive HOME while retaining and gradually redirecting LIFT velocity."""
        current = [float(self.current_joints[name]) for name in self.joint_names]
        desired_velocity, aligned = fixed_target_joint_velocities(
            current=current,
            target=list(goal),
            active_mask=[True] * 6,
            kp=self.top_down_joint_kp,
            velocity_limits=self.top_down_alignment_velocity_limits,
            tolerance=self.top_down_transit_joint_tolerance,
        )
        if aligned:
            self.position_command[:6] = [float(value) for value in goal]
            self.previous_velocity = [0.0] * 6
            return True

        desired_velocity = self.apply_joint_limit_slowdown(desired_velocity)
        limited_velocity = self.limit_acceleration(
            desired_velocity,
            dt,
            max_acceleration=self.top_down_stage_acceleration,
        )
        self.integrate_position_command(limited_velocity, dt)
        return False

    def measured_lift_home_clearance_reached(self) -> bool:
        """Check measured EE rise above the immutable planned grasp pose."""
        current = [float(self.current_joints[name]) for name in self.joint_names]
        measured_position, _ = forward_kinematics(current)
        grasp_position, _ = forward_kinematics(self.top_down_plan.grasp_joints)
        return vertical_lift_clearance_reached(
            measured_position[2],
            grasp_position[2],
            self.top_down_lift_home_clearance,
        )

    def advance_precomputed_top_down_stage(
        self,
        next_stage: str,
        preserve_velocity: bool = False,
    ) -> None:
        previous = self.top_down_stage
        self.top_down_stage = next_stage
        self.pick_approach_stage = next_stage
        if not preserve_velocity:
            self.previous_velocity = [0.0] * 6
        self.clear_top_down_segment_state()
        self.get_logger().warn(f"Top-down PICK stage: {previous} -> {next_stage}")

    def run_semantic_fixed_orientation_pick(self, dt: float) -> None:
        """Execute fixed-orientation semantic endpoints through smooth segments."""
        stage = self.top_down_stage
        self.gripper_position = (
            self.gripper_close_position
            if stage in ("GRASP", "LIFT", "HOME", "HOLD")
            else self.gripper_open_position
        )

        if stage in (
            "APPROACH",
            "YAW",
            "ARM_POSITION",
            "WRIST_ALIGN",
            "DESCEND",
            "LIFT",
            "HOME",
        ):
            if (
                stage == "LIFT"
                and self.return_home_after_pick
                and self.measured_lift_home_clearance_reached()
            ):
                self.grasp_phase = "HOME"
                self.top_down_continuous_home = True
                self.advance_precomputed_top_down_stage(
                    "HOME",
                    preserve_velocity=True,
                )
                return

            goal = semantic_top_down_stage_goal(
                stage,
                self.top_down_plan,
                self.home_positions,
            )
            goal_values = [float(value) for value in goal]
            if stage == "HOME" and self.top_down_continuous_home:
                stage_complete = self.command_continuous_top_down_home(
                    goal_values,
                    dt,
                )
            else:
                stage_complete = self.command_semantic_top_down_segment(
                    goal_values,
                    stage,
                    dt,
                )
            if stage_complete:
                if stage == "YAW":
                    self.advance_precomputed_top_down_stage("ARM_POSITION")
                elif stage == "ARM_POSITION":
                    self.advance_precomputed_top_down_stage("WRIST_ALIGN")
                elif stage == "WRIST_ALIGN":
                    self.advance_precomputed_top_down_stage("PREGRASP_VERIFY")
                elif stage == "APPROACH":
                    self.grasp_phase = "DESCEND"
                    self.advance_precomputed_top_down_stage("DESCEND")
                elif stage == "DESCEND":
                    self.grasp_phase = "GRASP"
                    self.grasp_phase_start_time = self.get_clock().now()
                    self.advance_precomputed_top_down_stage("GRASP")
                elif stage == "LIFT":
                    if self.return_home_after_pick:
                        self.grasp_phase = "HOME"
                        self.top_down_continuous_home = False
                        self.advance_precomputed_top_down_stage("HOME")
                    else:
                        self.grasp_phase = "HOLD"
                        self.advance_precomputed_top_down_stage("HOLD")
                elif stage == "HOME":
                    self.grasp_phase = "HOLD"
                    self.top_down_continuous_home = False
                    self.advance_precomputed_top_down_stage("HOLD")
                    self.force_safety_pose = True
                    self.disable_target_tracking()
                    self.get_logger().warn(
                        "Fixed-orientation PICK complete at home pose"
                    )

        elif stage == "PREGRASP_VERIFY":
            current = [
                float(self.current_joints[name]) for name in self.joint_names
            ]
            self.previous_velocity = [0.0] * 6
            if measured_pregrasp_pose_valid(
                current,
                self.top_down_plan,
                getattr(self, "top_down_descend_max_xy_deviation", 0.015),
                getattr(
                    self,
                    "top_down_descend_max_orientation_deviation",
                    0.035,
                ),
            ):
                self.grasp_phase = "DESCEND"
                self.advance_precomputed_top_down_stage("DESCEND")

        elif stage == "GRASP":
            self.gripper_position = self.gripper_close_position
            self.position_command[:6] = [
                float(value) for value in self.top_down_plan.grasp_joints
            ]
            elapsed = (
                self.get_clock().now() - self.grasp_phase_start_time
            ).nanoseconds * 1e-9
            if elapsed >= self.grasp_close_duration:
                self.grasp_phase = "LIFT"
                self.advance_precomputed_top_down_stage("LIFT")

        elif stage in ("HOLD", "PLAN_FAILED"):
            self.previous_velocity = [0.0] * 6

        self.publish_position_command()
        self.publish_task_state()
        self.last_debug = (
            f"TOP_DOWN_SEMANTIC stage={self.top_down_stage}, "
            f"duration={self.top_down_segment_duration:.2f}, "
            f"plan_latched={self.top_down_plan is not None}, "
            f"grip={self.gripper_position:.2f}"
        )

    def run_precomputed_top_down_pick(
        self,
        grasp_position: list[float],
        grasp_rotation: list[list[float]],
        dt: float,
    ) -> None:
        """Execute a latched joint plan; target TF is never consulted again."""
        self.cmd_pub.publish(Twist())
        if not self.ensure_precomputed_top_down_plan(grasp_position, grasp_rotation):
            self.previous_velocity = [0.0] * 6
            self.gripper_position = self.gripper_open_position
            self.publish_position_command()
            self.publish_task_state()
            self.last_debug = f"TOP_DOWN_PLAN_FAILED held; {self.top_down_plan_error}"
            return

        if getattr(self, "top_down_use_fixed_reachable_orientation", False):
            self.run_semantic_fixed_orientation_pick(dt)
            return

        pregrasp = [float(value) for value in self.top_down_plan.pregrasp_joints]
        stage = self.top_down_stage
        self.gripper_position = (
            self.gripper_close_position
            if stage in ("GRASP", "LIFT", "HOLD")
            else self.gripper_open_position
        )

        if stage == "APPROACH":
            waypoints = self.top_down_plan.approach_waypoints
            target = [float(value) for value in waypoints[self.top_down_waypoint_index]]
            if self.command_precomputed_joint_target(target, "APPROACH", dt):
                self.top_down_waypoint_index += 1
                if self.top_down_waypoint_index >= len(waypoints):
                    self.grasp_phase = "DESCEND"
                    self.top_down_waypoint_index = 0
                    self.advance_precomputed_top_down_stage("DESCEND")

        elif stage in ("YAW", "ARM", "WRIST"):
            if self.command_precomputed_joint_target(pregrasp, stage, dt):
                next_stage = {"YAW": "ARM", "ARM": "WRIST", "WRIST": "DESCEND"}[stage]
                if next_stage == "DESCEND":
                    self.grasp_phase = "DESCEND"
                    self.top_down_waypoint_index = 0
                self.advance_precomputed_top_down_stage(next_stage)

        elif stage == "DESCEND":
            waypoints = self.top_down_plan.descent_waypoints
            target = [float(value) for value in waypoints[self.top_down_waypoint_index]]
            if self.command_precomputed_joint_target(target, "DESCEND", dt):
                self.top_down_waypoint_index += 1
                if self.top_down_waypoint_index >= len(waypoints):
                    self.grasp_phase = "GRASP"
                    self.grasp_phase_start_time = self.get_clock().now()
                    self.advance_precomputed_top_down_stage("GRASP")

        elif stage == "GRASP":
            self.gripper_position = self.gripper_close_position
            final_target = [
                float(value) for value in self.top_down_plan.descent_waypoints[-1]
            ]
            self.position_command[:6] = final_target
            elapsed = (
                self.get_clock().now() - self.grasp_phase_start_time
            ).nanoseconds * 1e-9
            if elapsed >= self.grasp_close_duration:
                self.grasp_phase = "LIFT"
                self.top_down_waypoint_index = 0
                self.advance_precomputed_top_down_stage("LIFT")

        elif stage == "LIFT":
            lift_waypoints = list(
                reversed(
                    [self.top_down_plan.pregrasp_joints]
                    + self.top_down_plan.descent_waypoints[:-1]
                )
            )
            target = [float(value) for value in lift_waypoints[self.top_down_waypoint_index]]
            if self.command_precomputed_joint_target(target, "LIFT", dt):
                self.top_down_waypoint_index += 1
                if self.top_down_waypoint_index >= len(lift_waypoints):
                    self.grasp_phase = "HOLD"
                    self.advance_precomputed_top_down_stage("HOLD")
                    if self.return_home_after_pick:
                        self.disable_target_tracking()
                        self.force_safety_pose = True
                        self.set_control_state("RETURN_HOME")
                        self.get_logger().warn(
                            "Top-down PICK complete; returning to home pose"
                        )

        elif stage in ("HOLD", "PLAN_FAILED"):
            self.previous_velocity = [0.0] * 6

        self.publish_position_command()
        self.publish_task_state()
        self.last_debug = (
            f"TOP_DOWN_FIXED stage={self.top_down_stage}, "
            f"waypoint={self.top_down_waypoint_index}, "
            f"plan_latched={self.top_down_plan is not None}, "
            f"grip={self.gripper_position:.2f}"
        )

    def is_pregrasp_hold(self) -> bool:
        return (
            self.task_mode == "PICK"
            and self.grasp_phase == "APPROACH"
            and self.pick_approach_stage == "HOLD"
            and self.pregrasp_hold_positions is not None
        )

    def enter_pregrasp_hold(self) -> None:
        if self.position_command is None:
            return
        self.pick_approach_stage = "HOLD"
        self.pregrasp_hold_positions = list(self.position_command[:6])
        self.previous_velocity = [0.0] * 6
        self.set_control_state("ARM_TRACK")
        self.publish_task_state()
        self.get_logger().warn(
            "Pick approach stage: PREGRASP -> PREGRASP_HOLD; "
            "joint target and grasp snapshot latched"
        )

    def release_pregrasp_hold(self) -> None:
        self.pregrasp_hold_positions = None
        self.wrist_position_recovery = False
        self.previous_velocity = [0.0] * 6

    def maintain_pregrasp_hold(self, reason: str = "") -> None:
        if self.pregrasp_hold_positions is not None:
            self.position_command[:6] = self.pregrasp_hold_positions
        self.previous_velocity = [0.0] * 6
        self.cmd_pub.publish(Twist())
        self.publish_position_command()
        self.publish_task_state()
        suffix = f"; {reason}" if reason else ""
        self.last_debug = f"PREGRASP_HOLD latched{suffix}"

    def on_timer(self) -> None:
        if self.position_command is None or self.last_update_time is None:
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0 or dt > 0.2:
            self.previous_velocity = [0.0] * 6
            self.cmd_pub.publish(Twist())
            self.publish_position_command()
            return

        if not self.target_tracking_enabled:
            self.track_home_without_target(dt, "target tracking disabled")
            return

        if (
            self.enable_precomputed_top_down_sequence
            and self.task_mode == "PICK"
            and not self.force_safety_pose
        ):
            # Once planning has been attempted, execute or hold without any TF
            # lookup.  Later perception/marker changes cannot alter the pick.
            if self.top_down_plan is not None or self.top_down_plan_attempted:
                self.run_precomputed_top_down_pick(
                    [0.0, 0.0, 0.0],
                    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                    dt,
                )
                return
            try:
                target_transform = self.tf_buffer.lookup_transform(
                    self.arm_base_frame,
                    self.target_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                self.track_home_without_target(
                    dt, f"waiting for first top-down target TF: {exc}"
                )
                return
            target_stamp_ns = (
                int(target_transform.header.stamp.sec) * 1_000_000_000
                + int(target_transform.header.stamp.nanosec)
            )
            if target_stamp_ns < self.target_accept_after_ns:
                self.track_home_without_target(
                    dt,
                    "waiting for a top-down target TF newer than the PICK command",
                )
                return
            target = target_transform.transform.translation
            self.target_received = True
            self.run_precomputed_top_down_pick(
                [float(target.x), float(target.y), float(target.z)],
                quaternion_to_matrix(target_transform.transform.rotation),
                dt,
            )
            return

        try:
            target_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            target_stamp_ns = (
                int(target_transform.header.stamp.sec) * 1_000_000_000
                + int(target_transform.header.stamp.nanosec)
            )
            if target_stamp_ns < self.target_accept_after_ns:
                if self.is_pregrasp_hold():
                    self.maintain_pregrasp_hold("ignoring stale target TF")
                    return
                self.track_home_without_target(
                    dt,
                    "waiting for a target TF newer than the current task command",
                )
                return
            ee_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            base_target_transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            link3_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                "link3",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            if self.is_pregrasp_hold():
                self.maintain_pregrasp_hold(f"target TF temporarily unavailable: {exc}")
                return
            self.track_home_without_target(dt, f"TF lookup failed: {exc}")
            return

        self.target_received = True
        if self.is_pregrasp_hold():
            self.maintain_pregrasp_hold()
            return
        target = target_transform.transform.translation
        ee = ee_transform.transform.translation

        R_target = quaternion_to_matrix(target_transform.transform.rotation)
        R_ee = quaternion_to_matrix(ee_transform.transform.rotation)
        R_link3 = quaternion_to_matrix(link3_transform.transform.rotation)

        offset_local = [0.008493, 0.017565, 0.0]
        # Current wrist center position in link0 frame
        wc_x = ee.x - (R_ee[0][0]*offset_local[0] + R_ee[0][1]*offset_local[1] + R_ee[0][2]*offset_local[2])
        wc_y = ee.y - (R_ee[1][0]*offset_local[0] + R_ee[1][1]*offset_local[1] + R_ee[1][2]*offset_local[2])
        wc_z = ee.z - (R_ee[2][0]*offset_local[0] + R_ee[2][1]*offset_local[1] + R_ee[2][2]*offset_local[2])

        # Target wrist center position in link0 frame
        wc_target_x = target.x - (R_target[0][0]*offset_local[0] + R_target[0][1]*offset_local[1] + R_target[0][2]*offset_local[2])
        wc_target_y = target.y - (R_target[1][0]*offset_local[0] + R_target[1][1]*offset_local[1] + R_target[1][2]*offset_local[2])
        wc_target_z = target.z - (R_target[2][0]*offset_local[0] + R_target[2][1]*offset_local[1] + R_target[2][2]*offset_local[2])

        # Dynamic pre-grasp/descent/lift offset relative to the detected grasp target.
        wc_target_z = wc_target_z + self.grasp_z_offset

        base_target = base_target_transform.transform.translation
        target_yaw = atan2(wc_target_y, wc_target_x)
        target_rho = hypot(wc_target_x, wc_target_y)
        ee_yaw = atan2(wc_y, wc_x)
        
        joint_yaw_error = normalize_angle(
            target_yaw - self.joint1_sign * self.current_joints["joint1"]
        )
        ee_yaw_error = normalize_angle(target_yaw - ee_yaw)
        current_rho = hypot(wc_x, wc_y)
        if self.yaw_error_source == "ee" and current_rho >= 0.25:
            yaw_error = ee_yaw_error
        else:
            yaw_error = joint_yaw_error
        rho_error = target_rho - hypot(wc_x, wc_y)
        z_error = wc_target_z - wc_z

        self.update_control_state(target_rho, wc_target_z, target_yaw)

        # Check alignment of joints 1, 2, 3
        pos_aligned = (
            abs(yaw_error) <= self.yaw_tolerance
            and abs(rho_error) <= self.rho_tolerance
            and abs(z_error) <= self.z_tolerance
        )
        recovery_was_active = self.wrist_position_recovery
        if (
            self.task_mode == "PICK"
            and self.grasp_phase == "APPROACH"
            and self.pick_approach_stage == "WRIST"
        ):
            self.wrist_position_recovery = update_wrist_position_recovery(
                self.wrist_position_recovery,
                rho_error,
                z_error,
                self.wrist_recovery_enter_error_m,
                self.wrist_recovery_exit_error_m,
            )
        else:
            self.wrist_position_recovery = False
        if recovery_was_active != self.wrist_position_recovery:
            self.previous_velocity = [0.0] * 6
            state = "ENTER" if self.wrist_position_recovery else "EXIT"
            self.get_logger().warn(
                f"Wrist position recovery {state}: "
                f"rho_error={rho_error:.3f}, z_error={z_error:.3f}"
            )

        q4_curr = self.current_joints["joint4"]
        q5_curr = self.current_joints["joint5"]
        q6_curr = self.current_joints["joint6"]

        if (
            self.task_mode == "PLACE"
            and self.hold_wrist_during_place
            and not self.force_safety_pose
            and self.place_wrist_hold_positions is None
        ):
            self.place_wrist_hold_positions = [q4_curr, q5_curr, q6_curr]
            self.position_command[3:6] = self.place_wrist_hold_positions

        # Keep the wrist stable while carrying/releasing the object.
        q4_des = self.home_positions[3]
        q5_des = self.home_positions[4]
        q6_des = -1.5708  # -90 degrees to orient gripper for grasping
        link6_delta = 0.0
        link6_alignment_valid = True
        wrist_solution_valid = True
        max_wrist_clamp = 0.0
        previous_approach_stage = self.pick_approach_stage
        self.pick_approach_stage = update_pick_approach_stage(
            self.task_mode,
            self.grasp_phase,
            self.pick_approach_stage,
            pos_aligned,
        )
        if self.pick_approach_stage != previous_approach_stage:
            self.previous_velocity = [0.0] * 6
            self.get_logger().warn(
                "Pick approach stage: POSITION -> WRIST; holding pre-grasp position"
            )
        wrist_tracking_enabled = (
            self.task_mode == "PICK"
            and not self.force_safety_pose
            and self.pick_approach_stage in ("WRIST", "PREGRASP")
        )
        if wrist_tracking_enabled:
            if self.grasp_orientation_wrist_mode == "link6":
                q6_des, link6_delta, link6_alignment_valid = compute_link6_axis_target(
                    target_rotation=R_target,
                    ee_rotation=R_ee,
                    current_q6=q6_curr,
                    lower_limit=self.joint_lower_limits[5],
                    upper_limit=self.joint_upper_limits[5],
                    link6_axis=self.grasp_link6_axis,
                    gripper_closing_axis=self.grasp_gripper_closing_axis,
                    target_closing_axis=self.grasp_target_closing_axis,
                    link6_offset=self.grasp_link6_orientation_offset,
                )
            else:
                q4_des, q5_des, q6_des = compute_wrist_targets_from_orientation(
                    target_rotation=R_target,
                    link3_rotation=R_link3,
                    fallback_targets=[q4_des, q5_des, q6_des],
                    joint_lower_limits=self.joint_lower_limits,
                    joint_upper_limits=self.joint_upper_limits,
                    mode=self.grasp_orientation_wrist_mode,
                    link6_offset=self.grasp_link6_orientation_offset,
                    reference_targets=[q4_curr, q5_curr, q6_curr],
                )
                wrist_targets = [q4_des, q5_des, q6_des]
                bounded_targets = []
                for joint_index, target_value in enumerate(wrist_targets, start=3):
                    lower, upper = self.active_joint_bounds(joint_index)
                    bounded = clamp(target_value, lower, upper)
                    max_wrist_clamp = max(
                        max_wrist_clamp,
                        abs(normalize_angle(target_value - bounded)),
                    )
                    bounded_targets.append(bounded)
                q4_des, q5_des, q6_des = bounded_targets
                wrist_solution_valid = (
                    max_wrist_clamp <= self.max_wrist_target_clamp_rad
                )
        if (
            self.task_mode == "PLACE"
            and self.hold_wrist_during_place
            and not self.force_safety_pose
            and self.grasp_phase in ("APPROACH", "DESCEND", "RELEASE", "RETREAT")
            and self.place_wrist_hold_positions is not None
        ):
            q4_des, q5_des, q6_des = self.place_wrist_hold_positions

        e4 = normalize_angle(q4_des - q4_curr)
        e5 = normalize_angle(q5_des - q5_curr)
        e6 = normalize_angle(q6_des - q6_curr)
        orientation_error = rotation_error_angle(R_target, R_ee)
        wrist_aligned = is_wrist_aligned_for_phase(
            self.task_mode,
            self.grasp_phase,
            [e4, e5, e6],
            self.wrist_orientation_tolerance,
            self.require_wrist_alignment_before_descend,
            link6_alignment_valid and wrist_solution_valid,
        )
        if self.task_mode == "PICK" and self.grasp_orientation_wrist_mode == "full":
            wrist_aligned = (
                wrist_aligned
                and orientation_error <= self.wrist_orientation_tolerance
            )
        phase_aligned = pos_aligned and wrist_aligned

        # ── Pick/place sequence state machine ──
        self._update_task_phase(pos_aligned, wrist_aligned, dt)
        self.publish_task_state()
        
        joint4_velocity = clamp(self.k_wrist * e4, -self.max_wrist_velocity, self.max_wrist_velocity)
        joint5_velocity = clamp(self.k_wrist * e5, -self.max_wrist_velocity, self.max_wrist_velocity)
        joint6_velocity = clamp(self.k_wrist * e6, -self.max_wrist_velocity, self.max_wrist_velocity)
        wrist_velocities = [joint4_velocity, joint5_velocity, joint6_velocity]

        if self.control_state == "RETURN_HOME":
            home_velocity = self.compute_home_velocities()
            if self.force_safety_pose:
                limited_velocity = self.limit_acceleration(home_velocity, dt)
                self.integrate_position_command(limited_velocity, dt)
                self.cmd_pub.publish(Twist())
                self.publish_position_command()
                home_error = max(
                    abs(self.home_positions[index] - self.current_joints[self.joint_names[index]])
                    for index in range(6)
                )
                self.last_debug = (
                    f"MISSION_DONE_RETURN_HOME home_error={home_error:.3f}, "
                    f"qdot=[{limited_velocity[0]:.3f}, {limited_velocity[1]:.3f}, "
                    f"{limited_velocity[2]:.3f}], base stopped"
                )
                return

            # Force wrist back to home during RETURN_HOME
            q4_des = self.home_positions[3]
            q5_des = self.home_positions[4]
            q6_des = self.home_positions[5]
            e4 = normalize_angle(q4_des - q4_curr)
            e5 = normalize_angle(q5_des - q5_curr)
            e6 = normalize_angle(q6_des - q6_curr)
            home_wrist_vels = [
                clamp(self.k_wrist * e4, -self.max_wrist_velocity, self.max_wrist_velocity),
                clamp(self.k_wrist * e5, -self.max_wrist_velocity, self.max_wrist_velocity),
                clamp(self.k_wrist * e6, -self.max_wrist_velocity, self.max_wrist_velocity)
            ]
            tracking_velocity = self.compute_joint_velocities(
                yaw_error,
                rho_error,
                z_error,
                self.current_joints["joint2"],
                self.current_joints["joint3"],
            )
            tracking_velocity.extend(home_wrist_vels)
            mu = self.compute_switching(
                target_rho,
                self.switching_rho,
                self.switching_alpha,
            )
            arm_mu = self.compute_switching(
                target_rho,
                self.arm_switching_rho,
                self.arm_switching_alpha,
            )
            arm_blend = (
                arm_mu ** self.arm_blend_exponent
                if self.is_transition_target_safe(wc_target_z, target_yaw)
                else 0.0
            )
            desired_velocity = [
                (1.0 - arm_blend) * home + arm_blend * tracking
                for home, tracking in zip(home_velocity, tracking_velocity)
            ]
            limited_velocity = self.limit_acceleration(desired_velocity, dt)
            self.integrate_position_command(limited_velocity, dt)
            base_scale = 1.0 - mu
            twist = self.compute_base_twist(
                base_target.x,
                base_target.y,
                target_rho,
                base_scale,
            )
            self.cmd_pub.publish(twist)
            self.publish_position_command()
            home_error = max(
                abs(self.home_positions[index] - self.current_joints[self.joint_names[index]])
                for index in range(6)
            )
            self.last_debug = (
                f"RETURN_HOME target=[rho {target_rho:.3f}, yaw {target_yaw:.3f}, "
                f"z {wc_target_z:.3f}], home_error={home_error:.3f}, "
                f"mu={mu:.3f}, arm_mu={arm_mu:.3f}, "
                f"arm_blend={arm_blend:.3f}, "
                f"base_scale={base_scale:.3f}, "
                f"base=[vx {twist.linear.x:.3f}, wz {twist.angular.z:.3f}]"
            )
            return

        self.cmd_pub.publish(Twist())

        desired_velocity = self.compute_joint_velocities(
            yaw_error,
            rho_error,
            z_error,
            self.current_joints["joint2"],
            self.current_joints["joint3"],
        )
        desired_velocity.extend(wrist_velocities)
        desired_velocity = self.apply_stage_velocity_limits(desired_velocity)
        desired_velocity = self.apply_joint_limit_slowdown(desired_velocity)
        limited_velocity = self.limit_acceleration(
            desired_velocity,
            dt,
            max_acceleration=self.current_stage_acceleration(),
        )
        self.integrate_position_command(limited_velocity, dt)

        self.publish_position_command()
        self.last_debug = (
            f"ARM_TRACK target=[rho {target_rho:.3f}, yaw {target_yaw:.3f}, "
            f"z {wc_target_z:.3f}], error=[yaw {yaw_error:.3f}, "
            f"rho {rho_error:.3f}, z {z_error:.3f}], "
            f"qdot=[{limited_velocity[0]:.3f}, {limited_velocity[1]:.3f}, "
            f"{limited_velocity[2]:.3f}], qcmd=[{self.position_command[0]:.3f}, "
            f"{self.position_command[1]:.3f}, {self.position_command[2]:.3f}], "
            f"task={self.task_mode}, phase={self.grasp_phase}, "
            f"approach_stage={self.pick_approach_stage}, "
            f"z_off={self.grasp_z_offset:.3f}, "
            f"wrist_err=[{e4:.3f}, {e5:.3f}, {e6:.3f}], "
            f"wrist_aligned={wrist_aligned}, "
            f"orientation_error={orientation_error:.3f}, "
            f"position_recovery={self.wrist_position_recovery}, "
            f"link6=[cur {q6_curr:.3f}, des {q6_des:.3f}, "
            f"delta {link6_delta:.3f}, valid {link6_alignment_valid}], "
            f"wrist_solution_valid={wrist_solution_valid}, "
            f"wrist_clamp={max_wrist_clamp:.3f}, "
            f"grip={self.gripper_position:.2f}"
        )

    def track_home_without_target(self, dt: float, reason: str) -> None:
        """Return home without consulting or blending any cached target pose."""
        self.set_control_state("RETURN_HOME")
        desired_velocity = self.compute_home_velocities()
        limited_velocity = self.limit_acceleration(desired_velocity, dt)
        self.integrate_position_command(limited_velocity, dt)
        self.cmd_pub.publish(Twist())
        self.publish_position_command()
        self.last_debug = f"RETURN_HOME; cached target ignored; {reason}"

    def update_control_state(
        self,
        target_rho: float,
        target_z: float,
        target_yaw: float,
    ) -> None:
        if self.force_safety_pose:
            self.set_control_state("RETURN_HOME")
            return

        inside_enter = (
            self.safety_enter_rho_min <= target_rho <= self.safety_enter_rho_max
            and self.safety_enter_z_min <= target_z <= self.safety_enter_z_max
            and abs(target_yaw) <= self.safety_enter_yaw_max
        )
        inside_exit = (
            self.safety_exit_rho_min <= target_rho <= self.safety_exit_rho_max
            and self.safety_exit_z_min <= target_z <= self.safety_exit_z_max
            and abs(target_yaw) <= self.safety_exit_yaw_max
        )

        if self.control_state is None:
            self.set_control_state("ARM_TRACK" if inside_enter else "RETURN_HOME")
        elif self.control_state == "ARM_TRACK" and not inside_exit:
            self.set_control_state("RETURN_HOME")
        elif self.control_state == "RETURN_HOME" and inside_enter:
            self.set_control_state("ARM_TRACK")

    def set_control_state(self, state: str) -> None:
        if self.control_state == state:
            return
        previous = self.control_state or "UNINITIALIZED"
        self.control_state = state
        self.previous_velocity = [0.0] * 6
        self.state_pub.publish(String(data=state))
        self.get_logger().warn(f"Control state changed: {previous} -> {state}")
        if state == "RETURN_HOME" and not self.is_holding_object() and not self.force_safety_pose:
            self._reset_pick_phase()

    def is_holding_object(self) -> bool:
        if self.task_mode == "PLACE":
            return self.grasp_phase not in ("HOLD",)
        return self.grasp_phase in ("GRASP", "LIFT", "HOLD")

    def compute_home_velocities(self) -> list[float]:
        velocities = []
        for index, joint_name in enumerate(self.joint_names):
            error = self.home_positions[index] - self.current_joints[joint_name]
            if abs(error) <= self.home_position_tolerance:
                velocities.append(0.0)
            else:
                velocities.append(
                    clamp(
                        self.home_kp * error,
                        -self.home_max_joint_velocity,
                        self.home_max_joint_velocity,
                    )
                )
        return velocities

    def compute_base_twist(
        self,
        target_x: float,
        target_y: float,
        arm_target_rho: float,
        base_scale: float,
    ) -> Twist:
        twist = Twist()
        if not self.enable_base_motion or base_scale <= 0.0:
            return twist

        yaw_error = atan2(target_y, target_x)
        twist.angular.z = base_scale * clamp(
            self.k_base_yaw * yaw_error,
            -self.max_base_yaw_rate,
            self.max_base_yaw_rate,
        )
        if arm_target_rho > self.base_stop_rho:
            heading_scale = max(0.0, cos(yaw_error))
            nominal_linear = clamp(
                self.k_base_linear * (arm_target_rho - self.base_stop_rho),
                0.0,
                self.max_base_linear,
            )
            twist.linear.x = base_scale * heading_scale * nominal_linear
        return twist

    def compute_switching(
        self,
        arm_target_rho: float,
        switching_rho: float,
        switching_alpha: float,
    ) -> float:
        mu = 0.5 * (
            1.0
            - tanh(
                switching_alpha
                * (arm_target_rho - switching_rho)
            )
        )
        return clamp(mu, 0.0, 1.0)

    def is_transition_target_safe(
        self,
        target_z: float,
        target_yaw: float,
    ) -> bool:
        return (
            self.safety_exit_z_min <= target_z <= self.safety_exit_z_max
            and abs(target_yaw) <= self.safety_exit_yaw_max
        )

    def integrate_position_command(
        self,
        velocity: list[float],
        dt: float,
    ) -> None:
        if self.position_command is None:
            return
        for index in range(6):
            next_position = self.position_command[index] + velocity[index] * dt
            lower, upper = self.active_joint_bounds(index)
            self.position_command[index] = clamp(
                next_position,
                lower,
                upper,
            )

    def active_motion_stage(self) -> str:
        if self.task_mode != "PICK":
            return "POSITION"
        if self.grasp_phase == "DESCEND":
            return "DESCEND"
        if self.grasp_phase in ("GRASP", "LIFT", "HOLD"):
            return "PREGRASP"
        if self.wrist_position_recovery:
            return "RECOVER"
        if self.pick_approach_stage == "WRIST":
            return "WRIST"
        if self.pick_approach_stage in ("PREGRASP", "HOLD"):
            return "PREGRASP"
        return "POSITION"

    def active_joint_bounds(self, index: int) -> tuple[float, float]:
        lower = float(self.joint_lower_limits[index])
        upper = float(self.joint_upper_limits[index])
        wrist_position_compensation_joint = (
            str(getattr(self, "pick_approach_stage", "POSITION")).upper()
            in ("WRIST", "PREGRASP", "HOLD")
            and index in (1, 2)
        )
        if (
            self.control_state == "ARM_TRACK"
            and self.task_mode == "PICK"
            and self.pick_start_positions is not None
            and index < len(self.pick_max_joint_excursion)
            and not wrist_position_compensation_joint
        ):
            excursion = float(self.pick_max_joint_excursion[index])
            lower = max(lower, float(self.pick_start_positions[index]) - excursion)
            upper = min(upper, float(self.pick_start_positions[index]) + excursion)
        return lower, upper

    def apply_stage_velocity_limits(self, velocity: list[float]) -> list[float]:
        stage = self.active_motion_stage()
        yaw_limit, joint2_limit, joint3_limit, wrist_limit = self.stage_velocity_limits[stage]
        limits = [
            yaw_limit,
            joint2_limit,
            joint3_limit,
            wrist_limit,
            wrist_limit,
            wrist_limit,
        ]
        return [
            clamp(float(value), -float(limit), float(limit))
            for value, limit in zip(velocity, limits)
        ]

    def apply_joint_limit_slowdown(self, velocity: list[float]) -> list[float]:
        if self.position_command is None:
            return list(velocity)
        limited = list(velocity)
        for index, value in enumerate(limited):
            lower, upper = self.active_joint_bounds(index)
            position = float(self.position_command[index])
            margin = min(
                self.joint_soft_limit_margin,
                max(0.0, (upper - lower) * 0.25),
            )
            if margin <= 1e-9:
                continue
            if value < 0.0:
                distance = position - lower
                scale = clamp(distance / margin, 0.0, 1.0)
                limited[index] = value * scale
            elif value > 0.0:
                distance = upper - position
                scale = clamp(distance / margin, 0.0, 1.0)
                limited[index] = value * scale
        return limited

    def current_stage_acceleration(self) -> float:
        stage = self.active_motion_stage()
        return min(
            self.max_joint_acceleration,
            max(0.0, self.stage_acceleration_limits[stage]),
        )

    def compute_joint_velocities(
        self,
        yaw_error: float,
        rho_error: float,
        z_error: float,
        q2: float,
        q3: float,
    ) -> list[float]:
        q1_dot = 0.0
        if abs(yaw_error) > self.yaw_tolerance:
            q1_dot = self.joint1_sign * clamp(
                self.k_yaw * yaw_error,
                -self.max_yaw_velocity,
                self.max_yaw_velocity,
            )

        if abs(rho_error) <= self.rho_tolerance and abs(z_error) <= self.z_tolerance:
            return [q1_dot, 0.0, 0.0]

        approach_error = hypot(rho_error, z_error)
        approach_scale = clamp(
            approach_error / self.approach_slowdown_distance,
            self.approach_min_scale,
            1.0,
        )
        v_rho = approach_scale * clamp(
            self.k_rho * rho_error,
            -self.max_rho_velocity,
            self.max_rho_velocity,
        )
        v_z = approach_scale * clamp(
            self.k_z * z_error,
            -self.max_z_velocity,
            self.max_z_velocity,
        )

        # Dynamic analytical Jacobian
        q2_phys = q2 * self.joint2_sign
        q3_phys = q3 * self.joint3_sign

        # Correct row assignment: Row 1 = d_rho (cos), Row 2 = d_z (-sin)
        j11 = self.link1 * cos(q2_phys) + self.link2 * cos(q2_phys + q3_phys)
        j12 = self.link2 * cos(q2_phys + q3_phys)
        j21 = -self.link1 * sin(q2_phys) - self.link2 * sin(q2_phys + q3_phys)
        j22 = -self.link2 * sin(q2_phys + q3_phys)

        determinant = j11 * j22 - j12 * j21

        if abs(determinant) < self.singularity_epsilon:
            return [q1_dot, 0.0, 0.0]

        q2_dot_phys = (j22 * v_rho - j12 * v_z) / determinant
        q3_dot_phys = (-j21 * v_rho + j11 * v_z) / determinant
        
        q2_dot = q2_dot_phys * self.joint2_sign
        q3_dot = q3_dot_phys * self.joint3_sign

        # Preserve the joint2/joint3 velocity ratio so the end effector keeps
        # the requested rho-z direction when either joint reaches its limit.
        velocity_scale = min(
            1.0,
            self.max_joint2_velocity / max(abs(q2_dot), 1e-9),
            self.max_joint3_velocity / max(abs(q3_dot), 1e-9),
        )
        return [
            q1_dot,
            q2_dot * velocity_scale,
            q3_dot * velocity_scale,
        ]

    def limit_acceleration(
        self,
        desired_velocity: list[float],
        dt: float,
        max_acceleration: float | None = None,
    ) -> list[float]:
        acceleration = (
            self.max_joint_acceleration
            if max_acceleration is None
            else max(0.0, float(max_acceleration))
        )
        max_delta = acceleration * dt
        limited = []
        for previous, desired in zip(self.previous_velocity, desired_velocity):
            limited.append(previous + clamp(desired - previous, -max_delta, max_delta))
        self.previous_velocity = limited
        return limited

    def publish_position_command(self) -> None:
        if self.position_command is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names) + self.gripper_joint_names
        msg.position = list(self.position_command) + [
            self.gripper_position
        ] * len(self.gripper_joint_names)
        self.position_pub.publish(msg)

    # ── Pick/place sequence logic ─────────────────────────────────
    def _update_task_phase(
        self,
        pos_aligned: bool,
        wrist_aligned: bool,
        dt: float,
    ) -> None:
        phase_aligned = pos_aligned and wrist_aligned
        if self.task_mode == "PLACE":
            self._update_place_phase(phase_aligned, dt)
        else:
            self._update_pick_phase(pos_aligned, wrist_aligned, dt)

    def _update_pick_phase(
        self,
        pos_aligned: bool,
        wrist_aligned: bool,
        dt: float,
    ) -> None:
        """Drive the pick APPROACH → DESCEND → GRASP → LIFT → HOLD sequence."""
        if self.grasp_phase == "APPROACH":
            self.gripper_position = self.gripper_open_position
            if self.enable_staged_top_down_approach:
                if self.pick_approach_stage == "WRIST" and pos_aligned and wrist_aligned:
                    self.pick_approach_stage = "PREGRASP"
                    self.previous_velocity = [0.0] * 6
                    self.get_logger().warn(
                        "Pick approach stage: WRIST -> PREGRASP; descending with aligned wrist"
                    )
                    return
                if self.pick_approach_stage == "PREGRASP":
                    if self.grasp_z_offset > self.grasp_pregrasp_clearance_z:
                        self.grasp_z_offset = max(
                            self.grasp_pregrasp_clearance_z,
                            self.grasp_z_offset - self.grasp_descend_speed * dt,
                        )
                        return
                    if pos_aligned and wrist_aligned:
                        if (
                            self.require_descend_confirmation
                            and not self.descend_confirmed
                        ):
                            self.enter_pregrasp_hold()
                            return
                        self.grasp_phase = "DESCEND"
                        self.previous_velocity = [0.0] * 6
                        self.get_logger().warn(
                            "Grasp phase: PREGRASP -> DESCEND"
                        )
                    return
            elif pos_aligned and wrist_aligned:
                self.grasp_phase = "DESCEND"
                self.grasp_z_offset, _ = pick_z_offsets(
                    self.grasp_offset_z,
                    self.grasp_descend_depth,
                    self.grasp_approach_clearance_z,
                )
                self.get_logger().warn("Grasp phase: APPROACH → DESCEND")

        elif self.grasp_phase == "DESCEND":
            self.gripper_position = self.gripper_open_position
            # Gradually reduce the z offset to lower the arm
            if self.enable_staged_top_down_approach:
                descend_target_offset = self.grasp_final_offset_z
            else:
                _, descend_target_offset = pick_z_offsets(
                    self.grasp_offset_z,
                    self.grasp_descend_depth,
                    self.grasp_approach_clearance_z,
                )
            if self.grasp_z_offset > descend_target_offset:
                self.grasp_z_offset = max(
                    descend_target_offset,
                    self.grasp_z_offset - self.grasp_descend_speed * dt,
                )
                return

            if pos_aligned:
                self.grasp_phase = "GRASP"
                self.grasp_phase_start_time = self.get_clock().now()
                self.get_logger().warn("Grasp phase: DESCEND → GRASP")

        elif self.grasp_phase == "GRASP":
            self.gripper_position = self.gripper_close_position
            elapsed = (
                self.get_clock().now() - self.grasp_phase_start_time
            ).nanoseconds * 1e-9
            if elapsed >= self.grasp_close_duration:
                self.grasp_phase = "LIFT"
                self.lift_z_accumulated = 0.0
                self.get_logger().warn("Grasp phase: GRASP → LIFT")

        elif self.grasp_phase == "LIFT":
            self.gripper_position = self.gripper_close_position
            if self.lift_z_accumulated < self.grasp_lift_height:
                lift_step = min(
                    self.grasp_lift_speed * dt,
                    self.grasp_lift_height - self.lift_z_accumulated,
                )
                self.grasp_z_offset += lift_step
                self.lift_z_accumulated += lift_step
                return

            if pos_aligned:
                self.grasp_phase = "HOLD"
                if self.return_home_after_pick:
                    self.disable_target_tracking()
                    self.force_safety_pose = True
                    self.set_control_state("RETURN_HOME")
                    self.get_logger().warn(
                        "Grasp phase: LIFT → HOLD; returning to home pose"
                    )
                    return
                self.get_logger().warn("Grasp phase: LIFT → HOLD")

        elif self.grasp_phase == "HOLD":
            self.gripper_position = self.gripper_close_position

    def _update_place_phase(self, pos_aligned: bool, dt: float) -> None:
        """Drive the place APPROACH → DESCEND → RELEASE → RETREAT → HOLD sequence."""
        if self.grasp_phase == "APPROACH":
            self.gripper_position = self.gripper_close_position
            if pos_aligned:
                self.grasp_phase = "DESCEND"
                self.grasp_z_offset = self.place_offset_z
                self.get_logger().warn("Place phase: APPROACH → DESCEND")

        elif self.grasp_phase == "DESCEND":
            self.gripper_position = self.gripper_close_position
            release_offset = self.place_offset_z - self.place_descend_depth
            if self.grasp_z_offset > release_offset:
                self.grasp_z_offset = max(
                    release_offset,
                    self.grasp_z_offset - self.grasp_descend_speed * dt,
                )
                return

            if pos_aligned:
                self.grasp_phase = "RELEASE"
                self.grasp_phase_start_time = self.get_clock().now()
                self.get_logger().warn("Place phase: DESCEND → RELEASE")

        elif self.grasp_phase == "RELEASE":
            elapsed = (
                self.get_clock().now() - self.grasp_phase_start_time
            ).nanoseconds * 1e-9
            open_ratio = clamp(
                elapsed / max(self.place_open_duration, 1e-3),
                0.0,
                1.0,
            )
            self.gripper_position = (
                self.gripper_close_position
                + (self.gripper_open_position - self.gripper_close_position) * open_ratio
            )
            if elapsed >= self.place_open_duration:
                self.gripper_position = self.gripper_open_position
                self.grasp_phase = "RETREAT"
                self.get_logger().warn("Place phase: RELEASE → RETREAT")

        elif self.grasp_phase == "RETREAT":
            self.gripper_position = self.gripper_open_position
            if self.grasp_z_offset < self.place_offset_z:
                self.grasp_z_offset = min(
                    self.place_offset_z,
                    self.grasp_z_offset + self.grasp_lift_speed * dt,
                )
                return

            if pos_aligned:
                self.grasp_phase = "HOLD"
                if self.return_home_after_place:
                    self.force_safety_pose = True
                    self.set_control_state("RETURN_HOME")
                    self.get_logger().warn(
                        "Place phase: RETREAT → HOLD; returning to safety pose"
                    )
                    return
                self.get_logger().warn("Place phase: RETREAT → HOLD")

        elif self.grasp_phase == "HOLD":
            self.gripper_position = self.gripper_open_position

    def _reset_pick_phase(self) -> None:
        """Reset pick sequence before an object is held."""
        if self.grasp_phase != "APPROACH":
            self.get_logger().warn(
                f"Grasp phase reset: {self.grasp_phase} → APPROACH"
            )
        self.task_mode = "PICK"
        self.grasp_phase = "APPROACH"
        self.pick_approach_stage = "POSITION"
        self.descend_confirmed = False
        self.pregrasp_hold_positions = None
        self.grasp_z_offset, _ = pick_z_offsets(
            self.grasp_offset_z,
            self.grasp_descend_depth,
            self.grasp_approach_clearance_z,
        )
        if self.enable_staged_top_down_approach:
            self.grasp_z_offset = self.grasp_safe_clearance_z
        self.gripper_position = self.gripper_open_position
        self.lift_z_accumulated = 0.0

    def on_log_timer(self) -> None:
        if self.control_state is not None:
            self.state_pub.publish(String(data=self.control_state))
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmYawRhoZPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
