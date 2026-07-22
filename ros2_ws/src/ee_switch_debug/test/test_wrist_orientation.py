from math import cos, pi, sin
from pathlib import Path
from types import SimpleNamespace

import ee_switch_debug.arm_yaw_rho_z_position_controller as controller
import numpy as np
import pytest
import rclpy
from std_msgs.msg import String

from ee_switch_debug.arm_yaw_rho_z_position_controller import (
    compute_wrist_targets_from_orientation,
    fixed_target_joint_velocities,
    is_wrist_aligned_for_phase,
    measured_pregrasp_pose_valid,
    semantic_stage_joint_tolerance,
    semantic_stage_trajectory_profile,
    semantic_top_down_stage_goal,
    top_down_stage_mask,
    rotation_error_angle,
    update_pick_approach_stage,
    update_wrist_position_recovery,
    vertical_lift_clearance_reached,
)


def test_top_down_stages_move_only_the_selected_joint_group():
    assert top_down_stage_mask("APPROACH") == [True] * 6
    assert top_down_stage_mask("YAW") == [True, False, False, False, False, False]
    assert top_down_stage_mask("ARM") == [False, True, True, False, False, False]
    assert top_down_stage_mask("WRIST") == [False, False, False, True, True, True]
    assert top_down_stage_mask("DESCEND") == [True] * 6
    assert top_down_stage_mask("HOME") == [True] * 6


def test_group_approach_modes_are_mutually_exclusive():
    assert controller.configured_group_approach_mode(False, False) == "disabled"
    assert controller.configured_group_approach_mode(True, False) == "sequential"
    assert controller.configured_group_approach_mode(False, True) == "blended"
    with pytest.raises(ValueError, match="mutually exclusive"):
        controller.configured_group_approach_mode(True, True)


def test_group_sequential_stage_goals_and_masks():
    plan = SimpleNamespace(
        yaw_joints=np.arange(6),
        arm_joints=np.arange(6) + 10,
        pregrasp_joints=np.arange(6) + 20,
    )

    assert np.array_equal(
        semantic_top_down_stage_goal("YAW", plan, []),
        plan.yaw_joints,
    )
    assert np.array_equal(
        semantic_top_down_stage_goal("ARM_POSITION", plan, []),
        plan.arm_joints,
    )
    assert np.array_equal(
        semantic_top_down_stage_goal("WRIST_ALIGN", plan, []),
        plan.pregrasp_joints,
    )
    assert top_down_stage_mask("YAW") == [True, False, False, False, False, False]
    assert top_down_stage_mask("ARM_POSITION") == [False, True, True, False, False, False]
    assert top_down_stage_mask("WRIST_ALIGN") == [False, True, True, True, True, True]


def test_measured_pregrasp_pose_requires_position_and_orientation():
    pregrasp = np.array([0.1, -1.0, 1.5, -0.5, 1.2, -0.2])
    _, selected_rotation = controller.forward_kinematics(pregrasp)
    plan = SimpleNamespace(
        pregrasp_joints=pregrasp,
        selected_rotation=selected_rotation,
    )

    assert measured_pregrasp_pose_valid(pregrasp, plan, 0.01, 0.04)
    assert not measured_pregrasp_pose_valid(
        pregrasp + np.array([0.0, 0.2, 0.0, 0.0, 0.0, 0.0]),
        plan,
        0.01,
        0.04,
    )


def test_fixed_target_velocity_uses_measured_error_and_holds_other_groups():
    velocity, aligned = fixed_target_joint_velocities(
        current=[0.0] * 6,
        target=[0.2, 0.3, -0.4, 0.5, -0.6, 0.7],
        active_mask=top_down_stage_mask("ARM"),
        kp=2.0,
        velocity_limits=[0.25, 0.4, 0.4, 0.3, 0.3, 0.3],
        tolerance=0.02,
    )

    assert velocity == [0.0, 0.4, -0.4, 0.0, 0.0, 0.0]
    assert not aligned


def test_fixed_target_stage_completes_only_after_measured_joints_arrive():
    target = [0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    current = [9.0, 0.295, -0.395, -9.0, 9.0, -9.0]

    velocity, aligned = fixed_target_joint_velocities(
        current=current,
        target=target,
        active_mask=top_down_stage_mask("ARM"),
        kp=2.0,
        velocity_limits=[0.25, 0.4, 0.4, 0.3, 0.3, 0.3],
        tolerance=0.02,
    )

    assert velocity == [0.0] * 6
    assert aligned


def test_precomputed_plan_is_latched_once_for_an_active_pick(monkeypatch):
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_plan = None
    node.top_down_plan_attempted = False
    node.top_down_plan_error = ""
    node.current_joints = {
        name: value
        for name, value in zip(
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            [0.0, -1.57, 2.09, -0.52, 1.57, 0.0],
        )
    }
    node.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    node.joint_lower_limits = [-3.14] * 6
    node.joint_upper_limits = [3.14] * 6
    node.pick_start_positions = [0.0] * 6
    node.pick_max_joint_excursion = [3.0] * 6
    node.top_down_pregrasp_clearance = 0.10
    node.top_down_waypoint_spacing = 0.02
    node.top_down_stage = "WAIT_TARGET"
    node.top_down_waypoint_index = 99
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: None, "error": lambda self, message: None}
    )()
    calls = []

    def fake_plan(*args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(
            success=True,
            pregrasp_joints=[0.1] * 6,
            descent_waypoints=[[0.2] * 6],
            message="planned",
        )

    monkeypatch.setattr(controller, "plan_top_down_sequence", fake_plan)

    assert node.ensure_precomputed_top_down_plan([0.3, 0.0, 0.4], np.eye(3))
    assert node.ensure_precomputed_top_down_plan([9.0, 9.0, 9.0], np.eye(3))
    assert len(calls) == 1
    assert node.top_down_stage == "YAW"
    assert node.top_down_waypoint_index == 0


def test_blended_plan_is_selected_once_and_starts_synchronized_approach(monkeypatch):
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_plan = None
    node.top_down_plan_attempted = False
    node.top_down_plan_error = ""
    node.current_joints = {
        name: value
        for name, value in zip(
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            [0.0, -1.57, 2.09, -0.52, 1.57, 0.0],
        )
    }
    node.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    node.joint_lower_limits = [-3.14] * 6
    node.joint_upper_limits = [3.14] * 6
    node.pick_start_positions = [0.0] * 6
    node.pick_max_joint_excursion = [3.0] * 6
    node.top_down_pregrasp_clearance = 0.10
    node.top_down_waypoint_spacing = 0.01
    node.top_down_blend_orientation_during_descent = True
    node.top_down_radial_inward_offset = 0.025
    node.top_down_approach_joint_step = 0.12
    node.top_down_minimum_approach_clearance = 0.08
    node.top_down_stage = "WAIT_TARGET"
    node.top_down_waypoint_index = 99
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: None, "error": lambda self, message: None}
    )()
    calls = []

    def fake_blended_plan(*args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(
            success=True,
            pregrasp_joints=[0.1] * 6,
            approach_waypoints=[[0.05] * 6, [0.1] * 6],
            descent_waypoints=[[0.2] * 6],
            message="planned",
        )

    monkeypatch.setattr(controller, "plan_blended_top_down_sequence", fake_blended_plan)
    monkeypatch.setattr(
        controller,
        "plan_top_down_sequence",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("strict planner called")),
    )

    assert node.ensure_precomputed_top_down_plan([0.3, 0.0, 0.4], np.eye(3))
    assert node.ensure_precomputed_top_down_plan([9.0, 9.0, 9.0], np.eye(3))
    assert len(calls) == 1
    assert calls[0][1]["radial_inward_offset"] == 0.025
    assert node.top_down_stage == "APPROACH"
    assert node.top_down_waypoint_index == 0


def test_fixed_orientation_plan_is_converted_to_group_sequence_once(monkeypatch):
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_plan = None
    node.top_down_plan_attempted = False
    node.top_down_plan_error = ""
    node.current_joints = {
        name: value
        for name, value in zip(
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            [0.0, -1.57, 2.09, -0.52, 1.57, 0.0],
        )
    }
    node.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    node.joint_lower_limits = [-3.14] * 6
    node.joint_upper_limits = [3.14] * 6
    node.pick_start_positions = [0.0] * 6
    node.pick_max_joint_excursion = [3.0] * 6
    node.top_down_use_fixed_reachable_orientation = True
    node.top_down_use_group_sequential_approach = True
    node.top_down_blend_orientation_during_descent = False
    node.top_down_pregrasp_clearance = 0.10
    node.top_down_radial_inward_offset = 0.025
    node.top_down_minimum_orientation_fraction = 0.50
    node.top_down_orientation_search_steps = 20
    node.top_down_descend_max_xy_deviation = 0.015
    node.top_down_descend_max_orientation_deviation = 0.035
    node.top_down_minimum_approach_clearance = 0.08
    node.top_down_stage = "WAIT_TARGET"
    node.top_down_waypoint_index = 99
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: None, "error": lambda self, message: None}
    )()
    calls = []
    group_calls = []

    def fake_fixed_plan(*args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(
            success=True,
            pregrasp_joints=np.full(6, 0.1),
            grasp_joints=np.full(6, 0.2),
            approach_waypoints=[],
            descent_waypoints=[],
            orientation_fraction=0.8,
            message="planned",
        )

    monkeypatch.setattr(controller, "plan_fixed_orientation_top_down_sequence", fake_fixed_plan)
    def fake_group_plan(plan, **kwargs):
        group_calls.append((plan, kwargs))
        plan.yaw_joints = np.full(6, 0.03)
        plan.arm_joints = np.full(6, 0.06)
        return plan

    monkeypatch.setattr(controller, "build_group_sequential_approach", fake_group_plan)
    monkeypatch.setattr(
        controller,
        "plan_blended_top_down_sequence",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("blended planner called")),
    )

    assert node.ensure_precomputed_top_down_plan([0.3, 0.0, 0.4], np.eye(3))
    assert node.ensure_precomputed_top_down_plan([9.0, 9.0, 9.0], np.eye(3))
    assert len(calls) == 1
    assert len(group_calls) == 1
    assert group_calls[0][1]["minimum_clearance"] == 0.08
    assert calls[0][1]["minimum_orientation_fraction"] == 0.50
    assert node.top_down_stage == "YAW"


def test_fixed_orientation_plan_is_converted_to_blended_group_path_once(
    monkeypatch,
):
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_plan = None
    node.top_down_plan_attempted = False
    node.top_down_plan_error = ""
    node.current_joints = {
        name: value
        for name, value in zip(
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
            [0.0, -1.57, 2.09, -0.52, 1.57, 0.0],
        )
    }
    node.joint_names = list(node.current_joints)
    node.joint_lower_limits = [-3.14] * 6
    node.joint_upper_limits = [3.14] * 6
    node.pick_start_positions = [0.0] * 6
    node.pick_max_joint_excursion = [3.0] * 6
    node.top_down_use_fixed_reachable_orientation = True
    node.top_down_use_group_sequential_approach = False
    node.top_down_use_group_blended_approach = True
    node.top_down_blend_orientation_during_descent = False
    node.top_down_pregrasp_clearance = 0.10
    node.top_down_radial_inward_offset = 0.025
    node.top_down_minimum_orientation_fraction = 0.50
    node.top_down_orientation_search_steps = 20
    node.top_down_descend_max_xy_deviation = 0.015
    node.top_down_descend_max_orientation_deviation = 0.035
    node.top_down_minimum_approach_clearance = 0.08
    node.top_down_alignment_velocity_limits = [0.7] * 6
    node.top_down_stage_acceleration = 1.6
    node.top_down_segment_min_duration = 0.6
    node.top_down_stage = "WAIT_TARGET"
    node.top_down_waypoint_index = 99
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda *_args: None,
        error=lambda *_args: None,
    )
    fixed_calls = []
    blended_calls = []

    def fake_fixed_plan(*args, **kwargs):
        fixed_calls.append((args, kwargs))
        return SimpleNamespace(
            success=True,
            pregrasp_joints=np.full(6, 0.1),
            grasp_joints=np.full(6, 0.2),
            approach_waypoints=[],
            descent_waypoints=[],
            orientation_fraction=0.8,
            message="planned",
        )

    def fake_blended_group_plan(plan, **kwargs):
        blended_calls.append((plan, kwargs))
        plan.blended_approach = SimpleNamespace(duration=1.0)
        return plan

    monkeypatch.setattr(
        controller,
        "plan_fixed_orientation_top_down_sequence",
        fake_fixed_plan,
    )
    monkeypatch.setattr(
        controller,
        "build_group_blended_approach",
        fake_blended_group_plan,
        raising=False,
    )
    monkeypatch.setattr(
        controller,
        "build_group_sequential_approach",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("sequential fallback called")
        ),
    )

    assert node.ensure_precomputed_top_down_plan([0.3, 0.0, 0.4], np.eye(3))
    assert node.ensure_precomputed_top_down_plan([9.0, 9.0, 9.0], np.eye(3))
    assert len(fixed_calls) == 1
    assert len(blended_calls) == 1
    assert blended_calls[0][1]["minimum_clearance"] == 0.08
    assert node.top_down_stage == "BLENDED_APPROACH"


def test_semantic_stages_use_only_motion_endpoints():
    plan = SimpleNamespace(
        pregrasp_joints=np.full(6, 0.1),
        grasp_joints=np.full(6, 0.2),
    )
    home = np.full(6, -0.3)

    assert np.array_equal(semantic_top_down_stage_goal("APPROACH", plan, home), plan.pregrasp_joints)
    assert np.array_equal(semantic_top_down_stage_goal("DESCEND", plan, home), plan.grasp_joints)
    assert np.array_equal(semantic_top_down_stage_goal("LIFT", plan, home), plan.pregrasp_joints)
    assert np.array_equal(semantic_top_down_stage_goal("HOME", plan, home), home)


def test_semantic_stage_tolerance_keeps_descend_strict():
    for stage in ("YAW", "ARM_POSITION", "WRIST_ALIGN", "DESCEND"):
        assert semantic_stage_joint_tolerance(stage, 0.025, 0.050) == 0.025
    for stage in ("APPROACH", "LIFT", "HOME"):
        assert semantic_stage_joint_tolerance(stage, 0.025, 0.050) == 0.050
    assert semantic_stage_joint_tolerance("UNKNOWN", 0.025, 0.050) == 0.025


def test_semantic_stage_trajectory_profile_keeps_descend_conservative():
    for stage in ("APPROACH", "LIFT", "HOME"):
        assert semantic_stage_trajectory_profile(stage) == "compact"
    assert semantic_stage_trajectory_profile("DESCEND") == "quintic"
    assert semantic_stage_trajectory_profile("UNKNOWN") == "quintic"


def test_vertical_lift_clearance_uses_measured_height_boundary():
    assert not vertical_lift_clearance_reached(0.559, 0.500, 0.060)
    assert vertical_lift_clearance_reached(0.560, 0.500, 0.060)
    assert vertical_lift_clearance_reached(0.700, 0.500, 0.060)


def test_stage_transition_preserves_velocity_only_when_requested():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_stage = "LIFT"
    node.pick_approach_stage = "LIFT"
    node.previous_velocity = [0.1] * 6
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    node.clear_top_down_segment_state = lambda: None

    node.advance_precomputed_top_down_stage("HOME", preserve_velocity=True)

    assert node.previous_velocity == [0.1] * 6

    node.advance_precomputed_top_down_stage("DESCEND")

    assert node.previous_velocity == [0.0] * 6


def test_measured_lift_home_clearance_uses_forward_kinematics(monkeypatch):
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.current_joints = dict.fromkeys(node.joint_names, 0.1)
    node.top_down_plan = SimpleNamespace(grasp_joints=np.zeros(6))
    node.top_down_lift_home_clearance = 0.06

    def fake_forward_kinematics(joints):
        z = 0.500 if np.allclose(joints, np.zeros(6)) else 0.560
        return np.array([0.0, 0.0, z]), np.eye(3)

    monkeypatch.setattr(controller, "forward_kinematics", fake_forward_kinematics)

    assert node.measured_lift_home_clearance_reached()


def make_semantic_segment_node():
    class FakeTime:
        def __sub__(self, _other):
            return SimpleNamespace(nanoseconds=0)

    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.current_joints = dict.fromkeys(node.joint_names, 0.0)
    node.position_command = [0.0] * 6
    node.top_down_segment_stage = ""
    node.top_down_segment_start = None
    node.top_down_segment_goal = None
    node.top_down_segment_start_time = None
    node.top_down_segment_duration = 0.0
    node.top_down_segment_planned_complete = False
    node.top_down_segment_min_duration = 0.6
    node.top_down_stage_acceleration = 1.6
    node.top_down_alignment_velocity_limits = [1.0] * 6
    node.top_down_path_velocity_limits = [0.5] * 6
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime())
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    return node


def test_semantic_segment_uses_compact_profile_for_approach(monkeypatch):
    compact_samples = []
    quintic_samples = []
    monkeypatch.setattr(
        controller,
        "minimum_compact_duration",
        lambda *_args, **_kwargs: 2.0,
    )
    monkeypatch.setattr(
        controller,
        "minimum_quintic_duration",
        lambda *_args, **_kwargs: 3.0,
    )
    monkeypatch.setattr(
        controller,
        "sample_compact_joint_positions",
        lambda start, *_args: (
            compact_samples.append("APPROACH") or np.asarray(start),
            False,
        ),
    )
    monkeypatch.setattr(
        controller,
        "sample_quintic_joint_positions",
        lambda start, *_args: (
            quintic_samples.append("APPROACH") or np.asarray(start),
            False,
        ),
    )
    node = make_semantic_segment_node()

    assert not node.command_semantic_top_down_segment(
        [0.2] * 6, "APPROACH", 0.1
    )
    assert node.top_down_segment_duration == 2.0
    assert compact_samples == ["APPROACH"]
    assert quintic_samples == []


def test_semantic_segment_keeps_quintic_profile_for_descend(monkeypatch):
    compact_samples = []
    quintic_samples = []
    monkeypatch.setattr(
        controller,
        "minimum_compact_duration",
        lambda *_args, **_kwargs: 2.0,
    )
    monkeypatch.setattr(
        controller,
        "minimum_quintic_duration",
        lambda *_args, **_kwargs: 3.0,
    )
    monkeypatch.setattr(
        controller,
        "sample_compact_joint_positions",
        lambda start, *_args: (
            compact_samples.append("DESCEND") or np.asarray(start),
            False,
        ),
    )
    monkeypatch.setattr(
        controller,
        "sample_quintic_joint_positions",
        lambda start, *_args: (
            quintic_samples.append("DESCEND") or np.asarray(start),
            False,
        ),
    )
    node = make_semantic_segment_node()

    assert not node.command_semantic_top_down_segment(
        [0.2] * 6, "DESCEND", 0.1
    )
    assert node.top_down_segment_duration == 3.0
    assert compact_samples == []
    assert quintic_samples == ["DESCEND"]


def test_semantic_lift_records_bounded_sample_velocity(monkeypatch):
    node = make_semantic_segment_node()
    monkeypatch.setattr(
        controller,
        "minimum_compact_duration",
        lambda *_args, **_kwargs: 2.0,
    )
    monkeypatch.setattr(
        controller,
        "sample_compact_joint_positions",
        lambda *_args: (np.full(6, 0.2), False),
    )

    assert not node.command_semantic_top_down_segment([0.4] * 6, "LIFT", 0.1)

    assert np.allclose(node.previous_velocity, [0.5] * 6)
    assert np.allclose(node.position_command, [0.2] * 6)


def test_continuous_home_inherits_velocity_and_acceleration_limits():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.current_joints = dict.fromkeys(node.joint_names, 0.0)
    node.position_command = [0.0] * 6
    node.previous_velocity = [0.2] * 6
    node.top_down_joint_kp = 2.0
    node.top_down_transit_joint_tolerance = 0.05
    node.top_down_stage_acceleration = 0.5
    node.top_down_alignment_velocity_limits = [1.0] * 6
    node.apply_joint_limit_slowdown = lambda velocity: velocity

    def integrate(velocity, dt):
        node.position_command = [
            position + speed * dt
            for position, speed in zip(node.position_command, velocity)
        ]

    node.integrate_position_command = integrate

    assert not node.command_continuous_top_down_home([-1.0] * 6, 0.1)
    assert np.allclose(node.previous_velocity, [0.15] * 6)
    assert all(position > 0.0 for position in node.position_command)


def test_lift_clearance_switches_to_continuous_home_before_full_lift():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_stage = "LIFT"
    node.grasp_phase = "LIFT"
    node.return_home_after_pick = True
    node.gripper_close_position = 0.8
    node.gripper_open_position = 0.0
    node.top_down_plan = SimpleNamespace(
        pregrasp_joints=np.zeros(6),
        grasp_joints=np.zeros(6),
    )
    node.home_positions = [0.0] * 6
    node.measured_lift_home_clearance_reached = lambda: True
    calls = []
    node.advance_precomputed_top_down_stage = (
        lambda next_stage, preserve_velocity=False: calls.append(
            (next_stage, preserve_velocity)
        )
    )
    node.command_semantic_top_down_segment = lambda *_args: (_ for _ in ()).throw(
        AssertionError("full LIFT segment must not continue after clearance")
    )

    node.run_semantic_fixed_orientation_pick(0.1)

    assert node.top_down_continuous_home
    assert node.grasp_phase == "HOME"
    assert calls == [("HOME", True)]


def make_group_sequence_node(stage):
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_stage = stage
    node.grasp_phase = "APPROACH"
    node.gripper_close_position = 0.8
    node.gripper_open_position = 0.0
    node.top_down_plan = SimpleNamespace(
        yaw_joints=np.zeros(6),
        arm_joints=np.zeros(6),
        pregrasp_joints=np.zeros(6),
        grasp_joints=np.zeros(6),
        selected_rotation=np.eye(3),
    )
    node.home_positions = [0.0] * 6
    node.current_joints = {
        f"joint{index}": 0.0 for index in range(1, 7)
    }
    node.joint_names = list(node.current_joints)
    node.return_home_after_pick = True
    node.top_down_continuous_home = False
    node.top_down_segment_duration = 0.0
    node.publish_position_command = lambda: None
    node.publish_task_state = lambda: None
    node.disable_target_tracking = lambda: None
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    return node


def test_group_sequence_transitions_are_strictly_ordered():
    transitions = (
        ("YAW", "ARM_POSITION"),
        ("ARM_POSITION", "WRIST_ALIGN"),
        ("WRIST_ALIGN", "PREGRASP_VERIFY"),
    )
    for stage, expected in transitions:
        node = make_group_sequence_node(stage)
        commanded_stages = []
        node.command_semantic_top_down_segment = (
            lambda _goal, commanded_stage, _dt: (
                commanded_stages.append(commanded_stage) or True
            )
        )
        node.advance_precomputed_top_down_stage = (
            lambda next_stage, preserve_velocity=False: setattr(
                node, "top_down_stage", next_stage
            )
        )

        node.run_semantic_fixed_orientation_pick(0.1)

        assert commanded_stages == [stage]
        assert node.top_down_stage == expected


def test_blended_approach_transitions_only_to_pregrasp_verify():
    node = make_group_sequence_node("BLENDED_APPROACH")
    node.command_group_blended_approach = lambda _dt: True
    transitions = []
    node.advance_precomputed_top_down_stage = (
        lambda next_stage, preserve_velocity=False: transitions.append(
            next_stage
        )
    )

    node.run_semantic_fixed_orientation_pick(0.1)

    assert transitions == ["PREGRASP_VERIFY"]
    assert node.gripper_position == node.gripper_open_position


def test_blended_command_publishes_one_composed_position(monkeypatch):
    node = make_semantic_segment_node()
    node.top_down_plan = SimpleNamespace(
        blended_approach=SimpleNamespace(duration=2.0),
        pregrasp_joints=np.full(6, 0.5),
    )
    node.previous_velocity = [0.0] * 6
    monkeypatch.setattr(
        controller,
        "sample_group_blended_trajectory",
        lambda _trajectory, _elapsed: (np.arange(6, dtype=float), False),
    )

    assert not node.command_group_blended_approach(0.1)
    assert node.position_command[:6] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]


def test_semantic_yaw_segment_freezes_every_inactive_joint(monkeypatch):
    node = make_semantic_segment_node()
    measured = [0.0, -1.1, 1.6, -0.4, 1.2, -0.3]
    node.current_joints = dict(zip(node.joint_names, measured))
    node.position_command = list(measured)
    captured_goal = []
    monkeypatch.setattr(
        controller,
        "minimum_compact_duration",
        lambda *_args, **_kwargs: 2.0,
    )

    def sample(start, goal, *_args):
        captured_goal.extend(goal)
        return np.asarray(start), False

    monkeypatch.setattr(controller, "sample_compact_joint_positions", sample)

    node.command_semantic_top_down_segment(
        [0.3, 9.0, 9.0, 9.0, 9.0, 9.0],
        "YAW",
        0.1,
    )

    assert captured_goal == [0.3] + measured[1:]


def test_pregrasp_verify_blocks_descend_until_measured_pose_is_valid(monkeypatch):
    node = make_group_sequence_node("PREGRASP_VERIFY")
    transitions = []
    node.advance_precomputed_top_down_stage = (
        lambda next_stage, preserve_velocity=False: transitions.append(next_stage)
    )
    monkeypatch.setattr(
        controller,
        "measured_pregrasp_pose_valid",
        lambda *_args: False,
    )

    node.run_semantic_fixed_orientation_pick(0.1)

    assert transitions == []
    assert node.gripper_position == node.gripper_open_position

    monkeypatch.setattr(
        controller,
        "measured_pregrasp_pose_valid",
        lambda *_args: True,
    )
    node.run_semantic_fixed_orientation_pick(0.1)

    assert transitions == ["DESCEND"]
    assert node.grasp_phase == "DESCEND"


def test_semantic_descend_feedback_removes_steady_state_joint_error(monkeypatch):
    class FakeTime:
        def __sub__(self, _other):
            return SimpleNamespace(nanoseconds=int(10e9))

    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    goal = [-0.1031, 0.2982, 0.1984, 0.8463, 1.4227, -1.3080]
    measured = [-0.1037, 0.3000, 0.2416, 0.8338, 1.4197, -1.3105]
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.current_joints = dict(zip(node.joint_names, measured))
    node.position_command = list(goal)
    node.top_down_segment_stage = "DESCEND"
    node.top_down_segment_start = [0.0] * 6
    node.top_down_segment_goal = list(goal)
    node.top_down_segment_start_time = FakeTime()
    node.top_down_segment_duration = 1.0
    node.top_down_segment_planned_complete = False
    node.top_down_joint_tolerance = 0.025
    node.top_down_transit_joint_tolerance = 0.050
    node.top_down_joint_kp = 2.0
    node.top_down_stage_acceleration = 1.2
    node.top_down_alignment_velocity_limits = [1.0] * 6
    node.top_down_path_velocity_limits = [1.0] * 6
    node.top_down_settle_max_command_offset = 0.10
    node.previous_velocity = [0.0] * 6
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime())
    node.apply_joint_limit_slowdown = lambda velocity: velocity
    node.limit_acceleration = (
        lambda velocity, _dt, max_acceleration=None: list(velocity)
    )

    def integrate(velocity, dt):
        node.position_command = [
            position + speed * dt
            for position, speed in zip(node.position_command, velocity)
        ]

    node.integrate_position_command = integrate
    monkeypatch.setattr(
        controller,
        "sample_quintic_joint_positions",
        lambda *_args: (np.asarray(goal), True),
    )

    complete = node.command_semantic_top_down_segment(goal, "DESCEND", 0.1)

    assert not complete
    assert node.position_command[2] < goal[2]
    assert goal[2] - node.position_command[2] <= 0.10


def test_reset_clears_every_precomputed_pick_target():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_plan = object()
    node.top_down_plan_attempted = True
    node.top_down_plan_error = "old failure"
    node.top_down_stage = "DESCEND"
    node.top_down_waypoint_index = 4
    node.top_down_segment_stage = "DESCEND"
    node.top_down_segment_start = [0.1] * 6
    node.top_down_segment_goal = [0.2] * 6
    node.top_down_segment_start_time = object()
    node.top_down_segment_duration = 9.0
    node.top_down_segment_planned_complete = True
    node.top_down_continuous_home = True

    node.reset_precomputed_top_down_state()

    assert node.top_down_plan is None
    assert not node.top_down_plan_attempted
    assert node.top_down_plan_error == ""
    assert node.top_down_stage == "WAIT_TARGET"
    assert node.top_down_waypoint_index == 0
    assert node.top_down_segment_stage == ""
    assert node.top_down_segment_start is None
    assert node.top_down_segment_goal is None
    assert node.top_down_segment_start_time is None
    assert not node.top_down_continuous_home
    assert node.top_down_segment_duration == 0.0
    assert not node.top_down_segment_planned_complete


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


def test_rotation_error_uses_actual_ee_and_target_frames():
    assert rotation_error_angle(yzy(0.0, 0.0, 0.0), yzy(0.0, 0.0, 0.0)) < 1e-9
    assert abs(rotation_error_angle(rot_z(pi / 2.0), rot_z(0.0)) - pi / 2.0) < 1e-9


def test_wrist_position_recovery_has_enter_exit_hysteresis():
    assert update_wrist_position_recovery(False, 0.051, 0.0, 0.05, 0.025)
    assert update_wrist_position_recovery(True, 0.030, 0.0, 0.05, 0.025)
    assert not update_wrist_position_recovery(True, 0.020, 0.0, 0.05, 0.025)


def test_full_mode_selects_equivalent_solution_nearest_current_wrist():
    targets = compute_wrist_targets_from_orientation(
        target_rotation=yzy(2.8, 1.0, 2.8),
        link3_rotation=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        fallback_targets=[0.0, 0.0, 0.0],
        joint_lower_limits=[-3.1416] * 6,
        joint_upper_limits=[3.1416] * 6,
        mode="full",
        reference_targets=[-0.3, -1.0, -0.3],
    )

    assert abs(targets[0] - (2.8 - pi)) < 1e-6
    assert abs(targets[1] + 1.0) < 1e-6
    assert abs(targets[2] - (2.8 - pi)) < 1e-6


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


def test_pick_approach_aligns_position_before_enabling_wrist():
    assert update_pick_approach_stage("PICK", "APPROACH", "POSITION", False) == "POSITION"
    assert update_pick_approach_stage("PICK", "APPROACH", "POSITION", True) == "WRIST"
    assert update_pick_approach_stage("PICK", "APPROACH", "WRIST", False) == "WRIST"
    assert update_pick_approach_stage("PICK", "APPROACH", "PREGRASP", False) == "PREGRASP"
    assert update_pick_approach_stage("PICK", "APPROACH", "HOLD", False) == "HOLD"
    assert update_pick_approach_stage("PICK", "DESCEND", "POSITION", False) == "WRIST"


def test_staged_pick_moves_wrist_then_pregrasp_then_descend():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.grasp_phase = "APPROACH"
    node.pick_approach_stage = "WRIST"
    node.wrist_position_recovery = False
    node.enable_staged_top_down_approach = True
    node.gripper_open_position = 0.0
    node.gripper_position = 0.0
    node.grasp_z_offset = 0.15
    node.grasp_pregrasp_clearance_z = 0.04
    node.grasp_final_offset_z = 0.0
    node.require_descend_confirmation = False
    node.descend_confirmed = False
    node.grasp_descend_speed = 0.20
    node.previous_velocity = [1.0] * 6
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: None}
    )()

    node._update_pick_phase(pos_aligned=True, wrist_aligned=True, dt=0.1)
    assert node.pick_approach_stage == "PREGRASP"
    assert node.grasp_phase == "APPROACH"

    for _ in range(10):
        node._update_pick_phase(pos_aligned=True, wrist_aligned=True, dt=0.1)
        if node.grasp_phase == "DESCEND":
            break
    assert abs(node.grasp_z_offset - 0.04) < 1e-9
    assert node.grasp_phase == "DESCEND"

    for _ in range(10):
        node._update_pick_phase(pos_aligned=True, wrist_aligned=True, dt=0.1)
        if node.grasp_z_offset <= 0.0:
            break
    assert abs(node.grasp_z_offset) < 1e-9


def test_staged_pick_holds_pregrasp_until_descend_confirmation():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.grasp_phase = "APPROACH"
    node.pick_approach_stage = "PREGRASP"
    node.enable_staged_top_down_approach = True
    node.require_descend_confirmation = True
    node.descend_confirmed = False
    node.gripper_open_position = 0.0
    node.gripper_position = 0.0
    node.grasp_z_offset = 0.12
    node.grasp_pregrasp_clearance_z = 0.12
    node.grasp_final_offset_z = 0.0
    node.grasp_descend_speed = 0.20
    node.previous_velocity = [0.0] * 6
    node.position_command = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    node.pregrasp_hold_positions = None
    node.set_control_state = lambda state: None
    node.publish_task_state = lambda: None
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: None}
    )()

    node._update_pick_phase(pos_aligned=True, wrist_aligned=True, dt=0.1)
    assert node.grasp_phase == "APPROACH"
    assert node.pick_approach_stage == "HOLD"
    assert node.pregrasp_hold_positions == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    node.pick_approach_stage = "PREGRASP"
    node.pregrasp_hold_positions = None
    node.descend_confirmed = True
    node._update_pick_phase(pos_aligned=True, wrist_aligned=True, dt=0.1)
    assert node.grasp_phase == "DESCEND"


def test_pick_lift_can_finish_by_returning_home_with_gripper_closed():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.grasp_phase = "LIFT"
    node.gripper_close_position = 0.8
    node.gripper_position = 0.0
    node.lift_z_accumulated = 0.12
    node.grasp_lift_height = 0.12
    node.return_home_after_pick = True
    node.force_safety_pose = False
    calls = []
    node.disable_target_tracking = lambda: calls.append("disable")
    node.set_control_state = lambda state: calls.append(state)
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: None}
    )()

    node._update_pick_phase(pos_aligned=True, wrist_aligned=True, dt=0.1)

    assert node.grasp_phase == "HOLD"
    assert node.gripper_position == 0.8
    assert node.force_safety_pose
    assert calls == ["disable", "RETURN_HOME"]


def test_pregrasp_hold_republishes_latched_joint_target_without_motion():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.task_mode = "PICK"
    node.grasp_phase = "APPROACH"
    node.pick_approach_stage = "HOLD"
    node.pregrasp_hold_positions = [0.5, 0.4, 0.3, 0.2, 0.1, 0.0]
    node.position_command = [0.0] * 6
    node.previous_velocity = [1.0] * 6
    published = []
    node.cmd_pub = type("Publisher", (), {"publish": lambda self, msg: published.append("stop")})()
    node.publish_position_command = lambda: published.append("joints")
    node.publish_task_state = lambda: published.append("state")

    node.maintain_pregrasp_hold("test")

    assert node.position_command == [0.5, 0.4, 0.3, 0.2, 0.1, 0.0]
    assert node.previous_velocity == [0.0] * 6
    assert published == ["stop", "joints", "state"]


def test_descend_command_can_be_queued_before_pregrasp_hold():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.task_mode = "PICK"
    node.enable_staged_top_down_approach = True
    node.grasp_phase = "APPROACH"
    node.pick_approach_stage = "WRIST"
    node.descend_confirmed = False
    node.previous_velocity = [1.0] * 6
    messages = []
    node.get_logger = lambda: type(
        "Logger", (), {"warn": lambda self, message: messages.append(message)}
    )()

    node.on_task_command(String(data="DESCEND"))

    assert node.descend_confirmed
    assert node.pick_approach_stage == "WRIST"
    assert node.previous_velocity == [0.0] * 6
    assert "queued" in messages[-1]


def make_stage_limiter():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.task_mode = "PICK"
    node.grasp_phase = "APPROACH"
    node.pick_approach_stage = "WRIST"
    node.wrist_position_recovery = False
    node.control_state = "ARM_TRACK"
    node.stage_velocity_limits = {
        "POSITION": [0.35, 0.60, 0.80, 0.30],
        "WRIST": [0.25, 0.25, 0.35, 0.35],
        "RECOVER": [0.22, 0.30, 0.42, 0.08],
        "PREGRASP": [0.18, 0.20, 0.28, 0.22],
        "DESCEND": [0.12, 0.08, 0.12, 0.12],
    }
    node.stage_acceleration_limits = {
        "POSITION": 1.4,
        "WRIST": 0.9,
        "RECOVER": 0.75,
        "PREGRASP": 0.65,
        "DESCEND": 0.30,
    }
    node.max_joint_acceleration = 3.0
    node.joint_lower_limits = [-1.0] * 6
    node.joint_upper_limits = [1.0] * 6
    node.pick_start_positions = [0.0] * 6
    node.pick_max_joint_excursion = [0.8, 0.8, 0.8, 0.6, 0.6, 0.6]
    node.joint_soft_limit_margin = 0.2
    node.position_command = [0.0] * 6
    return node


def test_wrist_stage_strictly_limits_link2_link3_and_wrist_speed():
    node = make_stage_limiter()

    limited = node.apply_stage_velocity_limits([1.0] * 6)

    assert limited == [0.25, 0.25, 0.35, 0.35, 0.35, 0.35]
    assert node.current_stage_acceleration() == 0.9


def test_wrist_recovery_keeps_slow_wrist_motion_with_moderate_position_speed():
    node = make_stage_limiter()
    node.wrist_position_recovery = True

    limited = node.apply_stage_velocity_limits([1.0] * 6)

    assert limited == [0.22, 0.30, 0.42, 0.08, 0.08, 0.08]
    assert node.current_stage_acceleration() == 0.75


def test_wrist_stage_uses_hard_bounds_for_link2_and_link3_only():
    node = make_stage_limiter()

    assert node.active_joint_bounds(1) == (-1.0, 1.0)
    assert node.active_joint_bounds(2) == (-1.0, 1.0)
    assert node.active_joint_bounds(0) == (-0.8, 0.8)
    assert node.active_joint_bounds(3) == (-0.6, 0.6)

    node.pick_approach_stage = "POSITION"
    assert node.active_joint_bounds(1) == (-0.8, 0.8)
    assert node.active_joint_bounds(2) == (-0.8, 0.8)


def test_position_stage_clamps_commands_relative_to_pick_start_pose():
    node = make_stage_limiter()
    node.pick_approach_stage = "POSITION"
    node.position_command = [0.0] * 6

    node.integrate_position_command([10.0] * 6, dt=1.0)

    assert node.position_command == [0.8, 0.8, 0.8, 0.6, 0.6, 0.6]


def test_soft_joint_limit_slows_outward_motion_but_allows_return():
    node = make_stage_limiter()
    node.position_command[3] = 0.55

    outward = node.apply_joint_limit_slowdown([0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
    inward = node.apply_joint_limit_slowdown([0.0, 0.0, 0.0, -0.2, 0.0, 0.0])

    assert 0.0 < outward[3] < 0.2
    assert inward[3] == -0.2


def test_controller_axis_parameters_load_as_strings_from_yaml():
    config_path = Path(__file__).parents[1] / "config" / "arm_position_target_in.yaml"
    rclpy.init(args=["--ros-args", "--params-file", str(config_path)])
    node = None
    try:
        node = controller.ArmYawRhoZPositionController()
        assert node.grasp_link6_axis == "y"
        assert node.grasp_gripper_closing_axis == "x"
        assert node.grasp_target_closing_axis == "y"
        assert node.stage_velocity_limits["RECOVER"][3] == 0.08
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
