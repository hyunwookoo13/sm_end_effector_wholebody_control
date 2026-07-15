from math import atan2, hypot

from pytest import approx

from ee_switch_debug.navigation_geometry import (
    compute_standoff_pose,
    hybrid_blend_weight,
    rear_sector_clearance,
    should_skip_navigation,
)


def test_standoff_pose_stops_before_object_and_faces_it():
    goal = compute_standoff_pose(
        base_xy=(0.0, 0.0),
        object_xy=(3.0, 4.0),
        standoff_m=1.0,
    )

    assert goal.x == approx(2.4)
    assert goal.y == approx(3.2)
    assert goal.yaw == approx(atan2(4.0, 3.0))
    assert hypot(3.0 - goal.x, 4.0 - goal.y) == approx(1.0)


def test_standoff_pose_handles_coincident_base_and_object():
    goal = compute_standoff_pose(
        base_xy=(1.2, -0.4),
        object_xy=(1.2, -0.4),
        standoff_m=0.7,
    )

    assert goal.x == approx(1.2)
    assert goal.y == approx(-0.4)
    assert goal.yaw == approx(0.0)


def test_negative_standoff_is_clamped_to_zero():
    goal = compute_standoff_pose(
        base_xy=(0.0, 0.0),
        object_xy=(1.0, 0.0),
        standoff_m=-1.0,
    )

    assert goal.x == approx(1.0)
    assert goal.y == approx(0.0)


def test_rear_sector_clearance_ignores_self_returns_and_side_points():
    clearance = rear_sector_clearance(
        ranges=[0.34, 0.8, 2.0, 0.7, 0.35],
        angle_min=-1.0,
        angle_increment=0.5,
        range_min=0.05,
        range_max=10.0,
        half_angle=0.6,
        ignore_below=0.45,
    )

    assert clearance == approx(0.7)


def test_place_navigation_is_skipped_inside_direct_approach_range():
    skip, reason = should_skip_navigation(
        kind="place",
        base_xy=(0.0, 0.0),
        object_xy=(1.1, 0.0),
        goal_xy=(0.4, 0.0),
        goal_skip_distance_m=0.2,
        place_direct_approach_distance_m=1.2,
    )

    assert skip is True
    assert "precision approach range" in reason


def test_pick_navigation_is_not_skipped_by_place_direct_range():
    skip, _ = should_skip_navigation(
        kind="pick",
        base_xy=(0.0, 0.0),
        object_xy=(1.1, 0.0),
        goal_xy=(0.4, 0.0),
        goal_skip_distance_m=0.2,
        place_direct_approach_distance_m=1.2,
    )

    assert skip is False


def test_navigation_is_still_skipped_when_standoff_goal_is_nearby():
    skip, reason = should_skip_navigation(
        kind="pick",
        base_xy=(0.0, 0.0),
        object_xy=(0.8, 0.0),
        goal_xy=(0.1, 0.0),
        goal_skip_distance_m=0.2,
        place_direct_approach_distance_m=1.2,
    )

    assert skip is True
    assert "goal is nearby" in reason


def test_hybrid_blend_weight_has_smooth_exact_boundaries():
    assert hybrid_blend_weight(1.40, outer_m=1.40, inner_m=0.85) == approx(0.0)
    assert hybrid_blend_weight(0.85, outer_m=1.40, inner_m=0.85) == approx(1.0)
    assert hybrid_blend_weight(1.125, outer_m=1.40, inner_m=0.85) == approx(0.5)


def test_hybrid_blend_weight_is_clamped_outside_zone():
    assert hybrid_blend_weight(2.0, outer_m=1.40, inner_m=0.85) == approx(0.0)
    assert hybrid_blend_weight(0.5, outer_m=1.40, inner_m=0.85) == approx(1.0)
