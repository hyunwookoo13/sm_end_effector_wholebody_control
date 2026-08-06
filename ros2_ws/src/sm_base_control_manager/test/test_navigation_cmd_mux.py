from pytest import approx

from sm_base_control_manager.navigation_cmd_mux import (
    selected_source,
    slew_value,
    smooth_blend_value,
)


def test_navigation_mode_selects_navigation_command():
    assert selected_source("NAVIGATION") == "navigation"


def test_retreat_mode_selects_dedicated_command():
    assert selected_source("RETREAT") == "retreat"


def test_manipulation_mode_selects_manipulation_command():
    assert selected_source("MANIPULATION") == "manipulation"


def test_unknown_mode_stops_the_base():
    assert selected_source("unknown") is None


def test_hybrid_mode_selects_blended_command():
    assert selected_source("HYBRID") == "hybrid"


def test_blend_endpoints_and_midpoint_are_exact():
    assert smooth_blend_value(1.0, 0.2, 0.0) == approx(1.0)
    assert smooth_blend_value(1.0, 0.2, 1.0) == approx(0.2)
    assert smooth_blend_value(1.0, 0.2, 0.5) == approx(0.6)


def test_slew_value_limits_acceleration_in_both_directions():
    assert slew_value(0.0, 1.0, max_rate=2.0, dt=0.1) == approx(0.2)
    assert slew_value(0.5, -1.0, max_rate=2.0, dt=0.1) == approx(0.3)
