from sm_ee_wholebody_control.arm_yaw_rho_z_position_controller import (
    initial_task_state,
)


def test_default_controller_starts_as_idle_pick_approach():
    assert initial_task_state(False) == ("PICK", False, "APPROACH")


def test_resume_controller_starts_closed_in_transport_hold():
    assert initial_task_state(True) == ("TRANSPORT", True, "HOLD")
