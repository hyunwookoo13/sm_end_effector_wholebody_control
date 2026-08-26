from pathlib import Path


MODULE = (
    Path(__file__).resolve().parents[1]
    / "sm_ee_wholebody_control"
    / "arm_yaw_rho_z_position_controller.py"
)


def test_final_zero_command_requires_a_valid_ros_context():
    source = MODULE.read_text()
    main_source = source[source.index("def main(args=None)") :]

    assert "if rclpy.ok():\n            node.cmd_pub.publish(Twist())" in main_source
