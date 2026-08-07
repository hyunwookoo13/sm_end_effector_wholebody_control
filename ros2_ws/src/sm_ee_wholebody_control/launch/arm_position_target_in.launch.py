from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("sm_ee_wholebody_control"),
            "config",
            "arm_position_target_in.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="sm_ee_wholebody_control",
                executable="arm_yaw_rho_z_position_controller",
                name="arm_yaw_rho_z_position_controller",
                output="screen",
                emulate_tty=True,
                parameters=[config_file],
            )
        ]
    )
