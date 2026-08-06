from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    target_frame = LaunchConfiguration("target_frame")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    use_monitor = LaunchConfiguration("use_monitor")
    use_logger = LaunchConfiguration("use_logger")
    params_file = LaunchConfiguration("params_file")

    common_env = {"ROS_DOMAIN_ID": ros_domain_id}

    controller = Node(
        package="sm_ee_wholebody_control",
        executable="wholebody_yaw_rho_z_controller",
        name="wholebody_yaw_rho_z_controller",
        output="screen",
        parameters=[
            params_file,
            {"target_frame": target_frame},
        ],
        additional_env=common_env,
    )

    monitor = Node(
        package="sm_ee_wholebody_control",
        executable="live_yaw_rho_z_monitor",
        name="live_yaw_rho_z_monitor",
        output="screen",
        condition=IfCondition(use_monitor),
        parameters=[params_file],
        additional_env=common_env,
    )

    logger = Node(
        package="sm_ee_wholebody_control",
        executable="wholebody_debug_logger",
        name="wholebody_yaw_rho_z_logger",
        output="screen",
        condition=IfCondition(use_logger),
        parameters=[
            params_file,
        ],
        additional_env=common_env,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_frame", default_value="target_out"),
            DeclareLaunchArgument("ros_domain_id", default_value="1"),
            DeclareLaunchArgument("use_monitor", default_value="true"),
            DeclareLaunchArgument("use_logger", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_ee_wholebody_control"),
                        "config",
                        "wholebody_savepoint.yaml",
                    ]
                ),
            ),
            controller,
            monitor,
            logger,
        ]
    )
