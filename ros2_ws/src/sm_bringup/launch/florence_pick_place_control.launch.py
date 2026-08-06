import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    grasping_share = get_package_share_directory("sm_grasping_ros2")

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                grasping_share,
                "launch",
                "rsd455_florence_grasping.launch.py",
            )
        ),
        launch_arguments={
            "florence_config_file": LaunchConfiguration("florence_config_file"),
            "grasping_config_file": LaunchConfiguration("grasping_config_file"),
            "publish_fixed_camera_tf": LaunchConfiguration("publish_fixed_camera_tf"),
        }.items(),
    )

    task_manager = Node(
        package="sm_task_orchestrator",
        executable="pick_place_task_manager",
        name="pick_place_task_manager",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "parent_frame": LaunchConfiguration("target_parent_frame"),
                "target_frame": "pick_place_target",
                "pick_object": LaunchConfiguration("pick_object"),
                "place_object": LaunchConfiguration("place_object"),
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"),
                    value_type=bool,
                ),
                "place_offset_x": ParameterValue(
                    LaunchConfiguration("place_offset_x"),
                    value_type=float,
                ),
                "place_offset_y": ParameterValue(
                    LaunchConfiguration("place_offset_y"),
                    value_type=float,
                ),
                "place_offset_z": ParameterValue(
                    LaunchConfiguration("place_target_offset_z"),
                    value_type=float,
                ),
                "rate_hz": ParameterValue(
                    LaunchConfiguration("task_manager_rate_hz"),
                    value_type=float,
                ),
                "target_objects_publish_period": ParameterValue(
                    LaunchConfiguration("target_objects_publish_period"),
                    value_type=float,
                ),
                "tf_timeout_sec": ParameterValue(
                    LaunchConfiguration("task_manager_tf_timeout_sec"),
                    value_type=float,
                ),
            }
        ],
    )

    controller = Node(
        package="sm_ee_wholebody_control",
        executable="arm_yaw_rho_z_position_controller",
        name="arm_yaw_rho_z_position_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("controller_config_file"),
            {
                "use_sim_time": True,
                "target_frame": "pick_place_target",
                "enable_base_motion": ParameterValue(
                    LaunchConfiguration("enable_base_motion"),
                    value_type=bool,
                ),
                "grasp_offset_z": ParameterValue(
                    LaunchConfiguration("grasp_offset_z"),
                    value_type=float,
                ),
                "grasp_descend_depth": ParameterValue(
                    LaunchConfiguration("grasp_descend_depth"),
                    value_type=float,
                ),
                "place_offset_z": ParameterValue(
                    LaunchConfiguration("place_offset_z"),
                    value_type=float,
                ),
                "place_descend_depth": ParameterValue(
                    LaunchConfiguration("place_descend_depth"),
                    value_type=float,
                ),
                "place_open_duration": ParameterValue(
                    LaunchConfiguration("place_open_duration"),
                    value_type=float,
                ),
                "return_home_after_place": ParameterValue(
                    LaunchConfiguration("return_home_after_place"),
                    value_type=bool,
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "florence_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_florence_2_vlm_ros2"),
                        "config",
                        "sm_florence_2_vlm_rsd455.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "grasping_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_grasping_ros2"),
                        "config",
                        "sm_grasping_isaac.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "controller_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_ee_wholebody_control"),
                        "config",
                        "arm_position_target_in.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("pick_object", default_value="can"),
            DeclareLaunchArgument("place_object", default_value="dish"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("enable_base_motion", default_value="false"),
            DeclareLaunchArgument("publish_fixed_camera_tf", default_value="false"),
            DeclareLaunchArgument("target_parent_frame", default_value="odom"),
            DeclareLaunchArgument("grasp_offset_z", default_value="0.03"),
            DeclareLaunchArgument("grasp_descend_depth", default_value="0.07"),
            DeclareLaunchArgument("place_offset_z", default_value="0.10"),
            DeclareLaunchArgument("place_descend_depth", default_value="0.01"),
            DeclareLaunchArgument("place_open_duration", default_value="0.8"),
            DeclareLaunchArgument("return_home_after_place", default_value="true"),
            DeclareLaunchArgument("place_offset_x", default_value="0.0"),
            DeclareLaunchArgument("place_offset_y", default_value="0.0"),
            DeclareLaunchArgument("place_target_offset_z", default_value="0.0"),
            DeclareLaunchArgument("task_manager_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("target_objects_publish_period", default_value="1.0"),
            DeclareLaunchArgument("task_manager_tf_timeout_sec", default_value="0.1"),
            perception_launch,
            task_manager,
            controller,
        ]
    )
