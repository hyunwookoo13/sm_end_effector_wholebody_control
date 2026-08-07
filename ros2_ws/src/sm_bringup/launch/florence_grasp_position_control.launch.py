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
            "publish_fixed_camera_tf": LaunchConfiguration(
                "publish_fixed_camera_tf"
            ),
        }.items(),
    )

    bridge = Node(
        package="sm_ee_wholebody_control",
        executable="grasp_target_tf_bridge",
        name="grasp_target_tf_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/sm_grasping/grasp_best",
                "parent_frame": "chassis_link",
                "target_frame": "grasp_position_target",
                "rate_hz": 30.0,
                "timeout_sec": 0.8,
                "latch_target": False,
                "freeze_on_arm_track": True,
                "state_topic": "/arm_safety_state",
                "offset_x": 0.0,
                "offset_y": 0.0,
                "offset_z": 0.0,
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
                "target_frame": "grasp_position_target",
                "enable_base_motion": LaunchConfiguration("enable_base_motion"),
                "grasp_offset_z": ParameterValue(
                    LaunchConfiguration("grasp_offset_z"),
                    value_type=float,
                ),
                "grasp_descend_depth": ParameterValue(
                    LaunchConfiguration("grasp_descend_depth"),
                    value_type=float,
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
            DeclareLaunchArgument(
                "grasp_offset_z",
                default_value="0.03",
                description=(
                    "grasp target보다 위에서 먼저 도달할 pre-grasp Z 오프셋[m]."
                ),
            ),
            DeclareLaunchArgument(
                "grasp_descend_depth",
                default_value="0.03",
                description="pre-grasp 도달 후 그리퍼를 닫기 전에 내려갈 Z 거리[m].",
            ),
            DeclareLaunchArgument(
                "enable_base_motion",
                default_value="false",
                description=(
                    "좌표 검증 후에만 true로 설정. 기본값 false에서는 /cmd_vel=0"
                ),
            ),
            DeclareLaunchArgument(
                "publish_fixed_camera_tf",
                default_value="false",
            ),
            perception_launch,
            bridge,
            controller,
        ]
    )
