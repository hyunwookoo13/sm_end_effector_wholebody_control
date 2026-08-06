from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav2_condition = IfCondition(LaunchConfiguration("enable_nav2"))

    camera_tf_node = Node(
        package="ee_switch_debug",
        executable="fixed_camera_tf_publisher",
        name="rsd455_fixed_camera_tf_publisher",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("publish_fixed_camera_tf")),
        parameters=[
            {
                "use_sim_time": True,
                "parent_frame": "world",
                "child_frame": LaunchConfiguration("pick_camera_frame"),
                "isaac_x": -0.04447,
                "isaac_y": 3.46295,
                "isaac_z": 1.25048,
                "mapping": "isaac_y_to_ros_x",
                "qx": 0.7071068,
                "qy": 0.7071068,
                "qz": 0.0,
                "qw": 0.0,
                "rate_hz": 30.0,
            }
        ],
    )

    pick_yoloe_node = Node(
        package="sm_florence_2_vlm_ros2",
        executable="sm_yoloe_vlm_node",
        name="sm_yoloe_vlm",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("pick_yoloe_config_file"),
            {
                "use_sim_time": True,
                "camera_frame": LaunchConfiguration("pick_camera_frame"),
            },
        ],
    )

    place_yoloe_node = Node(
        package="sm_florence_2_vlm_ros2",
        executable="sm_yoloe_vlm_node",
        name="sm_yoloe_vlm_place",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("place_yoloe_config_file"),
            {
                "use_sim_time": True,
                "detections_topic": LaunchConfiguration("place_detections_topic"),
                "camera_frame": LaunchConfiguration("place_camera_frame"),
            },
        ],
    )

    pick_grasping_inference_node = Node(
        package="sm_grasping_ros2",
        executable="grasping_inference_node",
        name="sm_grasping_inference",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("grasping_config_file")],
    )

    gripper_marker_node = Node(
        package="sm_grasping_ros2",
        executable="gripper_marker_node",
        name="sm_gripper_marker",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("grasping_config_file")],
    )

    place_grasping_inference_node = Node(
        package="sm_grasping_ros2",
        executable="grasping_inference_node",
        name="sm_grasping_inference_place",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("place_grasping_config_file")],
    )

    task_manager = Node(
        package="ee_switch_debug",
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
                "place_detections_topic": LaunchConfiguration("place_detections_topic"),
                "extra_grasp_topics": ["/sm_grasping_place/grasp_best"],
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
                "fresh_grasp_delay_sec": ParameterValue(
                    LaunchConfiguration("fresh_grasp_delay_sec"),
                    value_type=float,
                ),
                "enable_navigation": ParameterValue(
                    LaunchConfiguration("enable_nav2"),
                    value_type=bool,
                ),
                "navigation_base_frame": "chassis_link",
                "pick_standoff_m": ParameterValue(
                    LaunchConfiguration("pick_standoff_m"),
                    value_type=float,
                ),
                "place_standoff_m": ParameterValue(
                    LaunchConfiguration("place_standoff_m"),
                    value_type=float,
                ),
                "place_direct_approach_distance_m": ParameterValue(
                    LaunchConfiguration("place_direct_approach_distance_m"),
                    value_type=float,
                ),
                "enable_hybrid_handoff": ParameterValue(
                    LaunchConfiguration("enable_hybrid_handoff"),
                    value_type=bool,
                ),
                "hybrid_outer_distance_m": ParameterValue(
                    LaunchConfiguration("hybrid_outer_distance_m"),
                    value_type=float,
                ),
                "hybrid_inner_distance_m": ParameterValue(
                    LaunchConfiguration("hybrid_inner_distance_m"),
                    value_type=float,
                ),
            }
        ],
    )

    navigation_cmd_mux = Node(
        package="sm_base_control_manager",
        executable="navigation_cmd_mux",
        name="navigation_cmd_mux",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "max_linear_velocity_mps": ParameterValue(
                    LaunchConfiguration("mux_max_linear_velocity_mps"),
                    value_type=float,
                ),
                "max_angular_velocity_rps": ParameterValue(
                    LaunchConfiguration("mux_max_angular_velocity_rps"),
                    value_type=float,
                ),
                "max_linear_acceleration_mps2": ParameterValue(
                    LaunchConfiguration("mux_max_linear_acceleration_mps2"),
                    value_type=float,
                ),
                "max_angular_acceleration_rps2": ParameterValue(
                    LaunchConfiguration("mux_max_angular_acceleration_rps2"),
                    value_type=float,
                ),
            }
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sm_navigation_nav2"),
                    "launch",
                    "nav2_navigation.launch.py",
                ]
            )
        ),
        condition=nav2_condition,
        launch_arguments={
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
        }.items(),
    )

    natural_language_task_parser = Node(
        package="sm_natural_language_task",
        executable="natural_language_task_parser",
        name="natural_language_task_parser",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("enable_natural_language_task_parser")),
        parameters=[
            {
                "use_sim_time": True,
                "natural_language_topic": LaunchConfiguration("natural_language_task_topic"),
                "task_topic": "/pick_place_task",
                "ollama_url": LaunchConfiguration("ollama_url"),
                "model": LaunchConfiguration("local_llm_model"),
                "use_ollama": ParameterValue(
                    LaunchConfiguration("use_local_llm"),
                    value_type=bool,
                ),
                "use_rule_fallback": ParameterValue(
                    LaunchConfiguration("use_rule_task_parser"),
                    value_type=bool,
                ),
            }
        ],
    )

    controller = Node(
        package="ee_switch_debug",
        executable="arm_yaw_rho_z_position_controller",
        name="arm_yaw_rho_z_position_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("controller_config_file"),
            {
                "use_sim_time": True,
                "target_frame": "pick_place_target",
                "cmd_topic": "/cmd_vel_manipulation",
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
                "pick_yoloe_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_florence_2_vlm_ros2"),
                        "config",
                        "sm_yoloe_vlm_rsd455.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "place_yoloe_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_florence_2_vlm_ros2"),
                        "config",
                        "sm_yoloe_vlm_rsd455_place.yaml",
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
                "place_grasping_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_grasping_ros2"),
                        "config",
                        "sm_grasping_isaac_place.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "controller_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ee_switch_debug"),
                        "config",
                        "arm_position_target_in.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_navigation_nav2"),
                        "config",
                        "nav2_rolling_odom.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("pick_object", default_value="can"),
            DeclareLaunchArgument("place_object", default_value="box"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("enable_base_motion", default_value="true"),
            DeclareLaunchArgument("enable_nav2", default_value="false"),
            DeclareLaunchArgument("pick_standoff_m", default_value="0.70"),
            DeclareLaunchArgument("place_standoff_m", default_value="0.70"),
            DeclareLaunchArgument("place_direct_approach_distance_m", default_value="1.20"),
            DeclareLaunchArgument("enable_hybrid_handoff", default_value="true"),
            DeclareLaunchArgument("hybrid_outer_distance_m", default_value="1.40"),
            DeclareLaunchArgument("hybrid_inner_distance_m", default_value="0.85"),
            DeclareLaunchArgument("mux_max_linear_velocity_mps", default_value="1.50"),
            DeclareLaunchArgument("mux_max_angular_velocity_rps", default_value="1.40"),
            DeclareLaunchArgument("mux_max_linear_acceleration_mps2", default_value="2.00"),
            DeclareLaunchArgument("mux_max_angular_acceleration_rps2", default_value="2.00"),
            DeclareLaunchArgument("publish_fixed_camera_tf", default_value="false"),
            DeclareLaunchArgument("target_parent_frame", default_value="odom"),
            DeclareLaunchArgument("pick_camera_frame", default_value="rsd455_color_optical_frame"),
            DeclareLaunchArgument(
                "place_camera_frame",
                default_value="rsd455_color_optical_frame2",
            ),
            DeclareLaunchArgument(
                "place_detections_topic",
                default_value="/sm_florence_2_vlm_place/detections",
            ),
            DeclareLaunchArgument("grasp_offset_z", default_value="0.03"),
            DeclareLaunchArgument("grasp_descend_depth", default_value="0.07"),
            DeclareLaunchArgument("place_offset_z", default_value="0.10"),
            DeclareLaunchArgument("place_descend_depth", default_value="0.03"),
            DeclareLaunchArgument("place_open_duration", default_value="0.8"),
            DeclareLaunchArgument("return_home_after_place", default_value="true"),
            DeclareLaunchArgument("place_offset_x", default_value="0.0"),
            DeclareLaunchArgument("place_offset_y", default_value="0.0"),
            DeclareLaunchArgument("place_target_offset_z", default_value="0.0"),
            DeclareLaunchArgument("task_manager_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("target_objects_publish_period", default_value="1.0"),
            DeclareLaunchArgument("task_manager_tf_timeout_sec", default_value="0.1"),
            DeclareLaunchArgument("fresh_grasp_delay_sec", default_value="0.35"),
            DeclareLaunchArgument("enable_natural_language_task_parser", default_value="true"),
            DeclareLaunchArgument(
                "natural_language_task_topic",
                default_value="/natural_language_task",
            ),
            DeclareLaunchArgument("use_local_llm", default_value="true"),
            DeclareLaunchArgument("use_rule_task_parser", default_value="false"),
            DeclareLaunchArgument("local_llm_model", default_value="gemma3:4b"),
            DeclareLaunchArgument("ollama_url", default_value="http://127.0.0.1:11434/api/chat"),
            camera_tf_node,
            pick_yoloe_node,
            place_yoloe_node,
            pick_grasping_inference_node,
            gripper_marker_node,
            place_grasping_inference_node,
            task_manager,
            navigation_cmd_mux,
            nav2_launch,
            natural_language_task_parser,
            controller,
        ]
    )
