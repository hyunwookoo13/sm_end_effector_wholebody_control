from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


WORKSPACE = "/home/kiro/Desktop/hw_ws/ros2_ws"
MAP_DIRECTORY = f"{WORKSPACE}/maps/warehouse_isaac_occupancy_20260813"


def generate_launch_description() -> LaunchDescription:
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    semantic_lookup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sm_bringup"), "launch", "semantic_task_lookup.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "db_path": LaunchConfiguration("db_path"),
            "use_ollama": LaunchConfiguration("use_ollama"),
            "use_rule_fallback": LaunchConfiguration("use_rule_fallback"),
        }.items(),
    )

    # The original launch owns perception, task management, Whole-body control,
    # and the sole cmd_vel mux. Its algorithms remain unchanged; the optional
    # external place-navigation gate only pauses the manager after PICK:HOLD.
    existing_pick_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sm_bringup"),
                    "launch",
                    "natural_language_pick_place.launch.py",
                ]
            )
        ),
        launch_arguments={
            "autostart": "false",
            "enable_nav2": "false",
            "enable_natural_language_task_parser": "false",
            "external_place_navigation": "true",
            "publish_fixed_camera_tf": "false",
            "target_parent_frame": "odom",
            # Do not run repeated prompt embedding during map-only navigation.
            # The original manager still force-publishes the selected targets
            # when Pick begins and when it transitions toward Place.
            "target_objects_publish_period": "3600.0",
        }.items(),
    )

    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_semantic_mvp_tf",
        output="screen",
        arguments=[
            "--x", "2.0666",
            "--y", "-3.6084",
            "--z", "0",
            "--yaw", "-0.052598",
            "--pitch", "0",
            "--roll", "0",
            "--frame-id", "world",
            "--child-frame-id", "map",
        ],
    )
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="warehouse_semantic_mvp_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map_yaml"),
            }
        ],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_semantic_mvp_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["warehouse_semantic_mvp_map_server"],
            }
        ],
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sm_navigation_nav2"), "launch", "nav2_navigation.launch.py"]
            )
        ),
        launch_arguments={
            "nav2_params_file": LaunchConfiguration("nav2_params_file")
        }.items(),
    )
    semantic_nav = Node(
        package="sm_task_orchestrator",
        executable="semantic_nav2_adapter",
        name="semantic_workspace_nav2_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "resolved_task_topic": "/semantic_lookup/resolved_task",
                "request_topic": "/semantic_navigation/request",
                "goal_preview_topic": "/semantic_navigation/goal_pose",
                "status_topic": "/semantic_navigation/status",
                "navigation_action_name": "/navigate_to_pose",
                "base_mode_topic": "/base_control_mode",
                "target_role": "pick",
                "auto_start_on_resolved_task": False,
                "expected_frame": "map",
                "navigation_base_frame": "chassis_link",
                "navigation_enabled": True,
                # The mission orchestrator owns request de-duplication and must
                # allow the same demonstration to be repeated without restart.
                "prevent_duplicate_goal": False,
                "handoff_distance_m": ParameterValue(
                    LaunchConfiguration("handoff_distance_m"),
                    value_type=float,
                ),
            }
        ],
    )
    mission_orchestrator = Node(
        package="sm_semantic_mvp",
        executable="semantic_mission_orchestrator",
        name="semantic_mission_orchestrator",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "resolved_task_topic": "/semantic_lookup/resolved_task",
                "navigation_request_topic": "/semantic_navigation/request",
                "navigation_status_topic": "/semantic_navigation/status",
                "existing_task_topic": "/pick_place_task",
                "task_state_topic": "/pick_place_task_state",
                "phase_command_topic": "/pick_place_phase_command",
                "status_topic": "/semantic_mvp/status",
            }
        ],
    )
    semantic_markers = Node(
        package="sm_semantic_map",
        executable="semantic_map_marker_publisher",
        name="semantic_map_marker_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "db_path": LaunchConfiguration("db_path"),
                "marker_topic": "/semantic_map/markers",
                "resolved_task_topic": "/semantic_lookup/resolved_task",
                "status_topic": "/semantic_mvp/status",
                # Keep the mission banner at a stable map location for RViz
                # recording instead of letting it follow and cover the robot.
                "status_frame": "map",
            }
        ],
    )
    demo_sequence = Node(
        package="sm_semantic_mvp",
        executable="demo_task_sequencer",
        name="semantic_demo_task_sequencer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("auto_demo_sequence")),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "task_topic": "/natural_language_task",
                "mission_status_topic": "/semantic_mvp/status",
                "sequence_status_topic": "/semantic_demo_sequence/status",
                "initial_delay_sec": ParameterValue(
                    LaunchConfiguration("demo_initial_delay_sec"),
                    value_type=float,
                ),
                "inter_task_delay_sec": ParameterValue(
                    LaunchConfiguration("demo_inter_task_delay_sec"),
                    value_type=float,
                ),
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="semantic_mvp_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("sm_semantic_mvp"), "config", "semantic_mvp.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("auto_demo_sequence", default_value="false"),
            DeclareLaunchArgument("demo_initial_delay_sec", default_value="10.0"),
            DeclareLaunchArgument("demo_inter_task_delay_sec", default_value="3.0"),
            DeclareLaunchArgument(
                "db_path",
                default_value=f"{MAP_DIRECTORY}/semantic_map.sqlite3",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=f"{MAP_DIRECTORY}/warehouse_map.yaml",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=f"{MAP_DIRECTORY}/nav2_static_map_mvp.yaml",
            ),
            # Must stay above the static Nav2 profile's 0.25m XY goal tolerance
            # so handoff occurs before Nav2 can report SUCCEEDED.
            DeclareLaunchArgument("handoff_distance_m", default_value="0.35"),
            DeclareLaunchArgument("use_ollama", default_value="false"),
            DeclareLaunchArgument("use_rule_fallback", default_value="true"),
            semantic_lookup,
            existing_pick_place,
            world_to_map,
            map_server,
            map_lifecycle,
            nav2,
            semantic_nav,
            mission_orchestrator,
            semantic_markers,
            demo_sequence,
            rviz,
        ]
    )
