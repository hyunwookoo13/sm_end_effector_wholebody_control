from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_DB = (
    "/home/kiro/Desktop/hw_ws/ros2_ws/maps/"
    "warehouse_isaac_occupancy_20260813/semantic_map.sqlite3"
)


def generate_launch_description() -> LaunchDescription:
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    semantic_map_server = Node(
        package="sm_semantic_map",
        executable="semantic_map_server",
        name="semantic_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "db_path": LaunchConfiguration("db_path"),
                "seed_path": LaunchConfiguration("seed_path"),
                "seed_database_when_empty": True,
                "find_service": "/semantic_map/find_object",
            }
        ],
    )
    natural_language_parser = Node(
        package="sm_natural_language_task",
        executable="natural_language_task_parser",
        name="semantic_lookup_natural_language_parser",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "natural_language_topic": "/natural_language_task",
                "task_topic": "/semantic_lookup/task",
                "status_topic": "/semantic_lookup/parser_status",
                "use_ollama": ParameterValue(
                    LaunchConfiguration("use_ollama"), value_type=bool
                ),
                "use_rule_fallback": ParameterValue(
                    LaunchConfiguration("use_rule_fallback"), value_type=bool
                ),
                "warm_model_on_startup": ParameterValue(
                    LaunchConfiguration("use_ollama"), value_type=bool
                ),
            }
        ],
    )
    resolver = Node(
        package="sm_task_orchestrator",
        executable="semantic_task_resolver",
        name="semantic_task_resolver",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "task_topic": "/semantic_lookup/task",
                "resolved_task_topic": "/semantic_lookup/resolved_task",
                "status_topic": "/semantic_lookup/status",
                "find_service": "/semantic_map/find_object",
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("db_path", default_value=DEFAULT_DB),
            DeclareLaunchArgument(
                "seed_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_semantic_map"),
                        "config",
                        "warehouse_objects.yaml",
                    ]
                ),
            ),
            # Deterministic aliases are the MVP default. Ollama remains a
            # launch-time replacement for open-vocabulary commands.
            DeclareLaunchArgument("use_ollama", default_value="false"),
            DeclareLaunchArgument("use_rule_fallback", default_value="true"),
            semantic_map_server,
            natural_language_parser,
            resolver,
        ]
    )
