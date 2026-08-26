from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


WORKSPACE = "/home/kiro/Desktop/hw_ws/ros2_ws"
MAP_DIRECTORY = f"{WORKSPACE}/maps/warehouse_isaac_occupancy_20260813"


def generate_launch_description() -> LaunchDescription:
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_semantic_rviz_tf",
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
        name="warehouse_semantic_rviz_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "yaml_filename": LaunchConfiguration("map_yaml"),
            }
        ],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_semantic_rviz_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["warehouse_semantic_rviz_map_server"],
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
                # Preview remains valid even while Isaac Sim is stopped. The
                # full mission launch anchors this status label to the robot.
                "status_frame": "map",
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="semantic_map_rviz",
        output="screen",
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
            DeclareLaunchArgument(
                "db_path",
                default_value=f"{MAP_DIRECTORY}/semantic_map.sqlite3",
            ),
            DeclareLaunchArgument(
                "map_yaml",
                default_value=f"{MAP_DIRECTORY}/warehouse_map.yaml",
            ),
            world_to_map,
            map_server,
            lifecycle_manager,
            semantic_markers,
            rviz,
        ]
    )
