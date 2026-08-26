#!/usr/bin/env python3
"""Visualize the static warehouse map against the live Isaac Sim rear scan.

This intentionally starts no navigation or velocity-control nodes.  The
world-to-map transform is adjustable from the command line so the exported
occupancy map can be aligned before it becomes the canonical Nav2 map.
"""

import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


HERE = Path(__file__).resolve().parent
MAP_YAML = HERE / "warehouse_map.yaml"
RVIZ_CONFIG = HERE / "map_alignment_validation.rviz"


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="warehouse_alignment_map_server",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "yaml_filename": str(MAP_YAML),
            }
        ],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_alignment",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": True,
                "node_names": ["warehouse_alignment_map_server"],
            }
        ],
    )
    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_alignment_tf",
        output="screen",
        arguments=[
            "--x",
            LaunchConfiguration("map_x_in_world_m"),
            "--y",
            LaunchConfiguration("map_y_in_world_m"),
            "--z",
            "0.0",
            "--yaw",
            LaunchConfiguration("map_yaw_in_world_rad"),
            "--pitch",
            "0.0",
            "--roll",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "map",
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_map_alignment_validation",
        output="screen",
        arguments=["-d", str(RVIZ_CONFIG)],
        parameters=[
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Initial estimate from a refined live rear-LiDAR scan match. These
            # are the pose of the map frame expressed in the world frame.
            DeclareLaunchArgument("map_x_in_world_m", default_value="2.0666"),
            DeclareLaunchArgument("map_y_in_world_m", default_value="-3.6084"),
            DeclareLaunchArgument("map_yaw_in_world_rad", default_value="-0.052598"),
            world_to_map,
            map_server,
            map_lifecycle,
            rviz,
        ]
    )


if __name__ == "__main__":
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(generate_launch_description())
    raise SystemExit(service.run())
