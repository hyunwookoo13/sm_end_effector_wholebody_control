#!/usr/bin/env python3
"""Isolated static-map Nav2 validation for the Isaac Sim warehouse."""

import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
NAV_LAUNCH = WORKSPACE / "src/sm_navigation_nav2/launch/nav2_navigation.launch.py"
PARAMS = HERE / "nav2_static_map_validation.yaml"
MAP_YAML = HERE / "warehouse_map.yaml"
RVIZ_CONFIG = HERE / "nav2_static_map_validation.rviz"
RVIZ_PARAMS = HERE / "rviz_sim_time.yaml"


def generate_launch_description() -> LaunchDescription:
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="warehouse_map_server",
        output="screen",
        parameters=[{"use_sim_time": True, "yaml_filename": str(MAP_YAML)}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_static_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["warehouse_map_server"],
            }
        ],
    )
    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_static_tf",
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
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(NAV_LAUNCH)),
        launch_arguments={"nav2_params_file": str(PARAMS)}.items(),
    )
    cmd_mux = Node(
        package="sm_base_control_manager",
        executable="navigation_cmd_mux",
        name="navigation_cmd_mux_validation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "max_linear_velocity_mps": 0.25,
                "max_angular_velocity_rps": 0.30,
                "max_linear_acceleration_mps2": 0.40,
                "max_angular_acceleration_rps2": 0.60,
            }
        ],
    )
    nav_mode = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/base_control_mode",
                    "std_msgs/msg/String",
                    "{data: NAVIGATION}",
                ],
                output="screen",
            )
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2_static_map_validation",
        output="screen",
        arguments=[
            "-d",
            str(RVIZ_CONFIG),
            "--ros-args",
            "--params-file",
            str(RVIZ_PARAMS),
        ],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            world_to_map,
            map_server,
            map_lifecycle,
            nav2,
            cmd_mux,
            nav_mode,
            rviz,
        ]
    )


if __name__ == "__main__":
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(generate_launch_description())
    raise SystemExit(service.run())
