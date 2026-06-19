#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("sm_florence_2_vlm_ros2")
    default_config = os.path.join(pkg_share, "config", "sm_florence_2_vlm_ros2.yaml")

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="sm_florence_2_vlm_ros2 파라미터 YAML 파일 경로",
    )

    node = Node(
        package="sm_florence_2_vlm_ros2",
        executable="sm_florence_2_vlm_node",
        name="sm_florence_2_vlm",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_arg, node])
