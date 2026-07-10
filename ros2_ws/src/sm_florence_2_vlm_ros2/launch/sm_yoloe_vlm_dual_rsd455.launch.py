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
    default_pick_config = os.path.join(pkg_share, "config", "sm_yoloe_vlm_rsd455.yaml")
    default_place_config = os.path.join(pkg_share, "config", "sm_yoloe_vlm_rsd455_place.yaml")

    pick_config_arg = DeclareLaunchArgument(
        "pick_config_file",
        default_value=default_pick_config,
        description="첫 번째 RSD455 YOLOE 파라미터 YAML 파일 경로",
    )
    place_config_arg = DeclareLaunchArgument(
        "place_config_file",
        default_value=default_place_config,
        description="두 번째 RSD455 YOLOE 파라미터 YAML 파일 경로",
    )

    pick_node = Node(
        package="sm_florence_2_vlm_ros2",
        executable="sm_yoloe_vlm_node",
        name="sm_yoloe_vlm",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("pick_config_file")],
    )

    place_node = Node(
        package="sm_florence_2_vlm_ros2",
        executable="sm_yoloe_vlm_node",
        name="sm_yoloe_vlm_place",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("place_config_file")],
    )

    return LaunchDescription([pick_config_arg, place_config_arg, pick_node, place_node])
