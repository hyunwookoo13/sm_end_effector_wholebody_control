#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("sm_grasping_ros2")
    default_config = os.path.join(pkg_share, "config", "sm_grasping_ros2.yaml")

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="sm_grasping_ros2 파라미터 YAML 파일 경로",
    )

    inference_node = Node(
        package="sm_grasping_ros2",
        executable="grasping_inference_node",
        name="sm_grasping_inference",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config_file")],
    )

    marker_node = Node(
        package="sm_grasping_ros2",
        executable="gripper_marker_node",
        name="sm_gripper_marker",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_arg, inference_node, marker_node])

