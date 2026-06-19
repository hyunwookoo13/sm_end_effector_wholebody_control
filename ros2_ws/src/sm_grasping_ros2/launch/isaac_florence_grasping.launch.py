#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    florence_share = get_package_share_directory("sm_florence_2_vlm_ros2")
    grasping_share = get_package_share_directory("sm_grasping_ros2")

    default_florence_config = os.path.join(
        florence_share,
        "config",
        "sm_florence_2_vlm_isaac.yaml",
    )
    default_grasping_config = os.path.join(
        grasping_share,
        "config",
        "sm_grasping_isaac.yaml",
    )

    florence_config_arg = DeclareLaunchArgument(
        "florence_config_file",
        default_value=default_florence_config,
        description="Isaac용 sm_florence_2_vlm_ros2 파라미터 YAML 파일 경로",
    )
    grasping_config_arg = DeclareLaunchArgument(
        "grasping_config_file",
        default_value=default_grasping_config,
        description="Isaac용 sm_grasping_ros2 파라미터 YAML 파일 경로",
    )

    florence_node = Node(
        package="sm_florence_2_vlm_ros2",
        executable="sm_florence_2_vlm_node",
        name="sm_florence_2_vlm",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("florence_config_file")],
    )

    grasping_inference_node = Node(
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

    return LaunchDescription(
        [
            florence_config_arg,
            grasping_config_arg,
            florence_node,
            grasping_inference_node,
            gripper_marker_node,
        ]
    )
