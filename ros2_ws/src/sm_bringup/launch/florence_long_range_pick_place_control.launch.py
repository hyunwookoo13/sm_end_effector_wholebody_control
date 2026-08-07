from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    source = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [
                FindPackageShare("sm_bringup"),
                "launch",
                "natural_language_pick_place.launch.py",
            ]
        )
    )
    return LaunchDescription([IncludeLaunchDescription(source)])
