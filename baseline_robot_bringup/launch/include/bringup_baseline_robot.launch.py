"""Start the single vehicle's localization, model, control and mission nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_directory = os.path.join(
        get_package_share_directory("baseline_robot_bringup"),
        "launch", "include", "baseline_robot",
    )
    components = (
        "localization.launch.py",
        "description.launch.py",
        "mvp_control.launch.py",
        "mvp_mission.launch.py",
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_directory, filename))
        )
        for filename in components
    ])
