"""Start the single vehicle's localization, model, control and mission nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    actions = [DeclareLaunchArgument("use_usbl", default_value="true"),
               DeclareLaunchArgument("localization_filter", default_value="pf", choices=["pf", "ekf"])]
    for filename in components:
        arguments = {name: LaunchConfiguration(name) for name in ("use_usbl", "localization_filter")} \
            if filename == "localization.launch.py" else {}
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_directory, filename)),
            launch_arguments=arguments.items(),
        ))
    return LaunchDescription(actions)
