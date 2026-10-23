"""Single-AUV Stonefish simulation, localization, control and visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("baseline_robot_bringup")

    def include(filename, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "include", filename)),
            **kwargs,
        )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="Open RViz"),
        DeclareLaunchArgument("localization_filter", default_value="pf", choices=["pf", "ekf"],
                              description="Navigation estimator (pf default, ekf reference)"),
        DeclareLaunchArgument("use_usbl", default_value="true",
                              description="Use raw USBL range/bearing for PF or converted position for EKF"),
        DeclareLaunchArgument(
            "record_path", default_value="true", description="Publish navigation Path and CSV",
        ),
        DeclareLaunchArgument(
            "csv_path", default_value="",
            description=(
                "CSV file; empty creates a timestamped file in "
                "~/.ros/baseline_robot/navigation"
            ),
        ),
        include("bringup_stonefish.launch.py"),
        include("bringup_baseline_robot.launch.py",
                launch_arguments={name: LaunchConfiguration(name)
                                  for name in ("use_usbl", "localization_filter")}.items()),
        include(
            "bringup_visualization.launch.py",
            launch_arguments={"rviz": LaunchConfiguration("rviz")}.items(),
        ),
        Node(
            package="baseline_robot_bringup", executable="navigation_path.py",
            namespace="baseline_robot", name="navigation_path", output="screen",
            condition=IfCondition(LaunchConfiguration("record_path")),
            parameters=[{"csv_path": LaunchConfiguration("csv_path")}],
        ),
    ])
