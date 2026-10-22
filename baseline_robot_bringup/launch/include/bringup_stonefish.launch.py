"""Load the repository-owned single-AUV world, plus its sensor drivers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("baseline_robot_bringup")
    assets = get_package_share_directory("world_of_stonefish")
    return LaunchDescription([
        Node(
            package="stonefish_ros2", executable="stonefish_simulator",
            name="stonefish_simulator", output="screen",
            arguments=[os.path.join(assets, "data", ""),
                       os.path.join(bringup, "scenarios", "single_auv.scn"),
                       "100", "1200", "800", "low"],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup, "launch", "include", "baseline_robot", "baseline_robot.launch.py")),
            launch_arguments={
                "arg_vehicle_name": "baseline_robot",
                "arg_world_frame": "baseline_robot/world",
                "arg_imu_frame": "baseline_robot/imu_sf",
            }.items(),
        ),
    ])
