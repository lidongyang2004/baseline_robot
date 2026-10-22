"""Connect ENU/NED reference frames and optionally open RViz."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("baseline_robot_description"),
                          "config", "config.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="baseline_robot_world_ned",
            arguments=[
                "--yaw", str(math.pi / 2), "--pitch", "0", "--roll", str(math.pi),
                "--frame-id", "baseline_robot/world",
                "--child-frame-id", "baseline_robot/world_ned",
            ],
        ),
        # Stonefish's ground-truth odometry uses the unprefixed world_ned frame.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="stonefish_world_ned",
            arguments=["--frame-id", "baseline_robot/world_ned", "--child-frame-id", "world_ned"],
        ),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", config],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
