"""Convert the fixed station's acoustic measurements to ENU vehicle position."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("baseline_robot_bringup"),
                          "config", "baseline_robot", "usbl.yaml")
    return LaunchDescription([
        Node(package="baseline_robot_bringup", executable="usbl_adapter.py",
             namespace="baseline_robot", name="usbl_adapter", output="screen",
             parameters=[config]),
    ])
