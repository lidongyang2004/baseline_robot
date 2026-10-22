"""Publish the vehicle model and fixed sensor/thruster transforms."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = (
        Path(get_package_share_directory("baseline_robot_description"))
        / "urdf" / "baseline_robot" / "base.urdf"
    )
    return LaunchDescription([
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace="baseline_robot", name="robot_state_publisher",
            parameters=[{
                "robot_description": urdf.read_text(encoding="utf-8"),
                "frame_prefix": "baseline_robot/",
            }],
        ),
    ])
