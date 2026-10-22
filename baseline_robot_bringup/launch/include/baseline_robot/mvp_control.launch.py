"""Start low-level control after the simulator and sensor interfaces."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory("baseline_robot_bringup"),
        "config", "baseline_robot", "mvp_control.yaml",
    )
    controller_config = os.path.join(
        get_package_share_directory("baseline_robot_config"),
        "mvp_control_config", "baseline_robot", "config.yaml",
    )
    return LaunchDescription([
        TimerAction(period=5.0, actions=[
            Node(
                package="mvp_control", executable="mvp_control_ros_node",
                namespace="baseline_robot", name="mvp_control_ros_node",
                prefix=["stdbuf -o L"], output="screen",
                parameters=[
                    {"config_file": controller_config},
                    {"tf_prefix": "baseline_robot"},
                    {"odometry_source": "/baseline_robot/odometry/filtered"},
                    parameters,
                ],
            ),
        ]),
    ])
