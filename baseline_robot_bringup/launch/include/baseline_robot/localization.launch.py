"""Fuse IMU, DVL and depth; establish the known simulation origin without GPS."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("baseline_robot_bringup"),
                          "config", "baseline_robot")
    return LaunchDescription([
        Node(package="robot_localization", executable="ekf_node",
             namespace="baseline_robot", name="ekf_filter_node",
             parameters=[os.path.join(config, "localization.yaml")]),
        # Known spawn x=y=0 and ENU axes, not a transform estimated from GPS.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="baseline_robot_world_odom",
             arguments=[
                 "--frame-id", "baseline_robot/world",
                 "--child-frame-id", "baseline_robot/odom",
             ]),
        Node(package="baseline_robot_bringup", executable="gps_truth.py",
             namespace="baseline_robot", name="gps_truth", output="screen",
             parameters=[os.path.join(config, "gps_truth.yaml")]),
    ])
