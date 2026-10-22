"""Optional UDP command/reporting nodes; external MVP C2 is required."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = os.path.join(
        get_package_share_directory("baseline_robot_bringup"),
        "config", "baseline_robot", "mvp_c2.yaml",
    )
    return LaunchDescription([
        Node(
            package="mvp_c2", executable="mvp_c2_udp_comm",
            namespace="baseline_robot", name="reporter_c2_udp_comm",
            output="screen", prefix=["stdbuf -o L"], parameters=[parameters],
            remappings=[
                ("dccl_msg_tx", "mvp_c2/dccl_msg_tx"),
                ("dccl_msg_rx", "mvp_c2/dccl_msg_rx"),
            ],
        ),
        Node(
            package="mvp_c2", executable="mvp_c2_reporter_ros",
            namespace="baseline_robot", name="mvp_c2_reporter",
            output="screen", prefix=["stdbuf -o L"], parameters=[parameters],
            remappings=[
                ("local/odometry", "odometry/filtered"),
                ("local/geopose", "odometry/geopose"),
                ("joy", "mvp_helm/bhv_teleop/joy"),
                ("mvp_helm/path", "bhv_path_following/get_next_waypoints"),
                ("mvp_helm/set_waypoints", "bhv_path_following/update_waypoints"),
            ],
        ),
    ])
