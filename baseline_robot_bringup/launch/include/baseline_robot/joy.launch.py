"""Optional joystick input; not included in the default simulation."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy", executable="joy_node",
            namespace="baseline_robot", name="joy_node", output="screen",
            remappings=[("joy", "mvp_helm/bhv_teleop/joy")],
            parameters=[{"autorepeat_rate": 0.0}],
        ),
    ])
