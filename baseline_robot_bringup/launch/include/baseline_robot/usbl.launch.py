"""Optional geographic acoustic adapter; does not implement USBL fusion."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mvp_acomm_utilities", executable="acomm_geopoint_node",
            namespace="baseline_robot", name="acomm_geopoint_node",
            output="screen", prefix=["stdbuf -o L"],
            parameters=[{
                "tf_prefix": "baseline_robot",
                "use_reference_geopose_orientation": True,
                "geopose_frame_id": "base_link",
            }],
            remappings=[("reference_geopose", "odometry/geopose")],
        ),
    ])
