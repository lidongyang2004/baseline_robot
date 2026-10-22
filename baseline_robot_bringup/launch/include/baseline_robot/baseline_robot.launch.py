"""Bridge Stonefish sensors and thrusters to the vehicle's ROS interfaces."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    vehicle = LaunchConfiguration("arg_vehicle_name")
    world_frame = LaunchConfiguration("arg_world_frame")
    imu_frame = LaunchConfiguration("arg_imu_frame")
    driver_parameters = PathJoinSubstitution([
        get_package_share_directory("baseline_robot_bringup"),
        "config", vehicle, "sim_params.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("arg_vehicle_name", default_value="baseline_robot"),
        DeclareLaunchArgument("arg_world_frame", default_value="baseline_robot/world"),
        DeclareLaunchArgument("arg_imu_frame", default_value="baseline_robot/imu_sf"),
        Node(
            package="world_of_stonefish", executable="imu_driver_node",
            namespace=vehicle, name="imu_driver_node",
            remappings=[
                ("imu_in/data", "imu/stonefish/data"),
                ("imu_out/data", "imu/data"),
            ],
            parameters=[{"frame_id": imu_frame}, driver_parameters],
        ),
        Node(
            package="world_of_stonefish", executable="thruster_driver_node",
            namespace=vehicle, name="thruster_driver_node",
            parameters=[driver_parameters],
        ),
        Node(
            package="baseline_robot_bringup", executable="sensor_adapter.py",
            namespace=vehicle, name="sensor_adapter",
            parameters=[driver_parameters, {"world_frame": world_frame}],
        ),
        # The bridge stays available; the current scene has no USBL transceiver.
        Node(
            package="world_of_stonefish", executable="usbl_driver_node",
            namespace=vehicle, name="usbl_driver_node",
        ),
    ])
