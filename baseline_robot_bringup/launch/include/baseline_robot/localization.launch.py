"""Fuse navigation sensors and optional USBL, using a known simulation origin."""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ekf_parameters(config, use_usbl):
    with open(os.path.join(config, "localization.yaml"), encoding="utf-8") as source:
        parameters = yaml.safe_load(source)["/baseline_robot/ekf_filter_node"]["ros__parameters"]
    # PyYAML treats e.g. 1e-1 as strings, whereas the ROS YAML parser accepts it.
    parameters["process_noise_covariance"] = [float(v) for v in parameters["process_noise_covariance"]]
    if not use_usbl:
        parameters = {key: value for key, value in parameters.items()
                      if key != "odom1" and not key.startswith("odom1_")
                      and key not in ("smooth_lagged_data", "history_length")}
    return parameters


def start_ekf(context, config):
    enabled = IfCondition(LaunchConfiguration("use_usbl")).evaluate(context)
    return [Node(package="robot_localization", executable="ekf_node",
                 namespace="baseline_robot", name="ekf_filter_node",
                 parameters=[ekf_parameters(config, enabled)])]


def generate_launch_description():
    config = os.path.join(get_package_share_directory("baseline_robot_bringup"),
                          "config", "baseline_robot")
    return LaunchDescription([
        DeclareLaunchArgument("use_usbl", default_value="true",
                              description="Fuse two-way USBL horizontal position"),
        OpaqueFunction(function=start_ekf, args=[config]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(os.path.dirname(__file__), "usbl.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_usbl")),
        ),
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
