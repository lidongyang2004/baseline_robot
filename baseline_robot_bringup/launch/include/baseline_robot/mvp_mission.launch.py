"""Start the mission manager after the low-level controller initializes."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("baseline_robot_bringup"),
                          "config", "baseline_robot")
    with open(os.path.join(params, "bhv_params.yaml"), encoding="utf-8") as stream:
        behaviors = yaml.safe_load(stream)
    behavior_params = {
        f"{name}/{key}": value
        for name, settings in behaviors.items()
        for key, value in settings.items()
    }
    helm_config = os.path.join(get_package_share_directory("baseline_robot_config"),
                               "mvp_mission_config", "baseline_robot", "helm.yaml")
    # MVP Control starts at 5 s; avoid querying its modes during initialization.
    return LaunchDescription([
        TimerAction(period=10.0, actions=[
            Node(
                package="mvp_helm", executable="mvp_helm",
                namespace="baseline_robot", name="mvp_helm", output="screen",
                remappings=[("datum", "gps/datum")],
                parameters=[{"helm_config_file": helm_config}, {"tf_prefix": "baseline_robot"},
                            os.path.join(params, "mvp_mission.yaml"), behavior_params],
            ),
        ]),
    ])
