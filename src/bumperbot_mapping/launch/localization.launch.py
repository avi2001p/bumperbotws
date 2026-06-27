"""
localization.launch.py
----------------------
Run slam_toolbox in LOCALIZATION mode against the saved 'arena' map.

It loads the map's posegraph (/home/pi/BMP/maps/arena.posegraph + .data) and
publishes the map->odom correction, so the robot's pose stops drifting and the
coverage planner (pose_source:=map) can follow the pattern accurately.

Usage:
  ros2 launch bumperbot_mapping localization.launch.py
"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "config",
            "localization.yaml",
        ),
        description="slam_toolbox localization YAML",
    )

    localization = Node(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_config"),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        slam_config_arg,
        localization,
    ])
