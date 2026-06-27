"""
coverage_localized.launch.py
----------------------------
Autonomous stadium coverage using LIDAR LOCALIZATION to correct odometry drift.

Starts:
  - hardware       (encoder, PID, motor, odometry, base_link->laser TF)
  - RPLidar C1     (/scan)
  - slam_toolbox LOCALIZATION  (loads 'arena' map -> publishes map->odom)
  - stadium_coverage with pose_source:=map  (drift-corrected pose)

Notes:
  * The lidar is used for LOCALIZATION here. The simple obstacle-safety is OFF
    (use_lidar_safety:=false) because the border walls would false-trigger it;
    supervise the run. (Map-based obstacle filtering is a later add-on.)
  * Place the robot where mapping started (map origin). If localization looks
    off in RViz, set the pose with "2D Pose Estimate" before it moves.

Usage:
  ros2 launch bumperbot_hardware coverage_localized.launch.py
  ros2 launch bumperbot_hardware coverage_localized.launch.py linear_speed:=0.12 auto_start:=false
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hw_dir = get_package_share_directory("bumperbot_hardware")
    lidar_dir = get_package_share_directory("rplidar_ros")
    slam_dir = get_package_share_directory("bumperbot_mapping")

    linear_speed_arg = DeclareLaunchArgument("linear_speed", default_value="0.15")
    auto_start_arg = DeclareLaunchArgument("auto_start", default_value="true")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_dir, "launch", "hardware.launch.py")
        )
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, "launch", "rplidar_c1_launch.py")
        ),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": "460800",
            "frame_id": "laser",
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", "localization.launch.py")
        )
    )

    coverage = Node(
        package="bumperbot_coverage",
        executable="stadium_coverage",
        name="stadium_coverage",
        output="screen",
        parameters=[{
            "pose_source": "map",          # use the drift-corrected localized pose
            "use_lidar_safety": False,     # border walls would false-trigger it
            "linear_speed": LaunchConfiguration("linear_speed"),
            "auto_start": LaunchConfiguration("auto_start"),
        }],
    )

    return LaunchDescription([
        linear_speed_arg,
        auto_start_arg,
        hardware,
        lidar,
        localization,
        coverage,
    ])
