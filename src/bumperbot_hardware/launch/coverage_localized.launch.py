"""
coverage_localized.launch.py
----------------------------
Autonomous stadium coverage.

  pose_source:=odom  (DEFAULT, reliable)
      Drives the pattern using WHEEL ODOMETRY heading — the signal we proved
      accurate (straight = 95%, single arc = exactly 180°). No lidar needed.

  pose_source:=map   (experimental)
      Uses slam_toolbox LOCALIZATION against the saved 'arena' map for a
      drift-corrected pose. NOTE: in this small SYMMETRIC stadium the localized
      heading can be ambiguous (both ends look alike), which can make the robot
      circle. Only use if odometry drift is unacceptable AND localization is
      confirmed solid in RViz first.

Starts:
  - hardware                 (encoder, PID, motor, odometry, base_link->laser TF)
  - RPLidar C1               (only when pose_source:=map)
  - slam_toolbox LOCALIZATION(only when pose_source:=map)
  - stadium_coverage

Usage:
  ros2 launch bumperbot_hardware coverage_localized.launch.py
  ros2 launch bumperbot_hardware coverage_localized.launch.py linear_speed:=0.12
  ros2 launch bumperbot_hardware coverage_localized.launch.py pose_source:=map
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hw_dir = get_package_share_directory("bumperbot_hardware")
    lidar_dir = get_package_share_directory("rplidar_ros")
    slam_dir = get_package_share_directory("bumperbot_mapping")

    pose_source_arg = DeclareLaunchArgument("pose_source", default_value="odom")
    linear_speed_arg = DeclareLaunchArgument("linear_speed", default_value="0.15")
    auto_start_arg = DeclareLaunchArgument("auto_start", default_value="true")
    use_lidar_safety_arg = DeclareLaunchArgument("use_lidar_safety", default_value="false")
    # Gap (m) between the robot's OUTER edge and the wall, on straights + turns.
    # Each 0.01 less = +0.02 m (2 cm) semicircle diameter. 0.025 = +5 cm vs the
    # original 0.05. Bump up if it scrapes the wall.
    wall_clearance_arg = DeclareLaunchArgument("wall_clearance", default_value="0.025")

    pose_source = LaunchConfiguration("pose_source")

    # lidar + localization only run in map mode
    want_map = IfCondition(PythonExpression(["'", pose_source, "' == 'map'"]))

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
        condition=want_map,
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", "localization.launch.py")
        ),
        condition=want_map,
    )

    coverage = Node(
        package="bumperbot_coverage",
        executable="stadium_coverage",
        name="stadium_coverage",
        output="screen",
        parameters=[{
            "pose_source": pose_source,
            "use_lidar_safety": LaunchConfiguration("use_lidar_safety"),
            "linear_speed": LaunchConfiguration("linear_speed"),
            "auto_start": LaunchConfiguration("auto_start"),
            "wall_clearance": LaunchConfiguration("wall_clearance"),
        }],
    )

    return LaunchDescription([
        pose_source_arg,
        linear_speed_arg,
        auto_start_arg,
        use_lidar_safety_arg,
        wall_clearance_arg,
        hardware,
        lidar,
        localization,
        coverage,
    ])
