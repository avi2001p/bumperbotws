"""
water_coverage_mission.launch.py
--------------------------------
FULL autonomous water-extraction mission — the one-command demo.

Starts everything:
  1. hardware.launch.py     — encoders + PID + motors + wheel odometry
  2. RPLiDAR C1             — /scan (frame 'laser')
  3. localization.launch.py — loads the saved 'arena' map (for RViz/demo)   [use_map:=true]
  4. wall_follow_coverage   — lidar wall-following spiral coverage
  5. water_actuator         — 2 water sensors -> pause + vacuum + fan -> resume

Flow: coverage drives the spiral; when a water sensor trips, water_actuator
publishes /water_cleaning_active, coverage pauses (PAUSED_WATER), vacuum+fan run
for fan_duration, then coverage resumes.

Usage:
  ros2 launch bumperbot_hardware water_coverage_mission.launch.py
  ros2 launch bumperbot_hardware water_coverage_mission.launch.py use_map:=false
  ros2 launch bumperbot_hardware water_coverage_mission.launch.py relay_active_high:=true

RViz (laptop): rviz2, Fixed Frame = map -> shows the saved map + robot covering it.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_map_arg = DeclareLaunchArgument("use_map", default_value="true")
    relay_arg = DeclareLaunchArgument("relay_active_high", default_value="false")
    fan_duration_arg = DeclareLaunchArgument("fan_duration", default_value="5.0")

    hardware_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_hardware"),
            "launch", "hardware.launch.py",
        ),
    )

    rplidar_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rplidar_ros"),
            "launch", "rplidar_c1_launch.py",
        ),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": "460800",
            "frame_id": "laser",
        }.items(),
    )

    # Load the saved map (localization) — for RViz/demo; coverage drives wall-relative
    localization_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch", "localization.launch.py",
        ),
        condition=IfCondition(LaunchConfiguration("use_map")),
    )

    wall_follow_coverage = Node(
        package="bumperbot_coverage",
        executable="wall_follow_coverage",
        name="wall_follow_coverage",
        output="screen",
        # Tuned defaults live in the node; auto_start begins on first lidar data.
    )

    # Simple water logic: EITHER sensor wet -> vacuum + fan ON (+ pause coverage);
    # both dry -> OFF (+ resume). Keeps cleaning until the water is gone.
    water_clean = Node(
        package="bumperbot_hardware",
        executable="water_clean",
        name="water_clean",
        output="screen",
        parameters=[{
            "relay_active_high": LaunchConfiguration("relay_active_high"),
        }],
    )

    return LaunchDescription([
        use_map_arg,
        relay_arg,
        fan_duration_arg,
        hardware_launch,
        rplidar_launch,
        localization_launch,
        wall_follow_coverage,
        water_clean,
    ])
