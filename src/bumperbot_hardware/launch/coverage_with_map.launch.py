"""
coverage_with_map.launch.py
---------------------------
FINAL autonomous run using the ALREADY-SAVED map (no re-mapping).

Pipeline (all on the Pi):
  1. hardware.launch.py      — encoders + PID + motors + wheel ODOMETRY (drives)
  2. rplidar_c1              — /scan in the 'laser' frame
  3. localization.launch.py  — slam_toolbox LOCALIZATION mode: loads the saved
                               'arena' map and publishes map->odom (drift fix).
                               This LOADS the map — it does NOT build a new one.
  4. stadium_coverage        — drives the concentric stadium pattern
  5. water_actuator          — vacuum + fan control

TF tree:  map --(localization)--> odom --(wheel odom)--> base_link --> laser

Usage:
  ros2 launch bumperbot_hardware coverage_with_map.launch.py
  ros2 launch bumperbot_hardware coverage_with_map.launch.py pose_source:=map
  ros2 launch bumperbot_hardware coverage_with_map.launch.py auto_start:=false linear_speed:=0.08

Args:
  pose_source   odom (default, smooth/reliable) | map (drift-corrected localized pose)
  auto_start    true (default) — start covering once a pose is available
  linear_speed  forward speed for the coverage pattern (m/s)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pose_source_arg = DeclareLaunchArgument("pose_source", default_value="odom")
    auto_start_arg = DeclareLaunchArgument("auto_start", default_value="true")
    linear_speed_arg = DeclareLaunchArgument("linear_speed", default_value="0.08")

    # 1. Hardware: encoders + PID + motors + wheel odometry + static base_link->laser
    hardware_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_hardware"),
            "launch", "hardware.launch.py",
        ),
    )

    # 2. RPLiDAR C1 — frame_id MUST be 'laser' to match the static TF
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

    # 3. LOCALIZATION against the saved map (loads arena.posegraph, no re-mapping)
    localization_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch", "localization.launch.py",
        ),
    )

    # 4. Stadium coverage planner (drives the pattern)
    stadium_coverage = Node(
        package="bumperbot_coverage",
        executable="stadium_coverage",
        name="stadium_coverage",
        output="screen",
        parameters=[{
            "pose_source": LaunchConfiguration("pose_source"),
            "auto_start": LaunchConfiguration("auto_start"),
            "linear_speed": LaunchConfiguration("linear_speed"),
            "use_lidar_safety": True,
            "safety_distance": 0.18,
            "safety_cone_deg": 20.0,
        }],
    )

    # 5. Water removal actuator (vacuum + fan)
    water_actuator = Node(
        package="bumperbot_hardware",
        executable="water_actuator",
        name="water_actuator",
        output="screen",
        parameters=[{
            "fan_duration": 5.0,
            "use_gpio_sensor": False,
            "cooldown_time": 2.0,
        }],
    )

    return LaunchDescription([
        pose_source_arg,
        auto_start_arg,
        linear_speed_arg,
        hardware_launch,
        rplidar_launch,
        localization_launch,
        stadium_coverage,
        water_actuator,
    ])
