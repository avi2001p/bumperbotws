"""
straight_line_demo.launch.py
----------------------------
ONE-COMMAND 2 m STRAIGHT-LINE DEMO — shows every subsystem in a single run,
without needing the full arena. Ideal as a viva fallback.

Starts:
  1. hardware.launch.py    — encoders + PID + motors + wheel odometry
  2. RPLiDAR C1            — /scan
  3. obstacle_stop_test    — drives straight 2 m, stops for obstacles, pauses for water
  4. water_clean          — water sensors -> stop + vacuum + fan + roller -> resume

What the panel sees, in one 2 m line:
  * the robot drives straight and stops on its own at 2 m;
  * put a hand / box in front  -> it STOPS and waits, remove it -> it CONTINUES;
  * put water under a sensor    -> it STOPS 5 s (vacuum + fan on), then drives on
    5 s more with the roller down (10 s extraction total), then continues.

Roller needs the pigpio daemon first:  sudo pigpiod

Usage:
  ros2 launch bumperbot_hardware straight_line_demo.launch.py
  ros2 launch bumperbot_hardware straight_line_demo.launch.py distance:=2.0 speed:=0.12
  ros2 launch bumperbot_hardware straight_line_demo.launch.py kff:=0.55   # rough floor
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    distance_arg = DeclareLaunchArgument("distance", default_value="2.0")
    speed_arg = DeclareLaunchArgument("speed", default_value="0.12")
    stop_dist_arg = DeclareLaunchArgument("obstacle_distance", default_value="0.30")
    kff_arg = DeclareLaunchArgument("kff", default_value="0.38")
    relay_arg = DeclareLaunchArgument("relay_active_high", default_value="false")
    roller_up_arg = DeclareLaunchArgument("roller_up_angle", default_value="20.0")
    roller_down_arg = DeclareLaunchArgument("roller_down_angle", default_value="0.0")

    hardware_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_hardware"),
            "launch", "hardware.launch.py",
        ),
        launch_arguments={"kff": LaunchConfiguration("kff")}.items(),
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

    straight_line = Node(
        package="bumperbot_hardware",
        executable="obstacle_stop_test",
        name="obstacle_stop_test",
        output="screen",
        parameters=[{
            "distance": LaunchConfiguration("distance"),
            "speed": LaunchConfiguration("speed"),
            "stop_distance": LaunchConfiguration("obstacle_distance"),
        }],
    )

    water_clean = Node(
        package="bumperbot_hardware",
        executable="water_clean",
        name="water_clean",
        output="screen",
        parameters=[{
            "relay_active_high": LaunchConfiguration("relay_active_high"),
            "poll_rate": 50.0,
            "roller_up_angle": LaunchConfiguration("roller_up_angle"),
            "roller_down_angle": LaunchConfiguration("roller_down_angle"),
            "park_at_start": False,
        }],
    )

    return LaunchDescription([
        distance_arg,
        speed_arg,
        stop_dist_arg,
        kff_arg,
        relay_arg,
        roller_up_arg,
        roller_down_arg,
        hardware_launch,
        rplidar_launch,
        straight_line,
        water_clean,
    ])
