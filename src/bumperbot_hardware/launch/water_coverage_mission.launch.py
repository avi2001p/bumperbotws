"""
water_coverage_mission.launch.py
--------------------------------
FULL autonomous water-extraction mission — the ONE-COMMAND demo.

Starts everything in a single launch:
  1. hardware.launch.py    — encoders + PID + motors + wheel odometry
  2. RPLiDAR C1            — /scan (frame 'laser')
  3. wall_follow_coverage — lidar wall-following spiral coverage
  4. water_clean          — 2 water sensors -> stop + vacuum + fan (+ roller) -> resume
  5. (optional) the saved map for RViz, with use_map:=true

Flow: coverage drives the spiral; when a water sensor trips, water_clean publishes
/water_cleaning_active, coverage pauses for 5 s with the vacuum + fan on, then
RESUMES while they keep running for another 5 s (10 s total). The roller drops
during the moving phase and lifts afterward.

Usage:
  # PROTOTYPE / VIVA arena (1.2 x 2.4 m stadium, smooth floor) — just:
  ros2 launch bumperbot_hardware water_coverage_mission.launch.py

  # a bit faster / slower:
  ros2 launch bumperbot_hardware water_coverage_mission.launch.py speed:=0.25

  # outdoor rectangle ground (square corners, measured width):
  ros2 launch bumperbot_hardware water_coverage_mission.launch.py \
      arena_shape:=rectangle arena_short_side:=1.48 kff:=0.55

  # show the saved map in RViz (Fixed Frame = map):
  ros2 launch bumperbot_hardware water_coverage_mission.launch.py use_map:=true

Roller needs the pigpio daemon first:  sudo pigpiod
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # --- Arguments (all have viva-ready defaults) ---
    # "stadium" = prototype arena (curved ends). "rectangle" = outdoor grounds.
    shape_arg = DeclareLaunchArgument("arena_shape", default_value="stadium")
    # Arena short side (m). 1.2 = prototype width; set the measured width for a
    # rectangle so the inward spiral stops at the centre, not the far wall.
    short_side_arg = DeclareLaunchArgument("arena_short_side", default_value="1.2")
    # Cruise speed. 0.20 is the tuned prototype speed; the node eases to 0.10 on
    # the curved ends on its own.
    speed_arg = DeclareLaunchArgument("speed", default_value="0.20")
    # Feed-forward gain. 0.38 suits the smooth prototype floor; rough ground needs
    # more (0.55).
    kff_arg = DeclareLaunchArgument("kff", default_value="0.38")
    # Relay board polarity (active-LOW module -> false).
    relay_arg = DeclareLaunchArgument("relay_active_high", default_value="false")
    # Roller angles from servo_test calibration.
    roller_up_arg = DeclareLaunchArgument("roller_up_angle", default_value="20.0")
    roller_down_arg = DeclareLaunchArgument("roller_down_angle", default_value="0.0")
    # Load the saved map for RViz (visual only — coverage never needs it).
    use_map_arg = DeclareLaunchArgument("use_map", default_value="false")
    # Stop-and-wait obstacle handling: ON by default. If something is in front of
    # the robot it STOPS and waits until the object is removed, then continues.
    obstacle_arg = DeclareLaunchArgument("obstacle_stop", default_value="true")
    obstacle_dist_arg = DeclareLaunchArgument("obstacle_distance", default_value="0.30")

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

    # Optional saved map for RViz/demo; coverage drives wall-relative regardless.
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
        parameters=[{
            "arena_shape": LaunchConfiguration("arena_shape"),
            "arena_short_side": LaunchConfiguration("arena_short_side"),
            "linear_speed_max": LaunchConfiguration("speed"),
            # keep moving through the curved ends but never stall
            "linear_speed_min": 0.10,
            # stop-and-wait if something is in front of the robot
            "obstacle_stop_enable": LaunchConfiguration("obstacle_stop"),
            "obstacle_stop_distance": LaunchConfiguration("obstacle_distance"),
        }],
    )

    # EITHER sensor wet -> stop, vacuum + fan + roller for 10 s, resume.
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
            # servo stays still until water is detected (MG995 startup-jerk fix)
            "park_at_start": False,
        }],
    )

    return LaunchDescription([
        shape_arg,
        short_side_arg,
        speed_arg,
        kff_arg,
        relay_arg,
        roller_up_arg,
        roller_down_arg,
        use_map_arg,
        obstacle_arg,
        obstacle_dist_arg,
        hardware_launch,
        rplidar_launch,
        localization_launch,
        wall_follow_coverage,
        water_clean,
    ])
