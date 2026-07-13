"""
rough_ground_mission.launch.py
------------------------------
FULL water-extraction mission on the REAL (rough, rectangular) ground.

This is the outdoor twin of water_coverage_mission.launch.py. The difference is
not cosmetic — the rough surface changes what the robot can physically do:

  * It needs MORE PWM for the same speed (higher kff), because rolling
    resistance is far greater than on the smooth indoor tile.
  * It therefore has a MINIMUM usable speed. Below ~0.10 m/s the motors cannot
    break static friction and the robot buzzes without moving. The coverage
    node's habit of slowing near walls and on turns is disabled here for exactly
    that reason (turn_slow_k=0, curve_min_frac=1.0) — that slowdown is what
    stalled it.
  * The arena has SQUARE corners, so the wall-follower needs the explicit
    90 deg pivot (arena_shape:=rectangle) rather than the stadium curve
    feed-forward.
  * Lanes step HALF a robot width, so consecutive passes overlap 50% and a strip
    missed on one lap is swept on the next.

Water sequence on detection:
  STOP 5 s (vacuum + fan on, roller up)
    -> MOVE 5 s (vacuum + fan still on, roller DOWN, sweeping)
    -> roller up, vacuum + fan off, cooldown.

The roller needs the pigpio daemon. Before launching:
    sudo pigpiod

Usage:
  ros2 launch bumperbot_hardware rough_ground_mission.launch.py

  # slower / faster (raise kff as speed comes DOWN — see the table below)
  ros2 launch bumperbot_hardware rough_ground_mission.launch.py speed:=0.12 kff:=0.65

  # no roller fitted yet
  ros2 launch bumperbot_hardware rough_ground_mission.launch.py use_roller:=false

Speed / kff pairing (rough ground). Too little kff at a given speed = stall:
      0.15 m/s -> kff 0.55
      0.12 m/s -> kff 0.65
      0.10 m/s -> kff 0.78     <-- default: slowest that still moves reliably
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hw_dir = get_package_share_directory("bumperbot_hardware")
    lidar_dir = get_package_share_directory("rplidar_ros")

    # Slow enough for the water sensors to resolve a wet patch, but not so slow
    # that the motors stall on the rough surface.
    speed_arg = DeclareLaunchArgument("speed", default_value="0.10")
    kff_arg = DeclareLaunchArgument("kff", default_value="0.78")
    use_roller_arg = DeclareLaunchArgument("use_roller", default_value="true")
    relay_arg = DeclareLaunchArgument("relay_active_high", default_value="false")
    # Set these from your servo_test calibration.
    roller_up_arg = DeclareLaunchArgument("roller_up_angle", default_value="20.0")
    roller_down_arg = DeclareLaunchArgument("roller_down_angle", default_value="0.0")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_dir, "launch", "hardware.launch.py")
        ),
        launch_arguments={"kff": LaunchConfiguration("kff")}.items(),
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

    coverage = Node(
        package="bumperbot_coverage",
        executable="wall_follow_coverage",
        name="wall_follow_coverage",
        output="screen",
        parameters=[{
            "arena_shape": "rectangle",
            # Hold ONE speed. The node's usual easing on turns and near walls
            # drops it below the stall threshold on this surface.
            "linear_speed_max": LaunchConfiguration("speed"),
            "linear_speed_min": LaunchConfiguration("speed"),
            "turn_slow_k": 0.0,
            "curve_min_frac": 1.0,
            "corner_turn_speed": 1.2,
        }],
    )

    water_clean = Node(
        package="bumperbot_hardware",
        executable="water_clean",
        name="water_clean",
        output="screen",
        parameters=[{
            "relay_active_high": LaunchConfiguration("relay_active_high"),
            "stop_duration": 5.0,
            "move_duration": 5.0,
            "poll_rate": 20.0,
            "use_roller": LaunchConfiguration("use_roller"),
            "roller_up_angle": LaunchConfiguration("roller_up_angle"),
            "roller_down_angle": LaunchConfiguration("roller_down_angle"),
        }],
    )

    return LaunchDescription([
        speed_arg,
        kff_arg,
        use_roller_arg,
        relay_arg,
        roller_up_arg,
        roller_down_arg,
        hardware,
        lidar,
        coverage,
        water_clean,
    ])
