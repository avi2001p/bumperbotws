"""
coverage_mission.launch.py
--------------------------
Complete autonomous coverage mission launch file for BumperBot.

Starts ALL nodes needed for autonomous stadium coverage with water removal:
  1. Hardware pipeline (encoder + PID + motor + odometry + robot_state_publisher)
  2. RPLiDAR C1 sensor
  3. SLAM Toolbox mapping (optional, enabled by default)
  4. Water detection & actuator control (vacuum + fan)
  5. Stadium coverage path planner

Usage:
  ros2 launch bumperbot_hardware coverage_mission.launch.py
  ros2 launch bumperbot_hardware coverage_mission.launch.py use_mapping:=false

Optional arguments:
  ground_width:=1.2  ground_straight_length:=1.2  linear_speed:=0.08
  use_mapping:=true  (enable/disable SLAM mapping)

Note: PID gains are set in hardware.launch.py / parameters.py, not here.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # === Launch arguments ===
    ground_width_arg = DeclareLaunchArgument("ground_width", default_value="1.2")
    ground_straight_arg = DeclareLaunchArgument(
        "ground_straight_length", default_value="1.2"
    )
    linear_speed_arg = DeclareLaunchArgument("linear_speed", default_value="0.08")
    use_mapping_arg = DeclareLaunchArgument(
        "use_mapping",
        default_value="true",
        description="Enable SLAM Toolbox mapping with RPLiDAR C1",
    )

    # === Include base hardware launch ===
    hardware_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_hardware"),
            "launch",
            "hardware.launch.py",
        ),
    )

    # === Include RPLiDAR C1 launch ===
    rplidar_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rplidar_ros"),
            "launch",
            "rplidar_c1_launch.py",
        ),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": "460800",
            # Must match the static TF child frame (base_link -> laser) published
            # in hardware.launch.py, or SLAM cannot transform the scan.
            "frame_id": "laser",
        }.items(),
    )

    # === Include SLAM Toolbox mapping (conditional) ===
    slam_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py",
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_mapping")),
    )

    # === Water actuator node ===
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

    # === Stadium coverage planner ===
    stadium_coverage = Node(
        package="bumperbot_coverage",
        executable="stadium_coverage",
        name="stadium_coverage",
        output="screen",
        parameters=[{
            "ground_width": LaunchConfiguration("ground_width"),
            "ground_straight_length": LaunchConfiguration("ground_straight_length"),
            "robot_coverage_width": 0.22,
            "overlap": 0.02,
            "linear_speed": LaunchConfiguration("linear_speed"),
            "auto_start": True,
            "use_lidar_safety": True,
            "safety_distance": 0.18,
            "safety_cone_deg": 20.0,
        }],
    )

    return LaunchDescription([
        ground_width_arg,
        ground_straight_arg,
        linear_speed_arg,
        use_mapping_arg,
        hardware_launch,
        rplidar_launch,
        slam_launch,
        water_actuator,
        stadium_coverage,
    ])
