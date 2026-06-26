"""
mapping.launch.py
-----------------
One-command SLAM mapping bring-up for BumperBot with the RPLidar C1.

Starts everything needed to BUILD a map:
  - hardware pipeline  (encoder_reader, pid, motor_driver, odometry,
                        and the static base_link -> laser TF)
  - RPLidar C1 driver  (publishes /scan in the 'laser' frame)
  - SLAM Toolbox       (builds the map, publishes /map and the map -> odom TF)

Then, in separate terminals:
  - drive with:   ros2 run teleop_twist_keyboard teleop_twist_keyboard
  - watch with:   rviz2   (Fixed Frame = map; add LaserScan /scan, Map /map, TF)
  - save with:    ros2 run nav2_map_server map_saver_cli -f ~/BMP/maps/arena
                  ros2 service call /slam_toolbox/serialize_map \
                      slam_toolbox/srv/SerializePoseGraph "{filename: '/home/pi/BMP/maps/arena'}"

Usage:
  ros2 launch bumperbot_hardware mapping.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hw_dir = get_package_share_directory("bumperbot_hardware")
    lidar_dir = get_package_share_directory("rplidar_ros")
    slam_dir = get_package_share_directory("bumperbot_mapping")

    # Core driving pipeline + odometry + base_link->laser TF
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_dir, "launch", "hardware.launch.py")
        )
    )

    # RPLidar C1 -> /scan (frame_id 'laser', 460800 baud by default)
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, "launch", "rplidar_c1_launch.py")
        )
    )

    # SLAM Toolbox (online async) -> /map and map->odom
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, "launch", "slam.launch.py")
        )
    )

    return LaunchDescription([
        hardware,
        lidar,
        slam,
    ])
