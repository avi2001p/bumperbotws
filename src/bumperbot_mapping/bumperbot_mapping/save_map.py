#!/usr/bin/env python3
"""
save_map.py
-----------
Utility node to save the current SLAM Toolbox map to disk.

Calls the nav2_map_server's /map_saver/save_map service to save
the occupancy grid as .pgm + .yaml files.

Usage:
  # Save with default name (arena_map) to ~/bumperbot_wsv2/maps/
  ros2 run bumperbot_mapping save_map

  # Save with custom name and path
  ros2 run bumperbot_mapping save_map --ros-args -p map_name:=my_map -p save_dir:=/home/pi/maps
"""

import os
import subprocess
import rclpy
from rclpy.node import Node


class MapSaver(Node):

    def __init__(self):
        super().__init__("map_saver")

        self.declare_parameter("map_name", "arena_map")
        self.declare_parameter("save_dir", "")

        map_name = self.get_parameter("map_name").value
        save_dir = self.get_parameter("save_dir").value

        # Default save directory: workspace maps/ folder
        if not save_dir:
            save_dir = os.path.expanduser("~/bumperbot_wsv2/maps")

        os.makedirs(save_dir, exist_ok=True)
        map_path = os.path.join(save_dir, map_name)

        self.get_logger().info(f"Saving map to: {map_path}")
        self.get_logger().info("Waiting 2 seconds for map data to stabilize...")

        # Use a one-shot timer to save after a short delay
        self.map_path = map_path
        self.timer = self.create_timer(2.0, self.save_map)

    def save_map(self):
        """Call ros2 CLI to save the map via the map_saver_server."""
        self.timer.cancel()

        try:
            result = subprocess.run(
                [
                    "ros2", "service", "call",
                    "/map_saver/save_map",
                    "nav2_msgs/srv/SaveMap",
                    f'{{"map_topic": "/map", "map_url": "{self.map_path}", '
                    f'"image_format": "pgm", "map_mode": "trinary", '
                    f'"free_thresh": 0.196, "occupied_thresh": 0.65}}',
                ],
                capture_output=True,
                text=True,
                timeout=15,
            )

            if result.returncode == 0:
                self.get_logger().info(
                    f"Map saved successfully!\n"
                    f"  PGM: {self.map_path}.pgm\n"
                    f"  YAML: {self.map_path}.yaml"
                )
            else:
                self.get_logger().warn(
                    f"Map save may have failed. stderr: {result.stderr.strip()}"
                )
                # Fallback: try ros2 run nav2_map_server map_saver_cli
                self.get_logger().info("Trying fallback method...")
                fallback = subprocess.run(
                    [
                        "ros2", "run", "nav2_map_server", "map_saver_cli",
                        "-f", self.map_path,
                        "--ros-args", "-p", "save_map_timeout:=10.0",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
                if fallback.returncode == 0:
                    self.get_logger().info(
                        f"Map saved via fallback!\n"
                        f"  PGM: {self.map_path}.pgm\n"
                        f"  YAML: {self.map_path}.yaml"
                    )
                else:
                    self.get_logger().error(
                        f"Fallback also failed: {fallback.stderr.strip()}"
                    )

        except subprocess.TimeoutExpired:
            self.get_logger().error("Map save timed out (15s). Is map_saver_server running?")
        except Exception as e:
            self.get_logger().error(f"Error saving map: {e}")

        # Shutdown after saving
        self.get_logger().info("Map saver shutting down.")
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
