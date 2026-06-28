#!/usr/bin/env python3
"""
wall_distance_monitor.py
------------------------
Prints the distance from the robot to the FRONT / LEFT / RIGHT walls using the
RPLidar C1, so you can:
  (a) confirm the lidar actually sees the walls, and
  (b) choose the turn-trigger distance for lidar-driven coverage.

The lidar is mounted yaw=3.14 rad, so ROBOT-FORWARD = lidar scan angle ~ +/- pi.

Run (with the lidar already publishing /scan):
  ros2 run bumperbot_hardware wall_distance_monitor
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def cone_distance(msg, center, half_width):
    """Median range of valid rays whose angle is within `half_width` of `center`.
    Returns NaN if no valid rays fall in the cone."""
    vals = []
    for i, r in enumerate(msg.ranges):
        if math.isinf(r) or math.isnan(r) or r < 0.05:
            continue
        a = msg.angle_min + i * msg.angle_increment
        d = a - center
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        if abs(d) <= half_width:
            vals.append(r)
    if not vals:
        return float("nan")
    vals.sort()
    return vals[len(vals) // 2]


class WallDistanceMonitor(Node):

    def __init__(self):
        super().__init__("wall_distance_monitor")
        self.create_subscription(LaserScan, "scan", self.cb, 10)
        self.get_logger().info(
            "Wall-distance monitor started. FRONT = lidar +/-pi (yaw=3.14 mount). "
            "LEFT/RIGHT are best-guess — verify by holding a board to one side."
        )

    def cb(self, msg):
        cone = math.radians(12.0)
        front = cone_distance(msg, math.pi, cone)        # robot forward
        right = cone_distance(msg, math.pi / 2.0, cone)  # robot right (verify)
        left = cone_distance(msg, -math.pi / 2.0, cone)  # robot left  (verify)
        self.get_logger().info(
            f"FRONT={front:5.2f} m | LEFT={left:5.2f} m | RIGHT={right:5.2f} m",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallDistanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
