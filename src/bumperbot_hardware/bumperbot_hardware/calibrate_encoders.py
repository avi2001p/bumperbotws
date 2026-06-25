#!/usr/bin/env python3
"""
calibrate_encoders.py
---------------------
Utility node to calibrate encoder ticks-per-revolution.

Usage:
  1. Launch this node: ros2 run bumperbot_hardware calibrate_encoders
  2. Manually rotate LEFT wheel exactly ONE full revolution
  3. Press Enter
  4. Manually rotate RIGHT wheel exactly ONE full revolution
  5. Press Enter
  6. The node prints the measured ticks — update TICKS_PER_REV in parameters.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from bumperbot_hardware.parameters import WHEEL_TICKS_TOPIC


class CalibrateEncoders(Node):

    def __init__(self):
        super().__init__("calibrate_encoders")

        self.left_ticks = 0
        self.right_ticks = 0

        self.create_subscription(
            Int32MultiArray,
            WHEEL_TICKS_TOPIC,
            self.tick_callback,
            10
        )

        self.get_logger().info("=" * 55)
        self.get_logger().info("  ENCODER CALIBRATION UTILITY")
        self.get_logger().info("=" * 55)
        self.get_logger().info("Make sure encoder_reader node is also running.")
        self.get_logger().info("")

        # Use a one-shot timer to start the interactive calibration
        # (so the ROS callbacks are active)
        self.create_timer(2.0, self.run_calibration)
        self.calibration_done = False

    def tick_callback(self, msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

    def run_calibration(self):
        if self.calibration_done:
            return
        self.calibration_done = True

        self.get_logger().info(f"Current ticks — L: {self.left_ticks}  R: {self.right_ticks}")
        self.get_logger().info("")
        self.get_logger().info(">>> Rotate the LEFT wheel exactly ONE full revolution.")
        self.get_logger().info("    Then come back and press Ctrl+C.")
        self.get_logger().info("")

        # Continuously print ticks
        self.print_timer = self.create_timer(0.5, self.print_ticks)

    def print_ticks(self):
        self.get_logger().info(
            f"  Live ticks — L: {self.left_ticks:+6d}  R: {self.right_ticks:+6d}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateEncoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("")
        node.get_logger().info("=" * 55)
        node.get_logger().info(f"  FINAL TICKS — Left: {node.left_ticks}  Right: {node.right_ticks}")
        node.get_logger().info(f"  Update TICKS_PER_REV in parameters.py with")
        node.get_logger().info(f"  the tick count from ONE wheel revolution.")
        node.get_logger().info("=" * 55)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
