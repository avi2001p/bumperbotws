#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from bumperbot_hardware.parameters import *


class PIDController(Node):

    def __init__(self):

        super().__init__("pid_controller")

        self.get_logger().info("PID Controller Started")

        # Desired robot velocity
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Actual wheel speeds (ticks/sec)
        self.left_speed = 0.0
        self.right_speed = 0.0

        # Target wheel speeds (ticks/sec)
        self.target_left_speed = 0.0
        self.target_right_speed = 0.0

        # Subscribers
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            "/wheel_speed",
            self.speed_callback,
            10
        )

        # Print status
        self.timer = self.create_timer(
            0.2,
            self.print_status
        )

    def cmd_vel_callback(self, msg):

        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        # Differential drive inverse kinematics
        left_linear = (
            self.linear_x -
            (self.angular_z * WHEEL_BASE / 2.0)
        )

        right_linear = (
            self.linear_x +
            (self.angular_z * WHEEL_BASE / 2.0)
        )

        # Convert linear wheel speed to encoder ticks/sec
        self.target_left_speed = (
            left_linear / WHEEL_CIRCUMFERENCE
        ) * TICKS_PER_REV

        self.target_right_speed = (
            right_linear / WHEEL_CIRCUMFERENCE
        ) * TICKS_PER_REV

    def speed_callback(self, msg):

        self.left_speed = msg.data[0]
        self.right_speed = msg.data[1]

    def print_status(self):

        self.get_logger().info(
            f"""
---------------------------------------
Target Left : {self.target_left_speed:.2f} ticks/s
Actual Left : {self.left_speed:.2f} ticks/s

Target Right: {self.target_right_speed:.2f} ticks/s
Actual Right: {self.right_speed:.2f} ticks/s
---------------------------------------
"""
        )


def main(args=None):

    rclpy.init(args=args)

    node = PIDController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Stopping PID Controller...")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()