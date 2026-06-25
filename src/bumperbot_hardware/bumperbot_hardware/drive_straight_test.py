#!/usr/bin/env python3
"""
drive_straight_test.py
----------------------
Standalone bring-up / calibration tool: drive the robot straight for a fixed
distance, holding heading with a P-controller on odometry, then stop and print
the measured distance.

Use this BEFORE the full coverage stack to confirm:
  * /cmd_vel actually reaches the PID (PID status should show non-zero out=)
  * the robot drives in a straight line (heading held)
  * 1.0 m commanded == ~1.0 m measured on the floor (TICKS_PER_REV is correct)

Subscribes:  /odom            (nav_msgs/Odometry)
Publishes:   /cmd_vel         (geometry_msgs/Twist)

Run (with hardware.launch.py already running):
  ros2 run bumperbot_hardware drive_straight_test
  ros2 run bumperbot_hardware drive_straight_test --ros-args -p distance:=1.0 -p speed:=0.12
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from bumperbot_hardware.parameters import (
    CMD_VEL_TOPIC,
    ODOM_TOPIC,
    MAX_LINEAR_SPEED,
    KP_HEADING,
    MAX_HEADING_CORRECTION,
)


def yaw_from_quaternion(q):
    """Extract yaw (rad) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class DriveStraightTest(Node):

    def __init__(self):
        super().__init__("drive_straight_test")

        # --- Parameters ---
        self.declare_parameter("distance", 1.0)          # meters to travel
        self.declare_parameter("speed", 0.12)            # m/s (gentle for testing)
        self.declare_parameter("heading_gain", KP_HEADING)

        self.target_distance = self.get_parameter("distance").value
        self.speed = min(self.get_parameter("speed").value, MAX_LINEAR_SPEED)
        self.heading_gain = self.get_parameter("heading_gain").value

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.start_x = None
        self.start_y = None
        self.start_theta = 0.0
        self.finished = False

        # --- ROS interfaces ---
        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Drive-straight test: target={self.target_distance:.2f} m "
            f"at {self.speed:.2f} m/s (heading_gain={self.heading_gain:.2f}). "
            f"Waiting for /odom..."
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = yaw_from_quaternion(msg.pose.pose.orientation)

        # Latch the starting pose on the first odom message
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.start_theta = self.theta
            self.get_logger().info("Odometry received — starting to drive.")

    def control_loop(self):
        if self.start_x is None or self.finished:
            return

        dx = self.x - self.start_x
        dy = self.y - self.start_y
        traveled = math.sqrt(dx * dx + dy * dy)

        if traveled >= self.target_distance:
            self.stop()
            self.finished = True
            self.get_logger().info(
                f"=== DONE === commanded {self.target_distance:.3f} m, "
                f"measured (odom) {traveled:.3f} m. "
                f"Measure the floor distance and adjust TICKS_PER_REV if they differ."
            )
            return

        # Heading-hold: correct any drift from the initial heading
        heading_error = normalize_angle(self.theta - self.start_theta)
        correction = -self.heading_gain * heading_error
        correction = max(-MAX_HEADING_CORRECTION,
                         min(MAX_HEADING_CORRECTION, correction))

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = correction
        self.cmd_pub.publish(twist)

    def stop(self):
        # Guard against publishing while the context is already shutting down
        # (e.g. on Ctrl+C), which raises RCLError.
        try:
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DriveStraightTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
