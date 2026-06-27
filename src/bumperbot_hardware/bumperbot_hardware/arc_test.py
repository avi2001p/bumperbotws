#!/usr/bin/env python3
"""
arc_test.py
-----------
Drive ONE arc (default: 0.6 m radius, 180 deg = one stadium semicircle end) and
report. Two purposes:
  1. Validate the curved motion before running the full coverage pattern.
  2. Calibrate WHEEL_BASE using a gentle MOVING turn (far less wheel slip than
     the in-place spin, which scrubbed and slipped).

It commands the SAME math the coverage planner uses:
    linear.x  = speed
    angular.z = speed / radius
until ODOMETRY reports it has turned `angle_deg`, then stops.

WHEEL_BASE calibration:
  Mark the robot's START heading (a tape arrow). After a 180 deg command it
  should physically face the OPPOSITE way.
    * physically turned MORE than 180 -> WHEEL_BASE too BIG  -> lower it
    * physically turned LESS than 180 -> WHEEL_BASE too SMALL -> raise it
    WHEEL_BASE_new = WHEEL_BASE_old * (commanded_deg / actual_deg)

Run (hardware.launch.py already running, ~1 m clear space around the robot):
  ros2 run bumperbot_hardware arc_test
  ros2 run bumperbot_hardware arc_test --ros-args -p radius:=0.6 -p angle_deg:=180 -p speed:=0.15
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from bumperbot_hardware.parameters import (
    CMD_VEL_TOPIC,
    ODOM_TOPIC,
    WHEEL_BASE,
    MAX_LINEAR_SPEED,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ArcTest(Node):

    def __init__(self):
        super().__init__("arc_test")

        self.declare_parameter("radius", 0.6)       # m (stadium semicircle = 0.6)
        self.declare_parameter("angle_deg", 180.0)  # deg to turn
        self.declare_parameter("speed", 0.15)       # m/s forward speed

        self.radius = float(self.get_parameter("radius").value)
        self.angle = math.radians(float(self.get_parameter("angle_deg").value))
        self.speed = min(float(self.get_parameter("speed").value), MAX_LINEAR_SPEED)

        self.cum = 0.0          # accumulated UNWRAPPED heading (rad)
        self.last_yaw = None
        self.start_x = None
        self.start_y = None
        self.x = 0.0
        self.y = 0.0
        self.done = False

        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  ARC TEST")
        self.get_logger().info(f"  radius={self.radius:.2f} m  angle={math.degrees(self.angle):.0f} deg"
                               f"  speed={self.speed:.2f} m/s")
        self.get_logger().info(f"  WHEEL_BASE = {WHEEL_BASE:.4f} m")
        self.get_logger().info("  Mark the robot's start heading, then watch the turn.")
        self.get_logger().info("=" * 60)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.last_yaw is None:
            self.last_yaw = yaw
            self.start_x = self.x
            self.start_y = self.y
            return
        d = yaw - self.last_yaw
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        self.cum += d
        self.last_yaw = yaw

    def loop(self):
        if self.last_yaw is None:
            return
        if self.done or abs(self.cum) >= self.angle:
            self.stop()
            if not self.done:
                self.done = True
                self.report()
            return
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.speed / self.radius   # +ve = left (CCW) turn
        self.pub.publish(twist)

    def report(self):
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  ARC COMPLETE")
        self.get_logger().info(f"  Odometry turned : {math.degrees(self.cum):.1f} deg "
                               f"(commanded {math.degrees(self.angle):.0f})")
        self.get_logger().info(f"  End offset from start: dx={dx:+.3f} m  dy={dy:+.3f} m")
        self.get_logger().info("  For a 180 deg arc the robot should face the OPPOSITE way.")
        self.get_logger().info("   - turned a bit MORE -> WHEEL_BASE too big  -> lower it")
        self.get_logger().info("   - turned a bit LESS -> WHEEL_BASE too small -> raise it")
        self.get_logger().info("  Tell me: did it over/under-turn, and by roughly how much?")
        self.get_logger().info("=" * 60)

    def stop(self):
        try:
            if rclpy.ok():
                self.pub.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ArcTest()
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
