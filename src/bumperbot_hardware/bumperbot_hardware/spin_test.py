#!/usr/bin/env python3
"""
spin_test.py
------------
Calibrate WHEEL_BASE by spinning the robot in place a known number of turns
(measured by odometry) and comparing to its ACTUAL physical rotation.

Why it works: the node stops the spin when ODOMETRY has accumulated exactly
`turns` full rotations. Odometry computes rotation as (d_right - d_left)/WHEEL_BASE,
so if WHEEL_BASE is wrong the physical rotation won't match the commanded turns:
  * robot physically spun MORE than `turns`  -> WHEEL_BASE is too BIG  -> lower it
  * robot physically spun LESS than `turns`  -> WHEEL_BASE is too SMALL -> raise it
New value:  WHEEL_BASE_new = WHEEL_BASE_old * (commanded_deg / actual_deg)

Best done AFTER the battery/fan are mounted (final weight = real traction = less
slip = accurate number).

How to use:
  1. Put a tape ARROW on top of the robot and align it to a fixed reference
     (a floor mark or a wall edge).
  2. Terminal 1: ros2 launch bumperbot_hardware hardware.launch.py
  3. Terminal 2: ros2 run bumperbot_hardware spin_test
        (options:  --ros-args -p turns:=5.0 -p angular_speed:=0.8)
  4. It spins, then stops itself after `turns` odometry rotations.
  5. Read how far the arrow is from the reference, and whether it stopped a bit
     SHORT of it (under-rotated) or a bit PAST it (over-rotated).
  6. Tell me: turns commanded, and the leftover offset (deg) + short/past.
     I'll compute and set WHEEL_BASE.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from bumperbot_hardware.parameters import CMD_VEL_TOPIC, ODOM_TOPIC, WHEEL_BASE


def yaw_from_quaternion(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SpinTest(Node):

    def __init__(self):
        super().__init__("spin_test")

        self.declare_parameter("turns", 5.0)            # full rotations to command
        self.declare_parameter("angular_speed", 0.8)    # rad/s spin rate
        self.turns = float(self.get_parameter("turns").value)
        self.wz = float(self.get_parameter("angular_speed").value)
        self.target = self.turns * 2.0 * math.pi        # radians (unwrapped)

        self.cum = 0.0          # accumulated UNWRAPPED heading (rad)
        self.last_yaw = None
        self.done = False

        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  WHEEL_BASE SPIN TEST")
        self.get_logger().info(f"  Commanding {self.turns:.0f} full turns "
                               f"({math.degrees(self.target):.0f} deg) at {self.wz:.2f} rad/s")
        self.get_logger().info(f"  Current WHEEL_BASE = {WHEEL_BASE:.4f} m")
        self.get_logger().info("  Align the tape arrow to a reference first.")
        self.get_logger().info("=" * 60)

    def odom_cb(self, msg):
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        yaw = yaw_from_quaternion(q)
        if self.last_yaw is None:
            self.last_yaw = yaw
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
            return  # wait for first odom

        if self.done or abs(self.cum) >= self.target:
            self.stop()
            if not self.done:
                self.done = True
                self.report()
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.wz
        self.pub.publish(twist)

    def report(self):
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  SPIN COMPLETE")
        self.get_logger().info(f"  Odometry turned : {math.degrees(self.cum):.1f} deg "
                               f"(commanded {self.turns:.0f} turns)")
        self.get_logger().info("  Now LOOK at the robot's arrow vs the reference:")
        self.get_logger().info("   - stopped a bit SHORT of it -> WHEEL_BASE too small")
        self.get_logger().info("   - stopped a bit PAST it      -> WHEEL_BASE too big")
        self.get_logger().info("  Tell me the leftover offset (deg) and short/past.")
        self.get_logger().info("=" * 60)

    def stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = SpinTest()
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
