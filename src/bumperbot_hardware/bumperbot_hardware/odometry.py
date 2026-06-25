#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry as OdometryMsg
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from bumperbot_hardware.parameters import *


class Odometry(Node):

    def __init__(self):
        super().__init__("odometry")
        self.get_logger().info("Odometry Node Started")

        # Current encoder ticks
        self.left_ticks  = 0
        self.right_ticks = 0

        # Previous encoder ticks
        self.prev_left_ticks  = 0
        self.prev_right_ticks = 0

        # Robot pose
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # /odom publisher
        self.odom_pub = self.create_publisher(OdometryMsg, '/odom', 10)

        # TF broadcaster (odom → base_link)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.wheel_ticks_callback,
            10
        )

        # Update odometry at 10 Hz
        self.timer = self.create_timer(0.1, self.update_odometry)

    def wheel_ticks_callback(self, msg):
        self.left_ticks  = msg.data[0]
        self.right_ticks = msg.data[1]

    def update_odometry(self):
        # Tick difference
        delta_left_ticks  = self.left_ticks  - self.prev_left_ticks
        delta_right_ticks = self.right_ticks - self.prev_right_ticks
        self.prev_left_ticks  = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # Convert ticks to wheel travel distance
        left_distance  = (delta_left_ticks  / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE
        right_distance = (delta_right_ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE

        # Robot movement
        distance    = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / WHEEL_BASE

        # Update pose using the MIDPOINT heading over this step (more accurate
        # on arcs than integrating x/y with the end-of-step heading).
        mid_theta = self.theta + delta_theta / 2.0
        self.x     += distance * math.cos(mid_theta)
        self.y     += distance * math.sin(mid_theta)
        self.theta += delta_theta

        # Throttled so it doesn't flood the console at 10 Hz
        self.get_logger().info(
            f"X={self.x:.3f}  Y={self.y:.3f}  Theta={math.degrees(self.theta):.2f} deg",
            throttle_duration_sec=1.0,
        )

        now = self.get_clock().now().to_msg()

        # ── Publish /odom ────────────────────────────────────
        odom = OdometryMsg()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        # ── Broadcast odom → base_link TF ───────────────────
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Odometry Node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()