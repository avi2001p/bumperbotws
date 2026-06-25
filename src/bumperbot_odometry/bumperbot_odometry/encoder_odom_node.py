import rclpy
from rclpy.node import Node
from smbus2 import SMBus
import time
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

TCA_ADDR = 0x70
AS5600_ADDR = 0x36
WHEEL_DIAMETER = 0.066
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
WHEEL_BASE = 0.180

class EncoderOdomNode(Node):

    def __init__(self):
        super().__init__('encoder_odom_node')
        self.bus = SMBus(1)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        try:
            self.right_prev = self.read_encoder(0)
            self.left_prev = self.read_encoder(1)
        except Exception as e:
            self.get_logger().warn(f"Encoder init failed: {e}")
            self.right_prev = 0.0
            self.left_prev = 0.0
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Odometry publisher started")

    def read_encoder(self, channel):
        self.bus.write_byte(TCA_ADDR, 1 << channel)
        time.sleep(0.01)
        high = self.bus.read_byte_data(AS5600_ADDR, 0x0E)
        low = self.bus.read_byte_data(AS5600_ADDR, 0x0F)
        raw = (high << 8) | low
        return raw * 360.0 / 4096.0

    def angle_diff(self, current, previous):
        diff = current - previous
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff

    def timer_callback(self):
        try:
            right_now = self.read_encoder(0)
            left_now = self.read_encoder(1)
            right_delta = self.angle_diff(right_now, self.right_prev)
            left_delta = -self.angle_diff(left_now, self.left_prev)
            right_distance = (right_delta / 360.0) * WHEEL_CIRCUMFERENCE
            left_distance = (left_delta / 360.0) * WHEEL_CIRCUMFERENCE
            distance = (right_distance + left_distance) / 2.0
            dtheta = (right_distance - left_distance) / WHEEL_BASE
            self.theta += dtheta
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
            self.right_prev = right_now
            self.left_prev = left_now
        except Exception as e:
            self.get_logger().warn(f"Encoder read failed: {e} - using last pose")

        q = quaternion_from_euler(0.0, 0.0, self.theta)
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()