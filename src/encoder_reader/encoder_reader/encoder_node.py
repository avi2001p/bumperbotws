import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from smbus2 import SMBus
import math
import time

AS5600_ADDR = 0x36
ANGLE_REG = 0x0E

WHEEL_DIAMETER_MM = 63
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER_MM


class EncoderNode(Node):

    def __init__(self):

        super().__init__('encoder_node')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.timer = self.create_timer(0.05, self.update_odometry)

        self.bus = SMBus(1)

        self.previous_angle = 0
        self.total_angle = 0

        self.x_position = 0.0

        self.last_time = time.time()

        self.tf_broadcaster = TransformBroadcaster(self)

    def update_odometry(self):

        high = self.bus.read_byte_data(AS5600_ADDR, ANGLE_REG)
        low = self.bus.read_byte_data(AS5600_ADDR, ANGLE_REG + 1)

        raw_angle = ((high << 8) | low)

        angle = (raw_angle * 360.0) / 4096.0

        difference = angle - self.previous_angle

        if difference > 180:
            difference -= 360

        elif difference < -180:
            difference += 360

        self.total_angle += difference

        self.previous_angle = angle

        distance_mm = (self.total_angle / 360.0) * WHEEL_CIRCUMFERENCE

        self.x_position = distance_mm / 1000.0

        current_time = time.time()

        dt = current_time - self.last_time

        velocity = 0.0

        if dt > 0:
            velocity = self.x_position / dt

        self.last_time = current_time

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x_position
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, 0)

        odom.pose.pose.orientation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )

        odom.twist.twist.linear.x = velocity

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x_position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        self.odom_pub.publish(odom)

        self.get_logger().info(
            f'X Position: {self.x_position:.3f} m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = EncoderNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
