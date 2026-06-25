import rclpy
from rclpy.node import Node

from smbus2 import SMBus
import time

TCA_ADDR = 0x70
AS5600_ADDR = 0x36


class EncoderOdomNode(Node):

    def __init__(self):
        super().__init__('encoder_odom_node')

        self.bus = SMBus(1)

        self.timer = self.create_timer(
            0.1,  # 10 Hz
            self.timer_callback
        )

        self.get_logger().info(
            "Encoder odometry node started"
        )

    def read_encoder(self, channel):

        self.bus.write_byte(TCA_ADDR, 1 << channel)

        time.sleep(0.01)

        high = self.bus.read_byte_data(
            AS5600_ADDR,
            0x0E
        )

        low = self.bus.read_byte_data(
            AS5600_ADDR,
            0x0F
        )

        raw = (high << 8) | low

        angle = raw * 360.0 / 4096.0

        return angle

    def timer_callback(self):

        try:

            right_angle = self.read_encoder(0)
            left_angle = self.read_encoder(1)

            self.get_logger().info(
                f"Right: {right_angle:.2f}°   "
                f"Left: {left_angle:.2f}°"
            )

        except Exception as e:

            self.get_logger().error(
                f"Encoder error: {str(e)}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = EncoderOdomNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()