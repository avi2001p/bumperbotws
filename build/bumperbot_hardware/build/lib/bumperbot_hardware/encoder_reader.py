#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

import RPi.GPIO as GPIO

from bumperbot_hardware.parameters import *


class EncoderReader(Node):

    def __init__(self):

        super().__init__("encoder_reader")

        self.get_logger().info("Encoder Reader Started")

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(LEFT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Encoder tick counters
        self.left_ticks = 0
        self.right_ticks = 0

        # Previous tick counters
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Wheel speeds (ticks/second)
        self.left_speed = 0.0
        self.right_speed = 0.0

        # Publishers
        self.tick_pub = self.create_publisher(
            Int32MultiArray,
            "/wheel_ticks",
            10
        )

        self.speed_pub = self.create_publisher(
            Float32MultiArray,
            "/wheel_speed",
            10
        )

        # Interrupts
        GPIO.add_event_detect(
            LEFT_ENCODER_A,
            GPIO.BOTH,
            callback=self.left_callback,
            bouncetime=1
        )

        GPIO.add_event_detect(
            RIGHT_ENCODER_A,
            GPIO.BOTH,
            callback=self.right_callback,
            bouncetime=1
        )

        # Publish at 10 Hz
        self.dt = 0.1

        self.timer = self.create_timer(
            self.dt,
            self.publish_data
        )

    def left_callback(self, channel):
        self.left_ticks += 1

    def right_callback(self, channel):
        self.right_ticks += 1

    def publish_data(self):

        # Tick message
        tick_msg = Int32MultiArray()
        tick_msg.data = [
            self.left_ticks,
            self.right_ticks
        ]

        self.tick_pub.publish(tick_msg)

        # Speed calculation
        delta_left = self.left_ticks - self.prev_left_ticks
        delta_right = self.right_ticks - self.prev_right_ticks

        self.left_speed = delta_left / self.dt
        self.right_speed = delta_right / self.dt

        speed_msg = Float32MultiArray()
        speed_msg.data = [
            self.left_speed,
            self.right_speed
        ]

        self.speed_pub.publish(speed_msg)

        # Store current counts
        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

    def destroy_node(self):

        GPIO.cleanup()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = EncoderReader()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Stopping Encoder Reader...")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()