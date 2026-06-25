#!/usr/bin/env python3
"""
encoder_reader.py
-----------------
Reads quadrature encoder ticks from the 25GA-370 motor's built-in
Hall-sensor encoder via Raspberry Pi GPIO interrupts.

Publishes:
  /wheel_ticks  (Int32MultiArray)  — cumulative signed tick counts
  /wheel_speed  (Float32MultiArray) — wheel speeds in ticks/second

Direction detection uses the phase relationship between Channel A
and Channel B of each encoder.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

import RPi.GPIO as GPIO

from bumperbot_hardware.parameters import *


class EncoderReader(Node):

    def __init__(self):

        super().__init__("encoder_reader")

        self.get_logger().info("Encoder Reader Started (with direction detection)")

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Setup encoder pins — Channel A as interrupt, Channel B for direction
        GPIO.setup(LEFT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LEFT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RIGHT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Signed encoder tick counters (can go negative for reverse)
        self.left_ticks = 0
        self.right_ticks = 0

        # Previous tick counters for speed calculation
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Wheel speeds (ticks/second)
        self.left_speed = 0.0
        self.right_speed = 0.0

        # Publishers
        self.tick_pub = self.create_publisher(
            Int32MultiArray,
            WHEEL_TICKS_TOPIC,
            10
        )

        self.speed_pub = self.create_publisher(
            Float32MultiArray,
            WHEEL_SPEED_TOPIC,
            10
        )

        # Interrupts on Channel A (BOTH edges) — read Channel B for direction
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

        # Publish at CONTROL_RATE (20 Hz) — must match the PID loop period
        self.dt = 1.0 / CONTROL_RATE

        self.timer = self.create_timer(
            self.dt,
            self.publish_data
        )

    def left_callback(self, channel):
        """Left encoder interrupt — direction from the A/B phase relationship."""
        a_state = GPIO.input(LEFT_ENCODER_A)
        b_state = GPIO.input(LEFT_ENCODER_B)
        # Base quadrature decode: A==B → one direction, A!=B → the other.
        # LEFT_ENCODER_SIGN (parameters.py) flips it so FORWARD = positive.
        step = 1 if a_state == b_state else -1
        self.left_ticks += LEFT_ENCODER_SIGN * step

    def right_callback(self, channel):
        """Right encoder interrupt — direction from the A/B phase relationship."""
        a_state = GPIO.input(RIGHT_ENCODER_A)
        b_state = GPIO.input(RIGHT_ENCODER_B)
        # Same base decode as the left; RIGHT_ENCODER_SIGN handles the
        # mirrored mounting so that FORWARD = positive on both wheels.
        step = 1 if a_state == b_state else -1
        self.right_ticks += RIGHT_ENCODER_SIGN * step

    def publish_data(self):

        # Tick message (signed)
        tick_msg = Int32MultiArray()
        tick_msg.data = [
            self.left_ticks,
            self.right_ticks
        ]

        self.tick_pub.publish(tick_msg)

        # Speed calculation (ticks per second, signed)
        delta_left = self.left_ticks - self.prev_left_ticks
        delta_right = self.right_ticks - self.prev_right_ticks

        self.left_speed = delta_left / self.dt
        self.right_speed = delta_right / self.dt

        speed_msg = Float32MultiArray()
        speed_msg.data = [
            float(self.left_speed),
            float(self.right_speed)
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