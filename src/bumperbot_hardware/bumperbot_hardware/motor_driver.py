#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
from bumperbot_hardware.parameters import *

# Dead-zone threshold as duty-cycle percentage
_DEADZONE_PCT = (MIN_PWM_DEADZONE / 255.0) * 100.0


class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')
        self.get_logger().info("Closed-Loop Motor Driver Started")

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        motor_pins = [
            RIGHT_EN, RIGHT_IN1, RIGHT_IN2,
            LEFT_EN,  LEFT_IN1,  LEFT_IN2
        ]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)

        self.right_pwm = GPIO.PWM(RIGHT_EN, PWM_FREQUENCY)
        self.left_pwm  = GPIO.PWM(LEFT_EN,  PWM_FREQUENCY)
        self.right_pwm.start(0)
        self.left_pwm.start(0)

        # Swapped subscription to listen directly to the PID output channel
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_pwm',
            self.motor_pwm_callback,
            10
        )

        # Safety timeout tracker
        self.timeout = 0.5
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)

    def motor_pwm_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        # Extract target control signals from the incoming PID array
        raw_left  = msg.data[0]
        raw_right = msg.data[1]

        # Log if we are trying to drive (throttle to avoid spam, or just log when it changes)
        if raw_left != 0.0 or raw_right != 0.0:
            # We'll log once a second just to be sure
            pass # Actually let's just log it directly to be absolutely sure it's reaching here
            self.get_logger().info(f"Motor Driver received PWM -> L: {raw_left}, R: {raw_right}")

        # Map the 8-bit scale (-255 to 255) to RPi.GPIO Duty Cycle percentages (0 to 100)
        left_pwm_pct  = min((abs(raw_left) / 255.0) * 100.0, 100.0)
        right_pwm_pct = min((abs(raw_right) / 255.0) * 100.0, 100.0)

        # Dead-zone compensation: boost small non-zero PWM to minimum
        # so both motors overcome static friction simultaneously
        if raw_left != 0.0 and left_pwm_pct < _DEADZONE_PCT:
            left_pwm_pct = _DEADZONE_PCT
        if raw_right != 0.0 and right_pwm_pct < _DEADZONE_PCT:
            right_pwm_pct = _DEADZONE_PCT

        # Right Motor H-Bridge Direction State Configuration
        if raw_right > 0:
            GPIO.output(RIGHT_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_IN2, GPIO.LOW)
        elif raw_right < 0:
            GPIO.output(RIGHT_IN1, GPIO.LOW)
            GPIO.output(RIGHT_IN2, GPIO.HIGH)
        else:
            GPIO.output(RIGHT_IN1, GPIO.LOW)
            GPIO.output(RIGHT_IN2, GPIO.LOW)

        # Left Motor H-Bridge Direction State Configuration
        if raw_left > 0:
            GPIO.output(LEFT_IN1, GPIO.LOW)
            GPIO.output(LEFT_IN2, GPIO.HIGH)
        elif raw_left < 0:
            GPIO.output(LEFT_IN1, GPIO.HIGH)
            GPIO.output(LEFT_IN2, GPIO.LOW)
        else:
            GPIO.output(LEFT_IN1, GPIO.LOW)
            GPIO.output(LEFT_IN2, GPIO.LOW)

        # Update physical hardware pin states
        self.right_pwm.ChangeDutyCycle(right_pwm_pct)
        self.left_pwm.ChangeDutyCycle(left_pwm_pct)

    def stop_motors(self):
        self.right_pwm.ChangeDutyCycle(0)
        self.left_pwm.ChangeDutyCycle(0)
        GPIO.output(RIGHT_IN1, GPIO.LOW)
        GPIO.output(RIGHT_IN2, GPIO.LOW)
        GPIO.output(LEFT_IN1,  GPIO.LOW)
        GPIO.output(LEFT_IN2,  GPIO.LOW)

    def check_timeout(self):
        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.stop_motors()

    def destroy_node(self):
        self.stop_motors()
        self.right_pwm.stop()
        self.left_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Motor Driver...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()