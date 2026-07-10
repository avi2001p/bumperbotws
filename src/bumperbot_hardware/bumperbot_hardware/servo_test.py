#!/usr/bin/env python3
"""
servo_test.py
-------------
SG90 servo calibration for the roller lift. Moves the servo to a chosen angle and
HOLDS it, so you can find the angle for roller UP and roller DOWN.

Usage:
  # hold a single angle (try several to find UP and DOWN)
  ros2 run bumperbot_hardware servo_test --ros-args -p angle:=0
  ros2 run bumperbot_hardware servo_test --ros-args -p angle:=90

  # slowly sweep 0..180..0 to see the full travel
  ros2 run bumperbot_hardware servo_test --ros-args -p sweep:=true

Once you know the two angles, put them in parameters.py:
  ROLLER_UP_ANGLE   = <angle where the roller is lifted>
  ROLLER_DOWN_ANGLE = <angle where the roller touches the ground>

If the servo can't reach an end, widen SERVO_MIN_PULSE_MS / SERVO_MAX_PULSE_MS.
"""

import time

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO

from bumperbot_hardware.parameters import (
    SERVO_PIN,
    SERVO_MIN_PULSE_MS,
    SERVO_MAX_PULSE_MS,
)


class ServoTest(Node):

    def __init__(self):
        super().__init__("servo_test")

        self.declare_parameter("angle", 90.0)      # 0..180 deg
        self.declare_parameter("sweep", False)     # sweep the full range
        self.declare_parameter("min_pulse_ms", SERVO_MIN_PULSE_MS)
        self.declare_parameter("max_pulse_ms", SERVO_MAX_PULSE_MS)

        self.min_ms = self.get_parameter("min_pulse_ms").value
        self.max_ms = self.get_parameter("max_pulse_ms").value
        angle = self.get_parameter("angle").value
        sweep = self.get_parameter("sweep").value

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, 50)   # 50 Hz for hobby servos
        self.pwm.start(0)

        self.get_logger().info(
            f"Servo on GPIO{SERVO_PIN}. Pulse {self.min_ms}-{self.max_ms} ms for 0-180 deg."
        )

        if sweep:
            angles = list(range(0, 181, 10)) + list(range(180, -1, -10))
            for a in angles:
                self.set_angle(a)
                self.get_logger().info(f"angle = {a} deg")
                time.sleep(0.4)
            self.get_logger().info("Sweep done — holding 90 deg. Ctrl+C to exit.")
            self.set_angle(90.0)
        else:
            self.set_angle(angle)
            self.get_logger().info(
                f"Holding angle = {angle} deg. Note this for UP or DOWN. Ctrl+C to exit."
            )

    def set_angle(self, angle):
        """Move to `angle` (0..180) and keep holding (PWM stays on)."""
        angle = max(0.0, min(180.0, angle))
        pulse_ms = self.min_ms + (angle / 180.0) * (self.max_ms - self.min_ms)
        duty = pulse_ms / 20.0 * 100.0       # 50 Hz => 20 ms period
        self.pwm.ChangeDutyCycle(duty)

    def destroy_node(self):
        try:
            self.pwm.ChangeDutyCycle(0)
            self.pwm.stop()
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
