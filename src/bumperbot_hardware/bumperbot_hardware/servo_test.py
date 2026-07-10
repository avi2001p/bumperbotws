#!/usr/bin/env python3
"""
servo_test.py
-------------
SG90 roller servo test / calibration.

Moves SMOOTHLY (gradual steps) and then RELEASES the PWM signal so the servo
holds still WITHOUT the buzzing/vibration you get from continuous software PWM.

Modes:
  # roller motion: smooth UP->DOWN, hold, smooth DOWN->UP
  ros2 run bumperbot_hardware servo_test --ros-args \
      -p cycle:=true -p up_angle:=90 -p down_angle:=80 -p hold:=5.0

  # hold a single angle (to find UP / DOWN positions)
  ros2 run bumperbot_hardware servo_test --ros-args -p angle:=90

  # sweep the whole range
  ros2 run bumperbot_hardware servo_test --ros-args -p sweep:=true

Tuning:
  up_angle / down_angle : the two roller positions (smaller gap = less rotation)
  smooth_time           : seconds for each up/down move (bigger = slower/smoother)
  repeat:=true          : loop the cycle
"""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import RPi.GPIO as GPIO

from bumperbot_hardware.parameters import (
    SERVO_PIN,
    SERVO_MIN_PULSE_MS,
    SERVO_MAX_PULSE_MS,
)


class ServoTest(Node):

    def __init__(self):
        super().__init__("servo_test")

        num = ParameterDescriptor(dynamic_typing=True)   # accept int or float
        self.declare_parameter("angle", 90.0, num)
        self.declare_parameter("sweep", False)
        self.declare_parameter("cycle", False)
        self.declare_parameter("up_angle", 90.0, num)      # roller OFF the surface
        # rotation = |down_angle - up_angle| deg. Roller must reach the surface,
        # so this is a big travel; find the real value with single-angle mode.
        self.declare_parameter("down_angle", 120.0, num)   # roller ON the surface
        self.declare_parameter("hold", 5.0, num)          # seconds down
        self.declare_parameter("smooth_time", 0.8, num)   # seconds per move
        self.declare_parameter("repeat", False)
        self.declare_parameter("min_pulse_ms", SERVO_MIN_PULSE_MS, num)
        self.declare_parameter("max_pulse_ms", SERVO_MAX_PULSE_MS, num)

        self.min_ms = float(self.get_parameter("min_pulse_ms").value)
        self.max_ms = float(self.get_parameter("max_pulse_ms").value)
        self.smooth_time = float(self.get_parameter("smooth_time").value)
        angle = float(self.get_parameter("angle").value)
        sweep = self.get_parameter("sweep").value
        cycle = self.get_parameter("cycle").value
        up_angle = float(self.get_parameter("up_angle").value)
        down_angle = float(self.get_parameter("down_angle").value)
        hold = float(self.get_parameter("hold").value)
        repeat = self.get_parameter("repeat").value

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(SERVO_PIN, 50)   # 50 Hz
        self.pwm.start(0)

        self.get_logger().info(
            f"Servo on GPIO{SERVO_PIN}. Smooth move + signal-release (no buzz)."
        )

        if cycle:
            self.move_smooth(up_angle, up_angle)          # settle at UP, release
            self.get_logger().info(f"UP = {up_angle:.0f} deg")
            time.sleep(0.5)
            while True:
                self.move_smooth(up_angle, down_angle)    # smooth DOWN
                self.get_logger().info(f"DOWN = {down_angle:.0f} deg — hold {hold:.0f}s (silent)")
                time.sleep(hold)
                self.move_smooth(down_angle, up_angle)    # smooth UP
                self.get_logger().info(f"UP = {up_angle:.0f} deg")
                if not repeat:
                    break
                time.sleep(1.0)
            self.get_logger().info("Cycle done — released at UP. Ctrl+C to exit.")
        elif sweep:
            self.move_smooth(90.0, 0.0, dur=1.2)
            self.move_smooth(0.0, 180.0, dur=2.0)
            self.move_smooth(180.0, 90.0, dur=1.2)
            self.get_logger().info("Sweep done — released at 90. Ctrl+C to exit.")
        else:
            self.move_smooth(90.0, angle)
            self.get_logger().info(
                f"At {angle:.0f} deg (signal released — no buzz). Ctrl+C to exit."
            )

    def _apply(self, a):
        a = max(0.0, min(180.0, a))
        pulse_ms = self.min_ms + (a / 180.0) * (self.max_ms - self.min_ms)
        self.pwm.ChangeDutyCycle(pulse_ms / 20.0 * 100.0)   # 50 Hz -> 20 ms period

    def _release(self):
        self.pwm.ChangeDutyCycle(0)     # stop signal -> servo holds, no buzz

    def move_smooth(self, from_a, to_a, dur=None, steps=25):
        """Ease from `from_a` to `to_a` and HOLD there so the roller stays put
        (does not droop). Continuous PWM may buzz slightly on an SG90."""
        if dur is None:
            dur = self.smooth_time
        for i in range(steps + 1):
            a = from_a + (to_a - from_a) * (i / steps)
            self._apply(a)
            time.sleep(dur / steps)
        # keep holding at the target (no release) so the position is maintained

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
