#!/usr/bin/env python3
"""
servo_test.py
-------------
SG90 roller servo test / calibration — driven by **pigpio** (hardware-timed PWM).

Why pigpio: RPi.GPIO software PWM makes servos buzz/vibrate, drift, and move
inconsistently. pigpio is smooth and silent, and HOLDS the commanded position
firmly, so the roller returns to the exact initial point and stays there.

>>> Start the pigpio daemon ONCE before running (survives until reboot):
        sudo pigpiod
    If it's missing:   sudo apt install pigpio python3-pigpio

Modes:
  # roller cycle: UP -> DOWN (on surface) -> hold -> back to UP (initial)
  ros2 run bumperbot_hardware servo_test --ros-args \
      -p cycle:=true -p up_angle:=90 -p down_angle:=115 -p hold:=5.0

  # hold a single angle (to find UP / DOWN)
  ros2 run bumperbot_hardware servo_test --ros-args -p angle:=90

Tuning:
  down_angle > up_angle  -> one direction;  down_angle < up_angle -> the other.
  rotation = |down_angle - up_angle| degrees.
  smooth_time = seconds per move.
"""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import pigpio

from bumperbot_hardware.parameters import (
    SERVO_PIN,
    SERVO_MIN_PULSE_MS,
    SERVO_MAX_PULSE_MS,
)


class ServoTest(Node):

    def __init__(self):
        super().__init__("servo_test")

        num = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("angle", 90.0, num)
        self.declare_parameter("cycle", False)
        self.declare_parameter("up_angle", 90.0, num)      # roller UP (off surface)
        self.declare_parameter("down_angle", 115.0, num)   # roller DOWN (on surface)
        self.declare_parameter("hold", 5.0, num)           # seconds down
        self.declare_parameter("smooth_time", 0.8, num)    # seconds per move
        self.declare_parameter("repeat", False)
        self.declare_parameter("min_us", int(SERVO_MIN_PULSE_MS * 1000))   # 0.5 ms -> 500 us
        self.declare_parameter("max_us", int(SERVO_MAX_PULSE_MS * 1000))   # 2.5 ms -> 2500 us

        self.min_us = int(self.get_parameter("min_us").value)
        self.max_us = int(self.get_parameter("max_us").value)
        self.smooth_time = float(self.get_parameter("smooth_time").value)
        angle = float(self.get_parameter("angle").value)
        cycle = self.get_parameter("cycle").value
        up_angle = float(self.get_parameter("up_angle").value)
        down_angle = float(self.get_parameter("down_angle").value)
        hold = float(self.get_parameter("hold").value)
        repeat = self.get_parameter("repeat").value

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error(
                "pigpio daemon is NOT running. Start it with:  sudo pigpiod   "
                "(install once:  sudo apt install pigpio python3-pigpio)"
            )
            raise RuntimeError("pigpiod not running")

        self.get_logger().info(
            f"Servo on GPIO{SERVO_PIN} via pigpio — smooth, silent, holds position."
        )

        if cycle:
            self._apply(up_angle)
            self.get_logger().info(f"UP = {up_angle:.0f} deg (held)")
            time.sleep(0.6)
            while True:
                self.move_smooth(up_angle, down_angle)
                self.get_logger().info(f"DOWN = {down_angle:.0f} deg — hold {hold:.0f}s")
                time.sleep(hold)
                self.move_smooth(down_angle, up_angle)
                self.get_logger().info(f"UP = {up_angle:.0f} deg (back to initial)")
                if not repeat:
                    break
                time.sleep(1.0)
            self.get_logger().info("Cycle done — parked at UP, held silently. Ctrl+C to exit.")
        else:
            self._apply(angle)
            self.get_logger().info(f"At {angle:.0f} deg — held silently. Ctrl+C to exit.")

    def _us(self, a):
        a = max(0.0, min(180.0, a))
        return int(self.min_us + (a / 180.0) * (self.max_us - self.min_us))

    def _apply(self, a):
        self.pi.set_servo_pulsewidth(SERVO_PIN, self._us(a))

    def _release(self):
        self.pi.set_servo_pulsewidth(SERVO_PIN, 0)   # stop pulses (servo goes limp)

    def move_smooth(self, from_a, to_a, steps=40):
        """Ease from `from_a` to `to_a`. pigpio then keeps holding the target —
        silent and drift-free, so it stays exactly where commanded."""
        dur = self.smooth_time
        for i in range(steps + 1):
            a = from_a + (to_a - from_a) * (i / steps)
            self._apply(a)
            time.sleep(dur / steps)

    def destroy_node(self):
        try:
            self._release()
            self.pi.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServoTest()
    except RuntimeError:
        rclpy.shutdown()
        return
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
