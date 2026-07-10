#!/usr/bin/env python3
"""
servo_test.py
-------------
SG90 roller servo test / calibration — driven by pigpio (smooth, SILENT, HOLDS).

pigpio holds the commanded position firmly and silently (no buzz, no droop), so
the roller returns to the exact initial point and stays there.

>>> Start the daemon once (survives until reboot):   sudo pigpiod

On this robot:  DOWN = low angle (0),  UP = higher angle.

Modes:
  # roller cycle: UP -> DOWN -> hold 5s -> back to UP (same distance)
  ros2 run bumperbot_hardware servo_test --ros-args \
      -p cycle:=true -p up_angle:=20 -p down_angle:=0 -p hold:=5.0

  # hold a single angle (to find UP / DOWN)
  ros2 run bumperbot_hardware servo_test --ros-args -p angle:=0
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
        self.declare_parameter("angle", 0.0, num)
        self.declare_parameter("cycle", False)
        self.declare_parameter("up_angle", 20.0, num)      # roller UP (initial)
        self.declare_parameter("down_angle", 0.0, num)     # roller DOWN
        self.declare_parameter("hold", 5.0, num)           # seconds down
        self.declare_parameter("smooth_time", 0.8, num)    # seconds per move
        self.declare_parameter("repeat", False)
        self.declare_parameter("min_us", int(SERVO_MIN_PULSE_MS * 1000))   # 500
        self.declare_parameter("max_us", int(SERVO_MAX_PULSE_MS * 1000))   # 2500

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
                "pigpio daemon is NOT running. Start it with:  sudo pigpiod"
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
                # Climb slightly PAST the target, then snap back onto it —
                # cancels the few degrees the servo settles short when
                # climbing slowly under the roller's weight.
                overshoot = 8.0 if up_angle > down_angle else -8.0
                self.move_smooth(down_angle, up_angle + overshoot)
                time.sleep(0.3)
                self._apply(up_angle)
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
        self.pi.set_servo_pulsewidth(SERVO_PIN, 0)

    def move_smooth(self, from_a, to_a, steps=40):
        """Ease from `from_a` to `to_a`; pigpio then holds the target silently."""
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
