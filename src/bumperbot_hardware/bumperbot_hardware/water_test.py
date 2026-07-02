#!/usr/bin/env python3
"""
water_test.py
-------------
Bench test for the water-extraction hardware. Cycles the VACUUM and FAN relays
once, then continuously reads BOTH water sensors — so you can verify the wiring
and find the correct relay polarity BEFORE running the full mission.

How to read the result:
  - When it says "VACUUM ON", the pump should RUN. If it runs on "OFF" instead,
    your relay board is the opposite polarity -> re-run with
    `-p relay_active_high:=true` (or false) until ON means ON.
  - Dip each water sensor in water; its SENSOR1 / SENSOR2 line should flip
    WET/DRY. If it reads inverted, flip WATER_SENSOR_ACTIVE_HIGH in parameters.py.

Run (lidar/motors NOT needed):
  ros2 run bumperbot_hardware water_test
  ros2 run bumperbot_hardware water_test --ros-args -p relay_active_high:=true
"""

import time

import rclpy
from rclpy.node import Node

import RPi.GPIO as GPIO

from bumperbot_hardware.parameters import (
    VACUUM_PUMP_PIN,
    DC_FAN_PIN,
    WATER_SENSOR_PIN_1,
    WATER_SENSOR_PIN_2,
    WATER_SENSOR_ACTIVE_HIGH,
)


class WaterTest(Node):

    def __init__(self):
        super().__init__("water_test")

        self.declare_parameter("relay_active_high", False)
        self.declare_parameter("on_seconds", 3.0)
        active_high = self.get_parameter("relay_active_high").value
        self.on = GPIO.HIGH if active_high else GPIO.LOW
        self.off = GPIO.LOW if active_high else GPIO.HIGH
        self.on_seconds = self.get_parameter("on_seconds").value

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(VACUUM_PUMP_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(DC_FAN_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(WATER_SENSOR_PIN_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(WATER_SENSOR_PIN_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.get_logger().info(
            f"Water test: relay_active_high={active_high} "
            f"(ON = GPIO {'HIGH' if self.on == GPIO.HIGH else 'LOW'}). "
            f"vacuum=GPIO{VACUUM_PUMP_PIN} fan=GPIO{DC_FAN_PIN} "
            f"sensor1=GPIO{WATER_SENSOR_PIN_1} sensor2=GPIO{WATER_SENSOR_PIN_2}"
        )

        self.done = False
        self.create_timer(1.0, self.tick)

    def read_sensor(self, pin):
        level = GPIO.input(pin)
        wet = (level == GPIO.HIGH) == bool(WATER_SENSOR_ACTIVE_HIGH)
        return wet, level

    def tick(self):
        if not self.done:
            self.done = True
            self.get_logger().info(">>> VACUUM ON — the pump should RUN now")
            GPIO.output(VACUUM_PUMP_PIN, self.on)
            time.sleep(self.on_seconds)
            GPIO.output(VACUUM_PUMP_PIN, self.off)
            self.get_logger().info(">>> VACUUM OFF")
            time.sleep(1.0)
            self.get_logger().info(">>> FAN ON — the fan should SPIN now")
            GPIO.output(DC_FAN_PIN, self.on)
            time.sleep(self.on_seconds)
            GPIO.output(DC_FAN_PIN, self.off)
            self.get_logger().info(">>> FAN OFF")
            self.get_logger().info("Now reading BOTH water sensors — dip each in water.")
            return

        # Continuous water-sensor read (both sensors)
        wet1, raw1 = self.read_sensor(WATER_SENSOR_PIN_1)
        wet2, raw2 = self.read_sensor(WATER_SENSOR_PIN_2)
        self.get_logger().info(
            f"SENSOR1(GPIO{WATER_SENSOR_PIN_1})={'WET' if wet1 else 'DRY'}(raw={raw1})  |  "
            f"SENSOR2(GPIO{WATER_SENSOR_PIN_2})={'WET' if wet2 else 'DRY'}(raw={raw2})",
            throttle_duration_sec=0.5,
        )

    def destroy_node(self):
        try:
            GPIO.output(VACUUM_PUMP_PIN, self.off)
            GPIO.output(DC_FAN_PIN, self.off)
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaterTest()
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
