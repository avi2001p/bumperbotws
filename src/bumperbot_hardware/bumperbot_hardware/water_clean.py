#!/usr/bin/env python3
"""
water_clean.py
--------------
Simple, direct water cleaning:
  * If EITHER of the two water sensors detects water -> vacuum pump + fan ON.
  * When BOTH sensors are dry            -> vacuum pump + fan OFF.

It also publishes /water_cleaning_active so the coverage node PAUSES while
cleaning and RESUMES when dry. Use this instead of water_actuator for a
keep-cleaning-until-dry behaviour.

Run:
  ros2 run bumperbot_hardware water_clean
  ros2 run bumperbot_hardware water_clean --ros-args -p relay_active_high:=true
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

from bumperbot_hardware.parameters import (
    VACUUM_PUMP_PIN,
    DC_FAN_PIN,
    WATER_SENSOR_PIN_1,
    WATER_SENSOR_PIN_2,
    WATER_SENSOR_ACTIVE_HIGH,
    WATER_CLEANING_TOPIC,
)


class WaterClean(Node):

    def __init__(self):
        super().__init__("water_clean")

        # Relay polarity — most 5V boards are active-LOW (default). Pins are
        # driven OFF at startup regardless, so nothing runs unexpectedly.
        self.declare_parameter("relay_active_high", False)
        active_high = self.get_parameter("relay_active_high").value
        self.on = GPIO.HIGH if active_high else GPIO.LOW
        self.off = GPIO.LOW if active_high else GPIO.HIGH

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(VACUUM_PUMP_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(DC_FAN_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(WATER_SENSOR_PIN_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(WATER_SENSOR_PIN_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # Tells the coverage node to pause/resume while cleaning
        self.pub = self.create_publisher(Bool, WATER_CLEANING_TOPIC, 10)

        self.cleaning = False
        self.timer = self.create_timer(0.2, self.loop)   # 5 Hz

        self.get_logger().info(
            f"Water clean started: EITHER sensor WET -> vacuum+fan ON. "
            f"vacuum=GPIO{VACUUM_PUMP_PIN} fan=GPIO{DC_FAN_PIN} "
            f"sensors=GPIO{WATER_SENSOR_PIN_1},GPIO{WATER_SENSOR_PIN_2}"
        )

    def sensor_wet(self, pin):
        level = GPIO.input(pin)
        return (level == GPIO.HIGH) if WATER_SENSOR_ACTIVE_HIGH else (level == GPIO.LOW)

    def loop(self):
        wet1 = self.sensor_wet(WATER_SENSOR_PIN_1)
        wet2 = self.sensor_wet(WATER_SENSOR_PIN_2)
        water = wet1 or wet2

        if water and not self.cleaning:
            # Turn the vacuum + fan ON and pause coverage
            GPIO.output(VACUUM_PUMP_PIN, self.on)
            GPIO.output(DC_FAN_PIN, self.on)
            self.cleaning = True
            self.pub.publish(Bool(data=True))
            self.get_logger().info(
                f"WATER DETECTED (s1={wet1} s2={wet2}) -> VACUUM + FAN ON, coverage PAUSED"
            )
        elif not water and self.cleaning:
            # Both dry again -> stop and resume coverage
            GPIO.output(VACUUM_PUMP_PIN, self.off)
            GPIO.output(DC_FAN_PIN, self.off)
            self.cleaning = False
            self.pub.publish(Bool(data=False))
            self.get_logger().info("DRY -> VACUUM + FAN OFF, coverage RESUMED")

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
    node = WaterClean()
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
