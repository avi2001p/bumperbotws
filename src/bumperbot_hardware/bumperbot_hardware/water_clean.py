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
        # Per-sensor enable — both sensors active. Set one to false to ignore a
        # faulty module (EITHER enabled sensor being wet turns the vacuum+fan on).
        self.declare_parameter("use_sensor1", True)    # GPIO12 (pin 32)
        self.declare_parameter("use_sensor2", True)    # GPIO16 (pin 36)
        # On detection: run vacuum+fan for fan_duration, then ignore new
        # detections for cooldown seconds before re-arming.
        self.declare_parameter("fan_duration", 5.0)
        self.declare_parameter("cooldown", 5.0)
        active_high = self.get_parameter("relay_active_high").value
        self.use_sensor1 = self.get_parameter("use_sensor1").value
        self.use_sensor2 = self.get_parameter("use_sensor2").value
        self.fan_duration = self.get_parameter("fan_duration").value
        self.cooldown = self.get_parameter("cooldown").value
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

        self.state = "MONITORING"   # MONITORING -> CLEANING -> COOLDOWN -> MONITORING
        self.t_mark = 0.0
        self.timer = self.create_timer(0.2, self.loop)   # 5 Hz

        self.get_logger().info(
            f"Water clean started: EITHER enabled sensor WET -> vacuum+fan ON. "
            f"vacuum=GPIO{VACUUM_PUMP_PIN} fan=GPIO{DC_FAN_PIN} "
            f"sensor1(GPIO{WATER_SENSOR_PIN_1})={'ON' if self.use_sensor1 else 'OFF'} "
            f"sensor2(GPIO{WATER_SENSOR_PIN_2})={'ON' if self.use_sensor2 else 'OFF'}"
        )

    def sensor_wet(self, pin):
        level = GPIO.input(pin)
        return (level == GPIO.HIGH) if WATER_SENSOR_ACTIVE_HIGH else (level == GPIO.LOW)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def water_detected(self):
        wet1 = self.use_sensor1 and self.sensor_wet(WATER_SENSOR_PIN_1)
        wet2 = self.use_sensor2 and self.sensor_wet(WATER_SENSOR_PIN_2)
        return wet1 or wet2

    def loop(self):
        now = self.now_sec()

        if self.state == "MONITORING":
            if self.water_detected():
                # Detected -> run vacuum + fan for fan_duration, pause coverage
                GPIO.output(VACUUM_PUMP_PIN, self.on)
                GPIO.output(DC_FAN_PIN, self.on)
                self.pub.publish(Bool(data=True))
                self.state = "CLEANING"
                self.t_mark = now
                self.get_logger().info(
                    f"WATER DETECTED -> VACUUM + FAN ON for {self.fan_duration:.0f}s, "
                    f"coverage PAUSED"
                )

        elif self.state == "CLEANING":
            if now - self.t_mark >= self.fan_duration:
                GPIO.output(VACUUM_PUMP_PIN, self.off)
                GPIO.output(DC_FAN_PIN, self.off)
                self.pub.publish(Bool(data=False))
                self.state = "COOLDOWN"
                self.t_mark = now
                self.get_logger().info(
                    f"Cleaning done -> OFF, coverage RESUMED "
                    f"(cooldown {self.cooldown:.0f}s before next detection)"
                )

        elif self.state == "COOLDOWN":
            if now - self.t_mark >= self.cooldown:
                self.state = "MONITORING"
                self.get_logger().info("Ready — monitoring for water again.")

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
