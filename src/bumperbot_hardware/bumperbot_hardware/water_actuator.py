#!/usr/bin/env python3
"""
water_actuator.py
-----------------
Controls the vacuum pump and DC fan for water removal on the BumperBot.

Behavior:
  1. Monitors /water_detected topic (or reads a GPIO water sensor directly)
  2. When water is detected:
     - Publishes /water_cleaning_active = True (signals coverage node to pause)
     - Turns ON vacuum pump relay via GPIO
     - Turns ON DC fan relay via GPIO
     - Waits FAN_ON_DURATION seconds (default 5s)
     - Turns OFF fan and vacuum
     - Publishes /water_cleaning_active = False (signals coverage to resume)

GPIO Pins (configurable in parameters.py):
  - VACUUM_PUMP_PIN: relay controlling the vacuum pump
  - DC_FAN_PIN: relay controlling the drying fan
  - WATER_SENSOR_PIN: digital water sensor input (optional)

Subscribes:
  /water_detected  (std_msgs/Bool) — external water detection trigger

Publishes:
  /water_cleaning_active (std_msgs/Bool) — pause/resume signal for coverage
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import RPi.GPIO as GPIO

from bumperbot_hardware.parameters import (
    VACUUM_PUMP_PIN,
    DC_FAN_PIN,
    WATER_SENSOR_PIN,
    WATER_SENSOR_ACTIVE_HIGH,
    FAN_ON_DURATION,
    WATER_DETECTED_TOPIC,
    WATER_CLEANING_TOPIC,
)


# States
MONITORING = "MONITORING"
CLEANING = "CLEANING"
COOLDOWN = "COOLDOWN"


class WaterActuator(Node):

    def __init__(self):
        super().__init__("water_actuator")

        self.get_logger().info("Water Actuator Node Started")

        # --- ROS parameters ---
        self.declare_parameter("fan_duration", FAN_ON_DURATION)
        self.declare_parameter("use_gpio_sensor", False)   # True = read GPIO directly
        self.declare_parameter("cooldown_time", 2.0)       # seconds after cleaning before re-checking

        self.fan_duration = self.get_parameter("fan_duration").value
        self.use_gpio_sensor = self.get_parameter("use_gpio_sensor").value
        self.cooldown_time = self.get_parameter("cooldown_time").value

        # --- GPIO setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Actuator outputs (relays — active HIGH by default)
        GPIO.setup(VACUUM_PUMP_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(DC_FAN_PIN, GPIO.OUT, initial=GPIO.LOW)

        # Water sensor input (if using GPIO directly)
        if self.use_gpio_sensor:
            GPIO.setup(WATER_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            self.get_logger().info(
                f"GPIO water sensor enabled on pin {WATER_SENSOR_PIN}"
            )

        # --- State ---
        self.state = MONITORING
        self.cleaning_start_time = None
        self.cooldown_start_time = None
        self.water_trigger = False

        # --- ROS interfaces ---
        self.water_sub = self.create_subscription(
            Bool,
            WATER_DETECTED_TOPIC,
            self.water_detected_callback,
            10
        )

        self.cleaning_pub = self.create_publisher(
            Bool,
            WATER_CLEANING_TOPIC,
            10
        )

        # --- Main loop at 10 Hz ---
        self.timer = self.create_timer(0.1, self.main_loop)

        self.get_logger().info(
            f"Config: fan_duration={self.fan_duration}s, "
            f"vacuum_pin=GPIO{VACUUM_PUMP_PIN}, fan_pin=GPIO{DC_FAN_PIN}"
        )

    def water_detected_callback(self, msg):
        """Receive water detection from external sensor/topic."""
        if msg.data and self.state == MONITORING:
            self.water_trigger = True

    def read_gpio_sensor(self):
        """Read water sensor directly from GPIO pin."""
        if not self.use_gpio_sensor:
            return False

        level = GPIO.input(WATER_SENSOR_PIN)
        if WATER_SENSOR_ACTIVE_HIGH:
            return level == GPIO.HIGH
        else:
            return level == GPIO.LOW

    def main_loop(self):
        """State machine for water cleaning."""
        now = self.get_clock().now()

        if self.state == MONITORING:
            # Check for water — either from topic or GPIO
            water_detected = self.water_trigger or self.read_gpio_sensor()
            self.water_trigger = False

            if water_detected:
                self.get_logger().info("💧 WATER DETECTED — Starting cleaning cycle")
                self.state = CLEANING
                self.cleaning_start_time = now

                # Activate actuators
                GPIO.output(VACUUM_PUMP_PIN, GPIO.HIGH)
                GPIO.output(DC_FAN_PIN, GPIO.HIGH)
                self.get_logger().info(
                    f"  Vacuum: ON | Fan: ON (for {self.fan_duration}s)"
                )

                # Signal coverage node to pause
                msg = Bool()
                msg.data = True
                self.cleaning_pub.publish(msg)

        elif self.state == CLEANING:
            # Check if fan duration has elapsed
            elapsed = (now - self.cleaning_start_time).nanoseconds / 1e9

            if elapsed >= self.fan_duration:
                self.get_logger().info("✅ Cleaning cycle complete — Deactivating actuators")

                # Deactivate actuators
                GPIO.output(VACUUM_PUMP_PIN, GPIO.LOW)
                GPIO.output(DC_FAN_PIN, GPIO.LOW)

                # Enter cooldown before resuming
                self.state = COOLDOWN
                self.cooldown_start_time = now

        elif self.state == COOLDOWN:
            elapsed = (now - self.cooldown_start_time).nanoseconds / 1e9

            if elapsed >= self.cooldown_time:
                self.get_logger().info("Cooldown done. Resuming coverage.")
                self.state = MONITORING

                # Signal coverage node to resume
                msg = Bool()
                msg.data = False
                self.cleaning_pub.publish(msg)

    def destroy_node(self):
        """Ensure actuators are OFF on shutdown."""
        try:
            GPIO.output(VACUUM_PUMP_PIN, GPIO.LOW)
            GPIO.output(DC_FAN_PIN, GPIO.LOW)
            GPIO.cleanup([VACUUM_PUMP_PIN, DC_FAN_PIN])
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaterActuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Water Actuator...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
