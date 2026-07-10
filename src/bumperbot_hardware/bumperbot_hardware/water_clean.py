#!/usr/bin/env python3
"""
water_clean.py
--------------
Water extraction sequence. When EITHER water sensor detects water:

  1. STOP + CLEAN  (fan_duration s): vacuum + fan ON, coverage PAUSED, roller UP.
  2. ROLL          (roller_run_duration s): vacuum + fan OFF, coverage RESUMES,
                    roller DOWN and rolling while the robot drives.
  3. Roller UP, then COOLDOWN before the next detection.

Publishes /water_cleaning_active so the coverage node pauses (step 1) and resumes
(step 2). Relays via RPi.GPIO; roller servo via pigpio (needs `sudo pigpiod`;
if it isn't running the roller is skipped and the rest still works).

Run:
  ros2 run bumperbot_hardware water_clean
  ros2 run bumperbot_hardware water_clean --ros-args -p relay_active_high:=true
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import RPi.GPIO as GPIO
import pigpio

from bumperbot_hardware.parameters import (
    VACUUM_PUMP_PIN,
    DC_FAN_PIN,
    WATER_SENSOR_PIN_1,
    WATER_SENSOR_PIN_2,
    WATER_SENSOR_ACTIVE_HIGH,
    WATER_CLEANING_TOPIC,
    SERVO_PIN,
    SERVO_MIN_PULSE_MS,
    SERVO_MAX_PULSE_MS,
    ROLLER_UP_ANGLE,
    ROLLER_DOWN_ANGLE,
    ROLLER_RUN_DURATION,
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
        self.declare_parameter("roller_run_duration", ROLLER_RUN_DURATION)
        active_high = self.get_parameter("relay_active_high").value
        self.use_sensor1 = self.get_parameter("use_sensor1").value
        self.use_sensor2 = self.get_parameter("use_sensor2").value
        self.fan_duration = self.get_parameter("fan_duration").value
        self.cooldown = self.get_parameter("cooldown").value
        self.roller_run_duration = self.get_parameter("roller_run_duration").value
        self.on = GPIO.HIGH if active_high else GPIO.LOW
        self.off = GPIO.LOW if active_high else GPIO.HIGH

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(VACUUM_PUMP_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(DC_FAN_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(WATER_SENSOR_PIN_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(WATER_SENSOR_PIN_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # --- Roller servo via pigpio (starts lifted UP) ---
        self.min_us = int(SERVO_MIN_PULSE_MS * 1000)
        self.max_us = int(SERVO_MAX_PULSE_MS * 1000)
        self.pi = pigpio.pi()
        if self.pi is not None and self.pi.connected:
            self.set_roller(ROLLER_UP_ANGLE)
            self.get_logger().info(f"Roller servo ready on GPIO{SERVO_PIN} (UP).")
        else:
            self.pi = None
            self.get_logger().warn(
                "pigpio not running -> roller DISABLED (start it: sudo pigpiod). "
                "Water cleaning still works without the roller."
            )

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

    def set_roller(self, angle):
        """Move the roller servo to `angle` (pigpio holds it silently)."""
        if self.pi is None:
            return
        angle = max(0.0, min(180.0, angle))
        us = int(self.min_us + (angle / 180.0) * (self.max_us - self.min_us))
        self.pi.set_servo_pulsewidth(SERVO_PIN, us)

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
                # Vacuum+fan done -> resume driving, then roll while moving
                GPIO.output(VACUUM_PUMP_PIN, self.off)
                GPIO.output(DC_FAN_PIN, self.off)
                self.pub.publish(Bool(data=False))     # coverage RESUMES (robot moves)
                self.set_roller(ROLLER_DOWN_ANGLE)     # roller DOWN
                self.state = "ROLLING"
                self.t_mark = now
                self.get_logger().info(
                    f"Vacuum+fan OFF, coverage RESUMED. Roller DOWN for "
                    f"{self.roller_run_duration:.0f}s while moving."
                )

        elif self.state == "ROLLING":
            # Robot is driving with the roller down; lift it after the run time
            if now - self.t_mark >= self.roller_run_duration:
                self.set_roller(ROLLER_UP_ANGLE)       # roller UP
                self.state = "COOLDOWN"
                self.t_mark = now
                self.get_logger().info(
                    f"Roller UP. Cooldown {self.cooldown:.0f}s before next detection."
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
        try:
            if self.pi is not None:
                self.set_roller(ROLLER_UP_ANGLE)              # park roller up
                self.pi.set_servo_pulsewidth(SERVO_PIN, 0)    # release
                self.pi.stop()
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
