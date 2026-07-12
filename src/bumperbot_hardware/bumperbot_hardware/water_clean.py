#!/usr/bin/env python3
"""
water_clean.py
--------------
Water cleaning. When EITHER water sensor detects water:
  1. STOP  (stop_duration s): robot stopped (coverage PAUSED), vacuum + fan ON.
  2. MOVE  (move_duration s): robot driving again (coverage RESUMED),
                              vacuum + fan STILL ON.
  3. Vacuum + fan OFF. Total runtime = stop_duration + move_duration (10 s).
  4. COOLDOWN before the next detection can trigger.

Publishes /water_cleaning_active so the coverage node pauses (phase 1) and
resumes (phase 2).

(The roller servo is not wired in yet — it will lower during phase 2 once it is
calibrated. Use servo_test to calibrate it.)

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
        # On detection the vacuum+fan run for (stop_duration + move_duration):
        #   stop_duration  -> robot STOPPED  (coverage paused)
        #   move_duration  -> robot MOVING   (coverage resumed)  <-- roller goes here later
        # then they switch OFF and a cooldown blocks re-triggering.
        self.declare_parameter("stop_duration", 5.0)
        self.declare_parameter("move_duration", 5.0)
        self.declare_parameter("cooldown", 5.0)
        active_high = self.get_parameter("relay_active_high").value
        self.use_sensor1 = self.get_parameter("use_sensor1").value
        self.use_sensor2 = self.get_parameter("use_sensor2").value
        self.stop_duration = self.get_parameter("stop_duration").value
        self.move_duration = self.get_parameter("move_duration").value
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
                # Phase 1: STOP the robot, vacuum + fan ON
                GPIO.output(VACUUM_PUMP_PIN, self.on)
                GPIO.output(DC_FAN_PIN, self.on)
                self.pub.publish(Bool(data=True))          # coverage PAUSES
                self.state = "CLEAN_STOPPED"
                self.t_mark = now
                self.get_logger().info(
                    f"WATER DETECTED -> ROBOT STOPPED, VACUUM + FAN ON "
                    f"({self.stop_duration:.0f}s stationary)"
                )

        elif self.state == "CLEAN_STOPPED":
            if now - self.t_mark >= self.stop_duration:
                # Phase 2: resume driving, vacuum + fan STAY ON
                self.pub.publish(Bool(data=False))         # coverage RESUMES
                self.state = "CLEAN_MOVING"
                self.t_mark = now
                self.get_logger().info(
                    f"ROBOT MOVING again — VACUUM + FAN STILL ON "
                    f"({self.move_duration:.0f}s while driving)"
                )

        elif self.state == "CLEAN_MOVING":
            if now - self.t_mark >= self.move_duration:
                # Done: total vacuum+fan runtime = stop_duration + move_duration
                GPIO.output(VACUUM_PUMP_PIN, self.off)
                GPIO.output(DC_FAN_PIN, self.off)
                self.state = "COOLDOWN"
                self.t_mark = now
                total = self.stop_duration + self.move_duration
                self.get_logger().info(
                    f"VACUUM + FAN OFF (ran {total:.0f}s total). "
                    f"Cooldown {self.cooldown:.0f}s before next detection."
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
