#!/usr/bin/env python3
"""
water_clean.py
--------------
Water cleaning. When EITHER water sensor detects water:

  1. STOP  (stop_duration s) — coverage PAUSED, robot stationary
                               vacuum + fan ON, roller UP
  2. MOVE  (move_duration s) — coverage RESUMED, robot driving
                               vacuum + fan STILL ON, roller DOWN (sweeping)
  3. Roller UP, then vacuum + fan OFF.
     Total extraction time = stop_duration + move_duration (10 s).
  4. COOLDOWN before the next detection can trigger.

Publishes /water_cleaning_active so the coverage node pauses (phase 1) and
resumes (phase 2).

The roller needs the pigpio daemon:   sudo pigpiod
If it is not running (or the servo is not fitted) the node logs a warning and
runs WITHOUT the roller — extraction still works. Disable it explicitly with
-p use_roller:=false.

Run:
  ros2 run bumperbot_hardware water_clean
  ros2 run bumperbot_hardware water_clean --ros-args \
      -p roller_up_angle:=20 -p roller_down_angle:=0 -p poll_rate:=50.0
"""

import time

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
    SERVO_PIN,
    SERVO_MIN_PULSE_MS,
    SERVO_MAX_PULSE_MS,
    ROLLER_UP_ANGLE,
    ROLLER_DOWN_ANGLE,
)

# pigpio drives the servo on hardware-timed PWM. RPi.GPIO's software PWM jitters,
# which makes the servo buzz and creep. Imported softly: a missing library or a
# stopped daemon must NOT take the whole water mission down in the field — the
# roller is an enhancement, extraction still works without it.
try:
    import pigpio
except ImportError:
    pigpio = None


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
        # How often the sensors are READ. This — not the driving speed — sets the
        # smallest wet spot the robot can catch: between two polls it travels
        # (speed / poll_rate) metres, and a patch narrower than that can pass under
        # the probe unseen. At 0.12 m/s, 5 Hz leaves a 24 mm blind gap; 20 Hz cuts
        # it to 6 mm. Polling faster is far cheaper than driving slower.
        self.declare_parameter("poll_rate", 20.0)   # Hz

        # --- Roller ---
        # Lowered onto the floor for the MOVING phase, so it sweeps water toward
        # the vacuum intake while the robot drives, then lifted again. Angles come
        # from your servo_test calibration.
        self.declare_parameter("use_roller", True)
        self.declare_parameter("roller_up_angle", ROLLER_UP_ANGLE)
        self.declare_parameter("roller_down_angle", ROLLER_DOWN_ANGLE)
        # Lifting the roller loads the servo, so it lands SHORT of the commanded
        # up angle. Climb this far past it, then settle back — the roller returns
        # to its exact park position instead of drooping a few degrees low.
        self.declare_parameter("roller_up_overshoot", 15.0)
        active_high = self.get_parameter("relay_active_high").value
        self.use_sensor1 = self.get_parameter("use_sensor1").value
        self.use_sensor2 = self.get_parameter("use_sensor2").value
        self.stop_duration = self.get_parameter("stop_duration").value
        self.move_duration = self.get_parameter("move_duration").value
        self.cooldown = self.get_parameter("cooldown").value
        self.poll_rate = max(1.0, self.get_parameter("poll_rate").value)
        self.use_roller = self.get_parameter("use_roller").value
        self.roller_up = self.get_parameter("roller_up_angle").value
        self.roller_down = self.get_parameter("roller_down_angle").value
        self.roller_overshoot = self.get_parameter("roller_up_overshoot").value
        self.on = GPIO.HIGH if active_high else GPIO.LOW
        self.off = GPIO.LOW if active_high else GPIO.HIGH

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(VACUUM_PUMP_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(DC_FAN_PIN, GPIO.OUT, initial=self.off)
        GPIO.setup(WATER_SENSOR_PIN_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(WATER_SENSOR_PIN_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # --- Roller servo (optional; degrades gracefully) ---
        self.pi = None
        if self.use_roller:
            if pigpio is None:
                self.get_logger().warn(
                    "use_roller is set but the pigpio library is not installed — "
                    "running WITHOUT the roller. Extraction still works."
                )
                self.use_roller = False
            else:
                self.pi = pigpio.pi()
                if not self.pi.connected:
                    self.get_logger().warn(
                        "use_roller is set but the pigpio daemon is not running "
                        "(start it with:  sudo pigpiod) — running WITHOUT the "
                        "roller. Extraction still works."
                    )
                    self.pi = None
                    self.use_roller = False
                else:
                    self.roller_lift()   # park it up before the mission starts
                    self.get_logger().info(
                        f"Roller ready on GPIO{SERVO_PIN}: "
                        f"UP={self.roller_up:.0f} deg, DOWN={self.roller_down:.0f} deg"
                    )

        # Tells the coverage node to pause/resume while cleaning
        self.pub = self.create_publisher(Bool, WATER_CLEANING_TOPIC, 10)

        # MONITORING -> CLEAN_STOPPED -> CLEAN_MOVING -> COOLDOWN -> MONITORING
        self.state = "MONITORING"
        self.t_mark = 0.0
        self.timer = self.create_timer(1.0 / self.poll_rate, self.loop)

        self.get_logger().info(
            f"Water clean started: EITHER enabled sensor WET -> vacuum+fan ON. "
            f"vacuum=GPIO{VACUUM_PUMP_PIN} fan=GPIO{DC_FAN_PIN} "
            f"sensor1(GPIO{WATER_SENSOR_PIN_1})={'ON' if self.use_sensor1 else 'OFF'} "
            f"sensor2(GPIO{WATER_SENSOR_PIN_2})={'ON' if self.use_sensor2 else 'OFF'}"
        )
        self.get_logger().info(
            f"Polling at {self.poll_rate:.0f} Hz -> at 0.10 m/s the robot moves "
            f"{0.10 / self.poll_rate * 1000.0:.0f} mm between sensor reads "
            f"(the smallest wet spot it can reliably catch). "
            f"Sequence: STOP {self.stop_duration:.0f}s + MOVE {self.move_duration:.0f}s "
            f"= {self.stop_duration + self.move_duration:.0f}s of vacuum+fan."
        )

    # ------------------------------ ROLLER ------------------------------

    def servo_us(self, angle):
        """Servo pulse width (microseconds) for an angle in degrees."""
        angle = max(0.0, min(180.0, angle))
        lo = SERVO_MIN_PULSE_MS * 1000.0
        hi = SERVO_MAX_PULSE_MS * 1000.0
        return int(lo + (angle / 180.0) * (hi - lo))

    def roller_drop(self):
        """Lower the roller onto the floor (sweep position)."""
        if not self.use_roller or self.pi is None:
            return
        self.pi.set_servo_pulsewidth(SERVO_PIN, self.servo_us(self.roller_down))

    def roller_lift(self):
        """Raise the roller clear of the floor (park position).

        Overshoot-then-settle: under the weight of the roller arm the servo lands
        short of the commanded angle, so it is driven PAST the park angle and then
        back onto it. Without this the roller creeps a few degrees lower on every
        cycle and eventually drags.
        """
        if not self.use_roller or self.pi is None:
            return
        direction = 1.0 if self.roller_up > self.roller_down else -1.0
        past = self.roller_up + direction * self.roller_overshoot
        self.pi.set_servo_pulsewidth(SERVO_PIN, self.servo_us(past))
        time.sleep(0.25)
        self.pi.set_servo_pulsewidth(SERVO_PIN, self.servo_us(self.roller_up))

    # ------------------------------ SENSORS ------------------------------

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
                # Phase 2: resume driving, ROLLER DOWN, vacuum + fan STAY ON
                self.roller_drop()
                self.pub.publish(Bool(data=False))         # coverage RESUMES
                self.state = "CLEAN_MOVING"
                self.t_mark = now
                self.get_logger().info(
                    f"ROBOT MOVING again — ROLLER DOWN, VACUUM + FAN STILL ON "
                    f"({self.move_duration:.0f}s while driving)"
                    + ("" if self.use_roller else "  [roller disabled]")
                )

        elif self.state == "CLEAN_MOVING":
            if now - self.t_mark >= self.move_duration:
                # Done: roller back up, then vacuum + fan off.
                # Lift FIRST — cutting the suction while the roller is still down
                # would drag the swept water back across clean floor.
                self.roller_lift()
                GPIO.output(VACUUM_PUMP_PIN, self.off)
                GPIO.output(DC_FAN_PIN, self.off)
                self.state = "COOLDOWN"
                self.t_mark = now
                total = self.stop_duration + self.move_duration
                self.get_logger().info(
                    f"ROLLER UP, VACUUM + FAN OFF (ran {total:.0f}s total). "
                    f"Cooldown {self.cooldown:.0f}s before next detection."
                )

        elif self.state == "COOLDOWN":
            if now - self.t_mark >= self.cooldown:
                self.state = "MONITORING"
                self.get_logger().info("Ready — monitoring for water again.")

    def destroy_node(self):
        try:
            # Park the roller up before releasing the servo, so it does not drop
            # onto the floor and drag when the node exits.
            self.roller_lift()
            if self.pi is not None:
                self.pi.stop()
        except Exception:
            pass
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
