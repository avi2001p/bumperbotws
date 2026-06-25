#!/usr/bin/env python3
"""
pid.py
------
Closed-loop PID speed controller for the BumperBot.

Subscribes to:
  /cmd_vel      (Twist)            — desired robot velocity
  /wheel_speed  (Float32MultiArray) — actual wheel speeds (ticks/s)

Publishes to:
  /motor_pwm    (Float32MultiArray) — PWM output [-255, 255] per wheel

Converts /cmd_vel (linear.x, angular.z) to per-wheel target speeds
in ticks/sec using differential drive inverse kinematics, then runs
independent PID loops for left and right wheels.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from bumperbot_hardware.parameters import *


class PIDController(Node):

    def __init__(self):

        super().__init__("pid_controller")

        self.get_logger().info("PID Controller Started")

        # --- ROS Parameters (runtime-tunable) ---
        self.declare_parameter("kp", KP)
        self.declare_parameter("ki", KI)
        self.declare_parameter("kd", KD)

        self.kp = self.get_parameter("kp").get_parameter_value().double_value
        self.ki = self.get_parameter("ki").get_parameter_value().double_value
        self.kd = self.get_parameter("kd").get_parameter_value().double_value

        # --- State variables ---
        # Desired robot velocity
        self.linear_x = 0.0
        self.angular_z = 0.0

        # Actual wheel speeds (ticks/sec) from encoder
        self.left_speed = 0.0
        self.right_speed = 0.0

        # Target wheel speeds (ticks/sec)
        self.target_left_speed = 0.0
        self.target_right_speed = 0.0

        # PID internal state — left wheel
        self.left_integral = 0.0
        self.left_prev_actual = 0.0   # for derivative-on-measurement
        self.left_output = 0.0

        # PID internal state — right wheel
        self.right_integral = 0.0
        self.right_prev_actual = 0.0  # for derivative-on-measurement
        self.right_output = 0.0

        # Control loop period
        self.dt = 1.0 / CONTROL_RATE

        # Safety watchdog — last time a /cmd_vel was received
        self.last_cmd_time = self.get_clock().now()

        # --- Subscribers ---
        self.create_subscription(
            Twist,
            CMD_VEL_TOPIC,
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            WHEEL_SPEED_TOPIC,
            self.speed_callback,
            10
        )

        # --- Publisher: PWM commands to motor driver ---
        self.pwm_pub = self.create_publisher(
            Float32MultiArray,
            MOTOR_PWM_TOPIC,
            10
        )

        # --- Control loop timer ---
        self.control_timer = self.create_timer(
            self.dt,
            self.control_loop
        )

        # --- Status logging timer ---
        self.status_timer = self.create_timer(
            0.5,
            self.print_status
        )

    def cmd_vel_callback(self, msg):
        """Convert desired robot velocity to per-wheel target speeds."""
        self.last_cmd_time = self.get_clock().now()
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        # Differential drive inverse kinematics
        # v_left  = v - (omega * L / 2)
        # v_right = v + (omega * L / 2)
        left_linear = (
            self.linear_x -
            (self.angular_z * WHEEL_BASE / 2.0)
        )

        right_linear = (
            self.linear_x +
            (self.angular_z * WHEEL_BASE / 2.0)
        )

        # Convert linear wheel speed (m/s) to encoder ticks/sec
        self.target_left_speed = (
            left_linear / WHEEL_CIRCUMFERENCE
        ) * TICKS_PER_REV

        self.target_right_speed = (
            right_linear / WHEEL_CIRCUMFERENCE
        ) * TICKS_PER_REV

    def speed_callback(self, msg):
        """Receive actual wheel speeds and low-pass filter them (EMA)."""
        a = SPEED_FILTER_ALPHA
        self.left_speed = a * msg.data[0] + (1.0 - a) * self.left_speed
        self.right_speed = a * msg.data[1] + (1.0 - a) * self.right_speed

    def compute_pid(self, target, actual, prev_actual, integral, ff_trim=1.0):
        """
        Compute feed-forward + PID output for one wheel.
        ff_trim scales this wheel's feed-forward to balance unequal motors.
        Returns: (output, new_prev_actual, new_integral)
        """
        # Re-read params for live tuning
        kp = self.get_parameter("kp").get_parameter_value().double_value
        ki = self.get_parameter("ki").get_parameter_value().double_value
        kd = self.get_parameter("kd").get_parameter_value().double_value

        error = target - actual

        # Feed-forward: baseline PWM proportional to the target speed so both
        # wheels move together immediately and the PID only trims the error.
        # ff_trim balances a stronger/weaker motor so equal command = equal speed.
        ff_term = KFF * target * ff_trim

        # Proportional
        p_term = kp * error

        # Integral with anti-windup
        integral += error * self.dt
        integral = max(-INTEGRAL_WINDUP_LIMIT, min(INTEGRAL_WINDUP_LIMIT, integral))
        i_term = ki * integral

        # Derivative on MEASUREMENT (not error) — avoids the spike when the
        # setpoint changes. d(error)/dt = -d(actual)/dt for a constant target.
        derivative = -(actual - prev_actual) / self.dt
        d_term = kd * derivative

        # Combined output
        output = ff_term + p_term + i_term + d_term
        output = self._dir_clamp(output, target)

        return output, actual, integral

    @staticmethod
    def _dir_clamp(output, target):
        """Direction-locked clamp: a wheel commanded FORWARD may only
        drive-forward-or-coast (never reverse), and vice-versa. Stops the
        controller flip-flopping the motor direction (the "dancing")."""
        if target > 0.0:
            return max(0.0, min(PID_OUTPUT_MAX, output))
        if target < 0.0:
            return max(PID_OUTPUT_MIN, min(0.0, output))
        return 0.0

    def _slew_limit(self, prev, target_out):
        """Limit how fast the output may change between control cycles."""
        delta = target_out - prev
        if delta > OUTPUT_SLEW_LIMIT:
            return prev + OUTPUT_SLEW_LIMIT
        if delta < -OUTPUT_SLEW_LIMIT:
            return prev - OUTPUT_SLEW_LIMIT
        return target_out

    def control_loop(self):
        """Run PID computation and publish motor PWM commands."""

        # Safety watchdog: if no /cmd_vel has arrived recently, force a stop so
        # the robot can't run away when the commanding node dies or is killed.
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > CMD_VEL_TIMEOUT:
            self.target_left_speed = 0.0
            self.target_right_speed = 0.0

        # If no velocity commanded, stop immediately (no PID needed)
        if self.target_left_speed == 0.0 and self.target_right_speed == 0.0:
            self.left_output = 0.0
            self.right_output = 0.0
            self.left_integral = 0.0
            self.right_integral = 0.0
            self.left_prev_actual = self.left_speed
            self.right_prev_actual = self.right_speed
        else:
            # Remember previous outputs for slew-rate limiting
            prev_left = self.left_output
            prev_right = self.right_output

            # PID for left wheel
            new_left, self.left_prev_actual, self.left_integral = (
                self.compute_pid(
                    self.target_left_speed,
                    self.left_speed,
                    self.left_prev_actual,
                    self.left_integral,
                    LEFT_FF_TRIM
                )
            )

            # PID for right wheel
            new_right, self.right_prev_actual, self.right_integral = (
                self.compute_pid(
                    self.target_right_speed,
                    self.right_speed,
                    self.right_prev_actual,
                    self.right_integral,
                    RIGHT_FF_TRIM
                )
            )

            # Cross-coupled wheel SYNCHRONISATION: drive the ACTUAL left-right
            # speed difference to the INTENDED difference. For straight driving
            # (intended diff = 0) this forces both wheels to the SAME speed even
            # if one motor is weaker — it locks the two wheels together.
            sync = K_SYNC * ((self.left_speed - self.right_speed)
                             - (self.target_left_speed - self.target_right_speed))
            new_left = self._dir_clamp(new_left - sync, self.target_left_speed)
            new_right = self._dir_clamp(new_right + sync, self.target_right_speed)

            # Slew-rate limit so the PWM ramps instead of snapping 0<->full,
            # which removes the surge/coast "move-stop-move" behaviour.
            self.left_output = self._slew_limit(prev_left, new_left)
            self.right_output = self._slew_limit(prev_right, new_right)

        # Publish PWM to motor driver
        pwm_msg = Float32MultiArray()
        pwm_msg.data = [
            float(self.left_output),
            float(self.right_output)
        ]
        self.pwm_pub.publish(pwm_msg)

    def print_status(self):

        # Diagnostic hint for the common "output stuck at 0.0" case
        if self.target_left_speed == 0.0 and self.target_right_speed == 0.0:
            hint = "  [target=0 → no /cmd_vel reaching PID]"
        elif self.left_speed == 0.0 and self.right_speed == 0.0:
            hint = "  [actual=0 → no /wheel_speed feedback (check encoders)]"
        else:
            hint = ""

        self.get_logger().info(
            f"PID | "
            f"L: tgt={self.target_left_speed:+7.1f} act={self.left_speed:+7.1f} out={self.left_output:+7.1f} | "
            f"R: tgt={self.target_right_speed:+7.1f} act={self.right_speed:+7.1f} out={self.right_output:+7.1f}"
            f"{hint}"
        )


def main(args=None):

    rclpy.init(args=args)

    node = PIDController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Stopping PID Controller...")

    finally:
        # Send zero PWM before shutting down
        try:
            stop_msg = Float32MultiArray()
            stop_msg.data = [0.0, 0.0]
            node.pwm_pub.publish(stop_msg)
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()