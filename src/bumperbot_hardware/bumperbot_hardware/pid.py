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
        self.left_prev_error = 0.0
        self.left_output = 0.0

        # PID internal state — right wheel
        self.right_integral = 0.0
        self.right_prev_error = 0.0
        self.right_output = 0.0

        # Control loop period
        self.dt = 1.0 / CONTROL_RATE

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
        """Receive actual wheel speeds from encoder reader."""
        self.left_speed = msg.data[0]
        self.right_speed = msg.data[1]

    def compute_pid(self, target, actual, prev_error, integral):
        """
        Compute PID output for one wheel.
        Returns: (output, new_prev_error, new_integral)
        """
        # Re-read params for live tuning
        kp = self.get_parameter("kp").get_parameter_value().double_value
        ki = self.get_parameter("ki").get_parameter_value().double_value
        kd = self.get_parameter("kd").get_parameter_value().double_value

        error = target - actual

        # Proportional
        p_term = kp * error

        # Integral with anti-windup
        integral += error * self.dt
        integral = max(-INTEGRAL_WINDUP_LIMIT, min(INTEGRAL_WINDUP_LIMIT, integral))
        i_term = ki * integral

        # Derivative
        derivative = (error - prev_error) / self.dt
        d_term = kd * derivative

        # Combined output, clamped
        output = p_term + i_term + d_term
        output = max(PID_OUTPUT_MIN, min(PID_OUTPUT_MAX, output))

        return output, error, integral

    def control_loop(self):
        """Run PID computation and publish motor PWM commands."""

        # If no velocity commanded, stop immediately (no PID needed)
        if self.target_left_speed == 0.0 and self.target_right_speed == 0.0:
            self.left_output = 0.0
            self.right_output = 0.0
            self.left_integral = 0.0
            self.right_integral = 0.0
            self.left_prev_error = 0.0
            self.right_prev_error = 0.0
        else:
            # PID for left wheel
            self.left_output, self.left_prev_error, self.left_integral = (
                self.compute_pid(
                    self.target_left_speed,
                    self.left_speed,
                    self.left_prev_error,
                    self.left_integral
                )
            )

            # PID for right wheel
            self.right_output, self.right_prev_error, self.right_integral = (
                self.compute_pid(
                    self.target_right_speed,
                    self.right_speed,
                    self.right_prev_error,
                    self.right_integral
                )
            )

        # Publish PWM to motor driver
        pwm_msg = Float32MultiArray()
        pwm_msg.data = [
            float(self.left_output),
            float(self.right_output)
        ]
        self.pwm_pub.publish(pwm_msg)

    def print_status(self):

        self.get_logger().info(
            f"PID | "
            f"L: tgt={self.target_left_speed:+7.1f} act={self.left_speed:+7.1f} out={self.left_output:+7.1f} | "
            f"R: tgt={self.target_right_speed:+7.1f} act={self.right_speed:+7.1f} out={self.right_output:+7.1f}"
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