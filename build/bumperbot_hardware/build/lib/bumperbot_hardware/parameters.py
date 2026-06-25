"""
parameters.py
--------------
Robot hardware parameters for the BumperBot.
"""
# ==========================================================
# MOTOR DRIVER (L298N)
# ==========================================================
# Right Motor (OUT1 / OUT2)
RIGHT_EN = 18      # GPIO18 (Physical Pin 12)
RIGHT_IN1 = 17     # GPIO17 (Physical Pin 11)
RIGHT_IN2 = 27     # GPIO27 (Physical Pin 13)
# Left Motor (OUT3 / OUT4)
LEFT_EN = 19       # GPIO19 (Physical Pin 35)
LEFT_IN1 = 22      # GPIO22 (Physical Pin 15)
LEFT_IN2 = 23      # GPIO23 (Physical Pin 16)
# PWM Settings
PWM_FREQUENCY = 1000      # Hz
MAX_PWM = 100             # %
MIN_PWM = 0               # %
# ==========================================================
# ENCODERS
# ==========================================================
# Left Encoder
LEFT_ENCODER_A = 5     # GPIO5  (Pin 29)
LEFT_ENCODER_B = 6     # GPIO6  (Pin 31)
# Right Encoder
RIGHT_ENCODER_A = 20   # GPIO20 (Pin 38)
RIGHT_ENCODER_B = 21   # GPIO21 (Pin 40)
ENCODER_VOLTAGE = 3.3
# ==========================================================
# ROBOT MECHANICAL PARAMETERS
# ==========================================================
# Wheel diameter (meters)
WHEEL_DIAMETER = 0.065
# Wheel radius (meters)
WHEEL_RADIUS = WHEEL_DIAMETER / 2.0
# Wheel circumference (meters)
WHEEL_CIRCUMFERENCE = 3.141592653589793 * WHEEL_DIAMETER
# Distance between wheel centers (meters)
WHEEL_BASE = 0.229       # measured: centre to centre of tyres
# ==========================================================
# ENCODER PARAMETERS
# ==========================================================
# Encoder ticks per wheel revolution
# 11 pulses × 2 channels × 41 gear ratio = 902
TICKS_PER_REV = 902
# ==========================================================
# ROBOT SPEED LIMITS
# ==========================================================
MAX_LINEAR_SPEED = 0.50      # m/s
MAX_ANGULAR_SPEED = 2.00     # rad/s
# ==========================================================
# PID CONTROLLER
# ==========================================================
KP = 1.0
KI = 0.0
KD = 0.0
CONTROL_RATE = 10.0      # Hz
# ==========================================================
# ROBOT INFORMATION
# ==========================================================
ROBOT_NAME = "bumperbot"
# ==========================================================
# ROS TOPICS
# ==========================================================
CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
WHEEL_TICKS_TOPIC = "/wheel_ticks"
WHEEL_SPEED_TOPIC = "/wheel_speed"
# ==========================================================
# NODE NAMES
# ==========================================================
MOTOR_NODE = "motor_driver"
ENCODER_NODE = "encoder_reader"
PID_NODE = "pid_controller"
ODOMETRY_NODE = "odometry"