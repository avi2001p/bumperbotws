"""
parameters.py
--------------
Robot hardware parameters for the BumperBot.
Updated for 25GA-370/12V/280RPM DC Reducer Gear Encoder Motor.
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
# Motor direction signs: set a motor to -1 if a POSITIVE command makes that
# wheel spin BACKWARD (i.e. the motor leads are wired reversed). This flips
# the H-bridge direction in software instead of re-wiring.
LEFT_MOTOR_SIGN = +1
RIGHT_MOTOR_SIGN = +1
# ==========================================================
# ENCODERS (25GA-370 built-in Hall sensor encoder)
# ==========================================================
# Left Encoder
LEFT_ENCODER_A = 6     # GPIO6  (Pin 31)
LEFT_ENCODER_B = 5     # GPIO5  (Pin 29)
# Right Encoder
RIGHT_ENCODER_A = 21   # GPIO21 (Pin 40)
RIGHT_ENCODER_B = 20   # GPIO20 (Pin 38)
ENCODER_VOLTAGE = 3.3
# ==========================================================
# ROBOT MECHANICAL PARAMETERS
# ==========================================================
# Wheel diameter (meters) — 65mm wheels
WHEEL_DIAMETER = 0.065
# Wheel radius (meters)
WHEEL_RADIUS = WHEEL_DIAMETER / 2.0
# Wheel circumference (meters)
WHEEL_CIRCUMFERENCE = 3.141592653589793 * WHEEL_DIAMETER
# Distance between wheel centers (meters) — measured centre to centre
WHEEL_BASE = 0.229
# Robot body dimensions (meters)
ROBOT_WIDTH = 0.220       # 220mm
ROBOT_LENGTH = 0.187      # 187mm
# Front wheel motor axle distance from front edge (meters)
MOTOR_AXLE_FROM_FRONT = 0.07   # 7cm from the front
# ==========================================================
# ENCODER PARAMETERS (25GA-370 / 280RPM)
# Motor: 25GA-370, 12V, 280RPM
# Built-in Hall sensor encoder: 11 PPR per motor shaft revolution
# Gear ratio for 280RPM variant: approximately 21.3:1
# Using GPIO.BOTH on Channel A: 11 * 2 = 22 counts per motor rev
# Per output shaft (wheel) revolution: 22 * 21.3 ≈ 469
# >>> CALIBRATE THIS: run calibrate_encoders node and manually
#     rotate each wheel exactly 1 full revolution to get true count.
# ==========================================================
ENCODER_PPR = 11              # Pulses per motor shaft revolution
GEAR_RATIO = 20.5             # Calibrated gear ratio
TICKS_PER_REV = 448           # Calibrated: 2 m roll = L4479/R4306 ticks over 9.794 revs (avg 448)
# Per-wheel tick calibration. Two 2 m rolls showed the LEFT encoder counts ~4%
# more ticks per metre than the RIGHT (L/R ratio = 1.040 then 1.045 — consistent
# across different push distances, so it's a REAL asymmetry, not push error).
# ODOMETRY uses these per-wheel values so straight driving reads as straight
# (kills the ~20° false heading drift a single value caused over 2 m). The PID
# still uses the single TICKS_PER_REV above (symmetric → does NOT disturb the
# mechanically-balanced straight line). Switch the PID to per-wheel later only
# if needed.
LEFT_TICKS_PER_REV = 457
RIGHT_TICKS_PER_REV = 440
# ----------------------------------------------------------
# ENCODER DIRECTION SIGNS
# ----------------------------------------------------------
# Sign applied to each wheel's tick increment so that DRIVING
# THE ROBOT FORWARD produces POSITIVE ticks on both wheels.
# >>> Verify on the robot: run encoder_reader, echo /wheel_ticks,
#     push each wheel forward by hand. If ticks go negative,
#     flip that wheel's sign to -1 (no other code change needed).
LEFT_ENCODER_SIGN = +1
RIGHT_ENCODER_SIGN = -1   # right encoder reads reversed on this robot (verified)
# ==========================================================
# ROBOT SPEED LIMITS
# ==========================================================
MAX_LINEAR_SPEED = 0.30      # m/s (conservative for coverage)
MAX_ANGULAR_SPEED = 2.00     # rad/s
# ==========================================================
# PID CONTROLLER
# ==========================================================
# Gains — FEED-FORWARD-DOMINANT design.
# The feed-forward (KFF) sets the base PWM so both wheels turn TOGETHER smoothly.
# KP is kept very small on purpose: a big KP reacts to the noisy encoder speed
# and makes the two independent wheel loops oscillate out of phase ("one wheel
# then the other"). KI slowly trims to the right speed. Straightness is handled
# by the heading loop (KP_HEADING), not by the per-wheel loops.
# FEED-FORWARD + SLOW-INTEGRAL design (no proportional, no derivative).
# The feed-forward (KFF) gives each wheel a steady base PWM; the integral (KI)
# slowly trims the stronger/weaker wheel so they MATCH and the robot goes
# straight. KI=0.5 is the value that gave the good "straight for ~50 cm" run.
# The late LEFT turn after 50 cm was the integral OVERSHOOTING: the filtered
# encoder speed lags reality, so the integral keeps winding PAST the balanced
# point and over-corrects into a left curve. The fix is NOT removing the
# integral (KI=0 made it curve right from the start) — it is the lower
# INTEGRAL_WINDUP_LIMIT below, which stops the integral winding far enough to
# overshoot, so it settles at straight instead of turning left.
KP = 0.0
KI = 0.5
KD = 0.0
# Feed-forward gain: baseline PWM per target tick/sec. The PID only has to
# TRIM the small remaining error instead of building up the whole command,
# which makes both wheels respond together and drive straight.
#   KFF ≈ PID_OUTPUT_MAX / (max ticks/sec at MAX_LINEAR_SPEED)
#       = 255 / ((0.30 / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV) ≈ 0.38
KFF = 0.38
# Per-wheel feed-forward TRIM — kept at 1.0 / 1.0 (NO software trim) ON PURPOSE.
# The straight-line balance is done MECHANICALLY: the 18650 battery pack is
# mounted on the LEFT side (the spot the phone was tested in), which loads the
# left wheel and cancels its over-drive. Do NOT add a software trim on top of
# that physical weight or the two corrections stack and the robot curves RIGHT.
# >>> After the 12V fan + battery pack are installed the weight changes, so we
#     RE-TUNE then (likely just INTEGRAL_WINDUP_LIMIT, maybe a small trim here).
LEFT_FF_TRIM = 1.0
RIGHT_FF_TRIM = 1.0
PID_OUTPUT_MIN = -255.0
PID_OUTPUT_MAX = 255.0
# Clamp on the per-wheel integral. THIS is the knob that fixes the late drift,
# and it is now BRACKETED:
#   150 -> integral over-corrects -> drifts LEFT after 50 cm
#    50 -> integral under-corrects -> drifts RIGHT after 50 cm
# so the balanced value is in the middle. 90 is the midpoint to try.
#   drifts LEFT at the end  -> lower this (80, 70)
#   drifts RIGHT at the end -> raise this (100, 110)
INTEGRAL_WINDUP_LIMIT = 90.0
CONTROL_RATE = 20.0             # Hz (control loop frequency)
MIN_PWM_DEADZONE = 40.0         # Minimum PWM (out of 255) to overcome motor static friction
# Safety watchdog: if no /cmd_vel arrives within this many seconds, the PID
# zeroes its targets so the robot stops (prevents runaway if the commander
# node dies or is Ctrl+C'd).
CMD_VEL_TIMEOUT = 0.5           # seconds
# Wheel-speed feedback smoothing (exponential moving average). The Hall
# encoder speed is noisy/quantized; filtering it stops the PID over-reacting
# and surging. 0 = no filter (use raw), 1 = instant; lower = smoother.
SPEED_FILTER_ALPHA = 0.2
# Output slew-rate limit (max PWM change per control cycle). Prevents the
# command jumping 0 -> full -> 0, which causes the "move-stop-move" surging.
OUTPUT_SLEW_LIMIT = 30.0
# Cross-coupling / wheel-synchronisation gain. Directly drives the ACTUAL
# left-right speed difference to the INTENDED difference, so for straight
# driving both wheels are forced to the SAME speed even if one motor is weaker.
# This is what keeps the two wheels "locked together". DISABLED (0.0): with the
# noisy encoder speed it ping-pongs power between the wheels = the "one then the
# other" alternating. Leave at 0 unless the speed feedback is well filtered.
K_SYNC = 0.0
# ----------------------------------------------------------
# HEADING-HOLD CONTROLLER (straight-line correction)
# ----------------------------------------------------------
# P heading controller — the ACTUAL straight-line fix.
# It steers angular.z = -KP_HEADING * heading_error to actively hold the robot on
# its starting heading, correcting ANY drift (motor imbalance, battery weight on
# one side, wheel slip) AUTOMATICALLY — which is exactly what per-wheel SPEED
# balance (trims/integral/added weight) cannot do. That's why the robot turned
# LEFT with the battery and RIGHT without it: speed balance can't hold heading,
# this can.
# It was DISABLED before only because wheel-odometry heading was biased (~20°/2 m
# false drift). The per-wheel tick calibration (LEFT/RIGHT_TICKS_PER_REV) fixed
# that — straight now reads ~0° — so the heading-hold can finally trust odometry.
#   wiggles / oscillates  -> lower KP_HEADING (1.0, 0.8)
#   corrects too slowly    -> raise KP_HEADING (2.0, 2.5)
KP_HEADING = 1.5
# Cross-track gain: steers back onto the line using odometry X/Y offset.
# DISABLED (0.0) on purpose: odometry heading/position on this robot is too
# noisy/biased (mismatched cheap encoders) for cross-track — correcting hard off
# it actually pushes the robot off course. Re-enable ONLY once heading comes
# from the IMU gyro (true physical yaw), not the wheel encoders.
K_CROSSTRACK = 0.0
KI_HEADING = 0.4   # (legacy)
# Max correction the heading-hold loop may command (rad/s)
MAX_HEADING_CORRECTION = 0.6
# Clamp on the heading integral (rad·s) to prevent windup
HEADING_INTEGRAL_LIMIT = 0.8
# Heading deadband (rad): below this error we command NO correction, so tiny
# odometry noise doesn't cause twitchy micro-steering. Tight (~0.3 deg) so the
# integral is allowed to drive the steady error close to zero.
HEADING_DEADBAND = 0.006   # ~0.3 degrees
# ==========================================================
# ACTUATORS (Water removal system)
# ==========================================================
# Vacuum pump relay GPIO (adjust to your wiring)
VACUUM_PUMP_PIN = 24    # GPIO24 (Physical Pin 18)
# DC fan relay GPIO (adjust to your wiring)
DC_FAN_PIN = 25         # GPIO25 (Physical Pin 22)
# Fan ON duration when water detected (seconds)
FAN_ON_DURATION = 5.0
# ==========================================================
# WATER SENSOR
# ==========================================================
# Digital water sensor GPIO (adjust to your wiring)
WATER_SENSOR_PIN = 16   # GPIO16 (Physical Pin 36)
# Active level: True = HIGH when water detected
WATER_SENSOR_ACTIVE_HIGH = True
# ==========================================================
# COVERAGE GROUND DIMENSIONS (stadium shape)
# ==========================================================
# Width of the ground (diameter of semicircular ends)
GROUND_WIDTH = 1.2              # meters
# Length of the straight rectangular section
GROUND_STRAIGHT_LENGTH = 1.2    # meters
# Semicircle radius (auto-calculated from width)
GROUND_SEMICIRCLE_RADIUS = GROUND_WIDTH / 2.0   # 0.6m
# Total ground length = straight + 2 * semicircle_radius
GROUND_TOTAL_LENGTH = GROUND_STRAIGHT_LENGTH + 2 * GROUND_SEMICIRCLE_RADIUS  # 2.4m
# Overlap between adjacent passes (meters)
COVERAGE_OVERLAP = 0.02
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
MOTOR_PWM_TOPIC = "/motor_pwm"
WATER_DETECTED_TOPIC = "/water_detected"
WATER_CLEANING_TOPIC = "/water_cleaning_active"
# ==========================================================
# NODE NAMES
# ==========================================================
MOTOR_NODE = "motor_driver"
ENCODER_NODE = "encoder_reader"
PID_NODE = "pid_controller"
ODOMETRY_NODE = "odometry"
COVERAGE_NODE = "stadium_coverage"
WATER_ACTUATOR_NODE = "water_actuator"