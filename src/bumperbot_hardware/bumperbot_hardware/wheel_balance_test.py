#!/usr/bin/env python3
"""
wheel_balance_test.py
---------------------
OPEN-LOOP wheel balance check — NO PID, NO ROS, NO correction.

Drives BOTH motors at the SAME fixed PWM and counts each wheel's encoder ticks
once per second, so you can directly SEE whether equal power gives equal speed.

  * If LEFT and RIGHT tick counts are CLOSE  -> motors are balanced; the curve
    under power is a control/encoder issue, not the motors.
  * If they DIFFER a lot                      -> the motors/L298N are unbalanced;
    that asymmetry is what curves the robot. The L/R ratio tells us how much to
    boost the weaker motor (feed-forward) so equal command = equal speed.

Run it (wheels UP is fine for a pure speed comparison):
  python3 wheel_balance_test.py        # both motors at 60%
  python3 wheel_balance_test.py 50     # both motors at 50%

Stop early with Ctrl+C.
"""

import sys
import time

import RPi.GPIO as GPIO

# --- Motor pins (match parameters.py) ---
RIGHT_EN, RIGHT_IN1, RIGHT_IN2 = 18, 17, 27
LEFT_EN,  LEFT_IN1,  LEFT_IN2  = 19, 22, 23

# --- Encoder pins (match parameters.py) ---
LEFT_ENCODER_A,  LEFT_ENCODER_B  = 6, 5
RIGHT_ENCODER_A, RIGHT_ENCODER_B = 21, 20

left_ticks = 0
right_ticks = 0


def left_cb(channel):
    global left_ticks
    left_ticks += 1


def right_cb(channel):
    global right_ticks
    right_ticks += 1


def main():
    global left_ticks, right_ticks

    pwm_level = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    pwm_level = max(0.0, min(100.0, pwm_level))

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    for pin in (RIGHT_EN, RIGHT_IN1, RIGHT_IN2, LEFT_EN, LEFT_IN1, LEFT_IN2):
        GPIO.setup(pin, GPIO.OUT)
    for pin in (LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B):
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    GPIO.add_event_detect(LEFT_ENCODER_A, GPIO.BOTH, callback=left_cb)
    GPIO.add_event_detect(RIGHT_ENCODER_A, GPIO.BOTH, callback=right_cb)

    right_pwm = GPIO.PWM(RIGHT_EN, 1000)
    left_pwm = GPIO.PWM(LEFT_EN, 1000)
    right_pwm.start(0)
    left_pwm.start(0)

    # Drive BOTH motors FORWARD (same direction convention as motor_driver.py)
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)

    print(f"\nBoth motors FORWARD at {pwm_level:.0f}% PWM (no PID).")
    print("Equal motors => LEFT and RIGHT ticks/sec should be CLOSE.\n")

    right_pwm.ChangeDutyCycle(pwm_level)
    left_pwm.ChangeDutyCycle(pwm_level)

    try:
        for sec in range(12):
            left_ticks = 0
            right_ticks = 0
            time.sleep(1.0)
            l, r = left_ticks, right_ticks
            ratio = (l / r) if r else 0.0
            diff_pct = ((l - r) / r * 100.0) if r else 0.0
            print(f"  t={sec + 1:2d}s   LEFT={l:5d}   RIGHT={r:5d}   "
                  f"L/R={ratio:.2f}   diff={diff_pct:+.0f}%")
    except KeyboardInterrupt:
        pass
    finally:
        right_pwm.ChangeDutyCycle(0)
        left_pwm.ChangeDutyCycle(0)
        GPIO.output(RIGHT_IN1, GPIO.LOW)
        GPIO.output(RIGHT_IN2, GPIO.LOW)
        GPIO.output(LEFT_IN1, GPIO.LOW)
        GPIO.output(LEFT_IN2, GPIO.LOW)
        right_pwm.stop()
        left_pwm.stop()
        GPIO.cleanup()
        print("\nStopped. If LEFT and RIGHT differ a lot, the motors are "
              "unbalanced — tell me the numbers and I'll balance them.")


if __name__ == "__main__":
    main()
