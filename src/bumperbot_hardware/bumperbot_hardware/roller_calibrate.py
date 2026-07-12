#!/usr/bin/env python3
"""
roller_calibrate.py
-------------------
INTERACTIVE roller calibration — set the UP and DOWN positions with your own
eyes using the keyboard. No angle guessing.

  u  = nudge roller one way  (+2 deg)
  d  = nudge roller other way (-2 deg)
  1  = save current position as UP   (not touching the surface)
  2  = save current position as DOWN (touching the surface)
  t  = TEST the cycle: UP -> DOWN -> wait 5 s -> UP
  q  = quit and print the final UP/DOWN numbers

Needs the pigpio daemon:  sudo pigpiod

Run:
  ros2 run bumperbot_hardware roller_calibrate
"""

import sys
import time
import termios
import tty

import pigpio

from bumperbot_hardware.parameters import SERVO_PIN

MIN_US = 600
MAX_US = 2500
STEP = 2.0          # degrees per keypress
HOLD_SEC = 5.0      # seconds down during the test cycle
SMOOTH_SEC = 1.5    # seconds per smooth move in the test


def angle_to_us(a):
    a = max(0.0, min(180.0, a))
    return int(MIN_US + (a / 180.0) * (MAX_US - MIN_US))


def us_to_angle(us):
    return (us - MIN_US) / (MAX_US - MIN_US) * 180.0


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio daemon is NOT running. Run first:  sudo pigpiod")
        return

    # Start from wherever the servo was last held (or 60 if unknown)
    cur_us = pi.get_servo_pulsewidth(SERVO_PIN)
    angle = us_to_angle(cur_us) if cur_us > 0 else 60.0
    pi.set_servo_pulsewidth(SERVO_PIN, angle_to_us(angle))

    up_angle = None
    down_angle = None

    def apply(a):
        pi.set_servo_pulsewidth(SERVO_PIN, angle_to_us(a))

    def move_smooth(from_a, to_a):
        steps = 30
        for i in range(steps + 1):
            apply(from_a + (to_a - from_a) * (i / steps))
            time.sleep(SMOOTH_SEC / steps)

    print(__doc__)
    print(f"Current angle: {angle:.0f}")

    while True:
        ch = getch()
        if ch == "u":
            angle = min(180.0, angle + STEP)
            apply(angle)
            print(f"angle = {angle:.0f}")
        elif ch == "d":
            angle = max(0.0, angle - STEP)
            apply(angle)
            print(f"angle = {angle:.0f}")
        elif ch == "1":
            up_angle = angle
            print(f">>> UP saved = {up_angle:.0f} deg (roller NOT touching)")
        elif ch == "2":
            down_angle = angle
            print(f">>> DOWN saved = {down_angle:.0f} deg (roller touching surface)")
        elif ch == "t":
            if up_angle is None or down_angle is None:
                print("Save both positions first: press 1 at UP, 2 at DOWN.")
                continue
            print(f"TEST: UP({up_angle:.0f}) -> DOWN({down_angle:.0f}) "
                  f"-> wait {HOLD_SEC:.0f}s -> UP({up_angle:.0f})")
            move_smooth(angle, up_angle)
            time.sleep(1.0)
            move_smooth(up_angle, down_angle)
            print("  ... down, waiting ...")
            time.sleep(HOLD_SEC)
            move_smooth(down_angle, up_angle)
            angle = up_angle
            print("  ... back UP. Test done.")
        elif ch in ("q", "\x03"):   # q or Ctrl+C
            break

    print("=" * 50)
    if up_angle is not None and down_angle is not None:
        print(f"  FINAL:  UP = {up_angle:.0f} deg   DOWN = {down_angle:.0f} deg")
        print("  Send these two numbers to Claude to lock them in.")
    else:
        print("  Positions not saved (press 1 and 2 next time).")
    print("=" * 50)
    # Keep holding the current position (do not release) so the roller stays put
    pi.stop()


if __name__ == "__main__":
    main()
