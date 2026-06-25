import RPi.GPIO as GPIO
import time
import sys

# Motor Driver Pins
RIGHT_EN = 18
RIGHT_IN1 = 17
RIGHT_IN2 = 27
LEFT_EN = 19
LEFT_IN1 = 22
LEFT_IN2 = 23

def test_motors():
    print("Testing Motors Directly (No ROS)...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    pins = [RIGHT_EN, RIGHT_IN1, RIGHT_IN2, LEFT_EN, LEFT_IN1, LEFT_IN2]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
        
    print("Starting PWM on ENA/ENB at 50%...")
    right_pwm = GPIO.PWM(RIGHT_EN, 1000)
    left_pwm = GPIO.PWM(LEFT_EN, 1000)
    right_pwm.start(50)
    left_pwm.start(50)
    
    print("Spinning FORWARD for 3 seconds...")
    GPIO.output(RIGHT_IN1, GPIO.HIGH)
    GPIO.output(RIGHT_IN2, GPIO.LOW)
    GPIO.output(LEFT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN2, GPIO.HIGH)
    time.sleep(3)
    
    print("Stopping.")
    GPIO.output(RIGHT_IN1, GPIO.LOW)
    GPIO.output(LEFT_IN1, GPIO.LOW)
    right_pwm.stop()
    left_pwm.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    test_motors()
