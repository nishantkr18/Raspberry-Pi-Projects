import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

# Setup GPIO Pins
GPIO.setup(33, GPIO.OUT)
GPIO.setup(35, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(36, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)

 
# Set PWM instance and their frequency
pwm33 = GPIO.PWM(33, 60)
pwm35 = GPIO.PWM(35, 60)
GPIO.output(12, False)
GPIO.output(32, True)
GPIO.output(36, True)
GPIO.output(38, False)
 
# Start PWM with 50% Duty Cycle
pwm33.start(100)
pwm35.start(100)

time.sleep(02)

pwm33.stop()
pwm35.stop()
# Cleans the GPIO
GPIO.cleanup()
