import RPi.GPIO as GPIO
import time
class motor:
    def __init__(self, direction_pin, speed_pin, enable_pin):
        self.velocity = 0
        # define all pins
        self.direction_pin = direction_pin
        self.motor_pin_b = speed_pin
        self.enable_pin = enable_pin
        # Pin Setup:
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        # Set both pins LOW and duty cycle to 0 to keep the motor idle
        GPIO.setup(self.direction_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.speed_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.enable_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(self.direction_pin, GPIO.LOW)

        # set up PWM for speed control
        self.speed = GPIO.PWM(self.speed_pin, 100)
        self.speed.ChangeDutyCycle(abs(self.velocity)*100)

    def set_velocity(self, velocity):
        self.velocity = velocity
        # set direction based on if velocity is positive or negative
        if(velocity < 0):
            GPIO.output(self.direction_pin, GPIO.LOW)
        else:
            GPIO.output(self.direction_pin, GPIO.HIGH)
        # set speed based on velocity
        self.speed.ChangeDutyCycle(abs(self.velocity)*100)

    # takes a true or false value to enable or disable the motor
    def enable(self, on_or_off):
        GPIO.output(self.enable_pin, on_or_off)

    