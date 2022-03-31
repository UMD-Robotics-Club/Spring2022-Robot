import RPi.GPIO as GPIO
import time
class motor:
    def __init__(self, direction_pin, speed_pin, enable_pin, wheel_diameter=0):
        # define some default parameters
        self.velocity = 0
        self.wheel_diameter = wheel_diameter
        self.is_inverted = False
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
            # the False or self.is_inverted is a fast and fancy way of inverting the direction pin if is_inverted is true
            # Ask an EE major about it
            GPIO.output(self.direction_pin, (False or self.is_inverted))
        else:
            # the True and not self.is_inverted is a fast and fancy way of inverting the direction pin if is_inverted is true
            # Ask an EE major about it
            GPIO.output(self.direction_pin, (True and not self.is_inverted))
        # set speed based on velocity
        self.speed.ChangeDutyCycle(abs(self.velocity)*100)

    # takes a true or false value to enable or disable the motor
    def enable(self, on_or_off):
        GPIO.output(self.enable_pin, on_or_off)
        current_angle = self.imu.get_angle()
        while current_angle > angle+8 or current_angle < angle-8:
            # set the turn velocity based on the difference between the current angle and the angle we want to turn to
            self.set_turn_velocity(self.velocity, (angle-current_angle)/(180*self.turn_ratio))
            # update the current angle
            current_angle = self.imu.get_angle()
        # set the turn velocity to 0 to stop the motors
        self.set_turn_velocity(0)
        return self.imu.get_angle()
    


        
    



    