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
    
    def set_wheel_diameter(self, diameter):
        self.wheel_diameter = diameter
    def get_circumference(self):
        return self.wheel_diameter * 3.14159265359
    def invert_dir_pin(self, is_inverted):
        self.is_inverted = is_inverted

class drive_train:
    # takes two motor objects and an optional IMU object to help with turning
    def __init__(self, motor1, motor2, imu = None, initial_angle = 0):
        self.motor1 = motor1
        self.motor2 = motor2
        self.imu = imu
        self.velocity = 0
        self.turn_ratio = 0
        # use the imu angle to set the initial angle, otherwise use an intial angle that can be manually set
        if imu != None:
            # as of 3/30/22 the IMU class does not exist and this is a theoretical function
            self.angle = imu.get_angle()
        else:
            self.angle = initial_angle
    
    # turn ratio is a number between 0 and 1 which controls how much the drive base will
    # turn relative to the velocity of the drive base
    def set_turn_velocity(self, velocity, turn_ratio=0):
        self.velocity = velocity
        self.turn_ratio = turn_ratio
        # set the motor velocities based on the turn ratio and velocity
        if turn_ratio >= 0:
            self.motor1.set_velocity(velocity)
            self.motor2.set_velocity(velocity*(1-2*turn_ratio))
        else:
            self.motor1.set_velocity(velocity*(1-2*turn_ratio))
            self.motor2.set_velocity(velocity)
    
    def turn_to_angle(self, angle):
        if self.imu == None:
            print("No IMU detected, cannot turn to a set angle without an imu initialized\nPlease add an IMU to the class initializer")
            return 1/0 # <-- this is probably a bad idea to get the programmer's attention
        # get the current angle
        current_angle = self.imu.get_angle()
        while current_angle > angle+8 or current_angle < angle-8:
            # set the turn velocity based on the difference between the current angle and the angle we want to turn to
            self.set_turn_velocity(self.velocity, (angle-current_angle)/(180*self.turn_ratio))
            # update the current angle
            current_angle = self.imu.get_angle()
        # set the turn velocity to 0 to stop the motors
        self.set_turn_velocity(0)
        return self.imu.get_angle()
    


        
    



    