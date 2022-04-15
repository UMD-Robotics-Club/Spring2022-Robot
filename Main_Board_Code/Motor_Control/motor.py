import RPi.GPIO as GPIO
import time
class motor:
    """Keeps track of all motor data and has several functions to setup and control the motor."""

    def __init__(self, direction_pin : int, speed_pin : int, enable_pin : int, wheel_diameter=0):
        """Set up all GPIO for motor control."""
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

    def set_velocity(self, velocity : float):
        """Set the velocity of the motor. Can be a value from -1 to 1."""
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

    def enable(self, on_or_off : bool):
        """Enable or disable the motor."""
        GPIO.output(self.enable_pin, on_or_off)
    
    def set_wheel_diameter(self, diameter : float):
        """Set the diameter of the wheel. This can be helpful for distance or angle measurement if encoders are used."""
        self.wheel_diameter = diameter
    def get_circumference(self):
        """Get the circumference of the wheel."""
        return self.wheel_diameter * 3.14159265359
    def invert_dir_pin(self, is_inverted : bool):
        """Invert the direction pin. This is useful if the motor is wired backwards or facing the other way."""
        self.is_inverted = is_inverted

class drive_train:
    """Keeps track of two motors and has a function to drive them in synchronicity."""

    # takes two motor objects and an optional IMU object to help with turning
    def __init__(self, motor1 : motor, motor2 : motor, imu = None, initial_angle = 0.0):
        """Set up the drive train."""
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
    def set_turn_velocity(self, velocity:float, turn_ratio=0.0):
        """Set the velocity of the drive train. Can be a value from -1 to 1.
        
        turn ratio is a number between 0 and 1 which controls how much the drive base will
        turn relative to the velocity of the drive base
        """
        self.velocity = velocity
        self.turn_ratio = turn_ratio
        # set the motor velocities based on the turn ratio and velocity
        if turn_ratio >= 0:
            self.motor1.set_velocity(velocity)
            self.motor2.set_velocity(velocity*(1-2*turn_ratio))
        else:
            self.motor1.set_velocity(velocity*(1-2*turn_ratio))
            self.motor2.set_velocity(velocity)


        
    



    