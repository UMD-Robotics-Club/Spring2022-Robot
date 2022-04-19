import Jetson.GPIO as GPIO
from motor import drive_train
from motor import motor
import time

direction_pin1 = 23  #Board pin 16 23
direction_pin2 = 24  #Board pin 18 24
speed_pin1 = 32      #Board pin 32 12
speed_pin2 = 33     #Board pin 33 13

motor1 = motor(direction_pin1, speed_pin1)
motor2 = motor(direction_pin2, speed_pin2)

drive_train= drive_train(motor1, motor2)
val = 25
inc = 5
timer = time.time()
while time.time() - timer <= 10:
    time.sleep(0.25)
    if val >= 100:
        incr = -incr
    if val <= 0:
        incr = -incr
    val += incr
    drive_train.set_turn_velocity(val/100)

GPIO.cleanup()
