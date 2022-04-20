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
val = 50
inc = 5
turn_ratio = 0
turn_incr = 0.1
timer = time.time()
print('Going straight')
while time.time() - timer <= 10:
    time.sleep(0.25)
    if val >= 100:      #2.5 to first max out
        incr = -incr
    if val <= 0:        #7.5 to first stop
        incr = -incr
    val += incr
    drive_train.set_turn_velocity(val/100, turn_ratio)
print("Turning one way")
turn_ratio += turn_incr
drive_train.set_turn_velocity(val/100, turn_ratio)
while time.time() - timer <=20 & time.time() - timer >=10:
    time.sleep(0.25)
    if turn_ratio >= 1:      
        turn_incr = -turn_incr
    if turn_ratio <= -1:        
        turn_incr = -turn_incr
    turn_ratio += turn_incr
    if (turn_ratio == 0):
        print("Turning the other way")
    drive_train.set_turn_velocity(val/100, turn_ratio)

GPIO.cleanup()
