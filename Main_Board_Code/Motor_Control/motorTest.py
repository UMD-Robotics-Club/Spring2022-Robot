import Jetson.GPIO as GPIO
from motor import drive_train
from motor import motor
import time

direction_pin1 = 16  #Board pin 16 23
direction_pin2 = 18  #Board pin 18 24
speed_pin1 = 32      #Board pin 32 12
speed_pin2 = 33     #Board pin 33 13

motor1 = motor(direction_pin1, speed_pin1, max_accel = .04)
motor2 = motor(direction_pin2, speed_pin2, max_accel = .04)
motor2.invert_dir_pin()

drive_train= drive_train(motor1, motor2)
val = 15
turn_ratio = 0
turn_incr = 0.1
timer = time.time()
print('Going straight')
drive_train.set_turn_velocity(.2)
while time.time() - timer <= 10:
    drive_train.set_turn_velocity(0.2, turn_ratio)
    drive_train.update()

print("Turning one way")
turn_ratio += turn_incr
drive_train.set_turn_velocity(val/100, turn_ratio)
has_turned_neg = False
while time.time() - timer <=10:
    time.sleep(0.25)
    if turn_ratio >= 1:
        turn_incr = -turn_incr
    if turn_ratio <= -1:
        turn_incr = -turn_incr
    turn_ratio += turn_incr
    turn_ratio = round(turn_ratio, 2)
    if (turn_ratio == 0):
        print("Turning the other way")
    drive_train.set_turn_velocity(.15, turn_ratio)
    drive_train.update()

GPIO.cleanup()
