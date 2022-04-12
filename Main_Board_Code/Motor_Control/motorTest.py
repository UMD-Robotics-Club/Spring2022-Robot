from re import M
from Main_Board_Code.Motor_Control.motor import drive_train
import motor
import time

direction_pin1 = 23  #Board pin 16
direction_pin2 = 24  #Board pin 18
speed_pin1 = 12      #Board pin 32
speed_pin2 = 13     #Board pin 33
enable_pin1 = 8     #Board pin 24
enable_pin2 = 7     #Board pin 26

motor1 = motor(direction_pin1, speed_pin1, enable_pin1)
motor2 = motor(direction_pin2, speed_pin2, enable_pin2)

drive_train= drive_train(motor1, motor2)
drive_train.set_turn_velocity(0.5)
time.sleep(3)
drive_train.set_turn_velocity(0.3, turn_ratio=0.5)
time.sleep(3)
drive_train.set_turn_velocity(0.3, turn_ratio=-0.5)
time.sleep(3)
drive_train.set_turn_velocity(0)
