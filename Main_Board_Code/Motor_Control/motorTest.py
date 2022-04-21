import Jetson.GPIO as GPIO
from motor import drive_train
from motor import motor
import time

print("Starting Up...")

direction_pin1 = 16  #Board pin 16 23
direction_pin2 = 18  #Board pin 18 24
speed_pin1 = 32      #Board pin 32 12
speed_pin2 = 33     #Board pin 33 13

motor1 = motor(direction_pin1, speed_pin1, max_accel = .075)
motor2 = motor(direction_pin2, speed_pin2, max_accel = .075)
motor1.invert_dir_pin(True)
motor2.invert_dir_pin(True)

drive_train= drive_train(motor1, motor2)
val = 15
turn_ratio = 0
turn_incr = 0.1
timer = time.time()
print('Going straight')
drive_train.set_turn_velocity(0.7)
while time.time() - timer <= 3:
    drive_train.set_turn_velocity(0.7)
    drive_train.update()

timer = time.time()
print("Begginning to turn")
for i in range(0,101,2):
    drive_train.set_turn_velocity(0.5, turn_ratio=i/100)
    drive_train.update()
    print(i/100)
    while time.time() - timer < 0.35:
        pass
    timer = time.time()

GPIO.cleanup()
