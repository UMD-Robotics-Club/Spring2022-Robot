from Serial.robot_serial import robot_serial as Serial
from Image_Recognition.num_recog import Camera as Cam
from IMU.imu import imu
#import Motor_Control.motor as MC
import cv2 as cv
from time import time

# create a camera object
vid = Cam()
# initialize serial
ser = Serial('COM5')
# initialize the motor objects TODO: Make sure these pinds are correct
#motor1 = MC.motor(31, 32, 29)
#motor2 = MC.motor(35, 33, 36)
# create a drivetrain controller object
#dr_train = MC.drive_train(motor1, motor2)


class Target:
    def __init__(self, x, y, target_num = 1):
        self.x = x
        self.y = y
        self.target_area = 0
        self.velx = 0
        self.vely = 0
        self.timer = time()
        self.current_target = target_num
        return
    
    # gets the velocity of the target. This function needs to be called often for accurate measurements
    def calc_vel(self, new_x, new_y, new_area):
        new_time = time()
        self.velx = (new_x - self.x) / (new_time - self.timer)
        self.vely = (new_y - self.y) / (new_time - self.timer)
        self.velArea = (new_area - self.target_area) / (new_time - self.timer)
        self.x = new_x
        self.y = new_y
        self.target_area = new_area
        self.time = new_time
        return

    # allows you to change which target check point you're looking for
    def set_current_target(self, target_num):
        self.current_target = target_num

is_using_motor_serial = True
show_im = True

# these variables keep track of what state the robot is in
is_looking_for_checkpoint = True
is_confirming_checkpoint_identity = False
is_moving_toward_checkpoint = False
has_reached_checkpoint = False

# set up the target object
target = Target(0, 0)
# get the x and y dimensions of the video frame from the camera
frame_dimensions = vid.get_frame().shape
frame_dimensions = (frame_dimensions[0], frame_dimensions[1])

# stuff to impliment deriviative control
last_time = time()
while True:
    yellow_frame = vid.get_frame()
    _, coords = vid.crop_image(yellow_frame)
    if show_im:
        display_image = yellow_frame.copy()
        display_image = cv.circle(display_image, ((int)(coords[0]+coords[2]/2),(int)(coords[1]+coords[3]/2)), radius=5, color=(0,0,255), thickness=3)
        cv.rectangle(display_image, (coords[0], coords[1]), (coords[0] + coords[2], coords[1] + coords[3]), (0, 255, 0), 2)
        cv.imshow('frame', display_image)
        # press q to stop the program (nothing else will work)
        if cv.waitKey(1) == ord('q'):
            break
    if is_looking_for_checkpoint:
        if target.target_area > 100:
            is_looking_for_checkpoint = False
            is_confirming_checkpoint_identity = True
            print("Potential Target Found, Moving towards target.")
        else:
            pass #TODO get rid of this
            # begin turning the robot until a target is found
            #ser.setMotor(0.3, -0.3) if is_using_motor_serial else dr_train.set_turn_velocity(0.3, 1) #TODO uncomment this
            
    
    if is_confirming_checkpoint_identity:
        kp, kd = 1, 1
        current_time = time()
        # start moving towards the target to confirm its identity and keep the target centered
        # calculate proportional error
        prop_error = frame_dimensions[0] / 2 - coords[0]
        # calculate velocity error
        deriv_error = ((frame_dimensions[0] - coords[0]) - (frame_dimensions[0] - target.x)) / (current_time - last_time)
        controller = kp * prop_error + kd * deriv_error
        velocity = 0.37
        turn_ratio = controller / (2*frame_dimensions[0]*kp*kd) # normalize the controller value to be between -1 and 1
        if turn_ratio >= 0:
            motor1 = velocity
            motor2 = velocity*(1-2*turn_ratio)
        else:
            motor1 = velocity*(1-2*turn_ratio)
            motor2 = velocity
        #ser.setMotor(motor1,motor2) if is_using_motor_serial else dr_train.set_turn_velocity(velocity, turn_ratio) #TODO uncomment this
        last_time = current_time
        print("Moving towards target to confirm identity.", velocity, turn_ratio)

    if is_moving_toward_checkpoint:
        pass

    if has_reached_checkpoint:
        pass

    # this tracks the target's position, velocity, area, and change in area over time
    target.calc_vel((coords[0] + coords[2] / 2), (coords[1] + coords[3] / 2), (coords[2] * coords[3]))


# clean up everything on exit
vid.close_video()
ser.close()

