"""This is the main code for the robot. It contains the main loop and navigation logic."""
from Motor_Control.motor import drive_train
#from Serial.robot_serial import robot_serial as Serial
from Image_Recognition.num_recog import Camera as Cam
import Motor_Control.motor as MC
import cv2 as cv
from time import time

# create a camera object
# quinn's desktop and laptop path:
#tes_path = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'
#jetson's path
tes_path = r'/home/robotics/tesseract-4.1.1/src/api/tesseract'
vid = Cam(tes_path)
# initialize serial
#ser = Serial('COM4')
# initialize the motor objects 
motor1 = MC.motor(16, 32, max_accel=0.15) 
motor2 = MC.motor(18, 33, max_accel=0.15) 
motor1.invert_dir_pin(True)
motor2.invert_dir_pin(True)
# create a drivetrain controller object
dr_train = MC.drive_train(motor1, motor2)


def sleep(sleep_time : float):
    """Sleep for a given amount of time in seconds."""
    start_time = time()
    while time() - start_time < sleep_time:
        pass
    return

class Target:
    """This class contains information about the currently tracked target and functions to keep track of the target."""
    
    def __init__(self, x : int, y : int):
        """Initialize with current x and y value of target.
        
        Use 0,0 if you don't know the current position of the target.
        """
        self.x = x
        self.y = y
        self.current_area = 0
        self.past_area = 0
        self.velx = 0
        self.vely = 0
        self.velArea = 0
        self.timer = time()
        self.guesses = {
            "1": 0,
            "2": 0,
            "3": 0,
            "4": 0,
            "5": 0
        }
        # keeps track of the checkpoints that have been reached and measured already
        self.reached_targets = [False, False, False, False, False]
        return

    # gets the velocity of the target. This function needs to be called often for accurate measurements
    def calc_vel(self, new_x : int, new_y : int, new_area : int):
        """Give the new x, y, and area of the target and it calculate the velocity of the target. Call this function often to maintain accuracy.
        
        This function also updates the target's x and y values.
        """
        self.past_area = self.current_area
        self.current_area = new_area
        new_time = time()
        self.velx = (new_x - self.x) / (new_time - self.timer)
        self.vely = (new_y - self.y) / (new_time - self.timer)
        self.velArea = (self.current_area - self.past_area) / (new_time - self.timer)
        self.x = new_x
        self.y = new_y
        self.timer = new_time
        return

    def infer_target(self, ocr_text : str) -> int:
        """Do statistical analysis on a list of guesses to choose the guess with the highest probability of being correct.

        Give a string of the current target's identity and it will add it to the list of guesses.
        Once you give it at least 5 guesses it will try and return the guess with over 75% probability of being correct.
        If no guess is above 75% probability or if there are not at least 5 guesses it will return 0.
        """
        # check to make sure the input is a valid guess
        if ocr_text not in self.guesses: return 0
        # add the guess to the list
        self.guesses[ocr_text] += 1
        # keeps track of the total number of guesses
        num_guesses = 0
        # keeps track of what is the current guess with the highest probability
        highest_prob = 0
        # keeps track of what is the current guess with the highest probability is
        highest_prob_num = 0
        # loop through the dictionary of guesses and find the one with highest probability
        for key in self.guesses:
            num_guesses += self.guesses[key]
            if self.guesses[key] > highest_prob:
                highest_prob = self.guesses[key]
                highest_prob_num = key

        # detect that there is an error if there have been 100 guesses and no right answer yet
        if num_guesses > 100:
            return -1
        # return the highest probability guess if there are at least 5 guesses and the probability is above 75%
        elif highest_prob/num_guesses > 0.75 and num_guesses > 5:
            self.clear_past_guesses()
            return highest_prob_num
        else:
            return 0

    def clear_past_guesses(self):
        """Clear the past guesses so that the next time you call infer_target it will start with a new list of guesses."""
        self.guesses = []
        return

# get the x and y center of the video frame from the camera
frame_center = vid.get_frame().shape
frame_center = (frame_center[1] / 2, frame_center[0] / 2)

# these are some quick config options which can control extra functionality of the code
is_using_motor_serial = False # if you want to use the arduino for motor control enable this
show_im = True # if you want the camera's current view to be displayed on screen, enable this

# These variables automatically scale with the resolution of the camera
min_area_thresh = int((vid.get_frame().shape[0]*vid.get_frame().shape[1])*2000/(1920*1080)) # the minimum area of a target to be considered a target
read_area_thresh = int((vid.get_frame().shape[0]*vid.get_frame().shape[1])*60000/(1920*1080)) # the minimum area of a target to try and read the number from the image


# these variables keep track of what state the robot is in
is_looking_for_checkpoint = True
is_moving_towards_target = False
has_temporarily_lost_target = False
is_confirming_target = False
has_reached_checkpoint = False
has_started_timer = False

# set up the target object
target = Target(0, 0)

# initialize controller data
last_time = time()
current_time = time()
prop_error = 0
vel_error = 0
kp, kd = 1, 0.1
# the speed of the robot is currently just kept constant, but should be proportional to the area of the yellow blob
velocity = 0.42 # this is generally the max speed the robot will travel at
turn_controller = 0 # this is the controller for the robot's turning, it is set to 0 initially

checkpoint_data = [] # this holds all of the data that the robot has gathered from measurements
print("Beggining search for checkpoint")
while True:
    dr_train.update()
    unprocessed_frame = vid.get_frame()
    yellow_frame = vid.find_yellow(unprocessed_frame)
    __, coords = vid.crop_image(yellow_frame.copy())
    target.current_area = coords[2] * coords[3]
    # update the PD controller with new measurements
    current_time = time()
    cent_x = coords[0] + coords[2] / 2
    cent_y = coords[1] + coords[3] / 2
    # only update PID if the target is big enough
    if target.current_area > min_area_thresh:
        
        # start moving towards the target to confirm its identity and keep the target centered
        # calculate proportional error
        prop_error = frame_center[0] - cent_x
        # calculate velocity error
        vel_error = ((frame_center[0] - cent_x) -
                     (frame_center[0] - target.x)) / (current_time - last_time)
        # this controller is designed to keep the target centered and controls how much the robot turns
        turn_controller = kp * prop_error + kd * vel_error
        # normalize the controller value to be between -1 and 1
        turn_controller /= 10*(2*frame_center[0]*kp*kd) #TODO: Consider removing the kp*kd term
        if turn_controller > 1:
            turn_controller = 1
        elif turn_controller < -1:
            turn_controller = -1
        # this tracks the target's position, velocity, area, and change in area over time
        target.calc_vel(cent_x, cent_y, target.current_area)

    # this block can be toggled on and off if you want a visual of what the camera is seeing
    if show_im:
        display_image = yellow_frame
        display_image = cv.circle(display_image, ((int)(cent_x), (int)(cent_y)), radius=5, color=(0, 0, 255), thickness=3)
        cv.rectangle(display_image, (coords[0], coords[1]), (
            coords[0] + coords[2], coords[1] + coords[3]), (0, 255, 0), 2)
        cv.imshow('frame', display_image)
        # press q to stop the program (nothing else will work)
        if cv.waitKey(1) == ord('q'):
            dr_train.cleanup()
            break   

    if has_temporarily_lost_target:
        # start a timer
        if not has_started_timer:
            finding_timeout = time()
            has_started_timer = True
        # if the target has been lost for more than 10 seconds, stop looking for it
        if time() - finding_timeout > 10:
            print("Could not find target in time. Going back to generic searching")
            has_temporarily_lost_target = False
            is_looking_for_checkpoint = True
            has_started_timer = False
            target.current_area = 0

        # use the last known target velocity and turn in that direction
        if target.current_area < min_area_thresh:
            if target.velx != 0:
                turn_direction = -target.velx/abs(target.velx)
            else:
                turn_direction = 1
            dr_train.set_turn_velocity(velocity, turn_ratio=turn_direction)
        else:
            # stop moving and go to looking for checkpoint state
            print("Target relocated, resuming movement...")
            dr_train.set_turn_velocity(0)
            has_temporarily_lost_target = False
            is_looking_for_checkpoint = True
            has_started_timer = False

    # this just makes the robot turn in a circle until it sees yellow
    if is_looking_for_checkpoint:
        if target.current_area > min_area_thresh:
            is_looking_for_checkpoint = False
            is_moving_towards_target = True
            dr_train.set_turn_velocity(0, turn_ratio=0)

            print("Potential Target Found, Moving towards target.")
        else:
            # begin turning the robot until a target is found
            dr_train.set_turn_velocity(0.4, turn_ratio=1)

    # this makes the robot move towards the largest blob of yellow
    if is_moving_towards_target:
        if coords[2]*coords[3] < 90:
            print("Target is lost, attempting to find it again.")
            has_temporarily_lost_target = True
            is_moving_towards_target = False
            dr_train.set_turn_velocity(0)

        dr_train.set_turn_velocity(velocity, turn_ratio=turn_controller)
        last_time = current_time
        #print("Moving towards target to confirm identity.", coords[2]*coords[3], turn_controller)
        # run image recognition if the target area is big enough
        if target.current_area > read_area_thresh:
            dr_train.set_turn_velocity(0)
            is_moving_towards_target = False
            is_confirming_target = True
            target.clear_past_guesses()
            print("Confirming identity of target...")

    if is_confirming_target:
        if coords[2]*coords[3] < 90:
            print("Target is lost, attempting to find it again.")
            has_temporarily_lost_target = True
            is_confirming_target = False
        text, im = vid.get_im_text()
        target_num = target.infer_target(text)
        
        if target_num == -1:
            print("Couldn't confirm target identity, moving on.")
            is_confirming_target = False
            is_looking_for_checkpoint = True
        elif target_num != 0:
            print("Found target with number:", target_num)
            # check to see if it has already visited this target
            if not target.reached_targets[target_num-1]:
                # add the new target to the list of found targets
                target.reached_targets[target_num-1] = True
                is_confirming_target = False
                has_reached_checkpoint = True
            else:
                print("This target has already been visited.")
                is_confirming_target = False
                is_looking_for_checkpoint = True
                # turn the robot away from this checkpoint so it doesn't refind it while searching
                dr_train.set_turn_velocity(0.4, turn_ratio=1)
                #sleep(1)

    if has_reached_checkpoint:
        print("Getting moisture measurements...")
        #sonar_data = ser.getSonar()
        '''shortest_dist = 10000
        for measurement in sonar_data:
            if measurement[1] < shortest_dist:
                shortest_dist = measurement[1]
        checkpoint_data.append((ser.getMoisture(timeout=20), shortest_dist))'''
        print("Moisture measurements taken:", checkpoint_data)
        has_reached_checkpoint = False
        is_looking_for_checkpoint = True
        target.current_area = 0
        print("looking for new checkpoint")


# clean up everything on exit
vid.close_video()
dr_train.cleanup()
