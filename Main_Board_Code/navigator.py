"""This is the main code for the robot. It contains the main loop and navigation logic."""
from Serial.robot_serial import robot_serial as Serial
from Image_Recognition.num_recog import Camera as Cam
from IMU.imu import imu
#import Motor_Control.motor as MC
import cv2 as cv
from time import time
from time import sleep

# create a camera object
# quinn's desktop and laptop path:
tes_path = 'C:\\Program Files\\Tesseract-OCR\\tesseract.exe'
vid = Cam(tes_path, cam_num=0)
# initialize serial
ser = Serial('COM5')
# initialize the motor objects TODO: Make sure these pins are correct
#motor1 = MC.motor(31, 32, 29)
#motor2 = MC.motor(35, 33, 36)
# create a drivetrain controller object
#dr_train = MC.drive_train(motor1, motor2)


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
        self.guesses = []
        self.time = time()
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
        self.time = new_time
        return

    # can collect a sample of tesseract guesses and will take the guess that's the highest probability
    def infer_target(self, ocr_text : str) -> str:
        """Do statistical analysis on a list of guesses to choose the guess with the highest probability of being correct.

        Give a string of the current target's identity and it will add it to the list of guesses.
        Once you give it at least 5 guesses it will try and return the guess with over 75% probability of being correct.
        If no guess is above 75% probability or if there are not at least 5 guesses it will return 0.
        """
        # convert the string to a number
        try:
            number = int(ocr_text)
        except ValueError:
            return 0
        # make sure the number is valid or else exit
        if number > 5:
            return 0
        # append the number to the list of guesses that tesseract spit out
        self.guesses.append(number)
        # check to see if the list is long enough to find the highest probability
        if len(self.guesses) >= 5:
            # initialize a percentile array which keeps track of the percentiles of each guess
            percent_occurencs = [0, 0, 0, 0, 0]
            # go through each of the guesses and tally of the percents
            for num in self.guesses:
                percent_occurencs[num-1] += 1/len(self.guesses)
            # find the highest probability guess
            highest_percent, highest_num = 0, 0
            for i in range(len(percent_occurencs)-1):
                if percent_occurencs[i] > highest_percent:
                    highest_percent = percent_occurencs[i]
                    highest_num = i
            # return the highest probability guess as long as it's at least 75% confident
            if highest_percent > 0.75 and highest_num != 0:
                return highest_num+1

        return 0

    def clear_past_guesses(self):
        """Clear the past guesses so that the next time you call infer_target it will start with a new list of guesses."""
        self.guesses = []
        return

# get the x and y dimensions of the video frame from the camera
frame_center = vid.get_frame().shape
frame_center = (frame_center[0] / 2, frame_center[1] / 2)

# these are some quick config options which can control extra functionality of the code
# TODO: use the jetson's GPIO instead of the arduino's for motor control
is_using_motor_serial = True # if you want to use the arduino for motor control enable this
show_im = True # if you want the camer'as current view to be displayed on screen, enable this

#These variables automatically scale with the resolution of the camera
min_area_thresh = int((vid.get_frame().shape[0]*vid.get_frame().shape[1])*500/(1920*1080)) # the minimum area of a target to be considered a target
read_area_thresh = int((vid.get_frame().shape[0]*vid.get_frame().shape[1])*15000/(1920*1080)) # the minimum area of a target to try and read the number from the image


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
kp, kd = 1, 1
# the speed of the robot is currently just kept constant, but should be proportional to the area of the yellow blob
velocity = 0.37
turn_controller = 0

checkpoint_data = []
print("Beggining search for checkpoint")
while True:
    unprocessed_frame = vid.get_frame()
    yellow_frame = vid.find_yellow(unprocessed_frame)
    _, coords = vid.crop_image(yellow_frame.copy())
    target.current_area = coords[2] * coords[3]
    # update the PD controller with new measurements
    current_time = time()
    # only update PID if the target is big enough
    if target.current_area > min_area_thresh:
        cent_x = coords[0] + coords[2] / 2
        cent_y = coords[1] + coords[3] / 2
        # start moving towards the target to confirm its identity and keep the target centered
        # calculate proportional error
        prop_error = frame_center[0] - cent_x
        # calculate velocity error
        vel_error = ((frame_center[0] - cent_x) -
                     (frame_center[0] - target.x)) / (current_time - last_time)
        # this controller is designed to keep the target centered and controls how much the robot turns
        turn_controller = kp * prop_error + kd * vel_error
        # normalize the controller value to be between -1 and 1
        turn_controller /= (2*frame_center[0]*kp*kd)
        if turn_controller > 1:
            turn_controller = 1
        elif turn_controller < -1:
            turn_controller = -1
        # this tracks the target's position, velocity, area, and change in area over time
        target.calc_vel(cent_x, cent_y, target.current_area)

    # this block can be toggled on and off if you want a visual of what the camera is seeing
    if show_im:
        display_image = yellow_frame
        display_image = cv.circle(display_image, ((int)(coords[0]+coords[2]/2), (int)(
            coords[1]+coords[3]/2)), radius=5, color=(0, 0, 255), thickness=3)
        cv.rectangle(display_image, (coords[0], coords[1]), (
            coords[0] + coords[2], coords[1] + coords[3]), (0, 255, 0), 2)
        cv.imshow('frame', display_image)
        # press q to stop the program (nothing else will work)
        if cv.waitKey(1) == ord('q'):
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
                turn_direction = target.velx/abs(target.velx)
            else:
                turn_direction = 1
            # ser.setMotor(0.2*turn_direction, -0.2*turn_direction) if is_using_motor_serial else dr_train.set_turn_velocity(0.2, turn_ratio=turn_direction) #TODO uncomment this
        else:
            # stop moving and go to looking for checkpoint state
            print("Target relocated, resuming movement...")
            # ser.setMotor(0, 0) if is_using_motor_serial else dr_train.set_turn_velocity(0) #TODO uncomment this
            has_temporarily_lost_target = False
            is_looking_for_checkpoint = True
            has_started_timer = False

    # this just makes the robot turn in a circle until it sees yellow
    if is_looking_for_checkpoint:
        if target.current_area > min_area_thresh:
            is_looking_for_checkpoint = False
            is_moving_towards_target = True
            # ser.setMotor(0, 0) if is_using_motor_serial else dr_train.set_turn_velocity(0) #TODO uncomment this

            print("Potential Target Found, Moving towards target.")
        else:
            pass  # TODO get rid of the pass statement and add motor commands here
            # begin turning the robot until a target is found
            # ser.setMotor(0.3, -0.3) if is_using_motor_serial else dr_train.set_turn_velocity(0.3, turn_ratio=1) #TODO uncomment this

    # this makes the robot move towards the largest blob of yellow
    if is_moving_towards_target:
        if coords[2]*coords[3] < 90:
            print("Target is lost, attempting to find it again.")
            has_temporarily_lost_target = True
            is_moving_towards_target = False
            # ser.setMotor(0,0) if is_using_motor_serial else dr_train.set_turn_velocity(0) #TODO uncomment this

        if turn_controller >= 0:
            motor1 = velocity
            motor2 = velocity*(1-2*turn_controller)
        else:
            motor1 = velocity*(1-2*turn_controller)
            motor2 = velocity
        # ser.setMotor(motor1,motor2) if is_using_motor_serial else dr_train.set_turn_velocity(velocity, turn_ratio=turn_ratio) #TODO uncomment this
        last_time = current_time
        #print("Moving towards target to confirm identity.", coords[2]*coords[3], turn_controller)
        # run image recognition if the target area is big enough
        if target.current_area > read_area_thresh:
            # ser.setMotor(0,0) if is_using_motor_serial else dr_train.set_turn_velocity(0) #TODO uncomment this
            is_moving_towards_target = False
            is_confirming_target = True
            target.clear_past_guesses()
            print("Confirming identity of target...")

    if is_confirming_target:
        if coords[2]*coords[3] < 90:
            print("Target is lost, attempting to find it again.")
            has_temporarily_lost_target = True
            is_confirming_target = False
        text, _ = vid.get_im_text()
        target_num = target.infer_target(text)
        if target_num != 0:
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
                # ser.setMotor(0.3, -0.3) if is_using_motor_serial else dr_train.set_turn_velocity(0.3, turn_ratio=1) #TODO uncomment this
                # sleep(2)

    if has_reached_checkpoint:
        print("Getting moisture measurements...")
        sonar_data = ser.getSonar()
        shortest_dist = 10000
        for measurement in sonar_data:
            if measurement[1] < shortest_dist:
                shortest_dist = measurement[1]
        checkpoint_data.append((ser.getMoisture(timeout=20), shortest_dist))
        print("Moisture measurements taken:", checkpoint_data)
        has_reached_checkpoint = False
        is_looking_for_checkpoint = True
        target.current_area = 0
        print("looking for new checkpoint")


# clean up everything on exit
vid.close_video()
ser.close()
