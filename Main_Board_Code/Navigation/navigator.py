from ..Serial.robot_serial import RobotSerial as Serial
from ..Image_Recognition.num_recog import Camera as Cam
from ..IMU.imu import imu

# create a camera object
vid = cam()
# initialize serial
ser = Serial('COM3')

def search_checkpoints():
    # get camera data
    return
