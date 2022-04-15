"""Holds a class that handles the serial communication with the robot."""
import serial # pip install pyserial to get this module
from time import sleep
from time import time

class robot_serial:
    """This class is used to send and recieve data from the arduino."""

    def __init__(self, serial_port : serial.Serial, baud_rate : int = 115200):
        """Give a path to the COM port and a baud rate. Default baud rate is 115200."""
        # create a serial port object
        self.port = serial.Serial(serial_port, baud_rate, timeout=0.5)
        message = ""
        # wiat until the arduino initialization message is recieved before doing anything else
        while message.find("Startup Complete") == -1:
            message = self.port.readline().decode('utf-8')

        return

    # will keep sending the serial command until it recieves a response or the timeout is reached
    # will return true if a response was recieved, false if it reached the timeout
    def send_confirmed_message(self, message : str, timeout : float = 2.5, recieve_confirmation : str ='recieved') -> str:
        """Send a serial message and wait for a confirmation message.

        Will keep sending the serial command until it recieves a response or the timeout is reached
        and will return true if a response was recieved, false if it reached the timeout
        recieve_confirmation is the message it is looking for to know that the message was recieved
        """   
        return_mes = ""
        start_time = time()
        while (return_mes.find(recieve_confirmation) == -1) and (time() - start_time < timeout):
            self.port.write(message)
            return_mes = self.port.readline().decode('utf-8')
        return return_mes

    def send_message(self, message : str):
        """Send a message and do not wait for a response."""
        self.port.write(message)
        return ""

    def getSonar(self, timeout : float = 1.0) -> list:
        """Send the getSonar command and return an array of the distances and angles in cm.
        
        Returns a list of tuples with the first eleement in the tuple being the angle and the second being the distance.
        [(angle0, dist0), (angle1, dist1), ...]
        """
        self.send_confirmed_message(b"!getSonar;\n", recieve_confirmation='getSonar')
        message = ""
        start_time = time()
        # keep looking for the message
        while len(message) < 3:
            message = self.port.readline().decode('utf-8')
            if time() - start_time > timeout:
                return None
        # splits the string up by commas
        message_array = message.split(',')
        dist_array = []
        # convert the string to an array of tuples which contain a=the angle and the disance
        for i in range(0, len(message_array)-1,2):
            # check to see if it's a number before trying to convert it to a number
            if message_array[i].isnumeric() and message_array[i+1][0].isnumeric():
                try:
                    dist_array.append((float(message_array[i]), float(message_array[i+1])))
                except:
                    print("Error converting to float while reading sonar array")
        return dist_array
    
    def getMoisture(self, timeout : float = 1) -> float:
        """Send the getMoisture command and then automatically processes the serial data and return a percentage."""
        self.send_confirmed_message(b"!getMoisture;\n", recieve_confirmation='getMoisture')
        message = ""
        start_time = time()
        # keep looking for the message until a number is returned or the timeout is reached
        while True:
            # exit the function if the timeout is reached
            if time() - start_time > timeout:
                print("Measurement timed out")
                return -1
            # check to see if a message was read
            if len(message) >= 3:
                # check to see if the message is a number. THis has to be done after the first check to avoid errors
                if message[0].isnumeric():
                    try:
                        return float(message)
                    except:
                        print("Error converting to float while reading moisture.\nMessage: ", message)
                        return -1
            message = self.port.readline().decode('utf-8')
        
    def setMotor(self, left_speed : float, right_speed : float) -> str:
        """Set the motor speeds."""
        message = "!setMotor," + str(left_speed) + "," + str(right_speed) + ";\n"
        return self.send_confirmed_message(bytes(message, 'utf-8'), recieve_confirmation='setting motor')
    
    def close(self):
        """Close the serial port. Call this on program exit."""
        self.port.close()



'''
tester = robot_serial("COM4")

from random import random

# test setMotor response time
total_time = 0
min = 100
max = 0
for i in range(0,30):
    _, timer = tester.setMotor(round(random(),4), round(random(),4))
    if timer < min:
        min = timer
    if timer > max:
        max = timer
    total_time += timer
    print(timer, "Return Message:", _)
print("setMotor Average:", total_time/30)
print("setMotor Min:", min)
print("setMotor Max:", max)


#test getSonar response time
total_time = 0
min = 100
max = 0
for i in range(0,30):
    _, timer = tester.getSonar()
    if timer < min:
        min = timer
    if timer > max:
        max = timer
    total_time += timer
    print(timer)
print("getSonar Average: ", total_time/30)
print("getSonar Min: ", min)
print("getSonar Max: ", max)

# test getMoisture response time
total_time = 0
min = 100
max = 0
for i in range(0,30):
    _, timer = tester.getMoisture()
    if timer < min:
        min = timer
    if timer > max:
        max = timer
    total_time += timer
    print(timer)
print("getMoisture Average: ", total_time/30)
print("getMoisture Min: ", min)
print("getMoisture Max: ", max)
'''
#tester.close()


