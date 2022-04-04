import serial
from time import sleep
from time import time

class robot_serial:
    # give it the path to the serial port and set a baud rate. (Default is 115200)
    def __init__(self, serial_port, baud_rate=115200):
        # create a serial port object
        self.port = serial.Serial(serial_port, baud_rate, timeout=0.5)
        message = ""
        # wiat until the arduino initialization message is recieved before doing anything else
        while message.find("Startup Complete") == -1:
            message = self.port.readline().decode('utf-8')

        return

    # will keep sending the serial command until it recieves a response or the timeout is reached
    # will return true if a response was recieved, false if it reached the timeout
    def send_confirmed_message(self, message, timeout=1):
        return_mes = ""
        start_time = time()
        while (return_mes.find('recieved') == -1) and (time() - start_time < timeout):
            self.port.write(message)
            return_mes = self.port.readline().decode('utf-8')
            print(return_mes)
        return True

    # send a message and does not wait for a response
    def send_message(self, message):
        self.port.write(message)
        return

    def getSonar(self, timeout=1):
        self.send_confirmed_message(b"!getSonar;\n")
        message = ""
        start_time = time()
        # keep looking for the message
        while len(message) < 3 and (time() - start_time < timeout):
            message = self.port.readline().decode('utf-8')

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
    
    def close(self):
        self.port.close()
        return

tester = robot_serial("COM4")
print(tester.getSonar())
tester.close()
