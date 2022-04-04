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
    def send_confirmed_message(self, message, timeout=1, recieve_confirmation='recieved'):
        return_mes = ""
        start_time = time()
        while (return_mes.find(recieve_confirmation) == -1) and (time() - start_time < timeout):
            self.port.write(message)
            return_mes = self.port.readline().decode('utf-8')
        return return_mes

    # send a message and does not wait for a response
    def send_message(self, message):
        self.port.write(message)
        return

    # sends the get sonar command and then automatically processes all of the data and returns it in an array
    def getSonar(self, timeout=1):
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
    
    # sends the getMoisture command and then automatically processes the serial data and return a percentage
    def getMoisture(self, timeout=1):
        self.send_confirmed_message(b"!getMoisture;\n", recieve_confirmation='getMoisture')
        message = ""
        start_time = time()
        # keep looking for the message until a number is returned or the timeout is reached
        while True:
            # exit the function if the timeout is reached
            if time() - start_time > timeout:
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
        return -1
            

        # try to convert the message to a number
        

    # close the serial port. Call this on program exit
    def close(self):
        self.port.close()
        return

tester = robot_serial("COM4")
print(tester.getSonar())
print(tester.getMoisture())
tester.close()
