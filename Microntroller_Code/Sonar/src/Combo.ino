// Use link below for Sonar theory and equation explanation
// https://create.arduino.cc/projecthub/abdularbi17/ultrasonic-sensor-hc-sr04-with-arduino-tutorial-327ff6
#include <Servo.h>
#include <Arduino.h>

Servo servo; 

#define servoPin 5
#define triggerpin 10
#define echopin 11

int position;
long duration;
long distance;
long x;
// array that stores the distance at each servo position
// The (index value)*5 will give you the angle that the measuremnt was taken out. The value at that index will be the distance
const int angleInc = 5;
const int angleNum = 180/angleInc;
float angleDis[angleNum];

// serial variables to read and process serial
bool recvInProgress = false;
bool new_data = false;
bool transmission_finished = false;
const int num_chars = 100;
char text_data[num_chars] = "";
// this array contains the parsed command and all of its arguments. index of 0 is the command, and the rest are the arguments
String command[10] = {"this", "is", "an", "example", "of", "a", "command", "", "", ""};

void setup() {
  servo.attach(servoPin);
  pinMode(triggerpin, OUTPUT);
  pinMode(echopin, INPUT);
  pinMode(9, OUTPUT);
  Serial.begin(9600);
  // init array
  for (int i = 0; i < angleNum; i++) {
    angleDis[i] = 0;
  }
  servo.write(0);
  delay(300);
  Serial.println("Startup Complete");
}
// gets the distance from a sonar sensor in centimeters
float get_distance(){
  // get the distance for a sonar sensor
  float distance = 0;
  digitalWrite(triggerpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerpin, LOW);
  unsigned long duration = pulseIn(echopin, HIGH);
  distance = ((float)(duration)/2) / 29.1;
  digitalWrite(triggerpin, LOW);
  return distance;

}

// Read serial data from the computer
void readSerial(bool* recvInProgress, bool* new_data, bool* transmission_finished, char text_data[], int num_chars){
  *recvInProgress = false;
  static int ndx = 0;
  char startMarker = '!';
  char endMarker = ';';
  char c;
  
  // read in new serial as long as its available and we're not overwriting unread serial
  while (Serial.available() > 0 && *new_data == false) {
    // Read in a character at a time
    c = Serial.read();
    // check to see if the character is a newline character. End reading if it is
    if(c == '\n'){*transmission_finished = true;}
    // check to see if the recieve is already in progress. If it isn't, check to see if the character read is the start marker
    if (*recvInProgress == true) {
      // if the character recieved isn't the end marker, add it to the text_data array
      if (c != endMarker) {
        text_data[ndx] = c;
        ndx++;
        if (ndx >= num_chars) {
          ndx = num_chars - 1;
        }
      }
      // if the character recieved is the end marker terminate the string and set the flag that new data is available
      else {
        text_data[ndx] = '\0'; // terminate the string
        *recvInProgress = false;
        ndx = 0;
        *new_data = true;
      }
    }
    // check to see if the character read is the start marker
    else if (c == startMarker) {
        *recvInProgress = true;
    }
  }
}

// split the recieved up by commas into the command array
void parseData(char text_data[]){
  // split the data into its parts
  char * indx; // this is used by strtok() as an index
  int i = 0;
  indx = strtok(text_data, ",");      // get the first part - the string
  while(indx != NULL){
    command[i] = indx;
    i++;
    indx = strtok(NULL, ","); // this continues where the previous call left off
  }
}

// arg_length should be 10
void executeCommand(String command[], int arg_length){
  // check to see what the command is
  // test command just prints out all of the parameters to serial
  if(command[0] == "test"){
    Serial.println("test command recieved, printing out command");
    for(int i = 0; i < arg_length; i++){
      Serial.print(command[i]);
      Serial.print(",");
    }
    Serial.println("");
  }
  // send all of the sonar distances and angles in a comma seperated list
  else if(command[0] == "getSonar"){
    Serial.println("getSonar command recieved, sending distance");
    for(int i = 0; i < angleNum; i++){
      Serial.print(i*angleInc);
      Serial.print(",");
      Serial.print(angleDis[i]);
      Serial.print(",");
    }
    Serial.println("");
  }
}

// variables to keep track of the servo's position
bool isGoingClockwise = false;
int angleIndex = 0;

void loop() {
  // using if statements is important to allow the arduino to multitask
  // This is all of the code needed to sweep and read distances
  if(angleIndex == angleNum){
    isGoingClockwise = false;
  }
  if(angleIndex == 0){
    isGoingClockwise = true;
  }
  if(isGoingClockwise){
    angleIndex++;
  }
  else{
    angleIndex--;
  }
  int angle = angleIndex*angleInc;
  servo.write(angle);
  float distance = get_distance();
  angleDis[angleIndex] = distance;

  // read serial data
  if(Serial.available() > 0){
    // read in the serial
    readSerial(&recvInProgress, &new_data, &transmission_finished, text_data, num_chars);
  }
  if(new_data){
    // create a copy of the text data because the parseData function will destroy the string while processing it
    char temp_data[num_chars] = "";
    strcpy(temp_data, text_data);
    // pase the text data
    parseData(temp_data);
    executeCommand(command, 10);
    // set the flag that the new data has been processed
    new_data = false;
  }
}
