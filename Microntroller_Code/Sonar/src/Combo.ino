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

void setup() {
  servo.attach(servoPin);
  pinMode(triggerpin, OUTPUT);
  pinMode(echopin, INPUT);
  pinMode(9,OUTPUT);
  Serial.begin(9600);
  // init array
  for (int i = 0; i < angleNum; i++) {
    angleDis[i] = 0;
  }
  servo.write(0);
  delay(300);
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
 
  while (Serial.available() > 0 && new_data == false) {
    c = Serial.read();
    if(c == '\n'){*transmission_finished = true;}
    if (*recvInProgress == true) {
      if (c != endMarker) {
        text_data[ndx] = c;
        ndx++;
        if (ndx >= num_chars) {
          ndx = num_chars - 1;
        }
      }
      else {
        text_data[ndx] = '\0'; // terminate the string
        *recvInProgress = false;
        ndx = 0;
        *new_data = true;
      }
    }
    else if (c == startMarker) {
        *recvInProgress = true;
    }
  }
}

//Split the string up by commas
void split(char* str, char* arr[], int num_chars){
  int i = 0;
  int j = 0;
  while (str[i] != '\0') {
    if (str[i] == ',') {
      arr[j] = str + i + 1;
      j++;
    }
    i++;
  }
}

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
  readSerial(&recvInProgress, &new_data, &transmission_finished, text_data, num_chars);
  if(new_data){
    new_data = false;
  }
}
