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

void loop() {
  for (int i = 0; i < angleNum; i++) {
    int angle = i*angleInc;
    servo.write(angle);
    float distance = get_distance();
    Serial.print("Distance :");
    Serial.println(distance);                   
    angleDis[i] = distance;
    delay(50);                       
  }
  for(int i = angleNum-1; i >= 0; i--){
    int angle = i*angleInc;
    servo.write(angle);
    float distance = get_distance();
    Serial.print("Distance :");
    Serial.println(distance);                   
    angleDis[i] = distance;
    delay(50); 
  }
}
