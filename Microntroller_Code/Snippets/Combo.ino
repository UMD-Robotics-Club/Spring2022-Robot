// Use link below for Sonar theory and equation explanation
// https://create.arduino.cc/projecthub/abdularbi17/ultrasonic-sensor-hc-sr04-with-arduino-tutorial-327ff6
#include <Servo.h>

Servo myservo; 

int position;
int triggerpin=10;
int echopin=11;
long duration;
long distance;
long x;
long angleDis[35][1];

void setup() {
  myservo.attach(9);
  pinMode(triggerpin,OUTPUT);
  pinMode(echopin,INPUT);
  pinMode(9,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  for (position = 0; position <= 180; position += 5) {
    int i = position/5;
    myservo.write(position);   
    digitalWrite(triggerpin,LOW); 
    delayMicroseconds(2);
    digitalWrite(triggerpin,HIGH);
    delayMicroseconds(10);    
    digitalWrite(triggerpin,HIGH);
    duration=pulseIn(echopin,HIGH);
    x=(duration/2)/29.1;               //calculated distance in cm 
    Serial.print("Distance :");
    Serial.print(x);      
    angleDis[i][0] = position;              
    angleDis[i][1] = x;
    delay(250);                       
  }
  for (position = 180; position >= 0; position -= 5) { 
    myservo.write(position);              
    delay(15);                       
  }
}
