#include <Servo.h>

Servo myservo; 

int pos;
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
  for (pos = 0; pos <= 180; pos += 5) {
    int i = pos/5;
    myservo.write(pos);   
    digitalWrite(triggerpin,LOW); 
    delayMicroseconds(2);
    digitalWrite(triggerpin,HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerpin,HIGH);
    duration=pulseIn(echopin,HIGH);
    x=(duration/2)/29.1;     
    Serial.print("Distance :");
    Serial.print(x);      
    angleDis[i][0] = pos;
    angleDis[i][1] = x;
    delay(250);                       
  }
  for (pos = 180; pos >= 0; pos -= 5) { 
    myservo.write(pos);              
    delay(15);                       
  }
}
