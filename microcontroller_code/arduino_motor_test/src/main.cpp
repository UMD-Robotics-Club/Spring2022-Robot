#include <Arduino.h>

#define motor1_speed_pin 2
#define motor1_dir_pin 3
#define motor2_speed_pin 4
#define motor2_dir_pin 5
#define enable_pin 6
#define random_pin 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // set up all of the motor pins
  pinMode(motor1_speed_pin, OUTPUT);
  pinMode(motor1_dir_pin, OUTPUT);
  pinMode(motor2_speed_pin, OUTPUT);
  pinMode(motor2_dir_pin, OUTPUT);
  pinMode(enable_pin, INPUT_PULLUP);
  digitalWrite(motor1_dir_pin, LOW);
  digitalWrite(motor2_dir_pin, LOW);
  analogWrite(motor1_speed_pin, 0);
  analogWrite(motor2_speed_pin, 0);

  pinMode(random_pin, OUTPUT);
  digitalWrite(random_pin, HIGH);
  
}

int speed = 1023;
// a function to set the speed of the motors from the serial port


void loop() {
  if(!digitalRead(enable_pin)){
    analogWrite(motor1_speed_pin, 1023);
    analogWrite(motor2_speed_pin, 1023);
    if(speed <= 1){
      speed = 1023;
      digitalWrite(motor1_dir_pin, HIGH);
      digitalWrite(motor2_dir_pin, HIGH);
    }
    speed--;
    delay(2);
    Serial.println(speed);
  }
  else{
    analogWrite(motor1_speed_pin, 0);
    analogWrite(motor2_speed_pin, 0);
  }
  
}