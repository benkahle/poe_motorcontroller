#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
int sensorPin = A0;
int sensorValue = 0;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

void setup() {
  Serial.begin(9600);  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

void loop() {
  uint8_t i;
  myMotor->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    sensorValue = analogRead(sensorPin);
    Serial.println(sensorValue);
    delay(100);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    sensorValue = analogRead(sensorPin);
    Serial.println(sensorValue);  
    delay(100);
  }
}