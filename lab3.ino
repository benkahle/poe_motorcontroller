#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

int sensorPin = A0;
int sensorValue = 0;
int motorPos = 0;
int motorPosHist[10];
int historyCount = 0;
int motorDir = 1; //1 = forward, -1 = reverse
int encoderPos = 0; //0 = light, 1 = dark
const int encoderLightCutoff = 200;
const int encoderRes = 20; // Resolution of encoder light change
int command = 0;
const int kp = 1;
const int kd = 0;
const int ki = .2;
int commandMode = 0; //0 = step, 1 = sweep
int positionGoal = 0;
int error = 0;
int motorCommand = 0;
int errorSum = 0;
int timeInCurrentPos = 0;
int timeInLastPos = 0;
const int loopDelay = 10;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motor = AFMS.getMotor(1);

void setup() {
  Serial.begin(9600);  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

void powerMotor(int speed) {
  if (speed < 0) {
    motorDir = -1;
    motor->run(BACKWARD);
  } else {
    motorDir = 1;
    motor->run(FORWARD);
  }
  motor->setSpeed(abs(speed));
}

void step() {
  error = positionGoal - motorPos;
  errorSum += error;
  motorCommand = error*kp + errorSum*ki;
  powerMotor(motorCommand);
}

void adjustPosition(int encPos) {
  //timeInLastPos = timeInCurrentPos;
  //timeInCurrentPos = 0;
  motorPos += encoderRes*motorDir; //incr. or decr. pos based on direction
  encoderPos = encPos;
  //motorPosHist[historyCount] = motorPos;
  //historyCount = historyCount++%10;
  //Serial.println(motorPos);
}

void printInfo() {
  Serial.print("Co: ");
  Serial.print(command);
  Serial.print(", Mode: ");
  Serial.print(commandMode);
  Serial.print(", Pos: ");
  Serial.print(motorPos);
  Serial.print(", Goal: ");
  Serial.print(positionGoal);
  Serial.print(", SensorRead: ");
  Serial.println(encoderPos);
  Serial.print(", MoCom: ");
  Serial.println(motorCommand);
}

void loop() { 
  sensorValue = analogRead(sensorPin);
  if (sensorValue < encoderLightCutoff && encoderPos == 1) {
    adjustPosition(0);
  } else if (sensorValue > encoderLightCutoff && encoderPos == 0) {
    adjustPosition(1);
  }
  if (Serial.available() > 0) {
    command = Serial.parseInt();
    errorSum = 0;
    if (command >= 0) {
      commandMode = 0;
      positionGoal = command;
    } else {
      commandMode = 1;
    }
  }
  if (!commandMode) {
    step();
  }
  //timeInCurrentPos += loopDelay;
  printInfo();
  delay(loopDelay);
}