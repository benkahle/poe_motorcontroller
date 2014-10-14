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
const int encoderRes = 9; // Resolution of encoder light change
int command = 0;
float kp;
float ki;
int commandMode = 0; //0 = step, 1 = sweep
int positionGoal = 0;
int error = 0;
int motorCommand = 0;
long errorSum = 0;
int timeInCurrentPos = 0;
int timeInLastPos = 0;
const int loopDelay = .2;
const float sweepFreq = 1; //Sweeps per second
int loopCounter = 0;
float loopsPerSecond = 1000/loopDelay;
float loopsPerSweepStep = loopsPerSecond/120;

static int sinWave[120] = {
    0x7ff, 0x86a, 0x8d5, 0x93f, 0x9a9, 0xa11, 0xa78, 0xadd, 0xb40, 0xba1,
    0xbff, 0xc5a, 0xcb2, 0xd08, 0xd59, 0xda7, 0xdf1, 0xe36, 0xe77, 0xeb4,
    0xeec, 0xf1f, 0xf4d, 0xf77, 0xf9a, 0xfb9, 0xfd2, 0xfe5, 0xff3, 0xffc,
    0xfff, 0xffc, 0xff3, 0xfe5, 0xfd2, 0xfb9, 0xf9a, 0xf77, 0xf4d, 0xf1f,
    0xeec, 0xeb4, 0xe77, 0xe36, 0xdf1, 0xda7, 0xd59, 0xd08, 0xcb2, 0xc5a,
    0xbff, 0xba1, 0xb40, 0xadd, 0xa78, 0xa11, 0x9a9, 0x93f, 0x8d5, 0x86a,
    0x7ff, 0x794, 0x729, 0x6bf, 0x655, 0x5ed, 0x586, 0x521, 0x4be, 0x45d,
    0x3ff, 0x3a4, 0x34c, 0x2f6, 0x2a5, 0x257, 0x20d, 0x1c8, 0x187, 0x14a,
    0x112, 0xdf, 0xb1, 0x87, 0x64, 0x45, 0x2c, 0x19, 0xb, 0x2,
    0x0, 0x2, 0xb, 0x19, 0x2c, 0x45, 0x64, 0x87, 0xb1, 0xdf,
    0x112, 0x14a, 0x187, 0x1c8, 0x20d, 0x257, 0x2a5, 0x2f6, 0x34c, 0x3a4,
    0x3ff, 0x45d, 0x4be, 0x521, 0x586, 0x5ed, 0x655, 0x6bf, 0x729, 0x794
};

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

void setMotor(float kp, float ki) {
  error = positionGoal - motorPos;
  errorSum += error;
  int pTerm = error+kp;
  int iTerm = errorSum*ki;
  motorCommand = min(pTerm + iTerm, 255);
  powerMotor(motorCommand);
}

void step() {
  kp = 1.2;
  ki = 0.0007;
  setMotor(kp, ki);
}

void sweep(int sweepPos, int command) {
  kp = 165;
  ki = 0.008;
  int waveIndex = sweepPos/loopsPerSweepStep;
  float fractionalPos = (float)sinWave[waveIndex]/4096;
  positionGoal = fractionalPos*command;
  setMotor(kp, ki);
}

void adjustPosition(int encPos) {
  motorPos += encoderRes*motorDir; //incr. or decr. pos based on direction
  encoderPos = encPos;
}

void printInfo() {
  // Serial.println(sensorValue);
  Serial.print(positionGoal);
  Serial.print(",");
  Serial.print(motorPos);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.println(motorCommand);
}

void loop() { 
  // motor->run(FORWARD);
  // motor->setSpeed(22);
  sensorValue = analogRead(sensorPin);
  if (sensorValue < encoderLightCutoff && encoderPos == 1) {
    adjustPosition(0);
  } else if (sensorValue > encoderLightCutoff && encoderPos == 0) {
    adjustPosition(1);
  }
  if (Serial.available() > 0) {
    command = Serial.parseInt();
    errorSum = 0;
    //Positive command = Go to <commmand> degrees
    if (command >= 0) {
      commandMode = 0;
      positionGoal = command;
      //Negative command = Sweep from 0 to <command> degrees
    } else {
      commandMode = 1;
      //Flip command to positive degrees
      command = -command;
    }
  }
  if (!commandMode) {
    step();
  } else {
    sweep(loopCounter, command);
  }
  printInfo();
  loopCounter = ++loopCounter%((int)(121*loopsPerSweepStep));
  delay(loopDelay);
}