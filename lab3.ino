#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

int sensorPin = A0;

const int encoderLightCutoff = 70; // Value of encoder between "high" and "low"
const int encoderRes = 9; // Degress per encoder band
const int loopDelay = .2; // Time between controller actions
const float sweepFreq = 1; // Sweeps per second (sinusoid following)

int sensorValue = 0;

int motorPos = 0;
int motorDir = 1; // 1 = forward, -1 = reverse
int encoderPos = 0; // 0 = light (high), 1 = dark(low)

int commandMode = 0; // 0 = step, 1 = sweep
int command = 0; // command from serial
int positionGoal = 0; // target degree parsed from serial command
int error = 0;
int motorCommand = 0; // Controller signal

long errorSum = 0;
float kp;
float ki;

int loopCounter = 0;
float loopsPerSecond = 1000/loopDelay;
float loopsPerSweepStep = loopsPerSecond/120; // # loops between sinusoid periods

// Discretized sinusoid
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
}

/*
* Wraps the motor API to allow specifying speed and direction through positive
* and negative integers
*/
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

/*
* Given a proportional control constant (kp) and integral control constant (ki)
* calculates the controller signal to the moter based on current motor position
* and target position
*/
void setMotor(float kp, float ki) {
  error = positionGoal - motorPos;
  errorSum += error;
  int pTerm = error+kp;
  int iTerm = errorSum*ki;
  motorCommand = min(pTerm + iTerm, 255);
  powerMotor(motorCommand);
}

/*
* Set control constants and call for a controller signal based on a step response
*/
void step() {
  kp = 1.2; // Proportional control constant for step response
  ki = 0.0007; // Integral control constant for step response
  setMotor(kp, ki);
}

/*
* Set the control constants and call for a controller signal based on a
* sinusoidal sweep to a certain target position.
*/
void sweep(int sweepPos, int command) {
  kp = 165; // Proportional control constant for sweep response
  ki = 0.008; // Integral control constant for sweep response
  int waveIndex = sweepPos/loopsPerSweepStep; // index into sinusoid period
  float fractionalPos = (float)sinWave[waveIndex]/4096; // sine value at index scaled to fraction of 1
  positionGoal = fractionalPos*command;
  setMotor(kp, ki);
}

/*
* recalculate the motor's position based on a change in encoder value
*/
void adjustPosition(int encPos) {
  motorPos += encoderRes*motorDir; //incr. or decr. pos based on direction
  encoderPos = encPos;
}

void printInfo() {
  Serial.print(positionGoal);
  Serial.print(",");
  Serial.print(motorPos);
  Serial.print(",");
  Serial.print(error);
  Serial.print(",");
  Serial.println(motorCommand);
}

void loop() { 
  sensorValue = analogRead(sensorPin);
  // Check for a change in motor position based on encoder value and last state
  if (sensorValue < encoderLightCutoff && encoderPos == 1) {
    adjustPosition(0);
  } else if (sensorValue > encoderLightCutoff && encoderPos == 0) {
    adjustPosition(1);
  }
  // Check for a command over serial port
  if (Serial.available() > 0) {
    command = Serial.parseInt();
    errorSum = 0; // Reset error sum when a new command is given
    if (command >= 0) { // Positive command = Go to <commmand> degrees
      commandMode = 0; // Step response
      positionGoal = command;
    } else { // Negative command = Sweep from 0 to <command> degrees
      commandMode = 1; // Sweep response
      command = -command; // Flip command to positive degrees
    }
  }
  if (!commandMode) {
    step();
  } else {
    sweep(loopCounter, command);
  }
  printInfo();
  // Increment loop counter and reset to zero after each sinusoid sweep
  loopCounter = ++loopCounter%((int)(121*loopsPerSweepStep));
  delay(loopDelay); // Allow motor to act before updating control
}