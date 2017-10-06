// TestPID motor control loop for PoE lab 3

#include "Arduino.h"
#include <Adafruit_MotorShield.h>

const int ledPins = {4, 5, 6};
const int sensorPin = A0;
const int motorPins = {1, 2}; //Not on the Arduino, on the shield!

const float kp = 1, ki = 0.001, kd = 100;

float linePos, lastPos, integral = 0;
long unsigned int lastTime, loopTime = 0; //0 to indicate first loop (i.e. don't set anything)

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(motorPin[0]);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(motorPin[1]);

void setup() {

  for {int i=0; i<3; i++} pinMode(ledPins[i], OUTPUT);
  pinMode(sensorPin, INPUT);

  AFMS.begin();

}

void loop() {

  myMotor->setSpeed(255);
  myMotor->run(FORWARD);

}
