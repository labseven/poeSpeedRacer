// TestPID motor control loop for PoE lab 3

#include "Arduino.h"
#include <Adafruit_MotorShield.h>

const int readDelay = 10;
const float sensorDiffToLinePos = 0.1;
const float kp = 1, ki = 0.001, kd = 100;

const int numLeds = 3;
const int ledPins = {4, 5, 6};
const int sensorPin = A0;
const int motorPins = {1, 2}; //Not on the Arduino, on the shield!
const int buttonPin = A2; //Start button

int sensorValues[3];
float linePos, lastPos, correction, integral = 0; //Higher values indicate further to the right
long unsigned int lastTime, loopTime = 0; //0 to indicate first loop (i.e. don't set anything)

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(motorPins[0]);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(motorPins[1]);

void setup() {
  Serial.begin(9600);
  Serial.println("Begin");

  for (int i=0; i<3; i++} pinMode(ledPins[i], OUTPUT);
  pinMode(sensorPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  AFMS.begin(); //Initialize the motor shield

  while (digitalRead(buttonPin) == HIGH) delay(10); //Wait for button press

}

void loop() {

  //First, read the sensors
  for(int i=0; i < numLeds; i++){
    digitalWrite(ledPins[i], HIGH);
    delay(readDelay);
    sensorValues[i] = analogRead(sensorPin);
    digitalWrite(ledPins[i], LOW);
  }

  //Calculate line position based on sensor readings
  linePos = (sensorValues[numLeds-1] - sensorValues[0]) * sensorDiffToLinePos; //This should be tuned!

  //Do the PID thing
  integral += linePos;
  correction = (kp * linePos) + (ki * integral) + (kd * (linePos-lastPos));
  lastPos = linePos;
  if (correction > 255) correction = 255; //Clamp to usable values
  if (correction < -255) correction = -255;

  //Set the motor speeds
  rightMotor->setSpeed(correction < 0 ? 255 : 255-correction);
  leftMotor->setSpeed(correction > 0 ? 255 : 255+correction);

  //And tell them to run
  rightMotor->run(FORWARD);
  leftMotor->run(FORWARD);

}
