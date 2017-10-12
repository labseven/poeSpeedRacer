// TestPID motor control loop for PoE lab 3

#include "Arduino.h"
#include <Adafruit_MotorShield.h>
#include "IRLibAll.h"

const int readDelay = 10;
const float sensorDiffToLinePos = 0.1;
const float kp = 100, ki = 0.001, kd = 100;
const int turnSpeedLeft = 255, turnSpeedRight = 0;

const int turnLength = 20; //iterations after releasing button to keep turning
int turnCounter = 0; //How long, in iterations, left in this turn

const int numLeds = 3;
const int ledPins[] = {5, 6, 7};
const int sensorPin = A0;
const int motorPins[] = {1, 2}; //Not on the Arduino, on the shield!
const int buttonPin = A2; //Start button

const int remotePin = 2; //The IR remote receiver pin
const int spareGround = 3; //Ground pin for great good

IRrecvPCI myReceiver(remotePin); //IR remote receiver
IRdecode myDecoder; //IR remote decoder

int sensorValues[3];
float linePos, lastPos, correction, integral = 0; //Higher values indicate further to the right
long unsigned int lastTime, loopTime = 0; //0 to indicate first loop (i.e. don't set anything)

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(motorPins[0]);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(motorPins[1]);

void setup() {
  Serial.begin(9600);
  Serial.println("Begin");

  for (int i=0; i<3; i++) pinMode(ledPins[i], OUTPUT);
  pinMode(sensorPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  AFMS.begin(); //Initialize the motor shield
  myReceiver.enableIRIn(); //Initialize the remote receiver

  pinMode(remotePin, INPUT_PULLUP); //Hopefully this won't mess with the IR library
  pinMode(spareGround, OUTPUT); digitalWrite(spareGround, LOW); //Spare ground pin for IR receiver

  //while (digitalRead(buttonPin) == HIGH) delay(10); //Wait for button press

}

void loop() {

  if (myReceiver.getResults()) { //First, read from the remote, since that can change lots of behavior
    myDecoder.decode();           //Decode it
    //myDecoder.dumpResults(true);  //Now print results -- or don't if we want speed
    myReceiver.enableIRIn();      //Restart receiver
    turnCounter = turnLength;
  } else if (turnCounter > 0) turnCounter--;

  if (turnCounter) { //If we're in a human-triggered sharp turn...
    leftMotor->setSpeed(turnSpeedLeft);
    rightMotor->setSpeed(turnSpeedRight);
  } else { //If we're not in a sharp turn

    //Read the sensors
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
    Serial.println("P: " + String(kp * linePos) + " I: " + (ki * integral) + " D: " + (kd * (linePos-lastPos)));
    lastPos = linePos;
    if (correction > 255) correction = 255; //Clamp to usable values
    if (correction < -255) correction = -255;

    //Set the motor speeds
    rightMotor->setSpeed(correction < 0 ? 255 : 255-correction);
    leftMotor->setSpeed(correction > 0 ? 255 : 255+correction);

    //And tell them to run
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);


  } //end if turning

}
