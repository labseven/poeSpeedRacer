// TestPID motor control loop for PoE lab 3

#include "Arduino.h"
#include <Adafruit_MotorShield.h>
#include <Adafruit_MS_PWMServoDriver.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *myMotor = AFMS.getMotor(1); // Select which 'port' M1, M2, M3 or M4. In this case, M1

long unsigned int last_time = 0;
const int direction_delay = 1000;

bool direction = true;

void setup() {
  Serial.begin(9600);
  Serial.println("begin.");

  AFMS.begin();

  Serial.println("AFMS began");
  myMotor->setSpeed(150);
  myMotor->run(FORWARD); //Stop the motor at the beginning

  // myMotor->run(RELEASE);


  Serial.println("Running!");

}

void loop() {
  uint8_t i;

  Serial.print("tick");

  myMotor->run(FORWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    delay(10);
  }

  Serial.print("tock");

  myMotor->run(BACKWARD);
  for (i=0; i<255; i++) {
    myMotor->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    myMotor->setSpeed(i);
    delay(10);
  }

  Serial.println("tech");
  myMotor->run(RELEASE);
  delay(1000);
}
