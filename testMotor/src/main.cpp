// TestPID motor control loop for PoE lab 3

#include "Arduino.h"
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *myMotor = AFMS.getMotor(1); // Select which 'port' M1, M2, M3 or M4. In this case, M1

long unsigned int last_time = 0;

bool direction = true;

void setup() {
  myMotor->run(RELEASE); //Stop the motor at the beginning
  myMotor->setSpeed(pwm);

  Serial.begin(9600);
  Serial.println("begin.")
}

void loop() {
  if(last_time > millis()){
    direction = !direction;

    if(heating) myMotor->run(direction);
  }
}
