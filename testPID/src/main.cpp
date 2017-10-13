// TestPID motor control loop for PoE lab 3

#include "Arduino.h"
#include <Adafruit_MotorShield.h>
#include "IRLibAll.h"

const int readDelay = 10;
const float sensorDiffToLinePos = 0.1;
float kp = 30, ki = 0.01, kd = 50;
//Working but slow: 30, 0.01, 50

const int fast_speed = 60;
const int slow_speed = 40;
int cur_speed = slow_speed;
bool fast = false;

const int sensor_calibration_offset = -60;
const float sensor_calibration_scale = 1.4;
const int turnSpeedLeft = 255, turnSpeedRight = 0;

const int turnLength = 20; //iterations after releasing button to keep turning
int turnCounter = 0; //How long, in iterations, left in this turn

int integral[50];
const int integral_window = 20;
int integral_i = 0;
int integral_sum = 0;

const int numLeds = 2;
const int ledPins[] = {5, 7};
const int sensorPin = A0;
const int analogPower = A1;
const int motorPins[] = {1, 2}; //Not on the Arduino, on the shield!
const int buttonPin = A2; //Start button

const int remotePin = 2; //The IR remote receiver pin
const int spareGround = 3; //Ground pin for great good

IRrecvPCI myReceiver(remotePin); //IR remote receiver
IRdecode myDecoder; //IR remote decoder

int sensorValues[3];
float linePos, lastPos, correction = 0; //Higher values indicate further to the right
long unsigned int lastTime, loopTime = 0; //0 to indicate first loop (i.e. don't set anything)

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(motorPins[0]);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(motorPins[1]);

void setup() {
  Serial.begin(9600);

  for (int i=0; i<3; i++) pinMode(ledPins[i], OUTPUT);
  pinMode(sensorPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(analogPower, OUTPUT);
  digitalWrite(analogPower, HIGH);

  AFMS.begin(); //Initialize the motor shield
  myReceiver.enableIRIn(); //Initialize the remote receiver

  pinMode(remotePin, INPUT_PULLUP); //Hopefully this won't mess with the IR library
  pinMode(spareGround, OUTPUT); digitalWrite(spareGround, LOW); //Spare ground pin for IR receiver

  //while (digitalRead(buttonPin) == HIGH) delay(10); //Wait for button press

  Serial.println("Begin");
}

void serial_tune_pid() {
  if(Serial.available()){
    rightMotor->run(RELEASE);
    leftMotor->run(RELEASE);

    Serial.read();
    const int num_inputs = 4;

    int c_index = 0;
    int parameter_i = 0;
    char in_string[7];
    float in_values[num_inputs];

    Serial.println("... Reading");
    Serial.println("cur:" + String(kp) + " " + String(ki) + " " + String(kd) + " " + cur_speed);

    while(parameter_i < num_inputs) {
      if(Serial.available()){
        char c = Serial.read();
        Serial.print(c);

        if(c == ' '){
          in_string[c_index++] = '\0';
          Serial.println(in_string);
          in_values[parameter_i] = atof(in_string);

          // Serial.print(in_string);
          c_index = 0;
          in_string[0] = '\0';
          parameter_i++;
        }
        else {
          in_string[c_index++] = c;
        }
      }

    }

    Serial.print("Setting PID to ");
    for(int i = 0; i<num_inputs; i++){
      Serial.print(in_values[i]);
      Serial.print(' ');
    }
    Serial.println();

    kp = in_values[0];
    ki = in_values[1];
    kd = in_values[2];
    cur_speed = in_values[3];
    // sensor_calibration_offset = int(in_values[3]);
    // sensor_calibration_scale = in_values[4];
  }
}

void loop() {
  serial_tune_pid();

  if (myReceiver.getResults()) { //First, read from the remote, since that can change lots of behavior
    myDecoder.decode();           //Decode it
    // myDecoder.dumpResults(true);  //Now print results -- or don't if we want speed
    myReceiver.enableIRIn();      //Restart receiver
    // turnCounter = turnLength;
    fast = !fast;
    cur_speed = fast ? fast_speed : slow_speed;
    Serial.println(cur_speed);
    Serial.println("GOTTA GO FAST");
  } // else if (turnCounter > 0) turnCounter--;

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

    // for(int i=0; i<numLeds; i++){
    //   Serial.print(sensorValues[i]);
    //   Serial.print("\t");
    // }
    // ADD auto calibration on button press
    /*Serial.print(sensorValues[0] - sensorValues[1]);
    Serial.print("\t");
    Serial.print(((sensorValues[0] - sensorValues[1]) + sensor_calibration_offset) * sensor_calibration_scale);*/

    //Calculate line position based on sensor readings
    linePos = ((sensorValues[0] - sensorValues[1]) + sensor_calibration_offset) * sensor_calibration_scale; //This should be tuned!

    //Do the PID thing
    integral_i++;
    if(integral_i > integral_window){ integral_i = 0; }
    integral[integral_i] = linePos;

    integral_sum = 0;
    for(int i=0; i<integral_window; i++){
      integral_sum += integral[i];
    }


    correction = (kp * linePos) + (ki * integral_sum) + (kd * (linePos-lastPos));
    Serial.print("\t" + String(correction));
    Serial.println("P: " + String(kp * linePos) + " I: " + (ki * integral_sum) + " D: " + (kd * (linePos-lastPos)) + " Cor: " + correction);
    lastPos = linePos;
    if (correction > cur_speed) correction = cur_speed; //Clamp to usable values
    if (correction < -cur_speed) correction = -cur_speed;

    //Set the motor speeds
    rightMotor->setSpeed(correction < 0 ? cur_speed : cur_speed-correction);
    leftMotor->setSpeed(correction > 0 ? cur_speed : cur_speed+correction);

    //And tell them to run
    rightMotor->run(FORWARD);
    leftMotor->run(FORWARD);


    Serial.println();
  } //end if turning

}
