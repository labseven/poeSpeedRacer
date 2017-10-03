#include "Arduino.h"

const int SENSOR_PIN = A0;
const int led_pins[] = {5,6,7};
const int num_led_pins = 3;

int sensor_values[num_led_pins] = {};

void setup()
{
  for(int i=0; i < num_led_pins; i++){
    pinMode(led_pins[i], OUTPUT);
    digitalWrite(led_pins[i], LOW);
  }

  Serial.begin(9600);
  Serial.println("Begin");

}

void loop()
{
  for(int i=0; i < num_led_pins; i++){
    digitalWrite(led_pins[i], HIGH);
    sensor_values[i] = analogRead(SENSOR_PIN);
    digitalWrite(led_pins[i], LOW);
  }

  for(int i=0; i < num_led_pins; i++){
    Serial.print(sensor_values[i]);
    Serial.print("\t");
  }
  Serial.print("\t|\t");
  for(int i=0; i < num_led_pins-1; i++){
    Serial.print(sensor_values[i]-sensor_values[i+1]);
    Serial.print("\t");
  }
  Serial.println();


  delay(50);
}
