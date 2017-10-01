#include "Arduino.h"

int IR_pin = A0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Begin");

}

void loop()
{
  Serial.println(analogRead(IR_pin));
  delay(50);
}
