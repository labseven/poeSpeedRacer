#include "Arduino.h"

int IR_pin = A0;
int a[3] = {};

void setup()
{
  Serial.begin(9600);
  Serial.println("Begin");

}

void loop()
{
  // a = {0, 0, 0};
  for(int i = 0; i < 3; i++){
    a[i] = analogRead(IR_pin);
  }
  for(int i = 0; i < 3; i++){
    Serial.print(a[i]);
    Serial.print("\t");
  }
  Serial.println();
  delay(50);
}
