#include "Arduino.h"
#include "IRLibAll.h"

IRrecvPCI myReceiver(2);
IRdecode myDecoder;

int IR_pin = A0;
int a[3] = {};

void setup()
{
  Serial.begin(9600);
  myReceiver.enableIRIn();
  Serial.println("Begin");

}

void loop()
{

  if (myReceiver.getResults()) {
    myDecoder.decode();           //Decode it
    myDecoder.dumpResults(true);  //Now print results. Use false for less detail
    myReceiver.enableIRIn();      //Restart receiver
  }

  // // a = {0, 0, 0};
  // for(int i = 0; i < 3; i++){
  //   a[i] = analogRead(IR_pin);
  // }
  // for(int i = 0; i < 3; i++){
  //   Serial.print(a[i]);
  //   Serial.print("\t");
  // }
  // Serial.println();
  // delay(50);
}
