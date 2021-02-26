#include <Arduino.h>
#include "wiring_private.h"

UART mySerial (digitalPinToPinName(6), digitalPinToPinName(5), NC, NC); // create a hardware serial port named mySerial with RX: pin 5 and TX: pin 6

void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  mySerial.write("A \n");
  Serial.write("A \n");
  delay(1000);
}
