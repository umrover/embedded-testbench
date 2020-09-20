#include "Adafruit_NeoPixel.h"
#include <SoftwareSerial.h>
import Serial
#ifdef _AVR_
#include <avr/power.h>
#endif

#define PIN 3

#define NUMPIXELS 64

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 400

SoftwareSerial uart(10,11);

void setup() {
  
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  pixels.begin();
  Serial.begin(9600);
  uart.begin(9600);
}

void loop() {
 pixels.clear();
  
 if(uart.available() > 0)
  {
    Serial.write(uart.read());
  }
  
 //auton
 if (Serial.read() == "a") {
   for(int i=0; i<NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(255, 0, 0));
     pixels.show();
     pixels.delay_ns(DELAYVAL)
   }
 }

//manual
 else if (Serial.read() == "m") {
   for(int i=0; i<NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(0, 0, 255));
     pixels.show();
     pixels.delay_ns(DELAYVAL)
   }
 }

//mission leg
 else if (Serial.read() == "l") {
   for(int i=0; i<NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(0, 150, 0));
     pixels.delay_ns(DELAYVAL)
     pixels.clear();
     pixels.delay_ns(DELAYVAL)
   }
 }

}
