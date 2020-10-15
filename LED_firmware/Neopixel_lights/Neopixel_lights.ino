#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#ifdef _AVR_
#include <avr/power.h>
#endif

#define PIN 3
#define NUMPIXELS 64

Adafruit_NeoPixel matrix(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial uart(10,11);

void setup() {
  matrix.begin();
  uart.begin(9600);
  Serial.begin(9600);
}

char command = 'M';

void loop() {
   if(uart.available()){
      command = char(uart.read());
   }
   
   //auton
   if (command == 'A') {
      fillScreen(matrix.Color(64, 0, 0));
      command = 'N';
   }
  
   //manual
   else if (command == 'M') {
      fillScreen(matrix.Color(0, 0, 64));
      command = 'N';
   }
  
   //mission leg done
   else if (command == 'D') {
   command = 'A';
      for (int i=0; i<3; i++){
          fillScreen(matrix.Color(0, 64, 0));
          delay(400);
          fillScreen(matrix.Color(0, 0, 0));
          delay(400);
      }
   }
}

void fillScreen(uint32_t color){
  for(int i = 0; i < matrix.numPixels(); i++){
     matrix.setPixelColor(i, color);
  }
  matrix.show();
}
