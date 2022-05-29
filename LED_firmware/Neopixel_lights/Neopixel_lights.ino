#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN 3 // Pin on arduino that is connected to NeoPixel
#define NUMPIXELS 64 // Number of NeoPixels attached to Arduino

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial uart(0, 1); // This is pin of RX and TX (just use usb)

// command can either be Default (off), Red, Green, Blue, or Off (for flashing state)
int command;
const int BUFFER_SIZE = 30;
char buf[BUFFER_SIZE];

void setup() {
  pixels.begin();
  Serial.begin(9600);
  // set data rate for uart serial port
  uart.begin(9600);
  fillScreen(pixels.Color(0, 0, 0));

  command = 3;
}

void loop() {
  //Serial.println(Serial.available());
   if (uart.available() > 0) {
      int rlen = uart.readBytes(buf, BUFFER_SIZE);
   }

   char delim[] = ",";
   char *identifier = strtok(buf,delim);

   if (!strcmp(identifier,"$LED")){
      command = atoi(strtok(NULL,delim));
   }
   switch (command) {
      case 0:
         fillScreen(pixels.Color(64, 0, 0));
         break;
      case 1:
         fillScreen(pixels.Color(0, 64, 0));
         delay(500);
         command = 4;
         break;
      case 2:
         fillScreen(pixels.Color(0, 0, 64));
         break;
      case 3:
         fillScreen(pixels.Color(0, 0, 0));
         break;
      case 4:
         fillScreen(pixels.Color(0, 0, 0));
         delay(500);
         command = 1;
         break;
      default:
         command = 3;  // If an invalid command is sent, turn off anyway
   }
   delay(100);
}

void fillScreen(uint32_t color){
  for(int i = 0; i < pixels.numPixels(); ++i){
     pixels.setPixelColor(i, color);
  }
  pixels.show();
}
