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
char command;

void setup() {
  pixels.begin();
  Serial.begin(9600);
  while (!Serial) {
     ; // waiting for serial port to connect
  }
  // set data rate for uart serial port
  uart.begin(9600);
  fillScreen(pixels.Color(0, 0, 0));

  command = 'D';
}

void loop() {

   if (uart.available()) {
      char char_read = char(uart.read());
      if (char_read == 'R' || char_read == 'G' || char_read == 'B' || char_read == 'O') {
        command = char_read;
      }
   }

   switch (command) {
      case 'R':
         fillScreen(pixels.Color(64, 0, 0));
         break;
      case 'G':
         fillScreen(pixels.Color(0, 64, 0));
         delay(1000);
         command = 'O';
         break;
      case 'B':
         fillScreen(pixels.Color(0, 0, 64));
         break;
      case 'O':
         fillScreen(pixels.Color(0, 0, 0));
         delay(1000);
         command = 'G';
         break;
      default:
         command = 'D';  // If an invalid command is sent, turn off anyway
   }
}

void fillScreen(uint32_t color){
  for(int i = 0; i < pixels.numPixels(); ++i){
     pixels.setPixelColor(i, color);
  }
  pixels.show();
}
