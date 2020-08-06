#include <SPI.h>

const int MISO_pin = 12;
const int MOSI_pin = 11;
const int SCLK_pin = 13;
const int SS_pin = 10;


void setup() {
  delay(250);
  Serial.begin(9600);

  SPI.begin();

  pinMode(MISO_pin, INPUT);
  pinMode(SS_pin, OUTPUT);
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  delay(100);

}

void loop() {
  
  digitalWrite(SS_pin, LOW);
  int high, low;
  //(0b00111111) & 
  high = (SPI.transfer(0));
  low = SPI.transfer(0);
  digitalWrite(SS_pin, HIGH);

  uint16_t angle = (high << 8) | low;
  double deg = angle / 65536.0;
  deg *= 360;
  Serial.println(deg);
  delay(100);
}
