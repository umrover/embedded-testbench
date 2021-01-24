
void setup() {
  // put your setup code here, to run once:
  int baudrate = 115200;
  Serial.begin(baudrate);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.write('A');
}
