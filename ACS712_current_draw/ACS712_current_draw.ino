                                                                                                                                                                            /*This code works with ACS712 Current sensor, it permits to read the raw data
  It's better to use it with Serial Plotter
  More details on www.surtrtech.com
*/

#define Current_sensor0 A0  //The sensor analog input pin
#define Current_sensor1 A1  //The sensor analog input pin
#define Current_sensor0 A2  //The sensor analog input pin
#define Current_sensor1 A3  //The sensor analog input pin


float i0;
float i1;
float i2;
float i3;



void setup() {

Serial.begin(9600);
pinMode(Current_sensor0, INPUT);
pinMode(Current_sensor1, INPUT);
pinMode(Current_sensor2, INPUT);
pinMode(Current_sensor3, INPUT);

}

void loop() {
  i0 = analogRead(Current_sensor0);
  i1 = analogRead(Current_sensor1);
  i2 = analogRead(Current_sensor2);
  i3 = analogRead(Current_sensor3);
  
  // 512 --> 0 i think, 0 - 5 V range
  i0 = i0 * 1/1023 - 0.5;
  i1 = i1 * 1/1023 - 0.5;
  i2 = i2 * 1/1023 - 0.5;
  i3 = i3 * 1/1023 - 0.5;  


  Serial.print(i0 * 5/.11);
  Serial.print(" ");
  
  Serial.println(i1 * 5/.11);
  Serial.print(" ");
  
  Serial.println(i2 * 5/.11);
  Serial.print(" ");
  
  Serial.println(i3 * 5/.11);

  
  //delay(100);                     //Modifying or removing the delay will change the way the signal is shown 
                                  //set it until you get the correct sinewave shap

}
