#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>

Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);


void setup() {
  // Be sure to set the serial monitor to display data at this rate when you open it 
  // tools -> serial monitor 
  Serial.begin(115200);
  while (!Serial);

  // prints this string to the serial monitor 
  Serial.println("Accelerometer Test"); Serial.println("");

  //find the accel function that will initalize the sensor
  //<init function> 

  // set the output range to 16G 
  // <set range function> 
  

}

void loop() {
  // uncomment the code below and find the function get the x, y, and z values 
  
  /* int x = <get x function> 
   * int y = <get y function>
   * int z = <get z function>
   */
   
  // the values from the getx,y and z function are raw readings, in order to scale the 
  // units to meters/s^2, they must be multiplied by constants that can be found in 
  // upon closer inspection in the arduino code's 'getEvent(sensors_event_t *event) function
  // the constants will be defined in the ADXL343.h file
  
  /* x = x * <adxl mutliplier> * <gravity standard>
   * y = y * <adxl mutliplier> * <gravity standard>
   * z = z * <adxl mutliplier> * <gravity standard>
   */

   // print the x, y, z values to the serial monitor using the Serial.print(<string/data>) function
   //Serial.print("x: ");Serial.print(x);

   delay(500);
}
