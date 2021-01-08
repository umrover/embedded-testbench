Code for the AS5048B Magnetic Rotary Encoder
======================================================================================
### About
This is the code for running the AS5048B magnetic rotary encoder. It is connected to the TSSOP-14 breakout board. i2c is used. It will output the angle position of the 14-bit angular position sensor as a number between 0 and 360. The slave address is determined by whether or not the A1 or A2 pins are connected to ground. This code assumes that A1 and A2 are connected to ground.

[Datasheet](https://ams.com/documents/20143/36005/AS5048_DS000298_4-00.pdf/910aef1f-6cd3-cbda-9d09-41f152104832)

### Usage
Required electrical components: \
1 AS5048B magnetic rotary encoder \
1 TSSOP-14 breakout board \
1 beaglebone green/back board 

Connect to ground, SDA, SCL, power to 3.3V, and power to 5V on beaglebone. A1 and A2 are connected to ground. i2c is used.

### Notes
This code has been tested with python 3.6 on a beaglebone black. Make sure to use the AS5048B encoder since i2c is being used. spi will require the usage of the AS5048A encoder.
