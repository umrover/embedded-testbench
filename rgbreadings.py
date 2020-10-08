import smbus
import numpy as np
from math import ceil

# bus initialization
# We initialize bus 2 because that is the only free on on the beaglebone.
bus = smbus.SMBus(2)

# TCS34725 RGB light sensor I2C address.
I2C_ADDRESS = 0x29

# Relevant registers.
R_LS = 0x16
G_LS = 0x18
B_LS = 0x1A

# Sets the RGBC Gain value to 1xgain
bus.write_byte_data(I2C_ADDRESS, 0x0F, 0x00)

# Reads byte data from the light sensor.
# You will need to use this to grab both the most significant bit and
# the least significant bit.
def read_data(register):
    # TODO
    readbytedata = bus.read_byte_data(I2C_ADDRESS, register)
    return readbytedata

# Gets the most significant bit (MSG) and shifts it by 8 bits to the left


# combines MSB and LSB (least significant bit) and scales data based off
# values in the Arduino ADXL343 digital accelerometer library.
# Uses read_data() function to get data and numpy to combine ls and ms.


# Rounds the number to a specified number of decimal places
def float_round(num, places = 0, direction = ceil):
    return direction(num * (10**places)) / float(10**places)

# *******************************************
# Write your data output loop here. It should print x, y, and z data.
# Make sure to use your get_decimal function.

File_object = open(r"rgb_data.txt","w")
while(True):
    # TODO
    rdecimal = read_word_data(I2C_ADDRESS, R_LS)
    gdecimal = read_word_data(I2C_ADDRESS, G_LS)
    bdecimal = read_word_data(I2C_ADDRESS, B_LS)
    print("R: " + str(rdecimal) +
          " G: " + str(gdecimal) +
          " B: " + str(bdecimal))
    L = [str(rdecimal) + "," + str(gdecimal) + "," + str(bdecimal)]
    File_object.writelines(L)


# *******************************************

# !!! Bonus !!!
# Try modifying this code to output data to a text file.
