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

# Powers the TCS34725 device on
bus.write_byte_data(I2C_ADDRESS, 0x00, 0x03)

# Sets the RGBC Gain value to 1xgain
bus.write_byte_data(I2C_ADDRESS, 0x0F, 0x00)

# Sets command register

# Prints data and saves to a file
File_object = open(r"rgb_data.txt","w")
while(True):
    # TODO
    rdecimal = read_word_data(I2C_ADDRESS, R_LS | 0xA6)
    gdecimal = read_word_data(I2C_ADDRESS, G_LS | 0xA6)
    bdecimal = read_word_data(I2C_ADDRESS, B_LS | 0xA6)
    print("R: " + str(rdecimal) +
          " G: " + str(gdecimal) +
          " B: " + str(bdecimal))
    L = [str(rdecimal) + "," + str(gdecimal) + "," + str(bdecimal)]
    File_object.writelines(L)


# *******************************************

# !!! Bonus !!!
# Try modifying this code to output data to a text file.
