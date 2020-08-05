import smbus
import numpy as np
from math import ceil

# bus initialization
bus = smbus.SMBus(2)

I2C_ADDRESS = 0x53

# Wakes the accelerometer from sleep mode
bus.write_byte_data(I2C_ADDRESS, 0x2D, 0x08)

# Gets raw byte data from the accelerometer
def read_data(num):
    pass

# Gets the most significant bit (MSG) and shifts it by 8
# combines MSB and LSB (least significant bit) and scales data based off
# values in the Arduino ADXL343 digital accelerometer library
# Essentially multiply your data output by .004 * -9.80665 * 9.80665 to 
# get to m/s2
# Uses read_data() function.
def get_decimal(ls, ms):
    pass

# Rounds the number to a specified number of decimal places
# !!! Optional !!!
def float_round(num, places = 0, direction = ceil):
    pass

# Prints data to the console
def display_data(num)
    pass

# *******************************************
# Write your data output loop here.
# Make sure to use your get_decimal function.

# *******************************************
# !!! Bonus !!!
# Try modifying this code to output data to a text file.
