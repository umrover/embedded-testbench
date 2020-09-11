import smbus
import numpy as np
from math import ceil

# bus initialization
# We initialize bus 2 because that is the only free on on the beaglebone.
bus = smbus.SMBus(2)

# Accelerometer I2C address.
I2C_ADDRESS = 0x53

# Relevant registers.
X_LS = 0x32
X_MS = 0x33
Y_LS = 0x34
Y_MS = 0x35
Z_LS = 0x36
Z_MS = 0x37

# Wakes the accelerometer from sleep mode.
bus.write_byte_data(I2C_ADDRESS, 0x2D, 0x08)

# Reads byte data from the accelerometer.
# You will need to use this to grab both the most significant bit and
# the least significant bit.
def read_data(register):
    # TODO
    pass

# Gets the most significant bit (MSG) and shifts it by 8 bits to the left
# combines MSB and LSB (least significant bit) and scales data based off
# values in the Arduino ADXL343 digital accelerometer library.
# Essentially multiply your data output by .004 * -9.80665 * 9.80665 to 
# get to m/s2
# Uses read_data() function to get data and numpy to combine ls and ms.
def get_decimal(ls, ms):
    # TODO
    #uncomment this code 
    # data = * combined MSB and LSB data* 
    return data * .004 * -9.80665 * 9.80665
    pass

# Rounds the number to a specified number of decimal places
def float_round(num, places = 0, direction = ceil):
    return direction(num * (10**places)) / float(10**places)

# *******************************************
# Write your data output loop here. It should print x, y, and z data.
# Make sure to use your get_decimal function.
while(True):
    # TODO
    print("X: " + str(""" x accel value""") + 
          " Y: " + str("""y accel value""") + 
          " Z: " + str("""z accel value""")) 

# *******************************************

# !!! Bonus !!!
# Try modifying this code to output data to a text file.
