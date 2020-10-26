import smbus
import numpy as np
from math import ceil

# bus initialization
# We initialize bus 2 because that is the only free on on the beaglebone.
bus = smbus.SMBus(2)

# Accelerometer I2C address.
# TODO: find this in the datasheet
I2C_DEV_ADDRESS = <insert address val here> 

# Relevant registers.
X_LSB = 0x32
X_MSB = 0x33
Y_LSB = 0x34
Y_MSB = 0x35
Z_LSB = 0x36
Z_MSB = 0x37

POWER_CTRL_REG = 0x2D
DATA_FORMAT_REG = 0x31


# Wakes the accelerometer from sleep mode.
# TODO write the value to wake the accelerometer from sleep mode
bus.write_byte_data(<insert i2c address>, <insert register to write to>, <insert value to set register to>)

# Formats output data (pg 24/25 on data sheet)
# TODO format data so that the output range is full resolution +/- 16g 
bus.write_byte_data(<insert i2c address>, <insert register to write to>, <insert value to set register to>)

# Reads 1 byte of data from a specified register on the accelerometer.
# You will need to use this to grab both the most significant bit and
# the least significant bit.
# this is a wrapper for smbus.read_byte_data
def read_data(register):
    # TODO
    pass

# Gets the most significant bit (MSG) and shifts it by 8 bits to the left
# combines MSB and LSB (least significant bit) and scales data based off
# values in the Arduino ADXL343 digital accelerometer library. 
# (numbers that can additionally be found in the datasheet)
# Uses read_data() function to get data and numpy to combine ls and ms.
def get_decimal(lsb_register, msb_register):
    # TODO
    # uncomment and finish this code 
    # data = <combined MSB and LSB data>
    return data * <scalers> 
    pass

# Rounds the number to a specified number of decimal places
# usage: float_round(<number>, <number of decimal places>)
def float_round(num, places = 0, direction = ceil):
    return direction(num * (10**places)) / float(10**places)

# *******************************************
# Write your data output loop here. It should print x, y, and z data.
# Make sure to use your get_decimal function.
while(True):
    # TODO: format this to print data correctly, and add the releveant functions/logic to 
    # get the x, y, z values 
    print("X: " + str(""" x accel value""") + 
          " Y: " + str("""y accel value""") + 
          " Z: " + str("""z accel value""")) 

# *******************************************

# !!! Bonus !!!
# Try modifying this code to output data to a text file.
