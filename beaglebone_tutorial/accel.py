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
    readbytedata = bus.read_byte_data(I2C_ADDRESS, register)
    return readbytedata

# Gets the most significant bit (MSG) and shifts it by 8 bits to the left


# combines MSB and LSB (least significant bit) and scales data based off
# values in the Arduino ADXL343 digital accelerometer library.
# Essentially multiply your data output by .004 * -9.80665 * 9.80665 to 
# get to m/s2
# Uses read_data() function to get data and numpy to combine ls and ms.
def get_decimal(ls, ms):
    lsb = read_data(ls)
    msb = read_data(ms) << 8
    data = msb | lsb
    return np.int16(data) * .004 * -9.80665
    pass


# Rounds the number to a specified number of decimal places
def float_round(num, places = 0, direction = ceil):
    return direction(num * (10**places)) / float(10**places)

# *******************************************
# Write your data output loop here. It should print x, y, and z data.
# Make sure to use your get_decimal function.

File_object = open(r"Acceleration_data.txt","a")
while(True):
    # TODO
    xdecimal = get_decimal(X_LS, X_MS)
    ydecimal = get_decimal(Y_LS, Y_MS)
    zdecimal = get_decimal(Z_LS, Z_MS)
    xdecimal = float_round(xdecimal)
    ydecimal = float_round(ydecimal)
    zdecimal = float_round(zdecimal)
    print("X: " + str(xdecimal) +
          " Y: " + str(ydecimal) +
          " Z: " + str(zdecimal))
    File_object.writelines(L) for L = [str(xdecimal), str(ydecimal), str(zdecimal)]


# *******************************************

# !!! Bonus !!!
# Try modifying this code to output data to a text file.
