import smbus
import numpy as np
import time
from math import ceil

# bus initialization
# We initialize bus 2 because that is the only free on on the beaglebone.
bus = smbus.SMBus(2)

# TCS34725 RGB light sensor I2C address.
I2C_ADDRESS = 0x29

# Relevant registers.
C_LS = 0x14
R_LS = 0x16
G_LS = 0x18
B_LS = 0x1A

# The following lines powers the TCS34725 device on

# Enable Register (0x00) PON bit.
bus.write_byte_data(I2C_ADDRESS, 0x00 | 0x80, 0x01)

# Enable Register (0x00) AEN bit.
time.sleep(0.5)
bus.write_byte_data(I2C_ADDRESS, 0x00 | 0x80, 0x03)

# Enable Register (0x00) WEN bit.
time.sleep(0.5)
bus.write_byte_data(I2C_ADDRESS, 0x00 | 0x80, 0x0B)

# The configuration register sets the wait time (not WLONG)
bus.write_byte_data(I2C_ADDRESS, 0x0D | 0x80, 0x00)

# Change Wait Time to 0.029 sec
bus.write_byte_data(I2C_ADDRESS, 0x03 | 0x80, 0xFF)

# Prints data and saves to a file
# File_object = open(r"rgb_data.txt","w")
while(True):
    # let it rest
    # read the raw data
    time.sleep(0.003)
    cdecimal = bus.read_word_data(I2C_ADDRESS, C_LS | 0xA0)
    time.sleep(0.003)
    rdecimal = bus.read_word_data(I2C_ADDRESS, R_LS | 0xA0)
    time.sleep(0.003)
    gdecimal = bus.read_word_data(I2C_ADDRESS, G_LS | 0xA0)
    time.sleep(0.003)
    bdecimal = bus.read_word_data(I2C_ADDRESS, B_LS | 0xA0)

    # change the raw data into numbers from 0 to 255
    if cdecimal == 0:
        r = 0
        g = 0
        b = 0
    else:
        r = rdecimal*255.0/cdecimal
        g = gdecimal*255.0/cdecimal
        b = bdecimal*255.0/cdecimal

    print("R: " + str(r) +
          " G: " + str(g) +
          " B: " + str(b))
    # L = [str(r) + "," + str(g) + "," + str(b)]
    # File_object.writelines(L)
