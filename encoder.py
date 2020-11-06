import smbus
import numpy as np
import time
from math import ceil

# Assuming using AS5048B
# Connect to BLANK : TSSOP14 Pin  to Beaglebone Pin
# Connect to GND : 13 to Cape P9 1 or 2
# Connect to PWR/5V : 11 to Cape P9 5 or 6
# Connect to SDA : 1 to Cape P9 20
# Connect to SCL : 2 to Cape P9 19

# bus initialization
# We initialize bus 2 because that is the only free on on the beaglebone.
bus = smbus.SMBus(2)

# I2C Slave Address of AS5048A/AS5048B
I2C_ADDRESS = 0x15

# Relevant register
AngleReg = 0x3FFF

# These registers are not being used 
MagnitudeReg = 0x3FFE
ClrErrorReg = 0x0001

# This value represents 180 degrees
MaxValue = 8190.0

# bus.read_byte_data(I2C_ADDRESS, register)

while(True):

    # read the raw data
    # two transmission sequences are necessary
    bus.read_byte_data(I2C_ADDRESS, AngleReg | 0x80)
    AngleData = bus.read_byte_data(I2C_ADDRESS, AngleReg | 0x80)

    # change the value of Angle Data if angle is over 180
    if AngleData > MaxValue:
        AngleData = -AngleData

    Degrees = 180 * (MaxValue + AngleData) / (MaxValue)
    Degrees180 = Degrees - 180
    print("Angle in Degrees: " + str(Degrees) + "; Degrees w/out 180:" + str(Degrees180) + "; Raw Angle Data: " + str(AngleData))

