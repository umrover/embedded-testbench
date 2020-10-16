#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "smbus.h"


const int I2C_MUX_ADDRESS = 0x70;
const int MUX_READ_WRITE_REG = 0xCC;  # filler address
const int DEVICE_SLAVE_ADDRESS = 0x49;
// ADDRESSES = {'t': 0x8, 'b': 0x2, 'y': 0x4, 'w': 0x1}
int ADDRESSES[3] = {0x1, 0x2, 0x3};