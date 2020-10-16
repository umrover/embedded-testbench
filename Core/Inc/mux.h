#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "smbus.h"


const int I2C_MUX_ADDRESS = 0x70;
const int MUX_READ_WRITE_REG = 0xCC;  // filler address
const int DEVICE_SLAVE_ADDRESS = 0x49;
//int ADDRESSES[3] = {0x1, 0x2, 0x3};

typedef struct {
    Bus *i2cbus;
    int channel_list[8];
} Mux;

Mux *new_mux(Mux *mux, Bus *i2cbus, int channel, int *channel_list);

long read(Mux *mux, int channel);

long write(Mux *mux, int channel);

