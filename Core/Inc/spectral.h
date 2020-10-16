#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "smbus.h"

#define CHANNELS 6

const int DEV_SEL = 0x4F;

const int I2C_AS72XX_SLAVE_STATUS_REG = 0x00;
const int I2C_AS72XX_SLAVE_WRITE_REG = 0x01;
const int I2C_AS72XX_SLAVE_READ_REG = 0x02;
const int I2C_AS72XX_SLAVE_TX_VALID = 0x02;
const int I2C_AS72XX_SLAVE_RX_VALID = 0x01;

const int DEVICE_SLAVE_ADDRESS_READ = 0x93;
const int DEVICE_SLAVE_ADDRESS_WRITE = 0x92;
const int DEVICE_SLAVE_ADDRESS = 0x49;

// registers for triad
const int RAW_VALUE_RGA_HIGH = 0x08;
const int RAW_VALUE_RGA_LOW = 0x09;

const int RAW_VALUE_SHB_HIGH = 0x0A;
const int RAW_VALUE_SHB_LOW = 0x0B;

const int RAW_VALUE_TIC_HIGH = 0x0C;
const int RAW_VALUE_TIC_LOW = 0x0D;

const int RAW_VALUE_UJD_HIGH = 0x0E;
const int RAW_VALUE_UJD_LOW = 0x0F;

const int RAW_VALUE_VKE_HIGH = 0x10;
const int RAW_VALUE_VKE_LOW = 0x11;

const int RAW_VALUE_WLF_HIGH = 0x12;
const int RAW_VALUE_WLF_LOW = 0x13;

static HAL_StatusTypeDef ret;
uint8_t buf[30];



