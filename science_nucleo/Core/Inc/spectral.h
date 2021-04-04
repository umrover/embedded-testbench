//#ifdef SPECTRAL_ENABLE

#ifndef SPECTRAL_H_
#define SPECTRAL_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "smbus.h"

// private data members
#define CHANNELS 6

enum {
	DEV_SEL = 0x4F,

	I2C_AS72XX_SLAVE_STATUS_REG = 0x00,
	I2C_AS72XX_SLAVE_WRITE_REG = 0x01,
	I2C_AS72XX_SLAVE_READ_REG = 0x02,
	I2C_AS72XX_SLAVE_TX_VALID = 0x02,
	I2C_AS72XX_SLAVE_RX_VALID = 0x01,

	DEVICE_SLAVE_ADDRESS_READ = 0x93,
	DEVICE_SLAVE_ADDRESS_WRITE = 0x92,
	DEVICE_SLAVE_ADDRESS = 0x49,

	// registers for spectral sensor
	RAW_VALUE_RGA_HIGH = 0x08,
	RAW_VALUE_RGA_LOW = 0x09,

	RAW_VALUE_SHB_HIGH = 0x0A,
	RAW_VALUE_SHB_LOW = 0x0B,

	RAW_VALUE_TIC_HIGH = 0x0C,
	RAW_VALUE_TIC_LOW = 0x0D,

	RAW_VALUE_UJD_HIGH = 0x0E,
	RAW_VALUE_UJD_LOW = 0x0F,

	RAW_VALUE_VKE_HIGH = 0x10,
	RAW_VALUE_VKE_LOW = 0x11,

	RAW_VALUE_WLF_HIGH = 0x12,
	RAW_VALUE_WLF_LOW = 0x13,

	CONTROL_SET_UP = 0x04,
	INT_TIME = 0x05
};

typedef struct {
	uint8_t lsb_register;
	uint8_t msb_register;
	uint16_t color_data;
} Channel;

// public data members

typedef struct {
    SMBus *i2cBus;
    Channel *channels[CHANNELS];
} Spectral;

//private function interface

uint16_t _read_channel(Spectral *spectral, int channel);

uint16_t _get_val(Spectral *spectral, uint8_t virtual_reg_l, uint8_t virtual_reg_h);

void _virtual_write(Spectral *spectral, uint8_t v_reg, uint8_t data);

uint8_t _virtual_read(Spectral *spectral, uint8_t v_reg);

void _get_channel_data(Spectral *spectral);

Channel* _new_channel(uint8_t lsb_r, uint8_t msb_r);

void _del_channel(Channel *channel);

//public function interface

// initalizes spectral object, adds bus to it
Spectral *new_spectral(SMBus *i2cBus);

// sets enable bits in devices
void enable_spectral(Spectral *spectral);

// gets the data as an array of 16 bit integers
// channels RSTUVW
void get_spectral_data(Spectral *spectral, uint16_t *data);

void del_spectral(Spectral *spectral);

#endif

//#endif
