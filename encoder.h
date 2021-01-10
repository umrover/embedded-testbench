//#ifdef ENCODER_ENABLE

// not too sure how the ifndef and define functions work

#ifndef ENCODER_H_
#define ENCODER_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "smbus.h"


enum {
	ANGLE_HIGH = 0xFE,
	ANGLE_LOW = 0xFF,

	DEVICE_SLAVE_ADDRESS_NONE_POWER = 0x40,
	DEVICE_SLAVE_ADDRESS_A1_POWER = 0x41,
	DEVICE_SLAVE_ADDRESS_A2_POWER = 0x42,
	DEVICE_SLAVE_ADDRESS_BOTH_POWER = 0x43,

	RAW_TO_180_DEGREES_CONVERSION_FACTOR = 8190
};

// public data members

typedef struct {
	int address;
} Encoder;

// functions

Encoder* new_encoder(bool A1_power, bool A2_power); // 1 if pin connected to power, 0 if pin connected to ground

uint16_t get_val_angle(Encoder* encoder);

// Deletes the encoder object
void deleteEncoder(Encoder*);

#endif

//#endif
