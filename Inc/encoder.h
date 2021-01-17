//#ifdef ENCODER_ENABLE

#ifndef ENCODER_H_
#define ENCODER_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stdint.h"
#include "stm32f3xx_hal.h"
#include "smbus.h"


enum {
	angle_high = 0xFE,
	angle_low = 0xFF,

	device_slave_address_none_power = 0x40,
	device_slave_address_a1_power = 0x41,
	device_slave_address_a2_power = 0x42,
	device_slave_address_both_power = 0x43,

	raw_to_180_degrees_conversion_factor = 8190
};

// public data members

typedef struct {
	int address;
	SMBus* i2cBus;
} Encoder;

// functions

Encoder* new_encoder(SMBus* i2cBus, _Bool A1_power, _Bool A2_power); // 1 if pin connected to power, 0 if pin connected to ground

uint16_t read_raw_angle(Encoder* encoder);

double get_angle_degrees(Encoder* encoder);

// Deletes the encoder object

void deleteEncoder(Encoder*);

#endif

//#endif
