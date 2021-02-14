//#ifdef ENCODER_ENABLE

#ifndef ENCODER_H_
#define ENCODER_H_

#define RAW_TO_180_DEGREES_CONVERSION_FACTOR 8192.0


#include "stdint.h"
#include "smbus.h"


enum {
	angle_high = 0xFE,
	angle_low = 0xFF,

	device_slave_address_none_power = 0x40,
	device_slave_address_a1_power = 0x41,
	device_slave_address_a2_power = 0x42,
	device_slave_address_both_power = 0x43,

};

// public data members

typedef struct {
	int address;
	SMBus* i2cBus;
} Encoder;

// functions

Encoder* new_encoder(SMBus* i2cBus, _Bool A1_power, _Bool A2_power); // 1 if pin connected to power, 0 if pin connected to ground

int read_raw_angle(Encoder* encoder);

float get_angle_degrees(Encoder* encoder);

// Deletes the encoder object

void deleteEncoder(Encoder*);

#endif

//#endif
