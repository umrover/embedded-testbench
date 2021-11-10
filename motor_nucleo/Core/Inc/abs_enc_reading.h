/*
 * abs_enc_read.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#ifndef INC_ABS_ENC_READING_H_
#define INC_ABS_ENC_READING_H_

#define RAW_TO_180_DEGREES_CONVERSION_FACTOR 8192.0


#include "stdint.h"
#include "smbus.h"


enum {

	device_slave_address_none_power = 0x40,
	device_slave_address_a1_power = 0x41,
	device_slave_address_a2_power = 0x42,
	device_slave_address_both_power = 0x43,

};

// public data members

typedef struct {
	int address;
	SMBus* i2cBus;
} AbsEncoder;

// functions

AbsEncoder* new_abs_encoder(SMBus* i2cBus, _Bool A1_power, _Bool A2_power); // 1 if pin connected to power, 0 if pin connected to ground

int read_raw_angle(AbsEncoder* encoder);

float get_angle_degrees(AbsEncoder* encoder);

// Deletes the encoder object

void delete_abs_encoder(AbsEncoder*);

AbsEncoder* abs_encoder_init(I2C_HandleTypeDef* abs_encoder_handle, _Bool A1, _Bool A2);

float read_abs_enc(AbsEncoder* abs_enc_0, AbsEncoder* abs_enc_1, uint8_t channel);


#endif /* INC_ABS_ENC_READING_H_ */
