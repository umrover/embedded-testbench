/*
 * abs_enc_read.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "abs_enc_reading.h"

#include "smbus.h"

AbsEncoder* new_abs_encoder(SMBus* i2cBus, _Bool A1_power, _Bool A2_power){
	AbsEncoder* abs_encoder = (AbsEncoder*) malloc(sizeof(AbsEncoder));

	if ((A1_power) && (A2_power))
	{
		abs_encoder->address = device_slave_address_both_power;
	}
	else if (A1_power)
	{
		abs_encoder->address = device_slave_address_a1_power;
	}
	else if (A2_power)
	{
		abs_encoder->address = device_slave_address_a2_power;
	}
	else
	{
		abs_encoder->address = device_slave_address_none_power;
    	abs_encoder->i2cBus = i2cBus;
	}

    return abs_encoder;
}

AbsEncoder* abs_encoder_init(I2C_HandleTypeDef* abs_encoder_handle, _Bool A1, _Bool A2){
	SMBus *i2cBus = new_smbus(abs_encoder_handle);
	return new_abs_encoder(i2cBus, A1, A2);
}

int read_raw_angle(AbsEncoder* abs_encoder) {
	int raw_data = read_word_data(abs_encoder->i2cBus, abs_encoder->address, 0xFF);
	int angle_left = ( raw_data >> 8 ) & 0xFF; // 0xFE
	int angle_right = raw_data & 0xFF; // 0xFF
	int angle_left_modified = angle_left & 0x3F;
	int angle_raw = (angle_right << 6) | angle_left_modified;
    return angle_raw;
}

float get_angle_degrees(AbsEncoder* encoder) {

    int angle_raw = read_raw_angle(encoder);
    float degrees_proportion = 180.0 * angle_raw;
    float degrees = degrees_proportion / (float)(RAW_TO_180_DEGREES_CONVERSION_FACTOR);

    return degrees;
}

float read_abs_enc(AbsEncoder* abs_enc_0, AbsEncoder* abs_enc_1, uint8_t channel) {
	float current_angle;
	if (channel == 0)
	{
		current_angle = get_angle_degrees(abs_enc_0);
	}
	else if (channel == 1)
	{
		current_angle = get_angle_degrees(abs_enc_1);
	}

	return current_angle;
}

void delete_abs_encoder(AbsEncoder* encoder){
    free(encoder);
}
