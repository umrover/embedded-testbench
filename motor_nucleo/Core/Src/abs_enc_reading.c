/*
 * abs_enc_read.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "abs_enc_reading.h"
#include "main.h"

SMBus *new_smbus(I2C_HandleTypeDef *hi2c) {
    SMBus *smbus = malloc(sizeof(SMBus));
    smbus->i2c = hi2c;
    smbus->DMA = TRUE;
    memset(smbus->buf, 0, sizeof(smbus->buf));
    return smbus;
}

void disable_DMA(SMBus *smbus) {
    smbus->DMA = FALSE;
}

long read_word_data(SMBus *smbus, uint8_t addr, char cmd) {
    smbus->buf[0] = cmd;
    if (!smbus->DMA) smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, HAL_MAX_DELAY);
    else smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 1);

    //reads from address sent above
    if (!smbus->DMA) smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, 2, HAL_MAX_DELAY);
    else smbus->ret = HAL_I2C_Master_Receive_DMA(smbus->i2c, (addr << 1) | 1, smbus->buf, 2);

    long data = smbus->buf[0] | (smbus->buf[1] << 8);
    return data;
}

void del_smbus(SMBus *smbus) {
	free(smbus->buf);
	free(smbus);
}

Abs_Encoder* new_abs_encoder(SMBus* i2cBus, _Bool A1_power, _Bool A2_power){
	Abs_Encoder* abs_encoder = (Abs_Encoder*) malloc(sizeof(Abs_Encoder));
    if ((A1_power) && (A2_power)) abs_encoder->address = device_slave_address_both_power;
    else if (A1_power) abs_encoder->address = device_slave_address_a1_power;
    else if (A2_power) abs_encoder->address = device_slave_address_a2_power;
    else abs_encoder->address = device_slave_address_none_power;
    abs_encoder->i2cBus = i2cBus;
    return abs_encoder;
}

int read_raw_angle(Abs_Encoder* abs_encoder) {
	int raw_data = read_word_data(abs_encoder->i2cBus, abs_encoder->address, 0xFF);
	int angle_left = ( raw_data >> 8 ) & 0xFF; // 0xFE
	int angle_right = raw_data & 0xFF; // 0xFF
	int angle_left_modified = angle_left & 0x3F;
	int angle_raw = (angle_right << 6) | angle_left_modified;
    return angle_raw;
}

float get_angle_degrees(Abs_Encoder* encoder) {
    int angle_raw = read_raw_angle(encoder);
    float degrees_proportion = 180.0 * angle_raw;
    float degrees = degrees_proportion / (RAW_TO_180_DEGREES_CONVERSION_FACTOR);
    return degrees;
}


void deleteEncoder(Abs_Encoder* abs_encoder){
    free(abs_encoder);
}

Abs_Encoder* abs_encoder_init(I2C_HandleTypeDef* abs_encoder_handle){
	SMBus* i2cBus = new_smbus(abs_encoder_handle);
	return new_abs_encoder(i2cBus, 0, 0);
}

void read_abs_enc(Abs_Encoder* abs_encoder, uint8_t channel) {
	float current_angle = get_angle_degrees(abs_encoder);
	((channels + channel)->abs_enc_value) = current_angle;
}
