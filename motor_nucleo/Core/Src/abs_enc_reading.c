/*
 * abs_enc_read.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "abs_enc_reading.h"

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
    if (!smbus->DMA) smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, 50);
    else smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 1);

    //reads from address sent above
    if (!smbus->DMA) smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, 2, 50);
    else smbus->ret = HAL_I2C_Master_Receive_DMA(smbus->i2c, (addr << 1) | 1, smbus->buf, 2);

    long data = smbus->buf[0] | (smbus->buf[1] << 8);
    return data;
}

void del_smbus(SMBus *smbus) {
	free(smbus->buf);
	free(smbus);
}

// A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
AbsEncoder* new_abs_encoder(SMBus* i2cBus, uint8_t A1, uint8_t A2){
	AbsEncoder* abs_encoder = (AbsEncoder*) malloc(sizeof(AbsEncoder));
    if ((A1) && (A2)) abs_encoder->address = device_slave_address_both_power;
    else if (A1) abs_encoder->address = device_slave_address_a1_power;
    else if (A2) abs_encoder->address = device_slave_address_a2_power;
    else abs_encoder->address = device_slave_address_none_power;
    abs_encoder->i2cBus = i2cBus;
    return abs_encoder;
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
    float degrees = degrees_proportion / (RAW_TO_180_DEGREES_CONVERSION_FACTOR);
    return degrees;
}

float get_angle_radians(AbsEncoder* encoder) {
	float degrees = get_angle_degrees(encoder);
	return degrees * 3.141529/180.0;
}

void deleteEncoder(AbsEncoder* abs_encoder){
    free(abs_encoder);
}

AbsEncoder* abs_encoder_init(I2C_HandleTypeDef* abs_encoder_handle, uint8_t A1, uint8_t A2){
	SMBus* i2cBus = new_smbus(abs_encoder_handle);
	return new_abs_encoder(i2cBus, A1, A2);
}
