/*
 * command_handling.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#ifndef INC_COMMAND_HANDLING_H_
#define INC_COMMAND_HANDLING_H_

#include "stm32f3xx_hal.h"
#include <string.h>
#include "main.h"
//#include "abs_enc_reading.h"

I2C_HandleTypeDef *i2c_bus_handle;
//IWDG_HandleTypeDef *watch_dog_handle;

//SMBus* i2cBus;
//I2C_HandleTypeDef* abs_encoder_handle;
//AbsEncoder* abs_enc_0;
//AbsEncoder* abs_enc_1;

typedef struct {
	enum {
		OFF=0x00,
		ON=0x0F,
		OPEN=0x10,
		OPEN_PLUS=0x1F,
		CLOSED=0x20,
		CLOSED_PLUS=0x2F,
		CONFIG_PWM=0x30,
		CONFIG_K=0x3F,
		QUAD_ENC=0x40,
		ADJUST=0x4F,
		ABS_ENC=0x50,
		LIMIT=0x60,
		UNKNOWN=0xFF} operation;
	uint8_t channel;
	uint8_t buffer[32];
	uint16_t tick;
} I2CBus;

extern I2CBus i2c_bus_default;

I2CBus i2c_bus;

uint8_t CH_num_receive();
uint8_t CH_num_send();
void CH_prepare_send();
void CH_process_received();
void CH_reset();
void CH_tick();

void HAL_I2C_AddrCallback(I2C_HandleTypeDef * hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef * hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef * hi2c);

#endif /* INC_COMMAND_HANDLING_H_ */
