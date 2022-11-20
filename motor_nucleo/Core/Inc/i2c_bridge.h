/*
 * i2c_bridge.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#pragma once

#include "stm32f3xx_hal.h"
#include "motor.h"
#include <string.h>

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
		CALIBRATED=0x6F,
		LIMIT_ON=0x7F,
		UNKNOWN=0xFF} operation;
	uint8_t motor_id;
	uint8_t buffer[32];
	uint16_t tick;
	I2C_HandleTypeDef* i2c_bus_handle;
} I2CBus;

I2CBus* new_i2c_bus(I2C_HandleTypeDef* _i2c_bus_handle);

uint8_t CH_num_receive(I2CBus* i2c_bus);
uint8_t CH_num_send(I2CBus* i2c_bus);
void CH_process_received(I2CBus* i2c_bus, Motor *motor);
void CH_prepare_send(I2CBus* i2c_bus, Motor *motor);
void CH_reset(I2CBus* i2c_bus, Motor *motors[], uint8_t num_motors);
void CH_tick(I2CBus* i2c_bus, Motor *motors[], uint8_t num_motors) ;
