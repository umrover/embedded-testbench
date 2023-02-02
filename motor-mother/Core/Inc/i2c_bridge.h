/*
 * i2c_bridge.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#pragma once

#include "stm32f1xx_hal.h"
#include "motor.h"
#include <string.h>

typedef struct {
    enum {
        OFF = 0x00,
        ON = 0x01,
        OPEN = 0x02,
        OPEN_PLUS = 0x03,
        CLOSED = 0x04,
        CLOSED_PLUS = 0x05,
        CONFIG_PWM = 0x06,
        CONFIG_K = 0x07,
        QUAD_ENC = 0x08,
        ADJUST = 0x09,
        ABS_ENC = 0x0A,
        LIMIT = 0x0B,
        CALIBRATED = 0x0C,
        LIMIT_ON = 0x0D,
        UNKNOWN = 0xFF
    } operation;
    uint8_t motor_id;
    uint8_t buffer[32];
    uint16_t tick;
    I2C_HandleTypeDef *i2c_bus_handle;
} I2CBus;

I2CBus *new_i2c_bus(I2C_HandleTypeDef *_i2c_bus_handle);

uint8_t CH_num_receive(I2CBus *i2c_bus);

uint8_t CH_num_send(I2CBus *i2c_bus);

void CH_process_received(I2CBus *i2c_bus, Motor *motor);

void CH_prepare_send(I2CBus *i2c_bus, Motor *motor);

void CH_reset(I2CBus *i2c_bus, Motor *motors[], uint8_t num_motors);

void CH_tick(I2CBus *i2c_bus, Motor *motors[], uint8_t num_motors);
