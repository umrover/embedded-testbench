/*
 * i2c_bridge.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "i2c_bridge.h"


I2CBus i2c_bus_default = {
        UNKNOWN, //operation
        0xFF, //channel
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //buffer
        0
};

// timeout ~half a second, prime number to avoid hitting unit testing reset bug again
int WATCHDOG_TIMEOUT = 443;

I2CBus *new_i2c_bus(I2C_HandleTypeDef *_i2c_bus_handle) {
    // TODO
    I2CBus *bus = (I2CBus *) malloc(sizeof(I2CBus));
    bus->i2c_bus_handle = _i2c_bus_handle;
    return bus;
}

uint8_t CH_num_receive(I2CBus *i2c_bus) {
    switch (i2c_bus->operation) {
        case OFF:
        case ON:
            return 0;
        case OPEN:
        case OPEN_PLUS:
            return 4;
        case CLOSED:
        case CLOSED_PLUS:
            return 8;
        case CONFIG_PWM:
            return 2;
        case CONFIG_K:
            return 12;
        case QUAD_ENC:
            return 0;
        case ADJUST:
            return 4;
        case ABS_ENC:
        case LIMIT:
        case CALIBRATED:
            return 0;
        case LIMIT_ON:
            return 1;
        case UNKNOWN:
            return 0;
    }
    return 0;
}

uint8_t CH_num_send(I2CBus *i2c_bus) {
    switch (i2c_bus->operation) {
        case OFF:
        case ON:
        case OPEN:
            return 0;
        case OPEN_PLUS:
            return 4;
        case CLOSED:
            return 0;
        case CLOSED_PLUS:
            return 4;
        case CONFIG_PWM:
        case CONFIG_K:
            return 0;
        case QUAD_ENC:
            return 4;
        case ADJUST:
            return 0;
        case ABS_ENC:
            return 4;
        case LIMIT:
        case CALIBRATED:
            return 1;
        case LIMIT_ON:
        case UNKNOWN:
            return 0;
    }
    return 0;
}


void CH_process_received(I2CBus *i2c_bus, Motor *motor) {
    switch (i2c_bus->operation) {
        case OFF:
            set_motor_speed(motor, 0.0f);
            return;
        case ON:
            return;
        case OPEN:
        case OPEN_PLUS:
            motor->mode = 0x00;
            memcpy(&(motor->desired_speed), i2c_bus->buffer, 4);
            return;
        case CLOSED:
        case CLOSED_PLUS:
            motor->mode = 0xFF;
            memcpy(&(motor->gains->kF), i2c_bus->buffer, 4);
            memcpy(&(motor->desired_angle), i2c_bus->buffer + 4, 4);
            return;
        case CONFIG_PWM: {
            int max = 0;
            memcpy(&(max), i2c_bus->buffer, 2);
            motor->speed_limit = (float) (max) / 100;
            return; //UPDATED
        }
        case CONFIG_K:
            memcpy(&(motor->gains->kP), i2c_bus->buffer, 4);
            memcpy(&(motor->gains->kI), i2c_bus->buffer + 4, 4);
            memcpy(&(motor->gains->kD), i2c_bus->buffer + 8, 4);
            return;
        case QUAD_ENC:
            return;
        case ADJUST:
            memcpy(&(motor->encoder->raw), i2c_bus->buffer, 4);
            return;
        case ABS_ENC:
        case LIMIT:
        case CALIBRATED:
            return;
        case LIMIT_ON:
            memcpy(&(motor->limit_enabled), i2c_bus->buffer, 1);
        case UNKNOWN:
            return;
    }
}

void CH_prepare_send(I2CBus *i2c_bus, Motor *motor) {
    switch (i2c_bus->operation) {
        case OFF:
        case ON:
        case OPEN:
            return;
        case OPEN_PLUS:
            memcpy(i2c_bus->buffer, &(motor->encoder->raw), 4);
            return;
        case CLOSED:
            return;
        case CLOSED_PLUS:
            memcpy(i2c_bus->buffer, &(motor->encoder->raw), 4);
            return;
        case CONFIG_PWM:
        case CONFIG_K:
            return;
        case QUAD_ENC:
            memcpy(i2c_bus->buffer, &(motor->encoder->raw), 4);
            return;
        case ADJUST:
            return;
            // case ABS_ENC: memcpy(i2c_bus->buffer, &(motor->abs_enc_value), 4); return;
        case ABS_ENC:
            return; // TODO add support for abs encoder
            // case LIMIT: memcpy(i2c_bus->buffer, &(motor->limit), 1); return;
        case LIMIT:
            return; // TODO add cases for both limits
        case CALIBRATED:
            memcpy(i2c_bus->buffer, &(motor->calibrated), 1);
            return;
        case LIMIT_ON:
        case UNKNOWN:
            return;
    }
}

void CH_reset(I2CBus *i2c_bus, Motor *motors[], uint8_t num_motors) {
    HAL_I2C_DeInit(i2c_bus->i2c_bus_handle);
    i2c_bus->operation = UNKNOWN;
    for (int i = 0; i < num_motors; ++i) {
        motors[i]->desired_speed = 0; // open loop setpoint
        motors[i]->mode = 0x00;
    }
    HAL_I2C_Init(i2c_bus->i2c_bus_handle);
    HAL_I2C_EnableListen_IT(i2c_bus->i2c_bus_handle);
}

void CH_tick(I2CBus *i2c_bus, Motor *motors[], uint8_t num_motors) {
    i2c_bus->tick += 1;
    if (i2c_bus->tick >= WATCHDOG_TIMEOUT) {
        i2c_bus->tick = 0;
        CH_reset(i2c_bus, motors, num_motors);
    }
}
