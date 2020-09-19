#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"

#ifndef SMBUS_H_
#define SMBUS_H_

#define TRUE 1
#define FALSE 0

typedef struct {
    I2C_HandleTypeDef *i2c;
    UART_HandleTypeDef *uart;
    HAL_StatusTypeDef ret;
    uint8_t buf[30];
} Bus;

Bus *new_bus(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

long read_byte(Bus *bus, uint8_t addr);

void write_byte(Bus *bus, uint8_t addr, uint8_t data);

long read_byte_data(Bus *bus, uint8_t addr, char cmd);

void write_byte_data(Bus *bus, uint8_t addr, char cmd, uint8_t data);

long read_word_data(Bus *bus, uint8_t addr, char cmd);

void write_word_data(Bus *bus, uint8_t addr, char cmd, uint16_t data);

long *read_block_data(Bus *bus, uint8_t addr, char cmd, uint8_t size);

void write_block_data(Bus *bus, uint8_t addr, char cmd, uint8_t *data, uint8_t size);

int _check_error(Bus *bus);

void reset(Bus *bus);

#endif
