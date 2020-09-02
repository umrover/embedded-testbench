#include <string.h>
#include <stdio.h>
#include "stm32f3xx_hal.h"

const int true = 1;
const int false = 0;

typedef struct {
    I2C_HandleTypeDef hi2c;
    UART_HandleTypeDef huart;
    HAL_StatusTypeDef ret;
    uint8_t buf[30];
} Bus;

Bus new_bus(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

long read_byte(Bus *bus, uint8_t addr);

void write_byte(Bus *bus, uint8_t addr, uint8_t data);

long read_byte_data(Bus *bus, uint8_t addr, char cmd);

void write_byte_data(Bus *bus, uint8_t addr, char cmd, uint8_t data);

long read_word_data(Bus *bus, uint8_t addr, char cmd);

void write_word_data(Bus *bus, uint8_t addr, char cmd, uint16_t data);

int check_error(Bus *bus);

void reset(Bus *bus);
