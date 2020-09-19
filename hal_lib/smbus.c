#include "smbus.h"


Bus *new_bus(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    Bus *bus = malloc(sizeof(Bus));
    bus->i2c = hi2c;
    bus->uart = huart;
    memset(bus->buf, 0, sizeof bus->buf);
    return bus;
}

uint8_t read_byte(Bus *bus, uint8_t addr) {
    bus->ret = HAL_I2C_Master_Receive(bus->i2c, (addr << 1) | 1, bus->buf, 1, HAL_MAX_DELAY);
    _check_error(bus);
    return bus->buf[0];
}

void write_byte(Bus *bus, uint8_t addr, uint8_t data) {
    bus->buf[0] = data;
    bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    _check_error(bus);
}

uint8_t read_byte_data(Bus *bus, uint8_t addr, char cmd) {
    //transmits the address to read from
    bus->buf[0] = cmd;
    bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    _check_error(bus);
    
    //reads from address sent above
    bus->ret = HAL_I2C_Master_Receive(bus->i2c, (addr << 1) | 1, bus->buf, 1, HAL_MAX_DELAY);
    _check_error(bus);
    return bus->buf[0];
}

void write_byte_data(Bus *bus, uint8_t addr, char cmd, uint8_t data) {
    bus->buf[0] = cmd;
    bus->buf[1] = data;

    //SMBUS docs first byte is cmd to write, second is data
    bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 2, HAL_MAX_DELAY);
    _check_error(bus);
}

uint16_t read_word_data(Bus *bus, uint8_t addr, char cmd) {
    bus->buf[0] = cmd;
    bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    _check_error(bus);
    
    //reads from address sent above
    bus->ret = HAL_I2C_Master_Receive(bus->i2c, (addr << 1) | 1, bus->buf, 2, HAL_MAX_DELAY);
    _check_error(bus);
    
    long data = bus->buf[0] | (bus->buf[1] << 8);
    return data;
}

void write_word_data(Bus *bus, uint8_t addr, char cmd, uint16_t data) {
    bus->buf[0] = cmd;
    bus->buf[1] = data & 0xFF; //LSB
    bus->buf[2] = (data & 0xFF00) >> 8; //MSB
    bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 3, HAL_MAX_DELAY);
    
    _check_error(bus);
}

uint8_t *read_block_data(Bus *bus, uint8_t addr, char cmd) {
    if (size > 28) {
        strcpy((char*)bus->buf, "Data too large, increase buffer size \r\n");
        HAL_UART_Transmit(bus->uart, bus->buf, strlen((char*)bus->buf), HAL_MAX_DELAY);
        return bus->buf;
    }

    bus->buf[0] = cmd;
    bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    _check_error(bus);

    bus->ret = HAL_I2C_Master_Receive(bus->i2c, (addr << 1) | 1, bus->buf, size + 1, HAL_MAX_DELAY);
    _check_error(bus);

    return bus->buf + 1 
}


void write_block_data(Bus *bus, uint8_t addr, char cmd, uint8_t *data, uint8_t size) {
    if (size > 28) {
        strcpy((char*)bus->buf, "Data too large, increase buffer size \r\n");
        HAL_UART_Transmit(bus->uart, bus->buf, strlen((char*)bus->buf), HAL_MAX_DELAY);
    }

    bus->buf[0] = cmd;
    bus->buf[1] = size;
    
    for (uint8_t i = 0; i < size; ++i) {
        bus->buf[2 + i] = *data;
        ++data;
    } 

    bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, size + 2, HAL_MAX_DELAY);
    _check_error(bus);
}



int _check_error(Bus *bus) {
    if (bus->ret != HAL_OK) {
        strcpy((char*)bus->buf, "Err \r\n");

        HAL_UART_Transmit(bus->uart, bus->buf, strlen((char*)bus->buf), HAL_MAX_DELAY);
        HAL_Delay(10);
        return FALSE;
    }
    return TRUE;
}

void reset(Bus *bus) {
    HAL_I2C_DeInit(bus->i2c);
    HAL_I2C_Init(bus->i2c);
}
