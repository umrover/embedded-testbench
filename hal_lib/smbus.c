#include "smbus.h"


<<<<<<< HEAD
Bus *new_bus(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    Bus *bus = malloc(sizeof(Bus));
    bus->i2c = hi2c;
    bus->uart = huart;
    memset(bus->buf, 0, sizeof bus->buf);

    return bus;
}

void disable_DMA(Bus *bus) {
    bus->DMA = FALSE;
}

long read_byte(Bus *bus, uint8_t addr) {
    if (!bus->DMA){
        bus->ret = HAL_I2C_Master_Receive(bus->i2c, (addr << 1) | 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        bus->ret = HAL_I2C_Master_Receive_DMA(bus->i2c, (addr << 1) | 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    _check_error(bus);
    return bus->buf[0];
}

void write_byte(Bus *bus, uint8_t addr, uint8_t data) {
    bus->buf[0] = data;
    if (!bus->DMA) {
        bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        bus->ret = HAL_I2C_Master_Transmit_DMA(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    _check_error(bus);
=======
SMBus *new_smbus(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    SMBus *smbus = malloc(sizeof(SMBus));
    smbus->i2c = hi2c;
    smbus->uart = huart;
    smbus->DMA = TRUE;
    memset(smbus->buf, 0, sizeof smbus->buf);

    return smbus;
}

void disable_DMA(SMBus *smbus) {
    smbus->DMA = FALSE;
}

long read_byte(SMBus *smbus, uint8_t addr) {
    if (!smbus->DMA){
        smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        smbus->ret = HAL_I2C_Master_Receive_DMA(smbus->i2c, (addr << 1) | 1, smbus->buf, 1);
    }
    _check_error(smbus);
    return smbus->buf[0];
}

void write_byte(SMBus *smbus, uint8_t addr, uint8_t data) {
    smbus->buf[0] = data;
    if (!smbus->DMA) {
        smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 1);
    }
    _check_error(smbus);
>>>>>>> hal_lib
}

long read_byte_data(SMBus *smbus, uint8_t addr, char cmd) {
    //transmits the address to read from
<<<<<<< HEAD
    bus->buf[0] = cmd;
    if (!bus->DMA) {
        bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        bus->ret = HAL_I2C_Master_Transmit_DMA(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    _check_error(bus);
    
    //reads from address sent above
    if (!bus->DMA) {
        bus->ret = HAL_I2C_Master_Receive(bus->i2c, (addr << 1) | 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        bus->ret = HAL_I2C_Master_Receive_DMA(bus->i2c, (addr << 1) | 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    _check_error(bus);
    return bus->buf[0];
=======
    smbus->buf[0] = cmd;
    if (!smbus->DMA) {
        smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 1);
    }
    _check_error(smbus);
    
    //reads from address sent above
    if (!smbus->DMA) {
        smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        smbus->ret = HAL_I2C_Master_Receive_DMA(smbus->i2c, (addr << 1) | 1, smbus->buf, 1);
    }
    _check_error(smbus);
    return smbus->buf[0];
>>>>>>> hal_lib
}

void write_byte_data(SMBus *smbus, uint8_t addr, char cmd, uint8_t data) {
    smbus->buf[0] = cmd;
    smbus->buf[1] = data;

    //SMBUS docs first byte is cmd to write, second is data
<<<<<<< HEAD
    if (!bus->DMA) {
        bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 2, HAL_MAX_DELAY);
    }
    else {
        bus->ret = HAL_I2C_Master_Transmit_DMA(bus->i2c, addr << 1, bus->buf, 2, HAL_MAX_DELAY);
    }
    _check_error(bus);
}

long read_word_data(Bus *bus, uint8_t addr, char cmd) {
    bus->buf[0] = cmd;
    if (!bus->DMA) {
        bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        bus->ret = HAL_I2C_Master_Transmit_DMA(bus->i2c, addr << 1, bus->buf, 1, HAL_MAX_DELAY);
    }
    _check_error(bus);
    
    //reads from address sent above
    if (!bus->DMA){
        bus->ret = HAL_I2C_Master_Receive(bus->i2c, (addr << 1) | 1, bus->buf, 2, HAL_MAX_DELAY);
    }
    else {
        bus->ret = HAL_I2C_Master_Receive_DMA(bus->i2c, (addr << 1) | 1, bus->buf, 2, HAL_MAX_DELAY);
    }
    _check_error(bus);
=======
    if (!smbus->DMA) {
        smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 2, HAL_MAX_DELAY);
    }
    else {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 2);
    }
    _check_error(smbus);
}

long read_word_data(SMBus *smbus, uint8_t addr, char cmd) {
    smbus->buf[0] = cmd;
    if (!smbus->DMA) {
        smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, HAL_MAX_DELAY);
    }
    else {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 1);
    }
    _check_error(smbus);
    
    //reads from address sent above
    if (!smbus->DMA){
        smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, 2, HAL_MAX_DELAY);
    }
    else {
        smbus->ret = HAL_I2C_Master_Receive_DMA(smbus->i2c, (addr << 1) | 1, smbus->buf, 2);
    }
    _check_error(smbus);
>>>>>>> hal_lib
    
    long data = smbus->buf[0] | (smbus->buf[1] << 8);
    return data;
}

<<<<<<< HEAD
void write_word_data(Bus *bus, uint8_t addr, char cmd, uint16_t data) {
    bus->buf[0] = cmd;
    bus->buf[1] = data & 0xFF; //LSB
    bus->buf[2] = (data & 0xFF00) >> 8; //MSB

    if(!bus->DMA) {
        bus->ret = HAL_I2C_Master_Transmit(bus->i2c, addr << 1, bus->buf, 3, HAL_MAX_DELAY);
    }
    else {
       bus->ret = HAL_I2C_Master_Transmit_DMA(bus->i2c, addr << 1, bus->buf, 3, HAL_MAX_DELAY); 
=======
void write_word_data(SMBus *smbus, uint8_t addr, char cmd, uint16_t data) {
    smbus->buf[0] = cmd;
    smbus->buf[1] = data & 0xFF; //LSB
    smbus->buf[2] = (data & 0xFF00) >> 8; //MSB

    if(!smbus->DMA) {
        smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 3, HAL_MAX_DELAY);
    }
    else {
       smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 3);
>>>>>>> hal_lib
    }
    
    _check_error(smbus);
}

int _check_error(SMBus *smbus) {
    if (smbus->ret != HAL_OK) {
        strcpy((char*)smbus->buf, "Err \r\n");

        HAL_UART_Transmit(smbus->uart, smbus->buf, strlen((char*)smbus->buf), HAL_MAX_DELAY);
        HAL_Delay(10);
        return FALSE;
    }
    return TRUE;
}

void reset(SMBus *smbus) {
    HAL_I2C_DeInit(smbus->i2c);
    HAL_I2C_Init(smbus->i2c);
}
