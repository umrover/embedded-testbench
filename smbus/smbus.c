#include "smbus.h"


SMBus *new_smbus(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    SMBus *smbus = malloc(sizeof(SMBus));
    smbus->i2c = hi2c;
    smbus->uart = huart;
    smbus->DMA = TRUE;
    memset(smbus->buf, 0, sizeof(smbus->buf));

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
}

long read_byte_data(SMBus *smbus, uint8_t addr, char cmd) {
    //transmits the address to read from
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
}

void write_byte_data(SMBus *smbus, uint8_t addr, char cmd, uint8_t data) {
    smbus->buf[0] = cmd;
    smbus->buf[1] = data;

    //SMBUS docs first byte is cmd to write, second is data
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
    
    long data = smbus->buf[0] | (smbus->buf[1] << 8);
    return data;
}

void write_word_data(SMBus *smbus, uint8_t addr, char cmd, uint16_t data) {
    smbus->buf[0] = cmd;
    smbus->buf[1] = data & 0xFF; //LSB
    smbus->buf[2] = (data & 0xFF00) >> 8; //MSB

    if(!smbus->DMA) {
        smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 3, HAL_MAX_DELAY);
    }
    else {
       smbus->ret = HAL_I2C_Master_Transmit_DMA(smbus->i2c, addr << 1, smbus->buf, 3);
    }
    
    _check_error(smbus);
}

long *read_block_data(SMBus *smbus, uint8_t addr, char cmd) {
    smbus->buf[0] = cmd;
    smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, HAL_MAX_DELAY);
    _check_error(smbus);

    smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, size + 1, HAL_MAX_DELAY);
    _check_error(smbus);

    return bus->buf + 1 
}


void write_block_data(SMBus *smbus, uint8_t addr, char cmd, uint8_t *data, uint8_t size) {
    if (size > 28) {
        strcpy((char*)smbus->buf, "Data too large, increase buffer size \r\n");

        HAL_UART_Transmit(smbus->uart, smbus->buf, strlen((char*)smbus->buf), HAL_MAX_DELAY);
    }

    smbus->buf[0] = cmd;
    smbus->buf[1] = size;

    for (uint8_t i = 0; i < size; ++i) {
        smbus->buf[2 + i] = *data;
        ++data;
    } 

    smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, size + 2, HAL_MAX_DELAY);
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

void del_smbus(SMBus *smbus) {
	free(smbus->buf);
	free(smbus);
}
