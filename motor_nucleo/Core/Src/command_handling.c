/*
 * command_handling.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "command_handling.h"
#include "abs_enc_reading.h"

I2CBus i2c_bus_default = {
	UNKNOWN, //operation
	0xFF, //channel
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //buffer
	0
};

uint8_t CH_num_receive() {
	switch(i2c_bus.operation) {
	case OFF:
	case ON: return 0;
	case OPEN:
	case OPEN_PLUS: return 4;
	case CLOSED:
	case CLOSED_PLUS: return 8;
	case CONFIG_PWM: return 2;
	case CONFIG_K: return 12;
	case QUAD_ENC: return 0;
	case ADJUST: return 4;
	case ABS_ENC:
	case LIMIT:
	case UNKNOWN: return 0;
	}
	return 0;
}

uint8_t CH_num_send() {
	switch(i2c_bus.operation) {
	case OFF:
	case ON:
	case OPEN: return 0;
	case OPEN_PLUS: return 4;
	case CLOSED:return 0;
	case CLOSED_PLUS: return 4;
	case CONFIG_PWM:
	case CONFIG_K: return 0;
	case QUAD_ENC: return 4;
	case ADJUST: return 0;
	case ABS_ENC: return 4;
	case LIMIT: return 1;
	case UNKNOWN: return 0;
	}
	return 0;
}

void CH_prepare_send() {
	if (i2c_bus.channel > 6) {return;}
	Channel *channel = channels + i2c_bus.channel;
	switch(i2c_bus.operation) {
	case OFF:
	case ON:
	case OPEN: return;
	case OPEN_PLUS: memcpy(i2c_bus.buffer, &(channel->quad_enc_value), 4); return;
	case CLOSED: return;
	case CLOSED_PLUS: memcpy(i2c_bus.buffer, &(channel->quad_enc_value), 4); return;
	case CONFIG_PWM:
	case CONFIG_K: return;
	case QUAD_ENC: memcpy(i2c_bus.buffer, &(channel->quad_enc_value), 4); return;
	case ADJUST: return;
	case ABS_ENC: read_abs_enc(abs_enc_0, abs_enc_1, i2c_bus.channel); memcpy(i2c_bus.buffer, &(channel->abs_enc_value), 2); return;
	case LIMIT: memcpy(i2c_bus.buffer, &(channel->limit), 1); return;
	case UNKNOWN: return;
	}
}

void CH_process_received() {
	if (i2c_bus.channel > 6) {return;}
	Channel *channel = channels + i2c_bus.channel;
	switch(i2c_bus.operation) {
	case OFF: channel->speedMax = 0; return;
	case ON: return;
	case OPEN:
	case OPEN_PLUS: channel->mode = 0x00; memcpy(&(channel->open_setpoint), i2c_bus.buffer, 4); return;
	case CLOSED:
	case CLOSED_PLUS: channel->mode = 0xFF; memcpy(&(channel->FF), i2c_bus.buffer, 4); memcpy(&(channel->closed_setpoint),i2c_bus.buffer+4,4); return;
	case CONFIG_PWM: {
		int max = 0;
		memcpy(&(max),i2c_bus.buffer,2);
		channel->speedMax = (float)(max)/100; return; //UPDATED
	}
	case CONFIG_K: memcpy(&(channel->KP),i2c_bus.buffer,4); memcpy(&(channel->KI),i2c_bus.buffer+4,4); memcpy(&(channel->KD),i2c_bus.buffer+8,4); return;
	case QUAD_ENC: return;
	case ADJUST: memcpy(&(channel->quad_enc_value), i2c_bus.buffer, 4);
	case ABS_ENC:
	case LIMIT:
	case UNKNOWN: return;
	}
}

void CH_reset() {
	HAL_I2C_DeInit(i2c_bus_handle);
	i2c_bus.operation = UNKNOWN;
	HAL_I2C_Init(i2c_bus_handle);
	HAL_I2C_EnableListen_IT(i2c_bus_handle);
}

void CH_tick() {
	i2c_bus.tick += 1;
	if (i2c_bus.tick >= 1000) {
		i2c_bus.tick = 0;
		CH_reset();
	}
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef * hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
	i2c_bus.channel = (0x000F & (AddrMatchCode >> 1));
	if (TransferDirection == I2C_DIRECTION_TRANSMIT){
		HAL_I2C_Slave_Seq_Receive_IT(i2c_bus_handle, i2c_bus.buffer, 1, I2C_LAST_FRAME);
		i2c_bus.operation = UNKNOWN;
	}
	else {
		CH_prepare_send();
		if (CH_num_send() != 0) {
			HAL_I2C_Slave_Seq_Transmit_IT(i2c_bus_handle, i2c_bus.buffer, CH_num_send(), I2C_LAST_FRAME);
		}
	}
	i2c_bus.tick = 0;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef * hi2c) {
	if (i2c_bus.operation == UNKNOWN) {
		i2c_bus.operation = i2c_bus.buffer[0];
		if (CH_num_receive() != 0) {
			HAL_I2C_Slave_Seq_Receive_IT(i2c_bus_handle, i2c_bus.buffer, CH_num_receive(), I2C_LAST_FRAME);
		}
		else {
			CH_process_received();
		}
	}
	else {
		CH_process_received();
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c){
	CH_reset();
}


void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef * hi2c) {
	HAL_I2C_EnableListen_IT(i2c_bus_handle);
}
