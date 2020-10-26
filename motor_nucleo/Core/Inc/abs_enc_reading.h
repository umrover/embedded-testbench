/*
 * abs_enc_read.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#ifndef INC_ABS_ENC_READING_H_
#define INC_ABS_ENC_READING_H_

#include "main.h"
#include "stm32f3xx_hal.h"

//uint16_t spiPin(uint8_t channel);
//
//void spiUpdate(uint8_t channel);

void read_abs_enc(uint8_t channel);

#endif /* INC_ABS_ENC_READING_H_ */
