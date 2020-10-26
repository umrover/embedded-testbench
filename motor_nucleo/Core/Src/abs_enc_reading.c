/*
 * abs_enc_read.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "abs_enc_reading.h"

//uint16_t spiPin(uint8_t channel) {
//	switch(channel) {
//	case 0: return GPIO_PIN_0;
//	case 1: return GPIO_PIN_1;
//	case 2: return GPIO_PIN_2;
//	}
//	return 0;
//}
//
//void spiUpdate(uint8_t channel) {
//	HAL_GPIO_WritePin(GPIOB, spiPin(channel), GPIO_PIN_RESET);
//	HAL_SPI_Receive(&hspi1,&((channels+channel)->spiEnc),1,72000);
//	HAL_GPIO_WritePin(GPIOB, spiPin(channel), GPIO_PIN_SET);
//}

void read_abs_enc(uint8_t channel) {
	((channels + channel)->abs_enc_value) = 0;
}
