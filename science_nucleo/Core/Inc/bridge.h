/*
 * bridge.h
 *
 *  Created on: Oct 23, 2020
 *      Author: cgiger
 */

// place for send/recieve_data functions for all the hardware
#ifndef INC_BRIDGE_H_
#define INC_BRIDGE_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"

//transmits the spectral data as a sentance
//$RSTUVW,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,P
void send_spectral_data(uint16_t *data, UART_HandleTypeDef * huart){
	//indicate start of sequence with spectral identifier
	char *identifier = "$RSTUVW,";
	HAL_UART_Transmit(huart, (uint8_t *)identifier, sizeof(identifier), 50);

	int channels = 6;
	int devices = 3;

	char buffer[sizeof(data)]; // Create a char buffer of right size

	// Copy the data to buffer
	for (uint8_t i = 0; i < devices; ++i) {
		for (uint8_t j = 0; j < channels; ++j) {
			sprintf((char*)buffer, "%d,", data[(channels + i) + j]);
		}
	}
	HAL_UART_Transmit(huart, (uint8_t *)buffer, sizeof(buffer), 50);

	char *end = "P";
	HAL_UART_Transmit(huart, (uint8_t *)end, sizeof(end), 50);
}


#endif /* INC_BRIDGE_H_ */
