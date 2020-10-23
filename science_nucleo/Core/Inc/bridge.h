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

void send_spectral_data(uint16_t *data, UART_HandleTypeDef * huart){
	//indicate start of sequence with spectral identifier
	uint8_t *identifer = "$RSTUVW";
	HAL_UART_Transmit(&huart, identifier, sizeof(identifier), 50);

	char buffer[sizeof(data)]; // Create a char buffer of right size

	//put identifier in buffer
	memcpy(buffer, &identifier, sizeof(identifier));

	// Copy the data to buffer
	for (uint8_t i = 0; i < devices; ++i) {
		for (uint8_t = 0; j < channels; ++j) {
			sprintf(spectral_data, "%d,", data[(channels + i) + j]);
		}
	}
	HAL_UART_Transmit(huart, (uint8_t *)buffer, sizeof(buffer), 50);
}


#endif /* INC_BRIDGE_H_ */
