#ifndef SPECTRAL_H_
#define SPECTRAL_H_

#include "stm32g0xx_hal.h"
#include "smbus.h"	// for SMBus
#include "stdint.h" // for uint types
#include "stdlib.h" // for malloc

// AS7262 Spectral sensor
typedef struct
{
	SMBus *smbus;
} Spectral;

// REQUIRES: i2c is the i2c channel
// and uart is the debugging UART channel or NULL,
// and dma tells if DMA is enabled
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(
	I2C_HandleTypeDef *i2c,
	UART_HandleTypeDef *uart,
	bool dma);

// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral);

// REQUIRES: spectral is an object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint32_t get_spectral_channel_data(Spectral *spectral, uint8_t channel);

#endif

//#endif
