#include "spectral.h"

#define SPECTRAL_DEVICES 1

// REQUIRES: i2c is the i2c channel
// and uart is the debugging UART channel or NULL,
// and dma tells if DMA is enabled
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(
    I2C_HandleTypeDef *i2c,
    UART_HandleTypeDef *uart,
    bool dma)
{
    SMBus *_smbus = new_smbus(i2c, uart, 0x49, dma);
    Spectral *spectral = malloc(sizeof(Spectral));
    spectral->smbus = _smbus;
    return spectral;
}

// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral)
{
	// Opens spectral channels to start listening
	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
		enable_spectral(spectral);
	}
}

// REQUIRES: spectral is an object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint32_t get_spectral_channel_data(Spectral *spectral, uint8_t channel)
{
    return 0;
}
