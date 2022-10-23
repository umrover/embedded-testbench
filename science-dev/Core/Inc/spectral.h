#ifndef SPECTRAL_H_
#define SPECTRAL_H_

#include "stm32g0xx_hal.h"
#include "smbus.h"	// for SMBus
#include "stdint.h" // for uint types
#include "stdlib.h" // for malloc

#define CHANNELS 6

enum {
	DEV_SEL = 0x4F,

	I2C_AS72XX_SLAVE_STATUS_REG = 0x00,
	I2C_AS72XX_SLAVE_WRITE_REG = 0x01,
	I2C_AS72XX_SLAVE_READ_REG = 0x02,
	I2C_AS72XX_SLAVE_TX_VALID = 0x02,
	I2C_AS72XX_SLAVE_RX_VALID = 0x01,

	DEVICE_SLAVE_ADDRESS_READ = 0x93,
	DEVICE_SLAVE_ADDRESS_WRITE = 0x92,
	DEVICE_SLAVE_ADDRESS = 0x49,

	// registers for spectral sensor
	RAW_VALUE_RGA_HIGH = 0x08,
	RAW_VALUE_RGA_LOW = 0x09,

	RAW_VALUE_SHB_HIGH = 0x0A,
	RAW_VALUE_SHB_LOW = 0x0B,

	RAW_VALUE_TIC_HIGH = 0x0C,
	RAW_VALUE_TIC_LOW = 0x0D,

	RAW_VALUE_UJD_HIGH = 0x0E,
	RAW_VALUE_UJD_LOW = 0x0F,

	RAW_VALUE_VKE_HIGH = 0x10,
	RAW_VALUE_VKE_LOW = 0x11,

	RAW_VALUE_WLF_HIGH = 0x12,
	RAW_VALUE_WLF_LOW = 0x13,
	AS7262_V_CAL = 0x14,
	AS7262_B_CAL = 0x18,
	AS7262_G_CAL = 0x1C,
	AS7262_Y_CAL = 0x20,
	AS7262_O_CAL = 0x24,
	AS7262_R_CAL = 0x28,

	CONTROL_SET_UP = 0x04,
	INT_TIME = 0x05
};

// Spectral channel. Each sensor has 6 of them
// We will combine the msb register and lsb register together
// before sending out the data
typedef struct
{
	uint8_t lsb_register;
	uint8_t msb_register;
	uint16_t color_data;
} Channel;

// AS7262 Spectral sensor
typedef struct
{
	SMBus *smbus;
	Channel *channels[CHANNELS];
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

// sets enable bits in devices
void enable_spectral(Spectral *spectral);

#endif

//#endif
