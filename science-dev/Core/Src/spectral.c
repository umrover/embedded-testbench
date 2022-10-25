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
    Spectral *spectral = malloc(sizeof(Spectral));
    spectral->smbus = new_smbus(i2c, uart, 0x49, dma);
	uint8_t START_REG = 0x08;
	for (uint8_t i = 0; i < CHANNELS; ++i) {
		spectral->channels[i].msb_register = START_REG + i * 2;
		spectral->channels[i].lsb_register = START_REG + i * 2 + 1;
	}
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
	uint16_t high = (virtual_read(spectral, spectral->channels[channe]->msb_register) & 0xFF) << 8;
    return high | (virtual_read(spectral, spectral->channels[channe]->lsb_register) & 0xFF);
}

// sets enable bits in devices
void enable_spectral(Spectral *spectral) {
    // 0x28 means the following:
    // RST is 0, so no reset is done
    // INT Is 0, so no interrupt
    // GAIN is 0b10, so it is 16x sensor channel gain
    // BANK is 0b10, so data conversion is Mode 2
    // DATA_RDY is 0 and RSVD is 0
    write_spectral(spectral, CONTROL_SET_UP, 0x28);  // runs twice to account for status miss
    HAL_Delay(5);
    write_spectral(spectral, CONTROL_SET_UP, 0x28);  // converts data bank to 2
	// Integration time is 0xFF * 2.8ms
    write_spectral(spectral, INT_TIME, 0xFF);  // increases integration time
}

/* PRIVATE FUNCTIONS */

void write_spectral(Spectral *spectral, uint8_t v_reg, uint8_t data) {
    // Taken from datasheet
    uint8_t status;

	while(1) {
		status = read_byte_data(spectral->smbus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	write_byte_data(spectral->smbus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, (v_reg | 0x80));

	while(1) {
		status = read_byte_data(spectral->smbus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	write_byte_data(spectral->smbus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, data);
}

uint8_t _virtual_read(Spectral *spectral, uint8_t v_reg) {
    // Taken from datasheet
    uint8_t status;
	uint8_t d;
	status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

	if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		// d = nucleo_byte_read(I2C_AS72XX_SLAVE_READ_REG);
		d = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);
	}

	while(1) {
		status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	write_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, v_reg);
	while(1) {
		status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	d = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);
	return d;
}