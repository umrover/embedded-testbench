#include "spectral.h"

// creates a channel
Channel *new_channel(uint8_t msb_r, uint8_t lsb_r) {
    Channel* ch = malloc(sizeof(Channel));
	ch->lsb_register = lsb_r;
	ch->msb_register = msb_r;
	return ch;
}

// REQUIRES: i2c is the i2c channel
// and uart is the debugging UART channel or NULL,
// and dma tells if DMA is enabled
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(SMBus *smbus)
{
    Spectral *spectral = malloc(sizeof(Spectral));
    spectral->smbus = smbus;
	uint8_t START_REG = RAW_VALUE_RGA_HIGH;
	for (uint8_t i = 0; i < CHANNELS; ++i) {
		spectral->channels[i] = new_channel(START_REG + i * 2, START_REG + i * 2 + 1);
	}
    return spectral;
}

// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral)
{
	// 0x28 means the following:
	// RST is 0, so no reset is done
	// INT Is 0, so no interrupt
	// GAIN is 0b10, so it is 16x sensor channel gain
	// BANK is 0b10, so data conversion is Mode 2
	// DATA_RDY is 0 and RSVD is 0
	virtual_write_spectral(spectral, CONTROL_SET_UP, 0x28);  // runs twice to account for status miss
	HAL_Delay(50);
	virtual_write_spectral(spectral, CONTROL_SET_UP, 0x28);  // converts data bank to 2
	// Integration time is 0xFF * 2.8ms
	HAL_Delay(50);
	virtual_write_spectral(spectral, INT_TIME, 0xFF);  // increases integration time
}

// REQUIRES: spectral is an object and 0 <= channel < 6
// MODIFIES: spectral.channels array
// EFFECTS: Updates values of spectral struct's channels array with data from spectral sensor
void update_channels(Spectral *spectral) {
	// Update ALL channels in the spectral struct
	for(int i = 0; i < CHANNELS; ++i) {
		virtual_read_spectral(spectral, spectral->channels[i]->msb_register);
		virtual_read_spectral(spectral, spectral->channels[i]->lsb_register);
	}
}

// REQUIRES: spectral is a Spectral object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint16_t get_spectral_channel_data(Spectral *spectral, uint8_t channel)
{
	uint16_t high = (spectral->channels[channel]->msb_register & 0xFF) << 8;
    return high | ((spectral->channels[channel]->lsb_register) & 0xFF);

}

/* PRIVATE FUNCTIONS */

// REQUIRES: spectral is a Spectral object,
// v_reg is the virtual register,
// and data is the data to pass
// MODIFIES: nothing
// EFFECTS: Writes to the virtual register as explained in page 18-20 of the datasheet
void virtual_write_spectral(Spectral *spectral, uint8_t v_reg, uint8_t data) {
    uint8_t status;

	while(1) {
		status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	smbus_write_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_WRITE_REG, (v_reg | 0x80));

	while(1) {
		status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	smbus_write_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_WRITE_REG, data);
}

// REQUIRES: spectral is a Spectral object and
// v_reg is the virtual register
// MODIFIES: nothing
// EFFECTS: Returns the value read from the virtual register
// as explained in page 18-20 of the datasheet
uint8_t virtual_read_spectral(Spectral *spectral, uint8_t v_reg) {
    // Taken from datasheet
    uint8_t status;
	uint8_t d;
	status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG);

	if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		// d = nucleo_byte_read(I2C_AS72XX_SLAVE_READ_REG);
		d = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_READ_REG);
	}

	while(1) {
		status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	smbus_write_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_WRITE_REG, v_reg);
	while(1) {
		status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	d = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_READ_REG);
	return d;
}
