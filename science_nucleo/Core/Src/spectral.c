#include "smbus.h"
#include "spectral.h"

// initalizes spectral object, adds bus to it
Spectral *new_spectral(SMBus *i2cBus) {
    Spectral *spectral = malloc(sizeof(Spectral));
    spectral->i2cBus = i2cBus;

	uint8_t START_REG = 0x08; //RAW_VALUE_RGA_LOW;

	for (uint8_t i = 0; i < CHANNELS; ++i) {
		spectral->channels[i] = new_channel(START_REG + (2 * i), START_REG + (2 * i) + 1);
	}
	return spectral;
}

// sets enable bits in devices
void enable_spectral(Spectral *spectral) {
    virtual_write(spectral, CONTROL_SET_UP, 0x28);  // runs twice to account for status miss
    HAL_Delay(5);
    virtual_write(spectral, CONTROL_SET_UP, 0x28);  // converts data bank to 2
    virtual_write(spectral, INT_TIME, 0xFF);  // increases integration time
}


// gets the data as an array of 16 bit integers
void get_spectral_data(Spectral *spectral, uint16_t *data) {

	 get_channel_data(spectral);
	 for (uint8_t i = 0; i < CHANNELS; ++i) {
		 data[i] = spectral->channels[i]->color_data;
	 }
}

void del_spectral(Spectral *spectral) {
	for (int i = 0; i < CHANNELS; ++i) {
		del_channel(spectral->channels[i]);
	}
	free(spectral);
}


/*private interface*/

// functionallly like write_byte  
void virtual_write(Spectral *spectral, uint8_t v_reg, uint8_t data) {
    uint8_t status;

	while(1) {
		status = read_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	write_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, (v_reg | 1 << 7));

	while(1) {
		status = read_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	write_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, data);
}

// functionally like read_byte
uint8_t virtual_read(Spectral *spectral, uint8_t v_reg) {
    uint8_t status;
	uint8_t d;
	status = read_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

	if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		// d = nucleo_byte_read(I2C_AS72XX_SLAVE_READ_REG);
		d = read_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);
	}

	while(1) {
		status = read_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	write_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, v_reg);
	while(1) {
		status = read_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	d = read_byte_data(spectral->i2cBus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);
	return d;
}

void get_channel_data(Spectral *spectral) {
    for (uint8_t i = 0; i < CHANNELS; ++i) {
        Channel *temp = spectral->channels[i];
        temp->color_data = get_val(spectral, temp->lsb_register, temp->msb_register);
        HAL_Delay(10);
        //return (spectral->i2cBus->ret != HAL_OK);
    }
}

// creates a channel
Channel* new_channel(uint8_t lsb_r, uint8_t msb_r) {
    Channel* ch = malloc(sizeof(Channel));
	ch->color_data = 0;
	ch->lsb_register = lsb_r;
	ch->msb_register = msb_r;
	return ch;
}

// gets value of channel 
uint16_t read_channel(Spectral *spectral, int channel) {
    return spectral->channels[channel]->color_data;
}

uint16_t get_val(Spectral *spectral, uint8_t virtual_reg_l, uint8_t virtual_reg_h) {
    uint16_t high = virtual_read(spectral, virtual_reg_h) << 8;
    return high | (virtual_read(spectral, virtual_reg_l) & 0xFF);
}

void del_channel(Channel *channel) {
	free(channel);
}

