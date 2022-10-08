#ifndef SPECTRAL_H_
#define SPECTRAL_H_

// AS7262 Spectral sensor
typedef struct {
	I2C_HandleTypeDef *i2c;
} Spectral;

// REQUIRES: hi2c is the i2c channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(I2C_HandleTypeDef *_i2c);

// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral* spectral);

// REQUIRES: spectral is an object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint32_t get_spectral_channel_data(Spectral *spectral, uint8_t channel);

#endif

//#endif
