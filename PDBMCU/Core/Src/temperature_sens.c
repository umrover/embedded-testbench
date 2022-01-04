/*
 * temperature_sensor.c
 *
 *  Created on: Apr 15, 2021
 */

#ifndef INC_TEMPERATURE_SENS_H_

#include <temperature_sens.h>

// Temperature Sensor: MCP9808T-E/MS

//
//
// All Public Functions

// EFFECTS: Create a new TemperatureSensor struct and returns pointer to struct.
TemperatureSensor* new_temperature_sensor(SMBus* _i2cBus, uint8_t A0, uint8_t A1, uint8_t A2){

	// Create a new struct
	TemperatureSensor* _temperature_sensor = (TemperatureSensor*) malloc(sizeof(TemperatureSensor));

	// Initialize the struct
	_temperature_sensor->address = 0b0011000;
	if (A0) _temperature_sensor->address |= 0b001;
	if (A1) _temperature_sensor->address |= 0b010;
	if (A2) _temperature_sensor->address |= 0b100;

	_temperature_sensor->i2cBus = _i2cBus;

	return _temperature_sensor;
}

// EFFECTS: Get temperature data from a temperature sensor in Celsius
float get_temperature_data(const TemperatureSensor* _temperature_sensor){

	int specific_address = _temperature_sensor->address;

	// rawData is the data for the ambient
	// 0b000000101 is 0x05
	float rawData = read_word_data(_temperature_sensor->i2cBus, specific_address, 0b000000101);

	float lower_byte = (int)rawData & 0xFF;; // original && 0000000...111111111
	float upper_byte = ((int)rawData & 0xFF00) >> 8;; // original bit shifted

	// Clear flag bits
	upper_byte = (int)upper_byte & 0x1F;

	// If the sign bit is zero,
	// then the ambient temperature is greater than 0 celsius.
	// If the sign bit is one,
	// then the ambient temperature is less than 0 celsius.
	int sign_bit = ((int)upper_byte & 0x10) >> 4;
	
	float actual_temperature = 0;
	if (sign_bit) {
		upper_byte = (int)upper_byte & 0x0F;
		actual_temperature = (float)256 - ((float)upper_byte * (float)16 + (1.0 / 16.0) * (float)lower_byte);
	}
	else {
		actual_temperature = (float)upper_byte * (float)16 + (1.0 / 16.0) * (float)lower_byte;
	}
	return actual_temperature;
}

// EFFECTS: Delete the TemperatureSensor object from memory
void delete_temperature_sensor(TemperatureSensor* _temperature_sensor){
	free(_temperature_sensor);
}

//
//
// All Private Functions

#endif
