/*
 * thermal_sens.c
 *
 *  Created on: Apr 15, 2021
 */

#ifndef INC_THERMAL_SENS_H_

#include "thermal_sens.h"

// Thermal Sensor: MCP9808T-E/MS

//
//
// All Public Functions

// EFFECTS: Create a new ThermalSensor struct and returns pointer to struct.
ThermalSensor* new_thermal_sensor(SMBus* _i2cBus, uint8_t A0, uint8_t A1, uint8_t A2){

	// Create a new struct
	ThermalSensor* _thermal_sensor = (ThermalSensor*) malloc(sizeof(ThermalSensor));

	// Initialize the struct
	_thermal_sensor->address = 0b0011000;
	if (A0) _thermal_sensor->address |= 0b001;
	if (A1) _thermal_sensor->address |= 0b010;
	if (A2) _thermal_sensor->address |= 0b100;

	_thermal_sensor->i2cBus = _i2cBus;

	return _thermal_sensor;
}

// EFFECTS: Get temperature data from a thermal sensor in Celsius
float get_thermal_data(const ThermalSensor* _thermal_sensor){

	int specific_address = _thermal_sensor->address;

	// rawData is the data for the ambient
	// 0b000000101 is 0x05
	float rawData = read_word_data(_thermal_sensor->i2cBus, specific_address, 0b000000101);

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

// EFFECTS: Delete the ThermalSensor object from memory
void delete_thermal_sensor(ThermalSensor* _thermal_sensor){
	free(_thermal_sensor);
}

//
//
// All Private Functions

#endif
