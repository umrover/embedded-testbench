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
ThermalSensor* newThermalSensor(SMBus* _i2cBus){

	// Create a new struct
	ThermalSensor* _ThermalSensor = (ThermalSensor*) malloc(sizeof(ThermalSensor));

	// Initialize the struct
	_ThermalSensor->address = 0x18; // Assumes 3 ground pins

	_ThermalSensor->i2cBus = _i2cBus;

	return _ThermalSensor;
}

// EFFECTS: Get temperature data from a thermal sensor in Celsius
float getThermalData(const ThermalSensor* _ThermalSensor){
	int specificAddress = _ThermalSensor->address;

	// rawData is the data for the ambient
	float rawData = read_word_data(_ThermalSensor->i2cBus, specificAddress, 0x5);

	float lowerByte = (int)rawData & 0xFF;; // original && 0000000...111111111
	float upperByte = ((int)rawData & 0xFF00) >> 8;; // original bit shifted

	// Clear flag bits
	upperByte = (int)upperByte & 0x1F;

	// If the sign bit is zero,
	// then the ambient temperature is greater than 0 celsius.
	// If the sign bit is one,
	// then the ambient temperature is less than 0 celsius.
	int signBit = ((int)upperByte & 0x10) >> 4;
	
	float actualTemperature = 0;
	if (signBit) {
		upperByte = (int)upperByte & 0x0F;
		actualTemperature = 256 - (upperByte * 16 + lowerByte / 16);
	}
	else {
		actualTemperature = upperByte * 16 + lowerByte / 16;
	}
	return actualTemperature;
}

// EFFECTS: Delete the ThermalSensor object from memory
void deleteThermalSensor(ThermalSensor* _ThermalSensor){
	free(_ThermalSensor);
}

//
//
// All Private Functions

#endif
