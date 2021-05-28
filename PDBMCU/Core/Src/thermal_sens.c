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
ThermalSensor* newThermalSensor(SMBus* _i2cBus, int _addresses[4]){

	// Create a new struct
	ThermalSensor* _ThermalSensor = (ThermalSensor*) malloc(sizeof(ThermalSensor));

	// Initialize the struct
	ThermalSensor->addresses[0] = _addresses[0];
	ThermalSensor->addresses[1] = _addresses[1];
	ThermalSensor->addresses[2] = _addresses[2];
	ThermalSensor->addresses[3] = _addresses[3];

	ThermalSensor->i2cBus = _i2cBus;

	return ThermalSensor;
}

// EFFECTS: Get temperature data from a specific thermal sensor in Celsius
// REQUIRES: whichThermal is either 0, 1, 2, or 3
float getThermalData(const ThermalSensor* _ThermalSensor, int whichThermal){
	int specificAddress = _ThermalSensor->addresses[whichThermal];

	// rawData is the data for the ambient
	float rawData = read_word_data(_ThermalSensor->i2cBus, specificAddress, 0x5);

	float lowerByte = rawData & 0xFF;; // original && 0000000...111111111
	float upperByte = (rawData & 0xFF00) >> 8;; // original bit shifted 

	// Clear flag bits
	upperByte = upperByte & 0x1F;

	// If the sign bit is zero,
	// then the ambient temperature is greater than 0 celsius.
	// If the sign bit is one,
	// then the ambient temperature is less than 0 celsius.
	int signBit = (upperByte & 0x10) >> 4;
	
	float actualTemperature = 0;
	if (signBit) {
		upperByte = upperByte & 0x0F;
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
