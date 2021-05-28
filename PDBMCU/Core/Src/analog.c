/*
 * analog.c
 *
 *  Created on: Apr 15, 2021
 *      Author: gutst
 */

#ifndef INC_ANALOG_H_

#include "analog.h"

// Current Sensor: ACS758LCB-050B-PFF-T

//
//
// All Public Functions

// EFFECTS: Create a new Analog struct and returns pointer to struct.
Analog* newAnalog(ADC_HandleTypeDef* _voltagePins[2], ADC_HandleTypeDef* _currentPins[2]){

	// Create a new struct
	Analog* _Analog = (Analog*) malloc(sizeof(Analog));

	// Initialize the struct
	_Analog->adcVoltagePins[0] = _voltagePins[0];
	_Analog->adcVoltagePins[1] = _voltagePins[1];
	_Analog->adcCurrentPins[2] = _currentPins[0];
	_Analog->adcCurrentPins[3] = _currentPins[1];

	_Analog->V1 = 3.3;

	return _Analog;

}

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
// REQUIRES: whichVoltage is either 0 or 1
float getVoltageData(const Analog* _Analog, int whichVoltage){
	// Read data from ADC, rawADCData is a number in [0, 4095]
	uint32_t rawADCData = readFromADC(_Analog->adcVoltagePins[whichVoltage]);
	// Convert from [0, 4095] to [0, 1] to [0, 3.3]
	return (rawADCData / 4095.0) * 3.3;
}

// EFFECTS: Get current data from a specific current sensor in Amps
// REQUIRES: whichCurrent is either 0 or 1
float getCurrentData(const Analog* _Analog, int whichCurrent){
	// Read data from ADC, rawADCData is a number in [0, 4095]
	uint32_t rawADCData = readFromADC(_Analog->adcCurrentPins[whichCurrent]);
	// Convert from [0, 4095] to [0, 1] to [0, 3.3]
	float rawVolts = (rawADCData / 4095.0) * 3.3;
	// Convert from [0, 3.3] to [-1.65, 1.65] to [-1650, -1650]
	float rawMillivolts = (rawVolts - 1.65) * 1000.0;
	// Convert based on sensitivity, which is 40 mV/A
	return rawMillivolts / 40.0;
}

// EFFECTS: Delete the Analog object from memory
void deleteAnalog(Analog* _Analog){
	free(_Analog);
}

//
//
// All Private Functions

// EFFECTS: Read from ADC object
// Returns a number between 0 and 4095
uint32_t readFromADC(ADC_HandleTypeDef* adcObject){
	HAL_ADC_Start(adcObject);
	HAL_ADC_PollForConversion(adcObject, HAL_MAX_DELAY);
    uint32_t raw = HAL_ADC_GetValue(adcObject);
    HAL_ADC_Stop(adcObject);
    return raw;
}

#endif
