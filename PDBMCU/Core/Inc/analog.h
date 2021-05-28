/*
 * analog.h
 *
 *  Created on: Apr 15, 2021
 *      Author: gutst
 */

#ifndef INC_ANALOG_H_
#define INC_ANALOG_H_

#include "stm32f1xx.h"
#include <stdlib.h>

// Current Sensor: ACS758LCB-050B-PFF-T

// Analog Struct

typedef struct {

	// Stores the four adcPins
	ADC_HandleTypeDef* adcVoltagePins[2];
	ADC_HandleTypeDef* adcCurrentPins[2];

	// Store voltage range of current sensor;
	float V1;

} Analog;

//
//
// All Public Functions

// EFFECTS: Create a new Analog struct and returns pointer to struct.
Analog* newAnalog(ADC_HandleTypeDef* _voltagePins[2], ADC_HandleTypeDef* _currentPins[2]);

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
// REQUIRES: whichVoltage is either 0 or 1
float getVoltageData(const Analog* _Analog, int whichVoltage);

// EFFECTS: Get current data from a specific current sensor in Amps
// REQUIRES: whichVoltage is either 0 or 1
float getCurrentData(const Analog* _Analog, int whichCurrent);

// EFFECTS: Delete the Analog object from memory
void deleteAnalog(Analog* _Analog);

//
//
// All Private Functions

// EFFECTS: Read from ADC object
// Returns a number between 0 and 4095
uint32_t readFromADC(ADC_HandleTypeDef* adcObject);

#endif /* INC_ANALOG_H_ */
