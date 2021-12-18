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
Analog* new_analog(ADC_HandleTypeDef* _adc_pin, uint8_t S0, uint8_t S1, uint8_t S2, uint8_t S3){

	// Create a new struct
	Analog* Analog_object = (Analog*) malloc(sizeof(Analog));

	Analog_object->adc_pin = _adc_pin;
	Analog_object->select_pins[0] = S0;
	Analog_object->select_pins[1] = S1;
	Analog_object->select_pins[2] = S2;
	Analog_object->select_pins[3] = S3;

	return Analog_object;

}

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
float get_voltage_data(const Analog* _Analog){
	// Read data from ADC, raw_ADC_data is a number in [0, 4095]
	uint32_t raw_ADC_data = read_from_ADC(_Analog->adc_pin);
	// Convert from [0, 4095] to [0, 1] to [0, 3.3]
	float steppedDownVoltage = (raw_ADC_data / 4095.0) * 3.3;
	// Convert steppedDownVoltage to actual output voltage
	return steppedDownVoltage * 8.0;
}

// EFFECTS: Get current data from a specific current sensor in Amps
float get_current_data(const Analog* _Analog){
	// Read data from ADC, raw_ADC_data is a number in [0, 4095]
	uint32_t raw_ADC_data = read_from_ADC(_Analog->adc_pin);
	// Convert from [0, 4095] to [0, 1] to [0, 3.3]
	float raw_volts = (raw_ADC_data / 4095.0) * 3.3 * 8.0;
	// Convert from [0, 3.3] to [-1.65, 1.65] to [-1650, -1650]
	float raw_millivolts = (raw_volts - 1.65) * 1000.0;
	// Convert based on sensitivity, which is 40 mV/A
	return raw_millivolts / 40.0;
}

// EFFECTS: Delete the Analog object from memory
void delete_analog(Analog* _Analog){
	free(_Analog);
}

//
//
// All Private Functions

// EFFECTS: Read from ADC object
// Returns a number between 0 and 4095
uint32_t read_from_ADC(ADC_HandleTypeDef* adc_object){
	HAL_ADC_Start(adc_object);
	HAL_ADC_PollForConversion(adc_object, HAL_MAX_DELAY);
    uint32_t raw = HAL_ADC_GetValue(adc_object);
    HAL_ADC_Stop(adc_object);
    return raw;
}

#endif
