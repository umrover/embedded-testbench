#pragma once
#include "analog.h"

// Current Sensor: ACS722LLCTR-10AU-T

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
	// Convert steppedDownVoltage to actual output voltage (multiplier of 14)
	return steppedDownVoltage * 14.0;
}

// EFFECTS: Get current data from a specific current sensor in Amps
float get_current_data(const Analog* _Analog){
	// Get offset voltage
	float offsetVolts = get_voltage_data(_Analog);
	// Account for offset which is zero current output voltage (3.3 * 0.1)
	float volts = offsetVolts - 3.3 * 0.1;
	// Convert from volts to millivolts
	float millivolts = (volts) * 1000.0;
	// Convert based on sensitivity, which is 264 mV/A
	return millivolts / 264.0;
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
