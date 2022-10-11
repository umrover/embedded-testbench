#pragma once
#include <current.h>

// All Public Functions

// EFFECTS: Create a new CurrentSensor struct and returns pointer to struct.
CurrentSensor* new_CurrentSensor(ADC_HandleTypeDef* _adc_pin, uint32_t _adc_channel){

	// Create a new struct
	CurrentSensor* CurrentSensor_object = (CurrentSensor*) malloc(sizeof(CurrentSensor));
	CurrentSensor_object->adc_pin = _adc_pin;
	CurrentSensor_object->adc_channel = _adc_channel;

	return CurrentSensor_object;

}

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
float get_voltage_data(const CurrentSensor* _CurrentSensor, ADC_ChannelConfTypeDef* _sConfig){
	_sConfig->Channel = _CurrentSensor->adc_channel;
	// Read data from ADC, raw_ADC_data is a number in [0, 4095]
	uint32_t raw_ADC_data = read_from_ADC(_CurrentSensor->adc_pin);
	// Convert from [0, 4095] to [0, 1] to [0, 3.3]
	float steppedDownVoltage = (raw_ADC_data / 4095.0) * 3.3;
	// Convert steppedDownVoltage to actual output voltage (multiplier of 14)
	return steppedDownVoltage * 14.0;
}

// EFFECTS: Get current data from a specific current sensor in Amps
float get_current_data(const CurrentSensor* _CurrentSensor, ADC_ChannelConfTypeDef* _sConfig){
	// Get offset voltage
	float offsetVolts = get_voltage_data(_CurrentSensor, _sConfig);
	// Account for offset which is zero current output voltage (3.3 * 0.1)
	float volts = offsetVolts - 3.3 * 0.1;
	// Convert from volts to millivolts
	float millivolts = (volts) * 1000.0;
	// Convert based on sensitivity, which is 264 mV/A
	return millivolts / 264.0;
}

// EFFECTS: Delete the CurrentSensor object from memory
void delete_CurrentSensor(CurrentSensor* _CurrentSensor){
	free(_CurrentSensor);
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
