#pragma once

#include "stm32g0xx.h"
#include <stdlib.h>

// Current Struct

typedef struct {

	ADC_HandleTypeDef* adc_pin;
	uint32_t adc_channel;

} CurrentSensor;

//
//
// All Public Functions

// EFFECTS: Create a new CurrentSensor struct and returns pointer to struct.
CurrentSensor* new_current(ADC_HandleTypeDef* _adc_pin, uint32_t _adc_channel);

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
float get_voltage_data(const CurrentSensor* _CurrentSensor, ADC_ChannelConfTypeDef *sConfig);

// EFFECTS: Get current data from a specific current sensor in Amps
float get_current_data(const CurrentSensor* _CurrentSensor, ADC_ChannelConfTypeDef* _sConfig);

// EFFECTS: Delete the CurrentSensor object from memory
void delete_CurrentSensor(CurrentSensor* _CurrentSensor);

//
//
// All Private Functions

// EFFECTS: Read from ADC object
// Returns a number between 0 and 4095
uint32_t read_from_ADC(ADC_HandleTypeDef* adc_object);
