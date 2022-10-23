#pragma once

#include "stm32g0xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// given by data sheet for calculating
static float constant_array[4][4] = {{3.3570420E-03, 2.5214848E-04, 3.3743283E-06, -6.4957311E-08},
		{3.3540170E-03, 2.5617244E-04, 2.1400943E-06, -7.2405219E-08},
		{3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08},
		{3.3536166E-03, 2.5377200E-04, 8.5433271E-07, -8.7912262E-08}};

// stores the values in r1
#define R_1 10000.0f
// resistance of thermistor at 25c
#define R_25 10000.0f

// should be 3.3V for the nucleos
#define V_1 3.3f


// TH10K Thermistor
// https://www.thorlabs.com/drawings/e0bb864659fef113-3B520676-F02C-64B3-AEC057713167DDAA/TH10K-SpecSheet.pdf
typedef struct {
	float temperature;
	uint8_t adc_channel;
	ADCSensor* adc_sensor;
} Thermistor;

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Thermistor object
Thermistor *new_thermistor(ADCSensor* _adc_sensor, uint8_t _adc_channel);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: temperature
// EFFECTS: Updates temperature of thermistor
void update_thermistor_temperature(Thermistor* therm);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Get temperature of thermistor in degrees Celsius
float get_thermistor_temperature(Thermistor* therm);

void deleteThermistors(Thermistor* thermistors);

