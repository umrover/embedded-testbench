#pragma once

#include "stm32f3xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DIAG_TEMP_COEFFICIENT 19.5f // mV/Celsius
#define DIAG_TEMP_ZERO_DEGREE_OUTPUT 400 //mV

// Part Link
// https://www.digikey.com/en/products/detail/microchip-technology/MCP9701T-E-TT/1987445
typedef struct {
    uint8_t channel;
    float temp;
    ADCSensor* adc_sensor;
} Temp_Sensor;

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created temp sensor object
Temp_Sensor* new_diagnostic_temp_sensor(ADCSensor* adc_sensor, int channel);

// REQUIRES: valid temp sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value
void update_temp_sensor_value(Temp_Sensor* sensor);

// REQUIRES: valid temp sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_temp_sensor_temp(Temp_Sensor* sensor);
