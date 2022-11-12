#pragma once

#include "stm32f3xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DIAG_CURR_VCC 3.3f
#define DIAG_CURR_MV_PER_AMP 122.1f  // MV_PER_AMP = 185mV/A * VCC/5.0

// ACHS-7121 Hall Effect-based isolated linear current sensor
// No clue whether or not it's a adc sensor or not.
typedef struct {
    int channel;
    float amps;
    ADCSensor* adc_sensor;
} Current_Sensor;

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created current sensor object
Current_Sensor* new_diagnostic_current_sensor(int channel, ADCSensor* adc_sensor);

// REQUIRES: valid current sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value
void update_current_sensor_value(Current_Sensor* sensor);


// REQUIRES: valid current sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_current_sensor_amps(Current_Sensor* sensor);
