#pragma once

#include "stm32g0xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define VCC 3.3
#define MV_PER_AMP 264

// ACS722LLCTR-10AU-T current sensor
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

void delete_current_sensor(Current_Sensor* sensor);