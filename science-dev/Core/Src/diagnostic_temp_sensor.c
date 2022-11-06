////////////////////////////////
//      Thermistor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////

#include "diagnostic_temp_sensor.h"


// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created current sensor object
Temp_Sensor* new_diagnostic_temp_sensor(ADCSensor* adc_sensor, int channel) {
    Temp_Sensor* temp_sensor = (Temp_Sensor*) malloc(sizeof(Temp_Sensor));
    temp_sensor->adc_sensor = adc_sensor;
    temp_sensor->channel = channel;
    temp_sensor->temp = 0;

    return temp_sensor;
}

// REQUIRES: valid temp sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value 
void update_current_sensor_value(Temp_Sensor* sensor) {
    // Vout = T(coefficient) * T(ambient) + V0 then solve for T(ambient)
    sensor->temp = (get_adc_sensor_value(sensor->adc_sensor, sensor->channel)*3.3/4096 - ZERO_DEGREE_OUTPUT) / TEMP_COEFFICIENT;
}

// REQUIRES: valid temp sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_current_sensor_amps(Temp_Sensor* sensor) {
    return sensor->temp;
}

void delete_current_sensor(Temp_Sensor* sensor) {
    free(sensor);
}