////////////////////////////////
//      Thermistor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////


#include "thermistor.h"

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Thermistor object
Thermistor *new_thermistor(ADCSensor* _adc_sensor, uint8_t _adc_channel) {
    Thermistor* therms = (Thermistor*) malloc(sizeof(Thermistor));
    therms->temperature = 100;
    therms->adc_channel = _adc_channel;
    therms->adc_sensor = _adc_sensor;

    return therms;
}

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: temperature
// EFFECTS: Updates temperature of thermistor
void update_thermistor_temperature(Thermistor* therm) {
    uint16_t raw_data = get_adc_sensor_value(therm->adc_sensor, therm->adc_channel);

	// done to avoid sending infinity/nan
	if (raw_data >= 4095) raw_data = 4094;

    // Logic to get actual Voltage from 12 bit string
    // NOTE pretty sure it is 12 bit that's what HAL says in documentation, but could be wrong
    float curr_volt = (raw_data * V_1) / 4095.0f; // 2^12 - 1= 4095 (12 bit string  )

    // Circuit math to get temperature from voltage
    float R_t = (R_1 * curr_volt) / (V_1 - curr_volt);

    uint8_t const_set = 0;
    if (R_t < 692600.0f && R_t >= 32770.0f){
        const_set = 0;
    } else if (R_t < 32770.0f && R_t >= 3599.0f){
        const_set = 1;
    } else if (R_t < 3599.0f && R_t >= 681.6f){
        const_set = 2;
    } else if (R_t < 681.6f && R_t >= 187.0f){
        const_set = 3;
    } else {
        // TODO error out cause OOB temp
    }
    float lnR_t_over_R_25 = log(R_t / R_25);

    float one_over_T = constant_array[const_set][0] + (constant_array[const_set][1] * lnR_t_over_R_25)
                     + (constant_array[const_set][2] * lnR_t_over_R_25 * lnR_t_over_R_25)
                     + (constant_array[const_set][3] * lnR_t_over_R_25 * lnR_t_over_R_25 * lnR_t_over_R_25);
    therm->temperature = (1 / one_over_T) - 273.15f;
}

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Get temperature of thermistor in degrees Celsius
float get_thermistor_temperature(Thermistor* therm) {
	return therm->temperature;
}

void deleteThermistors(Thermistor* thermistors){
    free(thermistors);
}

