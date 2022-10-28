////////////////////////////////
//      Thermistor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////

#include "diagnostic_current_sensor.h"

Current_Sensor* new_diagnostic_current_sensor(int channel, ADCSensor adc_sensor) {
    Current_Sensor* current_sensor = (Current_Sensor*) malloc(sizeof(Current_Sensor));
    current_sensor->adc_sensor = adc_sensor;
    current_sensor->channel = channel;
    current_sensor->amps = 0;

    return current_sensor;
}

void update_current_sensor_value(Current_Sensor* sensor) {
    // sensor returns volts (I think) so get to millivolts and solve the proportion for amps then add the offset. 
    sensor.amps = 1000 * get_adc_sensor_value(sensor, sensor.channel) / mVPerAmp + offset;
}

double get_Amps(Current_Sensor* sensor) {
    
    return sensor.amps;
}

void delete_current_sensor(Current_Sensor* sensor) {
    free(sensor);
}