////////////////////////////////
//      Thermistor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////


#include "thermistor.h"
#include "mosfet.h"

#include "stm32g0xx_hal.h"

#include <stdint.h>

Thermistor* thermistor;

Thermistor *new_thermistor(ADC_HandleTypeDef *_adc, float resistance, uint8_t channel) {
    Thermistor* thermistor = (Thermistor*) malloc(sizeof(Thermistor));

    thermistor->adc = _adc;

    thermistor->resistance = resistance;

    thermistor->channel = channel;

    return thermistor;
}

void initialize_thermistor(Thermistor* thermistor) {
    
}

void get_thermistor_temperature(Thermistor* thermistor) {

}
