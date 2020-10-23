//////////////////////////////////////
//
//      Thermistor Nucleo Header
//      Written By:
//      Nathan Richards
//      nricha@umich.edu
//
//////////////////////////////////////

#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include "stm32f3xx_hal.h"
#include "stdint.h"


///////////////////
//
// Struct Definitions
//
///////////////////

// Struct of Constant Vars, will get set in initThermistors
typedef struct {

    // These are given by the data sheet for calculating temp depending on resistance
    const float constantArray[4][4];

    // Values of R1
    const int R1vals[3];

    // This should be 5V for the nucleos
    const float V1;

    // R25 is the resistance of the thermistor at 25C
    const int R25;

    // Stores the three adcPins
    const ADC_HandleTypeDef* adcPins; 
    
}Thermistors;


///////////////////
//
// Public Functions
//
///////////////////

// Inits constants Struct
Thermistors* newThermistors(const ADC_HandleTypeDef*,
                            const ADC_HandleTypeDef*,
                            const ADC_HandleTypeDef*);

// Returns temp as a float in K given which thermistor you want
float getTemp(const uint8_t, const Thermistors*);


///////////////////
//
// Private Functions
// Don't use unless you know what you're doing
//
///////////////////

// Reads raw voltage from ADC pin given
uint32_t readVoltage(ADC_HandleTypeDef*);

#endif