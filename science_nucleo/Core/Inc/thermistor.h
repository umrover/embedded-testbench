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
#include "stdlib.h"
#include "math.h"


///////////////////
//
// Struct Definitions
//
///////////////////

// Struct of Constant Vars, will get set in initThermistors
typedef struct {

    // These are given by the data sheet for calculating temp depending on resistance
    float constantArray[4][4];
    
    // Stores the three adcPins
    ADC_HandleTypeDef* adcPins[3]; 

    // Values of R1
    int R1vals[3];

    // This should be 5V for the nucleos
    float V1;

    // R25 is the resistance of the thermistor at 25C
    int R25;

} Thermistors;


///////////////////
//
// Public Functions
//
///////////////////

// Inits constants Struct
Thermistors* newThermistors(const ADC_HandleTypeDef*,
                            const ADC_HandleTypeDef*,
                            const ADC_HandleTypeDef*);

// Returns temp as a float in K given which thermistor you want (0 1 or 2)
float getTemp(const uint8_t, const Thermistors*);

// Deletes the thermistor object
void deleteThermistors(Thermistors*);

///////////////////
//
// Private Functions
// Don't use unless you know what you're doing
//
///////////////////

// Reads raw voltage from ADC pin given
uint16_t readVoltage(const ADC_HandleTypeDef*);

#endif