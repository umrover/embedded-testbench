//////////////////////////////////////
//
//      Thermistor Nucleo Header
//      Written By:
//      Nathan Richards
//      nricha@umich.edu
//
//////////////////////////////////////
//#define THERMISTOR_ENABLE

#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include "stm32g0xx.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "main.h"
#include "string.h"

// THIS ORDER IS NOT A MISTAKE. CHANGE IT AROUND IF THE THERMISTORS ARE MONITORING THE WRONG HEATER.
typedef enum
{
	THERMISTOR_OF_HEATER_2,
	THERMISTOR_OF_HEATER_1,
	THERMISTOR_OF_HEATER_0,
	THERMISTOR_DEVICES
} THERMISTOR_DEVICE_NAME;

///////////////////
//
// Struct Definitions
//
///////////////////

// Struct of Constant Vars, will get set in initThermistors
typedef struct {

    // These are given by the data sheet for calculating temp depending on resistance
    float constant_array[4][4];
    
    // Stores the three adc_pins
    uint32_t adc_pins[3];

    // Values of R1
    int R1_vals[3];

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

// creates a new Thermistors struct in dynamic memory, inits the values, then returns a pointer to that struct
Thermistors* new_thermistors(uint32_t channel0,
							uint32_t channel1,
							uint32_t channel2);


// This is very interesting... the thermistors get filled into the array in the wrong order.
typedef enum
{
	INDEX_OF_THERMISTOR_1,
	INDEX_OF_THERMISTOR_2,
	INDEX_OF_THERMISTOR_0,
} INDICES_OF_THERMISTORS;


// Returns temp as a float in K given which thermistor you want (0 1 or 2)
float get_temp(const uint8_t, const Thermistors*);

// Deletes the thermistor object
void deleteThermistors(Thermistors*);

void send_thermistor_data(Thermistors* therms, UART_HandleTypeDef* huart);

///////////////////
//
// Private Functions
// Don't use unless you know what you're doing
//
///////////////////

// Returns raw data from the ADC pin given in a 12 bit string - DATA NOT FORMATTED
uint32_t _readVoltage(uint32_t adcChannel);

extern Thermistors* thermistors;
extern float curr_temps[3];

extern float max_temp;
#endif
