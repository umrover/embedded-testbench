#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include "stm32g0xx_hal.h"

#include <stdint.h>

// potentially change the order around if needed
typedef enum
{
	THERMISTOR_OF_HEATER_2,
	THERMISTOR_OF_HEATER_1,
	THERMISTOR_OF_HEATER_0,
	THERMISTOR_DEVICES
} THERMISTOR_DEVICE_NAME;

// A thermistor device
typedef struct {

	// given by data sheet for calculating 
	float constant_array[4][4];
	
	// Stores the 3 adc pins
	uint32_t adc_pins[3];
	
	// stores the values in r1
	int R1_vals[3];
	
	// should be 5V for the nucleos
	float V1;
	
	// resistance of thermistor at 25c
	int R25;
} Thermistor;

// REQUIRES: adc is the adc channel,
// resistance is the resistance of the resistor in ohms,
// and channel is the channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Thermistor object
Thermistor *new_thermistor(uint32_t channel0, uint32_t channel1, uint32_t channel2, ADC_HandleTypeDef hadc1);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Initializes the thermistor by changing the ADC settings
void initialize_thermistor(Thermistor* thermistor);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Returns temperature of thermistor in degrees Celsius
float get_thermistor_temperature(uint8_t which_therm, Thermistor* thermistor, ADC_HandleTypeDef hadc1);

void deleteThermistors(Thermistor* thermistors);


#endif

//#endif
