#ifndef INC_ANALOG_H_
#define INC_ANALOG_H_

#include "stm32g0xx.h"
#include <stdlib.h>

#define VOLTAGE_DEVICES = 12
#define CURRENT_DEVICES = 12

extern uint16_t pin_array[10];
extern GPIO_TypeDef* port_array[10];

// Current Sensor: ACS722LLCTR-10AU-T

// Analog Struct

typedef struct {

	// Stores the four adc_pins
	ADC_HandleTypeDef* in;
	uint8_t select_pins[4];

} Analog;

// All Public Functions

// NOTE: the way we get stuff from ADC is different here than in the PDB code
// EFFECTS: Create a new Analog object and returns pointer to object.
// TODO
Analog* new_analog(ADC_HandleTypeDef* in, uint8_t* select_pins, size_t sz);

// EFFECTS: Get voltage data from a specific voltage sensor in Volts
float get_voltage_data(const Analog* _Analog);

// EFFECTS: Get current data from a specific current sensor in Amps
float get_current_data(const Analog* _Analog);

// EFFECTS: Delete the Analog object from memory
void delete_analog(Analog* _Analog);

//
//
// All Private Functions

// EFFECTS: Read from ADC object
// Returns a number between 0 and 4095
uint32_t read_from_ADC(ADC_HandleTypeDef* adc_object);

#endif /* INC_ANALOG_H_ */
